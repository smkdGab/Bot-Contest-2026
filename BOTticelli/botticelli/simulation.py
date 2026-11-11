"""Offline IR-SIM dynamics, matrix raycasting and synthetic sensors."""
import math
import tempfile
import time

import numpy as np
import yaml

from .config import Config, Lidar
from .control import DriveCommand, Observation, map_command
from .protocol import LidarFrame, SensorReading


class RayCaster:
    def __init__(self, config: Lidar, boxes):
        self.config = config
        self.rng = np.random.default_rng(config.seed)
        # Cell centers: top row first, left to right. Physical sensor orientation is uncalibrated.
        horizontal = np.deg2rad((0.5 - (np.arange(64) + 0.5)/64) * config.horizontal_deg)
        vertical = np.deg2rad((0.5 - (np.arange(8) + 0.5)/8) * config.vertical_deg)
        az, el = np.meshgrid(horizontal, vertical)
        rays = np.stack([np.cos(el)*np.cos(az), np.cos(el)*np.sin(az), np.sin(el)], axis=-1).reshape(-1, 3)
        pitch = math.radians(config.tilt_deg)
        # Positive pitch lifts the forward beam.
        cp, sp = math.cos(pitch), math.sin(pitch)
        rays = rays @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        yaw = math.radians(config.yaw_deg)
        cy, sy = math.cos(yaw), math.sin(yaw)
        self.rays = rays @ np.array([[cy, sy, 0], [-sy, cy, 0], [0, 0, 1]])
        # Prepare constant bounds once, rather than rebuilding geometry for each scan.
        self.bounds = [(np.array([b.x-b.length/2, b.y-b.width/2, 0]),
                        np.array([b.x+b.length/2, b.y+b.width/2, b.height])) for b in boxes]
        self.frame_id = 0

    @staticmethod
    def intersections(origin, directions, lower, upper):
        # Slab ray/box test, vectorized over all 512 rays.
        parallel = np.abs(directions) < 1e-12
        safe = np.where(parallel, 1, directions)
        t1, t2 = (lower-origin)/safe, (upper-origin)/safe
        near = np.where(parallel, -np.inf, np.minimum(t1, t2)).max(axis=1)
        far = np.where(parallel, np.inf, np.maximum(t1, t2)).min(axis=1)
        outside = (parallel & ((origin < lower) | (origin > upper))).any(axis=1)
        return np.where((far >= np.maximum(near, 0)) & ~outside, np.maximum(near, 0), np.inf)

    def scan(self, pose, now: float, sim_time: float, opponent=None, opponent_size=None):
        c = self.config
        if self.rng.random() < c.drop_probability:
            return None
        x, y, yaw = pose
        co, si = math.cos(yaw), math.sin(yaw)
        rotate = np.array([[co, si, 0], [-si, co, 0], [0, 0, 1]])
        directions = self.rays @ rotate
        origin = np.array([x + co*c.x - si*c.y, y + si*c.x + co*c.y, c.height])
        dz = directions[:, 2]
        nearest = np.full(512, np.inf)
        np.divide(-c.height, dz, out=nearest, where=dz < -1e-12)
        for lower, upper in self.bounds:
            nearest = np.minimum(nearest, self.intersections(origin, directions, lower, upper))
        if opponent is not None:
            ox, oy, angle = opponent
            co, si = math.cos(angle), math.sin(angle)
            inverse = np.array([[co, -si, 0], [si, co, 0], [0, 0, 1]])
            local_origin = (origin - np.array([ox, oy, 0])) @ inverse
            length, width, height = opponent_size
            nearest = np.minimum(nearest, self.intersections(
                local_origin, directions @ inverse,
                np.array([-length/2, -width/2, 0]), np.array([length/2, width/2, height])))
        valid = (nearest >= c.minimum) & (nearest <= c.maximum)
        if c.noise_std:
            nearest[valid] += self.rng.normal(0, c.noise_std, np.count_nonzero(valid))
            nearest[valid] = np.clip(nearest[valid], c.minimum, c.maximum)
        values = np.zeros(512, dtype=np.uint16)  # 0 means no valid return in this approximation
        values[valid] = np.rint(nearest[valid]*1000).astype(np.uint16)
        frame = LidarFrame(self.frame_id, round(sim_time*1000), now, tuple(int(v) for v in values))
        self.frame_id = (self.frame_id + 1) & 0xFFFF
        return frame


class SimSensors:
    def __init__(self, config):
        self.config = config
        self.previous = None
        self.velocity = 0
        self.percent = config.arena.battery_percent
        self.readings = {}
        self.next_frame = 0
        self.frame_id = 0
        self.last_collision = False
        self.bounds = RayCaster(config.lidar, config.arena.boxes()).bounds

    def update(self, pose, opponent, collision, sim_time, now):
        velocity = yaw_rate = acceleration = 0.0
        if self.previous is not None:
            old_pose, old_time = self.previous
            dt = sim_time-old_time
            if dt > 0:
                velocity = ((pose[0]-old_pose[0])*math.cos(old_pose[2])+
                            (pose[1]-old_pose[1])*math.sin(old_pose[2]))/dt
                delta_yaw = math.atan2(math.sin(pose[2]-old_pose[2]), math.cos(pose[2]-old_pose[2]))
                yaw_rate = delta_yaw/dt
                acceleration = (velocity-self.velocity)/dt
                fraction = abs(velocity)/max(0.01, self.config.car.max_speed*self.config.arena.drive_speed_scale)
                self.percent = max(0, self.percent-(2+8*fraction)*dt/3600/self.config.arena.battery_wh*100)
        accel = [round(acceleration/9.80665*1000), round(velocity*yaw_rate/9.80665*1000), 1000]
        if collision and not self.last_collision:
            self.readings["bump"] = SensorReading("bump", now, dict(frame_id=self.frame_id,
                direction=1 if self.velocity >= 0 else 0, accel_mg=accel))
        self.previous = (pose, sim_time)
        self.velocity = velocity
        self.readings["motion"] = SensorReading("motion", now, {"velocity_mps": velocity})
        self.last_collision = collision
        self.readings["imu"] = SensorReading("imu", now, dict(frame_id=round(sim_time*1000), accel_mg=accel))
        if sim_time+1e-9 < self.next_frame:
            return tuple(self.readings.values())
        self.next_frame = sim_time+0.1
        self.frame_id += 1
        heading = (90-math.degrees(pose[2])) % 360
        self.readings.update({
            "compass": SensorReading("compass", now, dict(frame_id=self.frame_id,
                heading_deg=round(heading)%360,
                field_ut=[round(20*math.sin(pose[2])), round(20*math.cos(pose[2])), 45])),
            "battery": SensorReading("battery", now, {"percent": round(self.percent, 2)}),
            "vision": SensorReading("vision", now, self.vision(pose, opponent)),
        })
        return tuple(self.readings.values())

    def vision(self, pose, opponent):
        c, a = self.config.lidar, self.config.arena
        co, si = math.cos(pose[2]), math.sin(pose[2])
        origin = np.array([pose[0]+co*c.x-si*c.y, pose[1]+si*c.x+co*c.y, c.height])
        targets = [(opponent[0], opponent[1], a.opponent_width, a.opponent_height, 1, None)]
        targets += [(b.x, b.y, max(b.length, b.width), b.height, i+2, i)
                    for i, b in enumerate(a.obstacles)]
        blocks = []
        focal = 160/math.tan(math.radians(30))
        for x, y, width, height, identity, own_index in targets:
            delta = np.array([x, y, height/2])-origin
            forward = co*delta[0]+si*delta[1]
            lateral = -si*delta[0]+co*delta[1]
            if forward <= 0.1:
                continue
            distance = float(np.linalg.norm(delta))
            if distance > c.maximum:
                continue
            direction = (delta/distance).reshape(1, 3)
            occluded = any(float(RayCaster.intersections(origin, direction, low, high)[0]) < distance-0.01
                           for index, (low, high) in enumerate(self.bounds) if index != own_index)
            if occluded:
                continue
            cx, cy = 160-focal*lateral/forward, 120-focal*delta[2]/forward
            w, h = focal*width/forward, focal*height/forward
            left, right = max(0, cx-w/2), min(320, cx+w/2)
            top, bottom = max(0, cy-h/2), min(240, cy+h/2)
            if right > left and bottom > top:
                blocks.append([round((left+right)/2), round((top+bottom)/2),
                               max(1, round(right-left)), max(1, round(bottom-top)), identity, 100])
        return dict(frame_id=self.frame_id, width=320, height=240, blocks=blocks[:8], arrows=[])


class Simulation:
    period = 0.02

    def __init__(self, config: Config):
        config.validate()
        self.config = config
        self.env = None
        self.time = 0.0
        self.next_scan = 0.0
        self.command = DriveCommand()
        self.observation = Observation()
        a = config.arena
        self.boxes = a.boxes()
        self.rays = RayCaster(config.lidar, self.boxes)
        self.sensors = SimSensors(config)
        # Precompute closed route segment lengths.
        self.segments = []
        self.route_length = 0
        self.opponent_distance = 0.0
        for i, start in enumerate(a.opponent_route):
            end = a.opponent_route[(i+1) % len(a.opponent_route)]
            length = math.dist(start, end)
            self.segments.append((start, end, length))
            self.route_length += length

    def opponent_pose(self, distance=None):
        distance = (self.opponent_distance if distance is None else distance) % self.route_length
        for start, end, length in self.segments:
            if distance <= length:
                f = distance/length
                return (start[0]+f*(end[0]-start[0]), start[1]+f*(end[1]-start[1]),
                        math.atan2(end[1]-start[1], end[0]-start[0]))
            distance -= length
        raise RuntimeError("Invalid route")

    def connect(self):
        import irsim
        c, a = self.config.car, self.config.arena
        maximum = math.radians(c.max_steering_deg)
        speed = c.max_speed * a.drive_speed_scale
        spec = {
            "world": {"width": a.width, "height": a.height, "step_time": self.period,
                      "collision_mode": "unobstructed", "control_mode": "auto"},
            "robot": {"kinematics": {"name": "acker", "mode": "steer", "wheelbase": c.wheelbase},
                      "shape": {"name": "rectangle", "length": c.length, "width": c.width,
                                "wheelbase": c.wheelbase}, "state": [*a.start, 0],
                      "vel_min": [-speed, -maximum], "vel_max": [speed, maximum]},
            "obstacle": [{"shape": {"name": "rectangle", "length": b.length, "width": b.width},
                          "state": [b.x, b.y, 0]} for b in self.boxes]
                        + [{"kinematics": {"name": "omni"},
                            "shape": {"name": "rectangle", "length": a.opponent_length,
                                      "width": a.opponent_width}, "state": list(self.opponent_pose())}],
        }
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as source:
            yaml.safe_dump(spec, source)
            source.flush()
            self.env = irsim.make(source.name, headless=True, seed=self.config.lidar.seed, log_level="ERROR")
        self.robot = self.env.robot
        self.opponent = self.env.objects[-1]
        self.env.refresh()
        if self.robot.collision_flag or self.opponent.collision_flag:
            self.close()
            raise ValueError("Car or opponent start overlaps another object")
        self._observe(time.monotonic(), False)

    def _observe(self, now, collision):
        a, l = self.config.arena, self.config.lidar
        pose = tuple(float(v) for v in self.robot.state[:3, 0])
        opponent = tuple(float(v) for v in self.opponent.state[:3, 0])
        frame = self.observation.lidar
        if l.enabled and self.time + 1e-9 >= self.next_scan:
            new_frame = self.rays.scan(pose, now, self.time, opponent,
                                       (a.opponent_length, a.opponent_width, a.opponent_height))
            if new_frame is not None:
                frame = new_frame
            self.next_scan += 1/l.hz
        readings = self.sensors.update(pose, opponent, collision, self.time, now)
        self.observation = Observation(pose, opponent, collision, frame, readings)

    def apply(self, command: DriveCommand, *, valid_until: float = float("inf")):
        if time.monotonic() > valid_until:
            raise TimeoutError("Simulation input expired; discard command")
        mapped = map_command(command, self.config.car)
        self.command = command
        previous = self.robot.state.copy()
        old_geometry = self.robot.geometry
        self.time += self.period
        self._advance_opponent()
        empty = self.sensors.percent <= 0
        self.env.step([0 if empty else mapped.velocity * self.config.arena.drive_speed_scale,
                       float(previous[3, 0]) if empty else mapped.wheel_angle])
        swept = old_geometry.union(self.robot.geometry).convex_hull
        collision = self.robot.collision_flag or any(
            swept.intersects(obj.geometry) for obj in self.env.objects if obj is not self.robot)
        if collision:
            # IR-SIM's stop mode latches overlapping cars. Reject the penetrating step
            # instead so reverse commands can move away on the very next tick.
            self.robot.set_state(previous)
            self.env.refresh()
        # IR-SIM retains trajectories even headless; BOTticelli records its own bounded stream.
        for obj in (self.robot, self.opponent):
            obj.trajectory.clear()
        self._observe(time.monotonic(), collision)

    def _advance_opponent(self):
        a = self.config.arena
        distance = a.opponent_speed*self.period
        if distance == 0:
            return
        steps = max(1, math.ceil(distance/(min(a.opponent_length, a.opponent_width)/4)))
        if steps > 100:
            raise ValueError("Opponent speed too high for collision resolution")
        for _ in range(steps):
            previous = self.opponent.state.copy()
            old_geometry = self.opponent.geometry
            candidate_distance = self.opponent_distance+distance/steps
            self.opponent.set_state(list(self.opponent_pose(candidate_distance)))
            self.env.refresh()
            swept = old_geometry.union(self.opponent.geometry).convex_hull
            if any(swept.intersects(obj.geometry) for obj in self.env.objects if obj is not self.opponent):
                self.opponent.set_state(previous)
                self.env.refresh()
                break
            self.opponent_distance = candidate_distance

    def observe(self) -> Observation:
        return self.observation

    def stop(self):
        self.command = DriveCommand(0, self.command.steering)
        if self.env is not None:
            self.apply(self.command)

    def close(self):
        self.command = DriveCommand(0, self.command.steering)
        if self.env is not None:
            self.env.end(0)
            self.env = None
