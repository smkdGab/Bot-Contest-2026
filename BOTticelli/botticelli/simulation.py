"""IR-SIM Ackermann dynamics and collision detection, without its renderer."""
import math
import tempfile
import time

import yaml

from .config import Config
from .control import DriveCommand, map_command
from .lidar import Observation, RayCaster
from .sim_sensors import SimSensors


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
