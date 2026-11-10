"""Idealized sensor models driven by simulated motion and arena geometry.

Assumptions (not hardware calibration): level chassis, 50 uT local field,
60-degree pinhole vision, synthetic learned IDs, 2 W idle / 10 W drive load.
"""
import math

import numpy as np

from .lidar import RayCaster
from .sensors import SensorReading


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
