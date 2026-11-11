"""512 ray, height-aware approximation; raw hardware values stay in wire order."""
from dataclasses import dataclass
import math

import numpy as np

from .config import Lidar
from .sensors import SensorReading, LidarFrame, decode_frame


@dataclass(frozen=True)
class Observation:
    pose: tuple[float, float, float] | None = None  # rear axle; simulation only
    opponent: tuple[float, float, float] | None = None
    collision: bool = False
    lidar: LidarFrame | None = None
    sensors: tuple[SensorReading, ...] = ()


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
