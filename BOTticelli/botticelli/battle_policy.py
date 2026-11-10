"""Sensor-only policy interface. No poses, health, or hidden targets enter it."""
from dataclasses import dataclass
import math

from .control import DriveCommand


@dataclass(frozen=True)
class Detection:
    bearing: float  # radians, left positive, relative to the camera
    radius_angle: float  # apparent angular radius; no true target coordinates
    elevation: float


@dataclass(frozen=True)
class BattleObservation:
    timestamp: float
    distances: tuple[float, ...]  # seven forward rays, right to left
    balloons: tuple[Detection, ...]
    collision: bool = False
    fresh_seconds: float = .4


class HuntingPolicy:
    """Search, approach, align, avoid. reset/step are reusable by other policies."""
    def __init__(self, alignment=None):
        self.alignment = alignment or {'direction_deg': 0, 'position': [0, 0, .28]}
        self.reset()

    def reset(self):
        self.target = None
        self.seen = -math.inf
        self.reverse_until = 0
        self.reverses = 0
        self.turn = 1

    def step(self, observation: BattleObservation, time: float) -> DriveCommand:
        if not 0 <= time-observation.timestamp <= observation.fresh_seconds or not observation.distances:
            return DriveCommand()
        rays = observation.distances
        forward = min(rays[2:5])
        if observation.balloons:
            self.target = max(observation.balloons, key=lambda d: d.radius_angle)
            self.seen = time
        elif time-self.seen > .5:
            self.target = None
        if time < self.reverse_until:
            # A front-facing sensor cannot authorize reversing into a known rear
            # obstacle; collision feedback bounds these short recovery attempts.
            return DriveCommand(-.5, -self.turn)
        if forward < .16 or observation.collision:
            self.turn = 1 if sum(rays[4:]) >= sum(rays[:3]) else -1
            if self.reverses >= 2:
                return DriveCommand()
            self.reverses += 1
            self.reverse_until = time+.75
            return DriveCommand(-.5, -self.turn)
        if forward > .45:
            self.reverses = 0
        if forward < (.22 if self.target is not None else .45):
            self.turn = 1 if sum(rays[4:]) >= sum(rays[:3]) else -1
            return DriveCommand(.35, self.turn)
        if self.target is not None:
            desired = math.radians(self.alignment['direction_deg'])
            # The apparent radius estimates distance using the nominal practice
            # balloon radius. Only the observed bearing/size are available here.
            distance = .07/max(.001, math.tan(self.target.radius_angle))
            desired += math.atan2(self.alignment['position'][1], max(.08, distance))
            error = self.target.bearing-desired
            return DriveCommand(.8 if abs(error) < .25 else .4, max(-1, min(1, error*3)))
        return DriveCommand(.55, .6*self.turn)
