"""Pure input safety rules and one calibration mapping. No backend imports."""
from dataclasses import dataclass
import math

from .config import Car, Gamepad


@dataclass(frozen=True)
class DriveCommand:
    throttle: float = 0
    steering: float = 0  # positive = left wheel steering / positive yaw

    def __post_init__(self):
        if not all(math.isfinite(v) and -1 <= v <= 1 for v in (self.throttle, self.steering)):
            raise ValueError("DriveCommand values must be finite and in -1..1")


@dataclass(frozen=True)
class Input:
    timestamp: float
    connected: bool = False
    throttle: float = 0
    steering: float = 0
    lb: bool = False
    a: bool = False
    b: bool = False
    generation: int = 0  # changes whenever a gamepad is reopened


@dataclass(frozen=True)
class MappedCommand:
    speed_percent: int
    servo_angle: int
    velocity: float
    wheel_angle: float


def map_command(command: DriveCommand, car: Car) -> MappedCommand:
    """Map once for simulation and hardware; physical inversion affects servo wiring only."""
    s = command.steering * car.steering_sign
    angle = car.steering_center + s * (
        car.steering_max - car.steering_center if s >= 0 else car.steering_center - car.steering_min)
    limit = math.floor(car.speed_limit * 100)
    percent = max(-limit, min(limit, round(command.throttle * car.speed_limit * 100)))
    # Quantize identically to the hardware's integer speed command.
    return MappedCommand(percent * car.forward_sign, round(angle),
                         percent / 100 * car.max_speed,
                         math.radians(command.steering * car.max_steering_deg))


class DrivingRules:
    def __init__(self, config: Gamepad):
        self.config = config
        self.enabled = False
        self.ready = False
        self.state = "Release A, then hold A to drive"
        self.command = DriveCommand()
        self.generation = None
        self.previous_time = None

    def disable(self, reason: str) -> DriveCommand:
        self.enabled = self.ready = False
        self.state = reason
        self.command = DriveCommand(0, self.command.steering)
        return self.command

    def update(self, sample: Input, now: float) -> DriveCommand:
        dt = min(0.05, max(0, now-self.previous_time)) if self.previous_time is not None else 0
        self.previous_time = now
        if self.generation != sample.generation:
            self.disable("Controller changed: release A before driving")
            self.generation = sample.generation
        if (not sample.connected or not 0 <= now - sample.timestamp <= self.config.stale_seconds
                or not all(math.isfinite(v) and -1 <= v <= 1 for v in (sample.throttle, sample.steering))):
            return self.disable("Controller disconnected or input stale")
        if self.enabled and not sample.a:
            return self.disable("Stopped: A released")
        if not self.enabled:
            if not sample.a:
                self.ready = True
                self.state = "Ready: hold A to drive"
            if sample.a and self.ready:
                self.enabled = True
                self.ready = False
                self.state = "Driving: hold A; release to stop"
        if not self.enabled:
            self.command = DriveCommand(0, self.command.steering)
        else:
            def axis(v):
                d = self.config.deadzone
                return 0 if abs(v) <= d else math.copysign(((abs(v)-d)/(1-d))**self.config.response_curve, v)
            def ramp(value, previous, rate):
                return value if rate == 0 else previous+max(-rate*dt, min(rate*dt, value-previous))
            throttle, steering = axis(sample.throttle), axis(sample.steering)
            self.command = DriveCommand(
                ramp(throttle, self.command.throttle, self.config.acceleration_rate) if throttle else 0,
                ramp(steering, self.command.steering, self.config.steering_rate))
        return self.command
