"""Small, strict configuration shared by both backends (metres and degrees)."""
from dataclasses import asdict, dataclass, field
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Car:
    length: float = 0.30
    width: float = 0.20
    wheelbase: float = 0.20
    max_speed: float = 0.30
    max_steering_deg: float = 30
    speed_limit: float = 0.25
    drive_channel: int = 0
    steering_channel: int = 1
    drive_type: int = 2
    steering_type: int = 0
    forward_sign: int = 1
    steering_sign: int = 1
    steering_min: float = 60
    steering_center: float = 90
    steering_max: float = 120


@dataclass(frozen=True)
class Gamepad:
    index: int = 0
    throttle_axis: int = 1
    steering_axis: int = 0
    throttle_sign: int = -1
    steering_sign: int = -1  # stick right -> negative yaw (world axes: x forward, y left)
    lb_button: int = 4
    a_button: int = 0
    b_button: int = 1
    hat: int = 0
    dpad: bool = True  # D-pad overrides each active stick axis; both remain usable
    deadzone: float = 0.12
    stale_seconds: float = 0.12
    throttle_mode: str = "stick"
    forward_trigger_axis: int = 4
    reverse_trigger_axis: int = 5
    response_curve: float = 1.0
    acceleration_rate: float = 0.0  # normalized throttle / second; 0 disables ramp
    steering_rate: float = 0.0


@dataclass(frozen=True)
class Lidar:
    enabled: bool = True
    x: float = 0.16  # forward of rear axle
    y: float = 0
    height: float = 0.15
    tilt_deg: float = 0  # positive points up
    yaw_deg: float = 0
    horizontal_deg: float = 120
    vertical_deg: float = 20
    minimum: float = 0.1
    maximum: float = 5.0
    hz: int = 8
    stale_seconds: float = 0.4
    noise_std: float = 0
    drop_probability: float = 0
    seed: int = 7


@dataclass(frozen=True)
class Box:
    x: float
    y: float
    length: float
    width: float
    height: float


@dataclass(frozen=True)
class Arena:
    width: float = 4
    height: float = 4
    wall_height: float = 0.35
    wall_thickness: float = 0.05
    start: list[float] = field(default_factory=lambda: [1, 1, math.pi/2])  # north
    drive_speed_scale: float = 1.0  # simulation only; hardware limits are independent
    battery_wh: float = 7.4
    battery_percent: float = 100
    obstacles: list[Box] = field(default_factory=lambda: [Box(2, 2, 0.6, 0.4, 0.3)])
    opponent_route: list[list[float]] = field(default_factory=lambda: [[3, 1], [3, 3], [1, 3], [1, 1]])
    opponent_speed: float = 0.15
    opponent_length: float = 0.3
    opponent_width: float = 0.2
    opponent_height: float = 0.2

    def boxes(self) -> list[Box]:
        w, h, t, z = self.width, self.height, self.wall_thickness, self.wall_height
        return list(self.obstacles) + [
            Box(-t / 2, h / 2, t, h + 2*t, z), Box(w + t/2, h/2, t, h + 2*t, z),
            Box(w/2, -t/2, w, t, z), Box(w/2, h + t/2, w, t, z),
        ]


@dataclass(frozen=True)
class Hardware:
    host: str = ""  # no default live destination
    port: int = 24642
    timeout: float = 0.10
    channels_confirmed: bool = False
    types_confirmed: bool = False
    forward_confirmed: bool = False
    steering_confirmed: bool = False
    speed_confirmed: bool = False
    sensors_enabled: bool = False


@dataclass(frozen=True)
class Config:
    car: Car = field(default_factory=Car)
    gamepad: Gamepad = field(default_factory=Gamepad)
    lidar: Lidar = field(default_factory=Lidar)
    arena: Arena = field(default_factory=Arena)
    hardware: Hardware = field(default_factory=Hardware)

    def validate(self, real: bool = False) -> None:
        def require(condition, message):
            if not condition:
                raise ValueError(message)

        def finite(value):
            if isinstance(value, dict):
                for v in value.values():
                    finite(v)
            elif isinstance(value, list):
                for v in value:
                    finite(v)
            elif isinstance(value, (int, float)):
                require(math.isfinite(value), "Configuration numbers must be finite")

        finite(asdict(self))
        c, g, l, a, h = self.car, self.gamepad, self.lidar, self.arena, self.hardware
        require(0 < c.wheelbase <= c.length and c.width > 0, "Invalid car dimensions")
        require(0 < c.max_speed <= 2 and 0 < c.max_steering_deg < 80, "Invalid car limits")
        require(0 < c.speed_limit <= 1, "speed_limit must be in (0, 1]")
        require(all(type(i) is int and 0 <= i <= 5 for i in (c.drive_channel, c.steering_channel))
                and c.drive_channel != c.steering_channel, "Assign two distinct servo channels 0..5")
        require(type(c.drive_type) is int and type(c.steering_type) is int
                and c.drive_type == 2 and c.steering_type in (0, 1), "Use continuous drive and positional steering")
        require(all(v == round(v) for v in (c.steering_min, c.steering_center, c.steering_max)),
                "Servo calibration angles must be whole degrees")
        require(0 <= c.steering_min < c.steering_center < c.steering_max <= (180, 270)[c.steering_type],
                "Invalid steering calibration for servo type")
        require(all(i in (-1, 1) for i in (c.forward_sign, c.steering_sign, g.throttle_sign, g.steering_sign)),
                "Direction signs must be -1 or +1")
        require(0 <= g.deadzone < 0.5 and 0.06 <= g.stale_seconds <= 0.5, "Invalid input deadzone/expiry")
        require(all(type(i) is int and i >= 0 for i in (g.index, g.throttle_axis, g.steering_axis,
                g.lb_button, g.a_button, g.b_button, g.hat)), "Gamepad indices must be nonnegative integers")
        require(type(g.dpad) is bool and type(l.enabled) is bool, "Use YAML booleans for switches")
        require(g.throttle_mode in ("stick", "triggers") and 1 <= g.response_curve <= 3
                and g.acceleration_rate >= 0 and g.steering_rate >= 0, "Invalid control response")
        require(all(type(i) is int and i >= 0 for i in (g.forward_trigger_axis, g.reverse_trigger_axis)),
                "Trigger axes must be nonnegative integers")
        require(0 < l.minimum < l.maximum <= 5 and 1 <= l.hz <= 8 and type(l.hz) is int,
                "LiDAR range/rate outside model limits")
        require(l.height > 0 and 0 < l.horizontal_deg <= 180 and 0 < l.vertical_deg < 180
                and abs(l.tilt_deg) <= 90 and l.stale_seconds > 1/l.hz,
                "Invalid LiDAR mount, field of view, or freshness threshold")
        require(l.noise_std >= 0 and 0 <= l.drop_probability <= 1, "Invalid LiDAR noise/drop setting")
        require(a.width > c.length and a.height > c.length and a.wall_height > 0
                and a.wall_thickness > 0, "Invalid arena")
        require(0 < a.drive_speed_scale <= 10, "Simulation drive_speed_scale must be in (0, 10]")
        require(a.battery_wh > 0 and 0 <= a.battery_percent <= 100, "Invalid simulated battery")
        require(len(a.start) == 3 and 0 < a.start[0] < a.width and 0 < a.start[1] < a.height,
                "start must be [rear axle x, y, yaw] inside arena")
        require(len(a.opponent_route) >= 2 and all(len(p) == 2 and 0 < p[0] < a.width
                and 0 < p[1] < a.height for p in a.opponent_route), "Invalid opponent route")
        require(all(a.opponent_route[i] != a.opponent_route[i-1] for i in range(len(a.opponent_route))),
                "Opponent route segments must have nonzero length")
        require(a.opponent_speed >= 0 and min(a.opponent_length, a.opponent_width, a.opponent_height) > 0,
                "Invalid opponent size/speed")
        require(all(min(b.length, b.width, b.height) > 0 for b in a.obstacles), "Invalid obstacle dimensions")
        require(type(h.port) is int and 1 <= h.port <= 65535 and 0.03 <= h.timeout <= 0.25,
                "Invalid UDP port or timeout")
        require(type(h.sensors_enabled) is bool, "sensors_enabled must be a YAML boolean")
        if real:
            require(bool(h.host), "Set hardware.host to the bot IP")
            require(all(v is True for k, v in asdict(h).items() if k.endswith("_confirmed")),
                    "Confirm channels, servo types, forward direction, steering and speed before hardware use")


def from_dict(data: dict) -> Config:
    if not isinstance(data, dict):
        raise ValueError("Configuration must be a YAML mapping")
    data = dict(data)
    arena = dict(data.pop("arena", {}))
    if "obstacles" in arena:
        arena["obstacles"] = [Box(**box) for box in arena["obstacles"]]
    result = Config(arena=Arena(**arena), **{
        key: cls(**data.pop(key, {})) for key, cls in
        (("car", Car), ("gamepad", Gamepad), ("lidar", Lidar), ("hardware", Hardware))
    })
    if data:
        raise ValueError(f"Unknown configuration sections: {', '.join(data)}")
    result.validate()
    return result


def load(path: str | Path) -> Config:
    with open(path, encoding="utf8") as source:
        try:
            return from_dict(yaml.safe_load(source))
        except yaml.YAMLError as error:
            raise ValueError("Invalid YAML configuration") from error
