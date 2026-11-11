"""Driving observations, input rules and latest-only control over Robot primitives."""
from dataclasses import asdict, dataclass
import math
import threading
import time

from .config import Car, Gamepad
from .primitives import Robot
from .protocol import LidarFrame, SensorReading


@dataclass(frozen=True)
class Observation:
    pose: tuple[float, float, float] | None = None  # rear axle; simulation only
    opponent: tuple[float, float, float] | None = None
    collision: bool = False
    lidar: LidarFrame | None = None
    sensors: tuple[SensorReading, ...] = ()


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


class K10(Robot):
    """Driving adapter: calibration and observations built on Robot primitives."""
    def __init__(self, config, token, *, debug=False):
        config.validate(real=True)
        h = config.hardware
        super().__init__(h.host, token, port=h.port, timeout=h.timeout, debug=debug)
        self.config = config
        self.command = DriveCommand()
        self.next_battery = 0

    def connect(self):
        if self.registered:
            return super().connect()
        super().connect()
        c = self.config.car
        try:
            self.configure_servo(c.drive_channel, 'continuous')
            self.configure_servo(c.steering_channel, (180, 270)[c.steering_type],
                home=int(c.steering_center), minimum=int(c.steering_min), maximum=int(c.steering_max))
            self.stop()
            if self.config.lidar.enabled:
                self.stream('lidar', hz=self.config.lidar.hz)
            if self.config.hardware.sensors_enabled:
                for name, hz in (('imu', 10), ('compass', 10), ('camera', 8)):
                    status = self.stream(name, hz=hz, allow_status=True)
                    if status:
                        key = 'vision' if name == 'camera' else name
                        with self.transport.lock:
                            self.transport.sensors[key] = SensorReading(key, time.monotonic(),
                                {'error': f'Stream rejected: status {status}'})
                self._battery()
        except BaseException:
            self.close()
            raise

    def apply(self, command, *, valid_until=float('inf')):
        mapped = map_command(command, self.config.car)
        c = self.config.car
        if time.monotonic() > valid_until:
            self.transport.fault.set()
            raise TimeoutError('Input expired before servo write; discard command')
        ttl = min(.5, max(.001, valid_until-time.monotonic()))
        self.servo_angle(c.steering_channel, mapped.servo_angle, ttl=ttl)
        if time.monotonic() > valid_until:
            self.transport.fault.set()
            raise TimeoutError('Input expired while awaiting steering ACK; discard propulsion')
        self.servo_speed(c.drive_channel, mapped.speed_percent,
                         ttl=min(.5, max(.001, valid_until-time.monotonic())))
        self.command = command

    def observe(self):
        if self.transport.fault.is_set():
            raise RuntimeError('K10 communication or heartbeat failed; session disabled')
        if self.config.hardware.sensors_enabled and self.command.throttle == 0 and time.monotonic() >= self.next_battery:
            self._battery()
        with self.transport.lock:
            return Observation(lidar=self.transport.lidar, sensors=tuple(self.transport.sensors.values()))

    def _battery(self):
        values = self.battery(optional=True)
        if 'battery_percent' in values:
            values = {'percent': values['battery_percent']}
        with self.transport.lock:
            self.transport.sensors['battery'] = SensorReading('battery', time.monotonic(), values)
        self.next_battery = time.monotonic()+5

    def stop(self):
        self.command = DriveCommand(0, self.command.steering)
        super().stop()


class Driver:
    def __init__(self, backend, config, recorder=None):
        self.backend = backend
        self.config = config
        self.recorder = recorder
        self.rules = DrivingRules(config.gamepad)
        self.lock = threading.Lock()
        self.sample = Input(time.monotonic())
        self.observation = Observation()
        self.command = DriveCommand()
        self.state = self.rules.state
        self.error = None
        self.done = threading.Event()
        self.connected = threading.Event()
        self.wake = threading.Event()
        self.thread = threading.Thread(target=self._run, name="actuator-driver", daemon=True)

    def submit(self, sample: Input):
        with self.lock:
            self.sample = sample  # replacement, never a growing command queue
        self.wake.set()  # read the latest press or release without waiting for the actuator deadline

    def snapshot(self):
        with self.lock:
            return self.observation, self.command, self.state

    def start(self):
        self.thread.start()

    def _run(self):
        previous_command = previous_state = previous_lidar = None
        next_actuator = 0
        was_enabled = False
        try:
            self.backend.connect()
            self.connected.set()
            while not self.done.is_set():
                now = time.monotonic()
                with self.lock:
                    sample = self.sample
                command = self.rules.update(sample, now)
                if self.recorder:
                    if command != previous_command:
                        self.recorder.emit("command", asdict(command), now)
                    if self.rules.state != previous_state:
                        self.recorder.emit("state", self.rules.state, now)
                previous_command, previous_state = command, self.rules.state
                if now >= next_actuator or (was_enabled and not self.rules.enabled):
                    if self.rules.enabled:
                        # K10 checks this again between its two acknowledged servo writes.
                        self.backend.apply(command, valid_until=sample.timestamp + self.config.gamepad.stale_seconds)
                    else:
                        self.backend.stop()
                    observation = self.backend.observe()
                    if self.recorder:
                        self.recorder.emit("observation", {"pose": observation.pose,
                            "opponent": observation.opponent, "collision": observation.collision,
                            "sensors": [asdict(reading) for reading in observation.sensors]})
                        if observation.lidar is not None and observation.lidar is not previous_lidar:
                            self.recorder.emit("lidar", asdict(observation.lidar))
                            previous_lidar = observation.lidar
                    with self.lock:
                        self.observation = observation
                    next_actuator = max(now + getattr(self.backend, "period", 0.05), time.monotonic())
                was_enabled = self.rules.enabled
                with self.lock:
                    self.command, self.state = command, self.rules.state
                self.wake.wait(min(0.02, max(0, next_actuator-time.monotonic())))
                self.wake.clear()
        except BaseException as error:
            self.error = error
            with self.lock:
                self.command = self.rules.disable("Session failed: start a new session")
                self.state = self.rules.state
            if self.recorder:
                try:
                    self.recorder.emit("command", asdict(self.command))
                    self.recorder.emit("state", self.state)
                except RuntimeError:
                    pass
        finally:
            try:
                self.backend.close()
            except BaseException as error:
                self.error = self.error or error
            with self.lock:
                self.command = self.rules.disable("Session closed")
                self.state = self.rules.state
            if self.recorder:
                try:
                    self.recorder.emit("command", asdict(self.command))
                    self.recorder.emit("state", self.state)
                except RuntimeError as error:
                    self.error = self.error or error

    def close(self):
        self.done.set()
        self.wake.set()
        self.thread.join(timeout=3)
        if self.thread.is_alive():
            raise RuntimeError("Actuator cleanup did not finish within 3 seconds")
