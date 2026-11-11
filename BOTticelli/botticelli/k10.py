"""Driving adapter over the shared hardware primitives."""
import time

from .control import DriveCommand, map_command
from .lidar import Observation
from .sensors import SensorReading
from .transport import K10Transport  # retained import for existing callers

from .primitives import Robot


class K10(Robot):
    """Driving adapter: calibration and observations built on Robot primitives."""
    def __init__(self, config, token):
        config.validate(real=True)
        h = config.hardware
        super().__init__(h.host, token, port=h.port, timeout=h.timeout)
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
