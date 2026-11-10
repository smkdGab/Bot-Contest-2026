"""Minimal K10 UDP session. One receiver, one outstanding ACK, no motion retries.

Wire layouts follow ../binary-protocol.md and SensorsScreen's protocol helpers.
The protocol has no transaction IDs; any request timeout ends this session.
"""
import socket
import struct
import threading
import time

from .config import Config
from .control import DriveCommand, map_command
from .lidar import Observation, decode_frame
from .sensors import SensorReading, decode_sensor


class K10Transport:
    def __init__(self, host: str, port: int = 24642, timeout: float = 0.10):
        self.address = (socket.gethostbyname(host), port)
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.02)
        self.done = threading.Event()
        self.heartbeat_done = threading.Event()
        self.fault = threading.Event()
        self.reply_ready = threading.Event()
        self.request_lock = threading.Lock()
        self.lock = threading.Lock()
        self.pending = None
        self.reply = None
        self.error = None
        self.lidar = None
        self.sensors = {}
        self.receiver = threading.Thread(target=self._receive, name="k10-receiver", daemon=True)
        self.receiver.start()
        self.heartbeat = None

    def _receive(self):
        while not self.done.is_set():
            try:
                packet, sender = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                if not self.done.is_set():
                    self.fault.set()
                    self.reply_ready.set()
                return
            if sender != self.address or not packet:
                continue
            if packet[0] in (0x9F, 0xAF, 0xA0, 0x7F):
                try:
                    reading = decode_sensor(packet[:-8], time.monotonic())
                except ValueError:
                    continue
                with self.lock:
                    previous = self.sensors.get(reading.name)
                    if previous and "frame_id" in previous.values:
                        modulus = 2**32 if reading.name in ("imu", "bump") else 2**16
                        delta = (reading.values["frame_id"]-previous.values["frame_id"]) % modulus
                        if not 0 < delta < modulus//2:
                            continue
                    self.sensors[reading.name] = reading
                continue
            if packet[0] == 0x8F:
                try:
                    frame = decode_frame(packet[:-8], time.monotonic())
                except ValueError:
                    continue
                with self.lock:
                    if self.lidar is None or 0 < ((frame.frame_id-self.lidar.frame_id) & 0xFFFF) < 0x8000:
                        self.lidar = frame
                continue
            with self.lock:
                if self.pending is None or packet[0] != self.pending[0]:
                    continue
                action, length = self.pending
                body = packet[:-8]
                # A standard ACK is 2 bytes, PING 5; both append UDP diagnostics.
                short_error = action != 0x44 and len(body) == 2 and 1 <= body[1] <= 7
                if len(packet) != length + 8 and not short_error:
                    self.error = RuntimeError(f"Malformed reply for action 0x{action:02x}")
                else:
                    self.reply = body
                self.pending = None
                self.reply_ready.set()

    def request(self, frame: bytes, *, cleanup: bool = False, length: int = 2, allow_status: bool = False) -> bytes:
        with self.request_lock:
            if self.fault.is_set() and not cleanup:
                raise RuntimeError("K10 session failed; start a new session, never replay motion")
            with self.lock:
                self.reply_ready.clear()
                self.reply = self.error = None
                self.pending = (frame[0], length)
            try:
                self.sock.sendto(frame, self.address)
                if not self.reply_ready.wait(self.timeout):
                    raise TimeoutError(f"K10 action 0x{frame[0]:02x} timed out; outcome uncertain")
                with self.lock:
                    if self.error:
                        raise self.error
                    if self.reply is None:
                        raise RuntimeError("K10 receive dispatcher failed")
                    if frame[0] != 0x44 and self.reply[1] != 0 and not allow_status:
                        raise RuntimeError(f"K10 action 0x{frame[0]:02x} status 0x{self.reply[1]:02x}")
                    if frame[0] != 0x44 and self.reply[1] > 7:
                        raise RuntimeError("Unknown K10 response status")
                    if frame[0] == 0x44 and self.reply != frame:
                        raise RuntimeError("PING identifier mismatch")
                    return self.reply
            except (OSError, RuntimeError):
                self.fault.set()
                raise
            finally:
                with self.lock:
                    self.pending = None

    def start_heartbeat(self):
        def run():
            last = time.monotonic()
            deadline = last
            while not self.heartbeat_done.is_set():
                now = time.monotonic()
                if now - last > 0.050:
                    self.fault.set()
                try:
                    self.sock.sendto(b"\x43", self.address)
                except OSError:
                    self.fault.set()
                    return
                last = now
                deadline += 0.025
                self.heartbeat_done.wait(max(0, deadline-time.monotonic()))
        self.heartbeat = threading.Thread(target=run, name="k10-heartbeat", daemon=True)
        self.heartbeat.start()

    def close(self):
        self.heartbeat_done.set()
        if self.heartbeat is not None:
            self.heartbeat.join()
        self.done.set()
        self.sock.close()
        self.receiver.join()


class K10:
    def __init__(self, config: Config, token: str):
        config.validate(real=True)  # before creating any socket
        if len(token) != 5 or any(not 33 <= ord(c) <= 126 for c in token):
            raise ValueError("Use the five printable ASCII characters from the TFT")
        self.config = config
        self._token = token
        self.transport = None
        self.attempted = False
        self.registered = False
        self.continuous = set()
        self.command = DriveCommand()
        self.cleanup_errors = []
        self.streaming = False
        self.sensor_streams = []
        self.next_battery = 0

    def connect(self):
        h, c = self.config.hardware, self.config.car
        self.transport = K10Transport(h.host, h.port, h.timeout)
        try:
            self.attempted = True
            self.transport.request(b"\x41" + self._token.encode("ascii"))
            self.registered = True
            self._token = ""
            self.transport.start_heartbeat()
            # Track BEFORE sending: a lost ACK can still mean the type was changed.
            self.continuous.add(c.drive_channel)
            self.transport.request(bytes([0x22, 1 << c.drive_channel, c.drive_type]))
            self.transport.request(bytes([0x22, 1 << c.steering_channel, c.steering_type]))
            self.stop()
            if self.config.lidar.enabled:
                self.streaming = True
                self.transport.request(bytes([0x83, 1, self.config.lidar.hz, 0]))
            if h.sensors_enabled:
                # Geomag uses the rate layout in binary-protocol.md (u16be).
                # Older helper code differs; an explicit rejection is shown in its panel.
                for name, frame in (("imu", b"\xa2\x01\x00\x0a"),
                                    ("compass", b"\x93\x01\x00\x0a"), ("vision", b"\x72\x01\x08")):
                    self.sensor_streams.append(frame)
                    reply = self.transport.request(frame, allow_status=True)
                    if reply[1]:
                        with self.transport.lock:
                            self.transport.sensors[name] = SensorReading(name, time.monotonic(),
                                {"error": f"Stream rejected: status {reply[1]}"})
                self._battery()
        except BaseException:
            self.close()
            raise
        finally:
            self._token = ""

    def apply(self, command: DriveCommand, *, valid_until: float = float("inf")):
        mapped = map_command(command, self.config.car)
        c = self.config.car
        if time.monotonic() > valid_until:
            self.transport.fault.set()
            raise TimeoutError("Input expired before servo write; discard command")
        # Steering is only sent while enabled by runtime; stop() never changes its position.
        self.transport.request(bytes([0x24, 1 << c.steering_channel]) + struct.pack(">h", mapped.servo_angle))
        if time.monotonic() > valid_until:
            self.transport.fault.set()
            raise TimeoutError("Input expired while awaiting steering ACK; discard propulsion")
        self.transport.request(bytes([0x23, 1 << c.drive_channel]) + struct.pack("b", mapped.speed_percent))
        self.command = command

    def observe(self) -> Observation:
        if self.transport.fault.is_set():
            raise RuntimeError("K10 communication or heartbeat failed; session disabled")
        if self.config.hardware.sensors_enabled and self.command.throttle == 0 and time.monotonic() >= self.next_battery:
            self._battery()
        with self.transport.lock:
            return Observation(lidar=self.transport.lidar, sensors=tuple(self.transport.sensors.values()))

    def _battery(self):
        reply = self.transport.request(b"\x29", length=3, allow_status=True)
        values = {"percent": reply[2]} if reply[1] == 0 and reply[2] <= 100 else {"error": "Battery unavailable"}
        with self.transport.lock:
            self.transport.sensors["battery"] = SensorReading("battery", time.monotonic(), values)
        self.next_battery = time.monotonic()+5

    def stop(self):
        self.command = DriveCommand(0, self.command.steering)
        if self.transport is not None:
            # Try every assigned continuous channel even if an earlier stop fails.
            errors = []
            for channel in sorted(self.continuous):
                try:
                    self.transport.request(bytes([0x23, 1 << channel, 0]), cleanup=True)
                except (OSError, RuntimeError) as error:
                    errors.append(error)
            try:
                self.transport.request(b"\x28", cleanup=True)
            except (OSError, RuntimeError) as error:
                errors.append(error)
            if errors:
                raise errors[0]

    def close(self):
        if self.transport is None:
            return
        try:
            if self.attempted:
                if self.registered:
                    try:
                        self.stop()
                    except (OSError, RuntimeError) as error:
                        self.cleanup_errors.append(str(error))
                if self.streaming:
                    try:
                        self.transport.request(bytes([0x83, 0, self.config.lidar.hz, 0]), cleanup=True)
                    except (OSError, RuntimeError) as error:
                        self.cleanup_errors.append(str(error))
                for frame in self.sensor_streams:
                    try:
                        self.transport.request(frame[:1]+b"\x00"+frame[2:], cleanup=True)
                    except (OSError, RuntimeError) as error:
                        self.cleanup_errors.append(str(error))
                try:
                    self.transport.request(b"\x42", cleanup=True)
                except (OSError, RuntimeError) as error:
                    self.cleanup_errors.append(str(error))
        finally:
            self.transport.close()
            self.transport = None
