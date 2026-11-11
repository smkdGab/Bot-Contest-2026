"""Minimal K10 UDP session. One receiver, one outstanding ACK, no motion retries.

Wire layouts follow ../binary-protocol.md and SensorsScreen's protocol helpers.
The protocol has no transaction IDs; any request timeout ends this session.
"""
import socket
import threading
import time

from .sensors import decode_frame, decode_sensor


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
        self.lidar_intensity = None
        self.sensors = {}
        self.receiver = threading.Thread(target=self._receive, name="k10-receiver", daemon=True)
        self.receiver.start()
        self.heartbeat = None
        self.motion_deadline = float('inf')
        self.emergency_frames = (b'\x28',)

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
            if packet[0] in (0x8F, 0x8E):
                try:
                    frame = decode_frame(b"\x8f"+packet[1:-8], time.monotonic())
                except ValueError:
                    continue
                with self.lock:
                    name = "lidar" if packet[0] == 0x8F else "lidar_intensity"
                    previous = getattr(self, name)
                    if previous is None or 0 < ((frame.frame_id-previous.frame_id) & 0xFFFF) < 0x8000:
                        setattr(self, name, frame)
                continue
            with self.lock:
                if self.pending is None or packet[0] != self.pending[0]:
                    continue
                action, length = self.pending
                body = packet[:-8]
                # A standard ACK is 2 bytes, PING 5; both append UDP diagnostics.
                short_error = action != 0x44 and len(body) == 2 and 1 <= body[1] <= 7
                valid_length = (length[0] <= len(body) <= length[1]) if isinstance(length, tuple) else len(body) == length
                if not valid_length and not short_error:
                    self.error = RuntimeError(f"Malformed reply for action 0x{action:02x}")
                else:
                    self.reply = body
                self.pending = None
                self.reply_ready.set()

    def request(self, frame: bytes, *, cleanup: bool = False, length: int | tuple[int, int] = 2, allow_status: bool = False) -> bytes:
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
                if self.fault.is_set() and not cleanup:
                    raise RuntimeError('K10 watchdog or receiver failed during request')
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
                if now - last > 0.050 or now >= self.motion_deadline:
                    self.fault.set()
                    # No ACK waits here: the sole receiver remains the dispatcher.
                    # Only known stops/home targets, never the expired command.
                    for frame in self.emergency_frames:
                        try:
                            self.sock.sendto(frame, self.address)
                        except OSError:
                            pass
                    return  # let the board watchdog expire as well
                if self.fault.is_set():
                    return
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
