"""K10 wire replies, sensor frames and the single UDP dispatcher."""
from dataclasses import dataclass
import json
import socket
import struct
import threading
import time


STATUS = {1:'invalid parameters', 2:'invalid values', 3:'operation failed / device unsupported',
          4:'service not started', 5:'unknown service', 6:'unknown command', 7:'not master'}


@dataclass(frozen=True)
class LidarFrame:
    frame_id: int
    timestamp_ms: int
    received_at: float  # local monotonic seconds, never the bot's clock
    values_mm: tuple[int, ...]

    def fresh(self, now: float, threshold: float) -> bool:
        return 0 <= now - self.received_at <= threshold


def decode_frame(body: bytes, received_at: float) -> LidarFrame:
    """Decode after diagnostics removal, with exact full-matrix length validation."""
    if len(body) != 1034 or body[0] != 0x8F or body[1] != 0:
        raise ValueError('Invalid LiDAR action, status, or length')
    frame_id, timestamp, count = struct.unpack_from('<HIH', body, 2)
    if count != 512:
        raise ValueError('Expected full 64 x 8 LiDAR frame')
    return LidarFrame(frame_id, timestamp, received_at, struct.unpack_from('<512H', body, 10))


@dataclass(frozen=True)
class SensorReading:
    name: str
    received_at: float
    values: dict


def decode_sensor(body: bytes, now: float) -> SensorReading:
    if len(body) < 2 or body[1] != 0:
        raise ValueError("Invalid sensor status")
    action = body[0]
    if action == 0x9F and len(body) == 12:
        frame, heading, x, y, z = struct.unpack_from("<HHhhh", body, 2)
        return SensorReading("compass", now, dict(frame_id=frame, heading_deg=heading,
                                                  field_ut=[x, y, z]))
    if action == 0xAF and len(body) == 12:
        frame, x, y, z = struct.unpack_from("<Ihhh", body, 2)
        return SensorReading("imu", now, dict(frame_id=frame, accel_mg=[x, y, z]))
    if action == 0xA0 and len(body) == 13:
        frame, direction, x, y, z = struct.unpack_from("<IBhhh", body, 2)
        if direction > 5:
            raise ValueError("Invalid bump direction")
        return SensorReading("bump", now, dict(frame_id=frame, direction=direction, accel_mg=[x, y, z]))
    if action == 0x7F and len(body) >= 10:
        frame, width, height, blocks, arrows = struct.unpack_from("<HHHBB", body, 2)
        if blocks > 8 or arrows > 8 or not width or not height or len(body) != 10+10*(blocks+arrows):
            raise ValueError("Invalid HuskyLens frame")
        items = [list(struct.unpack_from("<HHHHBB", body, 10+i*10)) for i in range(blocks+arrows)]
        return SensorReading("vision", now, dict(frame_id=frame, width=width, height=height,
                                                 blocks=items[:blocks], arrows=items[blocks:]))
    raise ValueError("Invalid sensor action or exact payload length")


def decode_reply(kind, body):
    if kind=='raw': return body
    if kind=='ack': return {'ok':True}
    if kind=='ping': return {'pong':True}
    if kind=='name': return {'name':body[2:].decode('utf8')}
    if kind=='battery':
        if body[2]>100: raise ValueError('Invalid battery percentage')
        return {'battery_percent':body[2]}
    if kind in ('servo','motor','led'):
        m=body[2]; count=6 if kind=='servo' else 4 if kind=='motor' else 5
        if not m or m>=1<<count: raise ValueError('Invalid echoed channel mask')
        size=2 if kind=='servo' else 1 if kind=='motor' else 4
        if len(body)!=3+m.bit_count()*size: raise ValueError('Invalid channel readback length')
        values={}; offset=3
        for ch in range(count):
            if not m & (1<<ch): continue
            if kind=='servo': value=struct.unpack_from('>h',body,offset)[0]
            elif kind=='motor': value=struct.unpack_from('b',body,offset)[0]
            else: value=dict(zip(('red','green','blue','brightness'),body[offset:offset+4]))
            values[f'{"S" if kind=="servo" else "M" if kind=="motor" else "L"}{ch+(kind=="motor")}']=value
            offset+=size
        return {'commanded_degrees' if kind=='servo' else 'commanded_percent' if kind=='motor' else 'leds':values}
    if kind=='board-led': return {'board_leds':json.loads(body[2:].decode('utf8'))}
    if kind=='accel': return {'accel_mg':list(struct.unpack_from('<hhh',body,2))}
    if kind=='imu-config':
        a,b,c,d=struct.unpack_from('<BHBB',body,2)
        return dict(range_code=a,rate_hz=b,power_code=c,hpf_code=d)
    if kind=='bump-config':
        a,b=struct.unpack_from('<HH',body,2); return dict(threshold_mg=a,debounce_ms=b)
    if kind=='algorithm': return dict(algorithm_code=body[2],learned_id_count=struct.unpack_from('<H',body,3)[0])
    if kind=='controls': return dict(zip(('illumination','illumination_supported','rgb','rgb_supported','display','display_supported','sensor_command'),body[2:]))
    if kind=='lidar':
        frame=decode_frame(b'\x8f'+body[1:],time.monotonic())
        return dict(frame_id=frame.frame_id,timestamp_ms=frame.timestamp_ms,distance_mm=frame.values_mm)
    raise ValueError('Unknown response decoder')


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
                    self.error = RuntimeError(f'Malformed reply for action 0x{action:02x}: received {len(packet)} bytes; expected {length} payload bytes plus 8 diagnostics bytes')
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
                    raise TimeoutError(f'K10 action 0x{frame[0]:02x}: no reply within {self.timeout:g}s; command outcome uncertain')
                if self.fault.is_set() and not cleanup:
                    raise RuntimeError('K10 watchdog or receiver failed during request')
                with self.lock:
                    if self.error:
                        raise self.error
                    if self.reply is None:
                        raise RuntimeError("K10 receive dispatcher failed")
                    if frame[0] != 0x44 and self.reply[1] != 0 and not allow_status:
                        raise RuntimeError(f'K10 action 0x{frame[0]:02x}: {STATUS.get(self.reply[1], "unknown status")} (status 0x{self.reply[1]:02x})')
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
