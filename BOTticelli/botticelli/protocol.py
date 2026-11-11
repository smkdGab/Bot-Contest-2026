"""K10 wire replies, sensor frames and the single UDP dispatcher."""
from dataclasses import dataclass
import json
import queue
import socket
import struct
import sys
import threading
import time


STATUS = {1:'invalid parameters', 2:'invalid values', 3:'operation failed / device unsupported',
          4:'service not started', 5:'unknown service', 6:'unknown command', 7:'not master'}


class DebugTrace:
    """Bounded asynchronous stderr output; control threads never write to a terminal."""
    def __init__(self, stream=None):
        self.stream = stream if stream is not None else sys.stderr
        self.started = time.monotonic()
        self.lines = queue.Queue(maxsize=256)
        self.done = threading.Event()
        self.dropped = 0
        self.thread = threading.Thread(target=self._write, name='k10-debug', daemon=True)
        self.thread.start()

    def __call__(self, event, detail='', packet=None):
        if self.done.is_set():
            return
        wire = ''
        if packet is not None:
            # Register may echo a token on old firmware; Wi-Fi frames contain credentials.
            hex_bytes = f'{packet[0]:02x} <redacted>' if packet and packet[0] in (0x41, 0x47, 0x48) else packet.hex(' ')
            wire = f' bytes={len(packet)} hex={hex_bytes}'
        line = f'DEBUG +{time.monotonic()-self.started:09.3f}s {event:<10} {detail}{wire}\n'
        try:
            self.lines.put_nowait(line)
        except queue.Full:
            self.dropped += 1

    def _write(self):
        while not self.done.is_set() or not self.lines.empty():
            try:
                line = self.lines.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                if self.dropped:
                    dropped, self.dropped = self.dropped, 0
                    self.stream.write(f'DEBUG dropped {dropped} trace lines: output is too slow\n')
                self.stream.write(line)
                self.stream.flush()
            except (OSError, ValueError):
                self.done.set()
                return

    def close(self):
        self.done.set()
        self.thread.join(timeout=.5)


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
    if len(body) != 1034:
        raise ValueError(f'Invalid LiDAR length: {len(body)} payload bytes; expected 1034 (1042 UDP bytes including diagnostics)')
    if body[0] != 0x8F or body[1] != 0:
        raise ValueError(f'Invalid LiDAR action 0x{body[0]:02x} or status 0x{body[1]:02x}')
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
    def __init__(self, host: str, port: int = 24642, timeout: float = 1.0, *, debug=False):
        self.address = (socket.gethostbyname(host), port)
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 0))
        self.sock.settimeout(0.02)
        self.done = threading.Event()
        self.heartbeat_done = threading.Event()
        self.fault = threading.Event()
        self.fault_reason = ''
        self.reply_ready = threading.Event()
        self.request_lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.lock = threading.Lock()
        self.pending = None
        self.reply = None
        self.error = None
        self.received = self.other_sender = self.other_action = 0
        self.invalid_events, self.last_event_error = 0, ''
        self.lidar = None
        self.lidar_intensity = None
        self.sensors = {}
        self.trace = DebugTrace() if debug else None
        self.log('OPEN', f'UDP local={self.sock.getsockname()} remote={self.address} timeout={timeout:g}s')
        self.receiver = threading.Thread(target=self._receive, name="k10-receiver", daemon=True)
        self.receiver.start()
        self.heartbeat = None
        self.motion_deadline = float('inf')
        self.emergency_frames = (b'\x28',)

    def log(self, event, detail='', packet=None):
        if self.trace is not None:
            self.trace(event, detail, packet)

    def _send(self, frame, detail):
        with self.send_lock:
            try:
                self.sock.sendto(frame, self.address)
            except OSError as error:
                self.log('TX-ERROR', f'{detail}: {error}', frame)
                raise
            self.log('TX', detail, frame)

    def _diagnostics(self, packet):
        return 'rx_seq=%d server_ms=%d' % struct.unpack('>II', packet[-8:])

    def _invalid_event(self, packet, error):
        self.invalid_events += 1
        self.last_event_error = f'action 0x{packet[0]:02x}, wire bytes={len(packet)}: {error}'
        self.log('DROP', self.last_event_error)

    def _receive(self):
        while not self.done.is_set():
            try:
                packet, sender = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as error:
                if not self.done.is_set():
                    self.fault_reason = f'UDP receive failed: {error}'
                    self.log('RX-ERROR', self.fault_reason)
                    self.fault.set()
                    self.reply_ready.set()
                return
            self.log('RX', f'from={sender}', packet)
            if not packet:
                self.log('DROP', 'empty UDP datagram')
                continue
            if sender != self.address:
                self.other_sender += 1
                self.log('DROP', f'unexpected sender={sender}; expected={self.address}')
                continue
            self.received += 1
            if packet[0] in (0x9F, 0xAF, 0xA0, 0x7F):
                try:
                    reading = decode_sensor(packet[:-8], time.monotonic())
                except ValueError as error:
                    self._invalid_event(packet, error)
                    continue
                with self.lock:
                    previous = self.sensors.get(reading.name)
                    if previous and "frame_id" in previous.values:
                        modulus = 2**32 if reading.name in ("imu", "bump") else 2**16
                        delta = (reading.values["frame_id"]-previous.values["frame_id"]) % modulus
                        if not 0 < delta < modulus//2:
                            self.log('DROP', f'{reading.name} stale/duplicate frame={reading.values["frame_id"]} previous={previous.values["frame_id"]}')
                            continue
                    self.sensors[reading.name] = reading
                if self.trace is not None:
                    values = ' '.join(f'{key}={value!r}' for key, value in reading.values.items())
                    self.log('READ', f'{reading.name} {self._diagnostics(packet)} {values}')
                continue
            if packet[0] in (0x8F, 0x8E):
                try:
                    frame = decode_frame(b"\x8f"+packet[1:-8], time.monotonic())
                except ValueError as error:
                    self._invalid_event(packet, error)
                    continue
                with self.lock:
                    name = "lidar" if packet[0] == 0x8F else "lidar_intensity"
                    previous = getattr(self, name)
                    if previous is None or 0 < ((frame.frame_id-previous.frame_id) & 0xFFFF) < 0x8000:
                        setattr(self, name, frame)
                    else:
                        self.log('DROP', f'{name} stale/duplicate frame={frame.frame_id} previous={previous.frame_id}')
                        continue
                if self.trace is not None:
                    self.log('READ', f'{name} frame={frame.frame_id} device_ms={frame.timestamp_ms} {self._diagnostics(packet)} values={frame.values_mm}')
                continue
            with self.lock:
                if self.pending is None or packet[0] != self.pending[0]:
                    self.other_action += 1
                    self.log('UNMATCHED', f'action=0x{packet[0]:02x} pending={self.pending}')
                    continue
                action, length = self.pending
                body = packet[:-8]
                # A standard ACK is 2 bytes, PING 5; both append UDP diagnostics.
                short_error = action != 0x44 and len(body) == 2 and 1 <= body[1] <= 7
                valid_length = (length[0] <= len(body) <= length[1]) if isinstance(length, tuple) else len(body) == length
                if not valid_length and not short_error:
                    self.error = RuntimeError(f'Malformed reply for action 0x{action:02x}: received {len(packet)} bytes; expected {length} payload bytes plus 8 diagnostics bytes')
                    self.log('DROP', str(self.error))
                else:
                    self.reply = body
                    if self.trace is not None:
                        status = 'PING (no status byte)' if action == 0x44 else f'status=0x{body[1]:02x}'
                        self.log('MATCH', f'action=0x{action:02x} {status} {self._diagnostics(packet)}')
                self.pending = None
                self.reply_ready.set()

    def request(self, frame: bytes, *, cleanup: bool = False, length: int | tuple[int, int] = 2, allow_status: bool = False) -> bytes:
        with self.request_lock:
            if self.fault.is_set() and not cleanup:
                raise RuntimeError("K10 session failed; start a new session, never replay motion")
            received, other_sender, other_action = self.received, self.other_sender, self.other_action
            with self.lock:
                self.reply_ready.clear()
                self.reply = self.error = None
                self.pending = (frame[0], length)
            try:
                self.log('WAIT', f'action=0x{frame[0]:02x} expected={length} timeout={self.timeout:g}s cleanup={cleanup}')
                self._send(frame, 'cleanup' if cleanup else 'request')
                if not self.reply_ready.wait(self.timeout):
                    raise TimeoutError(f'K10 action 0x{frame[0]:02x}: no reply within {self.timeout:g}s; command outcome uncertain. '
                        f'UDP during this request: {self.received-received} packets from bot, '
                        f'{self.other_sender-other_sender} ignored sender, {self.other_action-other_action} unmatched replies'
                        + (f'; {self.fault_reason}' if self.fault_reason else ''))
                if self.fault.is_set() and not cleanup:
                    raise RuntimeError(self.fault_reason or 'K10 receiver failed during request')
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
            except (OSError, RuntimeError) as error:
                self.log('FAIL', str(error))
                self.fault.set()
                raise
            finally:
                with self.lock:
                    self.pending = None

    def start_heartbeat(self):
        self.log('HEARTBEAT', 'start interval=25ms limit=50ms; no reply expected')
        def run():
            last = time.monotonic()
            deadline = last
            while not self.heartbeat_done.is_set():
                now = time.monotonic()
                if now - last > 0.050 or now >= self.motion_deadline:
                    self.fault_reason = (f'Heartbeat sender gap {(now-last)*1000:.1f} ms exceeded 50 ms; stop/home attempted'
                        if now-last > 0.050 else 'Motion lease expired; stop/home attempted')
                    self.fault.set()
                    self.log('WATCHDOG', self.fault_reason)
                    # No ACK waits here: the sole receiver remains the dispatcher.
                    # Only known stops/home targets, never the expired command.
                    for frame in self.emergency_frames:
                        try:
                            self._send(frame, 'watchdog stop/home')
                        except OSError:
                            pass
                    return  # let the board watchdog expire as well
                if self.fault.is_set():
                    self.log('HEARTBEAT', 'stopped: session faulted')
                    return
                try:
                    self._send(b'\x43', f'heartbeat gap={(now-last)*1000:.2f}ms')
                except OSError as error:
                    self.fault_reason = f'Heartbeat send failed: {error}'
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
        self.log('CLOSE', f'UDP received={self.received} wrong_sender={self.other_sender} unmatched={self.other_action} invalid_events={self.invalid_events}')
        if self.trace is not None:
            self.trace.close()
