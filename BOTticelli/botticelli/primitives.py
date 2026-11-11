"""Small K10 operations. Units and payloads follow binary-protocol.md."""
import io
import getpass
import json
import math
import re
import secrets
import signal
import struct
import sys
import threading
import time
import warnings
import wave
from contextlib import contextmanager
from urllib.request import Request, urlopen

from .protocol import K10Transport, decode_reply


def private_token():
    if not sys.stdin.isatty():
        raise ValueError('Hardware sessions require a terminal for private TFT token entry')
    with warnings.catch_warnings():
        warnings.simplefilter('error', getpass.GetPassWarning)
        try:
            token = getpass.getpass('Five-character TFT token (hidden): ')
        except getpass.GetPassWarning as error:
            raise ValueError('Private token entry unavailable') from error
    if len(token) != 5 or any(not 33 <= ord(c) <= 126 for c in token):
        raise ValueError('Use five printable ASCII characters shown on the TFT')
    return token


@contextmanager
def termination_cleanup():
    """Unwind finally blocks on SIGTERM/Ctrl+C; repeated signals allow cleanup."""
    if threading.current_thread() is not threading.main_thread():
        yield
        return
    signals = (signal.SIGINT, signal.SIGTERM)
    previous = {s: signal.getsignal(s) for s in signals}
    def interrupt(signum, frame):
        for s in signals:
            signal.signal(s, signal.SIG_IGN)
        if signum == signal.SIGINT:
            raise KeyboardInterrupt
        raise SystemExit(128 + signum)
    try:
        for s in signals:
            signal.signal(s, interrupt)
        yield
    finally:
        for s, handler in previous.items():
            signal.signal(s, handler)


@contextmanager
def finish_cleanup():
    """Do not interrupt an already-running, bounded network cleanup."""
    if threading.current_thread() is not threading.main_thread():
        yield
        return
    previous = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        for s in previous:
            signal.signal(s, signal.SIG_IGN)
        yield
    finally:
        for s, handler in previous.items():
            signal.signal(s, handler)


STREAMS = {'lidar':(0x83,'lidar',8), 'intensity':(0x89,'lidar_intensity',8),
           'imu':(0xA2,'imu',400), 'compass':(0x93,'compass',50), 'camera':(0x72,'vision',20),
           'bump':(0xA2,'bump',400)}
ALGORITHMS = {'face':0,'tracking':1,'recognition':2,'line':3,'color':4,'tag':5}
SCREENS = {'splash':0,'app-info':1,'huskylens':2,'app-log':3,'svc-log':4,'debug-log':5,'esp-log':6}


def filename(text):
    if not re.fullmatch(r'[A-Za-z0-9_-][A-Za-z0-9_.-]*\.wav',text) or '..' in text or len(text)>48:
        raise ValueError('Use a simple ASCII filename ending in lowercase .wav, at most 48 bytes, without paths or ..')
    return text


def stream_frame(name,hz,fmt,enabled=True):
    action=STREAMS[name][0]
    prefix=bytes([action,int(enabled)])
    if name in ('lidar','intensity'): return prefix+bytes([hz,0])
    if name in ('imu','bump') or name=='compass' and fmt=='u16be': return prefix+struct.pack('>H',hz)
    return prefix+bytes([hz])


def integer(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'Expected integer {low}..{high}')
    return value


def mask(channels, count):
    value = sum(1 << c for c in {integer(c, 0, count-1) for c in channels})
    if not value:
        raise ValueError('Select at least one channel')
    return value


def servo_limits(kind, home, minimum=0, maximum=None):
    if kind not in (180, 270, 'continuous'):
        raise ValueError('Servo type must be 180, 270 or continuous')
    if kind == 'continuous':
        if home is not None:
            raise ValueError('Continuous servos use speed zero, not a home angle')
        return None
    maximum = kind if maximum is None else maximum
    integer(minimum, 0, kind); integer(maximum, 0, kind)
    if minimum >= maximum or home is None or not minimum <= integer(home, 0, kind) <= maximum:
        raise ValueError('Home and safe limits must satisfy min <= home <= max within servo range')
    return home, minimum, maximum


def wav_bytes(data):
    if len(data) > 300*1024:
        raise ValueError('WAV must be at most 300 KiB including headers')
    try:
        with wave.open(io.BytesIO(data), 'rb') as wav:
            if wav.getcomptype() != 'NONE' or wav.getsampwidth() not in (1, 2) or wav.getnchannels() not in (1, 2) or not 8000 <= wav.getframerate() <= 48000:
                raise ValueError('WAV must be PCM, 8/16-bit, mono/stereo, 8000..48000 Hz')
            if len(wav.readframes(wav.getnframes())) != wav.getnframes()*wav.getnchannels()*wav.getsampwidth():
                raise ValueError('Truncated WAV data')
    except (wave.Error, EOFError) as error:
        raise ValueError('Unsupported or malformed PCM WAV') from error
    return data


class Robot:
    """One registered session; configure selected servos before using them.

    Motion has a bounded lease (default 0.5 s). Repeat motion calls to renew it.
    Cleanup is best-effort; ACKs are not physical position feedback.
    """
    def __init__(self, host, token, *, port=24642, timeout=.25, http_port=80, geomag_rate_format='u16be'):
        if not host or not re.fullmatch(r'[A-Za-z0-9.-]+', host):
            raise ValueError('Set host to the K10 IP or hostname')
        integer(port, 1, 65535); integer(http_port, 1, 65535)
        if not math.isfinite(timeout) or not .03 <= timeout <= 2:
            raise ValueError('Timeout must be in 0.03..2 seconds')
        if geomag_rate_format not in ('u16be', 'u8'):
            raise ValueError('Unknown compass rate format')
        self.host, self.port, self.timeout = host, port, timeout
        self.http_port, self.geomag_rate_format = http_port, geomag_rate_format
        self._token = token
        self.transport = None
        self.attempted = self.registered = False
        self.continuous, self.homes, self.streams = set(), {}, {}
        self.cleanup_errors = []

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, kind, value, traceback):
        self.close()
        if kind is None and self.cleanup_errors:
            raise RuntimeError('Cleanup not confirmed: ' + '; '.join(self.cleanup_errors))

    def connect(self):
        if self.registered:
            if self.transport.fault.is_set():
                raise RuntimeError('Session failed; do not reconnect or replay motion')
            return
        if self.attempted:
            raise RuntimeError('Session ended; create a new Robot')
        token, self._token = self._token, ''
        if not isinstance(token, str) or len(token) != 5 or any(not 33 <= ord(c) <= 126 for c in token):
            raise ValueError('Use five printable ASCII characters from the TFT')
        self.transport = K10Transport(self.host, self.port, self.timeout)
        try:
            self.attempted = True
            self.transport.request(b'\x41' + token.encode('ascii'))
            self.registered = True
            self.transport.start_heartbeat()
        except BaseException:
            self.close()
            raise
        finally:
            token = ''

    def _request(self, frame, length=2, decoder='ack', **kwargs):
        if not self.registered:
            raise RuntimeError('Open a registered Robot session first')
        body = self.transport.request(frame, length=length, **kwargs)
        try:
            if decoder in ('servo', 'motor', 'led') and body[2] != frame[1]:
                raise ValueError('Reply channel mask does not match requested channels')
            return decode_reply(decoder, body)
        except (ValueError, struct.error, UnicodeError) as error:
            self.transport.fault.set()
            raise ValueError(f'Malformed component reply: {error}') from error

    def _emergency_frames(self):
        return (tuple(bytes([0x23, 1 << c, 0]) for c in sorted(self.continuous)) + (b'\x28',) +
                tuple(bytes([0x24, 1 << c]) + struct.pack('>h', h[0]) for c, h in sorted(self.homes.items())))

    def _lease(self, seconds):
        if not math.isfinite(seconds) or not 0 < seconds <= 60:
            raise ValueError('Motion lease must be in (0, 60] seconds')
        if not self.registered:
            raise RuntimeError('Open a registered Robot session first')
        self.transport.motion_deadline = time.monotonic() + seconds

    def configure_servo(self, channel, kind, *, home=None, minimum=0, maximum=None):
        integer(channel, 0, 5)
        limits = servo_limits(kind, home, minimum, maximum)
        if kind == 'continuous':
            self.continuous.add(channel)  # uncertain type writes still need a stop
        self.homes.pop(channel, None)
        if self.transport is not None:
            self.transport.emergency_frames = self._emergency_frames()
        self._request(bytes([0x22, 1 << channel, {180:0, 270:1, 'continuous':2}[kind]]))
        if limits is not None:
            self.continuous.discard(channel)
            self.homes[channel] = limits
        self.transport.emergency_frames = self._emergency_frames()

    def servo_angle(self, channel, degrees, *, ttl=.5):
        integer(channel, 0, 5)
        if channel not in self.homes:
            raise ValueError('Configure a positional servo with a safe home first')
        _, low, high = self.homes[channel]
        integer(degrees, low, high)
        self._lease(ttl)
        return self._request(bytes([0x24, 1 << channel]) + struct.pack('>h', degrees))

    def servo_increment(self, channel, delta, *, ttl=.5):
        integer(delta, -270, 270)
        if channel not in self.homes:
            raise ValueError('Configure a positional servo with a safe home first')
        current = self.servo_read(channel)['commanded_degrees'][f'S{channel}']
        _, low, high = self.homes[channel]
        if not low <= current <= high or not low <= current + delta <= high:
            raise ValueError('Reported angle or increment target exceeds safe limits')
        self.servo_angle(channel, current + delta, ttl=ttl)
        return dict(previous_commanded_degrees=current, target_degrees=current+delta)

    def servo_speed(self, channel, percent, *, ttl=.5):
        integer(channel, 0, 5); integer(percent, -100, 100)
        if channel not in self.continuous:
            raise ValueError('Configure a continuous servo first')
        self._lease(ttl)
        return self._request(bytes([0x23, 1 << channel, percent & 255]))

    def servo_read(self, *channels):
        m = mask(channels, 6)
        return self._request(bytes([0x27, m]), 3 + 2*m.bit_count(), 'servo')

    def motor_speed(self, channel, percent, *, ttl=.5):
        integer(channel, 1, 4); integer(percent, -100, 100)
        self._lease(ttl)
        return self._request(bytes([0x21, 1 << (channel-1), percent & 255]))

    def motor_read(self, *channels):
        m = mask([integer(c, 1, 4)-1 for c in channels], 4)
        return self._request(bytes([0x26, m]), 3 + m.bit_count(), 'motor')

    def ping(self):
        return self._request(b'\x44' + secrets.token_bytes(4), 5, 'ping')

    def name(self):
        return self._request(b'\x45', (2, 130), 'name')

    def battery(self, *, optional=False):
        if optional:
            reply = self._request(b'\x29', 3, 'raw', allow_status=True)
            return {'battery_percent': reply[2]} if reply[1] == 0 and reply[2] <= 100 else {'error': 'Battery unavailable'}
        return self._request(b'\x29', 3, 'battery')

    def led_set(self, channels, red, green, blue, brightness=64):
        color = [integer(v, 0, 255) for v in (red, green, blue, brightness)]
        return self._request(bytes([0x51, mask(channels, 5), *color]))

    def led_off(self, *channels):
        return self._request(bytes([0x52, mask(channels, 5)]))

    def led_read(self, *channels):
        m = mask(channels, 5)
        return self._request(bytes([0x54, m]), 3 + 4*m.bit_count(), 'led')

    def board_led_set(self, index, red, green, blue, brightness=64):
        return self._request(bytes([0x31, integer(index, 0, 2),
            *[integer(v, 0, 255) for v in (red, green, blue, brightness)]]))

    def board_led_off(self, index):
        return self._request(bytes([0x32, integer(index, 0, 2)]))

    def board_led_read(self):
        return self._request(b'\x34', (3, 1024), 'board-led')

    def screen(self, name):
        if name not in (*SCREENS, 'next', 'previous'):
            raise ValueError('Unknown screen')
        return self._request(b'\x61' if name == 'next' else b'\x62' if name == 'previous' else bytes([0x63, SCREENS[name]]))

    def read_lidar(self):
        return self._request(b'\x81', 1034, 'lidar')

    def read_imu(self):
        return self._request(b'\xa1', 8, 'accel')

    def imu_config(self):
        return self._request(b'\xa5', 7, 'imu-config')

    def configure_imu(self, range_code, rate_hz, power_code, hpf_code):
        values = [integer(v, 1 if i == 1 else 0, 65535 if i == 1 else 255)
                  for i, v in enumerate((range_code, rate_hz, power_code, hpf_code))]
        return self._request(b'\xa4' + struct.pack('>BHBB', *values))

    def reset_imu(self):
        return self._request(b'\xa6')

    def bump_config(self):
        return self._request(b'\xa8', 6, 'bump-config')

    def configure_bump(self, threshold_mg, debounce_ms):
        return self._request(b'\xa7' + struct.pack('>HH', integer(threshold_mg, 0, 65535), integer(debounce_ms, 0, 65535)))

    def camera_algorithm(self):
        return self._request(b'\x75', 5, 'algorithm')

    def set_camera_algorithm(self, name):
        if name not in ALGORITHMS:
            raise ValueError('Unknown camera algorithm')
        return self._request(bytes([0x74, ALGORITHMS[name]]))

    def camera_controls(self):
        return self._request(b'\x78', 9, 'controls')

    def camera_illumination(self, enabled):
        if type(enabled) is not bool:
            raise ValueError('Use a boolean for illumination')
        return self._request(bytes([0x71, int(enabled)]))

    def imu_stream_state(self):
        body = self._request(b'\xa3', 5, 'raw')
        if body[2] not in (0, 1) or not 1 <= struct.unpack_from('<H', body, 3)[0] <= 400:
            self.transport.fault.set()
            raise ValueError('Invalid IMU stream state')
        return dict(enabled=bool(body[2]), hz=struct.unpack_from('<H', body, 3)[0])

    def camera_stream_state(self):
        body = self._request(b'\x73', 4, 'raw')
        if body[2] not in (0, 1) or not 1 <= body[3] <= 20:
            self.transport.fault.set()
            raise ValueError('Invalid camera stream state')
        return dict(enabled=bool(body[2]), hz=body[3])

    def reset_lidar(self):
        return self._request(b'\x87')

    def stream(self, name, enabled=True, hz=8, *, allow_status=False):
        if name not in STREAMS:
            raise ValueError('Unknown sensor')
        integer(hz, 1, STREAMS[name][2])
        if type(enabled) is not bool:
            raise ValueError('Use a boolean for stream enable')
        frame = stream_frame(name, hz, self.geomag_rate_format, enabled)
        if enabled:
            self.streams[frame[0]] = frame  # include uncertain enables in cleanup
        reply = self._request(frame, decoder='raw', allow_status=allow_status)
        if not enabled and reply[1] == 0:
            self.streams.pop(frame[0], None)
        return reply[1]

    def latest(self, name, *, max_age=.5):
        if not self.registered:
            raise RuntimeError('Open a registered Robot session first')
        if name not in STREAMS:
            raise ValueError('Unknown sensor')
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('Sensor maximum age must be positive and finite')
        key = STREAMS[name][1]
        with self.transport.lock:
            reading = getattr(self.transport, key) if name in ('lidar', 'intensity') else self.transport.sensors.get(key)
        return reading if reading is not None and time.monotonic()-reading.received_at <= max_age else None

    def read_sensor(self, name, *, timeout=2, hz=8):
        if not self.registered:
            raise RuntimeError('Open a registered Robot session first')
        if name not in STREAMS:
            raise ValueError('Unknown sensor')
        integer(hz, 1, STREAMS[name][2])
        if not math.isfinite(timeout) or not 0 < timeout <= 60:
            raise ValueError('Sensor timeout must be in (0, 60]')
        started = time.monotonic()
        was_enabled = STREAMS[name][0] in self.streams
        try:
            if not was_enabled:
                self.stream(name, hz=hz)
            while time.monotonic()-started < timeout:
                if self.transport.fault.is_set():
                    raise RuntimeError('Sensor session failed')
                reading = self.latest(name)
                if reading is not None and reading.received_at >= started:
                    return reading
                self.transport.done.wait(.01)
            if name == 'bump':
                return None  # no impact is an ordinary result
            self.transport.fault.set()
            raise TimeoutError('No fresh sensor frame before deadline')
        finally:
            if not was_enabled:
                frame = self.streams.get(STREAMS[name][0])
                if frame:
                    self._cleanup(frame[:1] + b'\x00' + frame[2:])
                    self.streams.pop(frame[0], None)

    def read_compass(self, **kwargs):
        return self.read_sensor('compass', **kwargs)

    def read_camera(self, **kwargs):
        return self.read_sensor('camera', **kwargs)

    def read_intensity(self, **kwargs):
        return self.read_sensor('intensity', **kwargs)

    def read_bump(self, **kwargs):
        return self.read_sensor('bump', **kwargs)

    def _http(self, path, method='GET', data=None):
        if not self.registered or self.transport.fault.is_set():
            raise RuntimeError('Open a healthy registered Robot session first')
        url = f'http://{self.host}:{self.http_port}{path}'
        with urlopen(Request(url, data=data, method=method), timeout=max(2, self.timeout)) as response:
            return response.read(1024*1024)

    def sound_status(self):
        return json.loads(self._http('/sounds'))

    def sound_list(self):
        return self.sound_status()

    def _sound_idle(self):
        catalog = self.sound_status()
        if catalog.get('playback_state') in ('starting', 'playing', 'stopping') or catalog.get('playing'):
            raise ValueError('Wait for inactive sound playback before this operation')
        return catalog

    def sound_play(self, name):
        filename(name)
        if name not in self._sound_idle().get('sounds', []):
            raise ValueError('Sound is not in the bot catalog; upload it first')
        self._http('/sounds/' + name + '/play', 'POST')

    def sound_upload(self, name, data):
        filename(name); wav_bytes(data)
        if self._sound_idle().get('free_bytes', 0) < len(data) + 128*1024:
            raise ValueError('Upload must leave 128 KiB free')
        self._http('/sounds/' + name, 'POST', data)

    def sound_delete(self, name):
        filename(name); self._sound_idle()
        self._http('/sounds/' + name, 'DELETE')

    def sound_volume(self, percent):
        self._http('/sounds/volume?value=' + str(integer(percent, 0, 100)), 'POST')

    def sound_stop(self):
        self._http('/sounds/stop', 'POST')
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            state = self.sound_status()
            if state.get('playback_state') not in ('starting', 'playing', 'stopping') and not state.get('playing'):
                return
            self.transport.done.wait(.1)
        raise TimeoutError('Sound stop not confirmed within ten seconds')

    def _cleanup(self, frame):
        try:
            self.transport.request(frame, cleanup=True)
        except Exception as error:
            self.cleanup_errors.append(f'0x{frame[0]:02x}: {error}')

    def stop(self):
        if self.transport is None or not self.registered:
            return
        before = len(self.cleanup_errors)
        for ch in sorted(self.continuous):
            self._cleanup(bytes([0x23, 1 << ch, 0]))
        self._cleanup(b'\x28')
        if len(self.cleanup_errors) != before:
            raise RuntimeError('Stop not confirmed')
        self.transport.motion_deadline = float('inf')

    @finish_cleanup()
    def close(self):
        if self.transport is None:
            self._token = ''
            return
        self.transport.motion_deadline = float('inf')
        try:
            if self.registered:
                try:
                    self.stop()
                except RuntimeError:
                    pass
                for ch, (home, _, _) in sorted(self.homes.items()):
                    self._cleanup(bytes([0x24, 1 << ch]) + struct.pack('>h', home))
            for frame in self.streams.values():
                self._cleanup(frame[:1] + b'\x00' + frame[2:])
            if self.attempted:
                self._cleanup(b'\x42')
        finally:
            self.transport.close()
            self.transport = None
            self.registered = False
            self._token = ''
