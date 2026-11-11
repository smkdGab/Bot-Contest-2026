"""K10 component bench: validated, explicit tests; no arbitrary packet console.

Wire authority: binary-protocol.md, plus SensorsScreen's stream/illumination
helpers. No undocumented sensor configuration or calibration payloads are sent.
"""
import argparse
import math
from pathlib import Path
import re
import shlex
import sys
import time

from .primitives import Robot, STREAMS, ALGORITHMS, SCREENS, servo_limits, filename, wav_bytes
from .safety import termination_cleanup
from .cli import command


STATUS = {1:'invalid parameters', 2:'invalid values', 3:'operation failed / device unsupported',
          4:'service not started', 5:'unknown service', 6:'unknown command', 7:'not master'}
CAPABILITIES = {
    'servo': 'S0..S5: positional angle or continuous speed',
    'motor': 'M1..M4: speed and stop',
    'battery': 'Charge level, 0..100 percent',
    'lidar': '64x8 distance and intensity scans; streaming',
    'compass': 'Heading and magnetic field; streaming',
    'imu': 'Acceleration, bump events, configuration and streaming',
    'huskylens': 'Detections, algorithm selection, learned IDs and streaming',
    'leds': '3 K10 RGB LEDs and 2 expansion LEDs; color and brightness',
    'display': 'K10 TFT screen selection',
    'sound': 'Stored WAV files: list, upload, delete, play, stop, status and volume',
}


def output(value):
    from .display import output as show
    show(value)


def requested(args):
    hidden = {'run', 'upload', 'command', 'component', 'action', 'host', 'port', 'http_port',
              'timeout', 'geomag_rate_format', 'dry_run', 'full'}
    values = {k:v for k,v in vars(args).items() if k not in hidden and v is not None}
    prefix = {'servo':'S', 'motor':'M', 'led':'L'}.get(args.component)
    if prefix:
        for key in ('channel', 'channels'):
            if key in values:
                selected = values[key] if key == 'channels' else [values[key]]
                labels = [prefix+str(c+(args.component=='motor')) for c in selected]
                values[key] = labels if key == 'channels' else labels[0]
    if 'continuous' in values:
        values['continuous'] = ['S'+str(c) for c in values['continuous']]
    labels = {'home':'home_deg', 'min_angle':'min_deg', 'max_angle':'max_deg', 'seconds':'duration_s'}
    if 'value' in values:
        labels['value'] = ('type' if args.action=='type' else 'delta_deg' if args.action=='increment' else
                           'target_deg' if args.action=='angle' else 'percent')
    values = {labels.get(k,k):v for k,v in values.items()}
    return values


def integer_between(low, high):
    def parse(text):
        try:
            value = int(text)
        except ValueError as error:
            raise argparse.ArgumentTypeError('Expected an integer') from error
        if not low <= value <= high:
            raise argparse.ArgumentTypeError(f'Expected {low}..{high}')
        return value
    return parse


def channel(prefix, count, first=0):
    def parse(text):
        match = re.fullmatch(f'{prefix}([0-9]+)', text.upper())
        if not match or not first <= int(match[1]) < first+count:
            raise argparse.ArgumentTypeError(f'Use {prefix}{first}..{prefix}{first+count-1}')
        return int(match[1])-first
    return parse


def finite_seconds(text):
    try:
        value = float(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError('Expected seconds') from error
    if not math.isfinite(value) or not 0 < value <= 60:
        raise argparse.ArgumentTypeError('Seconds must be in (0, 60]')
    return value


def add_components(parser, *, shell=True):
    groups = parser.add_subparsers(dest='component', required=True)
    for name, summary in (('capabilities', 'List usable hardware features from the contest documentation; offline.'),
                          ('ping', 'Check the K10 connection with a matched ping reply.'),
                          ('name', 'Read the K10 name.'), ('battery', 'Read battery charge in percent.')):
        command(groups, name, summary, 'python -m botticelli hardware ' +
                (name if name == 'capabilities' else '--host YOUR_BOT_IP ' + name))
    if shell:
        command(groups, 'shell', 'Register once and run commands interactively. Use help COMMAND, exit or Ctrl+C.',
                'python -m botticelli hardware --host YOUR_BOT_IP shell')
    stop = command(groups, 'stop', 'Stop DC motors and explicitly selected continuous servos; best-effort network stop.',
                   'stop --continuous S0 S4')
    stop.add_argument('--continuous', nargs='+', type=channel('S',6), default=[], help='known continuous channels S0..S5 to zero individually')
    for name in ('servo','motor'):
        root = command(groups, name, 'Control servos S0..S5.' if name == 'servo' else 'Control DC motors M1..M4.', name+' read '+('S0' if name == 'servo' else 'M1'))
        actions = root.add_subparsers(dest='action', required=True)
        example_channel = 'S0' if name == 'servo' else 'M1'
        read = command(actions, 'read', 'Read commanded values; this is not physical encoder feedback.', name+' read '+example_channel)
        read.add_argument('channels', nargs='+', type=channel('S',6) if name=='servo' else channel('M',4,1), help='servo S0..S5' if name == 'servo' else 'motor M1..M4')
        speed = command(actions, 'speed', 'Run a timed speed test, then stop. Servo channels are configured as continuous.', name+' speed '+example_channel+' 15 --seconds 1')
        speed.add_argument('channel', type=channel('S',6) if name=='servo' else channel('M',4,1), help='servo S0..S5' if name == 'servo' else 'motor M1..M4')
        speed.add_argument('value', type=integer_between(-100,100), help='signed speed percent')
        speed.add_argument('--seconds', type=finite_seconds, default=1., help='default 1; maximum 60')
        if name=='servo':
            for name2 in ('angle','increment','type'):
                summary, example = {
                    'angle': ('Set an absolute angle in degrees; return to --home at session exit.', 'servo angle S0 45 --type 180 --home 90'),
                    'increment': ('Add degrees to the reported commanded angle; reject targets outside safe limits.', 'servo increment S0 10 --type 180 --home 90'),
                    'type': ('Configure a servo type. Positional types require --home; continuous servos stop with speed zero.', 'servo type S0 180 --home 90'),
                }[name2]
                action = command(actions, name2, summary, example)
                action.add_argument('channel', type=channel('S',6), help='servo channel S0..S5')
                if name2=='type':
                    action.add_argument('value', choices=('180','270','continuous'))
                else:
                    action.add_argument('value', type=integer_between(-270,270), help='degrees: absolute angle or signed increment')
                    action.add_argument('--type', dest='servo_type', type=int, choices=(180,270), required=True, help='actual positional servo range in degrees')
                    action.add_argument('--seconds', type=finite_seconds, default=1., help='maintain session for movement; default 1 second')
                action.add_argument('--home', type=integer_between(0,270), help='required safe home for positional servos')
                action.add_argument('--min-angle', type=integer_between(0,270), default=0, help='safe lower endpoint in degrees')
                action.add_argument('--max-angle', type=integer_between(0,270), help='safe upper endpoint in degrees; otherwise servo type maximum')
    for name in ('led','board-led'):
        actions = command(groups, name, 'Control unified LEDs L0..L4.' if name == 'led' else 'Control board LEDs by index 0..2.', name+' read'+(' L0' if name == 'led' else '')).add_subparsers(dest='action', required=True)
        for verb in ('set','read','off'):
            example = name+' '+verb+(' L0' if name == 'led' else ' 0' if verb != 'read' else '')+(' --rgb 255 0 0' if verb == 'set' else '')
            p = command(actions, verb, {'set':'Set RGB color and brightness.', 'read':'Read LED state.', 'off':'Turn selected LEDs off.'}[verb], example)
            if name=='led':
                p.add_argument('channels', nargs='+', type=channel('L',5), help='LED channels L0..L4')
            elif verb!='read':
                p.add_argument('index', type=integer_between(0,2), help='board LED index 0..2')
            if verb=='set':
                p.add_argument('--rgb', nargs=3, type=integer_between(0,255), required=True, metavar=('R','G','B'), help='red, green, blue bytes, each 0..255')
                p.add_argument('--brightness', type=integer_between(0,255), default=64, help='raw firmware brightness byte 0..255, default 64')
    screen = command(groups, 'screen', 'Select a K10 TFT screen or move to the next/previous screen.', 'screen app-info')
    screen.add_argument('screen', choices=(*SCREENS,'next','previous'), help='screen name or navigation direction')
    sensor = command(groups, 'sensor', 'Read or configure a sensor; camera means HuskyLens detections, not images.', 'sensor imu read').add_subparsers(dest='sensor',required=True)
    for name in ('lidar','intensity','imu','compass','camera','bump','all'):
        units = {'lidar':'distance in millimetres', 'intensity':'raw LiDAR intensity', 'imu':'acceleration in mg', 'compass':'heading in degrees and field in microteslas', 'camera':'HuskyLens blocks and arrows', 'bump':'impact events; no event is normal', 'all':'all available sensor streams'}
        actions = command(sensor, name, 'Inspect '+units[name]+'.', 'sensor '+name+' watch --seconds 5').add_subparsers(dest='action',required=True)
        for verb in ('read','watch') if name not in ('bump','all') else ('watch',):
            p = command(actions, verb, 'Read one fresh sample as a table or ASCII grid.' if verb == 'read' else 'Watch sensor tables and ASCII grids in this terminal; pipes receive plain snapshots.', 'sensor '+name+' '+verb)
            p.add_argument('--seconds', type=finite_seconds, default=5. if verb=='watch' else 2., help='watch duration or streamed-read deadline, in (0, 60] seconds')
            p.add_argument('--hz', type=integer_between(1,400), default=8 if name in ('lidar','intensity','camera','all') else 10, help='stream rate: LiDAR <=8, camera <=20, compass <=50, IMU <=400 Hz; unused for direct IMU/LiDAR reads')
            p.add_argument('--full', action='store_true',help='show all 512 exact values in numbered ASCII grids instead of a compact ASCII intensity/distance map')
        if name=='imu':
            command(actions, 'config', 'Read the current IMU configuration.', 'sensor imu config')
            command(actions, 'reset', 'Reset IMU configuration; does not reboot the K10.', 'sensor imu reset')
            p = command(actions, 'configure', 'Write raw IMU settings. Confirm codes supported by your device.', 'sensor imu configure --range-code 1 --rate-hz 100 --power-code 0 --hpf-code 1')
            p.add_argument('--range-code', type=integer_between(0,255),required=True, help='raw range code, 0..255')
            p.add_argument('--rate-hz', type=integer_between(1,65535),required=True, help='sampling rate in Hz; device may reject unsupported values')
            p.add_argument('--power-code', type=integer_between(0,255),required=True, help='raw power mode code, 0..255')
            p.add_argument('--hpf-code', type=integer_between(0,255),required=True, help='raw high-pass filter code, 0..255')
        if name=='bump':
            command(actions, 'config', 'Read the bump threshold and debounce interval.', 'sensor bump config')
            p = command(actions, 'configure', 'Set the impact threshold and debounce interval.', 'sensor bump configure --threshold-mg 500 --debounce-ms 100')
            p.add_argument('--threshold-mg', type=integer_between(0,65535),required=True, help='acceleration threshold in mg')
            p.add_argument('--debounce-ms', type=integer_between(0,65535),required=True, help='minimum interval between events in milliseconds')
        if name=='camera':
            command(actions, 'algorithm', 'Read the HuskyLens algorithm and learned-ID count.', 'sensor camera algorithm')
            p = command(actions, 'set-algorithm', 'Select a HuskyLens algorithm; training is done on the sensor.', 'sensor camera set-algorithm color')
            p.add_argument('algorithm',choices=ALGORITHMS, help='algorithm name')
            command(actions, 'controls', 'Read HuskyLens control state and support flags.', 'sensor camera controls')
            p = command(actions, 'illumination', 'Switch HuskyLens illumination on or off.', 'sensor camera illumination on')
            p.add_argument('state',choices=('on','off'), help='illumination state')
    sound = command(groups, 'sound', 'Manage stored PCM WAV files and playback over HTTP.', 'sound list').add_subparsers(dest='action',required=True)
    for verb in ('list','status','stop','play','upload','delete','volume'):
        summary, example = {
            'list': ('List stored sound files.', 'sound list'), 'status': ('Read playback state and errors.', 'sound status'),
            'stop': ('Stop playback and wait up to ten seconds for inactivity.', 'sound stop'),
            'play': ('Play a stored WAV; acceptance does not confirm audible playback.', 'sound play beep.wav'),
            'upload': ('Upload a PCM WAV (<=300 KiB), replacing the same filename. Playback must be inactive.', 'sound upload beep.wav ./beep.wav'),
            'delete': ('Delete a stored WAV while playback is inactive.', 'sound delete beep.wav'),
            'volume': ('Set playback volume from 0 to 100 percent.', 'sound volume 25'),
        }[verb]
        p = command(sound, verb, summary, example)
        if verb in ('play','upload','delete'):
            p.add_argument('filename',help='simple ASCII .wav filename stored on the bot')
        if verb=='upload':
            p.add_argument('path', type=Path, help='PCM WAV, at most 300 KiB; existing filename is replaced')
        if verb=='volume':
            p.add_argument('value',type=integer_between(0,100), help='volume percent, 0..100')


def add_parser(commands):
    parser = command(commands, 'hardware', 'Test K10 components. Live sessions register once; help and dry-run are offline. Put connection options before the component command.', 'python -m botticelli hardware --host YOUR_BOT_IP shell')
    parser.add_argument('--host',help='bot IP or hostname; no default live destination')
    parser.add_argument('--port', type=integer_between(1,65535),default=24642, help='K10 UDP port')
    parser.add_argument('--http-port', type=integer_between(1,65535),default=80, help='HTTP port for sound operations')
    parser.add_argument('--timeout', type=float,default=.25,help='UDP reply timeout seconds (0.05..2)')
    parser.add_argument('--geomag-rate-format',choices=('u16be','u8'),default='u16be', help='explicit compass rate encoding; no automatic fallback')
    parser.add_argument('--dry-run',action='store_true',help='validate and show operations without sockets or token prompt')
    add_components(parser)
    parser.set_defaults(run=run)



def private_token():
    from .safety import private_token as prompt
    return prompt()


def validate(args):
    """Validate a command before registration or any actuator configuration."""
    c, a = args.component, getattr(args, 'action', None)
    if c == 'servo' and a in ('angle', 'increment', 'type'):
        kind = args.value if a == 'type' else str(args.servo_type)
        kind = 'continuous' if kind == 'continuous' else int(kind)
        limits = servo_limits(kind, args.home, args.min_angle, args.max_angle)
        if a == 'angle' and not limits[1] <= args.value <= limits[2]:
            raise ValueError('Requested angle exceeds the configured safe range')
    if c == 'sensor' and a in ('read', 'watch'):
        names = list(STREAMS) if args.sensor == 'all' else [args.sensor]
        if any(args.hz > STREAMS[n][2] for n in names):
            raise ValueError('Rate exceeds selected sensor limit')
    if c == 'sound':
        if a in ('play', 'upload', 'delete'):
            filename(args.filename)
        if a == 'upload':
            if args.path.stat().st_size > 300*1024:
                raise ValueError('WAV must be at most 300 KiB')
            args.upload = wav_bytes(args.path.read_bytes())
    return args


class Bench(Robot):
    """CLI formatting and timed tests; hardware operations live in Robot."""
    def __init__(self, args, *, token_provider=None):
        super().__init__(args.host or 'offline', '', port=args.port, timeout=args.timeout,
                         http_port=args.http_port, geomag_rate_format=args.geomag_rate_format)
        self.args = args
        self.token_provider = token_provider or private_token

    def connect(self):
        if not self.attempted:
            self._token = self.token_provider()
        super().connect()

    def wait(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.transport.fault.is_set():
                raise RuntimeError('Communication or heartbeat failed; tests stopped')
            self.transport.done.wait(min(.02, max(0, deadline-time.monotonic())))

    def execute(self, args):
        validate(args)
        if args.component == 'capabilities':
            output(CAPABILITIES)
            return
        if self.args.dry_run:
            output(dict(test='dry run: '+args.component+' '+getattr(args, 'action', ''),
                register=True, request=requested(args),
                cleanup='stop propulsion; home selected positional servos; disable streams; unregister'))
            return
        self.connect()
        c, a = args.component, getattr(args, 'action', None)
        duration = getattr(args, 'seconds', 0) if c in ('servo', 'motor') else 0
        result = None
        try:
            if c in ('ping', 'name', 'battery'):
                result = getattr(self, c)()
            elif c == 'stop':
                self.continuous.update(args.continuous)
                self.stop()
            elif c in ('servo', 'motor'):
                if a == 'read':
                    result = getattr(self, c+'_read')(*[ch+(c=='motor') for ch in args.channels])
                elif c == 'motor':
                    result = self.motor_speed(args.channel+1, args.value, ttl=min(60, duration+.1))
                else:
                    kind = args.value if a == 'type' else 'continuous' if a == 'speed' else str(args.servo_type)
                    kind = 'continuous' if kind == 'continuous' else int(kind)
                    self.configure_servo(args.channel, kind, home=getattr(args, 'home', None),
                        minimum=getattr(args, 'min_angle', 0), maximum=getattr(args, 'max_angle', None))
                    if a in ('angle', 'increment', 'speed'):
                        if a == 'speed':
                            self.servo_speed(args.channel, 0)
                        result = getattr(self, 'servo_'+a)(args.channel, args.value, ttl=min(60, duration+.1))
            elif c in ('led', 'board-led'):
                base = c.replace('-', '_')
                channels = args.channels if c == 'led' else [args.index] if a != 'read' else []
                if a == 'set':
                    result = getattr(self, base+'_set')(channels if c == 'led' else args.index, *args.rgb, args.brightness)
                else:
                    result = getattr(self, base+'_'+a)(*channels)
            elif c == 'screen':
                result = self.screen(args.screen)
            elif c == 'sensor':
                s = args.sensor
                if a == 'read' and s in ('imu', 'lidar'):
                    result = getattr(self, 'read_'+s)()
                elif a in ('read', 'watch'):
                    self.watch(dict(names=list(STREAMS) if s == 'all' else [s], hz=args.hz,
                        seconds=args.seconds, once=a=='read', full=args.full))
                    return
                elif s == 'imu':
                    if a == 'config': result = self.imu_config()
                    elif a == 'reset': result = self.reset_imu()
                    else: result = self.configure_imu(args.range_code, args.rate_hz, args.power_code, args.hpf_code)
                elif s == 'bump':
                    result = self.bump_config() if a == 'config' else self.configure_bump(args.threshold_mg, args.debounce_ms)
                elif s == 'camera':
                    if a == 'algorithm': result = self.camera_algorithm()
                    elif a == 'controls': result = self.camera_controls()
                    elif a == 'set-algorithm': result = self.set_camera_algorithm(args.algorithm)
                    else: result = self.camera_illumination(args.state == 'on')
            elif c == 'sound':
                if a in ('list', 'status', 'stop'):
                    result = getattr(self, 'sound_'+a)()
                elif a == 'upload': result = self.sound_upload(args.filename, args.upload)
                elif a == 'volume': result = self.sound_volume(args.value)
                else: result = getattr(self, 'sound_'+a)(args.filename)
            if duration:
                self.wait(duration)
        finally:
            if duration:
                self.stop()  # before potentially blocked console output
        label = c+(' '+getattr(args, 'sensor') if c=='sensor' else '')+(' '+a if a else '')
        details = {'request':requested(args)} if not result or result.get('ok') else {}
        if c=='sensor' and a=='read':
            details.update(sensor=args.sensor, _received_at=time.monotonic())
        output({'test':label, **details, **(result or {'ok':True})})

    def watch(self, options):
        from .display import Watch
        enabled, seen, received = [], {}, set()
        records = {name:dict(sensor=name, state='WAITING') for name in options['names']}
        view = Watch(full=options['full'] or options['once'])
        try:
            started = time.monotonic()
            for name in options['names']:
                action = STREAMS[name][0]
                if action in enabled: continue  # IMU and bump share a stream
                status = self.stream(name, hz=options['hz'], allow_status=True)
                if status:
                    self.streams.pop(action, None)
                    records[name] = dict(sensor=name, status=status, error=STATUS[status])
                    if len(options['names']) == 1:
                        view.update(records)
                        raise RuntimeError('Sensor stream rejected')
                else: enabled.append(action)
            deadline, next_draw = time.monotonic()+options['seconds'], 0
            while time.monotonic() < deadline:
                if self.transport.fault.is_set(): raise RuntimeError('Sensor session or heartbeat failed')
                for name in options['names']:
                    reading = self.latest(name)
                    if reading is None or reading.received_at < started or seen.get(name) == reading.received_at: continue
                    seen[name] = reading.received_at
                    received.add(name)
                    if name in ('lidar', 'intensity'):
                        key = 'distance_mm' if name == 'lidar' else 'intensity_raw'
                        values = dict(frame_id=reading.frame_id, timestamp_ms=reading.timestamp_ms,
                                      **{key:reading.values_mm})
                    else: values = reading.values
                    record = dict(sensor=name, _received_at=reading.received_at, **values)
                    if name == 'bump': record['events_seen'] = records[name].get('events_seen', 0)+1
                    records[name] = record
                if time.monotonic() >= next_draw or options['once'] and received:
                    view.update(records)
                    next_draw = time.monotonic()+.25  # UI refresh, not sensor sampling rate
                if options['once'] and received: break
                self.transport.done.wait(.02)
            if not received and options['names'] == ['bump']:
                records['bump'] = dict(sensor='bump', state='NO EVENTS', events_received=0, seconds=options['seconds'])
            for record in records.values():
                if record.get('state') == 'WAITING': record['state'] = 'NO DATA'
            if not options['once'] or not received:
                view.update(records)
            if not received and options['names'] != ['bump']:
                self.transport.fault.set()
                raise TimeoutError('No fresh sensor frames received before the deadline')
        finally:
            view.close()
            for action in enabled:
                frame = self.streams[action]
                self._cleanup(frame[:1]+b'\x00'+frame[2:])
                self.streams.pop(action, None)


def run(args):
    if not math.isfinite(args.timeout) or not .05 <= args.timeout <= 2:
        raise ValueError('--timeout must be in 0.05..2 seconds')
    offline = args.dry_run or args.component == 'capabilities'
    if not offline and not args.host:
        raise ValueError('Set --host to the bot IP or hostname')
    if args.component != 'shell':
        validate(args)
    elif not sys.stdin.isatty():
        raise ValueError('Interactive shell requires a terminal')
    bench = Bench(args)
    with termination_cleanup():
        try:
            if args.component == 'shell':
                if not offline: bench.connect()  # register before the first prompt
                parser = argparse.ArgumentParser(prog='k10', exit_on_error=False)
                add_components(parser, shell=False)
                print('K10 bench. Type help for commands, exit to stop and return selected servos home.')
                while True:
                    try: line = input('k10> ').strip()
                    except EOFError: break
                    if line in ('exit', 'quit'): break
                    if not line: continue
                    if line in ('help', '?'): parser.print_help(); continue
                    try:
                        words = shlex.split(line)
                        if words and words[0] == 'help':
                            words = words[1:] + ['--help']
                        command = validate(parser.parse_args(words))
                    except (SystemExit, argparse.ArgumentError, ValueError, OSError) as error:
                        if not isinstance(error, SystemExit): print(f'Invalid command: {error}', file=sys.stderr)
                        continue
                    bench.execute(command)
            else:
                bench.execute(args)
        finally:
            bench.close()
            for error in bench.cleanup_errors: print('Cleanup not confirmed: '+error, file=sys.stderr)
    if bench.cleanup_errors:
        raise RuntimeError('One or more hardware cleanup operations were not confirmed')
