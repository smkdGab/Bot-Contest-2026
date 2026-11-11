"""K10 component bench: validated, explicit tests; no arbitrary packet console.

Wire authority: binary-protocol.md, plus SensorsScreen's stream/illumination
helpers. No undocumented sensor configuration or calibration payloads are sent.
"""
import argparse
from functools import partial
import math
from pathlib import Path
import re
import shlex
import sys
import time
from textwrap import fill

from .primitives import Robot, STREAMS, ALGORITHMS, SCREENS, servo_limits, filename, wav_bytes
from .primitives import private_token, termination_cleanup
from .protocol import STATUS


SENSOR_NAMES = {
    'lidar': 'DFRobot SEN0682 64x8 DTOF LiDAR',
    'imu': 'DFRobot SEN0409 LIS2DW12 accelerometer',
    'compass': 'DFRobot SEN0619 BMM350 magnetometer',
    'camera': 'DFRobot SEN0305 HuskyLens K210',
}
SENSOR_NAMES['intensity'] = SENSOR_NAMES['lidar']
SENSOR_NAMES['bump'] = SENSOR_NAMES['imu']
CAPABILITIES = {
    'servo': 'DFRobot DFR1216: S0..S5; separate positional and continuous commands',
    'dc-motor': 'DFRobot DFR1216: M1..M4 DC motors, speed and stop',
    'battery': 'DFRobot DFR1216: charge level, 0..100 percent',
    'lidar': SENSOR_NAMES['lidar'] + ': distance, intensity and streaming',
    'compass': SENSOR_NAMES['compass'] + ': heading, magnetic field and streaming',
    'imu': SENSOR_NAMES['imu'] + ': acceleration, bump events and configuration',
    'huskylens': SENSOR_NAMES['camera'] + ': detections, algorithms and streaming',
    'leds': 'UniHiker K10: 3 NeoPixels; DFR1216: 2 WS2812 LEDs',
    'display': 'UniHiker K10: TFT screen selection',
    'sound': 'UniHiker K10 speaker: stored WAV files, playback and volume',
}


class HelpFormatter(argparse.ArgumentDefaultsHelpFormatter):
    def _get_help_string(self, action):
        return action.help if action.default is None else super()._get_help_string(action)


class Parser(argparse.ArgumentParser):
    """UNIX-style help generated from the arguments actually accepted."""
    def __init__(self, *args, parent=None, **kwargs):
        self.connection = getattr(parent, 'connection', None)
        kwargs.setdefault('formatter_class', HelpFormatter)
        super().__init__(*args, **kwargs)
        self._positionals.title = 'ARGUMENTS'
        self._optionals.title = 'OPTIONS'

    def add_argument(self, *args, **kwargs):
        action = super().add_argument(*args, **kwargs)
        if action.choices is not None:
            action.metavar = action.metavar or (action.option_strings[-1].lstrip('-') if action.option_strings else action.dest).upper().replace('-', '_')
            action.help = (action.help or action.dest) + '; choices: ' + ', '.join(map(str, action.choices))
        if action.required:
            action.help = (action.help or action.dest) + ' (required)'
        return action

    def add_subparsers(self, **kwargs):
        kwargs.setdefault('title', 'commands (required)')
        kwargs.setdefault('help', 'choose a command (required)')
        kwargs.setdefault('parser_class', partial(type(self), parent=self))
        return super().add_subparsers(**kwargs)

    def synopsis(self):
        operands, options, subcommand = [], [], []
        formatter = self._get_formatter()
        for action in self._actions:
            if isinstance(action, argparse._HelpAction):
                continue
            if isinstance(action, argparse._SubParsersAction):
                subcommand = ['COMMAND [ARGS]']
                continue
            value = formatter._format_args(action, action.metavar or action.dest.upper())
            if action.option_strings:
                token = action.option_strings[-1] + (' ' + value if action.nargs != 0 else '')
                options.append(token if action.required else '[' + token + ']')
            else:
                operands.append(value)
        prog = self.prog.removeprefix('k10').lstrip() if self.prog == 'k10' or self.prog.startswith('k10 ') else self.prog
        if self.connection is not None and self.connection is not self:
            prog = prog.replace(self.connection.prog, self.connection.prog + ' [CONNECTION OPTIONS]', 1)
        return ' '.join(filter(None, [prog, *operands, *options, *subcommand]))

    def format_usage(self):
        return fill('usage: ' + self.synopsis(), width=78, subsequent_indent='    ') + '\n'

    def command_synopses(self):
        groups = [a for a in self._actions if isinstance(a, argparse._SubParsersAction)]
        if not groups:
            yield self.synopsis()
        for group in groups:
            for child in group.choices.values():
                yield from child.command_synopses()

    def format_help(self):
        formatter = self._get_formatter()
        formatter.start_section('DESCRIPTION')
        formatter.add_text(self.description)
        formatter.end_section()
        for group in self._action_groups:
            formatter.start_section(group.title.upper())
            formatter.add_arguments(group._group_actions)
            formatter.end_section()
        if self.connection is not None and self.connection is not self:
            formatter.start_section('CONNECTION OPTIONS (before the component command)')
            formatter.add_arguments(a for a in self.connection._optionals._group_actions if a.dest != 'help')
            formatter.end_section()
        for action in self._actions:
            if isinstance(action, argparse._SubParsersAction):
                formatter.start_section('COMMAND SYNOPSIS')
                for child in action.choices.values():
                    for syntax in child.command_synopses():
                        formatter.add_text(syntax)
                formatter.end_section()
        formatter.start_section('EXAMPLE')
        formatter.add_text(self.synopsis())
        formatter.add_text('Replace UPPERCASE operands; [brackets] mark optional arguments. Use COMMAND --help for details.')
        formatter.end_section()
        return self.format_usage() + '\n' + formatter.format_help()

    def parse_known_args(self, args=None, namespace=None):
        self.set_defaults(example=self.synopsis())
        return super().parse_known_args(args, namespace)

    def error(self, message):
        self.print_usage(sys.stderr)
        help_command = self.prog.removeprefix('k10 ').strip() + ' --help' if self.prog != 'k10' else 'help'
        self.exit(2, f'{self.prog}: {message}\nExample: {self.synopsis()}\nHelp: {help_command}\n')


def command(commands, name, summary, *, device=None):
    parser = commands.add_parser(name, help=summary,
        description=f'{device}: {summary}' if device else summary)
    if device:
        parser.set_defaults(device_name=device)
    return parser


def output(value):
    from .display import output as show
    show(value)


def requested(args):
    hidden = {'run', 'upload', 'command', 'component', 'action', 'host', 'port', 'http_port',
              'timeout', 'geomag_rate_format', 'dry_run', 'full', 'sensor', 'device_name', 'view', 'board', 'family', 'example'}
    values = {k:v for k,v in vars(args).items() if k not in hidden and v is not None}
    prefix = {'servo':'S', 'dc-motor':'M', 'led':'L'}.get(args.component)
    if prefix:
        for key in ('channel', 'channels'):
            if key in values:
                selected = values[key] if key == 'channels' else [values[key]]
                labels = [prefix+str(c+(args.component=='dc-motor')) for c in selected]
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
            raise argparse.ArgumentTypeError(f'got {text!r}; expected an integer from {low} to {high}') from error
        if not low <= value <= high:
            raise argparse.ArgumentTypeError(f'got {text!r}; allowed range is {low}..{high}')
        return value
    return parse


def channel(prefix, count, first=0):
    def parse(text):
        match = re.fullmatch(f'{prefix}([0-9]+)', text.upper())
        if not match or not first <= int(match[1]) < first+count:
            raise argparse.ArgumentTypeError(f'got {text!r}; use {prefix}{first}..{prefix}{first+count-1}')
        return int(match[1])-first
    return parse


def finite_seconds(text):
    try:
        value = float(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f'got {text!r}; expected a duration in seconds') from error
    if not math.isfinite(value) or not 0 < value <= 60:
        raise argparse.ArgumentTypeError(f'got {text!r}; duration must be greater than 0 and at most 60 seconds')
    return value


def add_sensor_commands(root, name):
    device = SENSOR_NAMES.get(name, 'All documented sensors')
    root.set_defaults(sensor=name, device_name=device)
    actions = root.add_subparsers(dest='action', required=True)
    for verb in ('watch',) if name in ('bump', 'all') else ('read', 'watch'):
        p = command(actions, verb, 'Read one fresh sample as a table or color heatmap.' if verb == 'read' else
            'Watch readings in this terminal; pipes receive plain snapshots.', device=device)
        p.add_argument('--seconds', type=finite_seconds, default=5. if verb=='watch' else 2., help='watch duration or streamed-read deadline, in (0, 60] seconds')
        p.add_argument('--hz', type=integer_between(1,400), default=8 if name in ('lidar','intensity','camera','all') else 10, help='stream rate: LiDAR <=8, HuskyLens <=20, compass <=50, IMU <=400 Hz; unused for direct IMU/LiDAR reads')
        p.set_defaults(full=False, cell=None)
        if name in ('lidar', 'intensity', 'all'):
            p.add_argument('--full', action='store_true', help='show all 512 exact values in numbered grids')
            p.add_argument('--cell', nargs=2, type=int, metavar=('ROW','COL'), help='inspect row 0..7, column 0..63')
    if name=='imu':
        command(actions, 'config', 'Read acceleration configuration.', device=device)
        command(actions, 'reset', 'Reset acceleration configuration; does not reboot the K10.', device=device)
        p = command(actions, 'configure', 'Write raw acceleration settings; confirm codes supported by your device.', device=device)
        p.add_argument('--range-code', type=integer_between(0,255), required=True, help='raw range code, 0..255')
        p.add_argument('--rate-hz', type=integer_between(1,65535), required=True, help='sampling rate in Hz; device may reject unsupported values')
        p.add_argument('--power-code', type=integer_between(0,255), required=True, help='raw power mode code, 0..255')
        p.add_argument('--hpf-code', type=integer_between(0,255), required=True, help='raw high-pass filter code, 0..255')
    elif name=='bump':
        command(actions, 'config', 'Read impact threshold and debounce interval.', device=device)
        p = command(actions, 'configure', 'Set impact threshold and debounce interval.', device=device)
        p.add_argument('--threshold-mg', type=integer_between(0,65535), required=True, help='acceleration threshold in mg')
        p.add_argument('--debounce-ms', type=integer_between(0,65535), required=True, help='minimum interval between events in milliseconds')
    elif name=='camera':
        command(actions, 'algorithm', 'Read algorithm and learned-ID count.', device=device)
        p = command(actions, 'set-algorithm', 'Select an algorithm; training is done on the sensor.', device=device)
        p.add_argument('algorithm', choices=ALGORITHMS, help='algorithm name')
        command(actions, 'controls', 'Read control state and support flags.', device=device)
        p = command(actions, 'illumination', 'Switch illumination on or off.', device=device)
        p.add_argument('state', choices=('on','off'), help='illumination state')
    return actions


def add_components(parser, *, shell=True):
    groups = parser.add_subparsers(dest='component', required=True)
    for name, summary in (('capabilities', 'List usable hardware features from the contest documentation; offline.'),
                          ('ping', 'Check the K10 connection with a matched ping reply.'),
                          ('name', 'Read the K10 name.'), ('battery', 'Read DFR1216 battery charge in percent.')):
        command(groups, name, summary,
                device='DFRobot DFR1216' if name == 'battery' else 'UniHiker K10')
    if shell:
        command(groups, 'shell', 'Register once and run commands interactively. Use help COMMAND, exit or Ctrl+C.', device='UniHiker K10')
    stop = command(groups, 'stop', 'Stop DC motors and explicitly selected continuous servos; best-effort network stop.', device='DFRobot DFR1216')
    stop.add_argument('--continuous', nargs='+', type=channel('S',6), default=[], help='known continuous channels S0..S5 to zero individually')
    servo = command(groups, 'servo', 'Select positional servos (degrees) or continuous servos (speed).', device='DFRobot DFR1216, S0..S5')
    families = servo.add_subparsers(dest='family', required=True)
    for family, parent, summary in (
            ('positional', families, 'Position in degrees; defaults: type 270, home 0.'),
            ('continuous', families, 'Rotation speed in percent; zero stops rotation.'),
            ('dc-motor', groups, 'DC motors on M1..M4, separate from the servo outputs S0..S5.')):
        dc = family == 'dc-motor'
        positional = family == 'positional'
        root = command(parent, family, summary, device='DFRobot DFR1216')
        actions = root.add_subparsers(dest='action', required=True)
        verbs = ('angle','increment','configure','read') if positional else ('speed','read') if dc else ('speed','configure')
        for verb in verbs:
            description = {'angle':'Set angle in degrees; return home at session exit.',
                'increment':'Add degrees to the reported angle, within safe limits.',
                'configure':'Configure only this servo channel.',
                'read':'Read commanded values; no physical encoder feedback.',
                'speed':'Set signed speed percent (-100..100), then stop after --seconds.'}[verb]
            p = command(actions, verb, description, device='DFRobot DFR1216')
            p.add_argument('channels' if verb=='read' else 'channel', nargs='+' if verb=='read' else None,
                type=channel('M',4,1) if dc else channel('S',6), metavar='CHANNEL',
                help='DC motor output M1..M4' if dc else 'positional servo S0..S5' if positional else 'continuous servo S0..S5')
            if verb in ('angle','increment','speed'):
                p.add_argument('value', type=integer_between(0 if verb=='angle' else -270 if positional else -100, 270 if positional else 100),
                    metavar='ANGLE' if verb=='angle' else 'DELTA' if verb=='increment' else 'PERCENT',
                    help='angle in degrees' if verb=='angle' else 'signed change in degrees' if verb=='increment' else 'signed speed percent; 0 stops')
                p.add_argument('--seconds', type=finite_seconds, default=1., metavar='SECONDS',
                    help=('hold speed for SECONDS, then stop' if verb == 'speed' else
                          'keep the session active for SECONDS; home on exit') + '; greater than 0, at most 60')
            if positional and verb != 'read':
                p.add_argument('--type', dest='servo_type', type=int, choices=(180,270), default=270, help='positional servo range in degrees')
                p.add_argument('--home', type=integer_between(0,270), default=0, help='home angle on exit, in degrees')
                p.add_argument('--min-angle', type=integer_between(0,270), default=0, help='safe lower endpoint in degrees')
                p.add_argument('--max-angle', type=integer_between(0,270), help='safe upper endpoint; default: servo type range')
    led = command(groups, 'led', 'Control K10 NeoPixels and DFR1216 WS2812 LEDs.', device='UniHiker K10 / DFRobot DFR1216')
    led.set_defaults(board=False)
    led_actions = led.add_subparsers(dest='action', required=True)
    board = command(led_actions, 'board', 'Use the DFR1216 board LED service, indexes 0..2.', device='DFRobot DFR1216')
    board.set_defaults(board=True)
    for board_mode, actions, device in (
            (False, led_actions, 'UniHiker K10 NeoPixels / DFR1216 WS2812 LEDs'),
            (True, board.add_subparsers(dest='action', required=True), 'DFRobot DFR1216')):
        for verb in ('set','read','off'):
            p = command(actions, verb, {'set':'Set RGB color and brightness.', 'read':'Read LED state.', 'off':'Turn selected LEDs off.'}[verb], device=device)
            if not board_mode:
                p.add_argument('channels', nargs='+', type=channel('L',5), help='L0..L2: K10 NeoPixels; L3/L4: DFR1216 WS2812 LEDs')
            elif verb!='read':
                p.add_argument('index', type=integer_between(0,2), help='board LED index 0..2')
            if verb=='set':
                p.add_argument('--rgb', nargs=3, type=integer_between(0,255), required=True, metavar=('R','G','B'), help='red, green, blue bytes, each 0..255')
                p.add_argument('--brightness', type=integer_between(0,255), default=64, help='raw firmware brightness byte 0..255, default 64')
    screen = command(groups, 'screen', 'Select a K10 TFT screen or move to the next/previous screen.', device='UniHiker K10 TFT')
    screen.add_argument('screen', choices=(*SCREENS,'next','previous'), help='screen name or navigation direction')
    sensor = command(groups, 'sensor', 'Read or configure sensors, grouped by physical device.').add_subparsers(dest='device', required=True)
    for name in ('lidar', 'imu', 'compass', 'huskylens', 'all'):
        stream = 'camera' if name == 'huskylens' else name
        device = SENSOR_NAMES.get(stream, 'All documented sensors')
        root = command(sensor, name, device+'.')
        root.set_defaults(view='')
        actions = add_sensor_commands(root, stream)
        if name in ('lidar', 'imu'):
            view = 'intensity' if name == 'lidar' else 'bump'
            summary = 'Raw signal strength from this LiDAR.' if name == 'lidar' else 'Impact detection from this accelerometer; no event is normal.'
            child = command(actions, view, summary, device=device)
            child.set_defaults(view=view)
            add_sensor_commands(child, view)
    sound = command(groups, 'sound', 'Manage stored PCM WAV files and playback over HTTP.', device='UniHiker K10 speaker').add_subparsers(dest='action',required=True)
    for verb in ('list','status','stop','play','upload','delete','volume'):
        summary = {
            'list': 'List stored sound files.', 'status': 'Read playback state and errors.',
            'stop': 'Stop playback and wait up to ten seconds for inactivity.',
            'play': 'Play a stored WAV; acceptance does not confirm audible playback.',
            'upload': 'Upload a PCM WAV (<=300 KiB), replacing the same filename. Playback must be inactive.',
            'delete': 'Delete a stored WAV while playback is inactive.',
            'volume': 'Set playback volume from 0 to 100 percent.',
        }[verb]
        p = command(sound, verb, summary, device='UniHiker K10 speaker')
        if verb in ('play','upload','delete'):
            p.add_argument('filename',help='simple ASCII .wav filename stored on the bot')
        if verb=='upload':
            p.add_argument('path', type=Path, help='PCM WAV, at most 300 KiB; existing filename is replaced')
        if verb=='volume':
            p.add_argument('value',type=integer_between(0,100), help='volume percent, 0..100')


def add_parser(commands):
    parser = command(commands, 'hardware', 'Test K10 components. Live sessions register once; help and dry-run are offline. Put connection options before the component command.', device='UniHiker K10 / DFRobot DFR1216')
    parser.connection = parser
    parser.add_argument('--host',help='bot IP or hostname (required for live commands)')
    parser.add_argument('--port', type=integer_between(1,65535),default=24642, help='K10 UDP port')
    parser.add_argument('--http-port', type=integer_between(1,65535),default=80, help='HTTP port for sound operations')
    parser.add_argument('--timeout', type=float,default=.25,help='UDP reply timeout seconds (0.05..2)')
    parser.add_argument('--geomag-rate-format',choices=('u16be','u8'),default='u16be', help='explicit compass rate encoding; no automatic fallback')
    parser.add_argument('--dry-run',action='store_true',help='validate and show operations without sockets or token prompt')
    add_components(parser)
    parser.set_defaults(run=run)



def validate(args):
    """Validate a command before registration or any actuator configuration."""
    c, a = args.component, getattr(args, 'action', None)
    if c == 'servo' and args.family == 'positional' and a != 'read':
        try:
            limits = servo_limits(args.servo_type, args.home, args.min_angle, args.max_angle)
        except ValueError as error:
            raise ValueError(f'S{args.channel}: {error}; --type {args.servo_type}, --home {args.home}, --min-angle {args.min_angle}, --max-angle {args.max_angle if args.max_angle is not None else args.servo_type}') from error
        if a == 'angle' and not limits[1] <= args.value <= limits[2]:
            raise ValueError(f'S{args.channel}: angle {args.value} degrees is outside the safe range {limits[1]}..{limits[2]} degrees')
    if c == 'sensor' and a in ('read', 'watch'):
        cell = getattr(args, 'cell', None)
        if cell is not None and not (0 <= cell[0] < 8 and 0 <= cell[1] < 64):
            raise ValueError(f'Cell {cell[0]}, {cell[1]} is invalid; use --cell ROW COL with row 0..7 and column 0..63')
        names = list(STREAMS) if args.sensor == 'all' else [args.sensor]
        for name in names:
            if args.hz > STREAMS[name][2]:
                raise ValueError(f'{SENSOR_NAMES[name]}: --hz {args.hz} exceeds the limit; use --hz 1..{STREAMS[name][2]}')
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
                raise RuntimeError(self.transport.fault_reason or 'Communication failed; session stopped')
            self.transport.done.wait(min(.02, max(0, deadline-time.monotonic())))

    def execute(self, args):
        validate(args)
        c, a = args.component, getattr(args, 'action', None)
        label = ' '.join(filter(None, (c, getattr(args, 'family', ''), getattr(args, 'device', ''), getattr(args, 'view', ''),
            'board' if getattr(args, 'board', False) else '', a)))
        device = getattr(args, 'device_name', 'UniHiker K10')
        if args.component == 'capabilities':
            output(CAPABILITIES)
            return
        if self.args.dry_run:
            output(dict(test='dry run: '+label, device=device, register=True, request=requested(args),
                cleanup='stop propulsion; home selected positional servos; disable streams; unregister'))
            return
        self.connect()
        duration = getattr(args, 'seconds', 0) if c in ('servo', 'dc-motor') else 0
        motion_started = None
        held_seconds = None
        result = None
        try:
            if c in ('ping', 'name', 'battery'):
                result = getattr(self, c)()
            elif c == 'stop':
                self.continuous.update(args.continuous)
                self.stop()
            elif c in ('servo', 'dc-motor'):
                if a == 'read':
                    result = (self.motor_read if c == 'dc-motor' else self.servo_read)(*[ch+(c=='dc-motor') for ch in args.channels])
                elif c == 'dc-motor':
                    result = self.motor_speed(args.channel+1, args.value, ttl=min(60, duration+.1))
                else:
                    kind = args.servo_type if args.family == 'positional' else 'continuous'
                    self.configure_servo(args.channel, kind, home=getattr(args, 'home', None),
                        minimum=getattr(args, 'min_angle', 0), maximum=getattr(args, 'max_angle', None))
                    if a in ('angle', 'increment', 'speed'):
                        if a == 'speed':
                            self.servo_speed(args.channel, 0)
                        result = getattr(self, 'servo_'+a)(args.channel, args.value, ttl=min(60, duration+.1))
            elif c == 'led':
                base = 'board_led' if args.board else 'led'
                channels = [args.index] if args.board and a != 'read' else [] if args.board else args.channels
                if a == 'set':
                    result = getattr(self, base+'_set')(args.index if args.board else channels, *args.rgb, args.brightness)
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
                        seconds=args.seconds, once=a=='read', full=args.full, cell=getattr(args, 'cell', None)))
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
                motion_started = time.monotonic()
                self.wait(duration)
        finally:
            if duration:
                failed = sys.exc_info()[0] is not None
                if motion_started is not None:
                    held_seconds = time.monotonic()-motion_started
                try:
                    self.stop()  # before potentially blocked console output
                except RuntimeError:
                    if not failed:
                        raise  # pending cleanup errors are reported separately
        details = {'request':requested(args)} if not result or result.get('ok') else {}
        if held_seconds is not None:
            details['command_held_s'] = round(held_seconds, 3)
        if c=='sensor' and a=='read':
            details.update(sensor=args.sensor, _received_at=time.monotonic(),
                           _cell=getattr(args, 'cell', None), _full=args.full)
        output({'test':label, 'device':device, **details, **(result or {'ok':True})})

    def watch(self, options):
        from .display import Watch
        enabled, seen, received = [], {}, set()
        records = {name:dict(sensor=name, device=SENSOR_NAMES[name], state='WAITING') for name in options['names']}
        view = Watch(full=options['full'], cell=options.get('cell'))
        try:
            started = time.monotonic()
            for name in options['names']:
                action = STREAMS[name][0]
                if action in enabled: continue  # IMU and bump share a stream
                status = self.stream(name, hz=options['hz'], allow_status=True)
                if status:
                    self.streams.pop(action, None)
                    records[name].update(state='REJECTED', status=status, error=STATUS[status])
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
                    record = dict(sensor=name, device=SENSOR_NAMES[name], _received_at=reading.received_at, **values)
                    if name == 'bump': record['events_seen'] = records[name].get('events_seen', 0)+1
                    records[name] = record
                if time.monotonic() >= next_draw or options['once'] and received:
                    view.update(records)
                    next_draw = time.monotonic()+.25  # UI refresh, not sensor sampling rate
                if options['once'] and received: break
                self.transport.done.wait(.02)
            if not received and options['names'] == ['bump']:
                records['bump'].update(state='NO EVENTS', events_received=0, seconds=options['seconds'])
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
        raise ValueError(f'--timeout {args.timeout} is invalid; use 0.05..2 seconds')
    offline = args.dry_run or args.component == 'capabilities'
    if not offline and not args.host:
        raise ValueError('Missing --host: use hardware --host YOUR_BOT_IP before the component command')
    if args.component != 'shell':
        validate(args)
    elif not sys.stdin.isatty():
        raise ValueError('Interactive shell requires a terminal')
    bench = Bench(args)
    with termination_cleanup():
        try:
            if args.component == 'shell':
                parser = Parser(prog='k10', description='K10 hardware shell; connection is already registered. Type exit to finish.')
                add_components(parser, shell=False)
                if not offline: bench.connect()  # register before the first prompt
                print('K10 bench. Type help for commands, exit to stop and return selected servos home.')
                while True:
                    try: line = input('k10> ').strip()
                    except EOFError: break
                    if line in ('exit', 'quit'): break
                    if not line: continue
                    if line in ('help', '?'): parser.print_help(); continue
                    parsed = None
                    try:
                        words = shlex.split(line)
                        if words and words[0] == 'help':
                            words = words[1:] + ['--help']
                        parsed = parser.parse_args(words)
                        validate(parsed)
                    except (SystemExit, argparse.ArgumentError, ValueError, OSError) as error:
                        if not isinstance(error, SystemExit):
                            print(f'Invalid command: {error}\nExample: {parsed.example if parsed else "help"}', file=sys.stderr)
                        continue
                    try:
                        bench.execute(parsed)
                    except ValueError as error:
                        if bench.transport is not None and bench.transport.fault.is_set():
                            raise
                        print(f'Invalid command: {error}\nExample: {parsed.example}', file=sys.stderr)
            else:
                bench.execute(args)
        finally:
            bench.close()
            for error in bench.cleanup_errors: print('Cleanup not confirmed: '+error, file=sys.stderr)
    if bench.cleanup_errors:
        raise RuntimeError('One or more hardware cleanup operations were not confirmed')
