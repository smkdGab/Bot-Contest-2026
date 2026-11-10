"""K10 component bench: validated, explicit tests; no arbitrary packet console.

Wire authority: binary-protocol.md, plus SensorsScreen's stream/illumination
helpers. No undocumented sensor configuration or calibration payloads are sent.
"""
import argparse
from dataclasses import dataclass, field
import getpass
import json
import math
from pathlib import Path
import re
import shlex
import secrets
import struct
import sys
import time
from urllib.request import Request, urlopen
import warnings
import wave

from .lidar import decode_frame


STATUS = {1:'invalid parameters', 2:'invalid values', 3:'operation failed / device unsupported',
          4:'service not started', 5:'unknown service', 6:'unknown command', 7:'not master'}
STREAMS = {'lidar':(0x83,'lidar',8), 'intensity':(0x89,'lidar_intensity',8),
           'imu':(0xA2,'imu',400), 'compass':(0x93,'compass',50), 'camera':(0x72,'vision',20),
           'bump':(0xA2,'bump',400)}
ALGORITHMS = {'face':0,'tracking':1,'recognition':2,'line':3,'color':4,'tag':5}
SCREENS = {'splash':0,'app-info':1,'huskylens':2,'app-log':3,'svc-log':4,'debug-log':5,'esp-log':6}
CAPABILITIES = {
    'servo': 'S0..S5: angle, increment, read, type, timed continuous speed',
    'motor': 'M1..M4: timed speed, read; stop supports explicit continuous-servo channels',
    'sensors': 'LiDAR distance/intensity; IMU acceleration/config/bump events; compass stream; HuskyLens detections/algorithm/illumination/controls',
    'outputs': 'L0..L4 RGB LEDs; board LEDs 0..2; K10 TFT screens; sound catalog/upload/play/status/volume/stop/delete',
    'diagnostics': 'ping, name, battery, bounded watch-all, shell, offline dry-run',
    'unavailable': {
        'compass calibration/config and LiDAR stats/config': 'Payload layouts absent from supplied reference; firmware source required.',
        'HuskyLens RGB light/LCD': 'V1 reports unsupported; controls reports support flags. Train on the sensor.',
        'temperature/humidity, ambient light, microphones, buttons, GPIO, IR, ultrasonic': 'No documented remote commands in this firmware.',
        'camera image/video': 'HuskyLens detections only; no general UDP image stream.',
    },
    'geomag_discrepancy': 'Protocol reference uses u16be stream rate; SensorsScreen uses u8. Default u16be; explicitly select --geomag-rate-format u8 if required. No automatic fallback.',
}


def output(value):
    print(json.dumps(value, allow_nan=False), flush=True)


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
    for name in ('capabilities','ping','name','battery'):
        groups.add_parser(name)
    if shell:
        groups.add_parser('shell', help='interactive component tests; token prompted once when needed')
    stop = groups.add_parser('stop', help='DC stop plus optional known continuous-servo channels')
    stop.add_argument('--continuous', nargs='+', type=channel('S',6), default=[])
    for name in ('servo','motor'):
        root = groups.add_parser(name)
        actions = root.add_subparsers(dest='action', required=True)
        read = actions.add_parser('read', help='read commanded values, not physical encoder feedback')
        read.add_argument('channels', nargs='+', type=channel('S',6) if name=='servo' else channel('M',4,1))
        speed = actions.add_parser('speed', help='bounded test; stops at the end or on interruption')
        speed.add_argument('channel', type=channel('S',6) if name=='servo' else channel('M',4,1))
        speed.add_argument('value', type=integer_between(-100,100), help='signed speed percent')
        speed.add_argument('--seconds', type=finite_seconds, default=1., help='default 1; maximum 60')
        if name=='servo':
            for name2 in ('angle','increment','type'):
                action = actions.add_parser(name2)
                action.add_argument('channel', type=channel('S',6))
                if name2=='type':
                    action.add_argument('value', choices=('180','270','continuous'))
                else:
                    action.add_argument('value', type=integer_between(-270,270), help='degrees: absolute angle or signed increment')
                    action.add_argument('--type', dest='servo_type', type=int, choices=(180,270), required=True)
                    action.add_argument('--min-angle', type=integer_between(0,270), default=0)
                    action.add_argument('--max-angle', type=integer_between(0,270))
                    action.add_argument('--seconds', type=finite_seconds, default=1., help='maintain session for movement; default 1 second')
    for name in ('led','board-led'):
        actions = groups.add_parser(name).add_subparsers(dest='action', required=True)
        for verb in ('set','read','off'):
            p = actions.add_parser(verb)
            if name=='led':
                p.add_argument('channels', nargs='+', type=channel('L',5))
            elif verb!='read':
                p.add_argument('index', type=integer_between(0,2))
            if verb=='set':
                p.add_argument('--rgb', nargs=3, type=integer_between(0,255), required=True, metavar=('R','G','B'))
                p.add_argument('--brightness', type=integer_between(0,255), default=64, help='raw firmware brightness byte 0..255, default 64')
    screen = groups.add_parser('screen')
    screen.add_argument('screen', choices=(*SCREENS,'next','previous'))
    sensor = groups.add_parser('sensor').add_subparsers(dest='sensor',required=True)
    for name in ('lidar','intensity','imu','compass','camera','bump','all'):
        actions = sensor.add_parser(name).add_subparsers(dest='action',required=True)
        for verb in ('read','watch') if name not in ('bump','all') else ('watch',):
            p = actions.add_parser(verb)
            p.add_argument('--seconds', type=finite_seconds, default=5. if verb=='watch' else 2.)
            p.add_argument('--hz', type=integer_between(1,400), default=8 if name in ('lidar','intensity','camera','all') else 10)
            p.add_argument('--full', action='store_true',help='include all 512 LiDAR/intensity cells in watched frames')
        if name=='imu':
            actions.add_parser('config')
            actions.add_parser('reset')
            p = actions.add_parser('configure')
            p.add_argument('--range-code', type=integer_between(0,255),required=True)
            p.add_argument('--rate-hz', type=integer_between(1,65535),required=True)
            p.add_argument('--power-code', type=integer_between(0,255),required=True)
            p.add_argument('--hpf-code', type=integer_between(0,255),required=True)
        if name=='bump':
            actions.add_parser('config')
            p = actions.add_parser('configure')
            p.add_argument('--threshold-mg', type=integer_between(0,65535),required=True)
            p.add_argument('--debounce-ms', type=integer_between(0,65535),required=True)
        if name=='camera':
            actions.add_parser('algorithm')
            p = actions.add_parser('set-algorithm'); p.add_argument('algorithm',choices=ALGORITHMS)
            actions.add_parser('controls')
            p = actions.add_parser('illumination'); p.add_argument('state',choices=('on','off'))
    sound = groups.add_parser('sound').add_subparsers(dest='action',required=True)
    for verb in ('list','status','stop','play','upload','delete','volume'):
        p = sound.add_parser(verb)
        if verb in ('play','upload','delete'):
            p.add_argument('filename',help='simple ASCII .wav filename stored on the bot')
        if verb=='upload':
            p.add_argument('path', type=Path, help='PCM WAV, at most 300 KiB; existing filename is replaced')
        if verb=='volume':
            p.add_argument('value',type=integer_between(0,100))


def add_parser(commands):
    parser = commands.add_parser('hardware',help='K10 hardware component bench; see hardware --help')
    parser.add_argument('--host',help='bot IP or hostname; no default live destination')
    parser.add_argument('--port', type=integer_between(1,65535),default=24642)
    parser.add_argument('--http-port', type=integer_between(1,65535),default=80)
    parser.add_argument('--timeout', type=float,default=.25,help='UDP reply timeout seconds (0.05..2)')
    parser.add_argument('--geomag-rate-format',choices=('u16be','u8'),default='u16be')
    parser.add_argument('--dry-run',action='store_true',help='validate and show operations without sockets or token prompt')
    add_components(parser)
    parser.set_defaults(run=run)


@dataclass
class Wire:
    frame: bytes
    length: int | tuple[int,int] = 2
    decoder: str = 'ack'


@dataclass
class Plan:
    label: str
    wires: list[Wire] = field(default_factory=list)
    protected: bool = False
    continuous: set[int] = field(default_factory=set)
    duration: float = 0
    watch: dict = field(default_factory=dict)
    http: dict = field(default_factory=dict)
    increment: dict = field(default_factory=dict)
    upload: bytes = b''


def mask(channels):
    return sum(1 << c for c in set(channels))


def filename(text):
    if not re.fullmatch(r'[A-Za-z0-9_-][A-Za-z0-9_.-]*\.wav',text) or '..' in text or len(text)>48:
        raise ValueError('Use a simple ASCII filename ending in lowercase .wav, at most 48 bytes, without paths or ..')
    return text


def build_plan(args):
    c, action = args.component, getattr(args,'action',None)
    plan = Plan(c+(' '+getattr(args,'sensor') if c=='sensor' else '')+(' '+action if action else ''))
    def wire(frame,length=2,decoder='ack'):
        plan.wires.append(Wire(frame,length,decoder))
    if c=='capabilities': return plan
    if c=='ping': wire(b'\x44'+secrets.token_bytes(4),5,'ping')
    elif c=='name': wire(b'\x45',(2,130),'name')
    elif c=='battery': wire(b'\x29',3,'battery')
    elif c=='stop':
        plan.continuous = set(args.continuous)
        plan.protected = bool(plan.continuous)
        for ch in sorted(plan.continuous): wire(bytes([0x23,1<<ch,0]))
        wire(b'\x28')
    elif c in ('servo','motor'):
        if action=='read':
            m = mask(args.channels)
            wire(bytes([0x27 if c=='servo' else 0x26,m]),3+len(set(args.channels))*(2 if c=='servo' else 1),c)
        else:
            plan.protected = True
            ch, value = args.channel,args.value
            if c=='servo':
                typ = {'180':0,'270':1,'continuous':2}[value] if action=='type' else 2 if action=='speed' else (0 if args.servo_type==180 else 1)
                if typ==2: plan.continuous.add(ch)
                wire(bytes([0x22,1<<ch,typ]))
            if action in ('angle','increment'):
                maximum = args.servo_type if args.max_angle is None else args.max_angle
                if not 0 <= args.min_angle < maximum <= args.servo_type:
                    raise ValueError('Safe angle limits must satisfy 0 <= min < max <= servo type')
                if action=='angle':
                    if not args.min_angle <= value <= maximum:
                        raise ValueError('Requested angle exceeds the configured safe range')
                    wire(bytes([0x24,1<<ch])+struct.pack('>h',value))
                else:
                    wire(bytes([0x27,1<<ch]),5,'servo')
                    plan.increment = dict(channel=ch,delta=value,minimum=args.min_angle,maximum=maximum)
                plan.duration = args.seconds
            elif action=='speed':
                if c=='servo': wire(bytes([0x23,1<<ch,0]))
                wire(bytes([0x23 if c=='servo' else 0x21,1<<ch,value & 255]))
                plan.duration = args.seconds
    elif c in ('led','board-led'):
        plan.protected = action!='read'
        if c=='led':
            m = mask(args.channels)
            if action=='set': wire(bytes([0x51,m,*args.rgb,args.brightness]))
            elif action=='off': wire(bytes([0x52,m]))
            else: wire(bytes([0x54,m]),3+4*len(set(args.channels)),'led')
        elif action=='set': wire(bytes([0x31,args.index,*args.rgb,args.brightness]))
        elif action=='off': wire(bytes([0x32,args.index]))
        else: wire(b'\x34',(3,1024),'board-led')
    elif c=='screen':
        plan.protected = True
        wire(b'\x61' if args.screen=='next' else b'\x62' if args.screen=='previous' else bytes([0x63,SCREENS[args.screen]]))
    elif c=='sensor':
        s = args.sensor
        if action in ('watch','read'):
            if action=='read' and s=='imu': wire(b'\xa1',8,'accel')
            elif action=='read' and s=='lidar': wire(b'\x81',1034,'lidar')
            else:
                names = ['lidar','intensity','imu','compass','camera','bump'] if s=='all' else [s]
                if any(args.hz>STREAMS[n][2] for n in names):
                    raise ValueError('Rate exceeds selected sensor limit (LiDAR 8, camera 20, compass 50, IMU 400 Hz)')
                plan.protected = True
                plan.watch = dict(names=names,hz=args.hz,seconds=args.seconds,once=action=='read',full=args.full)
        elif s=='imu':
            if action=='config': wire(b'\xa5',7,'imu-config')
            elif action=='reset': plan.protected=True; wire(b'\xa6')
            else:
                plan.protected=True
                wire(b'\xa4'+struct.pack('>BHBB',args.range_code,args.rate_hz,args.power_code,args.hpf_code))
        elif s=='bump':
            if action=='config': wire(b'\xa8',6,'bump-config')
            else:
                plan.protected=True; wire(b'\xa7'+struct.pack('>HH',args.threshold_mg,args.debounce_ms))
        elif s=='camera':
            if action=='algorithm': wire(b'\x75',5,'algorithm')
            elif action=='controls': wire(b'\x78',9,'controls')
            elif action=='set-algorithm': plan.protected=True; wire(bytes([0x74,ALGORITHMS[args.algorithm]]))
            elif action=='illumination': plan.protected=True; wire(bytes([0x71,int(args.state=='on')]))
    elif c=='sound':
        if action in ('play','upload','delete'): filename(args.filename)
        plan.http = dict(action=action)
        if action in ('play','upload','delete'): plan.http['filename']=args.filename
        if action=='volume': plan.http['value']=args.value
        if action=='upload':
            if args.path.stat().st_size>300*1024: raise ValueError('WAV must be at most 300 KiB including headers')
            data = args.path.read_bytes()
            if len(data)>300*1024: raise ValueError('WAV must be at most 300 KiB including headers')
            try:
                with wave.open(str(args.path),'rb') as wav:
                    if wav.getcomptype()!='NONE' or wav.getsampwidth() not in (1,2) or wav.getnchannels() not in (1,2) or not 8000<=wav.getframerate()<=48000:
                        raise ValueError('WAV must be PCM, 8/16-bit, mono/stereo, 8000..48000 Hz')
                    if len(wav.readframes(wav.getnframes())) != wav.getnframes()*wav.getnchannels()*wav.getsampwidth():
                        raise ValueError('Truncated WAV data')
            except (wave.Error, EOFError) as error:
                raise ValueError('Unsupported or malformed PCM WAV') from error
            plan.http['path']=str(args.path)
            plan.http['size_bytes']=len(data)
            plan.upload=data
    return plan


def decode(kind, body):
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


def stream_frame(name,hz,fmt,enabled=True):
    action=STREAMS[name][0]
    prefix=bytes([action,int(enabled)])
    if name in ('lidar','intensity'): return prefix+bytes([hz,0])
    if name in ('imu','bump') or name=='compass' and fmt=='u16be': return prefix+struct.pack('>H',hz)
    return prefix+bytes([hz])


def private_token():
    if not sys.stdin.isatty(): raise ValueError('Protected tests require a terminal for private TFT token entry')
    with warnings.catch_warnings():
        warnings.simplefilter('error',getpass.GetPassWarning)
        try: token=getpass.getpass('Five-character TFT token (hidden): ')
        except getpass.GetPassWarning as error: raise ValueError('Private token entry unavailable') from error
    if len(token)!=5 or any(not 33<=ord(c)<=126 for c in token):
        raise ValueError('Use the five printable ASCII characters shown on the TFT')
    return token


class Bench:
    def __init__(self,args, *, token_provider=None):
        self.args=args
        self.token_provider=token_provider or private_token
        self.transport=None
        self.attempted=self.registered=False
        self.continuous=set()
        self.streams={}
        self.cleanup_errors=[]

    def connect(self,protected=False):
        if self.transport is None:
            from .k10 import K10Transport
            self.transport=K10Transport(self.args.host,self.args.port,self.args.timeout)
        if protected and not self.registered:
            token=self.token_provider()
            self.attempted=True
            try: self.transport.request(b'\x41'+token.encode('ascii'))
            finally: token=''
            self.registered=True
            self.transport.start_heartbeat()

    def request(self,wire):
        body=self.transport.request(wire.frame,length=wire.length)
        if wire.decoder in ('servo','motor','led') and body[2]!=wire.frame[1]:
            self.transport.fault.set()
            raise ValueError('Reply channel mask does not match requested channels')
        try: return decode(wire.decoder,body)
        except (ValueError,struct.error,UnicodeError) as error:
            self.transport.fault.set()
            raise ValueError('Malformed component reply') from error

    def wait(self,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            if self.transport.fault.is_set(): raise RuntimeError('Communication or heartbeat failed; tests stopped')
            self.transport.done.wait(min(.02,max(0,deadline-time.monotonic())))

    def execute(self,plan):
        if plan.label=='capabilities': output(CAPABILITIES); return
        if self.args.dry_run:
            output({'test':plan.label,'register':plan.protected,'requests':[
                {'hex':w.frame.hex(' '),'reply_length':w.length} for w in plan.wires],
                'stream_enable':[stream_frame(n,plan.watch['hz'],self.args.geomag_rate_format).hex(' ') for n in plan.watch.get('names',[]) if n!='bump' or 'imu' not in plan.watch['names']],
                'duration_seconds':plan.duration,'watch':plan.watch,'http':plan.http,'increment':plan.increment,
                'cleanup':['zero each configured continuous servo','STOP_ALL_MOTORS','disable streams','UNREGISTER'] if plan.protected else []})
            return
        if plan.http: self.sound(plan.http,plan.upload); return
        self.connect(plan.protected)
        self.continuous.update(plan.continuous)  # before type ACK: an uncertain write still needs cleanup
        stopped=False
        try:
            result=None
            for wire in plan.wires:
                result=self.request(wire)
                if wire.frame[0]==0x22 and wire.frame[2]!=2:
                    for ch in range(6):
                        if wire.frame[1] & (1<<ch): self.continuous.discard(ch)
            if plan.increment:
                d=plan.increment; current=result['commanded_degrees'][f'S{d["channel"]}']
                target=current+d['delta']
                if not d['minimum']<=current<=d['maximum'] or not d['minimum']<=target<=d['maximum']:
                    raise ValueError('Reported angle or increment target exceeds safe limits; no angle sent')
                self.request(Wire(bytes([0x24,1<<d['channel']])+struct.pack('>h',target)))
                result=dict(target_degrees=target,previous_commanded_degrees=current)
            if plan.watch: self.watch(plan.watch)
            else:
                if plan.duration: self.wait(plan.duration)
                if plan.duration and plan.label.endswith('speed'):
                    self.stop()
                    stopped=True
                # Console backpressure must never extend a motor test.
                output({'test':plan.label,**(result or {'ok':True})})
        finally:
            if plan.duration and plan.label.endswith('speed') and not stopped:
                # A timed test must stop before returning to an interactive prompt.
                self.stop()

    def stop(self):
        errors=[]
        for ch in sorted(self.continuous):
            try: self.transport.request(bytes([0x23,1<<ch,0]),cleanup=True)
            except (OSError,RuntimeError) as error: errors.append(str(error))
        try: self.transport.request(b'\x28',cleanup=True)
        except (OSError,RuntimeError) as error: errors.append(str(error))
        self.cleanup_errors.extend(errors)
        if errors: raise RuntimeError('Stop not confirmed: '+'; '.join(errors))

    def watch(self,options):
        transport=self.transport; enabled=[]; seen={}; received=set()
        try:
            with transport.lock:
                for name in options['names']:
                    key=STREAMS[name][1]
                    if key in ('lidar','lidar_intensity'): setattr(transport,key,None)
                    else: transport.sensors.pop(key,None)
            for name in options['names']:
                frame=stream_frame(name,options['hz'],self.args.geomag_rate_format)
                if frame in enabled: continue  # bump and IMU share one stream
                self.streams[frame[0]]=frame
                enabled.append(frame)
                reply=transport.request(frame,allow_status=True)
                if reply[1]:
                    enabled.remove(frame)
                    self.streams.pop(frame[0],None)
                    output({'sensor':name,'status':reply[1],'error':STATUS[reply[1]]})
                    if len(options['names'])==1: raise RuntimeError('Sensor stream rejected')
            deadline=time.monotonic()+options['seconds']
            while time.monotonic()<deadline:
                if transport.fault.is_set(): raise RuntimeError('Sensor session or heartbeat failed')
                with transport.lock:
                    readings={name:getattr(transport,STREAMS[name][1]) if name in ('lidar','intensity') else transport.sensors.get(STREAMS[name][1]) for name in options['names']}
                for name, reading in readings.items():
                    if reading is None or seen.get(name)==reading.received_at: continue
                    seen[name]=reading.received_at; received.add(name)
                    if name in ('lidar','intensity'):
                        values=dict(frame_id=reading.frame_id,timestamp_ms=reading.timestamp_ms,
                                    **{'distance_mm' if name=='lidar' else 'intensity_raw':reading.values_mm})
                    else: values=reading.values
                    if name in ('lidar','intensity') and not options.get('full'):
                        key='distance_mm' if name=='lidar' else 'intensity_raw'
                        valid=[v for v in reading.values_mm if v]
                        values[key]=dict(count=len(reading.values_mm),nonzero_count=len(valid),
                            minimum=min(valid) if valid else None,maximum=max(valid) if valid else None,
                            mean=round(sum(valid)/len(valid),2) if valid else None)
                    output({'sensor':name,'age_seconds':max(0,time.monotonic()-reading.received_at),**values})
                if options['once'] and received: break
                transport.done.wait(.02)
            if not received and options['names']==['bump']:
                output({'sensor':'bump','events_received':0,'seconds':options['seconds']})
            elif not received:
                transport.fault.set()
                raise TimeoutError('No fresh sensor frames received before the deadline')
        finally:
            for frame in enabled:
                try:
                    transport.request(frame[:1]+b'\x00'+frame[2:],cleanup=True)
                    self.streams.pop(frame[0],None)
                except (OSError,RuntimeError) as error: self.cleanup_errors.append(str(error))

    def http(self,path,method='GET',data=None):
        url=f'http://{self.args.host}:{self.args.http_port}{path}'
        with urlopen(Request(url,data=data,method=method),timeout=max(2,self.args.timeout)) as response:
            return response.read(1024*1024)

    def sound(self,options,upload=b''):
        action=options['action']
        if action in ('list','status'):
            output(json.loads(self.http('/sounds'))); return
        if action in ('play','upload','delete'):
            catalog=json.loads(self.http('/sounds'))
            if catalog.get('playback_state') in ('starting','playing','stopping') or catalog.get('playing'):
                raise ValueError('Stop sound and wait for inactive playback before file changes or another test')
            if action=='play' and options['filename'] not in catalog.get('sounds',[]):
                raise ValueError('Sound is not in the bot catalog; upload it first')
            if action=='upload':
                data=upload
                if not data: raise ValueError('No validated WAV bytes supplied')
                if catalog.get('free_bytes',0)<len(data)+128*1024: raise ValueError('Upload must leave 128 KiB free')
                self.http('/sounds/'+options['filename'],'POST',data)
            elif action=='delete': self.http('/sounds/'+options['filename'],'DELETE')
            else: self.http('/sounds/'+options['filename']+'/play','POST')
        elif action=='stop':
            self.http('/sounds/stop','POST')
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                status=json.loads(self.http('/sounds'))
                if status.get('playback_state') not in ('starting','playing','stopping') and not status.get('playing'): break
                time.sleep(.1)
            else: raise TimeoutError('Sound stop not confirmed within ten seconds')
        else: self.http('/sounds/volume?value='+str(options['value']),'POST')
        output({'test':'sound '+action,'accepted':True})

    def close(self):
        if self.transport is None: return
        try:
            if self.registered:
                try: self.stop()
                except (OSError,RuntimeError): pass
            for frame in list(self.streams.values()):
                try: self.transport.request(frame[:1]+b'\x00'+frame[2:],cleanup=True)
                except (OSError,RuntimeError) as error: self.cleanup_errors.append(str(error))
            if self.attempted:
                try: self.transport.request(b'\x42',cleanup=True)
                except (OSError,RuntimeError) as error: self.cleanup_errors.append(str(error))
        finally:
            self.transport.close(); self.transport=None


def run(args):
    if not math.isfinite(args.timeout) or not .05<=args.timeout<=2:
        raise ValueError('--timeout must be in 0.05..2 seconds')
    if args.component!='capabilities' and not args.dry_run and (not args.host or not re.fullmatch(r'[A-Za-z0-9.-]+',args.host)):
        raise ValueError('Set --host to the bot IP or hostname, without a URL or port')
    bench=Bench(args)
    try:
        if args.component=='shell':
            if not sys.stdin.isatty(): raise ValueError('Interactive shell requires a terminal')
            parser=argparse.ArgumentParser(prog='k10',exit_on_error=False)
            add_components(parser,shell=False)
            print('K10 bench. Type help, capabilities, or a component command. exit quits. No command history is saved.')
            while True:
                try: line=input('k10> ').strip()
                except EOFError: break
                if line in ('exit','quit'): break
                if not line: continue
                if line in ('help','?'): parser.print_help(); continue
                try: command=parser.parse_args(shlex.split(line)); plan=build_plan(command)
                except (SystemExit,argparse.ArgumentError,ValueError,OSError) as error:
                    if not isinstance(error,SystemExit): print(f'Invalid command: {error}',file=sys.stderr)
                    continue
                # Validation mistakes do not end the shell; transport/decoder
                # failures do. Never automatically reconnect or replay a command.
                bench.execute(plan)
        else:
            bench.execute(build_plan(args))
    finally:
        bench.close()
        for error in bench.cleanup_errors: print('Cleanup not confirmed: '+error,file=sys.stderr)
    if bench.cleanup_errors: raise RuntimeError('One or more hardware cleanup operations were not confirmed')
