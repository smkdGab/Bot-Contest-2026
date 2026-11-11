"""Hardware bench integration uses ephemeral loopback UDP/HTTP servers only."""
import argparse
from contextlib import contextmanager
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import shlex
import socket
import struct
import threading
import time
import wave

import pytest

from botticelli import hardware as hw
from botticelli.k10 import K10Transport
from botticelli.primitives import stream_frame


def parse(command, server=None, *, dry=False):
    parser=argparse.ArgumentParser()
    commands=parser.add_subparsers(dest='command',required=True)
    hw.add_parser(commands)
    prefix='hardware '+('--dry-run ' if dry else '')
    if server: prefix+=f'--host 127.0.0.1 --port {server.address[1]} '
    return parser.parse_args(shlex.split(prefix+command))


class Bot:
    def __init__(self):
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1',0)); self.sock.settimeout(.005)
        self.address=self.sock.getsockname()
        self.packets=[]; self.drop=set(); self.reject=set(); self.wrong=set(); self.malformed=set()
        self.angles=[90]*6; self.types=[0]*6; self.active={}; self.bump=False
        self.done=threading.Event(); self.target=None; self.serial=0
        self.thread=threading.Thread(target=self.run,daemon=True); self.thread.start()

    def send(self,body,target):
        self.sock.sendto(body+struct.pack('>II',self.serial,123456),target)

    def emit(self):
        if not self.target: return
        self.serial+=1; n=self.serial & 65535
        for action in list(self.active):
            if action in (0x83,0x89):
                self.send(bytes([0x8F if action==0x83 else 0x8E,0])+struct.pack('<HIH',n,1234,512)+struct.pack('<512H',*([123]*512)),self.target)
            elif action==0xA2:
                self.send(b'\xaf\x00'+struct.pack('<Ihhh',self.serial,1,-2,1000),self.target)
                if self.bump: self.send(b'\xa0\x00'+struct.pack('<IBhhh',self.serial,1,100,200,1000),self.target)
            elif action==0x93: self.send(b'\x9f\x00'+struct.pack('<HHhhh',n,270,1,-2,45),self.target)
            elif action==0x72: self.send(b'\x7f\x00'+struct.pack('<HHHBB',n,320,240,0,0),self.target)

    def run(self):
        next_emit=time.monotonic()
        while not self.done.is_set():
            if time.monotonic()>=next_emit:
                self.emit(); next_emit=time.monotonic()+.025
            try: packet,sender=self.sock.recvfrom(2048)
            except socket.timeout: continue
            except OSError: return
            self.packets.append((time.monotonic(),packet)); self.target=sender
            a=packet[0]
            if a==0x43: continue
            if a in self.wrong:
                self.send(b'\x45\x00wrong',sender)
                with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as other:
                    other.sendto(bytes([a,0])+bytes(8),sender)
                continue
            if a in self.reject:
                self.send(bytes([a,3]),sender); continue
            reply=bytes([a,0])
            if a==0x22:
                for i in range(6):
                    if packet[1] & 1<<i: self.types[i]=packet[2]
            elif a==0x24:
                for i in range(6):
                    if packet[1] & 1<<i: self.angles[i]=struct.unpack_from('>h',packet,2)[0]
            elif a==0x27: reply+=packet[1:2]+b''.join(struct.pack('>h',self.angles[i]) for i in range(6) if packet[1]&1<<i)
            elif a==0x26: reply+=packet[1:2]+bytes(packet[1].bit_count())
            elif a==0x54: reply+=packet[1:2]+bytes([10,20,30,64])*packet[1].bit_count()
            elif a==0x34: reply+=b'{"leds":[]}'
            elif a==0x44: reply=packet
            elif a==0x45: reply+=b'loopback'
            elif a==0x29: reply+=bytes([85])
            elif a==0xA1: reply+=struct.pack('<hhh',1,-2,1000)
            elif a==0xA5: reply+=struct.pack('<BHBB',1,100,0,1)
            elif a==0xA8: reply+=struct.pack('<HH',500,100)
            elif a==0xA3: reply+=bytes([1])+struct.pack('<H',100)
            elif a==0x73: reply+=bytes([1,8])
            elif a==0x75: reply+=bytes([4])+struct.pack('<H',3)
            elif a==0x78: reply+=bytes([1,1,0,0,1,0,0x31])
            elif a==0x81: reply+=struct.pack('<HIH',4,1234,512)+struct.pack('<512H',*([123]*512))
            elif a in (0x83,0x89,0x93,0xA2,0x72):
                if packet[1]: self.active[a]=True; self.emit()  # event before ACK
                else: self.active.pop(a,None)
            if a in self.drop: continue
            if a in self.malformed: reply=bytes([a])
            self.send(reply,sender)

    def frames(self): return [p for _,p in self.packets if p[0]!=0x43]
    def close(self):
        self.done.set(); self.thread.join(timeout=1); self.sock.close()


@pytest.fixture
def bot():
    b=Bot()
    try: yield b
    finally: b.close()


@contextmanager
def bench(args):
    b=hw.Bench(args,token_provider=lambda:'test!')
    try: yield b
    finally: b.close()


@pytest.mark.parametrize('channel',range(6))
def test_every_servo_channel_configures_before_absolute_angle_and_cleans_up(bot,channel):
    args=parse(f'servo angle S{channel} 120 --type 180 --home 90 --seconds .001',bot)
    with bench(args) as b: b.execute(hw.validate(args))
    assert bot.frames()[:3]==[b'\x41test!',bytes([0x22,1<<channel,0]),bytes([0x24,1<<channel,0,120])]
    assert bot.frames()[-3:]==[b'\x28',bytes([0x24,1<<channel,0,90]),b'\x42']
    assert not b.cleanup_errors


def test_increment_reads_then_validates_and_uses_configured_angle_command(bot):
    args=parse('servo increment S4 15 --type 270 --home 90 --min-angle 30 --max-angle 150 --seconds .001',bot)
    with bench(args) as b: b.execute(hw.validate(args))
    assert bot.frames()[1:4]==[b'\x22\x10\x01',b'\x27\x10',b'\x24\x10\x00\x69']
    args=parse('servo increment S4 100 --type 270 --home 90 --max-angle 150 --seconds .001',bot)
    before=len(bot.frames())
    with bench(args) as b:
        with pytest.raises(ValueError,match='safe limits'): b.execute(hw.validate(args))
    assert [p for p in bot.frames()[before:] if p[0]==0x24]==[b'\x24\x10\x00\x5a']


@pytest.mark.parametrize('failure',['reject','drop'])
def test_failed_registration_never_configures_or_moves(bot,failure):
    getattr(bot,failure).add(0x41)
    args=parse('servo angle S0 90 --type 180 --home 90',bot)
    with bench(args) as b:
        with pytest.raises((RuntimeError,TimeoutError)): b.execute(hw.validate(args))
    assert bot.frames()==[b'\x41test!',b'\x42']


@pytest.mark.parametrize('failure',['drop','wrong','malformed','reject'])
def test_bad_reply_stops_session_without_motion_retry(bot,failure):
    getattr(bot,failure).add(0x24)
    args=parse('servo angle S2 120 --type 180 --home 90 --seconds .001',bot)
    with bench(args) as b:
        with pytest.raises((RuntimeError,TimeoutError)): b.execute(hw.validate(args))
        with pytest.raises(RuntimeError): b.execute(hw.validate(parse('servo speed S0 10 --seconds .001',bot)))
    assert sum(p==b'\x24\x04\x00\x78' for p in bot.frames())==1
    assert not any(p[0]==0x23 and p[2] for p in bot.frames())
    assert bot.frames()[-3:]==[b'\x28',b'\x24\x04\x00\x5a',b'\x42']


def test_uncertain_continuous_type_is_individually_zeroed_before_dc_stop_and_unregister(bot):
    bot.drop.add(0x22)
    args=parse('servo speed S5 -25 --seconds .001',bot)
    with bench(args) as b:
        with pytest.raises(TimeoutError): b.execute(hw.validate(args))
    assert bot.frames()[-3:]==[b'\x23\x20\x00',b'\x28',b'\x42']
    assert not any(p[0]==0x23 and p[2]!=0 for p in bot.frames())


def test_continuous_speed_is_timed_heartbeat_independent_and_interruption_stops(bot,monkeypatch):
    args=parse('servo speed S5 -25 --seconds .09',bot)
    with bench(args) as b:
        b.execute(hw.validate(args))
        assert bot.frames()[-2:]==[b'\x23\x20\x00',b'\x28']
        times=[t for t,p in bot.packets if p[0]==0x43]
        assert len(times)>=3 and max(y-x for x,y in zip(times,times[1:]))<.05
        assert b'\x23\x20\xe7' in bot.frames()
        def interrupted(seconds): raise KeyboardInterrupt
        monkeypatch.setattr(b,'wait',interrupted)
        with pytest.raises(KeyboardInterrupt): b.execute(hw.validate(args))
        assert bot.frames()[-2:]==[b'\x23\x20\x00',b'\x28']
    assert bot.frames()[-3:]==[b'\x23\x20\x00',b'\x28',b'\x42']


def test_multiple_continuous_cleanup_and_retyping_positionals(bot):
    args=parse('servo type S0 continuous',bot)
    with bench(args) as b:
        b.execute(hw.validate(args))
        b.execute(hw.validate(parse('servo type S4 continuous',bot)))
        b.execute(hw.validate(parse('servo type S0 180 --home 90',bot)))
        assert b.continuous=={4}
        bot.drop.add(0x23)
    assert bot.frames()[-4:]==[b'\x23\x10\x00',b'\x28',b'\x24\x01\x00\x5a',b'\x42']
    assert b.cleanup_errors


@pytest.mark.parametrize('command,frame',[
    ('motor speed M4 -20 --seconds .001',b'\x21\x08\xec'),
    ('led set L0 L4 --rgb 10 20 30 --brightness 64',b'\x51\x11\x0a\x14\x1e\x40'),
    ('led off L2',b'\x52\x04'),('board-led set 2 --rgb 1 2 3 --brightness 4',b'\x31\x02\x01\x02\x03\x04'),
    ('board-led off 1',b'\x32\x01'),('screen app-info',b'\x63\x01'),('screen next',b'\x61'),
    ('sensor imu configure --range-code 1 --rate-hz 100 --power-code 0 --hpf-code 1',b'\xa4\x01\x00\x64\x00\x01'),
    ('sensor imu reset',b'\xa6'),('sensor bump configure --threshold-mg 500 --debounce-ms 100',b'\xa7\x01\xf4\x00\x64'),
    ('sensor camera set-algorithm color',b'\x74\x04'),('sensor camera illumination off',b'\x71\x00'),
])
def test_output_and_configuration_frames(bot,command,frame):
    args=parse(command,bot)
    with bench(args) as b: b.execute(hw.validate(args))
    assert bot.frames()[0]==b'\x41test!' and frame in bot.frames()


@pytest.mark.parametrize('command,key',[
    ('ping','pong'),('name','loopback'),('battery','battery percent'),('servo read S0 S5','DEG (commanded)'),
    ('motor read M1 M4','% (commanded)'),('led read L0 L4','BRIGHTNESS'),('board-led read','no board LED readings'),
    ('sensor imu read','X mg'),('sensor imu config','rate hz'),('sensor bump config','threshold mg'),
    ('sensor camera algorithm','learned id count'),('sensor camera controls','illumination supported'),
    ('sensor lidar read','8x64 mm'),
])
def test_read_only_queries_register_first_without_configuring_servos(bot,command,key,capsys):
    args=parse(command,bot)
    with bench(args) as b: b.execute(hw.validate(args))
    text=capsys.readouterr().out
    assert key in text and not text.lstrip().startswith('{')
    assert bot.frames()[0]==b'\x41test!'
    assert bot.frames()[-2:]==[b'\x28',b'\x42']
    assert not any(p[0] in (0x22,0x23,0x24) for p in bot.frames())


@pytest.mark.parametrize('name',['lidar','intensity','imu','compass','camera','bump','all'])
def test_streams_separate_events_from_ack_and_disable_on_exit(bot,name,capsys):
    bot.bump=True
    args=parse(f'sensor {name} watch --hz 8 --seconds .06',bot)
    with bench(args) as b: b.execute(hw.validate(args))
    text=capsys.readouterr().out
    expected={'lidar':'8x64 mm','intensity':'8x64 raw','imu':'X mg','compass':'HEADING deg',
              'camera':'320x240 px','bump':'impact 1 X+'}
    for key in expected if name=='all' else [name]:
        assert expected[key] in text
    assert '\x1b' not in text
    enables=[p for p in bot.frames() if p[0] in (0x83,0x89,0xA2,0x93,0x72) and p[1]]
    assert len(enables)==5 if name=='all' else len(enables)==1
    for enable in enables: assert enable[:1]+b'\x00'+enable[2:] in bot.frames()
    assert not b.cleanup_errors


def test_compass_rate_format_explicit_and_no_automatic_fallback(bot,capsys):
    args=parse('sensor compass read --hz 10',bot)
    assert stream_frame('compass',10,'u16be')==b'\x93\x01\x00\x0a'
    assert stream_frame('compass',10,'u8')==b'\x93\x01\x0a'
    bot.reject.add(0x93)
    with bench(args) as b:
        with pytest.raises(RuntimeError,match='rejected'): b.execute(hw.validate(args))
    assert [p for p in bot.frames() if p[0]==0x93]==[b'\x93\x01\x00\x0a']
    assert 'unsupported' in capsys.readouterr().out


def test_bump_without_impact_is_not_transport_failure(bot,capsys):
    args=parse('sensor bump watch --seconds .04',bot)
    with bench(args) as b:
        b.execute(hw.validate(args))
        assert not b.transport.fault.is_set()
    text=capsys.readouterr().out
    assert 'NO EVENTS' in text and 'events received' in text


@pytest.mark.parametrize('command',[
    'servo angle S6 90 --type 180 --home 90','servo angle S0 181 --type 180 --home 90','servo angle S0 20',
    'servo angle S0 20 --type 180 --home 90 --min-angle 30','servo speed S0 101',
    'motor speed M0 10','motor speed M5 10','motor speed M1 1 --seconds nan',
    'motor speed M1 1 --seconds 0','led set L5 --rgb 0 0 0',
    'sensor lidar watch --hz 9','sound play ../bad.wav',
])
def test_invalid_values_fail_before_sockets(command,monkeypatch):
    monkeypatch.setattr(socket,'socket',lambda *a,**k:pytest.fail('Invalid input opened a socket'))
    with pytest.raises((ValueError,SystemExit)):
        hw.run(parse(command))


def test_dry_run_never_prompts_or_connects(monkeypatch,capsys):
    monkeypatch.setattr(socket,'socket',lambda *a,**k:pytest.fail('Dry run opened socket'))
    monkeypatch.setattr(hw,'private_token',lambda:pytest.fail('Dry run prompted for token'))
    args=parse('servo angle S3 90 --type 270 --home 90',dry=True)
    hw.run(args)
    text=capsys.readouterr().out
    assert 'DRY RUN: SERVO ANGLE' in text and 'request.home' in text and '90' in text


def test_capabilities_is_offline_and_omits_unavailable_inventory(monkeypatch,capsys):
    monkeypatch.setattr(socket,'socket',lambda *a,**k:pytest.fail('Capabilities opened a socket'))
    monkeypatch.setattr(hw,'private_token',lambda:pytest.fail('Capabilities prompted for token'))
    hw.run(parse('capabilities'))
    text=capsys.readouterr().out.lower()
    for feature in ('servo','motor','battery','lidar','compass','imu','huskylens','leds','display','sound'):
        assert feature in text
    for item in ('unavailable','unsupported','temperature','humidity','microphone','ultrasonic',
                 'geomag discrepancy','protocol gaps','dry-run','shell'):
        assert item not in text


def test_private_token_nonterminal_rejected_without_exposure(monkeypatch):
    monkeypatch.setattr('sys.stdin.isatty',lambda:False)
    with pytest.raises(ValueError,match='terminal'): hw.private_token()


def test_readback_wrong_echoed_mask_poisons_session(bot,monkeypatch):
    args=parse('servo read S0',bot)
    with bench(args) as b:
        b.connect()
        monkeypatch.setattr(b.transport,'request',lambda *a,**k:b'\x27\x00\x02\x00\x5a')
        with pytest.raises(ValueError,match='mask'): b.execute(hw.validate(args))
        assert b.transport.fault.is_set()


@pytest.fixture
def http_bot():
    state={'sounds':['beep.wav'],'playback_state':'idle','playing':False,'free_bytes':1000000}
    calls=[]
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def do_GET(self):
            calls.append(('GET',self.path,b''))
            data=json.dumps(state).encode(); self.send_response(200); self.end_headers(); self.wfile.write(data)
        def do_POST(self):
            data=self.rfile.read(int(self.headers.get('Content-Length','0')))
            calls.append(('POST',self.path,data))
            if self.path=='/sounds/stop': state.update(playback_state='idle',playing=False)
            self.send_response(200); self.end_headers(); self.wfile.write(b'OK')
        def do_DELETE(self):
            calls.append(('DELETE',self.path,b'')); self.send_response(200); self.end_headers()
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    thread=threading.Thread(target=server.serve_forever,daemon=True); thread.start()
    try: yield server.server_address[1],state,calls
    finally: server.shutdown(); server.server_close(); thread.join(timeout=1)


def test_sound_http_operations_register_first(http_bot,bot,monkeypatch,tmp_path):
    port,state,calls=http_bot
    monkeypatch.setattr(hw,'private_token',lambda:'test!')
    path=tmp_path/'beep.wav'
    with wave.open(str(path),'wb') as wav:
        wav.setparams((1,1,11025,0,'NONE','not compressed')); wav.writeframes(bytes([128])*100)
    for command in ('sound list','sound status','sound play beep.wav','sound volume 25','sound stop',
                    f'sound upload new.wav {path}','sound delete beep.wav'):
        args=parse(command,bot); args.http_port=port
        hw.run(args)
    assert ('POST','/sounds/beep.wav/play',b'') in calls
    assert ('POST','/sounds/volume?value=25',b'') in calls
    assert ('DELETE','/sounds/beep.wav',b'') in calls
    uploaded=[data for method,path,data in calls if path=='/sounds/new.wav']
    assert uploaded==[path.read_bytes()]
    state.update(playback_state='playing',playing=True)
    args=parse(f'sound upload new.wav {path}',bot); args.http_port=port
    with pytest.raises(ValueError,match='inactive'): hw.run(args)
    assert sum(p[0]==0x41 for p in bot.frames())==8
    assert bot.frames()[-1]==b'\x42'


def test_timed_motion_stops_before_output_can_block(bot,monkeypatch):
    args=parse('servo speed S2 10 --seconds .001',bot)
    def blocked_output(value):
        assert bot.frames()[-2:]==[b'\x23\x04\x00',b'\x28']
        raise BrokenPipeError('closed output')
    monkeypatch.setattr(hw,'output',blocked_output)
    with bench(args) as b:
        with pytest.raises(BrokenPipeError): b.execute(hw.validate(args))
    assert bot.frames()[-3:]==[b'\x23\x04\x00',b'\x28',b'\x42']


def test_no_sensor_frames_times_out_and_disables_stream(bot,monkeypatch):
    monkeypatch.setattr(bot,'emit',lambda:None)
    args=parse('sensor camera read --seconds .04',bot)
    with bench(args) as b:
        with pytest.raises(TimeoutError,match='fresh'): b.execute(hw.validate(args))
        assert b.transport.fault.is_set()
    assert b'\x72\x00\x08' in bot.frames() and bot.frames()[-1]==b'\x42'


def test_watch_output_failure_still_disables_streams_and_unregisters(bot,monkeypatch):
    from botticelli.display import Watch
    def broken_output(*args):
        raise BrokenPipeError('closed output')
    monkeypatch.setattr(Watch,'update',broken_output)
    args=parse('sensor all watch --seconds .05',bot)
    with bench(args) as b:
        with pytest.raises(BrokenPipeError): b.execute(hw.validate(args))
    assert not bot.active
    assert bot.frames()[-2:]==[b'\x28',b'\x42']


def test_shell_prompts_once_and_recovers_from_invalid_command(bot,monkeypatch,capsys):
    lines=iter(['servo angle S0 999 --type 180 --home 90','servo type S0 continuous',
                'servo angle S1 90 --type 180 --home 90 --seconds .001','exit'])
    def prompt(text):
        assert bot.frames()[0]==b'\x41test!'
        return next(lines)
    monkeypatch.setattr('builtins.input',prompt)
    monkeypatch.setattr('sys.stdin.isatty',lambda:True)
    prompts=[]
    monkeypatch.setattr(hw,'private_token',lambda:prompts.append(True) or 'test!')
    hw.run(parse('shell',bot))
    assert len(prompts)==1
    assert sum(p[0]==0x41 for p in bot.frames())==1
    assert b'\x24\x02\x00\x5a' in bot.frames()
    assert bot.frames()[-4:]==[b'\x23\x01\x00',b'\x28',b'\x24\x02\x00\x5a',b'\x42']
    captured=capsys.readouterr()
    assert 'test!' not in captured.out+captured.err


def test_shell_help_accepts_unix_flags_and_help_command(bot,monkeypatch,capsys):
    lines=iter(['help servo angle', 'sensor imu configure -h', 'servo speed --help', 'exit'])
    monkeypatch.setattr('builtins.input',lambda prompt:next(lines))
    monkeypatch.setattr('sys.stdin.isatty',lambda:True)
    monkeypatch.setattr(hw,'private_token',lambda:'test!')
    hw.run(parse('shell',bot))
    captured=capsys.readouterr()
    assert captured.out.count('usage:')==3
    assert captured.out.count('Example:')==3
    assert not captured.err
    assert bot.frames()==[b'\x41test!',b'\x28',b'\x42']
