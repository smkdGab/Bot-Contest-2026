"""Wire diagnostics use loopback only, including slow or broken trace output."""
import io
import socket
import struct
import threading
import time

import pytest

from botticelli import hardware as hw
from botticelli.primitives import Robot
from botticelli.protocol import DebugTrace, K10Transport
from test_hardware import bench, bot, http_bot, parse


def test_debug_servo_hold_and_lidar_share_one_registered_connection(bot, capsys):
    args = parse('--debug servo continuous speed S0 100 --seconds .1', bot, token='Q7z!9')
    with bench(args) as client:
        client.execute(args)
        client.execute(parse('sensor lidar watch --seconds .06', bot))
    captured = capsys.readouterr()
    trace = captured.err
    for term in ('OPEN', 'SESSION', 'WAIT', 'TX', 'RX', 'MATCH', 'MOTION', 'HOLD',
                 'HEARTBEAT', 'READ', 'CLEANUP', 'CLOSE', 'rx_seq=', 'server_ms=',
                 'hex=41 <redacted>', 'hex=22 01 02', 'hex=23 01 64', 'hex=43',
                 'hex=83 01 08 00', 'hex=8f 00', 'frame=', 'values=(123,',
                 'end elapsed=', 'zero continuous servo S0'):
        assert term in trace
    assert 'Q7z!9' not in trace + captured.out
    assert b'Q7z!9'.hex(' ') not in trace
    assert sum(p[0] == 0x41 for p in bot.frames()) == 1
    moving = [t for t, p in bot.packets if p == b'\x23\x01\x64']
    stopped = [t for t, p in bot.packets if p == b'\x23\x01\x00' and t > moving[0]]
    assert len(moving) == 1 and stopped[0] - moving[0] >= .1
    assert not client.cleanup_errors


@pytest.mark.parametrize('name,expected', [
    ('lidar', 'values=(123,'), ('intensity', 'values=(123,'),
    ('imu', 'accel_mg=[1, -2, 1000]'), ('bump', 'direction=1'),
    ('compass', 'heading_deg=270'), ('camera', 'blocks=[] arrows=[]'),
])
def test_debug_streams_show_decoded_readings_and_diagnostics(bot, capsys, name, expected):
    bot.bump = True
    with Robot(bot.address[0], 'test!', port=bot.address[1], debug=True) as robot:
        robot.read_sensor(name, timeout=.2)
    trace = capsys.readouterr().err
    assert 'READ' in trace and expected in trace
    assert 'rx_seq=' in trace and 'server_ms=123456' in trace


def test_debug_query_and_http_show_bytes_and_decoded_values(bot, http_bot, capsys):
    port, state, calls = http_bot
    with Robot(bot.address[0], 'test!', port=bot.address[1], http_port=port, debug=True) as robot:
        robot.ping()
        robot.read_lidar()
        robot.sound_status()
        robot.sound_volume(25)
    trace = capsys.readouterr().err
    for term in ('PING (no status byte)', 'DATA', 'distance_mm=(123,', 'HTTP-TX',
                 'GET http://127.0.0.1:', '/sounds/volume?value=25', 'HTTP-RX',
                 'status=200', 'hex=4f 4b'):
        assert term in trace


def test_debug_explains_missing_lidar_diagnostics_without_accepting_bad_frames(bot, monkeypatch, capsys):
    send = bot.send
    def missing_trailer(body, target):
        if body[0] == 0x8F:
            bot.sock.sendto(body, target)
        else:
            send(body, target)
    monkeypatch.setattr(bot, 'send', missing_trailer)
    args = parse('--debug sensor lidar watch --seconds .06', bot)
    with bench(args) as client:
        with pytest.raises(TimeoutError, match='last rejected event:.*wire bytes=1034'):
            client.execute(args)
        assert client.transport.lidar is None
    trace = capsys.readouterr().err
    assert 'DROP' in trace and 'expected 1034 (1042 UDP bytes including diagnostics)' in trace
    assert 'invalid_events=' in trace


@pytest.mark.parametrize('failure,terms', [
    ('wrong', ('unexpected sender=', 'UNMATCHED', 'command outcome uncertain')),
    ('malformed', ('DROP', 'Malformed reply', 'expected 2 payload bytes plus 8 diagnostics')),
    ('reject', ('MATCH', 'status=0x03', 'device unsupported')),
])
def test_debug_explains_failed_commands(bot, capsys, failure, terms):
    getattr(bot, failure).add(0x22)
    transport = K10Transport(*bot.address, timeout=.08, debug=True)
    try:
        with pytest.raises((OSError, RuntimeError)):
            transport.request(b'\x22\x01\x02')
    finally:
        transport.close()
    trace = capsys.readouterr().err
    for term in ('FAIL', *terms):
        assert term in trace


def test_debug_reports_duplicate_frames(bot, capsys):
    transport = K10Transport(*bot.address, debug=True)
    try:
        packet = b'\xaf\0' + struct.pack('<Ihhh', 8, 1, 2, 3) + bytes(8)
        bot.sock.sendto(packet, transport.sock.getsockname())
        bot.sock.sendto(packet, transport.sock.getsockname())
        deadline = time.monotonic() + .2
        while transport.received < 2 and time.monotonic() < deadline:
            transport.done.wait(.001)
    finally:
        transport.close()
    assert 'imu stale/duplicate frame=8 previous=8' in capsys.readouterr().err


def test_debug_redacts_registration_echo_and_wifi_before_queueing():
    stream = io.StringIO()
    trace = DebugTrace(stream)
    for action in (0x41, 0x47, 0x48):
        trace('RX', packet=bytes([action]) + b'Q7z!9')
    trace.close()
    text = stream.getvalue()
    assert text.count('<redacted>') == 3
    assert 'Q7z!9' not in text and b'Q7z!9'.hex(' ') not in text


def test_blocked_trace_never_blocks_heartbeats_or_cleanup(bot, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    class SlowOutput(io.StringIO):
        def write(self, text):
            entered.set()
            release.wait(3)
            return super().write(text)
    stream = SlowOutput()
    monkeypatch.setattr('sys.stderr', stream)
    robot = Robot(bot.address[0], 'test!', port=bot.address[1], debug=True)
    trace = None
    try:
        robot.connect()
        trace = robot.transport.trace
        assert entered.wait(.2)
        for _ in range(300):
            robot.transport.log('TEST', 'slow output')
        robot.transport.done.wait(.12)
        assert not robot.transport.fault.is_set(), robot.transport.fault_reason
        assert sum(p == b'\x43' for _, p in bot.packets) >= 4
        assert robot.transport.trace.lines.qsize() <= 256
        started = time.monotonic()
        robot.close()
        assert time.monotonic() - started < 1
        assert not robot.cleanup_errors and bot.frames()[-1] == b'\x42'
    finally:
        release.set()
        robot.close()
        if trace is not None:
            trace.thread.join(timeout=1)
    assert 'trace lines: output is too slow' in stream.getvalue()


def test_broken_trace_output_does_not_break_cleanup(bot, monkeypatch):
    class BrokenOutput:
        def write(self, text):
            raise BrokenPipeError('closed trace pipe')
    monkeypatch.setattr('sys.stderr', BrokenOutput())
    with Robot(bot.address[0], 'test!', port=bot.address[1], debug=True) as robot:
        robot.ping()
    assert not robot.cleanup_errors and bot.frames()[-1] == b'\x42'


def test_debug_off_has_no_trace_thread_or_output(bot, capsys):
    with Robot(bot.address[0], 'test!', port=bot.address[1]) as robot:
        assert robot.transport.trace is None
        robot.ping()
    assert not capsys.readouterr().err


def test_debug_dry_run_is_offline_and_hides_token(monkeypatch, capsys):
    monkeypatch.setattr(socket, 'socket', lambda *a, **k:pytest.fail('Dry run opened a socket'))
    args = parse('--debug servo continuous speed S0 100 --seconds 10', dry=True, token='Q7z!9')
    hw.run(args)
    captured = capsys.readouterr()
    assert not captured.err and 'Q7z!9' not in captured.out
    assert 'request.debug' not in captured.out and 'request.token' not in captured.out
