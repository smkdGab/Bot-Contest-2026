"""Public primitive API and process failure tests; localhost hardware only."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from botticelli.primitives import Robot
from test_hardware import bot, http_bot


def robot(bot, **kwargs):
    return Robot('127.0.0.1', 'test!', port=bot.address[1], **kwargs)


def test_primitives_import_without_site_packages():
    result = subprocess.run([sys.executable, '-S', '-c',
        'from botticelli.primitives import Robot, private_token, termination_cleanup'],
        capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('failure', [None, ValueError, KeyboardInterrupt, SystemExit])
def test_context_homes_only_selected_channels_after_stopping(bot, failure):
    client = robot(bot)
    def run():
        with client:
            client.configure_servo(0, 'continuous')
            client.configure_servo(4, 180, home=75, minimum=60, maximum=120)
            client.servo_speed(0, 15)
            client.servo_angle(4, 100)
            if failure:
                raise failure()
    if failure:
        with pytest.raises(failure): run()
    else:
        run()
    assert bot.frames()[-4:] == [b'\x23\x01\x00', b'\x28', b'\x24\x10\x00\x4b', b'\x42']
    assert all(p[1] in (1, 16) for p in bot.frames() if p[0] in (0x22, 0x23, 0x24))
    assert client._token == '' and client.transport is None
    before = list(bot.frames())
    client.close()
    assert bot.frames() == before


def test_session_registers_only_once_and_cannot_be_reopened(bot):
    client = robot(bot)
    with client:
        client.connect()
        assert client.ping()['pong']
        assert client.name()['name'] == 'loopback'
        assert client.battery()['battery_percent'] == 85
    assert sum(p[0] == 0x41 for p in bot.frames()) == 1
    with pytest.raises(RuntimeError): client.connect()


def test_documented_stream_state_and_lidar_reset(bot):
    with robot(bot) as client:
        assert client.imu_stream_state() == dict(enabled=True, hz=100)
        assert client.camera_stream_state() == dict(enabled=True, hz=8)
        assert client.reset_lidar() == {'ok':True}


@pytest.mark.parametrize('token', ['', '1234', '123456', '\x00abcd', None])
def test_invalid_token_opens_no_socket(monkeypatch, token):
    import socket
    monkeypatch.setattr(socket, 'socket', lambda *a, **k:pytest.fail('Invalid token opened a socket'))
    with pytest.raises(ValueError):
        Robot('127.0.0.1', token).connect()


def test_uncertain_positional_type_never_sends_an_unverified_home(bot):
    bot.drop.add(0x22)
    with robot(bot, timeout=.05) as client:
        with pytest.raises(TimeoutError): client.configure_servo(2, 180, home=90)
        assert not client.homes
    assert not any(p[0] == 0x24 for p in bot.frames())
    assert bot.frames()[-2:] == [b'\x28', b'\x42']


def test_cleanup_failure_is_reported_by_context(bot):
    bot.reject.add(0x28)
    with pytest.raises(RuntimeError, match='Cleanup not confirmed'):
        with robot(bot):
            pass
    assert bot.frames()[-1] == b'\x42'


def test_later_confirmed_stop_clears_only_the_resolved_cleanup_error(bot):
    client = robot(bot, timeout=.05)
    client.connect()
    bot.drop.add(0x28)
    with pytest.raises(RuntimeError, match='stop DC motors'):
        client.stop()
    assert len(client.cleanup_errors) == 1
    bot.drop.clear()
    client.close()
    assert not client.cleanup_errors
    assert bot.frames()[-2:] == [b'\x28', b'\x42']


def test_cleanup_rejection_names_the_servo_and_firmware_status(bot):
    client = robot(bot)
    client.connect()
    client.configure_servo(4, 270, home=0)
    bot.reject.add(0x24)
    client.close()
    assert len(client.cleanup_errors) == 1
    for term in ('home positional servo S4 to 0 degrees', '0x24', 'operation failed', 'status 0x03'):
        assert term in client.cleanup_errors[0]


def test_missing_cleanup_ack_stays_unconfirmed_with_action_and_timeout(bot):
    client = robot(bot, timeout=.05)
    client.connect()
    bot.drop.add(0x42)
    client.close()
    assert len(client.cleanup_errors) == 1
    for term in ('unregister', '0x42', 'no reply', '0.05s'):
        assert term in client.cleanup_errors[0]


def test_cleanup_ack_without_diagnostics_is_reported_as_malformed(bot, monkeypatch):
    send = bot.send
    def reply(body, target):
        if body[0] == 0x42:
            bot.sock.sendto(body, target)
        else:
            send(body, target)
    monkeypatch.setattr(bot, 'send', reply)
    client = robot(bot)
    client.connect()
    client.close()
    assert len(client.cleanup_errors) == 1
    for term in ('unregister', 'received 2 bytes', 'plus 8 diagnostics bytes'):
        assert term in client.cleanup_errors[0]


def test_delayed_cleanup_ack_is_confirmed_without_resending(bot, monkeypatch):
    send = bot.send
    def reply(body, target):
        if body[0] in (0x28, 0x42):
            time.sleep(.02)
        send(body, target)
    monkeypatch.setattr(bot, 'send', reply)
    with robot(bot) as client:
        pass
    assert not client.cleanup_errors
    assert bot.frames() == [b'\x41test!', b'\x28', b'\x42']


def test_unregistered_reads_never_open_a_socket(monkeypatch):
    import socket
    client = Robot('127.0.0.1', 'test!')
    monkeypatch.setattr(socket, 'socket', lambda *a, **k:pytest.fail('Unregistered read opened a socket'))
    for read in (client.read_imu, client.read_camera, client.imu_stream_state,
                 lambda:client.latest('imu'), client.sound_status):
        with pytest.raises(RuntimeError, match='session'): read()


@pytest.mark.parametrize('method,args,kwargs', [
    ('configure_servo', (6, 180), {'home':90}),
    ('configure_servo', (0, 180), {}),
    ('configure_servo', (0, 180), {'home':0, 'minimum':60}),
    ('configure_servo', (0, 'continuous'), {'home':90}),
    ('servo_angle', (0, 45), {}),
    ('servo_speed', (0, 25), {}),
    ('motor_speed', (0, 25), {}),
    ('motor_speed', (1, 101), {}),
    ('led_set', ([5], 0, 0, 0), {}),
    ('board_led_set', (3, 0, 0, 0), {}),
    ('stream', ('lidar',), {'hz':9}),
    ('stream', ('camera',), {'enabled':3}),
    ('set_camera_algorithm', ('invented',), {}),
])
def test_invalid_primitive_never_sends_command(bot, method, args, kwargs):
    with robot(bot) as client:
        before = list(bot.frames())
        with pytest.raises(ValueError): getattr(client, method)(*args, **kwargs)
        assert bot.frames() == before


def test_angle_and_speed_validate_safe_range_and_lease(bot):
    with robot(bot) as client:
        client.configure_servo(1, 270, home=90, minimum=60, maximum=120)
        client.configure_servo(5, 'continuous')
        before = list(bot.frames())
        for method, args, kwargs in [
            ('servo_angle', (1, 121), {}), ('servo_speed', (5, 101), {}),
            ('servo_angle', (1, 100), {'ttl':float('nan')}),
            ('servo_speed', (5, 20), {'ttl':0}),
        ]:
            with pytest.raises(ValueError): getattr(client, method)(*args, **kwargs)
        assert bot.frames() == before


@pytest.mark.parametrize('name', ['compass', 'camera', 'intensity', 'bump'])
def test_sensor_read_waits_for_new_frame_and_disables_only_its_stream(bot, name):
    bot.bump = True
    with robot(bot) as client:
        reading = getattr(client, 'read_'+name)(timeout=.3)
        assert reading.received_at > 0
        assert not client.streams
        client.stream(name)
        frame = next(iter(client.streams.values()))
        assert getattr(client, 'read_'+name)(timeout=.3) is not None
        assert frame[0] in client.streams


def test_sensor_timeout_and_no_bump_are_distinct(bot, monkeypatch):
    monkeypatch.setattr(bot, 'emit', lambda:None)
    with robot(bot) as client:
        assert client.read_bump(timeout=.03) is None
        assert not client.transport.fault.is_set()
        with pytest.raises(TimeoutError): client.read_camera(timeout=.03)
        assert client.transport.fault.is_set()
        assert not client.streams


def test_cleanup_attempts_every_home_stream_and_unregister_after_failures(bot):
    client = robot(bot)
    client.connect()
    client.configure_servo(0, 'continuous')
    client.configure_servo(2, 180, home=80)
    client.configure_servo(5, 270, home=100)
    client.stream('imu')
    bot.reject.update((0x23, 0x24))
    client.close()
    assert bot.frames()[-6:] == [b'\x23\x01\x00', b'\x28', b'\x24\x04\x00\x50',
                               b'\x24\x20\x00\x64', b'\xa2\x00\x00\x08', b'\x42']
    assert len(client.cleanup_errors) == 3


def test_watchdog_stops_and_homes_when_control_progress_stalls(bot):
    with robot(bot) as client:
        client.configure_servo(0, 'continuous')
        client.configure_servo(3, 180, home=70)
        client.servo_angle(3, 120)
        client.servo_speed(0, 20, ttl=.04)
        deadline = time.monotonic()+.5
        while b'\x24\x08\x00\x46' not in bot.frames() and time.monotonic() < deadline:
            time.sleep(.005)
        assert client.transport.fault.is_set()
        assert bot.frames()[-3:] == [b'\x23\x01\x00', b'\x28', b'\x24\x08\x00\x46']
        assert not client.transport.heartbeat.is_alive()
        with pytest.raises(RuntimeError): client.servo_speed(0, 20)
        assert sum(p == b'\x23\x01\x14' for p in bot.frames()) == 1


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_process_signal_runs_real_session_cleanup(bot, signum):
    script = '''
import os, signal
from botticelli.primitives import Robot, termination_cleanup
class CheckedRobot(Robot):
    def close(self):
        # A second signal during shutdown must not interrupt the remaining steps.
        assert signal.getsignal(signal.SIGTERM) == signal.SIG_IGN
        os.kill(os.getpid(), signal.SIGTERM)
        os.kill(os.getpid(), signal.SIGINT)
        super().close()
with termination_cleanup():
    with CheckedRobot('127.0.0.1', 'test!', port=PORT) as bot:
        bot.configure_servo(2, 180, home=80)
        bot.configure_servo(5, 'continuous')
        bot.servo_speed(5, 10, ttl=5)
        print('ready', flush=True)
        signal.pause()
'''.replace('PORT', str(bot.address[1]))
    process = subprocess.Popen([sys.executable, '-S', '-c', script], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               cwd=Path(__file__).resolve().parents[1], text=True)
    try:
        # Bound readiness as well as shutdown; a child failure must not hang pytest.
        import selectors
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            assert selector.select(timeout=5)
        assert process.stdout.readline().strip() == 'ready'
        os.kill(process.pid, signum)
        process.communicate(timeout=5)
        assert process.returncode != 0
        assert bot.frames()[-4:] == [b'\x23\x20\x00', b'\x28', b'\x24\x04\x00\x50', b'\x42']
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=5)


def test_signal_arriving_during_ordinary_cleanup_does_not_abort_it(bot):
    script = '''
import os, signal
from botticelli.primitives import Robot
with Robot('127.0.0.1', 'test!', port=PORT) as bot:
    bot.configure_servo(2, 180, home=80)
    request = bot.transport.request
    def signal_during_stop(frame, **kwargs):
        if frame == b'\\x28':
            os.kill(os.getpid(), signal.SIGTERM)
            os.kill(os.getpid(), signal.SIGINT)
        return request(frame, **kwargs)
    bot.transport.request = signal_during_stop
'''.replace('PORT', str(bot.address[1]))
    process = subprocess.run([sys.executable, '-S', '-c', script], capture_output=True, text=True, timeout=5,
                             cwd=Path(__file__).resolve().parents[1])
    assert process.returncode == 0, process.stderr
    assert bot.frames()[-3:] == [b'\x28', b'\x24\x04\x00\x50', b'\x42']
