from argparse import Namespace
import re
from types import SimpleNamespace

import pygame
import pytest

from botticelli.__main__ import drive, gamepad_check
from botticelli import simulation
from botticelli.control import Input
from test_runtime_recording import Backend
from botticelli.__main__ import build_parser


def command_paths(parser, prefix=()):
    """Visit every command level, including nested sensor operations."""
    import argparse
    yield prefix
    for action in parser._actions:
        if isinstance(action, argparse._SubParsersAction):
            for name, child in action.choices.items():
                yield from command_paths(child, prefix+(name,))


@pytest.mark.parametrize('path', list(command_paths(build_parser())), ids=lambda p:' '.join(p) or 'root')
@pytest.mark.parametrize('flag', ['-h', '--help'])
def test_every_command_has_offline_unix_help(path, flag, monkeypatch, capsys):
    import socket
    from botticelli import hardware
    monkeypatch.setattr(socket, 'socket', lambda *a, **k:pytest.fail('Help opened a socket'))
    monkeypatch.setattr(hardware, 'private_token', lambda:pytest.fail('Help prompted for token'))
    with pytest.raises(SystemExit) as result:
        build_parser().parse_args([*path, flag])
    assert result.value.code == 0
    captured = capsys.readouterr()
    assert not captured.err
    assert 'usage:' in captured.out and '-h, --help' in captured.out
    assert 'EXAMPLE:' in captured.out
    example = captured.out.split('EXAMPLE:', 1)[1]
    assert not re.search(r'\b\d+\b', example)
    assert '{' not in example and 'beep.wav' not in example
    if path[:2] == ('hardware', 'sensor') and len(path) >= 3 and path[2] != 'all':
        model = {'lidar':'SEN0682', 'imu':'LIS2DW12', 'compass':'BMM350', 'huskylens':'HuskyLens'}[path[2]]
        assert model in captured.out
    if path[:2] in (('hardware', 'servo'), ('hardware', 'dc-motor'), ('hardware', 'led')):
        assert 'DFR1216' in captured.out


def test_commands_group_views_under_their_physical_devices():
    paths = set(command_paths(build_parser()))
    assert {p[2] for p in paths if p[:2] == ('hardware', 'sensor') and len(p) == 3} == {
        'lidar', 'imu', 'compass', 'huskylens', 'all'}
    assert ('hardware', 'sensor', 'imu', 'bump', 'watch') in paths
    assert ('hardware', 'sensor', 'lidar', 'intensity', 'read') in paths
    assert ('hardware', 'led', 'board', 'read') in paths
    assert ('hardware', 'board-led') not in paths


def test_servo_help_explains_units_home_and_limits(capsys):
    with pytest.raises(SystemExit):
        build_parser().parse_args(['hardware', 'servo', 'positional', 'angle', '--help'])
    text = capsys.readouterr().out
    for term in ('degrees', '--home', '--min-angle', '--max-angle', 'EXAMPLE:'):
        assert term in text
    assert '(default: 270)' in text and '(default: 0)' in text
    assert text.count('(required)') == 2


def test_continuous_group_lists_speed_parameters_and_duration(capsys):
    with pytest.raises(SystemExit):
        build_parser().parse_args(['hardware', 'servo', 'continuous', '--help'])
    text = ' '.join(capsys.readouterr().out.split())
    assert 'COMMAND SYNOPSIS:' in text
    assert 'speed CHANNEL PERCENT [--seconds SECONDS]' in text
    assert 'configure CHANNEL' in text


def test_servo_help_includes_nested_parameters_without_hiding_seconds(capsys):
    with pytest.raises(SystemExit):
        build_parser().parse_args(['hardware', 'servo', '--help'])
    text = ' '.join(capsys.readouterr().out.split())
    assert 'continuous speed CHANNEL PERCENT [--seconds SECONDS]' in text
    assert 'positional angle CHANNEL ANGLE [--seconds SECONDS] [--type TYPE] [--home HOME]' in text
    assert '--min-angle MIN_ANGLE' in text and '--max-angle MAX_ANGLE' in text


def test_leaf_help_lists_connection_options_and_where_to_put_them(capsys):
    with pytest.raises(SystemExit):
        build_parser().parse_args(['hardware', 'servo', 'continuous', 'speed', '--help'])
    text = ' '.join(capsys.readouterr().out.split())
    assert 'hold speed for SECONDS, then stop' in text
    assert '(default: 1.0)' in text
    assert 'CONNECTION OPTIONS (before the component command)' in text
    for option in ('--host HOST', '--token TOKEN', '--port PORT', '--http-port HTTP_PORT', '--timeout TIMEOUT',
                   '--geomag-rate-format GEOMAG_RATE_FORMAT', '--dry-run', '--debug'):
        assert option in text


def test_generic_synopsis_keeps_required_options_and_multiple_operands():
    parser = build_parser()
    args = parser.parse_args(['hardware', '--dry-run', 'led', 'set', 'L0', '--rgb', '1', '2', '3'])
    assert '--rgb R G B' in args.example and '[--rgb' not in args.example
    assert 'L0' not in args.example and not re.search(r'\b\d+\b', args.example)
    args = parser.parse_args(['hardware', '--dry-run', 'sensor', 'imu', 'configure',
                              '--range-code', '1', '--rate-hz', '100', '--power-code', '0', '--hpf-code', '1'])
    assert '--range-code RANGE_CODE' in args.example and '[--range-code' not in args.example


def test_shell_help_uses_shell_syntax_without_connection_flags():
    from botticelli.hardware import Parser, add_components
    parser = Parser(prog='k10')
    add_components(parser, shell=False)
    args = parser.parse_args(['servo', 'continuous', 'speed', 's0', '100', '--seconds', '10'])
    assert args.example == 'servo continuous speed CHANNEL PERCENT [--seconds SECONDS]'
    assert '--host' not in args.example


@pytest.mark.parametrize('words, terms', [
    ('hardware servo positional angle', ('CHANNEL', 'ANGLE', 'required')),
    ('hardware servo positional angle S9 45', ('S9', 'S0..S5')),
    ('hardware servo continuous speed S0 150', ('PERCENT', '150', '-100..100')),
    ('hardware dc-motor speed S0 20', ('S0', 'M1..M4')),
    ('hardware led set L0', ('--rgb', 'required')),
])
def test_argument_errors_name_the_problem_and_show_an_example(words, terms, monkeypatch, capsys):
    import socket
    monkeypatch.setattr(socket, 'socket', lambda *a, **k:pytest.fail('Invalid command opened a socket'))
    with pytest.raises(SystemExit) as result:
        build_parser().parse_args(words.split())
    assert result.value.code == 2
    text = capsys.readouterr().err
    for term in (*terms, 'Example:', 'Help:'):
        assert term in text


def test_validation_error_names_the_servo_value_and_limits(monkeypatch, capsys):
    import socket
    import sys
    from botticelli.__main__ import main
    monkeypatch.setattr(socket, 'socket', lambda *a, **k:pytest.fail('Invalid command opened a socket'))
    monkeypatch.setattr(sys, 'argv', ['botticelli', 'hardware', '--dry-run', 'servo', 'positional',
                                    'angle', 'S0', '200', '--type', '180'])
    with pytest.raises(SystemExit) as result:
        main()
    assert result.value.code == 1
    text = capsys.readouterr().err
    for term in ('S0', '200 degrees', '0..180 degrees', 'Example:'):
        assert term in text


def test_drive_defaults_to_offline_simulation():
    assert build_parser().parse_args(['drive']).backend == 'sim'


def test_help_with_a_supplied_token_never_shows_its_value(capsys):
    with pytest.raises(SystemExit):
        build_parser().parse_args(['hardware', '--token', 'Q7z!9', '--help'])
    text = capsys.readouterr().out
    assert '--token TOKEN' in text and 'hidden prompt' in text
    assert 'Q7z!9' not in text


@pytest.mark.parametrize('flags', [['--token', 'Q7z!9'], ['--token=Q7z!9']])
def test_misplaced_token_is_redacted_in_argument_errors(flags, capsys):
    with pytest.raises(SystemExit) as result:
        build_parser().parse_args(['hardware', 'ping', *flags])
    assert result.value.code == 2
    text = capsys.readouterr().err
    assert 'Q7z!9' not in text and '<redacted>' in text


def test_real_drive_consumes_supplied_token_without_prompting(monkeypatch):
    from botticelli import control
    from botticelli import __main__ as cli
    from botticelli import config
    monkeypatch.setattr(config, 'load', lambda path:SimpleNamespace(validate=lambda **kwargs:None))
    monkeypatch.setattr(cli, 'private_token', lambda:pytest.fail('Supplied token prompted'))
    tokens = []
    def backend(config, token, *, debug=False):
        tokens.append((token, debug))
        raise RuntimeError('backend reached')
    monkeypatch.setattr(control, 'K10', backend)
    args = build_parser().parse_args(['drive', '--backend', 'real', '--token', 'Q7z!9', '--debug'])
    with pytest.raises(RuntimeError, match='backend reached'):
        drive(args)
    assert tokens == [('Q7z!9', True)] and args.token is None


def test_token_never_reaches_drive_output_or_recording(tmp_path):
    from pathlib import Path
    import subprocess
    import sys
    path = tmp_path / 'session.jsonl'
    result = subprocess.run([sys.executable, '-m', 'botticelli', 'drive', '--token', 'Q7z!9',
        '--headless', '--seconds', '.1', '--record', str(path)],
        cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert 'Q7z!9' not in result.stdout + result.stderr + path.read_text()


@pytest.mark.parametrize("interrupted", [False, True])
def test_window_close_and_interrupt_always_close_backend(monkeypatch, interrupted):
    backend = Backend()
    monkeypatch.setattr(simulation, "Simulation", lambda config: backend)
    def events():
        if interrupted:
            raise KeyboardInterrupt
        return [SimpleNamespace(type=pygame.QUIT)]
    monkeypatch.setattr(pygame.event, "get", events)
    args = Namespace(config="config/bot.yaml", backend="sim", headless=False, seconds=1,
                     screenshot=None, record=None)
    if interrupted:
        with pytest.raises(KeyboardInterrupt):
            drive(args)
    else:
        drive(args)
    assert backend.closed
    assert backend.calls[-1][1].throttle == 0


def test_hardware_cli_formats_dry_run():
    from pathlib import Path
    import subprocess
    import sys
    result = subprocess.run([sys.executable, '-m', 'botticelli', 'hardware', '--dry-run',
        'servo', 'positional', 'angle', 'S0', '45', '--type', '180', '--home', '90'],
        cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert 'DRY RUN: SERVO POSITIONAL ANGLE' in result.stdout and 'request.home' in result.stdout
