from argparse import Namespace
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
    if path:
        assert 'Example:' in captured.out
    if path[:2] == ('hardware', 'sensor') and len(path) >= 3 and path[2] != 'all':
        model = {'lidar':'SEN0682', 'imu':'LIS2DW12', 'compass':'BMM350', 'huskylens':'HuskyLens'}[path[2]]
        assert model in captured.out
    if path[:2] in (('hardware', 'servo'), ('hardware', 'motor'), ('hardware', 'led')):
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
        build_parser().parse_args(['hardware', 'servo', 'angle', '--help'])
    text = capsys.readouterr().out
    for term in ('degrees', '--home', '--min-angle', '--max-angle', 'Example:'):
        assert term in text


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
        'servo', 'angle', 'S0', '45', '--type', '180', '--home', '90'],
        cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert 'DRY RUN: SERVO ANGLE' in result.stdout and 'request.home' in result.stdout
