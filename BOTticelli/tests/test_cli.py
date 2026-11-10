from argparse import Namespace
from types import SimpleNamespace

import pygame
import pytest

from botticelli.__main__ import drive, gamepad_check
from botticelli import simulation
from botticelli.control import Input
from test_runtime_recording import Backend


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
