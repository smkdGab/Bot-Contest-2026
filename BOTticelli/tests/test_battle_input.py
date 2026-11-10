"""Keyboard play, source switching, and focus rearming through battle sessions."""
from dataclasses import replace
from types import SimpleNamespace
import time

import pygame as pg
import pytest

from botticelli.battle_config import generate
from botticelli.battle_input import BattleInput
from botticelli.battle_runtime import BattleSession
from botticelli.control import Input


class Gamepad:
    message = 'Test gamepad'

    def __init__(self, **kwargs):
        self.sample = Input(0, **kwargs)

    def poll(self, events):
        return replace(self.sample, timestamp=time.monotonic())


def key(code, *, up=False, repeat=False):
    return SimpleNamespace(type=pg.KEYUP if up else pg.KEYDOWN, key=code, repeat=repeat)


def session_step(session, controls, events=(), **kwargs):
    sample = controls.poll(events)
    session.update(sample, sample.timestamp, **kwargs)
    return sample


def test_keyboard_drives_without_gamepad_and_release_stops_only_player():
    session = BattleSession(generate(seed=42, opponents=1))
    controls = BattleInput(Gamepad())
    session_step(session, controls, [key(pg.K_RETURN)], resume=True)
    assert not session.paused
    start = list(session.world.cars[0].pose)
    session_step(session, controls, [key(pg.K_w)])
    assert session.world.commands[0].throttle == 1
    assert session.world.cars[0].pose != start
    session_step(session, controls, [key(pg.K_a)])
    assert session.world.commands[0].steering == 1
    session_step(session, controls, [key(pg.K_w, up=True), key(pg.K_a, up=True)])
    assert session.world.commands[0].throttle == 0
    assert not session.paused
    assert session.world.commands[1].throttle != 0
    session_step(session, controls, [key(pg.K_DOWN), key(pg.K_RIGHT)])
    assert session.world.commands[0].throttle == -1
    assert session.world.commands[0].steering == -1


@pytest.mark.parametrize('keys, expected', [
    ([pg.K_w,pg.K_UP], (1,0)), ([pg.K_w,pg.K_s], (0,0)),
    ([pg.K_LEFT,pg.K_RIGHT], (0,0)), ([pg.K_UP,pg.K_LEFT], (1,1)),
])
def test_aliases_and_opposing_keys(keys, expected):
    controls = BattleInput(Gamepad())
    sample = controls.poll([key(k) for k in keys])
    assert (sample.throttle, sample.steering) == expected


def test_focus_loss_clears_keys_and_requires_enter_and_fresh_press():
    session = BattleSession(generate(seed=42, opponents=1))
    controls = BattleInput(Gamepad())
    session_step(session, controls, resume=True)
    session_step(session, controls, [key(pg.K_UP)])
    stamp = session.world.time
    session_step(session, controls, [SimpleNamespace(type=pg.WINDOWFOCUSLOST)])
    assert session.paused and session.world.time == stamp
    session_step(session, controls, [SimpleNamespace(type=pg.WINDOWFOCUSGAINED), key(pg.K_UP, repeat=True)])
    assert session.paused and not controls.pressed
    session_step(session, controls, [key(pg.K_RETURN)], resume=True)
    assert not session.paused and session.world.commands[0].throttle == 0
    session_step(session, controls, [key(pg.K_UP)])
    assert session.world.commands[0].throttle == 1


def test_focus_loss_and_gain_in_one_poll_still_pauses():
    controls = BattleInput(Gamepad(connected=True))
    controls.poll([])
    controls.gamepad.sample = replace(controls.gamepad.sample, a=True)
    controls.poll([])
    sample = controls.poll([SimpleNamespace(type=pg.WINDOWFOCUSLOST),
                            SimpleNamespace(type=pg.WINDOWFOCUSGAINED)])
    assert not sample.connected


def test_keyboard_overrides_gamepad_without_blending_or_held_a_takeover():
    pad = Gamepad(connected=True)
    controls = BattleInput(pad)
    controls.poll([])
    pad.sample = replace(pad.sample, throttle=1, steering=-1, a=True)
    assert controls.poll([]).throttle == 1
    sample = controls.poll([key(pg.K_s), key(pg.K_a)])
    assert (sample.throttle,sample.steering) == (-1,1)
    sample = controls.poll([key(pg.K_s,up=True),key(pg.K_a,up=True)])
    assert sample.throttle == 0 and not sample.a
    assert controls.poll([]).throttle == 0  # held gamepad A cannot take over
    pad.sample = replace(pad.sample,a=False)
    controls.poll([])
    pad.sample = replace(pad.sample,a=True)
    assert controls.poll([]).throttle == 1
    assert controls.source == 'gamepad'


def test_active_pad_loss_pauses_enter_allows_keyboard_and_idle_pad_loss_is_ignored():
    pad = Gamepad(connected=True)
    controls = BattleInput(pad)
    session = BattleSession(generate(seed=42,opponents=1))
    session_step(session,controls,resume=True)
    pad.sample = replace(pad.sample,connected=False)
    session_step(session,controls)
    assert session.paused
    session_step(session,controls,[key(pg.K_RETURN)],resume=True)
    assert not session.paused and controls.source == 'keyboard'
    session_step(session,controls,[key(pg.K_w)])
    assert session.world.commands[0].throttle == 1
    pad.sample = replace(pad.sample,connected=True,generation=1,a=True,throttle=-1)
    session_step(session,controls)
    assert not session.paused and session.world.commands[0].throttle == 1
    pad.sample = replace(pad.sample,connected=False)
    session_step(session,controls)
    assert not session.paused
