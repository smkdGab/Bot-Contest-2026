from dataclasses import asdict, replace
import math

import pytest

from botticelli.config import Car, Config, Gamepad, from_dict, load
from botticelli.control import DriveCommand, DrivingRules, Input, map_command


def sample(t=1, **kwargs):
    return Input(t, connected=True, **kwargs)


def armed():
    rules = DrivingRules(Gamepad())
    rules.update(sample(), 1)
    rules.update(sample(lb=True, a=True), 1)
    assert rules.enabled
    return rules


def test_startup_requires_fresh_press_but_allows_stick_before_a():
    rules = DrivingRules(Gamepad())
    for s in [sample(lb=True, a=True), sample(throttle=1, a=True)]:
        assert rules.update(s, 1).throttle == 0
        assert not rules.enabled
    rules.update(sample(throttle=1, steering=-1), 1)
    assert rules.update(sample(throttle=1, steering=-1, a=True), 1) == DriveCommand(1, -1)


def test_hold_a_drives_release_stops_and_other_buttons_are_ignored():
    rules = armed()
    rules.update(sample(throttle=1, steering=0.8, a=True, b=True), 1)
    steering = rules.command.steering
    assert rules.enabled and rules.command.throttle == 1
    assert rules.update(sample(throttle=1), 1) == DriveCommand(0, steering)
    assert not rules.enabled
    rules.update(sample(a=True), 1)
    assert not rules.enabled
    rules.update(sample(), 1)
    rules.update(sample(a=True), 1)
    assert rules.enabled


@pytest.mark.parametrize("bad", [Input(1), sample(t=0), sample(t=2), sample(throttle=math.nan),
                                      sample(throttle=1.1), sample(generation=2, lb=True, a=True)])
def test_stale_loss_invalid_and_reconnect_require_rearm(bad):
    rules = armed()
    assert rules.update(bad, 1).throttle == 0
    assert not rules.enabled
    assert rules.update(sample(lb=True, a=True), 1).throttle == 0


def test_deadzones_direction_and_limits_are_shared():
    c = replace(Car(), forward_sign=-1, steering_sign=-1,
                steering_min=40, steering_center=85, steering_max=135)
    left, right = map_command(DriveCommand(1, 1), c), map_command(DriveCommand(-1, -1), c)
    assert (left.speed_percent, left.servo_angle) == (-25, 40)
    assert (right.speed_percent, right.servo_angle) == (25, 135)
    assert left.velocity == 0.075
    assert left.wheel_angle == math.radians(30)
    assert map_command(DriveCommand(1, 0), replace(c, speed_limit=0.256)).speed_percent == -25
    rules = armed()
    assert rules.update(sample(throttle=0.1, steering=0.1, lb=True, a=True), 1) == DriveCommand()
    for bad in [math.nan, math.inf, 1.01, -1.01]:
        with pytest.raises(ValueError):
            DriveCommand(bad, 0)


@pytest.mark.parametrize("changes", [
    {"drive_channel": 6}, {"drive_channel": 1}, {"drive_type": 0}, {"steering_type": 2},
    {"steering_max": 181}, {"steering_center": 60}, {"steering_min": 60.1},
    {"speed_limit": 0}, {"speed_limit": 1.1},
    {"max_speed": math.nan}, {"forward_sign": 0}, {"wheelbase": 0.4},
])
def test_rejects_invalid_calibration(changes):
    with pytest.raises(ValueError):
        replace(Config(), car=replace(Car(), **changes)).validate()


def test_example_and_unknown_fields_and_unconfirmed_hardware():
    cfg = load("config/bot.yaml")
    assert from_dict(asdict(cfg)) == cfg
    with pytest.raises(ValueError):
        cfg.validate(real=True)
    with pytest.raises((ValueError, TypeError)):
        from_dict({"token": "never accept secrets in YAML"})
    with pytest.raises(TypeError):
        from_dict({"hardware": {"token": "never stored"}})


def test_gentle_response_ramps_but_release_and_neutral_stop_immediately():
    rules = DrivingRules(Gamepad(response_curve=1.5, acceleration_rate=2.5, steering_rate=4))
    rules.update(sample(t=1), 1)
    command = rules.update(sample(t=1.02, throttle=0.5, a=True), 1.02)
    assert 0 < command.throttle <= 0.051
    command = rules.update(sample(t=1.04, throttle=0, a=True), 1.04)
    assert command.throttle == 0
    rules.update(sample(t=1.06, throttle=1, a=True), 1.06)
    assert rules.update(sample(t=1.08, throttle=1), 1.08).throttle == 0
