from dataclasses import replace
import math

import pytest

from botticelli.config import Config, Arena, Box
from botticelli.sim_sensors import SimSensors


def readings(model, pose, t, collision=False, opponent=(1, 2, 0)):
    return {r.name: r for r in model.update(pose, opponent, collision, t, 10+t)}


def test_stationary_gravity_heading_battery_and_motion():
    model = SimSensors(Config())
    first = readings(model, (1, 1, math.pi/2), 0)
    assert first["imu"].values["accel_mg"] == [0, 0, 1000]
    assert first["compass"].values["heading_deg"] == 0
    second = readings(model, (1, 1, math.pi/2), 60)
    assert second["battery"].values["percent"] < 100
    assert second["motion"].values["velocity_mps"] == 0
    assert all(r.received_at == 70 for r in second.values())


def test_body_acceleration_turning_and_contact():
    model = SimSensors(Config())
    readings(model, (1, 1, 0), 0)
    accelerating = readings(model, (1.01, 1, 0), 0.05)
    assert accelerating["imu"].values["accel_mg"][0] > 0
    turning = readings(model, (1.02, 1, 0.02), 0.1)
    assert turning["imu"].values["accel_mg"][1] > 0
    contact = readings(model, (1.02, 1, 0.02), 0.15, collision=True)
    assert contact["imu"].values["accel_mg"][0] < 0
    assert contact["motion"].values["velocity_mps"] == 0
    assert "bump" in contact
    assert readings(model, (1.02, 1, 0.02), 0.2, collision=True)["bump"] is contact["bump"]


def test_pinhole_targets_respect_fov_and_occlusion():
    cfg = replace(Config(), arena=replace(Arena(), obstacles=[]))
    model = SimSensors(cfg)
    front = model.vision((1, 1, math.pi/2), (1, 2, 0))
    assert front["blocks"] and front["blocks"][0][4] == 1
    assert not model.vision((1, 1, math.pi/2), (1, 0.5, 0))["blocks"]
    cfg = replace(cfg, arena=replace(cfg.arena, obstacles=[Box(1, 1.5, 0.8, 0.3, 1)]))
    hidden = SimSensors(cfg).vision((1, 1, math.pi/2), (1, 2, 0))
    assert all(block[4] != 1 for block in hidden["blocks"])
