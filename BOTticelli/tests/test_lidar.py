from dataclasses import replace
import math

import numpy as np
import pytest

from botticelli.config import Box, Lidar
from botticelli.simulation import RayCaster
from botticelli.protocol import decode_frame
from test_k10 import lidar_packet


def frame(caster):
    return caster.scan((0, 0, 0), 10, 2)


def test_floor_intersections_mount_height_tilt_and_row_order():
    cfg = replace(Lidar(), x=0, height=0.5, tilt_deg=-45)
    caster = RayCaster(cfg, [])
    values = np.array(frame(caster).values_mm).reshape(8, 64)
    assert (values > 0).all()
    expected = -cfg.height / caster.rays[:, 2] * 1000
    assert np.max(np.abs(values.ravel()-expected)) <= 0.5
    assert values[0, 31] > values[7, 31]  # upper rays travel farther to floor
    higher = RayCaster(replace(cfg, height=1), [])
    assert frame(higher).values_mm[31] == pytest.approx(values[0, 31]*2, abs=1)


def test_obstacle_top_sides_occlusion_limits_and_left_to_right_order():
    cfg = replace(Lidar(), x=0, height=0.5)
    near = Box(1, 0.5, 0.2, 0.5, 2)
    caster = RayCaster(cfg, [near, Box(2, 1, 0.4, 1, 2)])
    values = frame(caster).values_mm
    left = [v for v in values[:32] if v]
    assert left and min(left) < 1500
    assert all(v == 0 for v in values[32:64])  # right rays miss boxes and floor
    assert frame(RayCaster(cfg, [near])).values_mm == values  # nearer object occludes farther one
    # Low object top: tilted rays can hit it while horizontal beams pass over it.
    down = RayCaster(replace(cfg, tilt_deg=-20), [Box(1, 0, 0.5, 0.6, 0.2)])
    hit = frame(down).values_mm[4*64+31]
    floor_only = frame(RayCaster(replace(cfg, tilt_deg=-20), [])).values_mm[4*64+31]
    assert 0 < hit < floor_only
    short = frame(RayCaster(replace(cfg, maximum=0.5), [near]))
    assert all(v == 0 for v in short.values_mm)
    close = frame(RayCaster(replace(cfg, minimum=0.3), [Box(0.2, 0, 0.1, 0.2, 1), Box(1, 0, 0.1, 1, 1)]))
    assert close.values_mm[3*64+31] == 0  # below-minimum surface still occludes farther target


def test_offset_yaw_pose_and_oriented_opponent():
    cfg = replace(Lidar(), x=0.2, y=0.1, height=0.5)
    caster = RayCaster(cfg, [Box(1, 0, 0.2, 2, 2)])
    values = frame(caster).values_mm
    assert values[3*64+31] == pytest.approx(700, abs=2)
    rotated = RayCaster(replace(cfg, yaw_deg=90), [Box(0, 1, 2, 0.2, 2)])
    assert frame(rotated).values_mm[3*64+31] == pytest.approx(800, abs=2)
    values = caster.scan((0, 0, math.pi/2), 10, 2).values_mm
    assert values[3*64+31] == 0  # sensor faces away from box
    empty = RayCaster(replace(cfg, x=0, y=0), [])
    result = empty.scan((0, 0, 0), 10, 2, (1, 0, math.pi/2), (0.8, 0.2, 1))
    assert result.values_mm[3*64+31] == pytest.approx(900, abs=2)


def test_seeded_noise_drops_and_freshness():
    cfg = replace(Lidar(), height=0.5, tilt_deg=-45, noise_std=0.02, drop_probability=0.3)
    one, two = RayCaster(cfg, []), RayCaster(cfg, [])
    frames1, frames2 = [frame(one) for _ in range(30)], [frame(two) for _ in range(30)]
    assert frames1 == frames2
    assert any(f is None for f in frames1) and any(f is not None for f in frames1)
    value = next(f for f in frames1 if f is not None)
    assert all(v == 0 or 100 <= v <= 5000 for v in value.values_mm)
    assert value.fresh(10.39, 0.4)
    assert not value.fresh(10.41, 0.4)
    assert not value.fresh(9, 0.4)
    assert frame(RayCaster(replace(cfg, drop_probability=1), [])) is None


def test_raw_values_preserved_without_clipping_flipping_or_interpolation():
    raw = list(range(510)) + [65535, 0]
    decoded = decode_frame(lidar_packet(values=raw), 10)
    assert decoded.values_mm == tuple(raw)
    assert decoded.frame_id == 3 and decoded.timestamp_ms == 12345
    with pytest.raises(ValueError):
        decode_frame(lidar_packet() + bytes(8), 10)  # diagnostics must be removed first
