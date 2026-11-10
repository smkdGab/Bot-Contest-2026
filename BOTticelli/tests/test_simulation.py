from dataclasses import replace

import pytest

from botticelli.config import Arena, Box, Config
from botticelli.control import DriveCommand
from botticelli.simulation import Simulation


@pytest.fixture
def sim():
    arena = replace(Arena(), start=[1, 1, 0], obstacles=[], opponent_speed=0)
    instance = Simulation(replace(Config(), arena=arena))
    instance.connect()
    yield instance
    instance.close()


def test_forward_reverse_turning_and_no_sim_window(sim):
    assert sim.env.headless
    start = sim.observe().pose
    for _ in range(round(2/sim.period)):
        sim.apply(DriveCommand(1, 0))
    forward = sim.observe().pose
    assert forward[0]-start[0] == pytest.approx(0.15, abs=1e-5)
    for _ in range(round(2/sim.period)):
        sim.apply(DriveCommand(-1, 0))
    assert sim.observe().pose[0] == pytest.approx(start[0])
    for _ in range(round(2/sim.period)):
        sim.apply(DriveCommand(1, 1))
    assert sim.observe().pose[2] > 0.3
    assert sim.observe().pose[1] > start[1]
    assert len(sim.robot.trajectory) == 0


def test_collision_recovery_and_steering_hold():
    cfg = replace(Config(), arena=replace(Arena(), start=[1, 1, 0], obstacles=[Box(1.7, 1, 0.2, 0.8, 0.3)], opponent_speed=0))
    sim = Simulation(cfg)
    sim.connect()
    try:
        for _ in range(round(10/sim.period)):
            sim.apply(DriveCommand(1, 0))
        assert sim.observe().collision
        contact_x = sim.observe().pose[0]
        for _ in range(round(1/sim.period)):
            sim.apply(DriveCommand(-1, 0))
        assert sim.observe().pose[0] < contact_x-0.05
        assert not sim.observe().collision
        sim.apply(DriveCommand(0.5, 0.5))
        state = sim.robot.state.copy()
        sim.stop()
        assert sim.robot.state[:3].tolist() == state[:3].tolist()
        assert sim.robot.state[3, 0] == state[3, 0]
    finally:
        sim.close()


def test_wall_collision_cannot_escape_arena(sim):
    sim.robot.set_state([3.7, 1, 0, 0])
    sim.env.refresh()
    for _ in range(round(4/sim.period)):
        sim.apply(DriveCommand(1, 0))
    assert sim.observe().collision
    assert sim.observe().pose[0] <= 3.75
    sim.apply(DriveCommand(-1, 0))
    assert not sim.observe().collision


def test_opponent_repeatability_and_lidar_8hz(sim):
    first = sim.observe().lidar
    for _ in range(6):
        sim.stop()
    assert sim.observe().lidar is first
    sim.stop()
    assert sim.observe().lidar.frame_id == first.frame_id+1
    route = Simulation(Config())
    pose = route.opponent_pose(0.75)
    assert route.opponent_pose(0.75+route.route_length) == pytest.approx(pose)


def test_start_overlap_rejected():
    cfg = replace(Config(), arena=replace(Arena(), obstacles=[Box(1, 1, 1, 1, 0.3)]))
    with pytest.raises(ValueError, match="overlaps"):
        Simulation(cfg).connect()


def test_example_starts_north_and_reaches_practice_speed():
    import math
    from botticelli.config import load
    cfg = load("config/bot.yaml")
    sim = Simulation(cfg)
    sim.connect()
    try:
        start = sim.observe().pose
        assert start[2] == pytest.approx(math.pi/2)
        for _ in range(round(2/sim.period)):
            sim.apply(DriveCommand(1, 0))
        pose = sim.observe().pose
        assert pose[0] == pytest.approx(start[0])
        assert pose[1]-start[1] == pytest.approx(1.20)
        assert cfg.car.speed_limit == 0.25
    finally:
        sim.close()


def test_opponent_cannot_cross_obstacle_or_hide_behind_a_different_render_pose():
    arena = replace(Arena(), start=[1, 1, 0], opponent_route=[[1, 2], [3, 2]],
                    opponent_speed=0.8, obstacles=[Box(2, 2, 0.4, 0.6, 0.3)])
    sim = Simulation(replace(Config(), arena=arena))
    sim.connect()
    try:
        for _ in range(100):
            sim.stop()
            assert not any(sim.opponent.geometry.intersects(obj.geometry)
                           for obj in sim.env.objects if obj is not sim.opponent)
        assert sim.observe().opponent[0] < 1.7
        assert sim.observe().opponent == pytest.approx(tuple(sim.opponent.state[:3, 0]))
        distance = sim.opponent_distance
        for _ in range(20): sim.stop()
        assert sim.opponent_distance == distance
    finally:
        sim.close()


def test_opponent_waits_for_player_then_resumes():
    arena = replace(Arena(), start=[2, 2, 0], opponent_route=[[1, 2], [3, 2]],
                    opponent_speed=0.8, obstacles=[])
    sim = Simulation(replace(Config(), arena=arena))
    sim.connect()
    try:
        for _ in range(100): sim.stop()
        stopped = sim.opponent_distance
        assert stopped < 1
        sim.robot.set_state([2, 1, 0, 0])
        sim.env.refresh()
        for _ in range(10): sim.stop()
        assert sim.opponent_distance > stopped
    finally:
        sim.close()


def test_fast_player_cannot_tunnel_through_thin_obstacle():
    arena = replace(Arena(), start=[1, 1, 0], drive_speed_scale=10, opponent_speed=0,
                    obstacles=[Box(1.28, 1, 0.01, 0.5, 0.3)])
    cfg = replace(Config(), arena=arena, car=replace(Config().car, max_speed=2, speed_limit=1))
    sim = Simulation(cfg)
    sim.connect()
    try:
        sim.apply(DriveCommand(1, 0))
        assert sim.observe().collision
        assert sim.observe().pose[0] == 1
        sim.apply(DriveCommand(-1, 0))
        assert sim.observe().pose[0] < 1
    finally:
        sim.close()
