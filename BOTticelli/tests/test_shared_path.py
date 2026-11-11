from dataclasses import replace

import pytest

from botticelli.config import Arena, Lidar
from botticelli.control import DrivingRules, Input, K10, map_command
from botticelli.simulation import Simulation
from test_k10 import Bot, config


def test_identical_input_commands_reach_both_backends():
    server = Bot()
    cfg = replace(config(server), arena=replace(Arena(), obstacles=[], opponent_speed=0),
                  lidar=replace(Lidar(), enabled=False))
    sim, real = Simulation(cfg), K10(cfg, "test!")
    samples = [Input(1, True), Input(1, True, lb=True, a=True),
               Input(1, True, 1, -0.5, a=True), Input(1, True, -1, 0.5, a=True),
               Input(1, True, 1, 0, b=True), Input(1, True, a=True)]
    sim_rules, real_rules = DrivingRules(cfg.gamepad), DrivingRules(cfg.gamepad)
    try:
        sim.connect()
        real.connect()
        for sample in samples:
            sim_command = sim_rules.update(sample, 1)
            real_command = real_rules.update(sample, 1)
            assert sim_command == real_command
            if sim_rules.enabled:
                sim.apply(sim_command)
                real.apply(real_command)
                assert sim.command == real.command
                mapped = map_command(sim_command, cfg.car)
                assert sim.robot.velocity[0, 0] == pytest.approx(mapped.velocity)
                assert server.frames()[-1] == bytes([0x23, 1, mapped.speed_percent & 0xFF])
            else:
                sim.stop()
                real.stop()
                assert sim.command.throttle == real.command.throttle == 0
    finally:
        sim.close()
        real.close()
        server.close()
