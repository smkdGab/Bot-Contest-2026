from dataclasses import replace
from types import SimpleNamespace

import pygame

from botticelli.config import Config, Gamepad
from botticelli.control import DriveCommand
from botticelli.dashboard import Dashboard, GamepadReader
from botticelli.lidar import LidarFrame, Observation
from botticelli.sensors import SensorReading


def test_dashboard_fresh_stale_and_unknown_pose(tmp_path):
    dashboard = Dashboard(Config())
    try:
        frame = LidarFrame(1, 0, 10, tuple([100, 500, 1000, 2500, 5000, 65535, 0, 1000]*64))
        dashboard.draw(Observation((1, 1, 0), (3, 3, 1), False, frame), DriveCommand(0.5, -0.5),
                       "Driving: hold LB + A", now=10.2)
        key = dashboard.heatmap_key
        path = tmp_path / "dashboard.png"
        dashboard.save(path)
        assert path.stat().st_size > 10000
        dashboard.draw(Observation(lidar=frame), DriveCommand(), "Input stale", now=11, source="REAL")
        assert dashboard.heatmap_key == key  # unchanged frames reuse the heatmap
    finally:
        dashboard.close()


class Joystick:
    def init(self): pass
    def quit(self): pass
    def get_instance_id(self): return 42
    def get_numbuttons(self): return 6
    def get_numaxes(self): return 2
    def get_numhats(self): return 1
    def get_name(self): return "loopback gamepad"
    def get_axis(self, index): return [1, -1][index]
    def get_button(self, index): return index in (0, 4)
    def get_hat(self, index): return (1, 1)


def test_stick_dpad_buttons_and_hotplug_generation(monkeypatch):
    count = [1]
    monkeypatch.setattr(pygame.joystick, "get_count", lambda: count[0])
    monkeypatch.setattr(pygame.joystick, "Joystick", lambda index: Joystick())
    reader = GamepadReader(Gamepad())
    stick = reader.poll([])
    assert (stick.throttle, stick.steering, stick.lb, stick.a, stick.b) == (1, -1, True, True, False)
    reader.config = replace(Gamepad(), dpad=True)
    dpad = reader.poll([])
    assert (dpad.throttle, dpad.steering) == (stick.throttle, stick.steering)
    count[0] = 0
    lost = reader.poll([SimpleNamespace(type=pygame.JOYDEVICEREMOVED, instance_id=42)])
    assert not lost.connected
    count[0] = 1
    assert reader.poll([]).generation > stick.generation


def test_dpad_can_override_one_axis_while_stick_controls_other(monkeypatch):
    monkeypatch.setattr(pygame.joystick, "get_count", lambda: 1)
    monkeypatch.setattr(pygame.joystick, "Joystick", lambda index: Joystick())
    monkeypatch.setattr(Joystick, "get_hat", lambda self, index: (1, 0))
    monkeypatch.setattr(Joystick, "get_axis", lambda self, index: [0, -0.5][index])
    sample = GamepadReader(Gamepad()).poll([])
    assert sample.throttle == 0.5 and sample.steering == -1


def test_trigger_throttle_and_stick_steering(monkeypatch):
    monkeypatch.setattr(pygame.joystick, "get_count", lambda: 1)
    monkeypatch.setattr(pygame.joystick, "Joystick", lambda index: Joystick())
    monkeypatch.setattr(Joystick, "get_numaxes", lambda self: 6)
    monkeypatch.setattr(Joystick, "get_hat", lambda self,index: (0,0))
    monkeypatch.setattr(Joystick, "get_axis", lambda self,index: {0:0.2,4:0.6,5:-1}.get(index,0))
    sample = GamepadReader(Gamepad(throttle_mode="triggers")).poll([])
    assert sample.throttle == 0.8 and sample.steering == -0.2


def test_dashboard_renders_real_telemetry_and_stale_state(monkeypatch):
    dashboard = Dashboard(Config())
    readings = (
        SensorReading("imu", 10, {"accel_mg": [1, -2, 1000]}),
        SensorReading("compass", 10, {"heading_deg": 45, "field_ut": [1, 2, 3]}),
        SensorReading("vision", 10, {"blocks": [[100, 100, 20, 20, 2, 99]], "arrows": []}),
        SensorReading("battery", 10, {"percent": 85}),
    )
    labels = []
    render_text = dashboard.text
    def collect(value, *args, **kwargs):
        labels.append(str(value))
        render_text(value, *args, **kwargs)
    monkeypatch.setattr(dashboard, "text", collect)
    try:
        dashboard.draw(Observation(sensors=readings), DriveCommand(), "Ready", now=10.2, source="REAL")
        assert "45°" in labels and "85%" in labels and "2" in labels
        assert "LIVE" in labels
        labels.clear()
        dashboard.draw(Observation(sensors=readings), DriveCommand(), "Ready", now=11, source="REAL")
        assert any("STALE" in label for label in labels)
    finally:
        dashboard.close()
