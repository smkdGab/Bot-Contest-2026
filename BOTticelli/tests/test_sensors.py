from dataclasses import asdict, replace
import struct
import time

import pytest

from botticelli.k10 import K10
from botticelli.sensors import decode_sensor
from botticelli.lidar import Observation
from botticelli.recording import observation_from_dict
from test_k10 import bot, config


@pytest.mark.parametrize("body,name", [
    (b"\xaf\x00"+struct.pack("<Ihhh", 9, -12, 34, 1000), "imu"),
    (b"\x9f\x00"+struct.pack("<HHhhh", 8, 270, 3, -4, 5), "compass"),
    (b"\xa0\x00"+struct.pack("<IBhhh", 6, 3, 1, 2, 3), "bump"),
    (b"\x7f\x00"+struct.pack("<HHHBB", 7, 320, 240, 1, 0)+struct.pack("<HHHHBB", 100, 120, 30, 40, 2, 88), "vision"),
])
def test_exact_sensor_layouts_and_raw_recording_roundtrip(body, name):
    reading = decode_sensor(body, 10)
    assert reading.name == name and reading.received_at == 10
    assert observation_from_dict(asdict(Observation(sensors=(reading,)))).sensors == (reading,)
    for invalid in (body[:-1], body+bytes(8), body[:1]+b"\x03"+body[2:]):
        with pytest.raises(ValueError):
            decode_sensor(invalid, 10)


def test_sensor_streams_battery_dispatch_and_cleanup(bot):
    cfg = config(bot)
    cfg = replace(cfg, hardware=replace(cfg.hardware, sensors_enabled=True))
    bot.reject.add(0x93)  # optional unavailable sensor must not disable motion
    client = K10(cfg, "test!")
    client.connect()
    try:
        address = ("127.0.0.1", client.transport.sock.getsockname()[1])
        body = b"\xaf\x00"+struct.pack("<Ihhh", 9, -12, 34, 1000)
        bot.send(body, address)
        deadline = time.monotonic()+0.5
        while "imu" not in client.transport.sensors and time.monotonic() < deadline:
            time.sleep(0.005)
        readings = {r.name:r for r in client.observe().sensors}
        assert readings["imu"].values["accel_mg"] == [-12, 34, 1000]
        assert readings["battery"].values["percent"] == 85
        assert "error" in readings["compass"].values
        assert not client.transport.fault.is_set()
        original = readings["imu"]
        bot.send(body, address)  # duplicate must not refresh freshness
        bot.send(body[:-1], address)
        time.sleep(0.02)
        assert client.transport.sensors["imu"] is original
    finally:
        client.close()
    assert b"\xa2\x00\x00\x0a" in bot.frames()
    assert b"\x93\x00\x00\x0a" in bot.frames()
    assert b"\x72\x00\x08" in bot.frames()
    assert bot.frames()[-1] == b"\x42"


def test_unavailable_battery_ack_is_not_a_transport_failure(bot):
    bot.reject.add(0x29)
    cfg = config(bot)
    cfg = replace(cfg, hardware=replace(cfg.hardware, sensors_enabled=True))
    client = K10(cfg, "test!")
    client.connect()
    try:
        reading = next(r for r in client.observe().sensors if r.name == "battery")
        assert reading.values == {"error": "Battery unavailable"}
        assert not client.transport.fault.is_set()
    finally:
        client.close()
