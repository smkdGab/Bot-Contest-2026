from argparse import Namespace
from dataclasses import asdict
import socket
import threading
import time

import pytest

from botticelli.__main__ import replay
from botticelli.config import Config
from botticelli.control import DriveCommand, Input
from botticelli.lidar import Observation
from botticelli.recording import Recorder, read
from botticelli.runtime import Driver


class Backend:
    def __init__(self):
        self.calls = []
        self.closed = False
        self.error = None

    def connect(self):
        pass

    def apply(self, command, *, valid_until):
        if self.error:
            raise self.error
        self.calls.append((time.monotonic(), command))

    def observe(self):
        return Observation()

    def stop(self):
        self.calls.append((time.monotonic(), DriveCommand()))

    def close(self):
        self.stop()
        self.closed = True


def wait_for(condition):
    deadline = time.monotonic()+1
    while not condition() and time.monotonic() < deadline:
        time.sleep(0.005)
    assert condition()


def enable(driver):
    driver.submit(Input(time.monotonic(), connected=True))
    wait_for(lambda: "Ready" in driver.snapshot()[2])
    driver.submit(Input(time.monotonic(), connected=True, lb=True, a=True))
    wait_for(lambda: "Driving" in driver.snapshot()[2])


def test_latest_only_timing_stale_input_and_reconnect():
    backend = Backend()
    driver = Driver(backend, Config())
    driver.start()
    try:
        enable(driver)
        for i in range(1000):
            driver.submit(Input(time.monotonic(), True, i/999, 0, True, True))
        wait_for(lambda: any(c.throttle == 1 for _, c in backend.calls))
        # UI stalls: independent worker expires input, stops, and refuses held reconnect.
        wait_for(lambda: "stale" in driver.snapshot()[2])
        assert driver.snapshot()[1].throttle == 0
        assert backend.calls[-1][1].throttle == 0
        driver.submit(Input(time.monotonic(), True, 1, 0, True, True, generation=2))
        time.sleep(0.06)
        assert driver.snapshot()[1].throttle == 0
        assert len(backend.calls) < 15  # the 1000 submissions never became queued actuator work
    finally:
        driver.close()
    assert backend.closed


def test_actuation_limit_and_release_stop():
    backend = Backend()
    driver = Driver(backend, Config())
    driver.start()
    try:
        enable(driver)
        for _ in range(15):
            driver.submit(Input(time.monotonic(), True, 1, 0, a=True))
            time.sleep(0.02)
        times = [t for t, c in backend.calls if c.throttle == 1]
        assert 3 <= len(times) <= 7
        assert all(b-a >= 0.049 for a, b in zip(times, times[1:]))
        driver.submit(Input(time.monotonic(), True, 1))
        wait_for(lambda: driver.snapshot()[1].throttle == 0)
        assert backend.calls[-1][1].throttle == 0
    finally:
        driver.close()


def test_backend_exception_disables_and_cleans_up():
    backend = Backend()
    driver = Driver(backend, Config())
    driver.start()
    try:
        enable(driver)
        backend.error = TimeoutError("uncertain result")
        driver.submit(Input(time.monotonic(), True, 1, 0, True, True))
        wait_for(lambda: driver.error is not None)
        wait_for(lambda: backend.closed)
        assert driver.snapshot()[1].throttle == 0
    finally:
        driver.close()


def test_recording_and_replay_without_network(tmp_path, monkeypatch, capsys):
    path = tmp_path / "session.jsonl"
    recorder = Recorder(str(path), Config(), "real")
    driver = Driver(Backend(), Config(), recorder)
    driver.start()
    time.sleep(0.08)
    driver.close()
    recorder.close()
    records = list(read(path))
    assert records[0]["dependencies"]["ir-sim"] == "2.11.0"
    assert records[0]["config"] == asdict(Config())
    assert records[-1]["data"] == "Session closed"
    assert any(r["type"] == "command" for r in records)
    assert "test!" not in path.read_text()
    def no_network(*args, **kwargs):
        raise AssertionError("Replay tried to open a socket")
    monkeypatch.setattr(socket, "socket", no_network)
    replay(Namespace(path=path, headless=True))
    assert "offline" in capsys.readouterr().out


def test_writer_queue_bounded_and_disk_errors_propagated(tmp_path, monkeypatch):
    release = threading.Event()
    write = Recorder._write
    def stalled_writer(self):
        release.wait(1)
        write(self)
    monkeypatch.setattr(Recorder, "_write", stalled_writer)
    recorder = Recorder(str(tmp_path / "bounded.jsonl"), Config(), "sim")
    try:
        for _ in range(256):
            recorder.emit("state", "test")
        with pytest.raises(RuntimeError, match="keep up"):
            recorder.emit("state", "full")
        recorder.error = OSError("disk full")
        with pytest.raises(RuntimeError, match="Recording failed"):
            recorder.emit("state", "failed")
    finally:
        release.set()
        with pytest.raises(RuntimeError, match="Recording failed"):
            recorder.close()
