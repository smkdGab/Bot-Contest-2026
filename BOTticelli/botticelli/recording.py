"""Bounded asynchronous JSONL writer and a streaming, network-free reader."""
from dataclasses import asdict
from importlib.metadata import distributions
import json
from pathlib import Path
import platform
import queue
import threading
import time

from .config import Config
from .protocol import LidarFrame
from .control import Observation

IR_SIM_REVISION = "96e3d6454dbb6a2bbedeee18bf77e685f4071d3d"


def versions():
    return {d.metadata["Name"]: d.version for d in distributions()}


class Recorder:
    def __init__(self, path: str, config: Config, backend: str, *, setup=None):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.file = open(path, "x", encoding="utf8")
        self.queue = queue.Queue(maxsize=256)
        self.error = None
        self.closing = threading.Event()
        self.origin = time.monotonic()
        # Config is reconstructed from known dataclasses: token and arbitrary YAML keys cannot leak.
        metadata = {"type": "metadata", "schema": 1, "time": 0, "backend": backend,
                    "monotonic_origin": self.origin,
                    "config": asdict(config), "python": platform.python_version(),
                    "dependencies": versions(), "ir_sim_revision": IR_SIM_REVISION}
        if setup is not None:
            metadata.update(schema=2, backend="battle", setup=setup)
            metadata.pop("config")
        self.file.write(json.dumps(metadata, allow_nan=False) + "\n")
        self.thread = threading.Thread(target=self._write, name="jsonl-writer", daemon=True)
        self.thread.start()

    def emit(self, kind: str, data, now=None):
        if self.closing.is_set():
            raise RuntimeError("Recording is already closed")
        if self.error is not None:
            raise RuntimeError("Recording failed") from self.error
        item = {"type": kind, "time": (time.monotonic() if now is None else now)-self.origin, "data": data}
        try:
            self.queue.put_nowait(item)
        except queue.Full as error:
            # Stop the session rather than blocking control or silently losing records.
            raise RuntimeError("Recording cannot keep up; session stopped") from error

    def _write(self):
        try:
            while not self.closing.is_set() or not self.queue.empty():
                try:
                    item = self.queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                self.file.write(json.dumps(item, allow_nan=False, separators=(",", ":")) + "\n")
        except Exception as error:
            self.error = error
        finally:
            try:
                self.file.close()
            except OSError as error:
                self.error = error

    def close(self):
        self.closing.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError("Recording writer did not finish within 2 seconds")
        if self.error is not None:
            raise RuntimeError("Recording failed") from self.error


def read(path):
    """Yield one record at a time; no backend is imported or constructed by replay."""
    with open(path, encoding="utf8") as source:
        for line in source:
            item = json.loads(line)
            if item.get("type") not in {"metadata", "command", "state", "observation", "lidar", "setup", "commands", "snapshot", "contact", "damage", "pop", "elimination", "pause", "result"}:
                raise ValueError("Unknown recording record type")
            yield item


def observation_from_dict(data):
    data = dict(data)
    from .protocol import SensorReading
    data["sensors"] = tuple(SensorReading(**item) for item in data.get("sensors", []))
    if data.get("lidar") is not None:
        data["lidar"] = lidar_from_dict(data["lidar"])
    return Observation(**data)


def lidar_from_dict(data):
    data = dict(data)
    data["values_mm"] = tuple(data["values_mm"])
    if len(data["values_mm"]) != 512:
        raise ValueError("Recording LiDAR must have 512 values")
    return LidarFrame(**data)
