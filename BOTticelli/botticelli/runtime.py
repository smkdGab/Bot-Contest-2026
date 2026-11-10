"""Latest-only control: simulation at 50 Hz, hardware at 20 Hz; independent of rendering."""
from dataclasses import asdict
import threading
import time

from .control import DriveCommand, DrivingRules, Input
from .lidar import Observation


class Driver:
    def __init__(self, backend, config, recorder=None):
        self.backend = backend
        self.config = config
        self.recorder = recorder
        self.rules = DrivingRules(config.gamepad)
        self.lock = threading.Lock()
        self.sample = Input(time.monotonic())
        self.observation = Observation()
        self.command = DriveCommand()
        self.state = self.rules.state
        self.error = None
        self.done = threading.Event()
        self.connected = threading.Event()
        self.wake = threading.Event()
        self.thread = threading.Thread(target=self._run, name="actuator-driver", daemon=True)

    def submit(self, sample: Input):
        with self.lock:
            self.sample = sample  # replacement, never a growing command queue
        self.wake.set()  # read the latest press or release without waiting for the actuator deadline

    def snapshot(self):
        with self.lock:
            return self.observation, self.command, self.state

    def start(self):
        self.thread.start()

    def _run(self):
        previous_command = previous_state = previous_lidar = None
        next_actuator = 0
        was_enabled = False
        try:
            self.backend.connect()
            self.connected.set()
            while not self.done.is_set():
                now = time.monotonic()
                with self.lock:
                    sample = self.sample
                command = self.rules.update(sample, now)
                if self.recorder:
                    if command != previous_command:
                        self.recorder.emit("command", asdict(command), now)
                    if self.rules.state != previous_state:
                        self.recorder.emit("state", self.rules.state, now)
                previous_command, previous_state = command, self.rules.state
                if now >= next_actuator or (was_enabled and not self.rules.enabled):
                    if self.rules.enabled:
                        # K10 checks this again between its two acknowledged servo writes.
                        self.backend.apply(command, valid_until=sample.timestamp + self.config.gamepad.stale_seconds)
                    else:
                        self.backend.stop()
                    observation = self.backend.observe()
                    if self.recorder:
                        self.recorder.emit("observation", {"pose": observation.pose,
                            "opponent": observation.opponent, "collision": observation.collision,
                            "sensors": [asdict(reading) for reading in observation.sensors]})
                        if observation.lidar is not None and observation.lidar is not previous_lidar:
                            self.recorder.emit("lidar", asdict(observation.lidar))
                            previous_lidar = observation.lidar
                    with self.lock:
                        self.observation = observation
                    next_actuator = max(now + getattr(self.backend, "period", 0.05), time.monotonic())
                was_enabled = self.rules.enabled
                with self.lock:
                    self.command, self.state = command, self.rules.state
                self.wake.wait(min(0.02, max(0, next_actuator-time.monotonic())))
                self.wake.clear()
        except BaseException as error:
            self.error = error
            with self.lock:
                self.command = self.rules.disable("Session failed: start a new session")
                self.state = self.rules.state
            if self.recorder:
                try:
                    self.recorder.emit("command", asdict(self.command))
                    self.recorder.emit("state", self.state)
                except RuntimeError:
                    pass
        finally:
            try:
                self.backend.close()
            except BaseException as error:
                self.error = self.error or error
            with self.lock:
                self.command = self.rules.disable("Session closed")
                self.state = self.rules.state
            if self.recorder:
                try:
                    self.recorder.emit("command", asdict(self.command))
                    self.recorder.emit("state", self.state)
                except RuntimeError as error:
                    self.error = self.error or error

    def close(self):
        self.done.set()
        self.wake.set()
        self.thread.join(timeout=3)
        if self.thread.is_alive():
            raise RuntimeError("Actuator cleanup did not finish within 3 seconds")
