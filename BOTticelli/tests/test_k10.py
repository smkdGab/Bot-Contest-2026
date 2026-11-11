from dataclasses import replace
import socket
import struct
import threading
import time

import pytest

from botticelli.config import Config, Hardware
from botticelli.control import DriveCommand
from botticelli.k10 import K10, K10Transport


def lidar_packet(frame_id=3, values=None):
    values = values or list(range(512))
    return b"\x8f\x00" + struct.pack("<HIH", frame_id, 12345, 512) + struct.pack("<512H", *values)


class Bot:
    """Ephemeral loopback only: no bot address or participant token is used."""
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.01)
        self.address = self.sock.getsockname()
        self.packets = []
        self.drop = set()
        self.reject = set()
        self.malformed = set()
        self.wrong = set()
        self.stream = False
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, body, address):
        self.sock.sendto(body + struct.pack(">II", len(self.packets), 987654), address)

    def run(self):
        while not self.done.is_set():
            try:
                packet, sender = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            self.packets.append((time.monotonic(), packet))
            action = packet[0]
            if action == 0x43 or action in self.drop:
                continue
            if action in self.wrong:
                self.send(b"\x45\x00", sender)
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stranger:
                    stranger.sendto(bytes([action, 0]) + bytes(8), sender)
                continue
            if self.stream and action == 0x83 and packet[1]:
                self.send(lidar_packet(), sender)
            if action in self.malformed:
                self.send(bytes([action]), sender)
            elif action == 0x44:
                self.send(packet, sender)
            elif action == 0x29:
                self.send(b"\x29\x03" if action in self.reject else b"\x29\x00\x55", sender)
            else:
                # Wrong-action traffic and asynchronous events must not consume this ACK.
                self.send(b"\x45\x00", sender)
                self.send(bytes([action, 3 if action in self.reject else 0]), sender)

    def frames(self):
        return [p for _, p in self.packets if p[0] != 0x43]

    def close(self):
        self.done.set()
        self.thread.join(timeout=1)
        self.sock.close()


@pytest.fixture
def bot():
    server = Bot()
    yield server
    server.close()


def config(bot):
    return replace(Config(), hardware=Hardware(host="127.0.0.1", port=bot.address[1],
        channels_confirmed=True, types_confirmed=True, forward_confirmed=True,
        steering_confirmed=True, speed_confirmed=True))


def test_register_configure_diagnostics_lidar_motion_and_cleanup(bot):
    bot.stream = True
    cfg = config(bot)
    client = K10(cfg, "test!")
    client.connect()
    try:
        client.apply(DriveCommand(1, 1))
        observation = client.observe()
        assert observation.pose is None
        assert observation.lidar.values_mm == tuple(range(512))
        assert observation.lidar.timestamp_ms == 12345
        steering = [p for p in bot.frames() if p[0] == 0x24]
        assert steering == [b"\x24\x02\x00\x78"]
        assert b"\x23\x01\x19" in bot.frames()
        # Stop holds steering: no angle write during stop.
        client.stop()
        assert len([p for p in bot.frames() if p[0] == 0x24]) == 1
    finally:
        client.close()
    frames = bot.frames()
    assert [p[0] for p in frames[:3]] == [0x41, 0x22, 0x22]
    assert frames[-5:] == [b"\x23\x01\x00", b"\x28", b"\x24\x02\x00\x5a", b"\x83\x00\x08\x00", b"\x42"]
    assert not client.cleanup_errors


@pytest.mark.parametrize("failure", ["reject", "drop"])
def test_failed_registration_does_not_configure_or_drive(bot, failure):
    getattr(bot, failure).add(0x41)
    client = K10(config(bot), "test!")
    with pytest.raises((RuntimeError, TimeoutError)):
        client.connect()
    assert not any(p[0] in (0x22, 0x23, 0x24) for p in bot.frames())
    assert bot.frames()[-1] == b"\x42"


@pytest.mark.parametrize("failure", ["wrong", "malformed", "drop", "reject"])
def test_wrong_sender_action_malformed_timeout_and_status_poison_session(bot, failure):
    getattr(bot, failure).add(0x24)
    transport = K10Transport(*bot.address, timeout=0.08)
    try:
        with pytest.raises((RuntimeError, TimeoutError)):
            transport.request(b"\x24\x02\x00\x5a")
        assert transport.fault.is_set()
        before = len(bot.frames())
        with pytest.raises(RuntimeError):
            transport.request(b"\x23\x01\x19")
        assert len(bot.frames()) == before
    finally:
        transport.close()


def test_timeout_during_type_configuration_still_zeros_uncertain_channel(bot):
    bot.drop.add(0x22)
    client = K10(config(bot), "test!")
    with pytest.raises(TimeoutError):
        client.connect()
    assert bot.frames()[-3:] == [b"\x23\x01\x00", b"\x28", b"\x42"]


def test_heartbeat_independent_of_waiting_ack_and_ping_has_no_status(bot):
    transport = K10Transport(*bot.address, timeout=0.12)
    try:
        assert transport.request(b"\x44\x07\x01\x02\x03", length=5) == b"\x44\x07\x01\x02\x03"
        transport.request(b"\x41test!")
        transport.start_heartbeat()
        bot.drop.add(0x24)
        with pytest.raises(TimeoutError):
            transport.request(b"\x24\x02\x00\x5a")
        times = [t for t, p in bot.packets if p[0] == 0x43]
        assert len(times) >= 3
        assert max(b-a for a, b in zip(times, times[1:])) < 0.05
    finally:
        transport.close()


def test_cleanup_attempts_all_stops_and_unregister_even_if_zero_fails(bot):
    client = K10(config(bot), "test!")
    client.connect()
    client.continuous.add(4)  # exercise per-channel cleanup, never a combined mask
    bot.drop.add(0x23)
    client.close()
    frames = bot.frames()
    assert frames[-6:] == [b"\x23\x01\x00", b"\x23\x10\x00", b"\x28", b"\x24\x02\x00\x5a", b"\x83\x00\x08\x00", b"\x42"]
    assert client.cleanup_errors


def test_expired_input_never_sends_motion(bot):
    client = K10(config(bot), "test!")
    client.connect()
    try:
        with pytest.raises(TimeoutError):
            client.apply(DriveCommand(1, 1), valid_until=time.monotonic()-1)
        assert not any(p[0] == 0x24 for p in bot.frames())
        with pytest.raises(RuntimeError):
            client.apply(DriveCommand(1, 1))
    finally:
        client.close()


def test_input_expiring_between_acks_discards_propulsion(bot, monkeypatch):
    client = K10(config(bot), "test!")
    client.connect()
    request = client.transport.request
    def delayed_steering(frame, **kwargs):
        result = request(frame, **kwargs)
        if frame[0] == 0x24:
            time.sleep(0.04)
        return result
    monkeypatch.setattr(client.transport, "request", delayed_steering)
    try:
        with pytest.raises(TimeoutError, match="awaiting steering"):
            client.apply(DriveCommand(1, 1), valid_until=time.monotonic()+0.02)
        assert not any(p[0] == 0x23 and p[2] != 0 for p in bot.frames())
        assert client.transport.fault.is_set()
    finally:
        client.close()


def test_stream_rejects_bad_status_length_count_and_out_of_order(bot):
    transport = K10Transport(*bot.address)
    try:
        transport.request(b"\x41test!")
        address = transport.sock.getsockname()
        address = ("127.0.0.1", address[1])
        bot.send(lidar_packet(10), address)
        time.sleep(0.02)
        assert transport.lidar.frame_id == 10
        original = transport.lidar
        for packet in [lidar_packet(9), lidar_packet(10), b"\x8f\x01" + lidar_packet(11)[2:],
                       lidar_packet(11)[:-2], lidar_packet(11)[:8] + b"\x01\x00" + lidar_packet(11)[10:]]:
            bot.send(packet, address)
        time.sleep(0.03)
        assert transport.lidar is original
    finally:
        transport.close()
