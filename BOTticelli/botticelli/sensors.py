"""Documented K10 sensor event layouts; all values remain in firmware units.

References: binary-protocol.md and SensorsScreen/protocol.py. No calibration
or sensor algorithm changes are made here.
"""
from dataclasses import dataclass
import struct


@dataclass(frozen=True)
class LidarFrame:
    frame_id: int
    timestamp_ms: int
    received_at: float  # local monotonic seconds, never the bot's clock
    values_mm: tuple[int, ...]

    def fresh(self, now: float, threshold: float) -> bool:
        return 0 <= now - self.received_at <= threshold


def decode_frame(body: bytes, received_at: float) -> LidarFrame:
    """Decode after diagnostics removal, with exact full-matrix length validation."""
    if len(body) != 1034 or body[0] != 0x8F or body[1] != 0:
        raise ValueError('Invalid LiDAR action, status, or length')
    frame_id, timestamp, count = struct.unpack_from('<HIH', body, 2)
    if count != 512:
        raise ValueError('Expected full 64 x 8 LiDAR frame')
    return LidarFrame(frame_id, timestamp, received_at, struct.unpack_from('<512H', body, 10))


@dataclass(frozen=True)
class SensorReading:
    name: str
    received_at: float
    values: dict


def decode_sensor(body: bytes, now: float) -> SensorReading:
    if len(body) < 2 or body[1] != 0:
        raise ValueError("Invalid sensor status")
    action = body[0]
    if action == 0x9F and len(body) == 12:
        frame, heading, x, y, z = struct.unpack_from("<HHhhh", body, 2)
        return SensorReading("compass", now, dict(frame_id=frame, heading_deg=heading,
                                                  field_ut=[x, y, z]))
    if action == 0xAF and len(body) == 12:
        frame, x, y, z = struct.unpack_from("<Ihhh", body, 2)
        return SensorReading("imu", now, dict(frame_id=frame, accel_mg=[x, y, z]))
    if action == 0xA0 and len(body) == 13:
        frame, direction, x, y, z = struct.unpack_from("<IBhhh", body, 2)
        if direction > 5:
            raise ValueError("Invalid bump direction")
        return SensorReading("bump", now, dict(frame_id=frame, direction=direction, accel_mg=[x, y, z]))
    if action == 0x7F and len(body) >= 10:
        frame, width, height, blocks, arrows = struct.unpack_from("<HHHBB", body, 2)
        if blocks > 8 or arrows > 8 or not width or not height or len(body) != 10+10*(blocks+arrows):
            raise ValueError("Invalid HuskyLens frame")
        items = [list(struct.unpack_from("<HHHHBB", body, 10+i*10)) for i in range(blocks+arrows)]
        return SensorReading("vision", now, dict(frame_id=frame, width=width, height=height,
                                                 blocks=items[:blocks], arrows=items[blocks:]))
    raise ValueError("Invalid sensor action or exact payload length")
