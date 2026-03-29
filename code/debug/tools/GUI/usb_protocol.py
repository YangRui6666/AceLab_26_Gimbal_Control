from __future__ import annotations

import struct
import time
from dataclasses import dataclass, field

SOF = b"\xAA\x55"
EOF = b"\x5A\xA5"

CMD_GIMBAL_BOOT = 0x01
CMD_GIMBAL_HANDSHAKE_ACK = 0x02
CMD_GIMBAL_FEEDBACK = 0x03
CMD_GIMBAL_DISABLED = 0x08

CMD_VISION_HANDSHAKE_REQ = 0x81
CMD_VISION_SEARCH = 0x83
CMD_VISION_AUTO_AIM = 0x84
CMD_VISION_DISABLE = 0x87
CMD_VISION_UNLOCK = 0x88

MODE_DISABLED = 0
MODE_STANDBY = 1
MODE_SEARCH = 2
MODE_AUTO_AIM = 3
MODE_MANUAL = 4


@dataclass(slots=True)
class GimbalFeedback:
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    timestamp_ms: int
    mode: int
    reserved: int
    host_received_monotonic: float = field(default_factory=time.monotonic)


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    header = SOF + bytes([len(payload), cmd]) + payload
    return header + struct.pack("<H", crc16_modbus(header)) + EOF


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buffer.extend(data)
        frames: list[tuple[int, bytes]] = []

        while len(self.buffer) >= 8:
            if self.buffer[0:2] != SOF:
                del self.buffer[0]
                continue

            data_len = self.buffer[2]
            frame_len = 8 + data_len
            if len(self.buffer) < frame_len:
                break

            frame = bytes(self.buffer[:frame_len])
            if frame[-2:] != EOF:
                del self.buffer[0]
                continue

            expected_crc = struct.unpack("<H", frame[4 + data_len:6 + data_len])[0]
            actual_crc = crc16_modbus(frame[: 4 + data_len])
            if expected_crc != actual_crc:
                del self.buffer[0]
                continue

            frames.append((frame[3], frame[4 : 4 + data_len]))
            del self.buffer[:frame_len]

        return frames


def mode_name(mode: int) -> str:
    mapping = {
        MODE_DISABLED: "DISABLED",
        MODE_STANDBY: "STANDBY",
        MODE_SEARCH: "SEARCH",
        MODE_AUTO_AIM: "AUTO_AIM",
        MODE_MANUAL: "MANUAL",
    }
    return mapping.get(mode, f"UNKNOWN({mode})")


def parse_handshake_ack(payload: bytes) -> int:
    if len(payload) != 4:
        raise ValueError(f"invalid handshake payload length: {len(payload)}")
    return struct.unpack("<I", payload)[0]


def parse_feedback(payload: bytes, received_at: float | None = None) -> GimbalFeedback:
    if len(payload) != 12:
        raise ValueError(f"invalid feedback payload length: {len(payload)}")

    yaw, pitch, roll, timestamp_ms, mode, reserved = struct.unpack("<hhhIBB", payload)
    return GimbalFeedback(
        yaw_deg=yaw / 1000.0,
        pitch_deg=pitch / 1000.0,
        roll_deg=roll / 1000.0,
        timestamp_ms=timestamp_ms,
        mode=mode,
        reserved=reserved,
        host_received_monotonic=received_at if received_at is not None else time.monotonic(),
    )


def build_auto_aim_payload(
    yaw_deg: float,
    pitch_deg: float,
    timestamp_ms: int | None = None,
    fire_cmd: int = 0,
    anti_top: int = 0,
) -> bytes:
    if timestamp_ms is None:
        timestamp_ms = int(time.time() * 1000) & 0xFFFFFFFF

    return struct.pack(
        "<hhIBB",
        int(round(yaw_deg * 1000.0)),
        int(round(pitch_deg * 1000.0)),
        timestamp_ms,
        fire_cmd & 0xFF,
        anti_top & 0xFF,
    )


def format_feedback(feedback: GimbalFeedback) -> str:
    return (
        "RX 0x03 "
        f"yaw={feedback.yaw_deg:.3f}deg "
        f"pitch={feedback.pitch_deg:.3f}deg "
        f"roll={feedback.roll_deg:.3f}deg "
        f"timestamp={feedback.timestamp_ms} "
        f"mode={mode_name(feedback.mode)} "
        f"reserved={feedback.reserved}"
    )
