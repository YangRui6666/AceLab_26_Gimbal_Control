from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from usb_protocol import (
    LOCK_REASON_HEARTBEAT_TIMEOUT,
    MODE_AUTO_AIM,
    MODE_LOCK_PROTECT,
    MODE_STABLE,
    FrameParser,
    build_auto_aim_payload,
    build_frame,
    build_heartbeat_payload,
    lock_reason_name,
    mode_name,
    parse_feedback,
    parse_lock_notification,
)


class UsbProtocolTests(unittest.TestCase):
    def test_frame_parser_round_trip(self) -> None:
        payload = build_auto_aim_payload(12.5, -3.25, timestamp_ms=123456)
        frame = build_frame(0x84, payload)
        frames = FrameParser().feed(frame)
        self.assertEqual(frames, [(0x84, payload)])

    def test_build_heartbeat_payload_encodes_mode_and_timestamp(self) -> None:
        payload = build_heartbeat_payload(MODE_AUTO_AIM, timestamp_ms=42)
        self.assertEqual(payload, struct.pack("<BI", MODE_AUTO_AIM, 42))

    def test_parse_feedback_uses_latest_mode_enum(self) -> None:
        payload = struct.pack("<hhhIBB", 1500, -2500, 0, 88, MODE_LOCK_PROTECT, 0)
        feedback = parse_feedback(payload, received_at=1.5)
        self.assertAlmostEqual(feedback.yaw_deg, 1.5)
        self.assertAlmostEqual(feedback.pitch_deg, -2.5)
        self.assertEqual(feedback.timestamp_ms, 88)
        self.assertEqual(feedback.mode, MODE_LOCK_PROTECT)
        self.assertEqual(feedback.host_received_monotonic, 1.5)

    def test_parse_lock_notification(self) -> None:
        lock = parse_lock_notification(struct.pack("<IB", 1234, LOCK_REASON_HEARTBEAT_TIMEOUT))
        self.assertEqual(lock.timestamp_ms, 1234)
        self.assertEqual(lock.reason, LOCK_REASON_HEARTBEAT_TIMEOUT)
        self.assertEqual(lock_reason_name(lock.reason), "HEARTBEAT_TIMEOUT")

    def test_mode_names_match_new_firmware(self) -> None:
        self.assertEqual(mode_name(MODE_STABLE), "STABLE")
        self.assertEqual(mode_name(MODE_LOCK_PROTECT), "LOCK_PROTECT")


if __name__ == "__main__":
    unittest.main()
