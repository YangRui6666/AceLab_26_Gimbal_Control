from __future__ import annotations

import struct
import sys
import time
import unittest
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from usb_protocol import (
    CMD_VISION_AUTO_AIM,
    CMD_VISION_ENABLE_STREAM,
    CMD_VISION_HANDSHAKE_REQ,
    CMD_VISION_HEARTBEAT,
    MODE_AUTO_AIM,
    MODE_LOCK_PROTECT,
    MODE_SEARCH,
    MODE_STABLE,
)
from usb_session import UsbSession


class FakeSerial:
    def __init__(self) -> None:
        self.written: list[bytes] = []
        self.closed = False

    def write(self, data: bytes) -> int:
        self.written.append(data)
        return len(data)

    def read(self, _size: int) -> bytes:
        return b""

    def close(self) -> None:
        self.closed = True


def frame_cmd(frame: bytes) -> int:
    return frame[3]


def frame_payload(frame: bytes) -> bytes:
    data_len = frame[2]
    return frame[4 : 4 + data_len]


class UsbSessionTests(unittest.TestCase):
    def make_session(self, *, workflow: str = "semi", heartbeat_enabled: bool = True) -> tuple[UsbSession, FakeSerial]:
        session = UsbSession(
            workflow=workflow,
            keepalive_enabled=heartbeat_enabled,
            heartbeat_period_s=0.2,
            search_period_s=0.2,
            auto_period_s=0.005,
        )
        fake = FakeSerial()
        session._serial = fake
        session._connected = True
        session._port = "COM_TEST"
        return session, fake

    def test_connect_sequence_starts_with_handshake(self) -> None:
        session, fake = self.make_session(workflow="semi")
        session.send_handshake()
        self.assertEqual([frame_cmd(frame) for frame in fake.written], [CMD_VISION_HANDSHAKE_REQ])

    def test_auto_workflow_sends_stream_after_handshake(self) -> None:
        session, fake = self.make_session(workflow="auto")
        session.send_handshake()
        session._handle_incoming(0x02, struct.pack("<I", 123), time.monotonic())
        self.assertEqual(
            [frame_cmd(frame) for frame in fake.written],
            [CMD_VISION_HANDSHAKE_REQ, CMD_VISION_ENABLE_STREAM],
        )
        self.assertTrue(session.status_snapshot().stream_enabled)

    def test_semi_workflow_requires_explicit_stream(self) -> None:
        session, fake = self.make_session(workflow="semi")
        session.send_handshake()
        session._handle_incoming(0x02, struct.pack("<I", 123), time.monotonic())
        self.assertEqual([frame_cmd(frame) for frame in fake.written], [CMD_VISION_HANDSHAKE_REQ])
        session.send_stream_enable()
        self.assertEqual([frame_cmd(frame) for frame in fake.written][-1], CMD_VISION_ENABLE_STREAM)

    def test_workflow_switch_to_auto_enables_stream_after_handshake(self) -> None:
        session, fake = self.make_session(workflow="semi")
        session.send_handshake()
        session._handle_incoming(0x02, struct.pack("<I", 123), time.monotonic())
        session.set_workflow("auto")
        self.assertEqual([frame_cmd(frame) for frame in fake.written][-1], CMD_VISION_ENABLE_STREAM)
        self.assertTrue(session.status_snapshot().stream_enabled)

    def test_stop_mode_loop_keeps_heartbeat_running(self) -> None:
        session, fake = self.make_session(workflow="semi")
        session.send_handshake()
        session._handle_incoming(0x02, struct.pack("<I", 123), time.monotonic())
        session.start_search(periodic=True)
        session.stop_mode_loop()
        session._next_heartbeat_at = 0.0
        session._next_mode_tx_at = 0.0
        before = len(fake.written)
        session._maybe_send_periodic_traffic()
        after_cmds = [frame_cmd(frame) for frame in fake.written[before:]]
        self.assertEqual(after_cmds, [CMD_VISION_HEARTBEAT])
        heartbeat_payload = frame_payload(fake.written[-1])
        self.assertEqual(heartbeat_payload[0], MODE_STABLE)
        self.assertEqual(session.status_snapshot().mode_loop, "idle")

    def test_heartbeat_off_keeps_auto_loop_running(self) -> None:
        session, fake = self.make_session(workflow="semi")
        session.send_handshake()
        session._handle_incoming(0x02, struct.pack("<I", 123), time.monotonic())
        session.start_auto(10.0, -5.0, periodic=True)
        session.set_keepalive_enabled(False)
        session._next_heartbeat_at = 0.0
        session._next_mode_tx_at = 0.0
        before = len(fake.written)
        session._maybe_send_periodic_traffic()
        after_cmds = [frame_cmd(frame) for frame in fake.written[before:]]
        self.assertEqual(after_cmds, [CMD_VISION_AUTO_AIM])
        self.assertEqual(session.status_snapshot().target_mode, MODE_AUTO_AIM)

    def test_lock_notification_stops_mode_loop_and_updates_mode(self) -> None:
        session, _fake = self.make_session(workflow="semi")
        session.send_handshake()
        session._handle_incoming(0x02, struct.pack("<I", 123), time.monotonic())
        session.start_search(periodic=True)
        session._handle_incoming(0x08, struct.pack("<IB", 456, 3), time.monotonic())
        status = session.status_snapshot()
        self.assertEqual(status.current_mode, MODE_LOCK_PROTECT)
        self.assertEqual(status.target_mode, MODE_LOCK_PROTECT)
        self.assertEqual(status.mode_loop, "idle")
        self.assertEqual(status.last_lock_reason, 3)

    def test_feedback_updates_target_mode_when_loop_is_idle(self) -> None:
        session, _fake = self.make_session(workflow="semi")
        session.send_handshake()
        session._handle_incoming(0x02, struct.pack("<I", 123), time.monotonic())
        payload = struct.pack("<hhhIBB", 0, 0, 0, 99, MODE_SEARCH, 0)
        session._handle_incoming(0x03, payload, time.monotonic())
        self.assertEqual(session.status_snapshot().target_mode, MODE_SEARCH)


if __name__ == "__main__":
    unittest.main()
