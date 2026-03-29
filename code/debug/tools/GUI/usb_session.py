from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import serial

from usb_protocol import (
    CMD_GIMBAL_BOOT,
    CMD_GIMBAL_DISABLED,
    CMD_GIMBAL_FEEDBACK,
    CMD_GIMBAL_HANDSHAKE_ACK,
    CMD_VISION_AUTO_AIM,
    CMD_VISION_DISABLE,
    CMD_VISION_HANDSHAKE_REQ,
    CMD_VISION_SEARCH,
    CMD_VISION_UNLOCK,
    FrameParser,
    GimbalFeedback,
    build_auto_aim_payload,
    build_frame,
    format_feedback,
    parse_feedback,
    parse_handshake_ack,
)


@dataclass(slots=True)
class SessionEvent:
    kind: str
    text: str
    when_monotonic: float = field(default_factory=time.monotonic)
    cmd: Optional[int] = None
    payload: bytes = b""
    feedback: Optional[GimbalFeedback] = None
    timestamp_ms: Optional[int] = None
    port: Optional[str] = None


class UsbSession:
    def __init__(
        self,
        *,
        auto_search_on_handshake: bool = True,
        keepalive_enabled: bool = True,
        search_period_s: float = 0.2,
        auto_period_s: float = 0.02,
        read_timeout_s: float = 0.05,
    ) -> None:
        self.auto_search_on_handshake = auto_search_on_handshake
        self.keepalive_enabled = keepalive_enabled
        self.search_period_s = search_period_s
        self.auto_period_s = auto_period_s
        self.read_timeout_s = read_timeout_s

        self._serial: Optional[serial.Serial] = None
        self._port: Optional[str] = None
        self._parser = FrameParser()
        self._events: queue.Queue[SessionEvent] = queue.Queue()
        self._lock = threading.RLock()
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._connected = False
        self._keepalive_mode = "idle"
        self._next_keepalive_at = 0.0
        self._auto_yaw_deg = 0.0
        self._auto_pitch_deg = 0.0

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def port(self) -> Optional[str]:
        return self._port

    def get_event(self, timeout: float | None = None) -> SessionEvent:
        return self._events.get(timeout=timeout)

    def poll_events(self, limit: int = 100) -> list[SessionEvent]:
        events: list[SessionEvent] = []
        while len(events) < limit:
            try:
                events.append(self._events.get_nowait())
            except queue.Empty:
                break
        return events

    def connect(self, port: str, baudrate: int = 115200, timeout: float | None = None) -> None:
        if timeout is None:
            timeout = self.read_timeout_s

        if self._connected:
            self.disconnect()

        ser = serial.Serial(port, baudrate, timeout=timeout)

        with self._lock:
            self._serial = ser
            self._port = port
            self._parser = FrameParser()
            self._keepalive_mode = "idle"
            self._next_keepalive_at = 0.0
            self._stop_event.clear()
            self._connected = True
            self._thread = threading.Thread(target=self._io_loop, name="usb-session", daemon=True)
            self._thread.start()

        self._emit(SessionEvent("connected", f"Opened {port}", port=port))
        self.send_handshake()

    def disconnect(self) -> None:
        with self._lock:
            thread = self._thread
            if thread is None and not self._connected:
                return
            self._stop_event.set()

        if thread is not None:
            thread.join(timeout=1.0)

        with self._lock:
            if self._thread is thread:
                self._thread = None

    def set_keepalive_enabled(self, enabled: bool) -> None:
        with self._lock:
            self.keepalive_enabled = enabled
            if not enabled:
                self._set_keepalive_mode_locked("idle")
            if self._connected:
                text = "Periodic keepalive enabled" if enabled else "Periodic keepalive stopped"
                self._emit(SessionEvent("state", text))

    def set_periods(self, search_period_s: float, auto_period_s: float) -> None:
        with self._lock:
            self.search_period_s = max(0.01, search_period_s)
            self.auto_period_s = max(0.005, auto_period_s)
            if self._connected:
                self._emit(
                    SessionEvent(
                        "state",
                        (
                            f"Keepalive periods updated search={self.search_period_s:.3f}s "
                            f"auto={self.auto_period_s:.3f}s"
                        ),
                    )
                )

    def send_handshake(self) -> None:
        with self._lock:
            self._set_keepalive_mode_locked("idle")
            self._send_frame_locked(CMD_VISION_HANDSHAKE_REQ, b"", "TX 0x81 handshake")

    def start_search(self, *, periodic: Optional[bool] = None) -> None:
        if periodic is None:
            periodic = self.keepalive_enabled

        with self._lock:
            self._send_frame_locked(CMD_VISION_SEARCH, b"", "TX 0x83 search")
            self._set_keepalive_mode_locked("search" if periodic else "idle")
            if periodic:
                self._emit(SessionEvent("state", "Search keepalive armed"))

    def start_auto(self, yaw_deg: float, pitch_deg: float, *, periodic: Optional[bool] = None) -> None:
        if periodic is None:
            periodic = self.keepalive_enabled

        with self._lock:
            self._auto_yaw_deg = yaw_deg
            self._auto_pitch_deg = pitch_deg
            payload = build_auto_aim_payload(yaw_deg, pitch_deg)
            self._send_frame_locked(
                CMD_VISION_AUTO_AIM,
                payload,
                f"TX 0x84 auto yaw={yaw_deg:.3f} pitch={pitch_deg:.3f}",
            )
            self._set_keepalive_mode_locked("auto" if periodic else "idle")
            if periodic:
                self._emit(SessionEvent("state", "Auto-aim keepalive armed"))

    def disable(self) -> None:
        with self._lock:
            self._set_keepalive_mode_locked("idle")
            self._send_frame_locked(CMD_VISION_DISABLE, b"", "TX 0x87 disable")

    def unlock(self) -> None:
        with self._lock:
            self._send_frame_locked(CMD_VISION_UNLOCK, b"", "TX 0x88 unlock")
            if self.keepalive_enabled:
                self._set_keepalive_mode_locked("search")
                self._emit(SessionEvent("state", "Unlock sent, search keepalive armed"))

    def stop_keepalive(self) -> None:
        with self._lock:
            self._set_keepalive_mode_locked("idle")
            self._emit(SessionEvent("state", "Periodic keepalive stopped"))

    def _emit(self, event: SessionEvent) -> None:
        self._events.put(event)

    def _send_frame_locked(self, cmd: int, payload: bytes, text: str) -> None:
        ser = self._serial
        if ser is None:
            raise RuntimeError("serial port is not open")
        ser.write(build_frame(cmd, payload))
        self._emit(SessionEvent("tx", text, cmd=cmd, payload=payload, port=self._port))

    def _set_keepalive_mode_locked(self, mode: str) -> None:
        self._keepalive_mode = mode
        self._next_keepalive_at = time.monotonic()

    def _io_loop(self) -> None:
        disconnect_event: Optional[SessionEvent] = None

        try:
            while not self._stop_event.is_set():
                ser = self._serial
                if ser is None:
                    break

                try:
                    data = ser.read(256)
                except serial.SerialException as exc:
                    self._emit(SessionEvent("error", f"Serial error: {exc}"))
                    break

                if data:
                    received_at = time.monotonic()
                    for cmd, payload in self._parser.feed(data):
                        self._handle_incoming(cmd, payload, received_at)

                self._maybe_send_keepalive()
        finally:
            with self._lock:
                ser = self._serial
                port = self._port
                was_connected = self._connected
                self._serial = None
                self._connected = False
                self._keepalive_mode = "idle"
                self._thread = None

            if ser is not None:
                try:
                    ser.close()
                except serial.SerialException:
                    pass

            if was_connected:
                disconnect_event = SessionEvent("disconnected", f"Closed {port or 'port'}", port=port)

            if disconnect_event is not None:
                self._emit(disconnect_event)

    def _maybe_send_keepalive(self) -> None:
        with self._lock:
            if not self.keepalive_enabled or self._keepalive_mode == "idle":
                return

            now = time.monotonic()
            if now < self._next_keepalive_at:
                return

            if self._keepalive_mode == "search":
                self._send_frame_locked(CMD_VISION_SEARCH, b"", "TX 0x83 search keepalive")
                self._next_keepalive_at = now + self.search_period_s
            elif self._keepalive_mode == "auto":
                payload = build_auto_aim_payload(self._auto_yaw_deg, self._auto_pitch_deg)
                self._send_frame_locked(
                    CMD_VISION_AUTO_AIM,
                    payload,
                    (
                        "TX 0x84 auto keepalive "
                        f"yaw={self._auto_yaw_deg:.3f} pitch={self._auto_pitch_deg:.3f}"
                    ),
                )
                self._next_keepalive_at = now + self.auto_period_s

    def _handle_incoming(self, cmd: int, payload: bytes, received_at: float) -> None:
        if cmd == CMD_GIMBAL_BOOT:
            self._emit(SessionEvent("boot", "RX 0x01 boot ready", cmd=cmd, payload=payload))
            return

        if cmd == CMD_GIMBAL_HANDSHAKE_ACK:
            try:
                timestamp_ms = parse_handshake_ack(payload)
            except ValueError as exc:
                self._emit(SessionEvent("error", f"RX 0x02 {exc}", cmd=cmd, payload=payload))
                return

            self._emit(
                SessionEvent(
                    "handshake",
                    f"RX 0x02 handshake ok timestamp={timestamp_ms}",
                    cmd=cmd,
                    payload=payload,
                    timestamp_ms=timestamp_ms,
                )
            )
            if self.auto_search_on_handshake and self.keepalive_enabled:
                with self._lock:
                    self._set_keepalive_mode_locked("search")
                    self._send_frame_locked(
                        CMD_VISION_SEARCH,
                        b"",
                        "TX 0x83 search keepalive enabled (auto after handshake)",
                    )
                    self._next_keepalive_at = time.monotonic() + self.search_period_s
                    self._emit(SessionEvent("state", "Search keepalive armed after handshake"))
            return

        if cmd == CMD_GIMBAL_FEEDBACK:
            try:
                feedback = parse_feedback(payload, received_at=received_at)
            except ValueError as exc:
                self._emit(SessionEvent("error", f"RX 0x03 {exc}", cmd=cmd, payload=payload))
                return

            self._emit(
                SessionEvent(
                    "feedback",
                    format_feedback(feedback),
                    cmd=cmd,
                    payload=payload,
                    feedback=feedback,
                    timestamp_ms=feedback.timestamp_ms,
                )
            )
            return

        if cmd == CMD_GIMBAL_DISABLED:
            with self._lock:
                self._set_keepalive_mode_locked("idle")
            self._emit(SessionEvent("disabled", "RX 0x08 disabled", cmd=cmd, payload=payload))
            return

        self._emit(
            SessionEvent(
                "rx",
                f"RX 0x{cmd:02X} payload={payload.hex()}",
                cmd=cmd,
                payload=payload,
            )
        )
