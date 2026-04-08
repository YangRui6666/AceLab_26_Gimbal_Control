from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import serial

from usb_protocol import (
    CMD_GIMBAL_BOOT,
    CMD_GIMBAL_FEEDBACK,
    CMD_GIMBAL_HANDSHAKE_ACK,
    CMD_GIMBAL_LOCK_NOTIFICATION,
    CMD_VISION_AUTO_AIM,
    CMD_VISION_ENABLE_STREAM,
    CMD_VISION_HANDSHAKE_REQ,
    CMD_VISION_HEARTBEAT,
    CMD_VISION_LOCK,
    CMD_VISION_SEARCH,
    CMD_VISION_UNLOCK,
    FrameParser,
    GimbalFeedback,
    build_auto_aim_payload,
    build_frame,
    build_heartbeat_payload,
    format_feedback,
    format_lock_notification,
    lock_reason_name,
    mode_name,
    parse_boot,
    parse_feedback,
    parse_handshake_ack,
    parse_lock_notification,
    MODE_AUTO_AIM,
    MODE_LOCK_PROTECT,
    MODE_SEARCH,
    MODE_STABLE,
)

WORKFLOW_SEMI = "semi"
WORKFLOW_AUTO = "auto"
MODE_LOOP_IDLE = "idle"
MODE_LOOP_SEARCH = "search"
MODE_LOOP_AUTO = "auto"


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
    lock_reason: Optional[int] = None


@dataclass(slots=True)
class SessionStatus:
    connected: bool
    port: Optional[str]
    workflow: str
    handshake_completed: bool
    stream_enabled: bool
    heartbeat_enabled: bool
    current_mode: int
    target_mode: int
    mode_loop: str
    auto_yaw_deg: float
    auto_pitch_deg: float
    heartbeat_period_s: float
    search_period_s: float
    auto_period_s: float
    last_lock_reason: Optional[int]


class UsbSession:
    def __init__(
        self,
        *,
        workflow: str = WORKFLOW_SEMI,
        auto_search_on_handshake: bool = False,
        keepalive_enabled: bool = True,
        search_period_s: float = 0.2,
        auto_period_s: float = 0.005,
        heartbeat_period_s: float = 0.2,
        read_timeout_s: float = 0.05,
    ) -> None:
        if workflow not in {WORKFLOW_SEMI, WORKFLOW_AUTO}:
            raise ValueError(f"invalid workflow: {workflow}")

        if auto_search_on_handshake and workflow == WORKFLOW_SEMI:
            workflow = WORKFLOW_AUTO

        self.workflow = workflow
        self.keepalive_enabled = keepalive_enabled
        self.search_period_s = max(0.01, search_period_s)
        self.auto_period_s = max(0.005, auto_period_s)
        self.heartbeat_period_s = max(0.01, heartbeat_period_s)
        self.read_timeout_s = read_timeout_s

        self._serial: Optional[serial.Serial] = None
        self._port: Optional[str] = None
        self._parser = FrameParser()
        self._events: queue.Queue[SessionEvent] = queue.Queue()
        self._lock = threading.RLock()
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._connected = False

        self._handshake_completed = False
        self._stream_enabled = False
        self._current_mode = MODE_STABLE
        self._target_mode = MODE_STABLE
        self._mode_loop = MODE_LOOP_IDLE
        self._next_heartbeat_at = 0.0
        self._next_mode_tx_at = 0.0
        self._auto_yaw_deg = 0.0
        self._auto_pitch_deg = 0.0
        self._last_lock_reason: Optional[int] = None

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

    def status_snapshot(self) -> SessionStatus:
        with self._lock:
            return SessionStatus(
                connected=self._connected,
                port=self._port,
                workflow=self.workflow,
                handshake_completed=self._handshake_completed,
                stream_enabled=self._stream_enabled,
                heartbeat_enabled=self.keepalive_enabled,
                current_mode=self._current_mode,
                target_mode=self._target_mode,
                mode_loop=self._mode_loop,
                auto_yaw_deg=self._auto_yaw_deg,
                auto_pitch_deg=self._auto_pitch_deg,
                heartbeat_period_s=self.heartbeat_period_s,
                search_period_s=self.search_period_s,
                auto_period_s=self.auto_period_s,
                last_lock_reason=self._last_lock_reason,
            )

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
            self._reset_runtime_state_locked()
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

    def set_workflow(self, workflow: str) -> None:
        if workflow not in {WORKFLOW_SEMI, WORKFLOW_AUTO}:
            raise ValueError(f"invalid workflow: {workflow}")

        with self._lock:
            self.workflow = workflow
            self._emit(SessionEvent("state", f"Workflow set to {workflow}"))
            if workflow == WORKFLOW_AUTO and self._handshake_completed and not self._stream_enabled:
                self._send_stream_enable_locked("TX 0x82 stream enable (workflow auto)")

    def set_keepalive_enabled(self, enabled: bool) -> None:
        with self._lock:
            self.keepalive_enabled = enabled
            if enabled:
                self._next_heartbeat_at = time.monotonic()
            if self._connected:
                text = "Heartbeat enabled" if enabled else "Heartbeat stopped"
                self._emit(SessionEvent("state", text))

    def set_periods(
        self,
        search_period_s: float,
        auto_period_s: float,
        heartbeat_period_s: float | None = None,
    ) -> None:
        with self._lock:
            self.search_period_s = max(0.01, search_period_s)
            self.auto_period_s = max(0.005, auto_period_s)
            if heartbeat_period_s is not None:
                self.heartbeat_period_s = max(0.01, heartbeat_period_s)
            if self._connected:
                self._emit(
                    SessionEvent(
                        "state",
                        (
                            "Periods updated "
                            f"heartbeat={self.heartbeat_period_s:.3f}s "
                            f"search={self.search_period_s:.3f}s "
                            f"auto={self.auto_period_s:.3f}s"
                        ),
                    )
                )

    def send_handshake(self) -> None:
        with self._lock:
            if not self._serial_ready_locked("send handshake"):
                return
            self._reset_runtime_state_locked()
            self._send_frame_locked(CMD_VISION_HANDSHAKE_REQ, b"", "TX 0x81 handshake")

    def send_stream_enable(self) -> None:
        with self._lock:
            self._send_stream_enable_locked("TX 0x82 stream enable")

    def start_search(self, *, periodic: Optional[bool] = None) -> None:
        if periodic is None:
            periodic = True

        with self._lock:
            if not self._serial_ready_locked("enter search"):
                return
            if not self._handshake_completed:
                self._emit(SessionEvent("warn", "Cannot enter SEARCH before handshake completes"))
                return
            if self._current_mode == MODE_LOCK_PROTECT:
                self._emit(SessionEvent("warn", "Current mode is LOCK_PROTECT; send unlock first"))
                return

            self._send_frame_locked(CMD_VISION_SEARCH, b"", "TX 0x83 search")
            self._target_mode = MODE_SEARCH
            if periodic:
                self._set_mode_loop_locked(MODE_LOOP_SEARCH)
                self._emit(SessionEvent("state", "Search loop armed"))
            else:
                self._set_mode_loop_locked(MODE_LOOP_IDLE)
                self._emit(SessionEvent("state", "SEARCH one-shot sent"))

    def start_auto(self, yaw_deg: float, pitch_deg: float, *, periodic: Optional[bool] = None) -> None:
        if periodic is None:
            periodic = True

        with self._lock:
            if not self._serial_ready_locked("enter auto aim"):
                return
            if not self._handshake_completed:
                self._emit(SessionEvent("warn", "Cannot enter AUTO_AIM before handshake completes"))
                return
            if self._current_mode == MODE_LOCK_PROTECT:
                self._emit(SessionEvent("warn", "Current mode is LOCK_PROTECT; send unlock first"))
                return

            self._auto_yaw_deg = yaw_deg
            self._auto_pitch_deg = pitch_deg
            payload = build_auto_aim_payload(yaw_deg, pitch_deg)
            self._send_frame_locked(
                CMD_VISION_AUTO_AIM,
                payload,
                f"TX 0x84 auto yaw={yaw_deg:.3f} pitch={pitch_deg:.3f}",
            )
            self._target_mode = MODE_AUTO_AIM
            if periodic:
                self._set_mode_loop_locked(MODE_LOOP_AUTO)
                self._emit(SessionEvent("state", "AUTO_AIM loop armed"))
            else:
                self._set_mode_loop_locked(MODE_LOOP_IDLE)
                self._emit(SessionEvent("state", "AUTO_AIM one-shot sent"))

    def lock(self) -> None:
        with self._lock:
            if not self._serial_ready_locked("send lock"):
                return
            self._set_mode_loop_locked(MODE_LOOP_IDLE)
            self._target_mode = MODE_LOCK_PROTECT
            self._send_frame_locked(CMD_VISION_LOCK, b"", "TX 0x87 lock")

    def disable(self) -> None:
        self.lock()

    def unlock(self) -> None:
        with self._lock:
            if not self._serial_ready_locked("send unlock"):
                return
            self._current_mode = MODE_STABLE
            self._target_mode = MODE_STABLE
            self._send_frame_locked(CMD_VISION_UNLOCK, b"", "TX 0x88 unlock")

    def stop_mode_loop(self) -> None:
        with self._lock:
            self._set_mode_loop_locked(MODE_LOOP_IDLE)
            self._target_mode = MODE_STABLE
            self._emit(SessionEvent("state", "Mode loop stopped; heartbeat mode set to STABLE"))

    def stop_keepalive(self) -> None:
        self.stop_mode_loop()

    def _emit(self, event: SessionEvent) -> None:
        self._events.put(event)

    def _reset_runtime_state_locked(self) -> None:
        self._handshake_completed = False
        self._stream_enabled = False
        self._current_mode = MODE_STABLE
        self._target_mode = MODE_STABLE
        self._mode_loop = MODE_LOOP_IDLE
        self._next_heartbeat_at = 0.0
        self._next_mode_tx_at = 0.0
        self._auto_yaw_deg = 0.0
        self._auto_pitch_deg = 0.0
        self._last_lock_reason = None

    def _serial_ready_locked(self, action: str) -> bool:
        if self._serial is not None:
            return True
        self._emit(SessionEvent("warn", f"Cannot {action}: serial port is not open"))
        return False

    def _send_frame_locked(self, cmd: int, payload: bytes, text: str) -> None:
        ser = self._serial
        if ser is None:
            raise RuntimeError("serial port is not open")
        ser.write(build_frame(cmd, payload))
        self._emit(SessionEvent("tx", text, cmd=cmd, payload=payload, port=self._port))

    def _set_mode_loop_locked(self, mode_loop: str) -> None:
        self._mode_loop = mode_loop
        self._next_mode_tx_at = time.monotonic()

    def _send_stream_enable_locked(self, text: str) -> bool:
        if not self._serial_ready_locked("enable stream"):
            return False
        if not self._handshake_completed:
            self._emit(SessionEvent("warn", "Cannot enable stream before handshake completes"))
            return False

        self._send_frame_locked(CMD_VISION_ENABLE_STREAM, b"", text)
        self._stream_enabled = True
        return True

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

                self._maybe_send_periodic_traffic()
        finally:
            with self._lock:
                ser = self._serial
                port = self._port
                was_connected = self._connected
                self._serial = None
                self._connected = False
                self._thread = None
                self._reset_runtime_state_locked()

            if ser is not None:
                try:
                    ser.close()
                except serial.SerialException:
                    pass

            if was_connected:
                disconnect_event = SessionEvent("disconnected", f"Closed {port or 'port'}", port=port)

            if disconnect_event is not None:
                self._emit(disconnect_event)

    def _maybe_send_periodic_traffic(self) -> None:
        with self._lock:
            if self._serial is None:
                return

            now = time.monotonic()

            if self._handshake_completed and self.keepalive_enabled and now >= self._next_heartbeat_at:
                payload = build_heartbeat_payload(self._target_mode)
                self._send_frame_locked(
                    CMD_VISION_HEARTBEAT,
                    payload,
                    f"TX 0x85 heartbeat mode={mode_name(self._target_mode)}",
                )
                self._next_heartbeat_at = now + self.heartbeat_period_s

            if self._mode_loop == MODE_LOOP_SEARCH and now >= self._next_mode_tx_at:
                self._send_frame_locked(CMD_VISION_SEARCH, b"", "TX 0x83 search loop")
                self._next_mode_tx_at = now + self.search_period_s
            elif self._mode_loop == MODE_LOOP_AUTO and now >= self._next_mode_tx_at:
                payload = build_auto_aim_payload(self._auto_yaw_deg, self._auto_pitch_deg)
                self._send_frame_locked(
                    CMD_VISION_AUTO_AIM,
                    payload,
                    (
                        "TX 0x84 auto loop "
                        f"yaw={self._auto_yaw_deg:.3f} pitch={self._auto_pitch_deg:.3f}"
                    ),
                )
                self._next_mode_tx_at = now + self.auto_period_s

    def _handle_incoming(self, cmd: int, payload: bytes, received_at: float) -> None:
        if cmd == CMD_GIMBAL_BOOT:
            try:
                timestamp_ms = parse_boot(payload)
            except ValueError as exc:
                self._emit(SessionEvent("error", f"RX 0x01 {exc}", cmd=cmd, payload=payload))
                return

            self._emit(
                SessionEvent(
                    "boot",
                    f"RX 0x01 boot timestamp={timestamp_ms}",
                    cmd=cmd,
                    payload=payload,
                    timestamp_ms=timestamp_ms,
                )
            )
            return

        if cmd == CMD_GIMBAL_HANDSHAKE_ACK:
            try:
                timestamp_ms = parse_handshake_ack(payload)
            except ValueError as exc:
                self._emit(SessionEvent("error", f"RX 0x02 {exc}", cmd=cmd, payload=payload))
                return

            with self._lock:
                self._handshake_completed = True
                self._stream_enabled = False
                self._current_mode = MODE_STABLE
                self._target_mode = MODE_STABLE
                self._next_heartbeat_at = time.monotonic()

            self._emit(
                SessionEvent(
                    "handshake",
                    f"RX 0x02 handshake ok timestamp={timestamp_ms}",
                    cmd=cmd,
                    payload=payload,
                    timestamp_ms=timestamp_ms,
                )
            )

            with self._lock:
                if self.workflow == WORKFLOW_AUTO:
                    self._send_stream_enable_locked("TX 0x82 stream enable (auto after handshake)")
            return

        if cmd == CMD_GIMBAL_FEEDBACK:
            try:
                feedback = parse_feedback(payload, received_at=received_at)
            except ValueError as exc:
                self._emit(SessionEvent("error", f"RX 0x03 {exc}", cmd=cmd, payload=payload))
                return

            with self._lock:
                self._current_mode = feedback.mode
                if self._mode_loop == MODE_LOOP_IDLE:
                    self._target_mode = feedback.mode

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

        if cmd == CMD_GIMBAL_LOCK_NOTIFICATION:
            try:
                lock = parse_lock_notification(payload)
            except ValueError as exc:
                self._emit(SessionEvent("error", f"RX 0x08 {exc}", cmd=cmd, payload=payload))
                return

            with self._lock:
                self._current_mode = MODE_LOCK_PROTECT
                self._target_mode = MODE_LOCK_PROTECT
                self._mode_loop = MODE_LOOP_IDLE
                self._last_lock_reason = lock.reason

            self._emit(
                SessionEvent(
                    "locked",
                    format_lock_notification(lock),
                    cmd=cmd,
                    payload=payload,
                    timestamp_ms=lock.timestamp_ms,
                    lock_reason=lock.reason,
                )
            )
            return

        self._emit(
            SessionEvent(
                "rx",
                f"RX 0x{cmd:02X} payload={payload.hex()}",
                cmd=cmd,
                payload=payload,
            )
        )
