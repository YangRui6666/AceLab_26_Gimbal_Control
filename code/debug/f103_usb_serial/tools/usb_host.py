from __future__ import annotations

import argparse
import sys
import threading
import time
from dataclasses import dataclass
from queue import Empty, Queue

import serial

from usb_protocol import lock_reason_name, mode_name
from usb_session import SessionStatus, UsbSession


@dataclass
class HostCommandState:
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    disable_alias_warned: bool = False


def input_worker(queue: Queue[str]) -> None:
    while True:
        try:
            command = input().strip()
        except EOFError:
            queue.put("quit")
            return
        queue.put(command)
        if command == "quit":
            return


def print_help() -> None:
    print("Commands:")
    print("  help                         Show this help")
    print("  status                       Show workflow and session state")
    print("  workflow semi|auto           Switch CLI workflow")
    print("  handshake                    Send 0x81 handshake")
    print("  stream                       Send 0x82 enable stream")
    print("  search                       Send 0x83 and keep SEARCH loop running")
    print("  auto <yaw_deg> <pitch_deg>   Send 0x84 and keep AUTO_AIM loop running")
    print("  stop                         Stop 0x83/0x84 loops; heartbeat mode returns to STABLE")
    print("  lock                         Send 0x87 lock")
    print("  unlock                       Send 0x88 unlock")
    print("  heartbeat on|off             Enable or stop 0x85 heartbeat")
    print("  quit                         Exit the CLI")


def format_status(status: SessionStatus) -> str:
    lines = [
        f"connected={status.connected} port={status.port or '-'} workflow={status.workflow}",
        (
            "handshake="
            f"{'ok' if status.handshake_completed else 'pending'} "
            f"stream={'on' if status.stream_enabled else 'off'} "
            f"heartbeat={'on' if status.heartbeat_enabled else 'off'}"
        ),
        (
            "current_mode="
            f"{mode_name(status.current_mode)} "
            f"target_mode={mode_name(status.target_mode)} "
            f"mode_loop={status.mode_loop}"
        ),
        (
            "periods_ms="
            f"heartbeat={int(status.heartbeat_period_s * 1000)} "
            f"search={int(status.search_period_s * 1000)} "
            f"auto={int(status.auto_period_s * 1000)}"
        ),
        (
            "auto_target="
            f"yaw={status.auto_yaw_deg:.3f} "
            f"pitch={status.auto_pitch_deg:.3f}"
        ),
    ]
    if status.last_lock_reason is not None:
        lines.append(f"last_lock_reason={lock_reason_name(status.last_lock_reason)}")
    return "\n".join(lines)


def _require_connection(session: UsbSession) -> bool:
    if session.connected:
        return True
    print("Not connected.")
    return False


def handle_user_command(command: str, state: HostCommandState, session: UsbSession) -> bool:
    if command == "":
        return True

    if command == "quit":
        return False

    if command == "help":
        print_help()
        return True

    if command == "status":
        print(format_status(session.status_snapshot()))
        return True

    parts = command.split()
    verb = parts[0].lower()

    if verb == "workflow":
        if len(parts) != 2 or parts[1] not in {"semi", "auto"}:
            print("Usage: workflow semi|auto")
            return True
        session.set_workflow(parts[1])
        return True

    if verb == "heartbeat":
        if len(parts) != 2 or parts[1] not in {"on", "off"}:
            print("Usage: heartbeat on|off")
            return True
        session.set_keepalive_enabled(parts[1] == "on")
        return True

    if verb == "handshake":
        if not _require_connection(session):
            return True
        session.send_handshake()
        return True

    if verb == "stream":
        if not _require_connection(session):
            return True
        session.send_stream_enable()
        return True

    if verb == "search":
        if not _require_connection(session):
            return True
        session.start_search(periodic=True)
        return True

    if verb == "auto":
        if not _require_connection(session):
            return True
        if len(parts) != 3:
            print("Usage: auto <yaw_deg> <pitch_deg>")
            return True
        try:
            state.yaw_deg = float(parts[1])
            state.pitch_deg = float(parts[2])
        except ValueError:
            print("yaw_deg and pitch_deg must be numbers")
            return True
        session.start_auto(state.yaw_deg, state.pitch_deg, periodic=True)
        return True

    if verb == "stop":
        if not _require_connection(session):
            return True
        session.stop_mode_loop()
        return True

    if verb == "lock":
        if not _require_connection(session):
            return True
        session.lock()
        return True

    if verb == "disable":
        if not _require_connection(session):
            return True
        if not state.disable_alias_warned:
            print("`disable` is deprecated; sending `lock` (0x87).")
            state.disable_alias_warned = True
        session.lock()
        return True

    if verb == "unlock":
        if not _require_connection(session):
            return True
        session.unlock()
        return True

    print("Unknown command. Type help for command list.")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="STM32 USB CDC protocol test host")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    parser.add_argument("--baudrate", type=int, default=115200, help="Nominal baudrate for pyserial")
    parser.add_argument("--timeout", type=float, default=0.05, help="Read timeout in seconds")
    parser.add_argument(
        "--workflow",
        choices=["semi", "auto"],
        default="semi",
        help="CLI workflow. 'semi' requires an explicit stream command; 'auto' enables stream after handshake.",
    )
    parser.add_argument(
        "--heartbeat-period-ms",
        type=int,
        default=200,
        help="Heartbeat period for 0x85 packets in milliseconds",
    )
    parser.add_argument(
        "--search-period-ms",
        type=int,
        default=200,
        help="Period for repeated 0x83 SEARCH commands in milliseconds",
    )
    parser.add_argument(
        "--auto-period-ms",
        type=int,
        default=5,
        help="Period for repeated 0x84 AUTO_AIM commands in milliseconds",
    )
    args = parser.parse_args()

    state = HostCommandState()
    command_queue: Queue[str] = Queue()
    session = UsbSession(
        workflow=args.workflow,
        keepalive_enabled=True,
        search_period_s=max(0.01, args.search_period_ms / 1000.0),
        auto_period_s=max(0.005, args.auto_period_ms / 1000.0),
        heartbeat_period_s=max(0.01, args.heartbeat_period_ms / 1000.0),
        read_timeout_s=args.timeout,
    )

    try:
        session.connect(args.port, baudrate=args.baudrate, timeout=args.timeout)
    except serial.SerialException as exc:
        print(f"Failed to open {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"Opened {args.port}. workflow={args.workflow}. Type help for commands.")
    print_help()

    worker = threading.Thread(target=input_worker, args=(command_queue,), daemon=True)
    worker.start()

    try:
        while True:
            for event in session.poll_events():
                print(event.text)

            try:
                while True:
                    command = command_queue.get_nowait()
                    if not handle_user_command(command, state, session):
                        return 0
            except Empty:
                pass

            time.sleep(0.02)
    except KeyboardInterrupt:
        return 0
    finally:
        session.disconnect()
        for event in session.poll_events():
            print(event.text)


if __name__ == "__main__":
    raise SystemExit(main())
