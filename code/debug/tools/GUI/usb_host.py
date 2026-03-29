from __future__ import annotations

import argparse
import sys
import threading
import time
from dataclasses import dataclass
from queue import Empty, Queue

import serial

from usb_session import UsbSession


@dataclass
class HostCommandState:
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    keepalive_enabled: bool = True


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
    print(
        "Commands: handshake, search, auto <yaw_deg> <pitch_deg>, "
        "disable, unlock, stop, keepalive on, keepalive off, quit"
    )


def handle_user_command(command: str, state: HostCommandState, session: UsbSession) -> bool:
    if command == "":
        return True

    if command == "quit":
        return False

    if command == "help":
        print_help()
        return True

    if command == "handshake":
        session.send_handshake()
        return True

    if command == "search":
        session.start_search(periodic=state.keepalive_enabled)
        return True

    if command.startswith("auto "):
        parts = command.split()
        if len(parts) != 3:
            print("Usage: auto <yaw_deg> <pitch_deg>")
            return True
        try:
            state.yaw_deg = float(parts[1])
            state.pitch_deg = float(parts[2])
        except ValueError:
            print("yaw_deg and pitch_deg must be numbers")
            return True
        session.start_auto(state.yaw_deg, state.pitch_deg, periodic=state.keepalive_enabled)
        return True

    if command == "disable":
        session.disable()
        return True

    if command == "unlock":
        session.unlock()
        return True

    if command == "stop":
        session.stop_keepalive()
        return True

    if command == "keepalive on":
        state.keepalive_enabled = True
        session.set_keepalive_enabled(True)
        return True

    if command == "keepalive off":
        state.keepalive_enabled = False
        session.set_keepalive_enabled(False)
        return True

    print("Unknown command. Type help for command list.")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="STM32 USB CDC protocol test host")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    parser.add_argument("--baudrate", type=int, default=115200, help="Nominal baudrate for pyserial")
    parser.add_argument("--timeout", type=float, default=0.05, help="Read timeout in seconds")
    args = parser.parse_args()

    state = HostCommandState()
    command_queue: Queue[str] = Queue()
    session = UsbSession(
        auto_search_on_handshake=True,
        keepalive_enabled=state.keepalive_enabled,
        read_timeout_s=args.timeout,
    )

    try:
        session.connect(args.port, baudrate=args.baudrate, timeout=args.timeout)
    except serial.SerialException as exc:
        print(f"Failed to open {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"Opened {args.port}. Type help for commands.")
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

