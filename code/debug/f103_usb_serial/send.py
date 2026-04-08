from __future__ import annotations

import argparse
import struct
import sys
import time
from typing import Iterable

import serial
from serial.tools import list_ports

from usb_protocol import CMD_VISION_SEARCH, build_frame


def default_timestamp_ms() -> int:
    return int(time.time() * 1000) & 0xFFFFFFFF


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Auto-scan serial ports and keep sending RTK 0x83 SEARCH commands."
    )
    parser.add_argument(
        "--port",
        default=None,
        help=(
            "Serial port, for example COM5. If omitted, auto-detect the STMicroelectronics "
            "Virtual COM Port."
        ),
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="Nominal baudrate for pyserial",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="Serial timeout in seconds",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="How long to keep sending in seconds",
    )
    parser.add_argument(
        "--period-ms",
        type=int,
        default=100,
        help="Send period in milliseconds",
    )
    parser.add_argument(
        "--timestamp-ms",
        type=int,
        default=None,
        help="Override the initial 4-byte timestamp payload base",
    )
    return parser.parse_args()


def _port_text(port_info: object, attr: str) -> str:
    value = getattr(port_info, attr, None)
    return "" if value is None else str(value)


def describe_port(port_info: object) -> str:
    device = _port_text(port_info, "device")
    description = _port_text(port_info, "description")
    manufacturer = _port_text(port_info, "manufacturer")
    product = _port_text(port_info, "product")
    hwid = _port_text(port_info, "hwid")

    parts = [device]
    if description:
        parts.append(description)
    if manufacturer:
        parts.append(manufacturer)
    if product and product not in parts:
        parts.append(product)
    if hwid:
        parts.append(hwid)
    return " | ".join(part for part in parts if part)


def is_stmicroelectronics_port(port_info: object) -> bool:
    device_text = " ".join(
        _port_text(port_info, attr)
        for attr in ("device", "description", "manufacturer", "product", "interface", "hwid")
    ).lower()

    vid = getattr(port_info, "vid", None)
    pid = getattr(port_info, "pid", None)

    if vid == 0x0483:
        return True

    if "stmicroelectronics" in device_text:
        return True

    if "virtual com port" in device_text:
        return True

    if "stm32" in device_text and ("cdc" in device_text or "com port" in device_text):
        return True

    if "vid:pid=0483" in device_text:
        return True

    if pid is not None and vid is None and "stmicroelectronics" in device_text:
        return True

    return False


def _normalize_requested_port(explicit_port: str | None) -> str | None:
    if explicit_port is None:
        return None

    port = explicit_port.strip()
    if port.lower() in {"", "auto", "autoscan", "auto-scan"}:
        return None
    return port


def iter_candidate_ports(explicit_port: str | None) -> Iterable[object]:
    explicit_port = _normalize_requested_port(explicit_port)
    if explicit_port:
        yield explicit_port
        return

    for port in list_ports.comports():
        if is_stmicroelectronics_port(port):
            yield port


def list_available_ports() -> list[object]:
    return list(list_ports.comports())


def format_available_ports(ports: Iterable[object]) -> str:
    port_list = list(ports)
    if not port_list:
        return "(none)"
    return "; ".join(describe_port(port) for port in port_list)


def open_first_available_port(args: argparse.Namespace) -> tuple[serial.Serial, str]:
    last_error: Exception | None = None
    requested_port = _normalize_requested_port(args.port)
    candidates = list(iter_candidate_ports(requested_port))

    if not candidates:
        available = list_available_ports()
        if requested_port:
            raise serial.SerialException(f"failed to open {requested_port}: no such serial port")
        raise serial.SerialException(
            "no STMicroelectronics Virtual COM Port found. "
            f"Available ports: {format_available_ports(available)}"
        )

    for port_name in candidates:
        if hasattr(port_name, "device"):
            candidate_name = str(getattr(port_name, "device"))
        else:
            candidate_name = str(port_name)
        try:
            ser = serial.Serial(candidate_name, args.baudrate, timeout=args.timeout)
            return ser, candidate_name
        except serial.SerialException as exc:
            last_error = exc

    if requested_port:
        raise serial.SerialException(f"failed to open {requested_port}: {last_error}")

    raise serial.SerialException(
        "found STMicroelectronics Virtual COM Port candidate(s) but failed to open them: "
        f"{format_available_ports(candidates)}"
    )


def build_search_frame(timestamp_ms: int) -> bytes:
    payload = struct.pack("<I", timestamp_ms & 0xFFFFFFFF)
    return build_frame(CMD_VISION_SEARCH, payload)


def main() -> int:
    args = parse_args()
    if args.duration <= 0:
        print("--duration must be > 0", file=sys.stderr)
        return 2
    if args.period_ms <= 0:
        print("--period-ms must be > 0", file=sys.stderr)
        return 2

    base_timestamp_ms = default_timestamp_ms() if args.timestamp_ms is None else (args.timestamp_ms & 0xFFFFFFFF)

    try:
        ser, port_name = open_first_available_port(args)
    except serial.SerialException as exc:
        print(f"Failed to open serial port: {exc}", file=sys.stderr)
        return 1

    port_label = port_name
    for port_info in list_ports.comports():
        if getattr(port_info, "device", None) == port_name:
            port_label = describe_port(port_info)
            break

    sent_count = 0
    deadline = time.monotonic() + args.duration

    try:
        with ser:
            ser.reset_input_buffer()
            while True:
                now = time.monotonic()
                if now >= deadline:
                    break

                timestamp_ms = (base_timestamp_ms + int((now - (deadline - args.duration)) * 1000.0)) & 0xFFFFFFFF
                frame = build_search_frame(timestamp_ms)
                ser.write(frame)
                ser.flush()
                sent_count += 1
                time.sleep(args.period_ms / 1000.0)
    except serial.SerialException as exc:
        print(f"Failed while sending 0x83 search frames: {exc}", file=sys.stderr)
        return 1

    print(f"Opened {port_label} and sent 0x83 SEARCH for {args.duration:.1f}s.")
    print(f"period_ms={args.period_ms}")
    print(f"sent_count={sent_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
