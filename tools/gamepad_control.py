#!/usr/bin/env python3
"""
Linux Xbox-style gamepad controller for the gimbal firmware.

Dependencies:
    pip install -r tools/requirements-gamepad.txt

This script uses pygame for joystick input and pyserial for the STM32 USB CDC
virtual serial port. It speaks the existing USB frame protocol already handled
by the firmware:

    SOF:  AA 55
    LEN:  payload length
    CMD:  command id
    DATA: payload
    CRC:  CRC16/Modbus over SOF..DATA
    EOF:  5A A5
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
import shutil
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple


SOF_0 = 0xAA
SOF_1 = 0x55
EOF_0 = 0x5A
EOF_1 = 0xA5

CMD_ENABLE_TX = 0x82
CMD_ENTER_SEARCH = 0x83
CMD_AUTO_AIM_DELTA = 0x84
CMD_ENTER_LOCK = 0x87
CMD_EXIT_LOCK = 0x88


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
    if len(payload) > 255:
        raise ValueError("payload too large")

    frame = bytearray()
    frame.append(SOF_0)
    frame.append(SOF_1)
    frame.append(len(payload) & 0xFF)
    frame.append(cmd & 0xFF)
    frame.extend(payload)

    crc = crc16_modbus(bytes(frame))
    frame.extend(struct.pack("<H", crc))
    frame.append(EOF_0)
    frame.append(EOF_1)
    return bytes(frame)


def pack_u32_le(value: int) -> bytes:
    return struct.pack("<I", value & 0xFFFFFFFF)


def pack_aim_delta(yaw_deg: float, pitch_deg: float, timestamp_ms: int) -> bytes:
    yaw_raw = int(round(yaw_deg * 100.0))
    pitch_raw = int(round(pitch_deg * 100.0))
    yaw_raw = max(-32768, min(32767, yaw_raw))
    pitch_raw = max(-32768, min(32767, pitch_raw))
    return struct.pack("<hhI2x", yaw_raw, pitch_raw, timestamp_ms & 0xFFFFFFFF)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    return value


def axis_to_float(raw: float, deadzone: float, invert: bool = False) -> float:
    value = -raw if invert else raw
    return apply_deadzone(value, deadzone)


def apply_expo(value: float, exponent: float) -> float:
    exponent = max(0.1, exponent)
    if value == 0.0:
        return 0.0

    return math.copysign(abs(value) ** exponent, value)


def slew_towards(current: float, target: float, max_step: float) -> float:
    if max_step <= 0.0:
        return target

    delta = target - current
    if abs(delta) <= max_step:
        return target

    if delta > 0.0:
        return current + max_step

    return current - max_step


@dataclass
class Config:
    port: Optional[str]
    joystick_index: Optional[int]
    joystick_name: Optional[str]
    baudrate: int
    send_hz: float
    deadzone: float
    left_abs_range_deg: float
    abs_range_deg: float
    right_curve_exp: float
    abs_slew_rate_deg_s: float
    abs_center_yaw_deg: float
    abs_center_pitch_deg: float
    yaw_scale: float
    pitch_scale: float
    invert_yaw: bool
    left_invert_y: bool
    right_invert_y: bool
    status_period_s: float
    axis_left_x: int
    axis_left_y: int
    axis_right_x: int
    axis_right_y: int
    button_a: int
    button_b: int
    button_x: int
    button_lb: int
    button_rb: int
    button_start: int
    dry_run: bool
    scan_interval_s: float


@dataclass
class ButtonState:
    a: bool = False
    b: bool = False
    x: bool = False
    lb: bool = False
    rb: bool = False
    start: bool = False


@dataclass
class JoystickSample:
    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0
    buttons: ButtonState = field(default_factory=ButtonState)


class SerialLink:
    def __init__(self, config: Config) -> None:
        self._config = config
        self._serial = None
        self._port = config.port

    def _load_serial(self):
        try:
            import serial  # type: ignore
            from serial.tools import list_ports  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required. Install it with: pip install -r tools/requirements-gamepad.txt"
            ) from exc
        return serial, list_ports

    def discover_port(self, refresh: bool = False) -> Optional[str]:
        if self._config.port:
            return self._config.port
        if self._port and not refresh:
            return self._port

        _, list_ports = self._load_serial()
        ports = list(list_ports.comports())
        if not ports:
            return None

        scored: List[Tuple[int, str]] = []
        for item in ports:
            score = 0
            text = " ".join(
                part for part in [item.device, item.description, item.hwid] if part
            ).lower()
            if getattr(item, "vid", None) == 0x0483:
                score += 140
            if getattr(item, "pid", None) in {0x5740, 0x5750, 0xA4A7}:
                score += 120
            if "stm32" in text:
                score += 100
            if "virtual com" in text:
                score += 80
            if "stmicroelectronics" in text:
                score += 70
            if "cdc" in text:
                score += 60
            if "usb serial" in text:
                score += 40
            if item.device.lower().startswith("/dev/serial/by-id/"):
                score += 120
            if item.device.lower().startswith("/dev/ttyacm"):
                score += 90
            if item.device.lower().startswith("/dev/ttyusb"):
                score += 70
            scored.append((score, item.device))

        scored.sort(key=lambda pair: pair[0], reverse=True)
        return scored[0][1] if scored else None

    def open(self, refresh: bool = False) -> bool:
        serial_mod, _ = self._load_serial()
        port = self.discover_port(refresh=refresh)
        if port is None:
            return False

        try:
            self._serial = serial_mod.Serial(
                port=port,
                baudrate=self._config.baudrate,
                timeout=0,
                write_timeout=0,
            )
            self._port = port
            return True
        except Exception:
            self._serial = None
            if refresh:
                self._port = None
            return False

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    @property
    def port(self) -> Optional[str]:
        return self._port

    @property
    def is_open(self) -> bool:
        return self._serial is not None and getattr(self._serial, "is_open", False)

    def send(self, frame: bytes) -> bool:
        if self._config.dry_run:
            print(f"[dry-run] {frame.hex(' ')}")
            return True

        if not self.is_open:
            return False

        try:
            self._serial.write(frame)
            self._serial.flush()
            return True
        except Exception:
            self.close()
            return False


class GamepadReader:
    def __init__(self, config: Config) -> None:
        self._config = config
        self._pygame = None
        self._joystick = None

    def _load_pygame(self):
        try:
            import pygame  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "pygame is required. Install it with: pip install -r tools/requirements-gamepad.txt"
            ) from exc
        return pygame

    def initialize(self) -> bool:
        pygame = self._load_pygame()
        pygame.init()
        pygame.joystick.init()
        self._pygame = pygame
        return self._select_joystick()

    def _select_joystick(self) -> bool:
        assert self._pygame is not None
        pygame = self._pygame
        pygame.joystick.quit()
        pygame.joystick.init()

        count = pygame.joystick.get_count()
        if count <= 0:
            self._joystick = None
            return False

        indices = list(range(count))
        if self._config.joystick_index is not None:
            indices = [self._config.joystick_index] + [i for i in indices if i != self._config.joystick_index]

        for index in indices:
            if index < 0 or index >= count:
                continue
            joy = pygame.joystick.Joystick(index)
            joy.init()
            name = joy.get_name()
            if self._config.joystick_name and self._config.joystick_name.lower() not in name.lower():
                joy.quit()
                continue
            self._joystick = joy
            print(f"[gamepad] using joystick {index}: {name}")
            print(f"[gamepad] axes={joy.get_numaxes()} buttons={joy.get_numbuttons()} hats={joy.get_numhats()}")
            return True

        self._joystick = None
        return False

    def ensure_ready(self) -> bool:
        if self._pygame is None:
            return self.initialize()
        if self._joystick is not None:
            return True
        return self._select_joystick()

    def read(self) -> Optional[JoystickSample]:
        if not self.ensure_ready():
            return None

        assert self._pygame is not None and self._joystick is not None
        pygame = self._pygame
        joy = self._joystick
        pygame.event.pump()

        def get_axis(index: int) -> float:
            if index < 0 or index >= joy.get_numaxes():
                return 0.0
            return float(joy.get_axis(index))

        def get_button(index: int) -> bool:
            if index < 0 or index >= joy.get_numbuttons():
                return False
            return bool(joy.get_button(index))

        sample = JoystickSample(
            left_x=get_axis(self._config.axis_left_x),
            left_y=get_axis(self._config.axis_left_y),
            right_x=get_axis(self._config.axis_right_x),
            right_y=get_axis(self._config.axis_right_y),
            buttons=ButtonState(
                a=get_button(self._config.button_a),
                b=get_button(self._config.button_b),
                x=get_button(self._config.button_x),
                lb=get_button(self._config.button_lb),
                rb=get_button(self._config.button_rb),
                start=get_button(self._config.button_start),
            ),
        )
        return sample


class GimbalController:
    def __init__(self, config: Config) -> None:
        self._config = config
        self._last_buttons = ButtonState()
        self._virtual_yaw_deg = config.abs_center_yaw_deg
        self._virtual_pitch_deg = config.abs_center_pitch_deg
        self._right_target_yaw_deg = config.abs_center_yaw_deg
        self._right_target_pitch_deg = config.abs_center_pitch_deg
        self._tx_enabled_sent = False
        self._last_status_print = 0.0
        self._last_status_len = 0

    @staticmethod
    def _rising(now: bool, prev: bool) -> bool:
        return now and not prev

    def _compute_left_offset(self, sample: JoystickSample) -> Tuple[float, float]:
        left_x = axis_to_float(sample.left_x, self._config.deadzone, invert=self._config.invert_yaw)
        left_y = axis_to_float(sample.left_y, self._config.deadzone, invert=self._config.left_invert_y)
        offset_yaw = left_x * self._config.left_abs_range_deg * self._config.yaw_scale
        offset_pitch = left_y * self._config.left_abs_range_deg * self._config.pitch_scale
        return offset_yaw, offset_pitch

    def _update_right_target(self, sample: JoystickSample, dt_s: float) -> Tuple[float, float]:
        yaw_axis = apply_expo(
            axis_to_float(sample.right_x, self._config.deadzone, invert=self._config.invert_yaw),
            self._config.right_curve_exp,
        )
        pitch_axis = apply_expo(
            axis_to_float(sample.right_y, self._config.deadzone, invert=self._config.right_invert_y),
            self._config.right_curve_exp,
        )
        self._right_target_yaw_deg = clamp(
            self._right_target_yaw_deg + yaw_axis * self._config.abs_slew_rate_deg_s * dt_s * self._config.yaw_scale,
            -self._config.abs_range_deg,
            self._config.abs_range_deg,
        )
        self._right_target_pitch_deg = clamp(
            self._right_target_pitch_deg + pitch_axis * self._config.abs_slew_rate_deg_s * dt_s * self._config.pitch_scale,
            -self._config.abs_range_deg,
            self._config.abs_range_deg,
        )
        return self._right_target_yaw_deg, self._right_target_pitch_deg

    def _handle_commands(self, sample: JoystickSample, timestamp_ms: int, dt_s: float) -> List[bytes]:
        frames: List[bytes] = []

        if not self._tx_enabled_sent:
            frames.append(build_frame(CMD_ENABLE_TX))
            self._tx_enabled_sent = True

        if self._rising(sample.buttons.start, self._last_buttons.start):
            frames.append(build_frame(CMD_ENABLE_TX))
            self._tx_enabled_sent = True

        if self._rising(sample.buttons.a, self._last_buttons.a):
            frames.append(build_frame(CMD_ENTER_SEARCH, pack_u32_le(timestamp_ms)))

        if self._rising(sample.buttons.b, self._last_buttons.b):
            frames.append(build_frame(CMD_ENTER_LOCK))

        if self._rising(sample.buttons.x, self._last_buttons.x):
            frames.append(build_frame(CMD_EXIT_LOCK))

        self._right_target_yaw_deg, self._right_target_pitch_deg = self._update_right_target(sample, dt_s)

        left_offset_yaw, left_offset_pitch = self._compute_left_offset(sample)
        desired_yaw = self._right_target_yaw_deg + left_offset_yaw
        desired_pitch = self._right_target_pitch_deg + left_offset_pitch
        total_delta_yaw = desired_yaw - self._virtual_yaw_deg
        total_delta_pitch = desired_pitch - self._virtual_pitch_deg

        if abs(total_delta_yaw) > 1e-4 or abs(total_delta_pitch) > 1e-4:
            self._virtual_yaw_deg = desired_yaw
            self._virtual_pitch_deg = desired_pitch
            frames.append(
                build_frame(
                    CMD_AUTO_AIM_DELTA,
                    pack_aim_delta(total_delta_yaw, total_delta_pitch, timestamp_ms),
                )
            )

        self._last_buttons = sample.buttons
        return frames

    def maybe_print_status(self, sample: Optional[JoystickSample], serial_port: Optional[str], now_s: float) -> None:
        if now_s - self._last_status_print < self._config.status_period_s:
            return
        self._last_status_print = now_s

        if sample is None:
            line = (
                f"USB:{'OK' if serial_port else 'NO'} "
                "LX:+0.00 LY:+0.00 RX:+0.00 RY:+0.00 "
                "LT:+0.0/+0.0 RT:+0.0/+0.0 VT:+0.0/+0.0"
            )
            self._write_status_line(line)
            return

        left_offset_yaw, left_offset_pitch = self._compute_left_offset(sample)
        line = (
            f"USB:{'OK' if serial_port else 'NO'} "
            f"LX:{sample.left_x:+.2f} LY:{sample.left_y:+.2f} "
            f"RX:{sample.right_x:+.2f} RY:{sample.right_y:+.2f} "
            f"LT:{left_offset_yaw:+.1f}/{left_offset_pitch:+.1f} "
            f"RT:{self._right_target_yaw_deg:+.1f}/{self._right_target_pitch_deg:+.1f} "
            f"VT:{self._virtual_yaw_deg:+.1f}/{self._virtual_pitch_deg:+.1f}"
        )
        self._write_status_line(line)

    def _write_status_line(self, line: str) -> None:
        term_cols = shutil.get_terminal_size((120, 20)).columns
        max_len = max(20, term_cols - 1)
        if len(line) > max_len:
            line = line[: max_len - 3] + "..."
        sys.stdout.write("\r\x1b[2K" + line)
        sys.stdout.flush()
        self._last_status_len = len(line)

    def step(self, sample: Optional[JoystickSample], dt_s: float) -> List[bytes]:
        if sample is None:
            self._last_buttons = ButtonState()
            return []
        timestamp_ms = int(time.monotonic() * 1000.0) & 0xFFFFFFFF
        return self._handle_commands(sample, timestamp_ms, dt_s)


def parse_args(argv: Sequence[str]) -> Config:
    parser = argparse.ArgumentParser(description="Xbox gamepad control for the gimbal firmware (Linux + pygame + pyserial).")
    parser.add_argument("--port", default=None, help="Serial port, e.g. /dev/ttyACM0 or /dev/serial/by-id/...")
    parser.add_argument("--joystick-index", type=int, default=None, help="pygame joystick index")
    parser.add_argument("--joystick-name", default=None, help="Substring match for joystick name")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate (CDC ignores this, kept for completeness)")
    parser.add_argument("--send-hz", type=float, default=50.0, help="Command send rate")
    parser.add_argument("--deadzone", type=float, default=0.12, help="Analog stick deadzone")
    parser.add_argument("--left-abs-range", type=float, default=20.0, help="Left stick absolute offset range in degrees")
    parser.add_argument("--abs-range", type=float, default=45.0, help="Right stick absolute range in degrees")
    parser.add_argument("--right-curve-exp", type=float, default=1.8, help="Right stick expo curve exponent")
    parser.add_argument("--abs-slew-rate", type=float, default=60.0, help="Right stick absolute slew rate in deg/s")
    parser.add_argument("--abs-center-yaw", type=float, default=0.0, help="Right stick absolute center for yaw")
    parser.add_argument("--abs-center-pitch", type=float, default=0.0, help="Right stick absolute center for pitch")
    parser.add_argument("--yaw-scale", type=float, default=1.0, help="Global yaw gain")
    parser.add_argument("--pitch-scale", type=float, default=1.0, help="Global pitch gain")
    parser.add_argument("--invert-yaw", action="store_true", help="Invert yaw axis")
    parser.add_argument("--left-invert-y", dest="left_invert_y", action="store_true", default=True, help="Invert left stick Y axis")
    parser.add_argument("--no-left-invert-y", dest="left_invert_y", action="store_false", help="Do not invert left stick Y axis")
    parser.add_argument("--right-invert-y", dest="right_invert_y", action="store_true", default=False, help="Invert right stick Y axis")
    parser.add_argument("--no-right-invert-y", dest="right_invert_y", action="store_false", help="Do not invert right stick Y axis")
    parser.add_argument("--status-period", type=float, default=0.02, help="Status print interval in seconds")
    parser.add_argument("--axis-left-x", type=int, default=0)
    parser.add_argument("--axis-left-y", type=int, default=1)
    parser.add_argument("--axis-right-x", type=int, default=2)
    parser.add_argument("--axis-right-y", type=int, default=3)
    parser.add_argument("--button-a", type=int, default=0)
    parser.add_argument("--button-b", type=int, default=1)
    parser.add_argument("--button-x", type=int, default=2)
    parser.add_argument("--button-lb", type=int, default=4)
    parser.add_argument("--button-rb", type=int, default=5)
    parser.add_argument("--button-start", type=int, default=7)
    parser.add_argument("--dry-run", action="store_true", help="Print frames instead of sending them")
    parser.add_argument("--scan-interval", type=float, default=1.0, help="Serial rescan interval in seconds")

    args = parser.parse_args(argv)
    return Config(
        port=args.port,
        joystick_index=args.joystick_index,
        joystick_name=args.joystick_name,
        baudrate=args.baudrate,
        send_hz=args.send_hz,
        deadzone=args.deadzone,
        left_abs_range_deg=args.left_abs_range,
        abs_range_deg=args.abs_range,
        right_curve_exp=args.right_curve_exp,
        abs_slew_rate_deg_s=args.abs_slew_rate,
        abs_center_yaw_deg=args.abs_center_yaw,
        abs_center_pitch_deg=args.abs_center_pitch,
        yaw_scale=args.yaw_scale,
        pitch_scale=args.pitch_scale,
        invert_yaw=args.invert_yaw,
        left_invert_y=args.left_invert_y,
        right_invert_y=args.right_invert_y,
        status_period_s=args.status_period,
        axis_left_x=args.axis_left_x,
        axis_left_y=args.axis_left_y,
        axis_right_x=args.axis_right_x,
        axis_right_y=args.axis_right_y,
        button_a=args.button_a,
        button_b=args.button_b,
        button_x=args.button_x,
        button_lb=args.button_lb,
        button_rb=args.button_rb,
        button_start=args.button_start,
        dry_run=args.dry_run,
        scan_interval_s=args.scan_interval,
    )


def main(argv: Sequence[str]) -> int:
    config = parse_args(argv)
    gamepad = GamepadReader(config)
    serial_link = SerialLink(config)
    controller = GimbalController(config)

    if not gamepad.initialize():
        print("[gamepad] waiting for controller...", file=sys.stderr)

    if config.port is None:
        discovered = serial_link.discover_port()
        if discovered is not None:
            print(f"[serial] auto-selected port: {discovered}", file=sys.stderr)
        else:
            print("[serial] no port found yet, will keep scanning", file=sys.stderr)

    period_s = 1.0 / config.send_hz if config.send_hz > 0.0 else 0.02
    scan_interval_s = max(0.2, config.scan_interval_s)
    next_tick = time.monotonic()
    next_scan = 0.0

    try:
        while True:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(max(0.0, next_tick - now))
                continue
            next_tick = now + period_s

            sample = gamepad.read()
            if sample is None:
                controller.maybe_print_status(None, serial_link.port, now)
                if now >= next_scan:
                    if serial_link.open(refresh=True):
                        print(f"[serial] connected: {serial_link.port}", file=sys.stderr)
                    next_scan = now + scan_interval_s
                continue

            frames = controller.step(sample, period_s)
            if not serial_link.is_open and now >= next_scan:
                if serial_link.open(refresh=True):
                    print(f"[serial] connected: {serial_link.port}", file=sys.stderr)
                next_scan = now + scan_interval_s

            for frame in frames:
                ok = serial_link.send(frame)
                if not ok:
                    next_scan = 0.0
                    break

            controller.maybe_print_status(sample, serial_link.port, now)
    except KeyboardInterrupt:
        print("\n[exit] interrupted")
    finally:
        if controller._last_status_len:
            sys.stdout.write("\n")
            sys.stdout.flush()
        serial_link.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
