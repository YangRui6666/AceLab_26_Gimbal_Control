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
import json
import math
import os
import struct
import sys
import time
import shutil
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple


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


def damped_step(
    current: float,
    velocity: float,
    target: float,
    omega: float,
    dt_s: float,
    snap_epsilon: float = 1e-3,
) -> Tuple[float, float]:
    if dt_s <= 0.0 or omega <= 0.0:
        return target, 0.0

    # Stable critically-damped discretization adapted from the common SmoothDamp formulation.
    x = omega * dt_s
    exp = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x)
    delta = current - target
    temp = (velocity + omega * delta) * dt_s
    new_velocity = (velocity - omega * temp) * exp
    new_value = target + (delta + temp) * exp

    if abs(new_value - target) <= snap_epsilon and abs(new_velocity) <= snap_epsilon:
        return target, 0.0

    return new_value, new_velocity


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
    abs_return_rate_deg_s: float
    abs_center_yaw_deg: float
    abs_center_pitch_deg: float
    yaw_scale: float
    pitch_scale: float
    invert_yaw: bool
    left_invert_y: bool
    right_invert_y: bool
    status_period_s: float
    auto_calibrate: bool
    reset_calibration: bool
    calibration_path: str
    calibration_activation: float
    calibration_min: float
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
class AxisCalibration:
    positive: float = 0.0
    negative: float = 0.0

    def observe(self, value: float) -> bool:
        changed = False
        if value > 0.0 and value > self.positive:
            self.positive = value
            changed = True
        elif value < 0.0 and -value > self.negative:
            self.negative = -value
            changed = True
        return changed

    def normalize(self, value: float, activation: float, minimum: float) -> float:
        if value > 0.0:
            observed = self.positive
        elif value < 0.0:
            observed = self.negative
        else:
            return 0.0

        if observed >= activation:
            scale = max(observed, minimum)
        else:
            scale = 1.0

        return clamp(value / scale, -1.0, 1.0)

    def to_dict(self) -> Dict[str, float]:
        return {
            "positive": self.positive,
            "negative": self.negative,
        }

    @classmethod
    def from_dict(cls, data: object) -> "AxisCalibration":
        if not isinstance(data, dict):
            return cls()
        return cls(
            positive=float(data.get("positive", 0.0) or 0.0),
            negative=float(data.get("negative", 0.0) or 0.0),
        )


@dataclass
class CornerCalibration:
    x: float = 0.0
    y: float = 0.0

    def observe(self, x_value: float, y_value: float) -> bool:
        changed = False
        abs_x = abs(x_value)
        abs_y = abs(y_value)
        if abs_x > self.x:
            self.x = abs_x
            changed = True
        if abs_y > self.y:
            self.y = abs_y
            changed = True
        return changed

    def to_dict(self) -> Dict[str, float]:
        return {
            "x": self.x,
            "y": self.y,
        }

    @classmethod
    def from_dict(cls, data: object) -> "CornerCalibration":
        if not isinstance(data, dict):
            return cls()
        return cls(
            x=float(data.get("x", 0.0) or 0.0),
            y=float(data.get("y", 0.0) or 0.0),
        )


class CalibrationStore:
    def __init__(self, config: Config) -> None:
        self._config = config
        self._axes: Dict[str, AxisCalibration] = {
            "left_x": AxisCalibration(),
            "left_y": AxisCalibration(),
            "right_x": AxisCalibration(),
            "right_y": AxisCalibration(),
        }
        self._corners: Dict[str, CornerCalibration] = {
            "left_ul": CornerCalibration(),
            "left_ur": CornerCalibration(),
            "left_dl": CornerCalibration(),
            "left_dr": CornerCalibration(),
            "right_ul": CornerCalibration(),
            "right_ur": CornerCalibration(),
            "right_dl": CornerCalibration(),
            "right_dr": CornerCalibration(),
        }
        self._dirty = False
        self._last_save_s = 0.0

    def load(self) -> None:
        if self._config.reset_calibration:
            return
        if not os.path.exists(self._config.calibration_path):
            return

        try:
            with open(self._config.calibration_path, "r", encoding="utf-8") as fp:
                payload = json.load(fp)
        except Exception:
            return

        if not isinstance(payload, dict):
            return

        axes = payload.get("axes", {})
        if not isinstance(axes, dict):
            return

        for name in self._axes:
            self._axes[name] = AxisCalibration.from_dict(axes.get(name))

        corners = payload.get("corners", {})
        if isinstance(corners, dict):
            for name in self._corners:
                self._corners[name] = CornerCalibration.from_dict(corners.get(name))

    def observe(self, name: str, value: float) -> None:
        if not self._config.auto_calibrate:
            return
        axis = self._axes[name]
        if axis.observe(value):
            self._dirty = True

    def observe_pair(self, stick: str, x_value: float, y_value: float) -> None:
        if not self._config.auto_calibrate:
            return
        if abs(x_value) < self._config.calibration_activation or abs(y_value) < self._config.calibration_activation:
            return

        key = self._corner_key(stick, x_value, y_value)
        if self._corners[key].observe(x_value, y_value):
            self._dirty = True

    def normalized(self, name: str, value: float) -> float:
        return self._axes[name].normalize(
            value,
            activation=self._config.calibration_activation,
            minimum=self._config.calibration_min,
        )

    def normalized_pair(self, stick: str, x_name: str, y_name: str, x_value: float, y_value: float) -> Tuple[float, float]:
        self.observe(x_name, x_value)
        self.observe(y_name, y_value)
        self.observe_pair(stick, x_value, y_value)

        x_scale = self._axis_scale(x_name, x_value)
        y_scale = self._axis_scale(y_name, y_value)

        if abs(x_value) >= self._config.calibration_activation and abs(y_value) >= self._config.calibration_activation:
            corner = self._corners[self._corner_key(stick, x_value, y_value)]
            if corner.x >= self._config.calibration_activation:
                x_scale = max(corner.x, self._config.calibration_min)
            if corner.y >= self._config.calibration_activation:
                y_scale = max(corner.y, self._config.calibration_min)

        return (
            clamp(x_value / x_scale, -1.0, 1.0),
            clamp(y_value / y_scale, -1.0, 1.0),
        )

    def summary(self, name: str) -> AxisCalibration:
        return self._axes[name]

    def corner_summary(self, name: str) -> CornerCalibration:
        return self._corners[name]

    def maybe_save(self, now_s: float) -> None:
        if not self._dirty:
            return
        if now_s - self._last_save_s < 1.0:
            return
        self.save(now_s)

    def save(self, now_s: Optional[float] = None) -> None:
        if not self._dirty and now_s is not None:
            self._last_save_s = now_s
            return

        directory = os.path.dirname(self._config.calibration_path)
        if directory:
            os.makedirs(directory, exist_ok=True)

        payload = {
            "axes": {name: axis.to_dict() for name, axis in self._axes.items()},
            "corners": {name: corner.to_dict() for name, corner in self._corners.items()},
        }
        with open(self._config.calibration_path, "w", encoding="utf-8") as fp:
            json.dump(payload, fp, indent=2, sort_keys=True)
            fp.write("\n")

        self._dirty = False
        self._last_save_s = time.monotonic() if now_s is None else now_s

    def _axis_scale(self, name: str, value: float) -> float:
        axis = self._axes[name]
        if value > 0.0:
            observed = axis.positive
        elif value < 0.0:
            observed = axis.negative
        else:
            return 1.0

        if observed >= self._config.calibration_activation:
            return max(observed, self._config.calibration_min)
        return 1.0

    @staticmethod
    def _corner_key(stick: str, x_value: float, y_value: float) -> str:
        horizontal = "l" if x_value < 0.0 else "r"
        vertical = "u" if y_value < 0.0 else "d"
        return f"{stick}_{vertical}{horizontal}"


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
        self._calibration = CalibrationStore(config)
        self._calibration.load()
        self._last_buttons = ButtonState()
        self._search_active = False
        self._tx_enabled_sent = False
        self._last_status_print = 0.0
        self._last_status_len = 0

    @staticmethod
    def _rising(now: bool, prev: bool) -> bool:
        return now and not prev

    def _normalized_pair(
        self,
        stick: str,
        x_name: str,
        y_name: str,
        raw_x: float,
        raw_y: float,
        invert_x: bool,
        invert_y: bool,
    ) -> Tuple[float, float]:
        x_value = -raw_x if invert_x else raw_x
        y_value = -raw_y if invert_y else raw_y
        x_norm, y_norm = self._calibration.normalized_pair(stick, x_name, y_name, x_value, y_value)
        return (
            apply_deadzone(x_norm, self._config.deadzone),
            apply_deadzone(y_norm, self._config.deadzone),
        )

    def _compute_left_offset(self, sample: JoystickSample) -> Tuple[float, float]:
        left_x, left_y = self._normalized_pair(
            "left",
            "left_x",
            "left_y",
            sample.left_x,
            sample.left_y,
            invert_x=self._config.invert_yaw,
            invert_y=self._config.left_invert_y,
        )
        offset_yaw = left_x * self._config.left_abs_range_deg * self._config.yaw_scale
        offset_pitch = left_y * self._config.left_abs_range_deg * self._config.pitch_scale
        return offset_yaw, offset_pitch

    def _compute_right_offset(self, sample: JoystickSample) -> Tuple[float, float]:
        right_x, right_y = self._normalized_pair(
            "right",
            "right_x",
            "right_y",
            sample.right_x,
            sample.right_y,
            invert_x=self._config.invert_yaw,
            invert_y=self._config.right_invert_y,
        )
        yaw_axis = apply_expo(
            right_x,
            self._config.right_curve_exp,
        )
        pitch_axis = apply_expo(
            right_y,
            self._config.right_curve_exp,
        )
        offset_yaw = yaw_axis * self._config.abs_range_deg * self._config.yaw_scale
        offset_pitch = pitch_axis * self._config.abs_range_deg * self._config.pitch_scale
        return offset_yaw, offset_pitch

    def _handle_commands(self, sample: JoystickSample, timestamp_ms: int) -> List[bytes]:
        frames: List[bytes] = []

        if not self._tx_enabled_sent:
            frames.append(build_frame(CMD_ENABLE_TX))
            self._tx_enabled_sent = True

        if self._rising(sample.buttons.start, self._last_buttons.start):
            frames.append(build_frame(CMD_ENABLE_TX))
            self._tx_enabled_sent = True

        if self._rising(sample.buttons.a, self._last_buttons.a):
            frames.append(build_frame(CMD_ENTER_SEARCH, pack_u32_le(timestamp_ms)))
            self._search_active = True

        if self._rising(sample.buttons.b, self._last_buttons.b):
            frames.append(build_frame(CMD_ENTER_LOCK))
            self._search_active = False

        if self._rising(sample.buttons.x, self._last_buttons.x):
            frames.append(build_frame(CMD_EXIT_LOCK))
            self._search_active = False

        left_offset_yaw, left_offset_pitch = self._compute_left_offset(sample)
        right_offset_yaw, right_offset_pitch = self._compute_right_offset(sample)
        command_yaw = left_offset_yaw + right_offset_yaw
        command_pitch = left_offset_pitch + right_offset_pitch

        if abs(command_yaw) > 1e-4 or abs(command_pitch) > 1e-4:
            self._search_active = False

        if not self._search_active:
            frames.append(
                build_frame(
                    CMD_AUTO_AIM_DELTA,
                    pack_aim_delta(command_yaw, command_pitch, timestamp_ms),
                )
            )

        self._last_buttons = sample.buttons
        return frames

    def maybe_print_status(self, sample: Optional[JoystickSample], serial_port: Optional[str], now_s: float) -> None:
        self._calibration.maybe_save(now_s)
        if now_s - self._last_status_print < self._config.status_period_s:
            return
        self._last_status_print = now_s

        if sample is None:
            line = (
                f"USB:{'OK' if serial_port else 'NO'} "
                f"MODE:{'SEARCH' if self._search_active else 'AIM'} "
                "LX:+0.00 LY:+0.00 RX:+0.00 RY:+0.00 "
                "LT:+0.0/+0.0 RT:+0.0/+0.0 CMD:+0.0/+0.0"
            )
            self._write_status_line(line)
            return

        left_offset_yaw, left_offset_pitch = self._compute_left_offset(sample)
        right_offset_yaw, right_offset_pitch = self._compute_right_offset(sample)
        command_yaw = left_offset_yaw + right_offset_yaw
        command_pitch = left_offset_pitch + right_offset_pitch
        left_x_cal = self._calibration.summary("left_x")
        left_y_cal = self._calibration.summary("left_y")
        left_ul_cal = self._calibration.corner_summary("left_ul")
        line = (
            f"USB:{'OK' if serial_port else 'NO'} "
            f"MODE:{'SEARCH' if self._search_active else 'AIM'} "
            f"LX:{sample.left_x:+.2f} LY:{sample.left_y:+.2f} "
            f"RX:{sample.right_x:+.2f} RY:{sample.right_y:+.2f} "
            f"LT:{left_offset_yaw:+.1f}/{left_offset_pitch:+.1f} "
            f"RT:{right_offset_yaw:+.1f}/{right_offset_pitch:+.1f} "
            f"CMD:{command_yaw:+.1f}/{command_pitch:+.1f} "
            f"CAL:{left_x_cal.negative:.2f}/{left_x_cal.positive:.2f}|{left_y_cal.negative:.2f}/{left_y_cal.positive:.2f} "
            f"UL:{left_ul_cal.x:.2f}/{left_ul_cal.y:.2f}"
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
        return self._handle_commands(sample, timestamp_ms)

    def save_calibration(self) -> None:
        self._calibration.save()


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
    parser.add_argument("--abs-return-rate", type=float, default=45.0, help="Right target second-order damping return rate")
    parser.add_argument("--abs-center-yaw", type=float, default=0.0, help="Right stick absolute center for yaw")
    parser.add_argument("--abs-center-pitch", type=float, default=0.0, help="Right stick absolute center for pitch")
    parser.add_argument("--yaw-scale", type=float, default=1.0, help="Global yaw gain")
    parser.add_argument("--pitch-scale", type=float, default=1.0, help="Global pitch gain")
    parser.add_argument("--invert-yaw", dest="invert_yaw", action="store_true", default=True, help="Invert yaw axis")
    parser.add_argument("--no-invert-yaw", dest="invert_yaw", action="store_false", help="Do not invert yaw axis")
    parser.add_argument("--left-invert-y", dest="left_invert_y", action="store_true", default=True, help="Invert left stick Y axis")
    parser.add_argument("--no-left-invert-y", dest="left_invert_y", action="store_false", help="Do not invert left stick Y axis")
    parser.add_argument("--right-invert-y", dest="right_invert_y", action="store_true", default=True, help="Invert right stick Y axis")
    parser.add_argument("--no-right-invert-y", dest="right_invert_y", action="store_false", help="Do not invert right stick Y axis")
    parser.add_argument("--status-period", type=float, default=0.02, help="Status print interval in seconds")
    parser.add_argument("--auto-calibrate", dest="auto_calibrate", action="store_true", default=True, help="Automatically learn stick axis limits")
    parser.add_argument("--no-auto-calibrate", dest="auto_calibrate", action="store_false", help="Disable automatic stick calibration")
    parser.add_argument("--reset-calibration", action="store_true", help="Ignore stored calibration and relearn from scratch")
    parser.add_argument("--calibration-path", default=os.path.join("tools", "gamepad_calibration.json"), help="Calibration JSON path")
    parser.add_argument("--calibration-activation", type=float, default=0.55, help="Observed magnitude required before auto calibration becomes active")
    parser.add_argument("--calibration-min", type=float, default=0.35, help="Lower bound for learned calibration denominator")
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
        abs_return_rate_deg_s=args.abs_return_rate,
        abs_center_yaw_deg=args.abs_center_yaw,
        abs_center_pitch_deg=args.abs_center_pitch,
        yaw_scale=args.yaw_scale,
        pitch_scale=args.pitch_scale,
        invert_yaw=args.invert_yaw,
        left_invert_y=args.left_invert_y,
        right_invert_y=args.right_invert_y,
        status_period_s=args.status_period,
        auto_calibrate=args.auto_calibrate,
        reset_calibration=args.reset_calibration,
        calibration_path=args.calibration_path,
        calibration_activation=args.calibration_activation,
        calibration_min=args.calibration_min,
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
        controller.save_calibration()
        if controller._last_status_len:
            sys.stdout.write("\n")
            sys.stdout.flush()
        serial_link.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
