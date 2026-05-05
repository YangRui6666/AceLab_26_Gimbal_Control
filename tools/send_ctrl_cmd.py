#!/usr/bin/env python3
"""
Send control commands to the gimbal via USB CDC serial protocol.

Supports step-response and sine-wave excitation for system identification.
Default mode (no arguments) enters an interactive keyboard-controlled session.

Dependencies:
    pip install pyserial

Usage:
    python tools/send_ctrl_cmd.py                          # interactive mode (default)
    python tools/send_ctrl_cmd.py step --yaw 15 --pitch 10 --hold-s 3
    python tools/send_ctrl_cmd.py sine --axis yaw --amp 10 --period-s 4 --duration-s 12
    python tools/send_ctrl_cmd.py enable
    python tools/send_ctrl_cmd.py lock
    python tools/send_ctrl_cmd.py unlock
    python tools/send_ctrl_cmd.py search
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import struct
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Callable, Dict, List, Optional, Sequence, Tuple

# ── Protocol constants ──────────────────────────────────────────────────────

SOF_0 = 0xAA
SOF_1 = 0x55
EOF_0 = 0x5A
EOF_1 = 0xA5

CMD_ENABLE_TX = 0x82
CMD_ENTER_SEARCH = 0x83
CMD_AUTO_AIM_ABS = 0x84
CMD_ENTER_LOCK = 0x87
CMD_EXIT_LOCK = 0x88

CMD_STATUS_FEEDBACK = 0x03
CMD_LOCK_FEEDBACK = 0x08

YAW_LIMIT_DEG = 60.0
PITCH_LIMIT_DEG_LOW = -10.0
PITCH_LIMIT_DEG_HIGH = 40.0
YAW_RATE_LIMIT_DPS = 2000.0

HOME_YAW_DEG = 0.0
HOME_PITCH_DEG = 0.0


# ── CRC16-Modbus ────────────────────────────────────────────────────────────

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


def pack_aim_abs(yaw_deg: float, pitch_deg: float, yaw_rate_dps: float, timestamp_ms: int) -> bytes:
    yaw_raw = max(-32768, min(32767, int(round(yaw_deg * 100.0))))
    pitch_raw = max(-32768, min(32767, int(round(pitch_deg * 100.0))))
    yaw_rate_raw = max(-32768, min(32767, int(round(yaw_rate_dps))))
    return struct.pack("<hhIh", yaw_raw, pitch_raw, timestamp_ms & 0xFFFFFFFF, yaw_rate_raw)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_target_yaw(yaw_deg: float) -> float:
    return clamp(yaw_deg, -YAW_LIMIT_DEG, YAW_LIMIT_DEG)


def clamp_target_pitch(pitch_deg: float) -> float:
    return clamp(pitch_deg, PITCH_LIMIT_DEG_LOW, PITCH_LIMIT_DEG_HIGH)


# ── Serial port ─────────────────────────────────────────────────────────────

@dataclass
class SerialConfig:
    port: Optional[str] = None
    baudrate: int = 115200


class SerialLink:
    def __init__(self, config: SerialConfig) -> None:
        self._config = config
        self._serial = None
        self._port = config.port
        self._disconnected = False

    def _load_serial(self):
        try:
            import serial  # type: ignore
            from serial.tools import list_ports  # type: ignore
        except ImportError as exc:
            raise RuntimeError("pyserial is required. Install: pip install pyserial") from exc
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
            text = " ".join(part for part in [item.device, item.description, item.hwid] if part).lower()
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
            self._serial = serial_mod.Serial(port=port, baudrate=self._config.baudrate,
                                             timeout=0, write_timeout=0)
            self._port = port
            self._disconnected = False
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

    @property
    def is_connected(self) -> bool:
        return self.is_open and not self._disconnected

    def send(self, frame: bytes) -> bool:
        if not self.is_open:
            return False
        try:
            self._serial.write(frame)
            self._serial.flush()
            return True
        except Exception:
            self._disconnected = True
            self.close()
            return False

    def read_all(self) -> bytes:
        if not self.is_open:
            return b""
        try:
            n = self._serial.in_waiting or 0
            return self._serial.read(n) if n > 0 else b""
        except Exception:
            self._disconnected = True
            return b""


# ── Feedback parser ─────────────────────────────────────────────────────────

@dataclass
class StatusFeedback:
    yaw_target_deg: float = 0.0
    pitch_target_deg: float = 0.0
    roll_target_deg: float = 0.0
    timestamp_ms: int = 0
    mode: int = 0
    mode_name: str = ""
    MODE_NAMES = {0: "STABLE", 1: "SEARCH", 2: "AUTO_AIM", 3: "LOCK", 4: "DISABLE"}


class FeedbackParser:
    def __init__(self) -> None:
        self._buf = bytearray()
        self.frames: List[StatusFeedback] = []

    def feed(self, raw: bytes) -> None:
        self._buf.extend(raw)
        self._parse()

    def _parse(self) -> None:
        while True:
            if len(self._buf) < 8:
                return
            sof_idx = None
            for i in range(len(self._buf) - 1):
                if self._buf[i] == SOF_0 and self._buf[i + 1] == SOF_1:
                    sof_idx = i
                    break
            if sof_idx is None:
                if len(self._buf) > 1:
                    del self._buf[: -1]
                return
            if sof_idx > 0:
                del self._buf[:sof_idx]
            payload_len = self._buf[2]
            total_len = payload_len + 8
            if total_len > 263:
                del self._buf[:1]
                continue
            if len(self._buf) < total_len:
                return
            eof_pos = total_len - 2
            if self._buf[eof_pos] != EOF_0 or self._buf[eof_pos + 1] != EOF_1:
                del self._buf[:1]
                continue
            frame_data = self._buf[:total_len]
            crc_stored = struct.unpack_from("<H", frame_data, total_len - 4)[0]
            crc_computed = crc16_modbus(frame_data[:total_len - 4])
            if crc_stored != crc_computed:
                del self._buf[:1]
                continue
            cmd = self._buf[3]
            if cmd == CMD_STATUS_FEEDBACK and payload_len >= 12:
                payload = self._buf[4 : 4 + 12]
                yaw_raw, pitch_raw, roll_raw, ts, mode_byte, _reserved = struct.unpack("<hhhIBB", payload)
                self.frames.append(StatusFeedback(
                    yaw_target_deg=yaw_raw / 100.0,
                    pitch_target_deg=-pitch_raw / 100.0,
                    roll_target_deg=roll_raw / 100.0,
                    timestamp_ms=ts,
                    mode=mode_byte,
                    mode_name=StatusFeedback.MODE_NAMES.get(mode_byte, "?"),
                ))
            del self._buf[:total_len]


# ── Terminal display ────────────────────────────────────────────────────────

# ANSI escape codes
ANSI_RESET = "\x1b[0m"
ANSI_BOLD = "\x1b[1m"
ANSI_RED = "\x1b[31m"
ANSI_GREEN = "\x1b[32m"
ANSI_YELLOW = "\x1b[33m"
ANSI_CYAN = "\x1b[36m"
ANSI_UP = "\x1b[A"
ANSI_CLR = "\x1b[2K"
ANSI_HIDE_CURSOR = "\x1b[?25l"
ANSI_SHOW_CURSOR = "\x1b[?25h"


def colorize(text: str, code: str, enabled: bool = True) -> str:
    return f"{code}{text}{ANSI_RESET}" if enabled else text


def make_progress_bar(frac: float, width: int = 14) -> str:
    filled = max(0, min(width, int(round(frac * width))))
    bar = "█" * filled + "░" * (width - filled)
    return f"[{bar}] {frac*100:3.0f}%"


class TerminalDisplay:
    def __init__(self, use_color: bool = True, use_ansi: bool = True) -> None:
        self.use_color = use_color
        self.use_ansi = use_ansi and sys.stdout.isatty()
        self._lines_written = 0

    def _c(self, text: str, code: str) -> str:
        return colorize(text, code, self.use_color and self.use_ansi)

    def begin_frame(self) -> None:
        if self.use_ansi:
            sys.stdout.write(ANSI_HIDE_CURSOR)
            self._lines_written = 0

    def end_frame(self) -> None:
        if self.use_ansi:
            sys.stdout.write(ANSI_SHOW_CURSOR)
        sys.stdout.flush()

    def render(self, lines: List[str]) -> None:
        """Render lines in-place without flicker: overwrite previous frame."""
        cols = shutil.get_terminal_size((120, 20)).columns

        if self.use_ansi:
            # Move cursor back to start of display area
            if self._lines_written > 0:
                sys.stdout.write(f"\x1b[{self._lines_written}A")

            for line in lines:
                if len(line) > cols - 1:
                    line = line[:cols - 4] + "..."
                sys.stdout.write(ANSI_CLR + line + "\n")

            # Clear any leftover lines from a taller previous frame
            extra = self._lines_written - len(lines)
            for _ in range(extra):
                sys.stdout.write(ANSI_CLR + "\n")
                sys.stdout.write(f"\x1b[{1}A")  # move back up after clearing

            self._lines_written = len(lines)
        else:
            compact = " | ".join(l.strip() for l in lines if l.strip())
            if len(compact) > cols - 1:
                compact = compact[:cols - 4] + "..."
            sys.stdout.write("\r" + ANSI_CLR + compact + "\n")
            self._lines_written = 1

    def show_message(self, text: str) -> None:
        """Print a message above the display, then restore display area."""
        if self.use_ansi:
            if self._lines_written > 0:
                sys.stdout.write(f"\x1b[{self._lines_written}A")
            sys.stdout.write(ANSI_CLR + text + "\n")
            # Cursor is now at the top of where the display would be;
            # next render() will move back up and overwrite.
            self._lines_written = 0
        else:
            sys.stdout.write("\r" + ANSI_CLR + text + "\n")


# ── Command aliases ─────────────────────────────────────────────────────────

CMD_ALIASES = {
    "s": "step", "si": "sine", "h": "home", "hd": "hold",
    "l": "lock", "u": "unlock", "se": "search", "d": "defaults",
    "q": "exit", "x": "exit",
}


# ── Excitation generators ───────────────────────────────────────────────────

class StepGenerator:
    def __init__(self, yaw_deg: float, pitch_deg: float) -> None:
        self.yaw = clamp_target_yaw(yaw_deg)
        self.pitch = clamp_target_pitch(pitch_deg)

    def eval(self, _t_s: float) -> Tuple[float, float, float]:
        return self.yaw, self.pitch, 0.0


class SineGenerator:
    def __init__(self, axis: str, amp_deg: float, period_s: float,
                 bias_deg: float, other_deg: float) -> None:
        self.axis = axis
        self.amp = amp_deg
        self.period_s = period_s
        self.bias = bias_deg
        self.other = other_deg
        self.omega = 2.0 * math.pi / period_s if period_s > 0 else 0.0

    def eval(self, t_s: float) -> Tuple[float, float, float]:
        sin_val = math.sin(self.omega * t_s)
        cos_val = math.cos(self.omega * t_s)
        if self.axis == "yaw":
            yaw = self.bias + self.amp * sin_val
            pitch = self.other
            yaw_rate = self.amp * self.omega * cos_val
        else:
            yaw = self.other
            pitch = self.bias + self.amp * sin_val
            yaw_rate = 0.0
        return (clamp_target_yaw(yaw), clamp_target_pitch(pitch),
                clamp(yaw_rate, -YAW_RATE_LIMIT_DPS, YAW_RATE_LIMIT_DPS))


# ── Keyboard reader ─────────────────────────────────────────────────────────

class KeyReader:
    """Non-blocking keyboard input. Returns None when no key is available."""

    def __init__(self) -> None:
        self._impl = self._detect()

    @staticmethod
    def _detect() -> Optional[Callable[[], Optional[str]]]:
        if os.name == "nt":
            return KeyReader._read_windows
        try:
            import termios
            import tty
            import select
            # Test if stdin is a tty
            if sys.stdin.isatty():
                return KeyReader._read_unix
        except ImportError:
            pass
        return None

    def get(self) -> Optional[str]:
        """Return a key label or None.

        Labels: 'f1','f5','escape','space','enter','backspace',
                printable chars, or None when no input.
        """
        if self._impl is None:
            return None
        try:
            return self._impl()
        except Exception:
            return None

    @staticmethod
    def _read_windows() -> Optional[str]:
        import msvcrt
        if not msvcrt.kbhit():
            return None
        ch = msvcrt.getch()
        if ch == b'\x1b':
            return 'escape'
        if ch == b'\x00' or ch == b'\xe0':
            ch2 = msvcrt.getch()
            mapping = {b';': 'f1', b'<': 'f2', b'=': 'f3', b'>': 'f4',
                       b'?': 'f5', b'@': 'f6', b'A': 'f7', b'B': 'f8',
                       b'C': 'f9', b'D': 'f10', b'H': 'up', b'P': 'down'}
            return mapping.get(ch2)
        if ch == b'\r':
            return 'enter'
        if ch == b'\x08':
            return 'backspace'
        try:
            return ch.decode('utf-8', errors='replace')
        except Exception:
            return None

    @staticmethod
    def _read_unix() -> Optional[str]:
        import select
        import termios
        import tty
        if not select.select([sys.stdin], [], [], 0)[0]:
            return None
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.buffer.read(1)
            if not ch:
                return None
            if ch == b'\x1b':
                # Check for escape sequence within 50ms
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    ch2 = sys.stdin.buffer.read(1)
                    if ch2 == b'[' or ch2 == b'O':
                        ch3 = sys.stdin.buffer.read(1)
                        mapping = {b'A':'up', b'B':'down', b'C':'right', b'D':'left',
                                   b'1':'f1', b'15':'f5'}
                        if ch2 == b'O':
                            return mapping.get(ch3)
                        if ch3 == b'~':
                            return None
                        return mapping.get(ch3)
                    if ch2 == b'\x1b':
                        return 'escape'
                    return None
                return 'escape'
            if ch == b'\r':
                return 'enter'
            if ch == b'\x7f':
                return 'backspace'
            return ch.decode('utf-8', errors='replace')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ── Interactive defaults ────────────────────────────────────────────────────

@dataclass
class InteractiveDefaults:
    home_yaw: float = HOME_YAW_DEG
    home_pitch: float = HOME_PITCH_DEG
    step_yaw: float = 0.0
    step_pitch: float = 10.0
    step_hold_s: float = 3.0
    sine_axis: str = "yaw"
    sine_amp: float = 10.0
    sine_period_s: float = 4.0
    sine_bias: float = 0.0
    sine_other: float = 10.0
    sine_duration_s: float = 10.0
    rate_hz: float = 100.0


# ── Interactive mode ────────────────────────────────────────────────────────

def _send_aim(link: SerialLink, yaw: float, pitch: float, yaw_rate: float,
              dry_run: bool, log_msg: str) -> None:
    ts = int(time.monotonic() * 1000.0) & 0xFFFFFFFF
    frame = build_frame(CMD_AUTO_AIM_ABS, pack_aim_abs(yaw, pitch, yaw_rate, ts))
    if dry_run:
        print(f"[dry-run] {log_msg}: {frame.hex(' ')}")
    else:
        link.send(frame)




def _parse_simple_args(tokens: List[str]) -> Dict[str, str]:
    """Parse key=value or --key value pairs into a dict."""
    kv: Dict[str, str] = {}
    i = 0
    while i < len(tokens):
        t = tokens[i]
        if t.startswith("--"):
            key = t[2:]
            if "=" in key:
                k, v = key.split("=", 1)
                kv[k.replace("-", "_")] = v
            elif i + 1 < len(tokens) and not tokens[i + 1].startswith("--"):
                kv[key.replace("-", "_")] = tokens[i + 1]
                i += 1
            else:
                kv[key.replace("-", "_")] = "true"
        elif "=" in t:
            k, v = t.split("=", 1)
            kv[k.replace("-", "_")] = v
        else:
            kv[t] = "true"
        i += 1
    return kv


def _parse_float(kv: Dict[str, str], key: str, default: float) -> float:
    v = kv.get(key)
    if v is None:
        return default
    try:
        return float(v)
    except ValueError:
        return default


def _parse_str(kv: Dict[str, str], key: str, default: str) -> str:
    return kv.get(key, default)


def interactive_mode(link: SerialLink, defaults: InteractiveDefaults,
                     dry_run: bool, log_csv: Optional[str],
                     no_feedback: bool, use_color: bool = True) -> int:
    key_reader = KeyReader()
    display = TerminalDisplay(use_color=use_color)
    has_kbd = key_reader._impl is not None
    has_serial = not dry_run

    # Excitation state
    generator: Optional[object] = None
    gen_t0: float = 0.0
    gen_duration: float = 0.0
    gen_label: str = ""
    t_s = 0.0

    # Feedback
    parser = FeedbackParser()
    last_fb: Optional[StatusFeedback] = None

    # Command line buffer
    cmd_buf: str = ""

    # Connection state
    connected = link.is_connected if has_serial else False
    next_scan = 0.0
    scan_interval = 1.0

    csv_file = None
    csv_writer = None
    if log_csv:
        csv_file = open(log_csv, "w", newline="", encoding="utf-8")
        fieldnames = ["t_s", "cmd_yaw", "cmd_pitch", "fb_yaw", "fb_pitch", "fb_roll", "fb_ts", "fb_mode"]
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames, extrasaction="ignore")
        csv_writer.writeheader()

    def stop_excitation() -> None:
        nonlocal generator, gen_label
        if generator is not None:
            if connected:
                link.send(build_frame(CMD_ENTER_LOCK))
            gen_label = ""
        generator = None

    def try_connect() -> bool:
        nonlocal connected
        if connected and not link.is_connected:
            connected = False
        if not connected:
            if link.open(refresh=True):
                connected = True
                link.send(build_frame(CMD_ENABLE_TX))
                return True
        return connected

    def dispatch_excitation(cmd: str, kv: Dict[str, str]) -> None:
        nonlocal generator, gen_t0, gen_duration, gen_label, t_s
        stop_excitation()
        if cmd in ("home", "hold"):
            yaw = _parse_float(kv, "yaw", defaults.home_yaw)
            pitch = _parse_float(kv, "pitch", defaults.home_pitch)
            generator = StepGenerator(yaw, pitch)
            gen_t0 = time.monotonic()
            gen_duration = float('inf')
            gen_label = f"{cmd.upper()} yaw={yaw:.1f} pitch={pitch:.1f}"
            t_s = 0.0
        elif cmd == "step":
            yaw = _parse_float(kv, "yaw", defaults.step_yaw)
            pitch = _parse_float(kv, "pitch", defaults.step_pitch)
            hold_s = _parse_float(kv, "hold_s", defaults.step_hold_s)
            generator = StepGenerator(yaw, pitch)
            gen_t0 = time.monotonic()
            gen_duration = hold_s
            gen_label = f"STEP yaw={yaw:.1f} pitch={pitch:.1f}"
            t_s = 0.0
        elif cmd == "sine":
            axis = _parse_str(kv, "axis", defaults.sine_axis)
            amp = _parse_float(kv, "amp", defaults.sine_amp)
            period_s = _parse_float(kv, "period_s", defaults.sine_period_s)
            bias = _parse_float(kv, "bias", defaults.sine_bias)
            other = _parse_float(kv, "other", defaults.sine_other)
            duration_s = _parse_float(kv, "duration_s", defaults.sine_duration_s)
            generator = SineGenerator(axis, amp, period_s, bias, other)
            gen_t0 = time.monotonic()
            gen_duration = duration_s
            gen_label = f"SINE {axis} A={amp:.1f} T={period_s:.1f}s"
            t_s = 0.0

    def exec_command(line: str) -> None:
        nonlocal connected
        line = line.strip()
        if not line:
            return

        parts = line.split()
        cmd_raw = parts[0].lower()
        cmd = CMD_ALIASES.get(cmd_raw, cmd_raw)
        tokens = parts[1:]
        kv = _parse_simple_args(tokens)

        if cmd == "exit":
            stop_excitation()
            raise SystemExit(0)

        if cmd == "help":
            display.show_message(_help_text())
            return

        if cmd == "defaults":
            display.show_message(_defaults_text(defaults))
            return

        if cmd == "set":
            for k, v in kv.items():
                if hasattr(defaults, k):
                    try:
                        old = getattr(defaults, k)
                        if isinstance(old, float):
                            setattr(defaults, k, float(v))
                        elif isinstance(old, int):
                            setattr(defaults, k, int(v))
                        else:
                            setattr(defaults, k, v)
                    except ValueError:
                        pass
            display.show_message(_defaults_text(defaults))
            return

        if cmd in ("home", "hold", "step", "sine"):
            dispatch_excitation(cmd, kv)
            return

        if cmd == "lock":
            stop_excitation()
            if connected:
                link.send(build_frame(CMD_ENTER_LOCK))
            return

        if cmd == "unlock":
            stop_excitation()
            if connected:
                link.send(build_frame(CMD_EXIT_LOCK))
            return

        if cmd == "search":
            stop_excitation()
            if connected:
                ts = int(time.monotonic() * 1000.0) & 0xFFFFFFFF
                link.send(build_frame(CMD_ENTER_SEARCH, pack_u32_le(ts)))
            return

        if cmd == "enable":
            if connected:
                link.send(build_frame(CMD_ENABLE_TX))
            return

        display.show_message(f"Unknown: {cmd_raw}. Type 'help' for commands.")

    # ── Startup banner ──
    display.begin_frame()
    if has_serial and connected:
        print(f"  {colorize('CONNECTED', ANSI_GREEN, use_color)} {link.port}  "
              f"rate={defaults.rate_hz}Hz  F1/F5/SPC/ESC  'help' for more")
    elif has_serial:
        print(f"  {colorize('SCANNING...', ANSI_YELLOW, use_color)}  "
              f"Waiting for STM32 Virtual ComPort. Type 'help' for commands.")
    else:
        display.show_message("  DRY-RUN mode  |  F1:step F5:sine SPC:home ESC:stop  'help' for more")

    try:
        loop_period = 1.0 / max(defaults.rate_hz, 1.0)
        next_tick = time.monotonic()

        while True:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(0.005, next_tick - now))
                now = time.monotonic()
            next_tick = now + loop_period

            # ── Auto-scan for device ──
            if has_serial and not connected and now >= next_scan:
                connected = try_connect()
                if connected:
                    display.show_message(
                        colorize(f"  CONNECTED {link.port}", ANSI_GREEN, use_color))
                next_scan = now + scan_interval

            # ── Check disconnect ──
            if connected and not link.is_connected:
                connected = False
                stop_excitation()
                display.show_message(
                    colorize("  DISCONNECTED — scanning...", ANSI_RED, use_color))

            # ── Read keyboard & typed commands ──
            keys: List[str] = []
            while True:
                k = key_reader.get()
                if k is None:
                    break
                keys.append(k)

            for k in keys:
                if k == 'escape':
                    stop_excitation()
                    if connected:
                        link.send(build_frame(CMD_ENTER_LOCK))
                    cmd_buf = ""

                elif k == ' ':
                    if cmd_buf:
                        cmd_buf += ' '
                    else:
                        stop_excitation()
                        generator = StepGenerator(defaults.home_yaw, defaults.home_pitch)
                        gen_t0 = time.monotonic()
                        gen_duration = float('inf')
                        gen_label = f"HOME y={defaults.home_yaw:.1f} p={defaults.home_pitch:.1f}"
                        t_s = 0.0

                elif k == 'f1':
                    exec_command(f"step yaw={defaults.step_yaw} pitch={defaults.step_pitch} "
                                 f"hold_s={defaults.step_hold_s}")

                elif k == 'f5':
                    exec_command(f"sine axis={defaults.sine_axis} amp={defaults.sine_amp} "
                                 f"period_s={defaults.sine_period_s} bias={defaults.sine_bias} "
                                 f"other={defaults.sine_other} duration_s={defaults.sine_duration_s}")

                elif k == 'enter':
                    if cmd_buf.strip():
                        display.show_message(f"  > {cmd_buf}")
                    exec_command(cmd_buf)
                    cmd_buf = ""

                elif k == 'backspace':
                    if cmd_buf:
                        cmd_buf = cmd_buf[:-1]

                elif len(k) == 1 and k.isprintable():
                    cmd_buf += k

            # ── Run excitation step ──
            cmd_yaw, cmd_pitch, cmd_rate = 0.0, 0.0, 0.0
            if generator is not None and (connected or dry_run):
                t_s = now - gen_t0
                if t_s >= gen_duration:
                    display.show_message(f"  [done] {gen_label} ({gen_duration:.1f}s)")
                    generator = None
                else:
                    cmd_yaw, cmd_pitch, cmd_rate = generator.eval(t_s)
                    if connected:
                        _send_aim(link, cmd_yaw, cmd_pitch, cmd_rate, dry_run,
                                  f"t={t_s:.2f}s yaw={cmd_yaw:.1f} pitch={cmd_pitch:.1f}")

            # ── Read feedback ──
            if connected and not no_feedback:
                parser.feed(link.read_all())
                while parser.frames:
                    fb = parser.frames.pop(0)
                    last_fb = fb
                    if csv_writer is not None:
                        csv_writer.writerow({
                            "t_s": f"{t_s:.4f}" if generator else "0.0000",
                            "cmd_yaw": f"{cmd_yaw:.2f}",
                            "cmd_pitch": f"{cmd_pitch:.2f}",
                            "fb_yaw": f"{fb.yaw_target_deg:.2f}",
                            "fb_pitch": f"{fb.pitch_target_deg:.2f}",
                            "fb_roll": f"{fb.roll_target_deg:.2f}",
                            "fb_ts": str(fb.timestamp_ms),
                            "fb_mode": fb.mode_name,
                        })

            # ── Build display lines ──
            lines: List[str] = []

            # Line 1: connection + mode
            if has_serial:
                conn = colorize(f"CONNECTED {link.port}", ANSI_GREEN, use_color) if connected \
                    else colorize("SCANNING...", ANSI_YELLOW, use_color)
            else:
                conn = colorize("DRY-RUN", ANSI_CYAN, use_color)

            mode_str = last_fb.mode_name if last_fb else "---"

            if generator is not None:
                label = colorize(gen_label, ANSI_YELLOW, use_color)
                if gen_duration == float('inf'):
                    lines.append(f"── {conn}  MODE:{mode_str}  {label}  t={t_s:.1f}s")
                else:
                    frac = min(1.0, t_s / gen_duration) if gen_duration > 0 else 0
                    bar = make_progress_bar(frac, 8)
                    lines.append(f"── {conn}  MODE:{mode_str}  {label}  {bar} t={t_s:.1f}/{gen_duration:.1f}s")
            else:
                lines.append(f"── {conn}  MODE:{mode_str}  IDLE")

            # Line 2: CMD (sent values)
            if generator is not None:
                lines.append(f"│ CMD:  yaw={cmd_yaw:+8.2f}  pitch={cmd_pitch:+8.2f}  rate={cmd_rate:+5.0f} dps")
            else:
                lines.append(f"│ CMD:  ---")

            # Line 3: FB (feedback from gimbal)
            if last_fb is not None:
                fb_line = (f"│ FB:   yaw={last_fb.yaw_target_deg:+8.2f}  "
                           f"pitch={last_fb.pitch_target_deg:+8.2f}  "
                           f"roll={last_fb.roll_target_deg:+7.2f}  "
                           f"mode={last_fb.mode_name}")
                lines.append(colorize(fb_line, ANSI_CYAN, use_color))
            else:
                lines.append(f"│ FB:   --- (waiting for feedback)")

            # Line 4: prompt
            if cmd_buf:
                lines.append(f"└─ gimbal> {cmd_buf}_")
            else:
                lines.append(f"└─ gimbal> _")

            display.render(lines)

    except KeyboardInterrupt:
        display.show_message("")
    except SystemExit:
        display.show_message("")
    finally:
        if generator is not None and connected:
            link.send(build_frame(CMD_ENTER_LOCK))
        if csv_file:
            csv_file.close()
            display.show_message(f"  [log] saved to {log_csv}")
        display.end_frame()

    return 0


def _help_text() -> str:
    return """
Commands:
  s,  step    [yaw=] [pitch=] [hold_s=]     Step response (default y=0 p=10 t=3s)
  si, sine    [axis=] [amp=] [period_s=]    Sine excitation
                [bias=] [other=] [duration_s=]
  h,  home    [yaw=] [pitch=]               Hold at position (default y=0 p=0)
  hd, hold    yaw= pitch=                   Hold at specified position
  l,  lock                                  Lock gimbal
  u,  unlock                                Unlock gimbal
  se, search                                Enter search mode
  d,  defaults                              Show default parameters
  set key=val ...                           Update defaults (e.g. set step_yaw=15)
  q,  exit                                  Quit

Keyboard: F1=step  F5=sine  Space=home  Esc=stop+lock"""


def _defaults_text(defaults: InteractiveDefaults) -> str:
    return (f"""
  home:    yaw={defaults.home_yaw}  pitch={defaults.home_pitch}
  step:    yaw={defaults.step_yaw}  pitch={defaults.step_pitch}  hold_s={defaults.step_hold_s}
  sine:    axis={defaults.sine_axis}  amp={defaults.sine_amp}  period_s={defaults.sine_period_s}
           bias={defaults.sine_bias}  other={defaults.sine_other}  duration_s={defaults.sine_duration_s}
  rate:    {defaults.rate_hz} Hz""")


# ── Batch excitation (non-interactive) ──────────────────────────────────────

def run_excitation(link: SerialLink, generator, rate_hz: float,
                   duration_s: float, dry_run: bool, csv_writer,
                   no_feedback: bool) -> None:
    dt = 1.0 / rate_hz if rate_hz > 0 else 0.01
    n_steps = max(1, int(math.ceil(duration_s / dt)))
    parser = FeedbackParser()

    for i in range(n_steps):
        loop_start = time.monotonic()
        t_s = i * dt

        yaw, pitch, yaw_rate_f = generator.eval(t_s)
        yaw_rate = int(round(yaw_rate_f))
        ts = int(time.monotonic() * 1000.0) & 0xFFFFFFFF

        frame = build_frame(CMD_AUTO_AIM_ABS, pack_aim_abs(yaw, pitch, yaw_rate_f, ts))
        if dry_run:
            ts_str = datetime.now().strftime("%H:%M:%S")
            print(f"[{ts_str}] t={t_s:.3f}s yaw={yaw:+7.2f} pitch={pitch:+7.2f} "
                  f"rate={yaw_rate:+5d}  {frame.hex(' ')}")
        else:
            link.send(frame)

        if not no_feedback and not dry_run:
            parser.feed(link.read_all())
            while parser.frames:
                fb = parser.frames.pop(0)
                if csv_writer is not None:
                    csv_writer.writerow({
                        "t_s": f"{t_s:.4f}",
                        "cmd_yaw": f"{yaw:.2f}",
                        "cmd_pitch": f"{pitch:.2f}",
                        "fb_yaw": f"{fb.yaw_target_deg:.2f}",
                        "fb_pitch": f"{fb.pitch_target_deg:.2f}",
                        "fb_roll": f"{fb.roll_target_deg:.2f}",
                        "fb_ts": str(fb.timestamp_ms),
                        "fb_mode": fb.mode_name,
                    })

        elapsed = time.monotonic() - loop_start
        if elapsed < dt:
            time.sleep(dt - elapsed)

    if not dry_run:
        print(f"Sent {n_steps} frames at {rate_hz} Hz over {duration_s:.1f}s", file=sys.stderr)


# ── CLI ─────────────────────────────────────────────────────────────────────

def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send control commands to gimbal via USB CDC protocol",
    )
    sub = parser.add_subparsers(dest="subcommand", help="Command (default: interactive)")

    # ── step ──
    p_step = sub.add_parser("step", help="Step response excitation")
    p_step.add_argument("--yaw", type=float, default=0.0, help="Yaw target (deg, default 0)")
    p_step.add_argument("--pitch", type=float, default=10.0, help="Pitch target (deg, default 10)")
    p_step.add_argument("--hold-s", type=float, default=3.0, help="Hold duration (s, default 3)")

    # ── sine ──
    p_sine = sub.add_parser("sine", help="Sine wave excitation")
    p_sine.add_argument("--axis", choices=("yaw", "pitch"), default="yaw")
    p_sine.add_argument("--amp", type=float, default=10.0, help="Amplitude (deg, default 10)")
    p_sine.add_argument("--period-s", type=float, default=4.0, help="Period (s, default 4)")
    p_sine.add_argument("--bias", type=float, default=0.0, help="DC bias (deg, default 0)")
    p_sine.add_argument("--other", type=float, default=10.0, help="Other-axis constant (deg, default 10)")
    p_sine.add_argument("--duration-s", type=float, default=10.0, help="Duration (s, default 10)")

    # ── single-shot ──
    for name in ("enable", "lock", "unlock", "search"):
        p = sub.add_parser(name, help=f"Send {name} command")
        p.set_defaults(single_cmd=name)

    # ── interactive (explicit) ──
    p_int = sub.add_parser("interactive", help="Interactive keyboard mode (default)")
    _add_interactive_args(p_int)

    # ── common options for step/sine ──
    for p in (p_step, p_sine):
        p.add_argument("--port", default=None, help="Serial port (auto-detect)")
        p.add_argument("--baudrate", type=int, default=115200)
        p.add_argument("--dry-run", action="store_true", help="Print frames instead of sending")
        p.add_argument("--rate-hz", type=float, default=100.0, help="Send rate (Hz, default 100)")
        p.add_argument("--log-csv", default=None, help="Save feedback to CSV")
        p.add_argument("--no-feedback", action="store_true", help="Skip reading feedback")

    for p in (sub.choices["enable"], sub.choices["lock"], sub.choices["unlock"], sub.choices["search"]):
        p.add_argument("--port", default=None, help="Serial port (auto-detect)")
        p.add_argument("--dry-run", action="store_true", help="Print frames instead of sending")

    return parser.parse_args(argv)


def _add_interactive_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--port", default=None, help="Serial port (auto-detect)")
    p.add_argument("--baudrate", type=int, default=115200)
    p.add_argument("--dry-run", action="store_true", help="Print frames instead of sending")
    p.add_argument("--rate-hz", type=float, default=100.0, help="Send rate (Hz, default 100)")
    p.add_argument("--log-csv", default=None, help="Save feedback to CSV")
    p.add_argument("--no-feedback", action="store_true", help="Skip reading feedback")
    p.add_argument("--no-color", action="store_true", help="Disable ANSI colors")
    p.add_argument("--home-yaw", type=float, default=HOME_YAW_DEG, help="Home yaw (deg)")
    p.add_argument("--home-pitch", type=float, default=HOME_PITCH_DEG, help="Home pitch (deg)")
    p.add_argument("--step-yaw", type=float, default=0.0, help="Default step yaw (deg)")
    p.add_argument("--step-pitch", type=float, default=10.0, help="Default step pitch (deg)")
    p.add_argument("--step-hold-s", type=float, default=3.0, help="Default step hold (s)")
    p.add_argument("--sine-axis", choices=("yaw", "pitch"), default="yaw")
    p.add_argument("--sine-amp", type=float, default=10.0, help="Default sine amplitude (deg)")
    p.add_argument("--sine-period-s", type=float, default=4.0, help="Default sine period (s)")
    p.add_argument("--sine-bias", type=float, default=0.0, help="Default sine bias (deg)")
    p.add_argument("--sine-other", type=float, default=10.0, help="Default sine other-axis (deg)")
    p.add_argument("--sine-duration-s", type=float, default=10.0, help="Default sine duration (s)")


# ── Single-shot commands ────────────────────────────────────────────────────

def send_single(dry_run: bool, **kwargs) -> None:
    link: Optional[SerialLink] = None
    if not dry_run:
        cfg = SerialConfig(port=kwargs.get("port"))
        link = SerialLink(cfg)
        if not link.open():
            print("Error: could not open serial port", file=sys.stderr)
            sys.exit(1)
        print(f"[serial] {link.port}", file=sys.stderr)

    def do_send(frame: bytes, desc: str) -> None:
        if dry_run:
            print(f"[dry-run] {desc}: {frame.hex(' ')}")
        else:
            assert link is not None
            link.send(frame)
            print(f"[ok] {desc}", file=sys.stderr)

    do_send(build_frame(CMD_ENABLE_TX), "enable TX")
    cmd = kwargs.get("cmd", "")
    if cmd == "search":
        ts = int(time.monotonic() * 1000.0) & 0xFFFFFFFF
        do_send(build_frame(CMD_ENTER_SEARCH, pack_u32_le(ts)), "enter search")
    elif cmd == "lock":
        do_send(build_frame(CMD_ENTER_LOCK), "enter lock")
    elif cmd == "unlock":
        do_send(build_frame(CMD_EXIT_LOCK), "exit lock")
    if link:
        link.close()


# ── Main ────────────────────────────────────────────────────────────────────

def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)

    # ── No subcommand → interactive mode ──
    if args.subcommand is None:
        cfg = SerialConfig(port=None, baudrate=115200)
        link = SerialLink(cfg)
        link.open()  # best-effort; interactive mode handles scanning
        if link.is_open:
            link.send(build_frame(CMD_ENABLE_TX))

        defaults = InteractiveDefaults()
        try:
            return interactive_mode(link, defaults, dry_run=False,
                                    log_csv=None, no_feedback=False, use_color=True)
        finally:
            link.close()

    # ── interactive (explicit) ──
    if args.subcommand == "interactive":
        cfg = SerialConfig(port=args.port, baudrate=args.baudrate)
        link = SerialLink(cfg)
        dry_run = args.dry_run
        link.open()  # best-effort; interactive mode handles scanning
        if link.is_open:
            link.send(build_frame(CMD_ENABLE_TX))

        defaults = InteractiveDefaults(
            home_yaw=args.home_yaw, home_pitch=args.home_pitch,
            step_yaw=args.step_yaw, step_pitch=args.step_pitch,
            step_hold_s=args.step_hold_s,
            sine_axis=args.sine_axis, sine_amp=args.sine_amp,
            sine_period_s=args.sine_period_s, sine_bias=args.sine_bias,
            sine_other=args.sine_other, sine_duration_s=args.sine_duration_s,
            rate_hz=args.rate_hz,
        )
        use_color = not args.no_color
        try:
            return interactive_mode(link, defaults, dry_run, args.log_csv,
                                    args.no_feedback, use_color)
        finally:
            link.close()

    # ── Single-shot commands ──
    cmd = getattr(args, "single_cmd", None)
    if cmd is not None:
        send_single(dry_run=args.dry_run, port=args.port, cmd=cmd)
        return 0

    # ── Batch excitation (step / sine) ──
    cfg = SerialConfig(port=args.port, baudrate=args.baudrate)
    link = SerialLink(cfg)
    dry_run = args.dry_run

    if not dry_run:
        if not link.open():
            print("Error: could not open serial port", file=sys.stderr)
            return 1
        print(f"[serial] {link.port}", file=sys.stderr)

    csv_file = None
    csv_writer = None

    try:
        enable_frame = build_frame(CMD_ENABLE_TX)
        if dry_run:
            print(f"[dry-run] enable TX: {enable_frame.hex(' ')}")
        else:
            link.send(enable_frame)
            time.sleep(0.05)

        if args.subcommand == "step":
            generator = StepGenerator(yaw_deg=args.yaw, pitch_deg=args.pitch)
            duration_s = args.hold_s
        elif args.subcommand == "sine":
            generator = SineGenerator(axis=args.axis, amp_deg=args.amp, period_s=args.period_s,
                                      bias_deg=args.bias, other_deg=args.other)
            duration_s = args.duration_s
        else:
            return 1

        if args.log_csv:
            csv_file = open(args.log_csv, "w", newline="", encoding="utf-8")
            fieldnames = ["t_s", "cmd_yaw", "cmd_pitch", "fb_yaw", "fb_pitch", "fb_roll", "fb_ts", "fb_mode"]
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames, extrasaction="ignore")
            csv_writer.writeheader()

        run_excitation(link=link, generator=generator, rate_hz=args.rate_hz,
                       duration_s=duration_s, dry_run=dry_run, csv_writer=csv_writer,
                       no_feedback=args.no_feedback)

    except KeyboardInterrupt:
        print("\nInterrupted", file=sys.stderr)
    finally:
        if csv_file:
            csv_file.close()
            print(f"[log] saved to {args.log_csv}", file=sys.stderr)
        link.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
