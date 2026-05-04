#!/usr/bin/env python3
"""
Run gimbal tune tests through SEGGER RTT.

Dependencies:
    pip install -r tools/requirements-rtt.txt
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple


SAMPLE_HEADER = [
    "kind",
    "tick_ms",
    "mode",
    "axis",
    "target_yaw_deg",
    "ref_yaw_deg",
    "meas_yaw_deg",
    "target_pitch_deg",
    "ref_pitch_deg",
    "meas_pitch_deg",
    "yaw_speed_target_dps",
    "yaw_meas_speed_dps",
    "pitch_speed_target_dps",
    "pitch_meas_speed_dps",
    "yaw_current_pid",
    "yaw_current_ff",
    "yaw_cable_ff",
    "yaw_current_cmd",
    "yaw_current_meas",
    "pitch_current_pid",
    "pitch_current_ff",
    "pitch_cable_ff",
    "pitch_current_cmd",
    "pitch_current_meas",
]


@dataclass
class RttConfig:
    device: str = "STM32F405RG"
    serial_no: Optional[int] = None
    speed_khz: int = 4000
    up_buffer: int = 0
    down_buffer: int = 0
    read_size: int = 1024
    startup_timeout_s: float = 5.0
    poll_interval_s: float = 0.02


@dataclass
class RunConfig:
    out_root: str = os.path.join("tools", "out", "tune")
    timeout_s: float = 10.0
    sample_rate_ms: int = 10
    preset: Optional[str] = None
    script: Optional[str] = None
    amp: Optional[float] = None
    hold_ms: Optional[int] = None
    duration_s: float = 10.0


@dataclass
class Capture:
    out_dir: str
    comments: List[str] = field(default_factory=list)
    events: List[str] = field(default_factory=list)
    header: List[str] = field(default_factory=list)
    samples: List[Dict[str, str]] = field(default_factory=list)


class RttClient:
    def __init__(self, config: RttConfig) -> None:
        self.config = config
        self._jlink = None
        self._pylink = None
        self._rx_buffer = bytearray()

    def _load_pylink(self):
        try:
            import pylink  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "pylink-square is required. Install it with: pip install -r tools/requirements-rtt.txt"
            ) from exc
        return pylink

    def open(self) -> None:
        pylink = self._load_pylink()
        self._pylink = pylink
        self._jlink = pylink.JLink()

        if self.config.serial_no is None:
            self._jlink.open()
        else:
            self._jlink.open(self.config.serial_no)

        self._jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
        self._jlink.connect(self.config.device, speed=self.config.speed_khz)
        try:
            if self._jlink.halted():
                self._jlink.restart()
        except Exception:
            pass

        self._jlink.rtt_start()
        self.wait_until_ready(self.config.startup_timeout_s)

    def close(self) -> None:
        if self._jlink is None:
            return

        try:
            try:
                self._jlink.rtt_stop()
            except Exception:
                pass
        finally:
            self._jlink.close()
            self._jlink = None

    def write_line(self, line: str) -> None:
        if self._jlink is None:
            raise RuntimeError("RTT client is not open")

        payload = line.rstrip("\n").encode("utf-8") + b"\n"
        deadline = time.monotonic() + self.config.startup_timeout_s
        offset = 0
        # Keep writes conservative: old firmware used a 16-byte RTT down-buffer
        # with about 15 usable bytes, and some probes report success even when
        # the target consumes slowly. Byte-sized writes are slower but robust.
        max_chunk = 1

        while offset < len(payload):
            chunk = payload[offset : offset + max_chunk]
            written = self._jlink.rtt_write(self.config.down_buffer, chunk)
            if written > 0:
                offset += written

            if offset == len(payload):
                return

            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"failed to write complete RTT command: {offset}/{len(payload)} bytes "
                    f"(last_write={written})"
                )

            time.sleep(min(self.config.poll_interval_s, 0.005))

    def read_bytes(self) -> bytes:
        if self._jlink is None:
            raise RuntimeError("RTT client is not open")

        return bytes(self._jlink.rtt_read(self.config.up_buffer, self.config.read_size))

    def read_lines(self) -> List[str]:
        lines: List[str] = []
        chunk = self.read_bytes()
        if chunk:
            self._rx_buffer.extend(chunk)

        while True:
            try:
                newline = self._rx_buffer.index(0x0A)
            except ValueError:
                break
            raw_line = bytes(self._rx_buffer[:newline])
            del self._rx_buffer[: newline + 1]
            lines.append(raw_line.decode("utf-8", errors="replace").strip())

        return lines

    def pop_partial_line(self) -> Optional[str]:
        if not self._rx_buffer:
            return None

        raw_line = bytes(self._rx_buffer)
        self._rx_buffer.clear()
        return raw_line.decode("utf-8", errors="replace").strip()

    def wait_until_ready(self, timeout_s: float) -> None:
        if self._jlink is None:
            raise RuntimeError("RTT client is not open")

        deadline = time.monotonic() + timeout_s
        last_error = "RTT control block not ready"

        while time.monotonic() < deadline:
            try:
                up_buffers = self._jlink.rtt_get_num_up_buffers()
                down_buffers = self._jlink.rtt_get_num_down_buffers()
                if up_buffers > self.config.up_buffer and down_buffers > self.config.down_buffer:
                    return
                last_error = (
                    f"RTT buffers not ready: up={up_buffers}, down={down_buffers}, "
                    f"need up>{self.config.up_buffer}, down>{self.config.down_buffer}"
                )
            except Exception as exc:
                last_error = str(exc)

            time.sleep(self.config.poll_interval_s)

        raise RuntimeError(f"timed out waiting for RTT ready: {last_error}")


def split_csv_line(line: str) -> List[str]:
    return [part.strip() for part in line.split(",")]


def make_out_dir(root: str) -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.abspath(os.path.join(root, stamp))
    os.makedirs(out_dir, exist_ok=True)
    return out_dir


def preset_commands(name: str, config: RunConfig) -> Tuple[List[str], float]:
    amp = config.amp
    hold_ms = config.hold_ms

    if name == "speed-yaw":
        return [f"tune speed_step axis=yaw amp={amp if amp is not None else 40} hold_ms={hold_ms or 1500} other=10"], config.duration_s
    if name == "speed-pitch":
        return [f"tune speed_step axis=pitch amp={amp if amp is not None else 30} hold_ms={hold_ms or 1500} other=0"], config.duration_s
    if name == "angle-yaw":
        return [f"tune step axis=yaw amp={amp if amp is not None else 5} hold_ms={hold_ms or 1500} bias=0 other=10 planner=0"], config.duration_s
    if name == "angle-pitch":
        return [f"tune step axis=pitch amp={amp if amp is not None else 5} hold_ms={hold_ms or 1500} bias=10 other=0 planner=0"], config.duration_s
    if name == "hold-pitch":
        return ["tune hold yaw=0 pitch=10 planner=1"], config.duration_s
    if name == "planner-yaw":
        return [f"tune step axis=yaw amp={amp if amp is not None else 30} hold_ms={hold_ms or 2500} bias=0 other=10 planner=1"], config.duration_s
    if name == "planner-pitch":
        return [f"tune step axis=pitch amp={amp if amp is not None else 15} hold_ms={hold_ms or 2500} bias=15 other=0 planner=1"], config.duration_s
    if name == "sine-yaw":
        return [f"tune sine axis=yaw amp={amp if amp is not None else 20} freq=0.25 bias=0 other=10 planner=0"], config.duration_s
    if name == "sine-pitch":
        return [f"tune sine axis=pitch amp={amp if amp is not None else 10} freq=0.2 bias=15 other=0 planner=0"], config.duration_s
    if name == "all-basic":
        return [
            "tune hold yaw=0 pitch=10 planner=1",
            "tune step axis=yaw amp=5 hold_ms=1500 bias=0 other=10 planner=0",
            "tune step axis=pitch amp=5 hold_ms=1500 bias=10 other=0 planner=0",
            "tune speed_step axis=yaw amp=30 hold_ms=1500 other=10",
        ], max(config.duration_s, 32.0)

    raise ValueError(f"unknown preset: {name}")


def load_script(path: str) -> Tuple[List[str], float]:
    with open(path, "r", encoding="utf-8") as fp:
        data = json.load(fp)

    commands = data.get("commands")
    if not isinstance(commands, list) or not all(isinstance(item, str) for item in commands):
        raise ValueError("script JSON must contain a string list field named 'commands'")

    duration_s = float(data.get("duration_s", 10.0))
    return commands, duration_s


def process_line(line: str, capture: Capture) -> None:
    if not line:
        return

    if line.startswith("#"):
        capture.events.append(line)
        print(line)
        return

    parts = split_csv_line(line)
    if parts and parts[0] == "kind":
        capture.header = parts
        return

    header = capture.header or SAMPLE_HEADER
    if parts and parts[0] == "sample" and len(parts) == len(header):
        capture.samples.append(dict(zip(header, parts)))
        return

    capture.comments.append(line)


def drain_for(client: RttClient, capture: Capture, duration_s: float, poll_interval_s: float) -> None:
    deadline = time.monotonic() + duration_s

    while time.monotonic() < deadline:
        lines = client.read_lines()
        if lines:
            for line in lines:
                process_line(line, capture)
        else:
            time.sleep(poll_interval_s)


def ack_prefix_for(command: str) -> Optional[str]:
    tokens = command.strip().split()
    if len(tokens) >= 2 and tokens[0] == "tune":
        return f"#ACK tune {tokens[1]}"
    if len(tokens) >= 2 and tokens[0] == "ff":
        return f"#ACK ff {tokens[1]}"
    return None


def wait_for_ack(
    client: RttClient,
    capture: Capture,
    command: str,
    timeout_s: float,
    poll_interval_s: float,
) -> None:
    expected = ack_prefix_for(command)
    if expected is None:
        drain_for(client, capture, timeout_s, poll_interval_s)
        return

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        lines = client.read_lines()
        if lines:
            for line in lines:
                process_line(line, capture)
                if line.startswith("#ERR"):
                    raise RuntimeError(f"firmware rejected command '{command}': {line}")
                if line.startswith(expected):
                    return
        else:
            time.sleep(poll_interval_s)

    partial = client.pop_partial_line()
    if partial:
        process_line(partial, capture)
    raise RuntimeError(f"timed out waiting for {expected} after command: {command}")


def write_capture(capture: Capture, session: Dict[str, object]) -> None:
    raw_path = os.path.join(capture.out_dir, "raw_samples.csv")
    events_path = os.path.join(capture.out_dir, "events.log")
    session_path = os.path.join(capture.out_dir, "session.json")
    metrics_path = os.path.join(capture.out_dir, "metrics.csv")
    summary_path = os.path.join(capture.out_dir, "summary.md")

    header = capture.header or SAMPLE_HEADER
    with open(raw_path, "w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=header)
        writer.writeheader()
        for row in capture.samples:
            writer.writerow({key: row.get(key, "") for key in header})

    with open(events_path, "w", encoding="utf-8") as fp:
        for line in capture.events + capture.comments:
            fp.write(line + "\n")

    metrics = build_metrics(capture.samples)
    with open(metrics_path, "w", newline="", encoding="utf-8") as fp:
        fieldnames = ["metric", "value"]
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for key, value in metrics.items():
            writer.writerow({"metric": key, "value": value})

    session = dict(session)
    session["sample_count"] = len(capture.samples)
    session["metrics"] = metrics
    with open(session_path, "w", encoding="utf-8") as fp:
        json.dump(session, fp, indent=2, ensure_ascii=False)

    with open(summary_path, "w", encoding="utf-8") as fp:
        fp.write("# Tune RTT Summary\n\n")
        fp.write(f"- samples: {len(capture.samples)}\n")
        for key, value in metrics.items():
            fp.write(f"- {key}: {value}\n")


def float_values(rows: Iterable[Dict[str, str]], key: str) -> List[float]:
    values: List[float] = []
    for row in rows:
        try:
            values.append(float(row.get(key, "")))
        except ValueError:
            pass
    return values


def max_abs(values: Sequence[float]) -> float:
    return max((abs(value) for value in values), default=0.0)


def mean_abs(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return sum(abs(value) for value in values) / float(len(values))


def rms(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / float(len(values)))


def build_metrics(rows: List[Dict[str, str]]) -> Dict[str, str]:
    metrics: Dict[str, str] = {"sample_count": str(len(rows))}
    if not rows:
        return metrics

    yaw_error = []
    pitch_error = []
    for row in rows:
        try:
            yaw_error.append(float(row["target_yaw_deg"]) - float(row["meas_yaw_deg"]))
            pitch_error.append(float(row["target_pitch_deg"]) - float(row["meas_pitch_deg"]))
        except (KeyError, ValueError):
            pass

    yaw_cmd = float_values(rows, "yaw_current_cmd")
    pitch_cmd = float_values(rows, "pitch_current_cmd")
    yaw_pid = float_values(rows, "yaw_current_pid")
    pitch_pid = float_values(rows, "pitch_current_pid")

    metrics.update(
        {
            "yaw_error_rms_deg": f"{rms(yaw_error):.6f}",
            "yaw_error_max_abs_deg": f"{max_abs(yaw_error):.6f}",
            "pitch_error_rms_deg": f"{rms(pitch_error):.6f}",
            "pitch_error_max_abs_deg": f"{max_abs(pitch_error):.6f}",
            "yaw_current_cmd_peak": f"{max_abs(yaw_cmd):.6f}",
            "pitch_current_cmd_peak": f"{max_abs(pitch_cmd):.6f}",
            "yaw_current_pid_mean_abs": f"{mean_abs(yaw_pid):.6f}",
            "pitch_current_pid_mean_abs": f"{mean_abs(pitch_pid):.6f}",
        }
    )
    return metrics


def read_samples_csv(path: str) -> List[Dict[str, str]]:
    with open(path, newline="", encoding="utf-8") as fp:
        return list(csv.DictReader(fp))


def row_float(row: Dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, ""))
    except ValueError:
        return default


def row_tick_ms(row: Dict[str, str]) -> int:
    try:
        return int(float(row.get("tick_ms", "0")))
    except ValueError:
        return 0


def split_target_segments(rows: List[Dict[str, str]], target_key: str) -> List[Dict[str, object]]:
    segments: List[Dict[str, object]] = []
    current: Optional[Dict[str, object]] = None

    for row in rows:
        target = row_float(row, target_key)
        tick_ms = row_tick_ms(row)
        if current is None or abs(target - float(current["target"])) > 0.5:
            if current is not None:
                segments.append(current)
            current = {
                "target": target,
                "start_ms": tick_ms,
                "end_ms": tick_ms,
                "rows": [],
            }

        current["end_ms"] = tick_ms
        current_rows = current["rows"]
        assert isinstance(current_rows, list)
        current_rows.append(row)

    if current is not None:
        segments.append(current)

    return segments


def analyze_speed_step(rows: List[Dict[str, str]], axis: str) -> Tuple[List[str], List[str]]:
    lines: List[str] = []
    advice: List[str] = []
    target_key = f"{axis}_speed_target_dps"
    meas_key = f"{axis}_meas_speed_dps"
    cmd_key = f"{axis}_current_cmd"
    pid_key = f"{axis}_current_pid"
    ff_key = f"{axis}_current_ff"
    current_meas_key = f"{axis}_current_meas"

    active_rows = [row for row in rows if row.get("mode") in ("speed_hold", "speed_step", "speed_sine")]
    if not active_rows:
        return ["No active speed samples found."], ["先确认 raw_samples.csv 里 mode 是否进入 speed_step。"]

    segments = split_target_segments(active_rows, target_key)
    nonzero_segments = [segment for segment in segments if abs(float(segment["target"])) > 1.0]

    lines.append(f"samples: total={len(rows)}, active_speed={len(active_rows)}")
    lines.append(f"segments: {len(segments)}, nonzero_segments={len(nonzero_segments)}")
    lines.append(
        "ranges: "
        f"target=[{min(float_values(active_rows, target_key)):.1f}, {max(float_values(active_rows, target_key)):.1f}] dps, "
        f"meas=[{min(float_values(active_rows, meas_key)):.1f}, {max(float_values(active_rows, meas_key)):.1f}] dps, "
        f"cmd_peak={max_abs(float_values(active_rows, cmd_key)):.0f}, "
        f"pid_peak={max_abs(float_values(active_rows, pid_key)):.0f}, "
        f"ff_peak={max_abs(float_values(active_rows, ff_key)):.0f}, "
        f"current_meas_peak={max_abs(float_values(active_rows, current_meas_key)):.0f}"
    )

    overshoot_values: List[float] = []
    steady_error_values: List[float] = []
    response_times: List[float] = []
    positive_tail: List[float] = []
    negative_tail: List[float] = []

    lines.append("")
    lines.append("speed_step segments:")
    for segment in segments:
        target = float(segment["target"])
        segment_rows = segment["rows"]
        assert isinstance(segment_rows, list)
        if not segment_rows:
            continue

        meas_values = [row_float(row, meas_key) for row in segment_rows]
        cmd_values = [row_float(row, cmd_key) for row in segment_rows]
        tail_count = max(5, len(meas_values) // 5)
        tail_mean = sum(meas_values[-tail_count:]) / float(tail_count)
        duration_ms = int(segment["end_ms"]) - int(segment["start_ms"])
        cmd_peak = max_abs(cmd_values)

        response_ms: Optional[int] = None
        overshoot_pct = 0.0
        if target > 1.0:
            peak = max(meas_values)
            overshoot_pct = max(0.0, (peak - target) / abs(target) * 100.0)
            positive_tail.append(tail_mean)
            for row in segment_rows:
                if row_float(row, meas_key) >= target * 0.9:
                    response_ms = row_tick_ms(row) - int(segment["start_ms"])
                    break
        elif target < -1.0:
            peak = min(meas_values)
            overshoot_pct = max(0.0, (target - peak) / abs(target) * 100.0)
            negative_tail.append(tail_mean)
            for row in segment_rows:
                if row_float(row, meas_key) <= target * 0.9:
                    response_ms = row_tick_ms(row) - int(segment["start_ms"])
                    break
        else:
            peak = max_abs(meas_values)

        if abs(target) > 1.0:
            steady_error_pct = abs(target - tail_mean) / abs(target) * 100.0
            overshoot_values.append(overshoot_pct)
            steady_error_values.append(steady_error_pct)
            if response_ms is not None:
                response_times.append(float(response_ms))
            response_text = "n/a" if response_ms is None else f"{response_ms}ms"
            lines.append(
                f"- target={target:+.1f} dps, duration={duration_ms}ms, "
                f"tail_meas={tail_mean:+.1f} dps, steady_error={steady_error_pct:.1f}%, "
                f"overshoot={overshoot_pct:.1f}%, response90={response_text}, cmd_peak={cmd_peak:.0f}"
            )
        else:
            lines.append(
                f"- target={target:+.1f} dps, duration={duration_ms}ms, "
                f"tail_meas={tail_mean:+.1f} dps, stop_peak={peak:.1f} dps, cmd_peak={cmd_peak:.0f}"
            )

    max_overshoot = max(overshoot_values, default=0.0)
    mean_steady_error = sum(steady_error_values) / float(len(steady_error_values)) if steady_error_values else 0.0
    mean_response = sum(response_times) / float(len(response_times)) if response_times else 0.0
    cmd_peak_all = max_abs(float_values(active_rows, cmd_key))

    lines.append("")
    lines.append(
        f"summary: max_overshoot={max_overshoot:.1f}%, "
        f"mean_steady_error={mean_steady_error:.1f}%, "
        f"mean_response90={mean_response:.0f}ms, cmd_peak={cmd_peak_all:.0f}"
    )

    if cmd_peak_all > 8000.0:
        advice.append("电流命令峰值已经偏高，先降幅值或降低速度环输出，别继续加 P/I。")
    elif cmd_peak_all > 5000.0:
        advice.append("电流命令峰值有点高，后续每次只小步改参数，并观察电机温度和机械限位。")
    else:
        advice.append("电流峰值目前不算夸张，可以继续做小步参数试验。")

    if max_overshoot > 40.0:
        advice.append(
            f"{axis}_speed 过冲很大，先不要加 Ki；建议先把 {axis}_k_vel_ff 降 15%~25%，复测。"
        )
        advice.append(
            f"如果降前馈后过冲仍超过 30%，再把 {axis}_speed_kp 降 10%~20%；需要更快刹住时再少量加 {axis}_speed_kd。"
        )
    elif mean_steady_error > 25.0:
        advice.append(
            f"稳态速度误差偏大但过冲不夸张时，优先小幅增加 {axis}_k_vel_ff 或 {axis}_speed_kp，每次 5%~10%。"
        )
        advice.append(f"只有在 P/前馈调顺后仍有固定残差，才少量增加 {axis}_speed_ki。")
    elif mean_response > 500.0:
        advice.append(f"响应偏慢，可以小幅增加 {axis}_speed_kp，每次 5%~10%，复测过冲。")
    else:
        advice.append("速度阶跃形态基本可用，下一步可以进入角度环阶跃测试。")

    if positive_tail and negative_tail:
        pos_mean = sum(positive_tail) / float(len(positive_tail))
        neg_mean = sum(negative_tail) / float(len(negative_tail))
        if abs(abs(pos_mean) - abs(neg_mean)) > 0.25 * max(abs(pos_mean), abs(neg_mean), 1.0):
            advice.append("正反向响应不对称明显，注意检查重心、摩擦、线束拖拽或电机方向补偿，别只靠 Ki 硬补。")

    advice.append(
        f"下一次建议命令：python tools\\tune_rtt.py send \"tune set {axis}_k_vel_ff=<当前值*0.8>\"，"
        f"然后重跑 speed-{axis}。"
    )

    return lines, advice


def infer_axis_from_samples(rows: List[Dict[str, str]]) -> str:
    yaw_target = max_abs(float_values(rows, "yaw_speed_target_dps"))
    pitch_target = max_abs(float_values(rows, "pitch_speed_target_dps"))
    return "pitch" if pitch_target > yaw_target else "yaw"


def infer_axis_from_session(out_dir: str, rows: List[Dict[str, str]]) -> str:
    session_path = os.path.join(out_dir, "session.json")
    if os.path.exists(session_path):
        try:
            with open(session_path, encoding="utf-8") as fp:
                session = json.load(fp)
            preset = str(session.get("preset", ""))
            if preset.endswith("-yaw"):
                return "yaw"
            if preset.endswith("-pitch"):
                return "pitch"
            for command in session.get("commands", []):
                command_text = str(command)
                if "axis=yaw" in command_text:
                    return "yaw"
                if "axis=pitch" in command_text:
                    return "pitch"
        except (OSError, ValueError, TypeError):
            pass

    return infer_axis_from_samples(rows)


def command_analyze(out_dir: str) -> None:
    raw_path = os.path.join(out_dir, "raw_samples.csv")
    if not os.path.exists(raw_path):
        raise RuntimeError(f"raw_samples.csv not found: {raw_path}")

    rows = read_samples_csv(raw_path)
    if not rows:
        print("No samples found.")
        return

    axis = infer_axis_from_session(out_dir, rows)
    modes = sorted({row.get("mode", "") for row in rows if row.get("mode", "")})
    print(f"analyze: {out_dir}")
    print(f"modes: {', '.join(modes)}")
    print(f"axis: {axis}")
    print("")

    if any(mode.startswith("speed") for mode in modes):
        lines, advice = analyze_speed_step(rows, axis)
    else:
        lines = [
            f"samples: {len(rows)}",
            "This analyzer currently gives detailed recommendations for speed presets first.",
        ]
        advice = ["角度环建议先确认速度环调顺，再跑 angle-yaw / angle-pitch。"]

    print("Metrics")
    for line in lines:
        print(line)
    print("")
    print("Recommendation")
    for index, item in enumerate(advice, start=1):
        print(f"{index}. {item}")


def run_capture(client: RttClient, rtt_config: RttConfig, run_config: RunConfig) -> str:
    out_dir = make_out_dir(run_config.out_root)
    capture = Capture(out_dir=out_dir)

    if run_config.script:
        commands, duration_s = load_script(run_config.script)
    elif run_config.preset:
        commands, duration_s = preset_commands(run_config.preset, run_config)
    else:
        raise ValueError("run requires --preset or --script")

    session = {
        "commands": commands,
        "duration_s": duration_s,
        "preset": run_config.preset,
        "script": run_config.script,
        "device": rtt_config.device,
        "serial_no": rtt_config.serial_no,
        "speed_khz": rtt_config.speed_khz,
    }

    try:
        sample_command = f"tune sample on rate_ms={run_config.sample_rate_ms}"
        print(f"> {sample_command}")
        client.write_line(sample_command)
        wait_for_ack(client, capture, sample_command, run_config.timeout_s, rtt_config.poll_interval_s)
        for command in commands:
            print(f"> {command}")
            client.write_line(command)
            wait_for_ack(client, capture, command, run_config.timeout_s, rtt_config.poll_interval_s)
        drain_for(client, capture, duration_s, rtt_config.poll_interval_s)
    finally:
        try:
            client.write_line("tune off")
            client.write_line("tune sample off")
        except Exception:
            pass
        write_capture(capture, session)

    return out_dir


def command_send(client: RttClient, command: str, timeout_s: float, poll_interval_s: float) -> None:
    capture = Capture(out_dir=os.getcwd())
    print(f"> {command}")
    client.write_line(command)
    wait_for_ack(client, capture, command, timeout_s, poll_interval_s)
    drain_for(client, capture, 0.25, poll_interval_s)


def command_trial(client: RttClient, rtt_config: RttConfig, run_config: RunConfig, sets: Sequence[str]) -> str:
    capture = Capture(out_dir=os.getcwd())

    print("> tune get")
    client.write_line("tune get")
    wait_for_ack(client, capture, "tune get", run_config.timeout_s, rtt_config.poll_interval_s)

    for assignment in sets:
        command = f"tune set {assignment}"
        print(f"> {command}")
        client.write_line(command)
        wait_for_ack(client, capture, command, run_config.timeout_s, rtt_config.poll_interval_s)

    out_dir = run_capture(client, rtt_config, run_config)
    print(f"output: {out_dir}")
    print("")
    command_analyze(out_dir)
    return out_dir


def command_monitor(client: RttClient, duration_s: float, poll_interval_s: float) -> None:
    capture = Capture(out_dir=os.getcwd())
    drain_for(client, capture, duration_s, poll_interval_s)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run gimbal tune tests through SEGGER RTT.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_common(p: argparse.ArgumentParser) -> None:
        p.add_argument("--device", default="STM32F405RG")
        p.add_argument("--serial-no", type=int, default=None)
        p.add_argument("--speed-khz", type=int, default=4000)
        p.add_argument("--up-buffer", type=int, default=0)
        p.add_argument("--down-buffer", type=int, default=0)
        p.add_argument("--timeout", type=float, default=10.0)
        p.add_argument("--startup-timeout", type=float, default=5.0)
        p.add_argument("--poll-interval", type=float, default=0.02)

    run_parser = subparsers.add_parser("run", help="Run a preset or JSON command script")
    add_common(run_parser)
    run_parser.add_argument("--preset", default=None)
    run_parser.add_argument("--script", default=None)
    run_parser.add_argument("--duration", type=float, default=10.0)
    run_parser.add_argument("--sample-rate-ms", type=int, default=10)
    run_parser.add_argument("--amp", type=float, default=None)
    run_parser.add_argument("--hold-ms", type=int, default=None)
    run_parser.add_argument("--out-root", default=os.path.join("tools", "out", "tune"))

    trial_parser = subparsers.add_parser("trial", help="Set runtime params, run one preset, then analyze")
    add_common(trial_parser)
    trial_parser.add_argument("--preset", required=True)
    trial_parser.add_argument("--duration", type=float, default=10.0)
    trial_parser.add_argument("--sample-rate-ms", type=int, default=10)
    trial_parser.add_argument("--amp", type=float, default=None)
    trial_parser.add_argument("--hold-ms", type=int, default=None)
    trial_parser.add_argument("--out-root", default=os.path.join("tools", "out", "tune"))
    trial_parser.add_argument(
        "--set",
        dest="sets",
        action="append",
        default=[],
        metavar="PARAM=VALUE",
        help="Runtime tune parameter assignment. Can be passed more than once.",
    )

    send_parser = subparsers.add_parser("send", help="Send one RTT command")
    add_common(send_parser)
    send_parser.add_argument("line")

    monitor_parser = subparsers.add_parser("monitor", help="Print RTT output")
    add_common(monitor_parser)
    monitor_parser.add_argument("--duration", type=float, default=30.0)

    analyze_parser = subparsers.add_parser("analyze", help="Analyze an existing tune output directory")
    analyze_parser.add_argument("out_dir")

    return parser


def make_rtt_config(args: argparse.Namespace) -> RttConfig:
    return RttConfig(
        device=args.device,
        serial_no=args.serial_no,
        speed_khz=args.speed_khz,
        up_buffer=args.up_buffer,
        down_buffer=args.down_buffer,
        startup_timeout_s=args.startup_timeout,
        poll_interval_s=args.poll_interval,
    )


def main(argv: Sequence[str]) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command == "analyze":
        try:
            command_analyze(args.out_dir)
            return 0
        except Exception as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 1

    rtt_config = make_rtt_config(args)
    client = RttClient(rtt_config)

    try:
        client.open()
        if args.command == "run":
            run_config = RunConfig(
                out_root=args.out_root,
                timeout_s=args.timeout,
                sample_rate_ms=args.sample_rate_ms,
                preset=args.preset,
                script=args.script,
                amp=args.amp,
                hold_ms=args.hold_ms,
                duration_s=args.duration,
            )
            out_dir = run_capture(client, rtt_config, run_config)
            print(f"output: {out_dir}")
        elif args.command == "trial":
            run_config = RunConfig(
                out_root=args.out_root,
                timeout_s=args.timeout,
                sample_rate_ms=args.sample_rate_ms,
                preset=args.preset,
                script=None,
                amp=args.amp,
                hold_ms=args.hold_ms,
                duration_s=args.duration,
            )
            command_trial(client, rtt_config, run_config, args.sets)
        elif args.command == "send":
            command_send(client, args.line, args.timeout, args.poll_interval)
        elif args.command == "monitor":
            command_monitor(client, args.duration, args.poll_interval)
        else:
            parser.error(f"unknown command: {args.command}")
    except KeyboardInterrupt:
        try:
            client.write_line("tune off")
            client.write_line("tune sample off")
        except Exception:
            pass
        print("interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
