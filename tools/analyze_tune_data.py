#!/usr/bin/env python3
"""
Summarize one or more tune_rtt output directories.

This script is offline-only. It reads tools/out/tune/<run>/raw_samples.csv and
session.json, then prints a compact Markdown report for comparing experiments.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


DEFAULT_ROOT = os.path.join("tools", "out", "tune")
YAW_LIMIT_WARN_DEG = 55.0
PITCH_LOW_WARN_DEG = -8.0
PITCH_HIGH_WARN_DEG = 38.0
CENTER_YAW_OK_DEG = 2.0
CENTER_PITCH_TARGET_DEG = 10.0
CENTER_PITCH_OK_DEG = 1.0


@dataclass
class SegmentSummary:
    target: float
    duration_ms: int
    sample_count: int
    yaw_delta_deg: float
    pitch_delta_deg: float
    meas_avg: float
    meas_tail: float
    meas_min: float
    meas_max: float
    overshoot_pct: float
    steady_error_pct: float
    cmd_peak: float


@dataclass
class RunSummary:
    name: str
    path: str
    sample_count: int
    modes: List[str]
    axis: str
    preset: str = ""
    commands: List[str] = field(default_factory=list)
    duration_s: Optional[float] = None
    theoretical_step_deg: Optional[float] = None
    start_yaw_deg: float = 0.0
    start_pitch_deg: float = 0.0
    end_yaw_deg: float = 0.0
    end_pitch_deg: float = 0.0
    yaw_min_deg: float = 0.0
    yaw_max_deg: float = 0.0
    pitch_min_deg: float = 0.0
    pitch_max_deg: float = 0.0
    speed_target_min: float = 0.0
    speed_target_max: float = 0.0
    speed_meas_min: float = 0.0
    speed_meas_max: float = 0.0
    cable_ff_peak: float = 0.0
    current_cmd_peak: float = 0.0
    current_meas_peak: float = 0.0
    near_limit: bool = False
    dirty_end: bool = False
    positive_tail_mean: Optional[float] = None
    negative_tail_mean: Optional[float] = None
    asymmetry_ratio: Optional[float] = None
    segments: List[SegmentSummary] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)


def read_csv(path: str) -> List[Dict[str, str]]:
    with open(path, newline="", encoding="utf-8") as fp:
        return list(csv.DictReader(fp))


def read_session(path: str) -> Dict[str, object]:
    session_path = os.path.join(path, "session.json")
    if not os.path.exists(session_path):
        return {}

    try:
        with open(session_path, encoding="utf-8") as fp:
            data = json.load(fp)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def row_float(row: Dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return default


def row_tick_ms(row: Dict[str, str]) -> int:
    try:
        return int(float(row.get("tick_ms", "0")))
    except (TypeError, ValueError):
        return 0


def float_values(rows: Iterable[Dict[str, str]], key: str) -> List[float]:
    values: List[float] = []
    for row in rows:
        try:
            values.append(float(row.get(key, "")))
        except (TypeError, ValueError):
            pass
    return values


def max_abs(values: Iterable[float]) -> float:
    return max((abs(value) for value in values), default=0.0)


def mean(values: Sequence[float]) -> float:
    return sum(values) / float(len(values)) if values else 0.0


def infer_axis(rows: List[Dict[str, str]], session: Dict[str, object]) -> str:
    preset = str(session.get("preset", ""))
    if preset.endswith("-pitch"):
        return "pitch"
    if preset.endswith("-yaw"):
        return "yaw"

    commands = session.get("commands", [])
    if isinstance(commands, list):
        for command in commands:
            text = str(command)
            if "axis=pitch" in text:
                return "pitch"
            if "axis=yaw" in text:
                return "yaw"

    yaw_target = max_abs(float_values(rows, "yaw_speed_target_dps"))
    pitch_target = max_abs(float_values(rows, "pitch_speed_target_dps"))
    return "pitch" if pitch_target > yaw_target else "yaw"


def parse_theoretical_step_deg(commands: Sequence[str]) -> Optional[float]:
    for command in commands:
        if "speed_step" not in command:
            continue

        amp_match = re.search(r"\bamp=([-+]?\d+(?:\.\d+)?)", command)
        hold_match = re.search(r"\bhold_ms=(\d+)", command)
        if amp_match is None or hold_match is None:
            continue

        amp = abs(float(amp_match.group(1)))
        hold_ms = float(hold_match.group(1))
        return amp * hold_ms / 1000.0

    return None


def split_target_segments(rows: List[Dict[str, str]], target_key: str) -> List[List[Dict[str, str]]]:
    segments: List[List[Dict[str, str]]] = []
    current: List[Dict[str, str]] = []
    last_target: Optional[float] = None

    for row in rows:
        target = row_float(row, target_key)
        if last_target is None or abs(target - last_target) > 0.5:
            if current:
                segments.append(current)
            current = [row]
            last_target = target
        else:
            current.append(row)

    if current:
        segments.append(current)

    return segments


def summarize_segment(segment: List[Dict[str, str]], axis: str) -> SegmentSummary:
    target_key = f"{axis}_speed_target_dps"
    meas_key = f"{axis}_meas_speed_dps"
    cmd_key = f"{axis}_current_cmd"

    target = mean(float_values(segment, target_key))
    measured = float_values(segment, meas_key)
    commands = float_values(segment, cmd_key)
    yaw_values = float_values(segment, "meas_yaw_deg")
    pitch_values = float_values(segment, "meas_pitch_deg")

    tail_count = min(len(measured), max(5, len(measured) // 2))
    tail_values = measured[-tail_count:] if tail_count > 0 else []
    meas_tail = mean(tail_values)

    duration_ms = row_tick_ms(segment[-1]) - row_tick_ms(segment[0]) if segment else 0
    meas_min = min(measured, default=0.0)
    meas_max = max(measured, default=0.0)
    overshoot_pct = 0.0
    steady_error_pct = 0.0

    if target > 1.0:
        overshoot_pct = max(0.0, (meas_max - target) / abs(target) * 100.0)
        steady_error_pct = abs(target - meas_tail) / abs(target) * 100.0
    elif target < -1.0:
        overshoot_pct = max(0.0, (target - meas_min) / abs(target) * 100.0)
        steady_error_pct = abs(target - meas_tail) / abs(target) * 100.0

    return SegmentSummary(
        target=target,
        duration_ms=duration_ms,
        sample_count=len(segment),
        yaw_delta_deg=(yaw_values[-1] - yaw_values[0]) if len(yaw_values) >= 2 else 0.0,
        pitch_delta_deg=(pitch_values[-1] - pitch_values[0]) if len(pitch_values) >= 2 else 0.0,
        meas_avg=mean(measured),
        meas_tail=meas_tail,
        meas_min=meas_min,
        meas_max=meas_max,
        overshoot_pct=overshoot_pct,
        steady_error_pct=steady_error_pct,
        cmd_peak=max_abs(commands),
    )


def summarize_run(path: str) -> RunSummary:
    raw_path = os.path.join(path, "raw_samples.csv")
    if not os.path.exists(raw_path):
        raise FileNotFoundError(f"raw_samples.csv not found: {path}")

    rows = read_csv(raw_path)
    if not rows:
        raise ValueError(f"no samples found: {path}")

    session = read_session(path)
    commands = [str(item) for item in session.get("commands", [])] if isinstance(session.get("commands", []), list) else []
    axis = infer_axis(rows, session)
    modes = sorted({row.get("mode", "") for row in rows if row.get("mode", "")})
    active_rows = [row for row in rows if row.get("mode") != "off"] or rows

    yaw_values = float_values(active_rows, "meas_yaw_deg")
    pitch_values = float_values(active_rows, "meas_pitch_deg")
    target_key = f"{axis}_speed_target_dps"
    meas_key = f"{axis}_meas_speed_dps"
    cable_key = f"{axis}_cable_ff"
    cmd_key = f"{axis}_current_cmd"
    current_key = f"{axis}_current_meas"

    summary = RunSummary(
        name=os.path.basename(os.path.normpath(path)),
        path=path,
        sample_count=len(rows),
        modes=modes,
        axis=axis,
        preset=str(session.get("preset", "")),
        commands=commands,
        duration_s=float(session["duration_s"]) if "duration_s" in session else None,
        theoretical_step_deg=parse_theoretical_step_deg(commands),
        start_yaw_deg=row_float(rows[0], "meas_yaw_deg"),
        start_pitch_deg=row_float(rows[0], "meas_pitch_deg"),
        end_yaw_deg=row_float(rows[-1], "meas_yaw_deg"),
        end_pitch_deg=row_float(rows[-1], "meas_pitch_deg"),
        yaw_min_deg=min(yaw_values, default=0.0),
        yaw_max_deg=max(yaw_values, default=0.0),
        pitch_min_deg=min(pitch_values, default=0.0),
        pitch_max_deg=max(pitch_values, default=0.0),
        speed_target_min=min(float_values(active_rows, target_key), default=0.0),
        speed_target_max=max(float_values(active_rows, target_key), default=0.0),
        speed_meas_min=min(float_values(active_rows, meas_key), default=0.0),
        speed_meas_max=max(float_values(active_rows, meas_key), default=0.0),
        cable_ff_peak=max_abs(float_values(active_rows, cable_key)),
        current_cmd_peak=max_abs(float_values(active_rows, cmd_key)),
        current_meas_peak=max_abs(float_values(active_rows, current_key)),
    )

    summary.near_limit = (
        summary.yaw_min_deg <= -YAW_LIMIT_WARN_DEG
        or summary.yaw_max_deg >= YAW_LIMIT_WARN_DEG
        or summary.pitch_min_deg <= PITCH_LOW_WARN_DEG
        or summary.pitch_max_deg >= PITCH_HIGH_WARN_DEG
    )
    summary.dirty_end = (
        abs(summary.end_yaw_deg) > CENTER_YAW_OK_DEG
        or abs(summary.end_pitch_deg - CENTER_PITCH_TARGET_DEG) > CENTER_PITCH_OK_DEG
    )

    if any(mode.startswith("speed") for mode in modes):
        speed_rows = [row for row in rows if row.get("mode", "").startswith("speed")]
        for segment in split_target_segments(speed_rows, target_key):
            summary.segments.append(summarize_segment(segment, axis))

        positive_tails = [seg.meas_tail for seg in summary.segments if seg.target > 1.0]
        negative_tails = [seg.meas_tail for seg in summary.segments if seg.target < -1.0]
        if positive_tails:
            summary.positive_tail_mean = mean(positive_tails)
        if negative_tails:
            summary.negative_tail_mean = mean(negative_tails)
        if summary.positive_tail_mean is not None and summary.negative_tail_mean is not None:
            pos = abs(summary.positive_tail_mean)
            neg = abs(summary.negative_tail_mean)
            denom = max(pos, neg, 1.0)
            summary.asymmetry_ratio = abs(pos - neg) / denom

    if summary.near_limit:
        summary.notes.append("near-limit data; do not use directly for tuning")
    if summary.dirty_end:
        summary.notes.append("end pose is not centered")
    if summary.asymmetry_ratio is not None and summary.asymmetry_ratio > 0.25:
        summary.notes.append("direction asymmetry is significant")
    if summary.theoretical_step_deg is not None and summary.theoretical_step_deg > 12.0:
        summary.notes.append("single step displacement is too large")

    return summary


def list_latest(root: str, count: int) -> List[str]:
    if not os.path.isdir(root):
        return []

    dirs = [
        os.path.join(root, name)
        for name in os.listdir(root)
        if os.path.isdir(os.path.join(root, name)) and os.path.exists(os.path.join(root, name, "raw_samples.csv"))
    ]
    return sorted(dirs)[-count:]


def resolve_paths(paths: Sequence[str], root: str, latest: Optional[int]) -> List[str]:
    resolved: List[str] = []

    if latest is not None:
        resolved.extend(list_latest(root, latest))

    for path in paths:
        if os.path.isdir(path):
            resolved.append(path)
            continue

        rooted = os.path.join(root, path)
        if os.path.isdir(rooted):
            resolved.append(rooted)
            continue

        raise FileNotFoundError(f"run directory not found: {path}")

    if not resolved:
        resolved.extend(list_latest(root, 1))

    seen = set()
    unique: List[str] = []
    for path in resolved:
        norm = os.path.normpath(path)
        if norm in seen:
            continue
        seen.add(norm)
        unique.append(norm)
    return unique


def fmt(value: Optional[float], digits: int = 1) -> str:
    if value is None or math.isnan(value):
        return "n/a"
    return f"{value:.{digits}f}"


def render_report(summaries: Sequence[RunSummary]) -> str:
    lines: List[str] = []
    lines.append("# Tune Data Analysis")
    lines.append("")
    lines.append("| run | preset | axis | start yaw/pitch | end yaw/pitch | yaw range | speed meas | cable peak | cmd peak | flags |")
    lines.append("|---|---|---|---|---|---|---|---:|---:|---|")

    for item in summaries:
        flags = []
        if item.near_limit:
            flags.append("LIMIT")
        if item.dirty_end:
            flags.append("DIRTY_END")
        if item.asymmetry_ratio is not None and item.asymmetry_ratio > 0.25:
            flags.append("ASYM")
        if item.theoretical_step_deg is not None and item.theoretical_step_deg > 12.0:
            flags.append("BIG_STEP")
        flag_text = ", ".join(flags) if flags else "OK"
        lines.append(
            f"| {item.name} | {item.preset or '-'} | {item.axis} | "
            f"{fmt(item.start_yaw_deg)}/{fmt(item.start_pitch_deg)} | "
            f"{fmt(item.end_yaw_deg)}/{fmt(item.end_pitch_deg)} | "
            f"{fmt(item.yaw_min_deg)}..{fmt(item.yaw_max_deg)} | "
            f"{fmt(item.speed_meas_min)}..{fmt(item.speed_meas_max)} | "
            f"{item.cable_ff_peak:.0f} | "
            f"{item.current_cmd_peak:.0f} | {flag_text} |"
        )

    for item in summaries:
        lines.append("")
        lines.append(f"## {item.name}")
        lines.append("")
        lines.append(f"- path: `{item.path}`")
        lines.append(f"- modes: {', '.join(item.modes) if item.modes else '-'}")
        if item.commands:
            for command in item.commands:
                lines.append(f"- command: `{command}`")
        if item.theoretical_step_deg is not None:
            lines.append(f"- theoretical single-step displacement: {item.theoretical_step_deg:.1f} deg")
        lines.append(f"- yaw range: {item.yaw_min_deg:.2f} .. {item.yaw_max_deg:.2f} deg")
        lines.append(f"- pitch range: {item.pitch_min_deg:.2f} .. {item.pitch_max_deg:.2f} deg")
        lines.append(f"- speed target range: {item.speed_target_min:.1f} .. {item.speed_target_max:.1f} dps")
        lines.append(f"- measured speed range: {item.speed_meas_min:.1f} .. {item.speed_meas_max:.1f} dps")
        lines.append(f"- cable feedforward peak: {item.cable_ff_peak:.0f}")
        lines.append(f"- current command peak: {item.current_cmd_peak:.0f}")
        lines.append(f"- measured current peak: {item.current_meas_peak:.0f}")
        if item.positive_tail_mean is not None or item.negative_tail_mean is not None:
            lines.append(
                "- tail speed mean: "
                f"positive={fmt(item.positive_tail_mean)} dps, "
                f"negative={fmt(item.negative_tail_mean)} dps, "
                f"asymmetry={fmt(item.asymmetry_ratio * 100.0 if item.asymmetry_ratio is not None else None)}%"
            )
        if item.notes:
            lines.append(f"- notes: {'; '.join(item.notes)}")

        nonzero_segments = [segment for segment in item.segments if abs(segment.target) > 1.0]
        if nonzero_segments:
            lines.append("")
            lines.append("| target | duration | yaw delta | avg speed | tail speed | speed range | overshoot | steady error | cmd peak |")
            lines.append("|---:|---:|---:|---:|---:|---|---:|---:|---:|")
            for segment in nonzero_segments:
                lines.append(
                    f"| {segment.target:+.1f} | {segment.duration_ms} ms | "
                    f"{segment.yaw_delta_deg:+.2f} deg | {segment.meas_avg:+.1f} | "
                    f"{segment.meas_tail:+.1f} | {segment.meas_min:+.1f}..{segment.meas_max:+.1f} | "
                    f"{segment.overshoot_pct:.1f}% | {segment.steady_error_pct:.1f}% | {segment.cmd_peak:.0f} |"
                )

    return "\n".join(lines) + "\n"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Summarize tune_rtt output directories.")
    parser.add_argument("runs", nargs="*", help="Run directories or names under tools/out/tune.")
    parser.add_argument("--root", default=DEFAULT_ROOT, help="Tune output root directory.")
    parser.add_argument("--latest", type=int, default=None, help="Analyze the latest N runs from --root.")
    parser.add_argument("--out", default=None, help="Write Markdown report to this path.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    paths = resolve_paths(args.runs, args.root, args.latest)
    if not paths:
        print(f"No tune output directories found under {args.root}")
        return 1

    summaries = [summarize_run(path) for path in paths]
    report = render_report(summaries)
    print(report, end="")

    if args.out:
        with open(args.out, "w", encoding="utf-8", newline="\n") as fp:
            fp.write(report)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
