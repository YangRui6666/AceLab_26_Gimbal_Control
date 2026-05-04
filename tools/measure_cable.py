#!/usr/bin/env python3
"""
Measure yaw cable feedforward by sweeping yaw angles at fixed pitch.

At each yaw position, the PID current (yaw_current_pid) needed to hold position
reveals the cable restoring torque. Fit a linear model:

    current = cable_ff_offset + cable_ff_k_yaw * yaw_deg

Usage:
    python tools/measure_cable.py
"""

from __future__ import annotations

import csv
import json
import os
import sys
import time
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple

# Add tools dir to path for RttClient
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from tune_rtt import (
    Capture,
    RttClient,
    RttConfig,
    SAMPLE_HEADER,
    drain_for,
    float_values,
    make_out_dir,
    mean_abs,
    process_line,
    wait_for_ack,
    write_capture,
)


def make_rtt_config() -> RttConfig:
    return RttConfig(
        device="STM32F405RG",
        speed_khz=4000,
        startup_timeout_s=5.0,
        poll_interval_s=0.02,
    )


def main() -> int:
    # Sweep parameters
    yaw_sweep_deg = list(range(-45, 50, 10))  # -45, -35, ..., 45
    dwell_s = 2.5        # settle + collect at each position
    settle_s = 1.0       # extra settle time at first position
    pitch_deg = 10.0

    total_s = settle_s + len(yaw_sweep_deg) * dwell_s + 5.0
    print(f"Yaw sweep: {yaw_sweep_deg}")
    print(f"Estimated duration: {total_s:.0f}s")

    rtt_config = make_rtt_config()
    client = RttClient(rtt_config)

    try:
        client.open()
        print("RTT connected.")

        out_dir = make_out_dir(os.path.join("tools", "out", "cable"))
        capture = Capture(out_dir=out_dir)

        # Enable sampling
        client.write_line("tune sample on rate_ms=10")
        wait_for_ack(client, capture, "tune sample on rate_ms=10", 5.0, 0.02)

        # First hold at 0 to stabilize
        first_cmd = f"tune hold yaw=0 pitch={pitch_deg} planner=1"
        print(f"> {first_cmd}")
        client.write_line(first_cmd)
        wait_for_ack(client, capture, first_cmd, 5.0, 0.02)
        drain_for(client, capture, settle_s + 2.0, 0.02)

        # Sweep through all yaw positions
        for yaw in yaw_sweep_deg[1:]:
            cmd = f"tune hold yaw={yaw} pitch={pitch_deg} planner=1"
            print(f"> {cmd}")
            client.write_line(cmd)
            wait_for_ack(client, capture, cmd, 5.0, 0.02)
            drain_for(client, capture, dwell_s, 0.02)

        # Drain any remaining data
        drain_for(client, capture, 1.0, 0.02)

        # Turn off
        client.write_line("tune off")
        drain_for(client, capture, 0.5, 0.02)
        client.write_line("tune sample off")

    except KeyboardInterrupt:
        try:
            client.write_line("tune off")
            client.write_line("tune sample off")
        except Exception:
            pass
        print("\nInterrupted.")
        return 130
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()

    # Save and analyze
    session = {
        "type": "cable_measurement",
        "yaw_sweep_deg": yaw_sweep_deg,
        "pitch_deg": pitch_deg,
        "dwell_s": dwell_s,
    }
    write_capture(capture, session)
    print(f"\nRaw data saved to: {out_dir}")

    return analyze_cable(out_dir)


def split_by_hold_target(rows: List[Dict[str, str]]) -> List[Tuple[float, List[Dict[str, str]]]]:
    """Split samples into segments by hold target yaw angle."""
    segments: List[Tuple[float, List[Dict[str, str]]]] = []
    current_target: Optional[float] = None
    current_rows: List[Dict[str, str]] = []

    for row in rows:
        if row.get("mode") != "hold":
            continue
        try:
            target = float(row["target_yaw_deg"])
        except (KeyError, ValueError):
            continue

        if current_target is None or abs(target - current_target) > 1.0:
            if current_target is not None and current_rows:
                segments.append((current_target, current_rows))
            current_rows = [row]
            current_target = target
        else:
            current_rows.append(row)

    if current_target is not None and current_rows:
        segments.append((current_target, current_rows))

    return segments


def analyze_cable(out_dir: str) -> int:
    raw_path = os.path.join(out_dir, "raw_samples.csv")
    if not os.path.exists(raw_path):
        print(f"raw_samples.csv not found: {raw_path}")
        return 1

    with open(raw_path, newline="", encoding="utf-8") as fp:
        rows = list(csv.DictReader(fp))

    segments = split_by_hold_target(rows)
    if not segments:
        print("No hold segments found in data.")
        return 1

    print("\n=== Cable FF Measurement Results ===")
    print(f"{'yaw_target':>10} {'samples':>8} {'pid_mean':>10} {'pid_std':>10} {'cmd_mean':>10}")
    print("-" * 52)

    yaw_angles: List[float] = []
    pid_means: List[float] = []

    for target, seg_rows in segments:
        all_pid = [float(r.get("yaw_current_pid", "0")) for r in seg_rows]
        all_cmd = [float(r.get("yaw_current_cmd", "0")) for r in seg_rows]

        # Use tail 30% for steady-state estimate (remove transients)
        tail_n = max(10, len(all_pid) // 3)
        tail_pid = all_pid[-tail_n:]
        tail_cmd = all_cmd[-tail_n:]

        pid_mean = sum(tail_pid) / len(tail_pid) if tail_pid else 0.0
        pid_std = (
            (sum((v - pid_mean) ** 2 for v in tail_pid) / len(tail_pid)) ** 0.5
            if tail_pid
            else 0.0
        )
        cmd_mean = sum(tail_cmd) / len(tail_cmd) if tail_cmd else 0.0

        print(f"{target:+10.1f} {len(seg_rows):8d} {pid_mean:+10.2f} {pid_std:10.2f} {cmd_mean:+10.1f}")

        yaw_angles.append(target)
        pid_means.append(pid_mean)

    # Linear regression: pid_current = offset + k_yaw * yaw_deg
    n = len(yaw_angles)
    if n >= 2:
        sum_x = sum(yaw_angles)
        sum_y = sum(pid_means)
        sum_xy = sum(x * y for x, y in zip(yaw_angles, pid_means))
        sum_xx = sum(x * x for x in yaw_angles)

        denom = n * sum_xx - sum_x * sum_x
        if abs(denom) > 1e-9:
            k_yaw = (n * sum_xy - sum_x * sum_y) / denom
            offset = (sum_y - k_yaw * sum_x) / n

            # R²
            y_mean = sum_y / n
            ss_res = sum((y - (offset + k_yaw * x)) ** 2 for x, y in zip(yaw_angles, pid_means))
            ss_tot = sum((y - y_mean) ** 2 for y in pid_means)
            r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 0.0

            print(f"\nLinear fit: current = {offset:.4f} + ({k_yaw:.4f}) * yaw_deg")
            print(f"R^2 = {r2:.4f}")
            print(f"\nSuggested parameters:")
            print(f"  yaw_cable_ff_enable = 1")
            print(f"  yaw_cable_ff_offset = {offset:.2f}")
            print(f"  yaw_cable_ff_k_yaw  = {k_yaw:.4f}")
            print(f"  yaw_cable_ff_k_pitch = 0  (sweep only at one pitch angle)")

    # Save analysis
    analysis = {
        "yaw_angles": yaw_angles,
        "pid_means": pid_means,
        "cable_ff_offset": round(offset, 4) if n >= 2 else None,
        "cable_ff_k_yaw": round(k_yaw, 4) if n >= 2 else None,
        "r2": round(r2, 4) if n >= 2 else None,
    }
    with open(os.path.join(out_dir, "cable_analysis.json"), "w") as fp:
        json.dump(analysis, fp, indent=2)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
