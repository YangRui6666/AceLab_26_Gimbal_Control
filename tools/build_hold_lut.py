#!/usr/bin/env python3
"""
Build a hold-current LUT from a previously captured raw_hold_samples.csv file.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from collections import defaultdict
from typing import Dict, List, Sequence, Tuple


RAW_HEADER = [
    "kind",
    "run_id",
    "test",
    "axis",
    "pitch_deg",
    "target_deg",
    "direction",
    "tick_ms",
    "meas_deg",
    "meas_speed_dps",
    "current_meas",
    "current_cmd",
    "current_pid",
    "ff_total",
]

LUT_HEADER = [
    "axis",
    "pitch_deg",
    "target_deg",
    "direction",
    "mean_current",
    "count",
]


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build I_hold_lut.csv from raw_hold_samples.csv.")
    parser.add_argument("input_csv", help="Path to raw_hold_samples.csv")
    parser.add_argument(
        "--output",
        default=None,
        help="Output path for I_hold_lut.csv, defaults next to the input file",
    )
    parser.add_argument(
        "--current-column",
        default="current_meas",
        choices=["current_meas", "current_cmd"],
        help="Column used when averaging current",
    )
    return parser.parse_args(argv)


def read_rows(path: str) -> List[Dict[str, str]]:
    with open(path, "r", newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        fieldnames = reader.fieldnames or []
        required = [name for name in RAW_HEADER if name != "pitch_deg"]
        missing = [name for name in required if name not in fieldnames]
        if missing:
            raise ValueError(f"input CSV is missing columns: {', '.join(missing)}")

        rows = list(reader)
        if "pitch_deg" not in fieldnames:
            for row in rows:
                row["pitch_deg"] = "0.000"
        return rows


def build_lut_rows(rows: List[Dict[str, str]], current_column: str) -> List[Dict[str, str]]:
    grouped: Dict[Tuple[str, str, str, str], List[float]] = defaultdict(list)

    for row in rows:
        try:
            current_value = float(row.get(current_column, ""))
        except ValueError:
            continue

        axis = row.get("axis", "")
        pitch_deg = row.get("pitch_deg", "0.000")
        target_deg = row.get("target_deg", "")
        direction = row.get("direction", "")
        if not axis or not pitch_deg or not target_deg or not direction:
            continue

        grouped[(axis, pitch_deg, target_deg, direction)].append(current_value)

    lut_rows: List[Dict[str, str]] = []
    for key in sorted(grouped.keys(), key=lambda item: (item[0], float(item[1]), float(item[2]), item[3])):
        currents = grouped[key]
        mean_current = sum(currents) / float(len(currents))
        lut_rows.append(
            {
                "axis": key[0],
                "pitch_deg": key[1],
                "target_deg": key[2],
                "direction": key[3],
                "mean_current": f"{mean_current:.6f}",
                "count": str(len(currents)),
            }
        )

    return lut_rows


def write_rows(path: str, rows: List[Dict[str, str]]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=LUT_HEADER)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    input_csv = os.path.abspath(args.input_csv)
    output_csv = (
        os.path.abspath(args.output)
        if args.output is not None
        else os.path.join(os.path.dirname(input_csv), "I_hold_lut.csv")
    )

    try:
        rows = read_rows(input_csv)
        lut_rows = build_lut_rows(rows, args.current_column)
        write_rows(output_csv, lut_rows)
    except Exception as exc:
        print(f"failed to build LUT: {exc}", file=sys.stderr)
        return 1

    print(f"input:  {input_csv}")
    print(f"output: {output_csv}")
    print(f"rows:   {len(lut_rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
