#!/usr/bin/env python3
"""
Collect yaw hold sweep samples through SEGGER RTT and generate a hold-current LUT.

Dependencies:
    pip install -r tools/requirements-rtt.txt
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Sequence, Tuple


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


@dataclass
class Config:
    device: str
    serial_no: Optional[int]
    speed_khz: int
    up_buffer: int
    down_buffer: int
    read_size: int
    timeout_s: float
    poll_interval_s: float
    startup_timeout_s: float
    pitch_list: List[float]
    out_root: str


@dataclass
class CaptureResult:
    pitch_deg: float = 0.0
    status: str = "unknown"
    done_line: str = ""
    header: List[str] = field(default_factory=list)
    comments: List[str] = field(default_factory=list)
    samples: List[Dict[str, str]] = field(default_factory=list)


class RttClient:
    def __init__(self, config: Config) -> None:
        self._config = config
        self._pylink = None
        self._jlink = None

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

        if self._config.serial_no is None:
            self._jlink.open()
        else:
            self._jlink.open(self._config.serial_no)

        self._jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
        self._jlink.connect(self._config.device, speed=self._config.speed_khz)
        try:
            if self._jlink.halted():
                self._jlink.restart()
        except Exception:
            pass

        self._jlink.rtt_start()
        self.wait_until_ready(self._config.startup_timeout_s)

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

        payload = line.encode("utf-8")
        deadline = time.monotonic() + self._config.startup_timeout_s

        while True:
            written = self._jlink.rtt_write(self._config.down_buffer, payload)
            if written == len(payload):
                return

            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"failed to write complete RTT command: {written}/{len(payload)} bytes"
                )

            time.sleep(self._config.poll_interval_s)

    def read_bytes(self) -> bytes:
        if self._jlink is None:
            raise RuntimeError("RTT client is not open")

        data = self._jlink.rtt_read(self._config.up_buffer, self._config.read_size)
        return bytes(data)

    def wait_until_ready(self, timeout_s: float) -> None:
        if self._jlink is None:
            raise RuntimeError("RTT client is not open")

        deadline = time.monotonic() + timeout_s
        last_error = "RTT control block not ready"

        while time.monotonic() < deadline:
            try:
                up_buffers = self._jlink.rtt_get_num_up_buffers()
                down_buffers = self._jlink.rtt_get_num_down_buffers()
                if (up_buffers > self._config.up_buffer) and (down_buffers > self._config.down_buffer):
                    return
                last_error = (
                    f"RTT buffers not ready yet: up={up_buffers}, down={down_buffers}, "
                    f"need up>{self._config.up_buffer}, down>{self._config.down_buffer}"
                )
            except Exception as exc:
                last_error = str(exc)

            time.sleep(self._config.poll_interval_s)

        raise RuntimeError(f"timed out waiting for RTT ready: {last_error}")


def parse_args(argv: Sequence[str]) -> Config:
    parser = argparse.ArgumentParser(description="Collect hold-LUT data through SEGGER RTT.")
    parser.add_argument("--device", default="STM32F405RG", help="J-Link target device name")
    parser.add_argument("--serial-no", type=int, default=None, help="Optional J-Link serial number")
    parser.add_argument("--speed-khz", type=int, default=4000, help="SWD speed in kHz")
    parser.add_argument("--up-buffer", type=int, default=0, help="RTT up-buffer index")
    parser.add_argument("--down-buffer", type=int, default=0, help="RTT down-buffer index")
    parser.add_argument("--read-size", type=int, default=1024, help="RTT read chunk size")
    parser.add_argument("--timeout", type=float, default=180.0, help="Capture timeout in seconds")
    parser.add_argument("--poll-interval", type=float, default=0.02, help="RTT poll interval in seconds")
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=5.0,
        help="Time to wait for RTT buffers to become ready",
    )
    parser.add_argument(
        "--pitch-list",
        default="0",
        help="Comma-separated pitch list in degrees, for example -10,0,10",
    )
    parser.add_argument(
        "--out-root",
        default=os.path.join("tools", "out", "ff_hold"),
        help="Output root directory",
    )

    args = parser.parse_args(argv)
    pitch_list: List[float] = []
    for raw_item in args.pitch_list.split(","):
        text = raw_item.strip()
        if not text:
            continue
        try:
            pitch_list.append(float(text))
        except ValueError as exc:
            raise SystemExit(f"invalid --pitch-list value: {text}") from exc

    if not pitch_list:
        raise SystemExit("--pitch-list must contain at least one numeric value")

    return Config(
        device=args.device,
        serial_no=args.serial_no,
        speed_khz=args.speed_khz,
        up_buffer=args.up_buffer,
        down_buffer=args.down_buffer,
        read_size=args.read_size,
        timeout_s=args.timeout,
        poll_interval_s=args.poll_interval,
        startup_timeout_s=args.startup_timeout,
        pitch_list=pitch_list,
        out_root=args.out_root,
    )


def split_csv_line(line: str) -> List[str]:
    return [part.strip() for part in line.split(",")]


def format_pitch_deg(value: float) -> str:
    return f"{value:.3f}"


def make_hold_command(pitch_deg: float) -> str:
    return f"ff hold start pitch={format_pitch_deg(pitch_deg)}\n"


def monitor_capture(
    client: RttClient, config: Config, pitch_deg: float, result: Optional[CaptureResult] = None
) -> CaptureResult:
    if result is None:
        result = CaptureResult(pitch_deg=pitch_deg)
    buffer = bytearray()
    deadline = time.monotonic() + config.timeout_s

    client.write_line(make_hold_command(pitch_deg))

    while True:
        if time.monotonic() > deadline:
            result.status = "timeout"
            break

        chunk = client.read_bytes()
        if not chunk:
            time.sleep(config.poll_interval_s)
            continue

        buffer.extend(chunk)

        while True:
            newline_index = buffer.find(b"\n")
            if newline_index < 0:
                break

            raw_line = bytes(buffer[:newline_index])
            del buffer[:newline_index + 1]

            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            print(line)

            if line.startswith("#"):
                result.comments.append(line)
                if line.startswith("#DONE"):
                    result.done_line = line
                    result.status = "aborted" if "aborted" in line.lower() else "completed"
                    return result
                continue

            columns = split_csv_line(line)
            if columns == RAW_HEADER:
                result.header = columns
                continue

            if not result.header:
                continue

            if len(columns) != len(result.header):
                continue

            result.samples.append(dict(zip(result.header, columns)))

    return result


def request_stop_and_drain(client: RttClient, config: Config, result: CaptureResult) -> CaptureResult:
    try:
        client.write_line("ff hold stop\n")
        print("sent ff hold stop", file=sys.stderr)
    except Exception as exc:
        print(f"failed to send ff hold stop: {exc}", file=sys.stderr)
        return result

    stop_deadline = time.monotonic() + 3.0
    buffer = bytearray()

    while time.monotonic() < stop_deadline:
        chunk = client.read_bytes()
        if not chunk:
            time.sleep(config.poll_interval_s)
            continue

        buffer.extend(chunk)

        while True:
            newline_index = buffer.find(b"\n")
            if newline_index < 0:
                break

            raw_line = bytes(buffer[:newline_index])
            del buffer[:newline_index + 1]

            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            print(line)

            if line.startswith("#"):
                result.comments.append(line)
                if line.startswith("#DONE"):
                    result.done_line = line
                    result.status = "aborted" if "aborted" in line.lower() else "completed"
                    return result
                continue

            columns = split_csv_line(line)
            if columns == RAW_HEADER:
                result.header = columns
                continue

            if not result.header:
                continue

            if len(columns) != len(result.header):
                continue

            result.samples.append(dict(zip(result.header, columns)))

    if result.status == "interrupted":
        result.status = "aborted"
    return result


def write_raw_csv(path: str, samples: List[Dict[str, str]]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=RAW_HEADER)
        writer.writeheader()
        for row in samples:
            writer.writerow({key: row.get(key, "") for key in RAW_HEADER})


def build_lut_rows(samples: List[Dict[str, str]]) -> List[Dict[str, str]]:
    grouped: Dict[Tuple[str, str, str, str], List[float]] = defaultdict(list)

    for row in samples:
        axis = row.get("axis", "")
        pitch_deg = row.get("pitch_deg", "")
        target_deg = row.get("target_deg", "")
        direction = row.get("direction", "")
        try:
            current_meas = float(row.get("current_meas", ""))
        except ValueError:
            continue

        grouped[(axis, pitch_deg, target_deg, direction)].append(current_meas)

    rows: List[Dict[str, str]] = []
    for key in sorted(grouped.keys(), key=lambda item: (item[0], float(item[1]), float(item[2]), item[3])):
        currents = grouped[key]
        mean_current = sum(currents) / float(len(currents))
        rows.append(
            {
                "axis": key[0],
                "pitch_deg": key[1],
                "target_deg": key[2],
                "direction": key[3],
                "mean_current": f"{mean_current:.6f}",
                "count": str(len(currents)),
            }
        )

    return rows


def write_lut_csv(path: str, rows: List[Dict[str, str]]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=LUT_HEADER)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_session_summary(path: str, results: List[CaptureResult], config: Config) -> None:
    all_completed = all(result.status == "completed" for result in results)
    final_status = "completed" if all_completed else (results[-1].status if results else "unknown")
    payload = {
        "status": final_status,
        "all_completed": all_completed,
        "sample_count": sum(len(result.samples) for result in results),
        "device": config.device,
        "serial_no": config.serial_no,
        "speed_khz": config.speed_khz,
        "pitch_list": [format_pitch_deg(value) for value in config.pitch_list],
        "runs": [
            {
                "pitch_deg": format_pitch_deg(result.pitch_deg),
                "status": result.status,
                "done_line": result.done_line,
                "sample_count": len(result.samples),
                "comment_count": len(result.comments),
                "command": make_hold_command(result.pitch_deg).rstrip(),
            }
            for result in results
        ],
        "timestamp": datetime.now().isoformat(timespec="seconds"),
    }

    with open(path, "w", encoding="utf-8") as fp:
        json.dump(payload, fp, indent=2, ensure_ascii=False)
        fp.write("\n")


def make_output_dir(root: str) -> str:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(root, timestamp)
    os.makedirs(out_dir, exist_ok=True)
    return out_dir


def main(argv: Sequence[str]) -> int:
    config = parse_args(argv)
    client = RttClient(config)
    out_dir = make_output_dir(config.out_root)
    raw_csv_path = os.path.join(out_dir, "raw_hold_samples.csv")
    lut_csv_path = os.path.join(out_dir, "I_hold_lut.csv")
    summary_path = os.path.join(out_dir, "session.json")

    results: List[CaptureResult] = []
    active_result: Optional[CaptureResult] = None

    try:
        client.open()
        for pitch_deg in config.pitch_list:
            print(f"=== pitch_deg={format_pitch_deg(pitch_deg)} ===")
            active_result = CaptureResult(pitch_deg=pitch_deg, status="interrupted")
            result = monitor_capture(client, config, pitch_deg, active_result)
            active_result = None
            results.append(result)
            if result.status == "timeout":
                results[-1] = request_stop_and_drain(client, config, results[-1])
                break
            if result.status != "completed":
                break
    except KeyboardInterrupt:
        print("capture interrupted by user", file=sys.stderr)
        if active_result is not None:
            results.append(request_stop_and_drain(client, config, active_result))
            active_result = None
        elif results:
            results[-1] = request_stop_and_drain(client, config, results[-1])
        else:
            results.append(CaptureResult(status="aborted"))
    except Exception as exc:
        print(f"capture failed: {exc}", file=sys.stderr)
        if active_result is not None:
            active_result.status = "error"
            results.append(active_result)
            active_result = None
        elif results:
            if results[-1].status == "unknown":
                results[-1].status = "error"
        else:
            results.append(CaptureResult(status="error"))
    finally:
        client.close()

    all_samples: List[Dict[str, str]] = []
    for result in results:
        all_samples.extend(result.samples)

    final_status = results[-1].status if results else "error"
    write_raw_csv(raw_csv_path, all_samples)
    write_lut_csv(lut_csv_path, build_lut_rows(all_samples))
    write_session_summary(summary_path, results, config)

    print(f"raw samples: {raw_csv_path}")
    print(f"hold LUT:    {lut_csv_path}")
    print(f"summary:     {summary_path}")
    print(f"status:      {final_status}")

    if final_status in {"completed", "aborted"}:
        return 0

    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
