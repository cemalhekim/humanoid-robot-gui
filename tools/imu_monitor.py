#!/usr/bin/env python3
"""Live IMU monitor and logger for the H1-2.

Polls the dashboard's /api/state endpoint, shows every IMU field in the
terminal, and appends each sample to CSV.

Beyond the raw fields it derives the checks that matter for calibration:

  * accel magnitude          -- should sit at 9.81 m/s^2 when the robot is still
  * accel-derived roll/pitch -- gravity is an absolute tilt reference, so when
                                the robot is motionless this is ground truth
  * rpy - accel tilt         -- a sustained non-zero gap is the IMU level offset
  * quaternion-derived rpy   -- cross-check that quaternion and rpy agree
  * gyro magnitude           -- stillness gate; bias figures are only valid
                                while the robot is genuinely motionless

Running mean/sigma are kept for every channel (Welford), so the summary printed
on exit is directly usable as a before/after calibration record.

Examples:
    tools/imu_monitor.py                                  # live view, logs to recordings/imu/
    tools/imu_monitor.py --duration 60 --no-tui           # 60 s capture, plain output
    tools/imu_monitor.py --duration 120 --csv /tmp/pre.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Any

DEFAULT_HOST = "192.168.123.164:8088"
GRAVITY = 9.80665
# Below this total angular rate the robot counts as motionless and the
# gravity-based tilt reference is trustworthy. Measured noise floor on a
# powered, standing H1-2 is ~0.010 rad/s per axis, so this sits at roughly 3
# sigma of the magnitude -- tight enough to catch real motion, loose enough
# that servo hum does not flag every sample as moving.
STILL_GYRO_RAD_S = 0.05

REPO_ROOT = Path(__file__).resolve().parent.parent

CSV_COLUMNS = [
    "wall_time",
    "iso_time",
    "elapsed_s",
    "robot_timestamp",
    "robot_samples",
    "sample_rate_hz",
    "quat_w",
    "quat_x",
    "quat_y",
    "quat_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "gyro_mag",
    "accel_x",
    "accel_y",
    "accel_z",
    "accel_mag",
    "roll",
    "pitch",
    "yaw",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "roll_from_accel",
    "pitch_from_accel",
    "roll_error_deg",
    "pitch_error_deg",
    "roll_from_quat",
    "pitch_from_quat",
    "yaw_from_quat",
    "temperature",
    "still",
]

# Channels tracked with running statistics, in display order.
STAT_CHANNELS = [
    ("gyro_x", "gyro x", "rad/s"),
    ("gyro_y", "gyro y", "rad/s"),
    ("gyro_z", "gyro z", "rad/s"),
    ("accel_x", "accel x", "m/s2"),
    ("accel_y", "accel y", "m/s2"),
    ("accel_z", "accel z", "m/s2"),
    ("accel_mag", "|accel|", "m/s2"),
    ("roll_deg", "roll", "deg"),
    ("pitch_deg", "pitch", "deg"),
    ("yaw_deg", "yaw", "deg"),
    ("roll_error_deg", "roll - accel", "deg"),
    ("pitch_error_deg", "pitch - accel", "deg"),
    ("temperature", "IMU temp", "raw"),
]


class Running:
    """Welford accumulator: mean and sigma without keeping every sample."""

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self._m2 = 0.0
        self.min = math.inf
        self.max = -math.inf

    def add(self, value: float) -> None:
        self.n += 1
        delta = value - self.mean
        self.mean += delta / self.n
        self._m2 += delta * (value - self.mean)
        self.min = min(self.min, value)
        self.max = max(self.max, value)

    @property
    def sigma(self) -> float:
        return math.sqrt(self._m2 / (self.n - 1)) if self.n > 1 else 0.0


def quat_to_rpy(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """ZYX Euler angles from a w,x,y,z quaternion (Unitree's field order)."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def tilt_from_accel(ax: float, ay: float, az: float) -> tuple[float, float]:
    """Roll and pitch implied by the measured gravity vector.

    Valid only while the robot is motionless -- any real acceleration adds
    straight into this estimate.
    """
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    return roll, pitch


def wrap_deg(value: float) -> float:
    """Fold an angle difference into (-180, 180]."""
    return (value + 180.0) % 360.0 - 180.0


def fetch_state(url: str, timeout: float) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def derive(state: dict[str, Any], elapsed: float) -> dict[str, Any] | None:
    """Flatten one /api/state snapshot into a CSV row with derived checks."""
    imu = state.get("imu") or {}
    quat = list(imu.get("quaternion") or [])
    gyro = list(imu.get("gyroscope") or [])
    accel = list(imu.get("accelerometer") or [])
    rpy = list(imu.get("rpy") or [])
    if len(quat) < 4 or len(gyro) < 3 or len(accel) < 3 or len(rpy) < 3:
        return None

    gyro_mag = math.sqrt(sum(v * v for v in gyro))
    accel_mag = math.sqrt(sum(v * v for v in accel))
    roll_acc, pitch_acc = tilt_from_accel(*accel)
    roll_q, pitch_q, yaw_q = quat_to_rpy(*quat[:4])
    now = time.time()

    return {
        "wall_time": round(now, 6),
        "iso_time": datetime.fromtimestamp(now).isoformat(timespec="milliseconds"),
        "elapsed_s": round(elapsed, 3),
        "robot_timestamp": state.get("timestamp"),
        "robot_samples": state.get("samples"),
        "sample_rate_hz": state.get("sample_rate_hz"),
        "quat_w": quat[0],
        "quat_x": quat[1],
        "quat_y": quat[2],
        "quat_z": quat[3],
        "gyro_x": gyro[0],
        "gyro_y": gyro[1],
        "gyro_z": gyro[2],
        "gyro_mag": gyro_mag,
        "accel_x": accel[0],
        "accel_y": accel[1],
        "accel_z": accel[2],
        "accel_mag": accel_mag,
        "roll": rpy[0],
        "pitch": rpy[1],
        "yaw": rpy[2],
        "roll_deg": math.degrees(rpy[0]),
        "pitch_deg": math.degrees(rpy[1]),
        "yaw_deg": math.degrees(rpy[2]),
        "roll_from_accel": roll_acc,
        "pitch_from_accel": pitch_acc,
        "roll_error_deg": wrap_deg(math.degrees(rpy[0] - roll_acc)),
        "pitch_error_deg": wrap_deg(math.degrees(rpy[1] - pitch_acc)),
        "roll_from_quat": roll_q,
        "pitch_from_quat": pitch_q,
        "yaw_from_quat": yaw_q,
        "temperature": imu.get("temperature"),
        "still": int(gyro_mag < STILL_GYRO_RAD_S),
    }


def render(row: dict[str, Any], stats: dict[str, Running], meta: dict[str, Any]) -> str:
    still = "STILL " if row["still"] else "MOVING"
    quat_drift = max(
        abs(wrap_deg(math.degrees(row["roll"] - row["roll_from_quat"]))),
        abs(wrap_deg(math.degrees(row["pitch"] - row["pitch_from_quat"]))),
        abs(wrap_deg(math.degrees(row["yaw"] - row["yaw_from_quat"]))),
    )

    lines = [
        "H1-2 IMU MONITOR  (Ctrl-C ile bitir)",
        f"  host {meta['host']}   dds {row['sample_rate_hz']} Hz"
        f"   poll {meta['poll_hz']:.1f} Hz   n={stats['accel_mag'].n}"
        f"   t={row['elapsed_s']:.1f}s   {still}",
        f"  log  {meta['csv']}",
        "",
        "RAW",
        f"  quaternion  w {row['quat_w']:+.6f}  x {row['quat_x']:+.6f}"
        f"  y {row['quat_y']:+.6f}  z {row['quat_z']:+.6f}",
        f"  gyro rad/s  x {row['gyro_x']:+.6f}  y {row['gyro_y']:+.6f}"
        f"  z {row['gyro_z']:+.6f}   |w| {row['gyro_mag']:.6f}",
        f"  gyro deg/s  x {math.degrees(row['gyro_x']):+8.3f}"
        f"  y {math.degrees(row['gyro_y']):+8.3f}  z {math.degrees(row['gyro_z']):+8.3f}",
        f"  accel m/s2  x {row['accel_x']:+.6f}  y {row['accel_y']:+.6f}"
        f"  z {row['accel_z']:+.6f}   |a| {row['accel_mag']:.6f}",
        f"  rpy  rad    r {row['roll']:+.6f}  p {row['pitch']:+.6f}  y {row['yaw']:+.6f}",
        f"  rpy  deg    r {row['roll_deg']:+8.3f}  p {row['pitch_deg']:+8.3f}"
        f"  y {row['yaw_deg']:+8.3f}",
        f"  temperature {row['temperature']}",
        "",
        "CALIBRATION CHECKS",
        f"  |accel| sapmasi   {row['accel_mag'] - GRAVITY:+.4f} m/s2"
        f"   (olcum {row['accel_mag']:.4f}, beklenen {GRAVITY:.4f})",
        f"  yercekimi egimi   roll {math.degrees(row['roll_from_accel']):+8.3f} deg"
        f"   pitch {math.degrees(row['pitch_from_accel']):+8.3f} deg",
        f"  rpy - egim        roll {row['roll_error_deg']:+8.3f} deg"
        f"   pitch {row['pitch_error_deg']:+8.3f} deg   <-- level offset",
        f"  quat/rpy uyumu    {quat_drift:.4f} deg",
        "",
        f"{'ISTATISTIK':<14}{'ortalama':>13}{'sigma':>12}{'min':>12}{'max':>12}  birim",
    ]
    for key, label, unit in STAT_CHANNELS:
        acc = stats[key]
        if acc.n == 0:
            continue
        lines.append(
            f"  {label:<12}{acc.mean:>13.5f}{acc.sigma:>12.5f}"
            f"{acc.min:>12.5f}{acc.max:>12.5f}  {unit}"
        )
    return "\n".join(lines)


def summarise(stats: dict[str, Running], still_n: int, total_n: int) -> str:
    lines = ["", "=" * 74, f"OZET  ({total_n} ornek, {still_n} tanesi hareketsiz)", "=" * 74]
    lines.append(f"{'kanal':<14}{'ortalama':>13}{'sigma':>12}{'min':>12}{'max':>12}  birim")
    for key, label, unit in STAT_CHANNELS:
        acc = stats[key]
        if acc.n == 0:
            continue
        lines.append(
            f"  {label:<12}{acc.mean:>13.5f}{acc.sigma:>12.5f}"
            f"{acc.min:>12.5f}{acc.max:>12.5f}  {unit}"
        )

    if still_n < total_n * 0.9:
        lines += [
            "",
            "UYARI: orneklerin cogunda robot hareketliydi. Bias rakamlari yalnizca",
            "robot tamamen sabitken gecerlidir; olcumu tekrarla.",
        ]
        return "\n".join(lines)

    gyro_bias = max(abs(stats[f"gyro_{a}"].mean) for a in "xyz")
    lines += [
        "",
        "YORUM",
        f"  gyro bias (mutlak en buyuk eksen)  {gyro_bias:.6f} rad/s"
        f"  = {math.degrees(gyro_bias) * 3600:.1f} deg/saat surukleme",
        f"  |accel| hatasi                     {stats['accel_mag'].mean - GRAVITY:+.4f} m/s2",
        f"  roll level offset                  {stats['roll_error_deg'].mean:+.3f} deg"
        f"  (sigma {stats['roll_error_deg'].sigma:.3f})",
        f"  pitch level offset                 {stats['pitch_error_deg'].mean:+.3f} deg"
        f"  (sigma {stats['pitch_error_deg'].sigma:.3f})",
        "",
        "  Level offset, robotun gercekten dik ve zeminin terazide oldugu",
        "  varsayimiyla gecerlidir. Sigma degeri ortalamanin yaninda kucukse",
        "  offset gercek bir sabit kaymadir, gurultu degil.",
    ]
    return "\n".join(lines)


def default_csv_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return REPO_ROOT / "recordings" / "imu" / f"imu-{stamp}.csv"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"dashboard host:port (default {DEFAULT_HOST})")
    parser.add_argument("--rate", type=float, default=20.0, help="poll rate in Hz (default 20)")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (default: run until Ctrl-C)")
    parser.add_argument("--csv", type=Path, default=None, help="CSV output path (default recordings/imu/imu-<stamp>.csv)")
    parser.add_argument("--no-csv", action="store_true", help="live view only, write nothing")
    parser.add_argument("--no-tui", action="store_true", help="append plain lines instead of redrawing")
    parser.add_argument("--timeout", type=float, default=3.0, help="HTTP timeout in seconds (default 3)")
    args = parser.parse_args(argv)

    url = f"http://{args.host}/api/state"
    period = 1.0 / args.rate if args.rate > 0 else 0.0

    try:
        state = fetch_state(url, args.timeout)
    except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
        print(f"{url} okunamadi: {exc}", file=sys.stderr)
        return 1
    if not state.get("connected"):
        print(f"Dashboard ayakta ama rt/lowstate baglantisi yok (connected={state.get('connected')}).", file=sys.stderr)
        return 1

    csv_path: Path | None = None
    writer = None
    handle = None
    if not args.no_csv:
        csv_path = args.csv or default_csv_path()
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        handle = csv_path.open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        writer.writeheader()

    stats = {key: Running() for key, _, _ in STAT_CHANNELS}
    stats.setdefault("accel_mag", Running())
    stopping = False

    def on_signal(_signum: int, _frame: Any) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    start = time.monotonic()
    total_n = 0
    still_n = 0
    errors = 0
    last_row: dict[str, Any] | None = None

    try:
        while not stopping:
            loop_start = time.monotonic()
            elapsed = loop_start - start
            if args.duration and elapsed >= args.duration:
                break

            try:
                state = fetch_state(url, args.timeout)
            except (urllib.error.URLError, OSError, json.JSONDecodeError) as exc:
                errors += 1
                if errors <= 3 or errors % 50 == 0:
                    print(f"[{elapsed:7.1f}s] okuma hatasi ({errors}): {exc}", file=sys.stderr)
                time.sleep(min(period, 0.5))
                continue

            row = derive(state, elapsed)
            if row is None:
                errors += 1
                time.sleep(period)
                continue

            total_n += 1
            still_n += row["still"]
            for key in stats:
                value = row.get(key)
                if isinstance(value, (int, float)):
                    stats[key].add(float(value))
            if writer is not None:
                writer.writerow(row)
                if total_n % 20 == 0 and handle is not None:
                    handle.flush()
            last_row = row

            meta = {
                "host": args.host,
                "csv": csv_path if csv_path else "(kayit yok)",
                "poll_hz": total_n / elapsed if elapsed > 0 else 0.0,
            }
            if args.no_tui:
                print(
                    f"[{elapsed:7.1f}s] r {row['roll_deg']:+7.3f} p {row['pitch_deg']:+7.3f}"
                    f" y {row['yaw_deg']:+8.3f} deg | |a| {row['accel_mag']:.4f}"
                    f" | |w| {row['gyro_mag']:.5f} | level offset"
                    f" r {row['roll_error_deg']:+6.3f} p {row['pitch_error_deg']:+6.3f} deg"
                    f" | {'still' if row['still'] else 'MOVING'}",
                    flush=True,
                )
            else:
                sys.stdout.write("\033[H\033[J" + render(row, stats, meta) + "\n")
                sys.stdout.flush()

            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        if handle is not None:
            handle.close()

    if last_row is None:
        print("Hic gecerli ornek alinamadi.", file=sys.stderr)
        return 1

    print(summarise(stats, still_n, total_n))
    if csv_path is not None:
        print(f"\nCSV: {csv_path}  ({total_n} satir)")
    if errors:
        print(f"okuma hatasi sayisi: {errors}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
