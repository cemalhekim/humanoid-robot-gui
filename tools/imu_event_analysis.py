#!/usr/bin/env python3
"""Analyse a full-rate telemetry recording around a lean / near-fall / recovery.

The dashboard's recorder writes every DDS sample (~500 Hz) to JSONL, so a
walking run that nearly topples and catches itself is captured in full. This
tool streams that file (never loading it into memory -- a few minutes of
walking is ~1 GB) and reports:

  * the disturbance itself: onset, peak lean, peak angular rate, how long the
    robot took to come back, and which leg joints did the work
  * quiet segments before and after, with the gravity-referenced level offset
    computed in each one

That second part is the reason to run this after a near-fall. The level offset
(rpy minus the tilt gravity implies) is only measurable while the robot is
motionless, so a violent event cannot be judged directly -- but comparing the
offset in the quiet stretch before the event against the one after it shows
whether the disturbance permanently shifted the IMU's orientation estimate. A
static offset that survives a shake is a calibration constant; one that jumps
means the filter is being corrupted by dynamics, which is a different fault
with a different fix.

Examples:
    tools/imu_event_analysis.py --fetch-latest
    tools/imu_event_analysis.py recordings/20260831-140154-walk.jsonl
    tools/imu_event_analysis.py run.jsonl --event-csv /tmp/event.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Iterator

sys.path.insert(0, str(Path(__file__).resolve().parent))
from imu_monitor import GRAVITY, Running, quat_to_rpy, tilt_from_accel, wrap_deg  # noqa: E402

DEFAULT_HOST = "192.168.123.164:8088"
REPO_ROOT = Path(__file__).resolve().parent.parent

# A lean this far past the resting baseline counts as the disturbance starting.
EVENT_TILT_DEG = 5.0
# The robot is back under control once it is within this band of baseline again.
RECOVERY_BAND_DEG = 1.5
# ...and stays there this long. Prevents calling a recovery mid-oscillation.
RECOVERY_HOLD_S = 0.5
# Angular rate below which gravity is a trustworthy tilt reference.
QUIET_GYRO_RAD_S = 0.05
# Shortest run of quiet samples worth reporting a level offset for.
MIN_QUIET_S = 1.0

LEG_JOINTS = [
    "LeftHipYaw", "LeftHipPitch", "LeftHipRoll", "LeftKnee", "LeftAnklePitch", "LeftAnkleRoll",
    "RightHipYaw", "RightHipPitch", "RightHipRoll", "RightKnee", "RightAnklePitch", "RightAnkleRoll",
    "WaistYaw",
]

EVENT_CSV_COLUMNS = [
    "t", "tilt_deg", "roll_deg", "pitch_deg", "yaw_deg",
    "gyro_x", "gyro_y", "gyro_z", "gyro_mag",
    "accel_x", "accel_y", "accel_z", "accel_mag",
    "temperature",
]


def tilt_from_rpy(roll: float, pitch: float) -> float:
    """Angle between the body's up-axis and true vertical, in degrees."""
    cos_tilt = max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))
    return math.degrees(math.acos(cos_tilt))


def iter_samples(path: Path) -> Iterator[dict[str, Any]]:
    """Yield derived per-sample dicts, streaming the file line by line."""
    t0: float | None = None
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue
            if record.get("type") != "telemetry_sample":
                continue
            body = record.get("body") or {}
            imu = body.get("imu") or {}
            quat = list(imu.get("quaternion") or [])
            gyro = list(imu.get("gyroscope") or [])
            accel = list(imu.get("accelerometer") or [])
            rpy = list(imu.get("rpy") or [])
            if len(quat) < 4 or len(gyro) < 3 or len(accel) < 3 or len(rpy) < 3:
                continue

            timestamp = record.get("timestamp") or 0.0
            if t0 is None:
                t0 = timestamp
            roll, pitch, yaw = rpy[0], rpy[1], rpy[2]
            roll_acc, pitch_acc = tilt_from_accel(*accel)
            gyro_mag = math.sqrt(sum(v * v for v in gyro))

            yield {
                "t": timestamp - t0,
                "timestamp": timestamp,
                "roll": roll, "pitch": pitch, "yaw": yaw,
                "roll_deg": math.degrees(roll),
                "pitch_deg": math.degrees(pitch),
                "yaw_deg": math.degrees(yaw),
                "tilt_deg": tilt_from_rpy(roll, pitch),
                "gyro_x": gyro[0], "gyro_y": gyro[1], "gyro_z": gyro[2],
                "gyro_mag": gyro_mag,
                "accel_x": accel[0], "accel_y": accel[1], "accel_z": accel[2],
                "accel_mag": math.sqrt(sum(v * v for v in accel)),
                "roll_error_deg": wrap_deg(math.degrees(roll - roll_acc)),
                "pitch_error_deg": wrap_deg(math.degrees(pitch - pitch_acc)),
                "quat_drift_deg": max(
                    abs(wrap_deg(math.degrees(a - b)))
                    for a, b in zip((roll, pitch, yaw), quat_to_rpy(*quat[:4]))
                ),
                "quiet": gyro_mag < QUIET_GYRO_RAD_S,
                "motors": body.get("motors") or [],
            }


class QuietSegment:
    def __init__(self, start: float) -> None:
        self.start = start
        self.end = start
        self.n = 0
        self.stats = {
            key: Running()
            for key in ("roll_deg", "pitch_deg", "yaw_deg", "accel_mag", "roll_error_deg", "pitch_error_deg")
        }

    def add(self, row: dict[str, Any]) -> None:
        self.end = row["t"]
        self.n += 1
        for key, acc in self.stats.items():
            acc.add(row[key])

    @property
    def duration(self) -> float:
        return self.end - self.start


def analyse(path: Path, tilt_threshold: float) -> dict[str, Any]:
    overall = {key: Running() for key in ("tilt_deg", "gyro_mag", "accel_mag", "quat_drift_deg")}
    quiet_segments: list[QuietSegment] = []
    current_quiet: QuietSegment | None = None

    baseline_acc = Running()
    peak: dict[str, Any] | None = None
    peak_gyro: dict[str, Any] | None = None
    joint_peak: dict[str, dict[str, float]] = {name: {"tau": 0.0, "dq": 0.0} for name in LEG_JOINTS}

    onset_t: float | None = None
    recovery_t: float | None = None
    below_since: float | None = None
    n = 0
    first_t = None
    last_t = 0.0

    for row in iter_samples(path):
        n += 1
        if first_t is None:
            first_t = row["t"]
        last_t = row["t"]
        for key, acc in overall.items():
            acc.add(row[key])

        # Baseline is the resting lean, taken from quiet samples seen before the
        # disturbance starts.
        if row["quiet"] and onset_t is None:
            baseline_acc.add(row["tilt_deg"])

        if row["quiet"]:
            if current_quiet is None:
                current_quiet = QuietSegment(row["t"])
            current_quiet.add(row)
        elif current_quiet is not None:
            if current_quiet.duration >= MIN_QUIET_S:
                quiet_segments.append(current_quiet)
            current_quiet = None

        baseline = baseline_acc.mean if baseline_acc.n else 0.0
        excess = row["tilt_deg"] - baseline

        if onset_t is None and excess > tilt_threshold:
            onset_t = row["t"]
        if onset_t is not None and recovery_t is None:
            if peak is None or row["tilt_deg"] > peak["tilt_deg"]:
                peak = dict(row)
                peak.pop("motors", None)
            if peak_gyro is None or row["gyro_mag"] > peak_gyro["gyro_mag"]:
                peak_gyro = dict(row)
                peak_gyro.pop("motors", None)
            for motor in row["motors"]:
                name = motor.get("name")
                if name in joint_peak:
                    joint_peak[name]["tau"] = max(joint_peak[name]["tau"], abs(motor.get("tau_est") or 0.0))
                    joint_peak[name]["dq"] = max(joint_peak[name]["dq"], abs(motor.get("dq") or 0.0))
            if excess <= RECOVERY_BAND_DEG:
                if below_since is None:
                    below_since = row["t"]
                elif row["t"] - below_since >= RECOVERY_HOLD_S:
                    recovery_t = below_since
            else:
                below_since = None

    if current_quiet is not None and current_quiet.duration >= MIN_QUIET_S:
        quiet_segments.append(current_quiet)

    duration = (last_t - first_t) if first_t is not None else 0.0
    return {
        "samples": n,
        "duration": duration,
        "rate_hz": n / duration if duration > 0 else 0.0,
        "overall": overall,
        "baseline_tilt": baseline_acc.mean if baseline_acc.n else None,
        "baseline_n": baseline_acc.n,
        "quiet_segments": quiet_segments,
        "onset_t": onset_t,
        "recovery_t": recovery_t,
        "peak": peak,
        "peak_gyro": peak_gyro,
        "joint_peak": joint_peak,
    }


def write_event_csv(path: Path, out: Path, start: float, end: float) -> int:
    out.parent.mkdir(parents=True, exist_ok=True)
    written = 0
    with out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=EVENT_CSV_COLUMNS)
        writer.writeheader()
        for row in iter_samples(path):
            if row["t"] < start:
                continue
            if row["t"] > end:
                break
            writer.writerow({key: row[key] for key in EVENT_CSV_COLUMNS})
            written += 1
    return written


def report(result: dict[str, Any], path: Path, tilt_threshold: float) -> str:
    lines = [
        "=" * 78,
        f"OLAY ANALIZI  {path.name}",
        "=" * 78,
        f"  {result['samples']} ornek, {result['duration']:.2f} s, {result['rate_hz']:.1f} Hz",
    ]
    if result["baseline_tilt"] is not None:
        lines.append(f"  dinlenme egimi (baseline): {result['baseline_tilt']:.3f} deg"
                     f"  ({result['baseline_n']} sakin ornek)")
    else:
        lines.append("  UYARI: olay oncesi sakin ornek yok, baseline 0 kabul edildi.")

    overall = result["overall"]
    lines += [
        "",
        "TUM KAYIT",
        f"  egim       ort {overall['tilt_deg'].mean:7.3f}  max {overall['tilt_deg'].max:7.3f} deg",
        f"  |gyro|     ort {overall['gyro_mag'].mean:7.4f}  max {overall['gyro_mag'].max:7.4f} rad/s"
        f"  ({math.degrees(overall['gyro_mag'].max):.1f} deg/s)",
        f"  |accel|    ort {overall['accel_mag'].mean:7.4f}  max {overall['accel_mag'].max:7.4f} m/s2"
        f"  ({overall['accel_mag'].max / GRAVITY:.2f} g)",
        f"  quat/rpy   max sapma {overall['quat_drift_deg'].max:.4f} deg",
    ]

    lines += ["", "OLAY"]
    if result["onset_t"] is None:
        lines.append(f"  Baseline uzerine {tilt_threshold:.1f} deg asan bir olay bulunamadi.")
        lines.append("  --tilt-threshold ile esigi dusurup tekrar dene.")
    else:
        peak = result["peak"]
        peak_gyro = result["peak_gyro"]
        lines.append(f"  baslangic      t = {result['onset_t']:.3f} s")
        lines.append(f"  tepe egim      t = {peak['t']:.3f} s   {peak['tilt_deg']:.3f} deg"
                     f"  (roll {peak['roll_deg']:+.2f}, pitch {peak['pitch_deg']:+.2f})")
        lines.append(f"  tepe donme     t = {peak_gyro['t']:.3f} s   {peak_gyro['gyro_mag']:.4f} rad/s"
                     f"  = {math.degrees(peak_gyro['gyro_mag']):.1f} deg/s")
        if result["recovery_t"] is not None:
            lines.append(f"  toparlanma     t = {result['recovery_t']:.3f} s")
            lines.append(f"  sapma suresi   {result['recovery_t'] - result['onset_t']:.3f} s"
                         f"  (tepeden itibaren {result['recovery_t'] - peak['t']:.3f} s)")
        else:
            lines.append("  toparlanma     BULUNAMADI - kayit robot baseline'a donmeden bitmis")

        ranked = sorted(result["joint_peak"].items(), key=lambda kv: -kv[1]["tau"])
        lines += ["", "  Olay boyunca en cok zorlanan eklemler (|tau_est| tepe, Nm):"]
        for name, vals in ranked[:8]:
            if vals["tau"] <= 0:
                continue
            lines.append(f"    {name:<18}{vals['tau']:8.2f} Nm   |dq| tepe {vals['dq']:7.3f} rad/s")

    lines += ["", "SAKIN SEGMENTLER (level offset yalnizca burada olculebilir)"]
    segments = result["quiet_segments"]
    if not segments:
        lines.append(f"  {MIN_QUIET_S:.1f} s'den uzun sakin bolum yok - offset karsilastirmasi yapilamadi.")
    else:
        lines.append(f"  {'t_bas':>8}{'t_son':>8}{'sure':>7}"
                     f"{'roll off':>11}{'pitch off':>11}{'|accel|':>10}   (offset = rpy - yercekimi)")
        for seg in segments:
            lines.append(
                f"  {seg.start:8.2f}{seg.end:8.2f}{seg.duration:7.2f}"
                f"{seg.stats['roll_error_deg'].mean:11.3f}{seg.stats['pitch_error_deg'].mean:11.3f}"
                f"{seg.stats['accel_mag'].mean:10.4f}"
            )
        if len(segments) >= 2:
            first, last = segments[0], segments[-1]
            d_roll = last.stats["roll_error_deg"].mean - first.stats["roll_error_deg"].mean
            d_pitch = last.stats["pitch_error_deg"].mean - first.stats["pitch_error_deg"].mean
            noise = math.hypot(first.stats["pitch_error_deg"].sigma / math.sqrt(max(first.n, 1)),
                               last.stats["pitch_error_deg"].sigma / math.sqrt(max(last.n, 1)))
            lines += [
                "",
                "  ILK -> SON SAKIN SEGMENT DEGISIMI",
                f"    roll offset  {d_roll:+.3f} deg",
                f"    pitch offset {d_pitch:+.3f} deg   (olcum belirsizligi ~{noise:.3f} deg)",
            ]
            if abs(d_pitch) > 3 * max(noise, 1e-6):
                lines += [
                    "",
                    "    Offset olaydan sonra kaydi. Bu sabit bir montaj/level hatasi degil:",
                    "    yonelim filtresi dinamik bozulmadan kalici sekilde etkileniyor.",
                    "    Statik rekalibrasyon bunu kalici olarak duzeltmez.",
                ]
            else:
                lines += [
                    "",
                    "    Offset olaydan etkilenmedi. Sabit bir level hatasi gibi davraniyor,",
                    "    yani statik rekalibrasyon dogru mudahale.",
                ]
    return "\n".join(lines)


def fetch_recording(host: str, name: str | None, dest_dir: Path) -> Path:
    base = f"http://{host}"
    if name is None:
        with urllib.request.urlopen(f"{base}/api/recording/files", timeout=10) as response:
            files = json.loads(response.read().decode("utf-8")).get("files") or []
        jsonl = [f for f in files if str(f.get("name", "")).endswith(".jsonl")]
        if not jsonl:
            raise SystemExit("Robotta .jsonl kaydi yok.")
        jsonl.sort(key=lambda f: -(f.get("modified_at") or 0))
        name = jsonl[0]["name"]
        print(f"en yeni kayit: {name}  ({jsonl[0].get('size', 0) / 1e6:.1f} MB)")

    dest_dir.mkdir(parents=True, exist_ok=True)
    dest = dest_dir / name
    if dest.exists():
        print(f"zaten indirilmis: {dest}")
        return dest
    print(f"indiriliyor -> {dest}")
    urllib.request.urlretrieve(f"{base}/api/recording/files/{name}", dest)
    return dest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recording", nargs="?", type=Path, help="local .jsonl recording")
    parser.add_argument("--fetch-latest", action="store_true", help="download the newest .jsonl from the robot")
    parser.add_argument("--fetch", metavar="NAME", help="download this recording by name from the robot")
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"dashboard host:port (default {DEFAULT_HOST})")
    parser.add_argument("--tilt-threshold", type=float, default=EVENT_TILT_DEG,
                        help=f"lean past baseline that starts an event, deg (default {EVENT_TILT_DEG})")
    parser.add_argument("--event-csv", type=Path, help="write the full-rate event window to this CSV")
    parser.add_argument("--pad", type=float, default=2.0, help="seconds of context around the event CSV (default 2)")
    args = parser.parse_args(argv)

    if args.fetch_latest or args.fetch:
        try:
            path = fetch_recording(args.host, args.fetch, REPO_ROOT / "recordings" / "imu")
        except (urllib.error.URLError, OSError) as exc:
            print(f"indirme basarisiz: {exc}", file=sys.stderr)
            return 1
    elif args.recording:
        path = args.recording
    else:
        parser.error("bir kayit dosyasi ver ya da --fetch-latest kullan")

    if not path.exists():
        print(f"dosya yok: {path}", file=sys.stderr)
        return 1

    result = analyse(path, args.tilt_threshold)
    if result["samples"] == 0:
        print("Kayitta telemetry_sample bulunamadi.", file=sys.stderr)
        return 1
    print(report(result, path, args.tilt_threshold))

    if args.event_csv and result["onset_t"] is not None:
        end = result["recovery_t"] if result["recovery_t"] is not None else result["duration"]
        start = max(0.0, result["onset_t"] - args.pad)
        written = write_event_csv(path, args.event_csv, start, end + args.pad)
        print(f"\nolay penceresi CSV: {args.event_csv}  ({written} satir, {start:.2f}-{end + args.pad:.2f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
