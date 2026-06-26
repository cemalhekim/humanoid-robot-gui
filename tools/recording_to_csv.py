#!/usr/bin/env python3
"""Convert dashboard JSONL / pose recordings into CSV tables."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


def rows_from_recording(path: Path) -> list[dict[str, Any]]:
    if path.name.endswith(".pose.json"):
        pose = json.loads(path.read_text(encoding="utf-8"))
        snapshot = pose.get("snapshot") or {}
        return [
            {
                "type": "telemetry_sample",
                "timestamp": snapshot.get("timestamp") or pose.get("timestamp"),
                "sample": snapshot.get("sample") or 1,
                "body": {
                    "motors": snapshot.get("motors") or [],
                    "imu": snapshot.get("imu") or {},
                    "robot": snapshot.get("robot") or {},
                    "battery": snapshot.get("battery") or {},
                    "foot_force": snapshot.get("foot_force") or [],
                    "foot_force_est": snapshot.get("foot_force_est") or [],
                },
                "hands": snapshot.get("hands") or {},
            }
        ]

    records = []
    with path.open(encoding="utf-8") as source:
        for line in source:
            line = line.strip()
            if not line:
                continue
            records.append(json.loads(line))
    return records


def scalar(value: Any) -> Any:
    if isinstance(value, list):
        return "|".join("" if item is None else str(item) for item in value)
    if isinstance(value, dict):
        return json.dumps(value, separators=(",", ":"))
    return value


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: scalar(row.get(key)) for key in fieldnames})


def convert(input_path: Path, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    samples = []
    body_motors = []
    hand_joints = []
    imu_rows = []
    force_rows = []
    events = []

    for record in rows_from_recording(input_path):
        record_type = record.get("type")
        timestamp = record.get("timestamp")
        sample = record.get("sample")
        if record_type == "telemetry_sample":
            body = record.get("body") or {}
            hands = record.get("hands") or {}
            robot = body.get("robot") or {}
            samples.append(
                {
                    "sample": sample,
                    "timestamp": timestamp,
                    "mode_pr": robot.get("mode_pr"),
                    "mode_machine": robot.get("mode_machine"),
                    "tick": robot.get("tick"),
                    "crc": robot.get("crc"),
                }
            )
            for motor in body.get("motors") or []:
                body_motors.append(
                    {
                        "sample": sample,
                        "timestamp": timestamp,
                        "motor_index": motor.get("index"),
                        "motor_name": motor.get("name"),
                        "mode": motor.get("mode"),
                        "q": motor.get("q"),
                        "dq": motor.get("dq"),
                        "ddq": motor.get("ddq"),
                        "tau": motor.get("tau"),
                        "tau_est": motor.get("tau_est"),
                        "temperature": motor.get("temperature"),
                        "vol": motor.get("vol"),
                    }
                )
            for joint in hands.get("joints") or []:
                hand_joints.append(
                    {
                        "sample": sample,
                        "timestamp": timestamp,
                        "hand_index": joint.get("index"),
                        "hand_name": joint.get("name"),
                        "mode": joint.get("mode"),
                        "q": joint.get("q"),
                        "dq": joint.get("dq"),
                        "ddq": joint.get("ddq"),
                        "tau": joint.get("tau"),
                        "tau_est": joint.get("tau_est"),
                        "temperature": joint.get("temperature"),
                        "vol": joint.get("vol"),
                    }
                )
            imu = body.get("imu") or {}
            imu_rows.append(
                {
                    "sample": sample,
                    "timestamp": timestamp,
                    "quaternion": imu.get("quaternion"),
                    "gyroscope": imu.get("gyroscope"),
                    "accelerometer": imu.get("accelerometer"),
                    "rpy": imu.get("rpy"),
                    "temperature": imu.get("temperature"),
                }
            )
            force_rows.append(
                {
                    "sample": sample,
                    "timestamp": timestamp,
                    "foot_force": body.get("foot_force"),
                    "foot_force_est": body.get("foot_force_est"),
                }
            )
        elif record_type == "command_event":
            events.append(
                {
                    "timestamp": timestamp,
                    "name": record.get("name"),
                    "payload_json": json.dumps(record.get("payload") or {}, separators=(",", ":")),
                }
            )

    write_csv(output_dir / "samples.csv", ["sample", "timestamp", "mode_pr", "mode_machine", "tick", "crc"], samples)
    write_csv(
        output_dir / "body_motors.csv",
        ["sample", "timestamp", "motor_index", "motor_name", "mode", "q", "dq", "ddq", "tau", "tau_est", "temperature", "vol"],
        body_motors,
    )
    write_csv(
        output_dir / "hand_joints.csv",
        ["sample", "timestamp", "hand_index", "hand_name", "mode", "q", "dq", "ddq", "tau", "tau_est", "temperature", "vol"],
        hand_joints,
    )
    write_csv(output_dir / "imu.csv", ["sample", "timestamp", "quaternion", "gyroscope", "accelerometer", "rpy", "temperature"], imu_rows)
    write_csv(output_dir / "forces.csv", ["sample", "timestamp", "foot_force", "foot_force_est"], force_rows)
    write_csv(output_dir / "events.csv", ["timestamp", "name", "payload_json"], events)


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert a robot telemetry recording to CSV tables.")
    parser.add_argument("recording", help="Input .jsonl or .pose.json recording")
    parser.add_argument("--output-dir", help="Output directory. Defaults to <recording stem>-csv")
    args = parser.parse_args()

    input_path = Path(args.recording)
    output_dir = Path(args.output_dir) if args.output_dir else input_path.with_suffix("").with_name(f"{input_path.stem}-csv")
    convert(input_path, output_dir)
    print(output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
