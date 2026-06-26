#!/usr/bin/env python3
"""Generate synthetic H1-2 telemetry recordings for dashboard replay tests."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import server


def motor_row(index: int, name: str, q: float, dq: float, t: float) -> dict[str, object]:
    return {
        "index": index,
        "name": name,
        "mode": 1,
        "q": round(q, 6),
        "dq": round(dq, 6),
        "ddq": 0.0,
        "tau_est": round(0.6 * math.sin(t + index * 0.17), 6),
        "temperature": [34 + (index % 5), 34 + (index % 5)],
        "vol": 24.0,
        "sensor": [],
        "reserve": 0,
    }


def hand_row(index: int, name: str, q: float, dq: float, t: float) -> dict[str, object]:
    return {
        "index": index,
        "name": name,
        "mode": 1,
        "q": round(q, 6),
        "dq": round(dq, 6),
        "tau_est": round(0.08 * math.sin(t + index), 6),
        "temperature": [30 + (index % 3)],
        "vol": 12.0,
        "sensor": [],
        "reserve": 0,
    }


def synthetic_snapshot(frame: int, frames: int, start_time: float) -> dict[str, object]:
    phase = (frame / max(1, frames - 1)) * math.tau
    timestamp = start_time + frame / 60.0
    motors = []
    for index, name in server.JOINT_NAMES.items():
        leg_scale = 0.16 if index < 12 else 0.0
        waist_scale = 0.08 if index == 12 else 0.0
        arm_scale = 0.28 if index >= 13 else 0.0
        amplitude = leg_scale + waist_scale + arm_scale
        offset = index * 0.23
        q = amplitude * math.sin(phase + offset)
        dq = amplitude * math.cos(phase + offset) * math.tau
        motors.append(motor_row(index, name, q, dq, phase))

    hand_joints = []
    for index, name in server.HAND_JOINT_NAMES.items():
        q = 0.45 + 0.35 * math.sin(phase + index * 0.31)
        dq = 0.35 * math.cos(phase + index * 0.31) * math.tau
        hand_joints.append(hand_row(index, name, q, dq, phase))

    return {
        "connected": False,
        "timestamp": timestamp,
        "sample": frame + 1,
        "samples": frame + 1,
        "sample_rate_hz": 60,
        "motor_count": len(motors),
        "motors": motors,
        "imu": {
            "quaternion": [1, 0, 0, 0],
            "gyroscope": [0.0, 0.0, round(0.05 * math.sin(phase), 6)],
            "accelerometer": [0.0, 0.0, 9.81],
            "rpy": [round(0.03 * math.sin(phase), 6), round(0.02 * math.cos(phase), 6), 0.0],
            "temperature": 36.5,
        },
        "robot": {"mode_pr": 0, "mode_machine": 0, "tick": frame, "crc": 0},
        "battery": {"soc": 92, "current": 0.0, "temperature": [32, 33]},
        "foot_force": [round(18 + 2 * math.sin(phase + i), 4) for i in range(4)],
        "foot_force_est": [round(17 + 2 * math.cos(phase + i), 4) for i in range(4)],
        "hands": {
            "connected": True,
            "topic": "rt/inspire/state",
            "samples": frame + 1,
            "timestamp": timestamp,
            "joint_count": len(hand_joints),
            "joints": hand_joints,
        },
    }


def write_sequence(path: Path, frames: int) -> None:
    start_time = time.time()
    with path.open("w", encoding="utf-8") as output:
        output.write(
            json.dumps(
                {
                    "type": "recording_start",
                    "timestamp": start_time,
                    "monotonic_ns": time.monotonic_ns(),
                    "schema": "h1_2_telemetry_jsonl_v1",
                    "synthetic": True,
                    "body_joint_names": server.JOINT_NAMES,
                    "hand_joint_names": server.HAND_JOINT_NAMES,
                },
                separators=(",", ":"),
            )
            + "\n"
        )
        for frame in range(frames):
            snapshot = synthetic_snapshot(frame, frames, start_time)
            output.write(
                json.dumps(
                    {
                        "type": "telemetry_sample",
                        "timestamp": snapshot["timestamp"],
                        "monotonic_ns": time.monotonic_ns() + frame,
                        "sample": frame + 1,
                        "body": {
                            "topic": "rt/lowstate",
                            "motors": snapshot["motors"],
                            "imu": snapshot["imu"],
                            "robot": snapshot["robot"],
                            "battery": snapshot["battery"],
                            "foot_force": snapshot["foot_force"],
                            "foot_force_est": snapshot["foot_force_est"],
                        },
                        "hands": snapshot["hands"],
                        "hand_samples": frame + 1,
                        "hand_timestamp": snapshot["timestamp"],
                    },
                    separators=(",", ":"),
                )
                + "\n"
            )
        output.write(
            json.dumps(
                {
                    "type": "recording_stop",
                    "timestamp": start_time + frames / 60.0,
                    "monotonic_ns": time.monotonic_ns(),
                    "samples": frames,
                    "events": 0,
                },
                separators=(",", ":"),
            )
            + "\n"
        )


def write_pose(path: Path, frames: int) -> None:
    snapshot = synthetic_snapshot(frames - 1, frames, time.time())
    path.write_text(
        json.dumps(
            {
                "type": "pose_point",
                "schema": "h1_2_pose_point_v1",
                "timestamp": time.time(),
                "monotonic_ns": time.monotonic_ns(),
                "synthetic": True,
                "snapshot": snapshot,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate synthetic dashboard replay recordings.")
    parser.add_argument("--output-dir", default=str(server.RECORDINGS_DIR))
    parser.add_argument("--frames", type=int, default=240)
    parser.add_argument("--prefix", default="synthetic-h1-2")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = server.recording_timestamp()
    sequence_path = output_dir / f"{stamp}-{args.prefix}-sequence.jsonl"
    pose_path = output_dir / f"{stamp}-{args.prefix}-pose.pose.json"
    write_sequence(sequence_path, max(2, args.frames))
    write_pose(pose_path, max(2, args.frames))
    print(sequence_path)
    print(pose_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
