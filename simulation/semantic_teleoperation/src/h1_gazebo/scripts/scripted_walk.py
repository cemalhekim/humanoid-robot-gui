#!/usr/bin/env python3

import argparse
import math
import shutil
import subprocess
import time


GAIT_JOINTS = {
    "left_hip_pitch_joint": 0.0,
    "left_knee_joint": 0.0,
    "left_ankle_joint": 0.0,
    "right_hip_pitch_joint": 0.0,
    "right_knee_joint": 0.0,
    "right_ankle_joint": 0.0,
    "left_shoulder_pitch_joint": 0.0,
    "right_shoulder_pitch_joint": 0.0,
    "left_elbow_joint": 0.0,
    "right_elbow_joint": 0.0,
    "torso_joint": 0.0,
}


def run_gz_joint(model, joint, position, p_gain, d_gain):
    subprocess.run(
        [
            "gz",
            "joint",
            "-m",
            model,
            "-j",
            joint,
            "--pos-t",
            f"{position:.5f}",
            "--pos-p",
            f"{p_gain:.3f}",
            "--pos-i",
            "0",
            "--pos-d",
            f"{d_gain:.3f}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def wait_for_model(model, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        result = subprocess.run(
            ["gz", "model", "-m", model, "-i"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        if result.returncode == 0:
            return True
        time.sleep(0.25)
    return False


def gait_positions(t, frequency):
    phase = 2.0 * math.pi * frequency * t
    left = math.sin(phase)
    right = math.sin(phase + math.pi)
    left_lift = max(0.0, left)
    right_lift = max(0.0, right)

    return {
        "left_hip_pitch_joint": -0.28 * left,
        "left_knee_joint": 0.55 * left_lift,
        "left_ankle_joint": -0.24 * left_lift + 0.06 * left,
        "right_hip_pitch_joint": -0.28 * right,
        "right_knee_joint": 0.55 * right_lift,
        "right_ankle_joint": -0.24 * right_lift + 0.06 * right,
        "left_shoulder_pitch_joint": 0.32 * right,
        "right_shoulder_pitch_joint": 0.32 * left,
        "left_elbow_joint": 0.35 + 0.08 * left_lift,
        "right_elbow_joint": 0.35 + 0.08 * right_lift,
        "torso_joint": 0.04 * math.sin(phase),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="h1")
    parser.add_argument("--frequency", type=float, default=0.65)
    parser.add_argument("--rate", type=float, default=12.0)
    parser.add_argument("--p-gain", type=float, default=120.0)
    parser.add_argument("--d-gain", type=float, default=4.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    args, _ = parser.parse_known_args()

    if shutil.which("gz") is None:
        raise SystemExit("The 'gz' command was not found.")

    if not wait_for_model(args.model, args.timeout):
        raise SystemExit(f"Timed out waiting for Gazebo model '{args.model}'.")

    print(
        f"Starting scripted walking animation for '{args.model}'. "
        "This is a visual gait, not a Unitree hardware controller."
    )

    period = 1.0 / max(args.rate, 1.0)
    start = time.time()
    while True:
        t = time.time() - start
        positions = gait_positions(t, args.frequency)
        for joint, position in positions.items():
            run_gz_joint(args.model, joint, position, args.p_gain, args.d_gain)
        time.sleep(period)


if __name__ == "__main__":
    main()
