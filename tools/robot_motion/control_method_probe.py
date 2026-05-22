#!/usr/bin/env python3
"""Probe Unitree H1-2 control methods one at a time.

Default actions are read-only. Motion actions require explicit flags.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


SDK_PATHS = [
    Path.home() / "Workspace/semantic-teleoperation/execution/external/unitree_sdk2_python",
    Path.home() / "unitree_sdk2_python",
]
for sdk_path in SDK_PATHS:
    if sdk_path.exists():
        sys.path.insert(0, str(sdk_path))

try:
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
except ImportError as exc:
    print(f"Missing Unitree SDK2 Python: {exc}", file=sys.stderr)
    raise SystemExit(2)


RIGHT_WRIST_YAW = 26
ARM_SDK_WEIGHT_SLOT = 27
ARM_SDK_JOINTS = [13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 12]
ARM_SDK_KP = [120, 120, 80, 50, 50, 50, 50, 120, 120, 80, 50, 50, 50, 50, 200]
ARM_SDK_KD = [2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 2.0]


class LowStateOnce:
    def __init__(self) -> None:
        self.msg = None

    def update(self, msg) -> None:
        self.msg = msg

    def wait(self, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.msg is not None:
                return self.msg
            time.sleep(0.02)
        return None


def print_json(data) -> None:
    print(json.dumps(data, indent=2, sort_keys=True))


def read_lowstate(timeout: float):
    state = LowStateOnce()
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(state.update, 10)
    return state.wait(timeout)


def check_motion_switcher(timeout: float) -> int:
    client = MotionSwitcherClient()
    client.SetTimeout(timeout)
    client.Init()
    code, result = client.CheckMode()
    print_json({"method": "motion_switcher.check_mode", "code": code, "result": result})
    return 0 if code == 0 else 1


def release_motion_switcher(timeout: float) -> int:
    client = MotionSwitcherClient()
    client.SetTimeout(timeout)
    client.Init()
    before_code, before = client.CheckMode()
    rel_code, rel_result = client.ReleaseMode()
    time.sleep(0.5)
    after_code, after = client.CheckMode()
    print_json(
        {
            "method": "motion_switcher.release_mode",
            "before": {"code": before_code, "result": before},
            "release": {"code": rel_code, "result": rel_result},
            "after": {"code": after_code, "result": after},
        }
    )
    return 0 if rel_code == 0 else 1


def select_motion_mode(timeout: float, mode: str) -> int:
    client = MotionSwitcherClient()
    client.SetTimeout(timeout)
    client.Init()
    before_code, before = client.CheckMode()
    select_code, select_result = client.SelectMode(mode)
    time.sleep(0.5)
    after_code, after = client.CheckMode()
    print_json(
        {
            "method": "motion_switcher.select_mode",
            "mode": mode,
            "before": {"code": before_code, "result": before},
            "select": {"code": select_code, "result": select_result},
            "after": {"code": after_code, "result": after},
        }
    )
    return 0 if select_code == 0 else 1


def check_loco(timeout: float) -> int:
    client = LocoClient()
    client.SetTimeout(timeout)
    client.Init()
    # H1 Python client exposes setters, but not getters. Use zero-duration stop
    # only in explicit motion mode elsewhere; this read-only check just verifies init.
    print_json({"method": "loco.init", "ok": True, "note": "client initialized; no motion command sent"})
    return 0


def build_arm_sdk_cmd(msg, target_q: float, kp: float, kd: float) -> LowCmd_:
    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = int(getattr(msg, "mode_pr", 0) or 0)
    cmd.mode_machine = int(getattr(msg, "mode_machine", 0) or 0)
    for i in range(35):
        motor = cmd.motor_cmd[i]
        motor.mode = 0
        motor.q = 0.0
        motor.dq = 0.0
        motor.tau = 0.0
        motor.kp = 0.0
        motor.kd = 0.0
        motor.reserve = 0

    # Official H1-2 arm_sdk example uses kNotUsedJoint index 27 q as the
    # arm_sdk control weight: 1 enables arm SDK, 0 releases it.
    cmd.motor_cmd[ARM_SDK_WEIGHT_SLOT].q = 1.0

    for joint, hold_kp, hold_kd in zip(ARM_SDK_JOINTS, ARM_SDK_KP, ARM_SDK_KD):
        motor = cmd.motor_cmd[joint]
        motor.mode = 1
        motor.q = float(msg.motor_state[joint].q)
        motor.dq = 0.0
        motor.tau = 0.0
        motor.kp = hold_kp
        motor.kd = hold_kd

    wrist = cmd.motor_cmd[RIGHT_WRIST_YAW]
    wrist.mode = 1
    wrist.q = target_q
    wrist.dq = 0.0
    wrist.tau = 0.0
    wrist.kp = kp
    wrist.kd = kd
    cmd.crc = CRC().Crc(cmd)
    return cmd


def arm_sdk_nudge(args: argparse.Namespace) -> int:
    msg = read_lowstate(args.timeout)
    if msg is None:
        print("No rt/lowstate received; not publishing.", file=sys.stderr)
        return 1

    start_q = float(msg.motor_state[RIGHT_WRIST_YAW].q)
    target_q = start_q + args.delta
    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()
    cmd = build_arm_sdk_cmd(msg, target_q, args.kp, args.kd)
    print_json(
        {
            "method": "arm_sdk.nudge",
            "start_q": start_q,
            "target_q": target_q,
            "kp": args.kp,
            "kd": args.kd,
            "seconds": args.seconds,
        }
    )
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        pub.Write(cmd)
        time.sleep(0.02)

    time.sleep(0.3)
    after = read_lowstate(args.timeout)
    if after is None:
        print("No post-test lowstate received.", file=sys.stderr)
        return 1
    after_q = float(after.motor_state[RIGHT_WRIST_YAW].q)
    print_json({"observed_q": after_q, "observed_delta": after_q - start_q})
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("interface")
    parser.add_argument(
        "method",
        choices=[
            "motion-check",
            "motion-release",
            "motion-select",
            "loco-check",
            "arm-sdk-nudge",
        ],
    )
    parser.add_argument("--mode", default="ai")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--delta", type=float, default=0.03)
    parser.add_argument("--seconds", type=float, default=0.5)
    parser.add_argument("--kp", type=float, default=4.0)
    parser.add_argument("--kd", type=float, default=0.35)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--i-understand-risk", action="store_true")
    args = parser.parse_args()
    motion_methods = {"motion-release", "motion-select", "arm-sdk-nudge"}
    if args.method in motion_methods and not (args.execute and args.i_understand_risk):
        parser.error(f"{args.method} requires --execute --i-understand-risk")
    return args


def main() -> int:
    args = parse_args()
    ChannelFactoryInitialize(args.domain_id, args.interface)
    if args.method == "motion-check":
        return check_motion_switcher(args.timeout)
    if args.method == "motion-release":
        return release_motion_switcher(args.timeout)
    if args.method == "motion-select":
        return select_motion_mode(args.timeout, args.mode)
    if args.method == "loco-check":
        return check_loco(args.timeout)
    if args.method == "arm-sdk-nudge":
        return arm_sdk_nudge(args)
    raise ValueError(args.method)


if __name__ == "__main__":
    raise SystemExit(main())
