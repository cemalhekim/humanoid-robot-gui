#!/usr/bin/env python3
"""Oscillate one H1-2 right wrist joint until Ctrl-C.

This is a real low-level command publisher. It uses the H1-2/unitree_hg
35-motor LowCmd message, subscribes to rt/lowstate, and only applies gains to
one right wrist motor. All other motors are sent with stop sentinels and zero
gains so this script does not intentionally take over the full body.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path


REPO_DIR = Path(__file__).resolve().parents[2]
SDK_PATHS = [
    REPO_DIR / "execution/semantic_teleoperation/external/unitree_sdk2_python",
    REPO_DIR / "vendor/unitree_sdk2_python",
    Path.home() / "unitree_sdk2_python",
]
for sdk_path in SDK_PATHS:
    if sdk_path.exists():
        sys.path.insert(0, str(sdk_path))

try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
except ImportError as exc:
    print(
        "Missing Unitree SDK2 Python. Run this on the robot PC or set PYTHONPATH "
        "to the unitree_sdk2_python checkout.\n"
        f"Import error: {exc}",
        file=sys.stderr,
    )
    raise SystemExit(2)


POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0
NUM_MOTORS = 35

RIGHT_WRIST = {
    "roll": 24,
    "pitch": 25,
    "yaw": 26,
}

DEFAULT_LIMITS = {
    # Very small defaults for first contact. Pass --min/--max only after
    # checking that the motion is quiet and mechanically free.
    "roll": (-0.25, 0.25),
    "pitch": (-0.2, 0.2),
    "yaw": (-0.25, 0.25),
}


class LowStateBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._msg = None

    def update(self, msg) -> None:
        with self._lock:
            self._msg = msg

    def snapshot(self):
        with self._lock:
            return self._msg

    def wait(self, timeout: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.snapshot()
            if msg is not None:
                return msg
            time.sleep(0.02)
        return None


def smoothstep(alpha: float) -> float:
    return 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(1.0, alpha)))


def blend(a: float, b: float, alpha: float) -> float:
    return a + (b - a) * smoothstep(alpha)


class RightWristSpinner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.motor_index = RIGHT_WRIST[args.joint]
        self.crc = CRC()
        self.state = LowStateBuffer()

        ChannelFactoryInitialize(args.domain_id, args.interface)
        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_sub.Init(self.state.update, 10)
        self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_pub.Init()

    def make_command(self, q: float, *, enabled: bool = True) -> LowCmd_:
        cmd = unitree_hg_msg_dds__LowCmd_()
        msg = self.state.snapshot()
        cmd.mode_pr = self.args.mode_pr if self.args.mode_pr is not None else int(getattr(msg, "mode_pr", 0) or 0)
        cmd.mode_machine = (
            self.args.mode_machine
            if self.args.mode_machine is not None
            else int(getattr(msg, "mode_machine", 0) or 0)
        )

        for i in range(NUM_MOTORS):
            motor = cmd.motor_cmd[i]
            motor.mode = 0
            motor.q = POS_STOP_F
            motor.dq = VEL_STOP_F
            motor.tau = 0.0
            motor.kp = 0.0
            motor.kd = 0.0
            motor.reserve = 0

        wrist = cmd.motor_cmd[self.motor_index]
        wrist.mode = 1 if enabled else 0
        wrist.q = q
        wrist.dq = 0.0
        wrist.tau = 0.0
        wrist.kp = self.args.kp if enabled else 0.0
        wrist.kd = self.args.kd if enabled else 0.0

        cmd.crc = self.crc.Crc(cmd)
        return cmd

    def publish(self, q: float) -> None:
        self.lowcmd_pub.Write(self.make_command(q))

    def relax_wrist(self, q: float) -> None:
        cmd = self.make_command(q, enabled=False)
        for _ in range(20):
            self.lowcmd_pub.Write(cmd)
            time.sleep(0.01)

    def current_q(self) -> float | None:
        msg = self.state.snapshot()
        if msg is None:
            return None
        return float(msg.motor_state[self.motor_index].q)

    def nudge(self, current_q: float) -> int:
        target_q = current_q + self.args.nudge_delta
        target_q = max(self.args.min_q, min(self.args.max_q, target_q))
        print(
            f"Nudge test: current {current_q:.6f}, target {target_q:.6f}, "
            f"duration {self.args.nudge_seconds:.2f}s."
        )
        end = time.monotonic() + self.args.nudge_seconds
        while time.monotonic() < end:
            self.publish(target_q)
            time.sleep(1.0 / self.args.rate)
        time.sleep(0.2)
        observed_q = self.current_q()
        self.relax_wrist(target_q)
        if observed_q is None:
            print("Nudge sent, but no lowstate was available afterwards.")
            return 1
        print(f"Observed after nudge: q={observed_q:.6f}, delta={observed_q - current_q:.6f}.")
        return 0

    def ramp_to_start(self, current_q: float, target_q: float) -> None:
        steps = max(1, int(self.args.ramp_seconds * self.args.rate))
        dt = 1.0 / self.args.rate
        for step in range(steps):
            q = blend(current_q, target_q, (step + 1) / steps)
            self.publish(q)
            time.sleep(dt)

    def run(self) -> int:
        msg = self.state.wait(self.args.wait_timeout)
        if msg is None:
            print("No rt/lowstate received. Not publishing commands.", file=sys.stderr)
            return 1

        current_q = float(msg.motor_state[self.motor_index].q)
        if self.args.probe:
            mode_pr = getattr(msg, "mode_pr", None)
            mode_machine = getattr(msg, "mode_machine", None)
            motor = msg.motor_state[self.motor_index]
            print(
                f"Probe ok: right wrist {self.args.joint} motor {self.motor_index}, "
                f"q={float(motor.q):.6f}, dq={float(motor.dq):.6f}, mode={int(motor.mode)}, "
                f"mode_pr={mode_pr}, mode_machine={mode_machine}."
            )
            return 0
        if self.args.nudge:
            return self.nudge(current_q)

        print(
            f"Connected. Commanding right wrist {self.args.joint} "
            f"(motor {self.motor_index}) from {self.args.min_q:.3f} to {self.args.max_q:.3f} rad."
        )
        print("Press Ctrl-C to stop publishing.")

        self.ramp_to_start(current_q, self.args.min_q)

        dt = 1.0 / self.args.rate
        last_q = self.args.min_q
        half_period = max(0.2, self.args.period / 2.0)
        start = time.monotonic()
        try:
            while True:
                phase = ((time.monotonic() - start) % self.args.period) / half_period
                if phase <= 1.0:
                    q = blend(self.args.min_q, self.args.max_q, phase)
                else:
                    q = blend(self.args.max_q, self.args.min_q, phase - 1.0)
                max_step = self.args.max_velocity * dt
                q = max(last_q - max_step, min(last_q + max_step, q))
                self.publish(q)
                last_q = q
                time.sleep(dt)
        except KeyboardInterrupt:
            print("\nCtrl-C received. Relaxing wrist and stopping command stream.")
            self.relax_wrist(last_q)
            return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("interface", help="Robot Ethernet interface, for example enp0s31f6.")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--joint", choices=sorted(RIGHT_WRIST), default="yaw")
    parser.add_argument("--min", dest="min_q", type=float, default=None)
    parser.add_argument("--max", dest="max_q", type=float, default=None)
    parser.add_argument("--period", type=float, default=6.0, help="Seconds for one end-to-end-to-end cycle.")
    parser.add_argument("--rate", type=float, default=200.0)
    parser.add_argument("--kp", type=float, default=4.0)
    parser.add_argument("--kd", type=float, default=0.35)
    parser.add_argument("--max-velocity", type=float, default=0.35, help="Maximum commanded wrist speed in rad/s.")
    parser.add_argument("--ramp-seconds", type=float, default=3.0)
    parser.add_argument("--wait-timeout", type=float, default=5.0)
    parser.add_argument("--mode-pr", type=int, default=None)
    parser.add_argument("--mode-machine", type=int, default=None)
    parser.add_argument("--probe", action="store_true", help="Connect and print wrist state without publishing.")
    parser.add_argument("--nudge", action="store_true", help="Send a tiny short command and print observed q change.")
    parser.add_argument("--nudge-delta", type=float, default=0.05)
    parser.add_argument("--nudge-seconds", type=float, default=0.5)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--i-understand-risk", action="store_true")
    args = parser.parse_args()

    default_min, default_max = DEFAULT_LIMITS[args.joint]
    if args.min_q is None:
        args.min_q = default_min
    if args.max_q is None:
        args.max_q = default_max
    if args.min_q >= args.max_q:
        parser.error("--min must be lower than --max")
    if args.rate <= 0:
        parser.error("--rate must be positive")
    if args.period <= 0:
        parser.error("--period must be positive")
    if not args.probe and (not args.execute or not args.i_understand_risk):
        parser.error("refusing to publish: pass --execute --i-understand-risk")
    return args


def main() -> int:
    return RightWristSpinner(parse_args()).run()


if __name__ == "__main__":
    raise SystemExit(main())
