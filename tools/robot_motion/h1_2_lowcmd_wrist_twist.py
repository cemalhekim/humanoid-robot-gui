#!/usr/bin/env python3
"""H1-2 low-level right wrist yaw twist test.

This follows the official H1-2 low-level control pattern more closely than the
earlier one-joint script:

- checks/releases the active motion mode through MotionSwitcherClient
- subscribes to rt/lowstate
- publishes a complete rt/lowcmd for all 27 real H1-2 joints
- holds every real joint at the latest observed position
- adds a small sine offset only to right wrist yaw
- restores the requested motion mode on exit
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
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
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC
except ImportError as exc:
    print(f"Missing Unitree SDK2 Python: {exc}", file=sys.stderr)
    raise SystemExit(2)


REAL_MOTORS = 27
RIGHT_WRIST_YAW = 26


def default_kp(index: int) -> float:
    if index < 13:
        return 70.0
    if index == RIGHT_WRIST_YAW:
        return 8.0
    return 25.0


def default_kd(index: int) -> float:
    if index < 13:
        return 1.0
    if index == RIGHT_WRIST_YAW:
        return 0.5
    return 0.8


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


class LowCmdWristTwist:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.state = LowStateBuffer()
        self.crc = CRC()
        self.running = True
        self.mode_before = None

        ChannelFactoryInitialize(args.domain_id, args.interface)
        self.motion = MotionSwitcherClient()
        self.motion.SetTimeout(args.timeout)
        self.motion.Init()
        self.publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.publisher.Init()
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.state.update, 10)

    def check_mode(self):
        code, result = self.motion.CheckMode()
        print(f"motion mode: code={code}, result={result}")
        return result if code == 0 else None

    def release_mode(self) -> None:
        self.mode_before = self.check_mode()
        if self.mode_before and self.mode_before.get("name"):
            print(f"releasing motion mode {self.mode_before.get('name')!r}")
            code, _ = self.motion.ReleaseMode()
            print(f"release mode code={code}")
            time.sleep(0.5)
            self.check_mode()

    def restore_mode(self) -> None:
        mode = self.args.restore_mode
        if not mode:
            return
        print(f"restoring motion mode {mode!r}")
        code, _ = self.motion.SelectMode(mode)
        print(f"restore mode code={code}")
        time.sleep(0.5)
        self.check_mode()

    def make_command(self, hold_q: list[float], wrist_q: float, mode_machine: int) -> LowCmd_:
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 0
        cmd.mode_machine = mode_machine
        for i in range(REAL_MOTORS):
            motor = cmd.motor_cmd[i]
            motor.mode = 1
            motor.q = wrist_q if i == RIGHT_WRIST_YAW else hold_q[i]
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = self.args.kp if i == RIGHT_WRIST_YAW else default_kp(i)
            motor.kd = self.args.kd if i == RIGHT_WRIST_YAW else default_kd(i)
            motor.reserve = 0
        cmd.crc = self.crc.Crc(cmd)
        return cmd

    def run(self) -> int:
        if self.args.release_mode:
            self.release_mode()

        msg = self.state.wait(self.args.timeout)
        if msg is None:
            print("No rt/lowstate received; not publishing.", file=sys.stderr)
            self.restore_mode()
            return 1

        hold_q = [float(msg.motor_state[i].q) for i in range(REAL_MOTORS)]
        center = hold_q[RIGHT_WRIST_YAW]
        mode_machine = int(getattr(msg, "mode_machine", 0) or 0)

        print(
            f"twisting right wrist yaw around {center:.6f} rad, "
            f"amplitude={self.args.amplitude:.3f}, period={self.args.period:.3f}s"
        )
        print("Press Ctrl-C to stop.")

        start = time.monotonic()
        dt = 1.0 / self.args.rate
        try:
            while True:
                t = time.monotonic() - start
                offset = self.args.amplitude * math.sin((2.0 * math.pi * t) / self.args.period)
                cmd = self.make_command(hold_q, center + offset, mode_machine)
                self.publisher.Write(cmd)
                time.sleep(dt)
        except KeyboardInterrupt:
            print("\nCtrl-C received; sending hold commands briefly.")
            hold_cmd = self.make_command(hold_q, center, mode_machine)
            for _ in range(40):
                self.publisher.Write(hold_cmd)
                time.sleep(0.01)
            self.restore_mode()
            return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("interface")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=200.0)
    parser.add_argument("--amplitude", type=float, default=0.05)
    parser.add_argument("--period", type=float, default=4.0)
    parser.add_argument("--kp", type=float, default=8.0)
    parser.add_argument("--kd", type=float, default=0.5)
    parser.add_argument("--release-mode", action="store_true")
    parser.add_argument("--restore-mode", default="ai")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--i-understand-risk", action="store_true")
    args = parser.parse_args()
    if not args.execute or not args.i_understand_risk:
        parser.error("refusing to publish: pass --execute --i-understand-risk")
    if args.amplitude <= 0 or args.amplitude > 0.25:
        parser.error("--amplitude must be in (0, 0.25]")
    if args.period <= 0:
        parser.error("--period must be positive")
    return args


def main() -> int:
    return LowCmdWristTwist(parse_args()).run()


if __name__ == "__main__":
    raise SystemExit(main())
