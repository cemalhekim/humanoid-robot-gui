#!/usr/bin/env python3
"""Guarded H1 handshake runner for real hardware.

Default mode is dry-run. To publish arm low-level commands, pass both
`--execute-lowcmd-arms` and `--i-understand-risk`.

This script intentionally does not release the robot's locomotion mode. It asks
the high-level H1 loco service to stand up, then publishes arm joint targets
with all non-arm joints set to PosStop/VelStop and zero gains. Depending on the
robot firmware, those arm commands may be ignored while loco owns the robot.
That is safer than forcefully taking over all motors.
"""

import argparse
import math
import sys
import threading
import time

try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.utils.crc import CRC
except ImportError as exc:
    print(
        "Missing Unitree SDK2 Python dependency. Run through:\n"
        "  ./scripts/run_h1_real_handshake_guarded.sh <network-interface>\n\n"
        f"Import error: {exc}",
        file=sys.stderr,
    )
    raise SystemExit(2)


POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0
NUM_MOTORS = 20

MOTOR_INDEX = {
    "right_shoulder_pitch_joint": 12,
    "right_shoulder_roll_joint": 13,
    "right_shoulder_yaw_joint": 14,
    "right_elbow_joint": 15,
    "left_shoulder_pitch_joint": 16,
    "left_shoulder_roll_joint": 17,
    "left_shoulder_yaw_joint": 18,
    "left_elbow_joint": 19,
}

ARM_MOTORS = set(MOTOR_INDEX.values())

ARM_HOME = {
    "right_shoulder_pitch_joint": 0.0,
    "right_shoulder_roll_joint": 0.0,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.0,
    "left_shoulder_pitch_joint": 0.0,
    "left_shoulder_roll_joint": 0.0,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 0.0,
}

HANDSHAKE_READY = {
    "right_shoulder_pitch_joint": -0.45,
    "right_shoulder_roll_joint": -0.30,
    "right_shoulder_yaw_joint": -0.10,
    "right_elbow_joint": 0.95,
    "left_shoulder_pitch_joint": 0.0,
    "left_shoulder_roll_joint": 0.0,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 0.0,
}


class LowStateBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_msg = None

    def update(self, msg):
        with self.lock:
            self.last_msg = msg

    def q(self):
        with self.lock:
            if self.last_msg is None:
                return None
            return [self.last_msg.motor_state[i].q for i in range(NUM_MOTORS)]


def blend(a, b, alpha):
    smooth = 0.5 - 0.5 * math.cos(math.pi * alpha)
    return a + (b - a) * smooth


def pose_dict_to_list(pose):
    target = [0.0] * NUM_MOTORS
    for joint, value in pose.items():
        target[MOTOR_INDEX[joint]] = value
    return target


class ArmLowCmdPublisher:
    def __init__(self, rate_hz):
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.crc = CRC()
        self.state = LowStateBuffer()
        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_sub.Init(self.state.update, 10)
        self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_pub.Init()
        self.last_target = pose_dict_to_list(ARM_HOME)

    def wait_for_lowstate(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            q = self.state.q()
            if q is not None:
                self.last_target = q
                return q
            time.sleep(0.05)
        return None

    def make_command(self, arm_target):
        cmd = unitree_go_msg_dds__LowCmd_()
        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF
        cmd.gpio = 0

        for i in range(NUM_MOTORS):
            motor = cmd.motor_cmd[i]
            motor.mode = 0x01
            motor.tau = 0.0
            if i in ARM_MOTORS:
                motor.q = arm_target[i]
                motor.dq = 0.0
                motor.kp = 18.0
                motor.kd = 1.0
            else:
                motor.q = POS_STOP_F
                motor.dq = VEL_STOP_F
                motor.kp = 0.0
                motor.kd = 0.0

        cmd.crc = self.crc.Crc(cmd)
        return cmd

    def publish(self, arm_target):
        self.lowcmd_pub.Write(self.make_command(arm_target))
        self.last_target = arm_target[:]

    def move_to(self, target, duration, label):
        print(label)
        start = self.last_target[:]
        steps = max(1, int(duration * self.rate_hz))
        for step in range(steps):
            alpha = (step + 1) / steps
            pose = start[:]
            for i in ARM_MOTORS:
                pose[i] = blend(start[i], target[i], alpha)
            self.publish(pose)
            time.sleep(self.dt)

    def hold(self, target, duration, label):
        print(label)
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.publish(target)
            time.sleep(self.dt)

    def handshake(self, cycles):
        print("Small handshake oscillation.")
        base = pose_dict_to_list(HANDSHAKE_READY)
        steps_per_cycle = max(8, int(0.8 * self.rate_hz))
        for _ in range(cycles):
            for step in range(steps_per_cycle):
                phase = (step / steps_per_cycle) * 2.0 * math.pi
                pose = base[:]
                pose[MOTOR_INDEX["right_shoulder_roll_joint"]] += 0.08 * math.sin(phase)
                pose[MOTOR_INDEX["right_elbow_joint"]] += 0.10 * math.sin(phase)
                self.publish(pose)
                time.sleep(self.dt)


def check_call(name, code):
    if code == 0 or code is None:
        print(f"{name}: ok")
        return True
    print(f"{name}: failed with code {code}", file=sys.stderr)
    return False


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("interface", help="Robot network interface, for example enp3s0.")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--standup-delay", type=float, default=4.0)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--wait-timeout", type=float, default=10.0)
    parser.add_argument("--skip-standup", action="store_true")
    parser.add_argument("--execute-lowcmd-arms", action="store_true")
    parser.add_argument("--i-understand-risk", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    ChannelFactoryInitialize(args.domain_id, args.interface)

    print("H1 guarded handshake runner.")
    print("Keep the robot supported by its normal safety setup and keep E-stop ready.")

    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()

    if not args.skip_standup:
        if not check_call("StandUp", loco.StandUp()):
            return 1
        time.sleep(args.standup_delay)

    if not args.execute_lowcmd_arms:
        print("Dry-run only. Add --execute-lowcmd-arms --i-understand-risk to publish arm commands.")
        return 0

    if not args.i_understand_risk:
        print("Refusing to publish lowcmd arms without --i-understand-risk.", file=sys.stderr)
        return 2

    arm = ArmLowCmdPublisher(args.rate)
    q = arm.wait_for_lowstate(args.wait_timeout)
    if q is None:
        print("No rt/lowstate received; not publishing arm commands.", file=sys.stderr)
        return 1

    start = q[:]
    for i in range(NUM_MOTORS):
        if i not in ARM_MOTORS:
            start[i] = 0.0
    arm.last_target = start

    arm.move_to(pose_dict_to_list(ARM_HOME), 2.0, "Moving arms to home.")
    arm.move_to(pose_dict_to_list(HANDSHAKE_READY), 2.0, "Raising right arm.")
    arm.handshake(args.cycles)
    arm.move_to(pose_dict_to_list(ARM_HOME), 2.0, "Returning arms home.")
    arm.hold(pose_dict_to_list(ARM_HOME), 1.0, "Holding arm home briefly.")
    print("Handshake sequence complete.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
