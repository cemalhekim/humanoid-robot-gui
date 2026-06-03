#!/usr/bin/env python3
"""Shared H1 MuJoCo motion utilities and controller abstractions.

This module centralizes the H1 pose definitions, low-level DDS command
publishing, and reusable motion primitives for scripted walk, jump, and
handshake behaviors.
"""

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
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.utils.crc import CRC
except ImportError as exc:
    print(
        "Missing Unitree SDK2 Python dependency. Run:\n"
        "  ./scripts/install_unitree_mujoco_python_deps.sh\n\n"
        f"Import error: {exc}",
        file=sys.stderr,
    )
    raise SystemExit(2)

MOTOR_INDEX = {
    "right_hip_roll_joint": 0,
    "right_hip_pitch_joint": 1,
    "right_knee_joint": 2,
    "left_hip_roll_joint": 3,
    "left_hip_pitch_joint": 4,
    "left_knee_joint": 5,
    "torso_joint": 6,
    "left_hip_yaw_joint": 7,
    "right_hip_yaw_joint": 8,
    "not_use_joint": 9,
    "left_ankle_joint": 10,
    "right_ankle_joint": 11,
    "right_shoulder_pitch_joint": 12,
    "right_shoulder_roll_joint": 13,
    "right_shoulder_yaw_joint": 14,
    "right_elbow_joint": 15,
    "left_shoulder_pitch_joint": 16,
    "left_shoulder_roll_joint": 17,
    "left_shoulder_yaw_joint": 18,
    "left_elbow_joint": 19,
}

NUM_MOTORS = 20

HIP_KNEE_MOTORS = {0, 1, 2, 3, 4, 5, 7, 8}
ANKLE_MOTORS = {10, 11}
TORSO_MOTORS = {6}
ARM_MOTORS = {12, 13, 14, 15, 16, 17, 18, 19}


def build_pose(**joints):
    pose = [0.0] * NUM_MOTORS
    for name, value in joints.items():
        pose[MOTOR_INDEX[name]] = value
    return pose


def with_joints(base, **joints):
    pose = base[:]
    for name, value in joints.items():
        pose[MOTOR_INDEX[name]] = value
    return pose


def clamp_pose(pose):
    limits = {
        "right_hip_roll_joint": (-0.35, 0.35),
        "left_hip_roll_joint": (-0.35, 0.35),
        "right_hip_pitch_joint": (-1.2, 0.25),
        "left_hip_pitch_joint": (-1.2, 0.25),
        "right_knee_joint": (0.25, 1.45),
        "left_knee_joint": (0.25, 1.45),
        "right_ankle_joint": (-0.75, 0.25),
        "left_ankle_joint": (-0.75, 0.25),
        "torso_joint": (-0.25, 0.25),
    }
    clamped = pose[:]
    for joint, (lo, hi) in limits.items():
        index = MOTOR_INDEX[joint]
        clamped[index] = min(hi, max(lo, clamped[index]))
    return clamped


def interpolate_pose(start, target, blend):
    return [start[i] + (target[i] - start[i]) * blend for i in range(NUM_MOTORS)]


HOME_POSE = build_pose(
    right_hip_pitch_joint=-0.4,
    right_knee_joint=0.8,
    right_ankle_joint=-0.4,
    left_hip_pitch_joint=-0.4,
    left_knee_joint=0.8,
    left_ankle_joint=-0.4,
    torso_joint=0.0,
    right_shoulder_pitch_joint=0.0,
    right_shoulder_roll_joint=0.0,
    right_shoulder_yaw_joint=0.0,
    right_elbow_joint=0.0,
    left_shoulder_pitch_joint=0.0,
    left_shoulder_roll_joint=0.0,
    left_shoulder_yaw_joint=0.0,
    left_elbow_joint=0.0,
)

HANDSHAKE_READY = build_pose(
    right_hip_pitch_joint=-0.4,
    right_knee_joint=0.8,
    right_ankle_joint=-0.4,
    left_hip_pitch_joint=-0.4,
    left_knee_joint=0.8,
    left_ankle_joint=-0.4,
    torso_joint=0.0,
    right_shoulder_pitch_joint=-2.55,
    right_shoulder_roll_joint=-0.90,
    right_shoulder_yaw_joint=-0.40,
    right_elbow_joint=1.25,
    left_shoulder_pitch_joint=0.0,
    left_shoulder_roll_joint=0.0,
    left_shoulder_yaw_joint=0.0,
    left_elbow_joint=0.0,
)


class LowStateBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._last_msg = None

    def update(self, msg):
        with self._lock:
            self._last_msg = msg

    def q(self):
        with self._lock:
            if self._last_msg is None:
                return None
            return [self._last_msg.motor_state[i].q for i in range(NUM_MOTORS)]


class H1MotionController:
    def __init__(self, domain_id, interface, rate_hz):
        ChannelFactoryInitialize(domain_id, interface)

        self.state = LowStateBuffer()
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.state.update, 10)

        self.publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.publisher.Init()
        self.crc = CRC()
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.last_target = HOME_POSE[:]

    def wait_for_simulator(self, timeout):
        print("Waiting for MuJoCo lowstate on rt/lowstate...")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            q = self.state.q()
            if q is not None:
                self.last_target = q
                print("Connected to MuJoCo lowstate.")
                return True
            time.sleep(0.05)
        return False

    def make_command(self, q_target):
        cmd = unitree_go_msg_dds__LowCmd_()
        cmd.head[0] = 0xFE
        cmd.head[1] = 0xEF
        cmd.level_flag = 0xFF
        cmd.gpio = 0

        for i in range(NUM_MOTORS):
            motor = cmd.motor_cmd[i]
            motor.mode = 0x01
            motor.q = q_target[i]
            motor.dq = 0.0
            motor.tau = 0.0

            if i == MOTOR_INDEX["not_use_joint"]:
                motor.kp = 0.0
                motor.kd = 0.0
            elif i in HIP_KNEE_MOTORS:
                motor.kp = 200.0
                motor.kd = 5.0
            elif i in ANKLE_MOTORS:
                motor.kp = 60.0
                motor.kd = 3.0
            elif i in TORSO_MOTORS:
                motor.kp = 120.0
                motor.kd = 4.0
            elif i in ARM_MOTORS:
                motor.kp = 45.0
                motor.kd = 2.0
            else:
                motor.kp = 35.0
                motor.kd = 2.0

        cmd.crc = self.crc.Crc(cmd)
        return cmd

    def publish_pose(self, pose):
        self.publisher.Write(self.make_command(pose))
        self.last_target = pose[:]

    def hold(self, pose, duration, label):
        print(label)
        if duration is None:
            while True:
                self.publish_pose(pose)
                time.sleep(self.dt)

        end_time = time.monotonic() + duration
        while time.monotonic() < end_time:
            self.publish_pose(pose)
            time.sleep(self.dt)

    def move_to(self, target, duration, label):
        print(label)
        start = self.last_target[:]
        steps = max(1, int(duration * self.rate_hz))
        for step in range(steps):
            t = (step + 1) / steps
            blend = 0.5 - 0.5 * math.cos(math.pi * t)
            pose = interpolate_pose(start, target, blend)
            self.publish_pose(pose)
            time.sleep(self.dt)

    def make_pose(self, **joints):
        return clamp_pose(with_joints(HOME_POSE, **joints))

    def scaled_duration(self, duration, speed_scale=1.0):
        return duration / max(0.25, speed_scale)

    def step_once(self, side, speed_scale=1.0):
        if side not in {"right", "left"}:
            raise ValueError(f"unsupported step side: {side}")

        swing = side
        support = "left" if side == "right" else "right"
        sign = -1.0 if side == "right" else 1.0

        shift = {
            f"{support}_hip_roll_joint": 0.12 * sign,
            f"{swing}_hip_roll_joint": 0.08 * sign,
            "torso_joint": 0.04 * sign,
        }

        lift = {
            **shift,
            f"{swing}_hip_pitch_joint": -0.72,
            f"{swing}_knee_joint": 1.18,
            f"{swing}_ankle_joint": -0.50,
        }
        swing_forward = {
            **shift,
            f"{swing}_hip_pitch_joint": -0.18,
            f"{swing}_knee_joint": 0.72,
            f"{swing}_ankle_joint": -0.48,
            f"{support}_hip_pitch_joint": -0.52,
            f"{support}_knee_joint": 0.92,
            f"{support}_ankle_joint": -0.40,
        }
        settle = {
            f"{swing}_hip_pitch_joint": -0.28,
            f"{swing}_knee_joint": 0.78,
            f"{swing}_ankle_joint": -0.44,
            f"{support}_hip_pitch_joint": -0.50,
            f"{support}_knee_joint": 0.88,
            f"{support}_ankle_joint": -0.38,
        }

        print(f"Step with {side} leg.")
        self.move_to(
            self.make_pose(**shift),
            self.scaled_duration(0.7, speed_scale),
            f"Shifting weight to {support}.",
        )
        self.move_to(
            self.make_pose(**lift),
            self.scaled_duration(0.8, speed_scale),
            f"Lifting {swing} foot.",
        )
        self.move_to(
            self.make_pose(**swing_forward),
            self.scaled_duration(0.9, speed_scale),
            f"Swinging {swing} foot.",
        )
        self.move_to(
            self.make_pose(**settle),
            self.scaled_duration(0.8, speed_scale),
            f"Planting {swing} foot.",
        )
        self.hold(
            self.make_pose(**settle),
            self.scaled_duration(0.4, speed_scale),
            f"Settling on {side} step.",
        )

    def walk_steps(self, count=2, initial_side="right", speed_scale=1.0):
        if count < 1:
            raise ValueError("count must be at least 1")

        self.move_to(
            HOME_POSE,
            self.scaled_duration(3.0, speed_scale),
            "Moving to home pose.",
        )
        self.hold(
            HOME_POSE,
            self.scaled_duration(2.0, speed_scale),
            "Holding home pose.",
        )

        side = initial_side
        for step in range(count):
            self.step_once(side, speed_scale)
            side = "left" if side == "right" else "right"

        self.move_to(
            HOME_POSE,
            self.scaled_duration(2.0, speed_scale),
            "Returning to home pose.",
        )
        self.hold(
            HOME_POSE,
            self.scaled_duration(0.4, speed_scale),
            "Still at home after walking steps.",
        )

    def jump(self, height=0.12, duration=1.5):
        if duration <= 0.0:
            raise ValueError("duration must be positive")

        squat = self.make_pose(
            right_hip_pitch_joint=-0.72,
            left_hip_pitch_joint=-0.72,
            right_knee_joint=1.28,
            left_knee_joint=1.28,
            right_ankle_joint=-0.22,
            left_ankle_joint=-0.22,
            torso_joint=0.04,
        )
        launch = self.make_pose(
            right_hip_pitch_joint=-0.30,
            left_hip_pitch_joint=-0.30,
            right_knee_joint=0.60,
            left_knee_joint=0.60,
            right_ankle_joint=-0.60,
            left_ankle_joint=-0.60,
            torso_joint=min(0.20, height),
        )

        self.move_to(squat, self.scaled_duration(duration * 0.30), "Squatting for jump.")
        self.move_to(launch, self.scaled_duration(duration * 0.25), "Extending legs for jump.")
        self.move_to(HOME_POSE, self.scaled_duration(duration * 0.45), "Landing back in home pose.")
        self.hold(HOME_POSE, 1.0, "Holding home pose after jump.")

    def handshake(self, cycles):
        print("Handshake motion.")
        base = HANDSHAKE_READY[:]
        steps_per_cycle = max(8, int(0.7 * self.rate_hz))
        for _ in range(cycles):
            for step in range(steps_per_cycle):
                phase = (step / steps_per_cycle) * 2.0 * math.pi
                pose = base[:]
                pose[MOTOR_INDEX["right_shoulder_roll_joint"]] += 0.16 * math.sin(phase)
                pose[MOTOR_INDEX["right_elbow_joint"]] += 0.18 * math.sin(phase)
                self.publish_pose(pose)
                time.sleep(self.dt)

    def run_handshake(self, stand_seconds, cycles, final_hold_seconds):
        self.move_to(HOME_POSE, 3.0, "Moving to home pose.")
        self.hold(HOME_POSE, stand_seconds, "Holding home pose.")
        self.move_to(HANDSHAKE_READY, 2.0, "Raising right arm.")
        self.handshake(cycles)
        self.move_to(HOME_POSE, 2.5, "Returning to home pose.")
        duration = None if final_hold_seconds <= 0.0 else final_hold_seconds
        self.hold(HOME_POSE, duration, "Demo complete, holding home pose.")


H1DemoController = H1MotionController
