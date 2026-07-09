#!/usr/bin/env python3
"""Unitree H1-2 telemetry web dashboard.

Runs on the robot PC. Subscribes to rt/lowstate continuously and serves a
dependency-free web UI with JSON and Server-Sent Events endpoints.
"""

from __future__ import annotations

import argparse
import json
import math
import mimetypes
import os
import queue
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
import contextlib
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit
from typing import Any

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None

try:
    import numpy as np  # type: ignore
except Exception:  # pragma: no cover
    np = None


APP_DIR = Path(__file__).resolve().parent
STATIC_DIR = APP_DIR / "static"
CAMERA_JPEG_PATH = Path("/tmp/robot_telemetry_front_camera.jpg")
RECORDINGS_DIR = APP_DIR / "recordings"
DOCS_DIR = APP_DIR / "docs"
XR_TELEOP_MODE_DROPIN = Path.home() / ".config/systemd/user/xr-teleop.service.d/10-control-mode.conf"
XR_MOTION_SERVICES = ("xr-home-watchdog.service", "xr-teleop.service")
XR_TELEOP_PROCESS_PATTERN = "teleop_hand_and_arm.py"
UNITREE_ROS2_INSTALL = (
    APP_DIR
    / "execution/semantic_teleoperation/external/unitree_ros2/cyclonedds_ws/install"
)
UNITREE_GO_PYTHON = UNITREE_ROS2_INSTALL / "unitree_go/local/lib/python3.10/dist-packages"
if UNITREE_GO_PYTHON.exists():
    sys.path.insert(0, str(UNITREE_GO_PYTHON))

SDK_PATHS = [
    APP_DIR / "execution/semantic_teleoperation/external/unitree_sdk2_python",
    APP_DIR / "vendor/unitree_sdk2_python",
    Path.home() / "unitree_sdk2_python",
]
for sdk_path in reversed(SDK_PATHS):
    if sdk_path.exists():
        sys.path.insert(0, str(sdk_path))

TELEIMAGER_PATHS = [
    APP_DIR / "teleoperation/vision_pro_control/external/xr_teleoperate/teleop/teleimager/src",
    Path.home() / "teleimager/src",
]
for teleimager_path in reversed(TELEIMAGER_PATHS):
    if teleimager_path.exists():
        sys.path.insert(0, str(teleimager_path))

XR_TELEOP_PATHS = [
    Path("/home/unitree/xr_teleoperate"),
    Path("/home/unitree/xr_teleoperate/teleop/robot_control/dex-retargeting/src"),
    APP_DIR / "teleoperation/vision_pro_control/external/xr_teleoperate",
    APP_DIR / "teleoperation/vision_pro_control/external/xr_teleoperate/teleop/robot_control/dex-retargeting/src",
]

JOINT_NAMES = {
    0: "LeftHipYaw",
    1: "LeftHipPitch",
    2: "LeftHipRoll",
    3: "LeftKnee",
    4: "LeftAnklePitch",
    5: "LeftAnkleRoll",
    6: "RightHipYaw",
    7: "RightHipPitch",
    8: "RightHipRoll",
    9: "RightKnee",
    10: "RightAnklePitch",
    11: "RightAnkleRoll",
    12: "WaistYaw",
    13: "LeftShoulderPitch",
    14: "LeftShoulderRoll",
    15: "LeftShoulderYaw",
    16: "LeftElbow",
    17: "LeftWristRoll",
    18: "LeftWristPitch",
    19: "LeftWristYaw",
    20: "RightShoulderPitch",
    21: "RightShoulderRoll",
    22: "RightShoulderYaw",
    23: "RightElbow",
    24: "RightWristRoll",
    25: "RightWristPitch",
    26: "RightWristYaw",
}

JOINT_GROUPS = {
    "left_leg": list(range(0, 6)),
    "right_leg": list(range(6, 12)),
    "waist": [12],
    "left_arm": list(range(13, 20)),
    "right_arm": list(range(20, 27)),
    "reserved": list(range(27, 35)),
}

LOWER_BODY_JOINTS = JOINT_GROUPS["left_leg"] + JOINT_GROUPS["right_leg"]
TRAJECTORY_ROUTE_EPSILON = 0.015
TRAJECTORY_MAX_FRAME_DELTA_RAD = 0.18
TRAJECTORY_MAX_VELOCITY_RAD_S = 2.0
TRAJECTORY_DEFAULT_DT = 1.0 / 60.0
TRAJECTORY_APPROACH_SECONDS = 3.0
TRAJECTORY_ADAPTIVE_SAMPLE_HZ = 60.0
TRAJECTORY_DENSE_MAX_DT = 1.0 / 30.0
TRAJECTORY_MAX_INTERPOLATED_STEP_RAD = 0.05
TRAJECTORY_MAX_REPORTED_VIOLATIONS = 20
ARM_REPLAY_CLOSED_LOOP_DEFAULT = True
ARM_REPLAY_HOLD_AFTER_CONVERGENCE_DEFAULT = True
ARM_REPLAY_LOCK_TOLERANCE_M = 0.01
ARM_REPLAY_LOCK_TOLERANCE_RAD = 0.01
ARM_REPLAY_TOLERANCE_RAD = ARM_REPLAY_LOCK_TOLERANCE_RAD
ARM_REPLAY_SETTLE_SECONDS = 0.6
ARM_REPLAY_TIMEOUT_SECONDS = 10.0
ARM_REPLAY_SMOOTH_APPROACH_SECONDS = 4.5
# The initial move from the robot's current pose to the FIRST frame (of a pose or
# a sequence) is velocity-bounded, not just time-bounded, so it is always smooth
# regardless of how far the first frame is or how high the response dial is. The
# ramp duration is stretched so the smootherstep PEAK joint velocity stays under
# this cap (smootherstep peak = 1.875x average), with a small floor for tiny moves.
ARM_REPLAY_APPROACH_PEAK_VEL_RAD_S = 0.6
ARM_REPLAY_APPROACH_MIN_SECONDS = 2.0
ARM_REPLAY_MAX_PID_CORRECTION_RAD = 0.12
ARM_REPLAY_INTEGRAL_LIMIT = 0.35
# Gravity feed-forward low-pass time constant. Kept fairly slow on purpose: the
# feed-forward is derived from measured torque (which contains the PD reaction),
# so a fast filter lets it chase the control loop and limit-cycle. A slower
# filter makes the feed-forward a steadier gravity estimate; the adaptive learn
# term supplies the accurate steady holding torque.
ARM_REPLAY_GRAVITY_TAU_FILTER_SECONDS = 0.4
ARM_REPLAY_INNER_KP_SCALE = 0.35
ARM_REPLAY_INNER_KD_SCALE = 1.2
ARM_REPLAY_RESPONSE_DEFAULT = 0.5
ARM_REPLAY_RESPONSE_MAX = 2.5
# --- Convergence / holding upgrade (drive the arm ONTO the recorded pose) ---
# Gravity feed-forward completeness. When a joint is inside the lock band and
# steady we feed forward (almost) the full measured holding torque, so the
# inner motor PD needs ~zero standing error to hold. Blended CONTINUOUSLY with
# |error| (no destabilising jump at the lock boundary like the old 0.65/0.25).
ARM_REPLAY_GRAVITY_HOLD_SCALE = 0.95
ARM_REPLAY_GRAVITY_MOVE_SCALE = 0.5
# The hold/move blend is driven by how STATIONARY the joint is, not by whether
# it is already locked: as any joint slows near its target it gets near-full
# gravity support. The velocity threshold is deliberately HIGH so that only a
# genuinely fast reaching motion reduces support -- the small velocities of a
# holding joint that starts to sag keep near-full gravity support, otherwise
# support gets cut exactly when the arm starts to fall and it limit-cycles
# (sag 1-2 cm -> catch -> sag) at the hold point.
ARM_REPLAY_GRAVITY_RAMP_VEL_FACTOR = 30.0
# Stiff active "hold": there is no mechanical brake, so during the hold phase we
# raise position stiffness (kp) so a given feed-forward error maps to a smaller
# sag. We deliberately do NOT raise kd much: the gravity feed-forward is built
# from MEASURED torque, which already contains the controller's own PD reaction,
# so a high kd feeds its own damping torque back through the feed-forward and
# amplifies the bob (positive feedback). Expressed relative to the raw nominal
# arm_sdk gains; kp stays below nominal, kd stays near nominal.
ARM_REPLAY_HOLD_KP_SCALE = 0.55
ARM_REPLAY_HOLD_KD_SCALE = 1.2
# Hold-loop rate: run the settle/hold loop faster than the 60 Hz playback so the
# setpoint + gravity feed-forward refresh sooner and the arm is caught before it
# can drift far.
ARM_REPLAY_HOLD_HZ = 120.0
# Adaptive gravity "learning": a bounded integral ON THE FEED-FORWARD TORQUE
# that nulls residual holding error WITHOUT moving the commanded setpoint off
# the target, so the joint keeps driving its true error toward zero. Engages in
# a band around the target (not only when fully locked) so it also erases the
# approach residual. Bounded by the same per-joint gravity tau limits, so it can
# never exceed existing safety envelopes.
ARM_REPLAY_GRAVITY_LEARN_GAIN = 22.0
ARM_REPLAY_GRAVITY_LEARN_LIMIT = 4.0
ARM_REPLAY_LEARN_BAND_FACTOR = 3.0
# A joint only counts as "settled" when inside the band AND nearly stationary.
ARM_REPLAY_CONVERGE_VELOCITY_RAD_S = 0.05
# Hysteresis so one noisy joint cannot reset the whole convergence latch.
ARM_REPLAY_SETTLE_HYSTERESIS = 1.6
# Cartesian "silhouette" proxy: weight each joint by an approximate lever arm
# and require the weighted end-effector error to be small too, so convergence
# tracks the visible red/blue gap rather than joint angles alone.
ARM_REPLAY_CARTESIAN_TOLERANCE_M = 0.006
ARM_REPLAY_JOINT_LEVER_M = {
    13: 0.55, 14: 0.55, 15: 0.42, 16: 0.32, 17: 0.20, 18: 0.14, 19: 0.08,
    20: 0.55, 21: 0.55, 22: 0.42, 23: 0.32, 24: 0.20, 25: 0.14, 26: 0.08,
    12: 0.60,
}
# Stall escalation: if error plateaus above the band, ramp gravity learning and
# correction authority (bounded) instead of stalling forever.
ARM_REPLAY_STALL_SECONDS = 2.5
ARM_REPLAY_ESCALATION_MAX = 2.0
ARM_REPLAY_ESCALATION_STEP = 0.25
# Convergence is the ONLY success exit. If the arm still has not converged after
# this absolute ceiling it enters a FLAGGED safe-hold (keeps holding, reports
# converged=false) -- it never claims success or releases at the wrong pose.
ARM_REPLAY_ABSOLUTE_CEILING_SECONDS = 90.0
HAND_STATE_TOPIC = "rt/inspire/state"
HAND_COMMAND_TOPIC = "rt/inspire/cmd"
HAND_TRAJECTORY_EPSILON = 0.02
HAND_TRAJECTORY_MAX_FRAME_DELTA = 0.18
HAND_TRAJECTORY_MAX_VELOCITY = 3.0
LOWCMD_BASE_GAINS = {
    "hip": (40.0, 1.0),
    "knee": (45.0, 1.1),
    "ankle": (30.0, 0.8),
    "waist": (35.0, 1.0),
    "shoulder": (25.0, 0.8),
    "elbow": (20.0, 0.7),
    "wrist": (12.0, 0.5),
}
GAIN_NOMINALS = {
    "hip": {"step": 0.04, "velocity": 0.8},
    "knee": {"step": 0.05, "velocity": 0.9},
    "ankle": {"step": 0.035, "velocity": 0.7},
    "waist": {"step": 0.03, "velocity": 0.6},
    "shoulder": {"step": 0.05, "velocity": 1.0},
    "elbow": {"step": 0.06, "velocity": 1.1},
    "wrist": {"step": 0.07, "velocity": 1.2},
}
ARM_REPLAY_PID_GAINS = {
    "shoulder": (0.28, 0.035, 0.018),
    "elbow": (0.24, 0.03, 0.014),
    "wrist": (0.18, 0.02, 0.012),
    "waist": (0.12, 0.01, 0.01),
}
# Per-joint gravity feed-forward magnitude clamp (Nm). Sized to cover the real
# worst-case static gravity of an outstretched H1-2 arm (shoulder ~12 Nm) with
# margin, while still bounding the feed-forward so a measured-torque spike (e.g.
# contact) cannot command an unbounded push. NOTE: feeding measured torque
# forward is unsafe on hard contact -- model-based gravity + collision detection
# is the recommended hardware-validated follow-up.
ARM_REPLAY_GRAVITY_TAU_LIMITS = {
    "shoulder": 15.0,
    "elbow": 10.0,
    "wrist": 4.0,
    "waist": 6.0,
}
RIGHT_WRIST_YAW = 26
ARM_SDK_WEIGHT_SLOT = 27
ARM_SDK_JOINTS = [13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 12]
ARM_SDK_KP = [120, 120, 80, 50, 50, 50, 50, 120, 120, 80, 50, 50, 50, 50, 200]
ARM_SDK_KD = [2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 2.0]
REPLAY_COMMAND_SCOPES = {
    "all": list(JOINT_NAMES),
    "arms": JOINT_GROUPS["left_arm"] + JOINT_GROUPS["right_arm"] + JOINT_GROUPS["waist"],
    "both_arms": JOINT_GROUPS["left_arm"] + JOINT_GROUPS["right_arm"],
    "right_arm": JOINT_GROUPS["right_arm"],
    "left_arm": JOINT_GROUPS["left_arm"],
}
JOINT_LIMITS = {
    13: (-3.14, 1.57),
    14: (-0.38, 3.4),
    15: (-2.66, 3.01),
    16: (-0.95, 3.18),
    17: (-3.01, 2.75),
    18: (-0.4625, 0.4625),
    19: (-1.27, 1.27),
    20: (-3.14, 1.57),
    21: (-3.4, 0.38),
    22: (-3.01, 2.66),
    23: (-0.95, 3.18),
    24: (-2.75, 3.01),
    25: (-0.4625, 0.4625),
    26: (-1.27, 1.27),
}
WRIST_LIMITS = (-1.2, 1.2)
# Torso ("belly") twist about WaistYaw (joint 12). Mechanical range is +/-2.35 rad;
# we use a conservative subset because twisting the torso shifts the CoM and can
# affect balance while standing. Commanded through the arm_sdk path (waist gains),
# velocity-slewed so a fast UI drag still yields a smooth twist.
TORSO_YAW_JOINT = 12
TORSO_LIMITS = (-1.0, 1.0)
TORSO_TWIST_KP = 200.0
TORSO_TWIST_KD = 4.0
TORSO_TWIST_MAX_VEL_RAD_S = 0.7
TORSO_TWIST_RATE_HZ = 120.0
LOCO_LIMITS = {
    "vx": [-1.0, 1.0],
    "vy": [-0.5, 0.5],
    "vyaw": [-1.0, 1.0],
    "duration": [0.1, 10.0],
    "stand_height": [0.0, 1.0],
    "swing_height": [0.0, 0.3],
    "target_x": [-2.0, 2.0],
    "target_y": [-2.0, 2.0],
    "target_yaw": [-3.14, 3.14],
}
LOCO_ACTIONS = [
    "ready",
    "balance_stand",
    "stand_up",
    "start",
    "stop_move",
    "damp",
    "zero_torque",
    "high_stand",
    "low_stand",
    "set_height",
    "set_swing_height",
    "velocity",
    "move",
    "continuous_gait_on",
    "continuous_gait_off",
    "next_foot_left",
    "next_foot_right",
    "wave_hand",
    "shake_hand",
    "shake_hand_start",
    "shake_hand_end",
    "enable_odom",
    "disable_odom",
    "get_odom",
    "set_target_position",
    "get_fsm_id",
    "get_fsm_mode",
    "get_balance_mode",
    "get_swing_height",
    "get_stand_height",
    "get_phase",
]

HAND_JOINT_NAMES = {
    0: "RightPinky",
    1: "RightRing",
    2: "RightMiddle",
    3: "RightIndex",
    4: "RightThumbBend",
    5: "RightThumbRotation",
    6: "LeftPinky",
    7: "LeftRing",
    8: "LeftMiddle",
    9: "LeftIndex",
    10: "LeftThumbBend",
    11: "LeftThumbRotation",
}

MAX_JSON_BODY_BYTES = 1_000_000


def recording_timestamp() -> str:
    return time.strftime("%Y%m%d-%H%M%S", time.localtime())


class TelemetryRecorder:
    def __init__(self, directory: Path) -> None:
        self.directory = directory
        self.lock = threading.Lock()
        self.file: Any | None = None
        self.path: Path | None = None
        self.started_at: float | None = None
        self.samples = 0
        self.events = 0
        self.bytes_written = 0
        self.last_error: str | None = None
        self.last_sample_at: float | None = None

    def status(self) -> dict[str, Any]:
        with self.lock:
            return self._status_locked()

    def _status_locked(self) -> dict[str, Any]:
        return {
            "active": self.file is not None,
            "path": str(self.path) if self.path else None,
            "filename": self.path.name if self.path else None,
            "started_at": self.started_at,
            "elapsed_seconds": round(time.time() - self.started_at, 3) if self.started_at else 0,
            "samples": self.samples,
            "events": self.events,
            "bytes_written": self.bytes_written,
            "last_sample_at": self.last_sample_at,
            "last_error": self.last_error,
        }

    def start(self, label: str | None = None) -> dict[str, Any]:
        with self.lock:
            if self.file is not None:
                return self.status()
            self.directory.mkdir(parents=True, exist_ok=True)
            safe_label = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in (label or "telemetry"))
            safe_label = safe_label.strip("_")[:48] or "telemetry"
            self.path = self.directory / f"{recording_timestamp()}-{safe_label}.jsonl"
            self.file = self.path.open("a", encoding="utf-8")
            self.started_at = time.time()
            self.samples = 0
            self.events = 0
            self.bytes_written = 0
            self.last_error = None
            self.last_sample_at = None
            self._write_locked(
                {
                    "type": "recording_start",
                    "timestamp": self.started_at,
                    "monotonic_ns": time.monotonic_ns(),
                    "schema": "h1_2_telemetry_jsonl_v1",
                    "body_joint_names": JOINT_NAMES,
                    "hand_joint_names": HAND_JOINT_NAMES,
                }
            )
            return self._status_locked()

    def stop(self) -> dict[str, Any]:
        with self.lock:
            if self.file is None:
                return self.status()
            self._write_locked(
                {
                    "type": "recording_stop",
                    "timestamp": time.time(),
                    "monotonic_ns": time.monotonic_ns(),
                    "samples": self.samples,
                    "events": self.events,
                }
            )
            with contextlib.suppress(Exception):
                self.file.flush()
                self.file.close()
            self.file = None
            return self._status_locked()

    def write_sample(self, sample: dict[str, Any]) -> None:
        with self.lock:
            if self.file is None:
                return
            try:
                self._write_locked(sample)
                self.samples += 1
                self.last_sample_at = sample.get("timestamp")
                if self.samples % 100 == 0:
                    self.file.flush()
            except Exception as exc:
                self.last_error = str(exc)

    def write_event(self, name: str, payload: dict[str, Any]) -> None:
        with self.lock:
            if self.file is None:
                return
            try:
                self._write_locked(
                    {
                        "type": "command_event",
                        "timestamp": time.time(),
                        "monotonic_ns": time.monotonic_ns(),
                        "name": name,
                        "payload": payload,
                    }
                )
                self.events += 1
            except Exception as exc:
                self.last_error = str(exc)

    def _write_locked(self, data: dict[str, Any]) -> None:
        if self.file is None:
            return
        line = json.dumps(data, ensure_ascii=False, separators=(",", ":")) + "\n"
        self.file.write(line)
        self.bytes_written += len(line.encode("utf-8"))


def compact_record_motor(index: int, motor: Any, names: dict[int, str]) -> dict[str, Any]:
    return fields_from(
        motor,
        [
            "mode",
            "q",
            "dq",
            "ddq",
            "tau",
            "tau_est",
            "temperature",
            "vol",
            "sensor",
            "reserve",
        ],
    ) | {"index": index, "name": names.get(index, f"Motor{index}")}


def lowstate_record(
    msg: Any,
    samples: int,
    hands: dict[str, Any],
    hand_samples: int,
    hand_timestamp: float | None,
) -> dict[str, Any]:
    timestamp = time.time()
    record = {
        "type": "telemetry_sample",
        "timestamp": timestamp,
        "monotonic_ns": time.monotonic_ns(),
        "sample": samples,
        "body": {
            "topic": "rt/lowstate",
            "motors": [compact_record_motor(i, motor, JOINT_NAMES) for i, motor in enumerate(msg.motor_state)],
            "imu": fields_from(
                msg.imu_state,
                ["quaternion", "gyroscope", "accelerometer", "rpy", "temperature"],
            ),
            "robot": fields_from(
                msg,
                ["version", "mode_pr", "mode_machine", "tick", "crc", "wireless_remote"],
            ),
        },
        "hands": hands,
        "hand_samples": hand_samples,
        "hand_timestamp": hand_timestamp,
    }
    if hasattr(msg, "bms_state"):
        record["body"]["battery"] = fields_from(
            msg.bms_state,
            ["version_h", "version_l", "bms_status", "soc", "current", "cycle", "temperature"],
        )
    if hasattr(msg, "foot_force"):
        record["body"]["foot_force"] = listify(msg.foot_force)
    if hasattr(msg, "foot_force_est"):
        record["body"]["foot_force_est"] = listify(msg.foot_force_est)
    return record


def has_risk_ack(payload: dict[str, Any]) -> bool:
    return payload.get("armed") is True and payload.get("i_understand_risk") is True


def public_host() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def default_interface() -> str | None:
    try:
        for line in Path("/proc/net/route").read_text().splitlines()[1:]:
            fields = line.split()
            if len(fields) >= 2 and fields[1] == "00000000":
                return fields[0]
    except OSError:
        return None
    return None


def network_type(interface: str | None) -> str:
    if not interface:
        return "Network"
    lowered = interface.lower()
    if lowered.startswith(("eth", "en")):
        return "Ethernet"
    if lowered.startswith(("wl", "wifi")):
        return "Wi-Fi"
    if lowered.startswith(("ww", "wwan", "cell", "usb")):
        return "Cellular"
    if lowered.startswith(("tun", "tap", "wg")):
        return "VPN"
    return interface


def route_interface(destination: str) -> str | None:
    try:
        output = subprocess.check_output(
            ["ip", "route", "get", destination],
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=1,
        )
    except Exception:
        return None

    parts = output.split()
    if "dev" in parts:
        index = parts.index("dev")
        if index + 1 < len(parts):
            return parts[index + 1]
    return None


def interface_status(interface: str | None, host: str | None = None) -> dict[str, Any]:
    return {
        "type": network_type(interface),
        "interface": interface or "unknown",
        "host": host or public_host(),
        "quality": "Connected" if interface else "Disconnected",
    }


def network_status(robot_host: str) -> dict[str, Any]:
    return {
        "host": interface_status(default_interface()),
        "robot": {
            **interface_status(route_interface(robot_host), robot_host),
            "target": robot_host,
        },
    }


def finite_number(value: Any) -> Any:
    if isinstance(value, float):
        if value != value or value in (float("inf"), float("-inf")):
            return None
        return round(value, 6)
    return value


def listify(value: Any) -> list[Any]:
    try:
        return [finite_number(item) for item in value]
    except TypeError:
        return []


def fields_from(obj: Any, names: list[str]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for name in names:
        if hasattr(obj, name):
            value = getattr(obj, name)
            if isinstance(value, (str, int, bool)) or value is None:
                out[name] = value
            elif isinstance(value, float):
                out[name] = finite_number(value)
            elif isinstance(value, (list, tuple)):
                out[name] = listify(value)
            else:
                try:
                    out[name] = listify(value)
                except Exception:
                    out[name] = str(value)
    return out


def motor_to_dict(index: int, motor: Any) -> dict[str, Any]:
    data = fields_from(
        motor,
        [
            "mode",
            "q",
            "dq",
            "ddq",
            "tau_est",
            "temperature",
            "vol",
            "sensor",
            "reserve",
        ],
    )
    data["index"] = index
    data["name"] = JOINT_NAMES.get(index, f"ReservedMotorSlot{index}")
    return data


def hand_motor_to_dict(index: int, motor: Any) -> dict[str, Any]:
    data = fields_from(
        motor,
        [
            "mode",
            "q",
            "dq",
            "ddq",
            "tau",
            "tau_est",
            "temperature",
            "vol",
            "sensor",
            "reserve",
        ],
    )
    data["index"] = index
    data["name"] = HAND_JOINT_NAMES.get(index, f"HandMotor{index}")
    return data


def handstate_to_dict(msg: Any | None, samples: int, timestamp: float | None) -> dict[str, Any]:
    if msg is None:
        return {
            "connected": False,
            "topic": HAND_STATE_TOPIC,
            "samples": samples,
            "timestamp": timestamp,
            "joints": [],
            "note": "No hand state received. Start inspire_h1 service if the RH56BFX hands are connected over serial.",
        }

    states = getattr(msg, "states", [])
    return {
        "connected": True,
        "topic": HAND_STATE_TOPIC,
        "samples": samples,
        "timestamp": timestamp,
        "joint_count": len(states),
        "joints": [hand_motor_to_dict(i, motor) for i, motor in enumerate(states)],
    }


def numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        if math.isfinite(float(value)):
            return float(value)
    return None


def motor_temperature(motor: dict[str, Any]) -> float | None:
    values = [numeric(value) for value in motor.get("temperature", [])]
    values = [value for value in values if value is not None and value > 0]
    return max(values) if values else None


def is_reserved_motor(motor: dict[str, Any]) -> bool:
    return str(motor.get("name", "")).startswith("ReservedMotorSlot")


def compact_motor(motor: dict[str, Any], value: float | None = None) -> dict[str, Any] | None:
    if not motor:
        return None
    data: dict[str, Any] = {
        "index": motor.get("index"),
        "name": motor.get("name"),
    }
    if value is not None:
        data["value"] = round(value, 6)
    return data


def summarize_motor_groups(motors: list[dict[str, Any]]) -> dict[str, Any]:
    by_index = {motor.get("index"): motor for motor in motors}
    groups: dict[str, Any] = {}
    for group, indexes in JOINT_GROUPS.items():
        group_motors = [by_index[index] for index in indexes if index in by_index]
        temps = [motor_temperature(motor) for motor in group_motors]
        temps = [temp for temp in temps if temp is not None]
        groups[group] = {
            "count": len(group_motors),
            "moving": sum(1 for motor in group_motors if abs(numeric(motor.get("dq")) or 0.0) > 0.05),
            "max_temperature": round(max(temps), 1) if temps else None,
        }
    return groups


def summarize_motors(motors: list[dict[str, Any]]) -> dict[str, Any]:
    real_motors = [motor for motor in motors if not is_reserved_motor(motor)]
    reserved_motors = [motor for motor in motors if is_reserved_motor(motor)]
    mode_counts: dict[str, int] = {}
    for motor in real_motors:
        key = str(motor.get("mode", "unknown"))
        mode_counts[key] = mode_counts.get(key, 0) + 1

    hottest = max(real_motors, key=lambda motor: motor_temperature(motor) or -1.0, default={})
    max_tau = max(real_motors, key=lambda motor: abs(numeric(motor.get("tau_est")) or 0.0), default={})
    max_velocity = max(real_motors, key=lambda motor: abs(numeric(motor.get("dq")) or 0.0), default={})

    return {
        "real_count": len(real_motors),
        "reserved_count": len(reserved_motors),
        "mode_counts": mode_counts,
        "moving_count": sum(
            1 for motor in real_motors if abs(numeric(motor.get("dq")) or 0.0) > 0.05
        ),
        "hottest": compact_motor(hottest, motor_temperature(hottest)) if hottest else None,
        "max_abs_tau": compact_motor(max_tau, abs(numeric(max_tau.get("tau_est")) or 0.0))
        if max_tau
        else None,
        "max_abs_velocity": compact_motor(max_velocity, abs(numeric(max_velocity.get("dq")) or 0.0))
        if max_velocity
        else None,
        "groups": summarize_motor_groups(motors),
    }


def summarize_imu(imu: dict[str, Any]) -> dict[str, Any]:
    rpy = imu.get("rpy") or []
    roll, pitch, yaw = (list(rpy) + [None, None, None])[:3]
    return {
        "roll_deg": round(math.degrees(roll), 2) if numeric(roll) is not None else None,
        "pitch_deg": round(math.degrees(pitch), 2) if numeric(pitch) is not None else None,
        "yaw_deg": round(math.degrees(yaw), 2) if numeric(yaw) is not None else None,
        "temperature": imu.get("temperature"),
    }


def health_flags(snapshot: dict[str, Any], motor_summary: dict[str, Any]) -> list[dict[str, str]]:
    flags: list[dict[str, str]] = []
    if not snapshot.get("connected"):
        flags.append({"level": "critical", "message": "No LowState telemetry is being received."})

    mode_counts = motor_summary.get("mode_counts", {})
    real_count = motor_summary.get("real_count", 0)
    if real_count and mode_counts == {"0": real_count}:
        flags.append({"level": "info", "message": "All real motors report mode 0, so the robot is passive/idle."})

    hottest = motor_summary.get("hottest") or {}
    hottest_value = numeric(hottest.get("value"))
    if hottest_value is not None and hottest_value >= 70:
        flags.append(
            {
                "level": "warning",
                "message": f"Hottest motor is {hottest.get('name')} at {round(hottest_value, 1)} C.",
            }
        )

    imu_temp = numeric((snapshot.get("imu") or {}).get("temperature"))
    if imu_temp is not None and imu_temp >= 75:
        flags.append({"level": "warning", "message": f"IMU temperature is {round(imu_temp, 1)} C."})

    if not (snapshot.get("hands") or {}).get("connected"):
        flags.append({"level": "info", "message": "Hand telemetry is offline on rt/inspire/state."})

    if (snapshot.get("battery") or {}).get("state"):
        flags.append({"level": "info", "message": "Battery details are not exposed by this LowState firmware."})

    return flags


def analyze_snapshot(snapshot: dict[str, Any]) -> dict[str, Any]:
    motor_summary = summarize_motors(snapshot.get("motors", []))
    imu_summary = summarize_imu(snapshot.get("imu", {}))
    flags = health_flags(snapshot, motor_summary)
    return {
        "motors": motor_summary,
        "imu": imu_summary,
        "health": {
            "state": "warning" if any(flag["level"] == "warning" for flag in flags) else "ok",
            "flags": flags,
        },
    }


def lowstate_to_dict(
    msg: Any, samples: int, rate_hz: float, hands: dict[str, Any] | None = None
) -> dict[str, Any]:
    motors = [motor_to_dict(i, motor) for i, motor in enumerate(msg.motor_state)]
    imu = fields_from(
        msg.imu_state,
        ["quaternion", "gyroscope", "accelerometer", "rpy", "temperature"],
    )

    data = {
        "connected": True,
        "timestamp": time.time(),
        "samples": samples,
        "sample_rate_hz": round(rate_hz, 2),
        "motor_count": len(motors),
        "motors": motors,
        "imu": imu,
        "robot": fields_from(
            msg,
            [
                "version",
                "mode_pr",
                "mode_machine",
                "tick",
                "crc",
                "wireless_remote",
            ],
        ),
        "hands": hands or handstate_to_dict(None, 0, None),
    }

    if hasattr(msg, "bms_state"):
        data["battery"] = fields_from(
            msg.bms_state,
            ["version_h", "version_l", "bms_status", "soc", "current", "cycle", "temperature"],
        )
    else:
        data["battery"] = {
            "state": "not exposed by this LowState firmware",
            "checked_fields": ["bms_state", "battery_state", "power_v", "power_a"],
        }

    if hasattr(msg, "foot_force"):
        data["foot_force"] = listify(msg.foot_force)
    if hasattr(msg, "foot_force_est"):
        data["foot_force_est"] = listify(msg.foot_force_est)

    data["analysis"] = analyze_snapshot(data)
    return data


def h264_payload_from_raw_ros(data: bytes, target_resolution: int = 360) -> bytes | None:
    offset = 4
    if len(data) < offset + 8:
        return None
    offset += 8
    payloads: dict[int, bytes] = {}
    for resolution in (720, 360, 180):
        if offset + 4 > len(data):
            break
        payload_size = struct.unpack_from("<I", data, offset)[0]
        offset += 4
        if payload_size > len(data) - offset:
            break
        payload = data[offset : offset + payload_size]
        offset += payload_size
        while offset % 4:
            offset += 1
        if not payload:
            continue
        payloads[resolution] = payload
    return payloads.get(target_resolution) or payloads.get(720) or payloads.get(360) or payloads.get(180)


def clean_h264_payload(payload: bytes) -> bytes | None:
    for marker in (b"\x00\x00\x00\x01", b"\x00\x00\x01"):
        index = payload.find(marker)
        if index >= 0:
            return payload[index:]
    return payload or None


def h264_payload_from_video_msg(msg: Any, target_resolution: int = 360) -> bytes | None:
    fields = {
        720: bytes(getattr(msg, "video720p", b"")),
        360: bytes(getattr(msg, "video360p", b"")),
        180: bytes(getattr(msg, "video180p", b"")),
    }
    payload = fields.get(target_resolution) or fields[720] or fields[360] or fields[180]
    return clean_h264_payload(payload) if payload else None


def configure_ros2_camera_environment(interface: str) -> None:
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    if interface and "CYCLONEDDS_URI" not in os.environ:
        os.environ["CYCLONEDDS_URI"] = (
            "<CycloneDDS><Domain><General><Interfaces>"
            f'<NetworkInterface name="{interface}" priority="default" multicast="default" />'
            "</Interfaces></General></Domain></CycloneDDS>"
        )


def ros2_command() -> list[str] | None:
    setup = Path("/opt/ros/humble/setup.bash")
    if setup.exists():
        return ["bash", "-lc"]
    ros2_bin = os.environ.get("ROS2_BIN") or shutil.which("ros2")
    if ros2_bin:
        return [ros2_bin]
    for candidate in (Path("/opt/ros/humble/bin/ros2"), Path("/opt/ros/foxy/bin/ros2")):
        if candidate.exists():
            return [str(candidate)]
    return None


def ros2_environment() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    ros_python = Path("/opt/ros/humble/lib/python3.10/site-packages")
    if ros_python.exists():
        env["PYTHONPATH"] = f"{ros_python}{os.pathsep}{env.get('PYTHONPATH', '')}".rstrip(os.pathsep)
    ros_local_python = Path("/opt/ros/humble/local/lib/python3.10/dist-packages")
    if ros_local_python.exists():
        env["PYTHONPATH"] = f"{ros_local_python}{os.pathsep}{env.get('PYTHONPATH', '')}".rstrip(os.pathsep)
    ros_bin = Path("/opt/ros/humble/bin")
    if ros_bin.exists():
        env["PATH"] = f"{ros_bin}{os.pathsep}{env.get('PATH', '')}".rstrip(os.pathsep)
    return env


def run_ros2_command(args: list[str], timeout: float = 2.5) -> tuple[bool, str]:
    command = ros2_command()
    if command is None:
        return False, "ros2 executable was not found. Install ROS 2 or set ROS2_BIN."
    if command == ["bash", "-lc"]:
        import shlex

        shell_args = " ".join(shlex.quote(arg) for arg in args)
        command = ["bash", "-lc", f"source /opt/ros/humble/setup.bash && exec ros2 {shell_args}"]
    else:
        command = [*command, *args]
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            check=False,
            env=ros2_environment(),
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    output = result.stdout.strip() or result.stderr.strip()
    return result.returncode == 0, output


def parse_topic_list(output: str) -> dict[str, list[str]]:
    topics: dict[str, list[str]] = {}
    for line in output.splitlines():
        line = line.strip()
        if not line or " [" not in line or not line.endswith("]"):
            continue
        name, raw_types = line.rsplit(" [", 1)
        topics[name] = [item.strip() for item in raw_types[:-1].split(",") if item.strip()]
    return topics


def parse_node_info(name: str, output: str) -> dict[str, Any]:
    sections = {
        "Subscribers:": "subscribers",
        "Publishers:": "publishers",
        "Service Servers:": "service_servers",
        "Service Clients:": "service_clients",
        "Action Servers:": "action_servers",
        "Action Clients:": "action_clients",
    }
    node = {key: [] for key in sections.values()}
    node["name"] = name
    current: str | None = None
    for raw_line in output.splitlines():
        stripped = raw_line.strip()
        if stripped in sections:
            current = sections[stripped]
            continue
        if current is None or not raw_line.startswith("    ") or ":" not in stripped:
            continue
        topic, msg_type = stripped.split(":", 1)
        node[current].append({"name": topic.strip(), "type": msg_type.strip()})
    return node


def collect_ros_graph(interface: str) -> dict[str, Any]:
    configure_ros2_camera_environment(interface)
    timestamp = time.time()
    ok_nodes, node_output = run_ros2_command(["node", "list"])
    ok_topics, topic_output = run_ros2_command(["topic", "list", "-t"])
    if not ok_nodes:
        return {"timestamp": timestamp, "nodes": [], "topics": {}, "subscriptions": [], "error": node_output}

    node_names = [line.strip() for line in node_output.splitlines() if line.strip()]
    topic_types = parse_topic_list(topic_output if ok_topics else "")
    nodes = []
    subscriptions = []
    publishers = []
    for node_name in node_names[:40]:
        ok_info, info_output = run_ros2_command(["node", "info", node_name], timeout=2.0)
        if not ok_info:
            nodes.append({"name": node_name, "subscribers": [], "publishers": [], "error": info_output})
            continue
        node = parse_node_info(node_name, info_output)
        nodes.append(node)
        for sub in node["subscribers"]:
            subscriptions.append({"node": node_name, "topic": sub["name"], "type": sub["type"]})
        for pub in node["publishers"]:
            publishers.append({"node": node_name, "topic": pub["name"], "type": pub["type"]})

    return {
        "timestamp": timestamp,
        "interface": interface or "default",
        "nodes": nodes,
        "topics": topic_types,
        "subscriptions": subscriptions,
        "publishers": publishers,
        "error": None if ok_topics else topic_output,
    }


def camera_decoder_worker(store: "TelemetryStore", fifo_path: str) -> None:
    if cv2 is None:
        store.set_camera_error("OpenCV is not available for H264 decode.")
        return
    os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "quiet")
    try:
        cv2.setLogLevel(0)
    except AttributeError:
        pass
    try:
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, 2)
        os.close(devnull)
    except OSError:
        pass
    cap = cv2.VideoCapture(fifo_path, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        store.set_camera_error("OpenCV could not open H264 camera stream.")
        return
    while store.running:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.02)
            continue
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        if ok:
            store.set_camera_frame(encoded.tobytes())
    cap.release()


def camera_fifo_writer(store: "TelemetryStore", fifo_path: str, payloads: "queue.Queue[bytes]") -> None:
    while store.running:
        try:
            with open(fifo_path, "wb", buffering=0) as stream:
                while store.running:
                    stream.write(payloads.get())
        except BrokenPipeError:
            time.sleep(0.1)
        except OSError as exc:
            store.set_camera_error(f"H264 pipe writer failed: {exc}")
            time.sleep(0.5)


def decode_h264_file(path: str) -> bytes | None:
    if cv2 is None:
        return None
    cap = cv2.VideoCapture(path, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        return None
    latest = None
    try:
        for _ in range(120):
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            latest = frame
    finally:
        cap.release()
    if latest is None:
        return None
    ok, encoded = cv2.imencode(".jpg", latest, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    return encoded.tobytes() if ok else None


def camera_buffer_decoder_worker(store: "TelemetryStore", payloads: "queue.Queue[bytes]") -> None:
    if cv2 is None:
        store.set_camera_error("OpenCV is not available for H264 decode.")
        return
    os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "quiet")
    buffer = bytearray()
    max_bytes = 4_000_000
    decode_path = f"/tmp/robot_telemetry_front_camera_{os.getpid()}.h264"
    next_decode = 0.0
    last_frame_at = 0.0
    while store.running:
        try:
            payload = payloads.get(timeout=0.2)
        except queue.Empty:
            continue
        buffer.extend(payload)
        if len(buffer) > max_bytes:
            del buffer[: len(buffer) - max_bytes]
            first_start = buffer.find(b"\x00\x00\x00\x01")
            if first_start > 0:
                del buffer[:first_start]
        now = time.time()
        if now < next_decode or len(buffer) < 32_000:
            continue
        next_decode = now + 0.25
        try:
            Path(decode_path).write_bytes(buffer)
            frame = decode_h264_file(decode_path)
        except Exception as exc:
            store.set_camera_error(f"H264 buffer decode failed: {exc}")
            continue
        if frame is not None:
            store.set_camera_frame(frame)
            last_frame_at = now
        elif last_frame_at == 0.0 or now - last_frame_at > 3.0:
            store.set_camera_error("Waiting for H264 keyframe from front video stream.")


def camera_bridge_main(camera_source: str, resolution: int, output_path: Path) -> None:
    configure_ros2_camera_environment(camera_source)
    try:
        import rclpy
        from rclpy.node import Node
        from unitree_go.msg import Go2FrontVideoData
    except Exception as exc:
        print(f"Could not import ROS2 camera dependencies: {exc}", file=sys.stderr)
        return

    payloads: queue.Queue[bytes] = queue.Queue(maxsize=240)

    def decoder() -> None:
        buffer = bytearray()
        max_bytes = 4_000_000
        decode_path = Path(f"/tmp/robot_telemetry_front_camera_bridge_{os.getpid()}.h264")
        next_decode = 0.0
        while True:
            payload = payloads.get()
            buffer.extend(payload)
            if len(buffer) > max_bytes:
                del buffer[: len(buffer) - max_bytes]
                first_start = buffer.find(b"\x00\x00\x00\x01")
                if first_start > 0:
                    del buffer[:first_start]
            now = time.time()
            if now < next_decode or len(buffer) < 8_000:
                continue
            next_decode = now + 0.2
            try:
                decode_path.write_bytes(buffer)
                frame = decode_h264_file(str(decode_path))
                if frame is None:
                    continue
                tmp_path = output_path.with_suffix(".jpg.tmp")
                tmp_path.write_bytes(frame)
                os.replace(tmp_path, output_path)
            except Exception as exc:
                print(f"Camera bridge decode failed: {exc}", file=sys.stderr)

    threading.Thread(target=decoder, daemon=True).start()

    rclpy.init(args=None)

    class FrontVideoNode(Node):
        def __init__(self) -> None:
            super().__init__("robot_telemetry_front_video_bridge")
            self.create_subscription(Go2FrontVideoData, "/frontvideostream", self.on_frame, 10)

        def on_frame(self, msg: Any) -> None:
            payload = h264_payload_from_video_msg(msg, resolution)
            if not payload:
                return
            try:
                payloads.put_nowait(payload)
            except queue.Full:
                with contextlib.suppress(queue.Empty):
                    payloads.get_nowait()
                payloads.put_nowait(payload)

    node = FrontVideoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def camera_file_watcher(store: "TelemetryStore", image_path: Path) -> None:
    last_mtime = 0.0
    while store.running:
        try:
            stat = image_path.stat()
            if stat.st_mtime != last_mtime and stat.st_size > 0:
                frame = image_path.read_bytes()
                if frame.startswith(b"\xff\xd8"):
                    last_mtime = stat.st_mtime
                    store.set_camera_frame(frame)
            elif store.camera_frame is None:
                store.set_camera_error("Waiting for camera bridge frame.")
        except FileNotFoundError:
            if store.camera_frame is None:
                store.set_camera_error("Waiting for camera bridge frame.")
        except OSError as exc:
            store.set_camera_error(f"Camera frame watcher failed: {exc}")
        time.sleep(0.1)


def teleimager_camera_worker(store: "TelemetryStore") -> None:
    host = os.environ.get("TELEIMAGER_HOST", "127.0.0.1")
    try:
        from teleimager.image_client import ImageClient
    except Exception as exc:
        store.set_camera_error(f"Teleimager client is not available: {exc}")
        return

    client = None
    last_error = 0.0
    last_jpg = None
    while store.running:
        try:
            if client is None:
                client = ImageClient(host=host, request_bgr=False)
                store.camera_topic = "teleimager/head"
            frame = client.get_head_frame()
            jpg = frame.jpg if frame else None
            if jpg and jpg is not last_jpg and jpg.startswith(b"\xff\xd8"):
                last_jpg = jpg
                store.set_camera_frame(jpg)
            elif store.camera_frame is None and time.time() - last_error > 2.0:
                last_error = time.time()
                store.set_camera_error("Waiting for Teleimager head camera frame.")
            time.sleep(0.04)
        except Exception as exc:
            if client is not None:
                with contextlib.suppress(Exception):
                    client.close()
            client = None
            last_jpg = None
            if time.time() - last_error > 2.0:
                last_error = time.time()
                store.set_camera_error(f"Teleimager camera failed: {exc}")
            time.sleep(1.0)

    if client is not None:
        with contextlib.suppress(Exception):
            client.close()


def start_camera_bridge(store: "TelemetryStore") -> None:
    with contextlib.suppress(FileNotFoundError):
        CAMERA_JPEG_PATH.unlink()
    backend = (store.camera_backend or "auto").lower()
    if backend in ("auto", "teleimager"):
        threading.Thread(target=teleimager_camera_worker, args=(store,), daemon=True).start()
    if backend != "ros2":
        return
    cmd = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--camera-bridge",
        "--camera-source",
        store.camera_source,
        "--camera-resolution",
        str(store.camera_resolution),
        "--camera-output",
        str(CAMERA_JPEG_PATH),
    ]
    store.camera_process = subprocess.Popen(
        cmd,
        cwd=str(APP_DIR),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    threading.Thread(target=camera_file_watcher, args=(store, CAMERA_JPEG_PATH), daemon=True).start()


def ros_camera_worker(store: "TelemetryStore") -> None:
    configure_ros2_camera_environment(store.camera_source)
    try:
        import rclpy
        from rclpy.node import Node
        from unitree_go.msg import Go2FrontVideoData
    except Exception as exc:
        store.set_camera_error(f"Could not import ROS2 camera dependencies: {exc}")
        return

    payloads: queue.Queue[bytes] = queue.Queue(maxsize=240)
    threading.Thread(target=camera_buffer_decoder_worker, args=(store, payloads), daemon=True).start()

    rclpy.init(args=None)

    class FrontVideoNode(Node):
        def __init__(self) -> None:
            super().__init__("robot_telemetry_front_video")
            self.create_subscription(Go2FrontVideoData, "/frontvideostream", self.on_frame, 10)

        def on_frame(self, msg: Any) -> None:
            if isinstance(msg, bytes):
                payload = h264_payload_from_raw_ros(bytes(msg), store.camera_resolution)
                payload = clean_h264_payload(payload) if payload else None
            else:
                payload = h264_payload_from_video_msg(msg, store.camera_resolution)
            if not payload:
                store.set_camera_error("Front video packet did not contain H264 payload.")
                return
            try:
                payloads.put_nowait(payload)
            except queue.Full:
                try:
                    payloads.get_nowait()
                except queue.Empty:
                    pass
                payloads.put_nowait(payload)

    node = FrontVideoNode()
    store.set_camera_error(None)
    try:
        while store.running:
            rclpy.spin_once(node, timeout_sec=0.2)
    except Exception as exc:
        store.set_camera_error(f"ROS2 front video subscriber failed: {exc}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


class TelemetryStore:
    def __init__(self, domain: int, robot_host: str) -> None:
        self.domain = domain
        self.robot_host = robot_host
        self.camera_source = os.environ.get("CAMERA_SOURCE", "")
        self.lock = threading.Lock()
        self.camera_lock = threading.Lock()
        self.camera_condition = threading.Condition(self.camera_lock)
        self.camera_frame: bytes | None = None
        self.camera_timestamp: float | None = None
        self.camera_error: str | None = None
        self.camera_process: subprocess.Popen[bytes] | None = None
        self.camera_backend = os.environ.get("CAMERA_BACKEND", "auto")
        self.camera_topic = "/frontvideostream"
        self.camera_resolution = int(os.environ.get("CAMERA_RESOLUTION", "360"))
        self.recorder = TelemetryRecorder(RECORDINGS_DIR)
        self.latest: dict[str, Any] = {
            "connected": False,
            "network": network_status(self.robot_host),
            "timestamp": None,
            "samples": 0,
            "sample_rate_hz": 0,
            "motor_count": 0,
            "motors": [],
            "imu": {},
            "robot": {},
            "hands": handstate_to_dict(None, 0, None),
            "error": "Subscriber has not started yet.",
        }
        self.running = False
        self.thread: threading.Thread | None = None
        self.sample_times: deque[float] = deque(maxlen=300)
        self.samples = 0
        self.ros_graph_cache: dict[str, Any] | None = None
        self.ros_graph_timestamp = 0.0
        self.lowstate_msg: Any | None = None
        self.command_lock = threading.Lock()
        self.wrist_publisher: Any | None = None
        self.lowcmd_publisher: Any | None = None
        self.motion_switcher: Any | None = None
        self.loco_client: Any | None = None
        self.lowcmd_factory: Any | None = None
        self.lowcmd_type: Any | None = None
        self.crc: Any | None = None
        self.wrist_cancel: threading.Event | None = None
        self.wrist_thread: threading.Thread | None = None
        self.replay_cancel: threading.Event | None = None
        self.replay_thread: threading.Thread | None = None
        self.torso_cancel: threading.Event | None = None
        self.torso_thread: threading.Thread | None = None
        self.torso_target_q: float = 0.0
        self.torso_status: dict[str, Any] = {
            "available": False,
            "active": False,
            "message": "Torso twist idle.",
            "target_q": 0.0,
            "last_command": None,
            "updated_at": None,
        }
        self.wrist_status: dict[str, Any] = {
            "available": False,
            "active": False,
            "message": "DDS command publisher has not started yet.",
            "last_command": None,
            "updated_at": None,
        }
        self.loco_status: dict[str, Any] = {
            "available": False,
            "active": False,
            "message": "H1 loco client has not started yet.",
            "last_command": None,
            "history": [],
            "motion_mode": None,
            "updated_at": None,
        }

    def start(self) -> None:
        self.running = True
        self.thread = threading.Thread(target=self._run, name="unitree-lowstate", daemon=True)
        self.thread.start()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            latest = dict(self.latest)
        with self.command_lock:
            loco_status = dict(self.loco_status)
            loco_available = bool(self.loco_client)
        robot = latest.get("robot") or {}
        return {
            **latest,
            "network": network_status(self.robot_host),
            "loco": self._loco_status_payload(loco_status, robot, loco_available, include_metadata=False),
        }

    def camera_snapshot(self) -> dict[str, Any]:
        with self.camera_lock:
            return {
                "source": self.camera_topic,
                "interface": self.camera_source or "default",
                "backend": self.camera_backend,
                "resolution": self.camera_resolution,
                "available": self.camera_frame is not None,
                "timestamp": self.camera_timestamp,
                "error": self.camera_error,
            }

    def ros_graph_snapshot(self) -> dict[str, Any]:
        now = time.time()
        if self.ros_graph_cache is not None and now - self.ros_graph_timestamp < 3.0:
            return self.ros_graph_cache
        graph = collect_ros_graph(self.camera_source)
        self.ros_graph_cache = graph
        self.ros_graph_timestamp = now
        return graph

    def recording_status(self) -> dict[str, Any]:
        return self.recorder.status()

    def recording_files(self) -> dict[str, Any]:
        RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
        files = []
        paths = [
            *RECORDINGS_DIR.glob("*.jsonl"),
            *RECORDINGS_DIR.glob("*.pose.json"),
            *RECORDINGS_DIR.glob("*.sequence.json"),
        ]
        for path in sorted(paths, key=lambda item: item.stat().st_mtime, reverse=True):
            try:
                stat = path.stat()
            except OSError:
                continue
            files.append(
                {
                    "name": path.name,
                    "path": str(path),
                    "size": stat.st_size,
                    "modified_at": stat.st_mtime,
                }
            )
        return {"files": files}

    def recording_file_path(self, filename: str) -> Path:
        name = Path(filename).name
        if not (name.endswith(".jsonl") or name.endswith(".pose.json") or name.endswith(".sequence.json")):
            raise ValueError("Recording filename must end with .jsonl, .pose.json, or .sequence.json")
        path = (RECORDINGS_DIR / name).resolve()
        root = RECORDINGS_DIR.resolve()
        if root not in path.parents:
            raise ValueError("Recording path is outside the recordings directory")
        if not path.exists():
            raise FileNotFoundError(name)
        return path

    def diagram_files(self) -> dict[str, Any]:
        DOCS_DIR.mkdir(parents=True, exist_ok=True)
        files = []
        for path in sorted(DOCS_DIR.glob("*.drawio"), key=lambda item: item.name.lower()):
            try:
                stat = path.stat()
            except OSError:
                continue
            files.append(
                {
                    "name": path.name,
                    "size": stat.st_size,
                    "modified_at": stat.st_mtime,
                }
            )
        return {"files": files}

    def diagram_file_path(self, filename: str) -> Path:
        name = Path(filename).name
        if not name.endswith(".drawio"):
            raise ValueError("Diagram filename must end with .drawio")
        path = (DOCS_DIR / name).resolve()
        root = DOCS_DIR.resolve()
        if root not in path.parents:
            raise ValueError("Diagram path is outside the docs directory")
        if not path.exists():
            raise FileNotFoundError(name)
        return path

    def start_recording(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        label = str(payload.get("label", "telemetry")).strip() if payload else "telemetry"
        status = self.recorder.start(label)
        return 200, {"ok": True, "status": status}

    def stop_recording(self) -> tuple[int, dict[str, Any]]:
        status = self.recorder.stop()
        return 200, {"ok": True, "status": status}

    def capture_pose(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        label = str(payload.get("label", "pose_point")).strip() if payload else "pose_point"
        safe_label = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in label)
        safe_label = safe_label.strip("_")[:48] or "pose_point"
        payload_snapshot = payload.get("snapshot") if isinstance(payload, dict) else None
        if isinstance(payload_snapshot, dict):
            snapshot = json.loads(json.dumps(payload_snapshot))
        else:
            with self.lock:
                snapshot = json.loads(json.dumps(self.latest))
        if not snapshot.get("motors"):
            return 409, {"ok": False, "error": "No body motor telemetry is available to capture as a pose point."}
        RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
        path = RECORDINGS_DIR / f"{recording_timestamp()}-{safe_label}.pose.json"
        payload_out = {
            "type": "pose_point",
            "schema": "h1_2_pose_point_v1",
            "timestamp": time.time(),
            "monotonic_ns": time.monotonic_ns(),
            "snapshot": snapshot,
        }
        path.write_text(json.dumps(payload_out, ensure_ascii=False, indent=2), encoding="utf-8")
        return 200, {
            "ok": True,
            "file": {
                "name": path.name,
                "path": str(path),
                "size": path.stat().st_size,
                "modified_at": path.stat().st_mtime,
            },
        }

    def save_sequence(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        label = str(payload.get("label", "sequence")).strip() if payload else "sequence"
        safe_label = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in label)
        safe_label = safe_label.strip("_")[:48] or "sequence"
        points = payload.get("points") if isinstance(payload, dict) else None
        if not isinstance(points, list) or not points:
            return 400, {"ok": False, "error": "Sequence requires a non-empty points array."}
        clean_points = []
        for index, point in enumerate(points):
            if not isinstance(point, dict):
                return 400, {"ok": False, "error": f"Point {index + 1} must be an object."}
            snapshot = json.loads(json.dumps(point))
            if not snapshot.get("motors"):
                return 400, {"ok": False, "error": f"Point {index + 1} does not contain motors."}
            snapshot.setdefault("type", "telemetry_sample")
            snapshot.setdefault("timestamp", time.time() + index * TRAJECTORY_DEFAULT_DT)
            clean_points.append(snapshot)
        RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
        path = RECORDINGS_DIR / f"{recording_timestamp()}-{safe_label}.sequence.json"
        payload_out = {
            "type": "trajectory",
            "schema": "h1_2_sequence_v1",
            "timestamp": time.time(),
            "monotonic_ns": time.monotonic_ns(),
            "points": clean_points,
        }
        path.write_text(json.dumps(payload_out, ensure_ascii=False, indent=2), encoding="utf-8")
        return 200, {
            "ok": True,
            "file": {
                "name": path.name,
                "path": str(path),
                "size": path.stat().st_size,
                "modified_at": path.stat().st_mtime,
            },
        }

    def request_robot_replay(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        filename = str(payload.get("filename", "")).strip()
        command_scope = str(payload.get("command_scope", "all") or "all").strip()
        if command_scope not in REPLAY_COMMAND_SCOPES:
            return 400, {
                "ok": False,
                "error": f"command_scope must be one of {', '.join(sorted(REPLAY_COMMAND_SCOPES))}.",
            }
        # Preview/simulate is no longer required before robot playback; the other
        # gates (arm_sdk-only, trajectory validity, XR suspend, gains) still apply.
        try:
            path = self.recording_file_path(filename)
        except FileNotFoundError:
            return 404, {"ok": False, "error": "Recording file was not found."}
        except ValueError as exc:
            return 400, {"ok": False, "error": str(exc)}
        plan = self.plan_replay_control_path(path, command_scope=command_scope)
        if payload.get("dry_run") is True:
            return 200, {"ok": True, "recording": path.name, "plan": plan}
        if payload.get("execute_arm_sdk") is True:
            return self.execute_arm_sdk_replay(path, plan, payload)
        return 409, {
            "ok": False,
            "error": (
                "Robot playback is intentionally locked. The recording preview is valid, "
                "but sending raw recorded joint trajectories to the physical robot requires "
                "a safety controller with interpolation, joint/velocity/torque limits, "
                "controller ownership checks, and emergency stop supervision."
            ),
            "recording": path.name,
            "plan": plan,
        }

    def execute_arm_sdk_replay(
        self,
        path: Path,
        plan: dict[str, Any],
        payload: dict[str, Any] | None = None,
    ) -> tuple[int, dict[str, Any]]:
        if plan.get("control_path") != "arm_sdk":
            return 409, {"ok": False, "error": "Only arm_sdk arm/waist trajectories are enabled.", "recording": path.name, "plan": plan}
        if not plan.get("valid_for_execution"):
            return 409, {"ok": False, "error": "Trajectory failed safety validation.", "recording": path.name, "plan": plan}
        if plan.get("hand_plan", {}).get("enabled"):
            return 409, {"ok": False, "error": "Hand/finger execution is not enabled yet; arm_sdk body trajectory was not published.", "recording": path.name, "plan": plan}
        frames = self._recording_trajectory_frames(path)
        if not frames:
            return 400, {"ok": False, "error": "Trajectory has no frames.", "recording": path.name, "plan": plan}
        with self.command_lock:
            publisher = self.wrist_publisher
            msg = self.lowstate_msg
            previous_cancel = self.replay_cancel
            torso_cancel = self.torso_cancel
        if publisher is None:
            return 503, {"ok": False, "error": "DDS arm_sdk publisher is not available.", "recording": path.name, "plan": plan}
        if msg is None:
            return 503, {"ok": False, "error": "No rt/lowstate sample is available yet.", "recording": path.name, "plan": plan}
        if self.lowcmd_factory is None or self.crc is None:
            return 503, {"ok": False, "error": "DDS command factory is not available.", "recording": path.name, "plan": plan}
        if previous_cancel is not None:
            previous_cancel.set()
        if torso_cancel is not None:
            torso_cancel.set()

        payload = payload or {}
        closed_loop = bool(payload.get("closed_loop", ARM_REPLAY_CLOSED_LOOP_DEFAULT))
        hold_after_convergence = bool(
            payload.get("hold_after_convergence", ARM_REPLAY_HOLD_AFTER_CONVERGENCE_DEFAULT)
        )
        try:
            position_tolerance = float(payload.get("position_tolerance_rad", ARM_REPLAY_TOLERANCE_RAD))
        except (TypeError, ValueError):
            return 400, {"ok": False, "error": "position_tolerance_rad must be a number.", "recording": path.name, "plan": plan}
        if not math.isfinite(position_tolerance) or position_tolerance < 0.005 or position_tolerance > 0.25:
            return 400, {
                "ok": False,
                "error": "position_tolerance_rad must be between 0.005 and 0.25.",
                "recording": path.name,
                "plan": plan,
            }
        tuning = self._arm_replay_tuning(payload)

        xr_suspend = self._suspend_xr_motion_publishers()
        if not xr_suspend.get("ok"):
            return 409, {
                "ok": False,
                "error": "XR teleop motion publisher is still active; arm_sdk replay would be overwritten.",
                "recording": path.name,
                "plan": plan,
                "xr_suspend": xr_suspend,
            }

        raw_gain_by_index = {
            int(item["index"]): (float(item["kp"]), float(item["kd"]))
            for item in plan.get("gain_plan", [])
            if isinstance(item, dict) and "index" in item
        }
        gain_by_index = {
            index: (
                kp * (tuning["inner_kp_scale"] if closed_loop else tuning["direct_kp_scale"]),
                kd * (tuning["inner_kd_scale"] if closed_loop else tuning["direct_kd_scale"]),
            )
            for index, (kp, kd) in raw_gain_by_index.items()
        }
        approach_gain_by_index = {
            index: (kp * tuning["approach_kp_scale"], kd * tuning["approach_kd_scale"])
            for index, (kp, kd) in raw_gain_by_index.items()
        }
        # Stiffer, better-damped gains used during the settle/hold phase so the
        # arm actively resists sagging at the target instead of bobbing.
        hold_gain_by_index = {
            index: (kp * ARM_REPLAY_HOLD_KP_SCALE, kd * ARM_REPLAY_HOLD_KD_SCALE)
            for index, (kp, kd) in raw_gain_by_index.items()
        }
        cancel = threading.Event()
        commanded_body_joints = {
            int(item["index"])
            for item in plan.get("commanded_body_joints", [])
            if isinstance(item, dict) and "index" in item
        }
        if not commanded_body_joints:
            commanded_body_joints = set(ARM_SDK_JOINTS)

        closed_loop_state: dict[int, dict[str, float]] = {
            joint: {"integral": 0.0, "last_error": 0.0, "last_desired_q": math.nan}
            for joint in commanded_body_joints
        }
        approach_frame_count = 0
        if closed_loop:
            first_targets = self._arm_replay_frame_targets(frames[0], commanded_body_joints) if frames else {}
            if first_targets:
                approach_frame_count = max(1, math.ceil(tuning["smooth_approach_seconds"] / TRAJECTORY_DEFAULT_DT))
            execution_frames = self._smooth_arm_replay_frames(
                frames,
                msg,
                commanded_body_joints,
                tuning["smooth_approach_seconds"],
            )
        else:
            execution_frames = frames

        def run_replay() -> None:
            previous_timestamp: float | None = None
            writes = 0
            final_error: dict[str, Any] = {"max_error_rad": None, "per_joint": []}
            converged = False
            fault_reason: str | None = None
            hold_announced = False
            ceiling_announced = False
            consecutive_converged = 0
            escalation = 1.0
            best_error = float("inf")
            settle_cycles = max(1, math.ceil(tuning["settle_seconds"] / TRAJECTORY_DEFAULT_DT))
            per_joint_latched: dict[int, bool] = {joint: False for joint in commanded_body_joints}
            try:
                for frame in execution_frames:
                    if cancel.is_set():
                        break
                    with self.command_lock:
                        latest_msg = self.lowstate_msg
                        latest_publisher = self.wrist_publisher
                    if latest_msg is None or latest_publisher is None:
                        break
                    target_by_index = self._arm_replay_frame_targets(frame, commanded_body_joints)
                    feedforward_tau_by_index: dict[int, float] = {}
                    publish_targets = (
                        self._closed_loop_arm_targets(
                            latest_msg,
                            target_by_index,
                            closed_loop_state,
                            TRAJECTORY_DEFAULT_DT,
                            tuning,
                        )
                        if closed_loop
                        else (target_by_index, {}, {})
                    )
                    target_by_index, _, feedforward_tau_by_index = publish_targets
                    frame_gain_by_index = (
                        approach_gain_by_index
                        if closed_loop and writes < approach_frame_count
                        else gain_by_index
                    )
                    latest_publisher.Write(
                        self._build_arm_sdk_trajectory_cmd(
                            latest_msg,
                            target_by_index,
                            frame_gain_by_index,
                            feedforward_tau_by_index,
                            weight=1.0,
                        )
                    )
                    writes += 1
                    timestamp = numeric(frame.get("timestamp"))
                    if previous_timestamp is not None and timestamp is not None:
                        time.sleep(max(0.0, min(0.1, timestamp - previous_timestamp) / tuning["playback_speed"]))
                    else:
                        time.sleep(TRAJECTORY_DEFAULT_DT / tuning["playback_speed"])
                    if timestamp is not None:
                        previous_timestamp = timestamp
                # Phase B: converge onto the recorded pose and hold. Convergence
                # is the ONLY normal success exit; the loop never terminates at a
                # wrong pose on a timer. It keeps servoing (escalating effort if it
                # stalls) until every joint is settled and the weighted end-effector
                # error is small, or a safety fault / operator cancel occurs.
                phase_b_start = time.monotonic()
                last_progress_t = phase_b_start
                # Run the hold loop faster than playback so the arm is caught
                # before it can drift; keep the settle window the same wall-clock.
                hold_dt = 1.0 / ARM_REPLAY_HOLD_HZ
                settle_cycles = max(1, math.ceil(tuning["settle_seconds"] / hold_dt))
                open_loop_hold_until = phase_b_start + max(
                    TRAJECTORY_APPROACH_SECONDS, plan.get("duration_seconds", 0.0) or 0.0
                )
                while not cancel.is_set():
                    now = time.monotonic()
                    with self.command_lock:
                        latest_msg = self.lowstate_msg
                        latest_publisher = self.wrist_publisher
                    if latest_msg is None or latest_publisher is None or not execution_frames:
                        fault_reason = "telemetry_lost"
                        break
                    final_frame = execution_frames[-1]
                    target_by_index = self._arm_replay_frame_targets(final_frame, commanded_body_joints)
                    if closed_loop:
                        publish_targets, final_error, feedforward_tau_by_index = self._closed_loop_arm_targets(
                            latest_msg,
                            target_by_index,
                            closed_loop_state,
                            hold_dt,
                            tuning,
                            escalation,
                        )
                        # Per-joint settle latch with hysteresis so one noisy joint
                        # cannot repeatedly reset the whole convergence counter.
                        for pj in final_error.get("per_joint", []):
                            joint_index = pj["index"]
                            joint_error = abs(pj.get("error_rad", 0.0))
                            if per_joint_latched.get(joint_index):
                                if joint_error > position_tolerance * ARM_REPLAY_SETTLE_HYSTERESIS:
                                    per_joint_latched[joint_index] = False
                            elif joint_error <= position_tolerance and pj.get("stationary"):
                                per_joint_latched[joint_index] = True
                        all_latched = bool(per_joint_latched) and all(per_joint_latched.values())
                        cartesian_error = final_error.get("cartesian_proxy_error_m")
                        cartesian_ok = not isinstance(cartesian_error, (int, float)) or cartesian_error <= final_error.get(
                            "cartesian_tolerance_m", ARM_REPLAY_CARTESIAN_TOLERANCE_M
                        )
                        velocity_ok = final_error.get("max_velocity_rad_s", 0.0) <= tuning.get(
                            "converge_velocity_rad_s", ARM_REPLAY_CONVERGE_VELOCITY_RAD_S
                        )
                        if all_latched and cartesian_ok and velocity_ok:
                            consecutive_converged += 1
                        else:
                            consecutive_converged = 0
                        converged = consecutive_converged >= settle_cycles
                        # Stall escalation: if the error plateaus above the band,
                        # ramp gravity learning + correction authority (bounded).
                        current_error = final_error.get("max_error_rad")
                        if isinstance(current_error, (int, float)) and current_error < best_error - 1e-4:
                            best_error = current_error
                            last_progress_t = now
                        elif not converged and (now - last_progress_t) > ARM_REPLAY_STALL_SECONDS:
                            escalation = min(ARM_REPLAY_ESCALATION_MAX, escalation + ARM_REPLAY_ESCALATION_STEP)
                            last_progress_t = now
                    else:
                        publish_targets = target_by_index
                        feedforward_tau_by_index = {}
                        converged = now >= open_loop_hold_until
                    latest_publisher.Write(
                        self._build_arm_sdk_trajectory_cmd(
                            latest_msg,
                            publish_targets,
                            hold_gain_by_index,
                            feedforward_tau_by_index,
                            weight=1.0,
                        )
                    )
                    writes += 1
                    if converged:
                        if hold_after_convergence:
                            if not hold_announced:
                                self._set_wrist_status(
                                    active=True,
                                    message="arm_sdk target reached; holding final pose.",
                                    last_command=self._arm_replay_status_payload(
                                        path, plan, tuning, approach_frame_count,
                                        writes=writes, closed_loop=closed_loop,
                                        hold_after_convergence=hold_after_convergence,
                                        position_tolerance=position_tolerance,
                                        final_error=final_error, converged=True,
                                        escalation=escalation, holding=True,
                                    ),
                                )
                                hold_announced = True
                        else:
                            break
                    elif (now - phase_b_start) > ARM_REPLAY_ABSOLUTE_CEILING_SECONDS:
                        # Absolute ceiling reached without convergence: never report
                        # success or release at the wrong pose. Flag it and keep
                        # holding the best pose (or stop, if holding is disabled).
                        if not ceiling_announced:
                            self._set_wrist_status(
                                active=True,
                                message="arm_sdk NOT converged within ceiling; holding best pose (not at target).",
                                last_command=self._arm_replay_status_payload(
                                    path, plan, tuning, approach_frame_count,
                                    writes=writes, closed_loop=closed_loop,
                                    hold_after_convergence=hold_after_convergence,
                                    position_tolerance=position_tolerance,
                                    final_error=final_error, converged=False,
                                    escalation=escalation, holding=True,
                                    ceiling_reached=True,
                                ),
                            )
                            ceiling_announced = True
                        if not hold_after_convergence:
                            fault_reason = "ceiling_not_converged"
                            break
                    time.sleep(hold_dt)
            finally:
                with self.command_lock:
                    if self.replay_cancel is cancel:
                        self.replay_cancel = None
                        self.replay_thread = None
                self._set_wrist_status(
                    active=False,
                    message=(
                        "arm_sdk trajectory replay cancelled."
                        if cancel.is_set()
                        else (
                            f"arm_sdk closed-loop replay converged ({writes} writes)."
                            if closed_loop and converged
                            else (
                                f"arm_sdk replay stopped: {fault_reason} ({writes} writes)."
                                if fault_reason
                                else f"arm_sdk trajectory replay complete ({writes} writes)."
                            )
                        )
                    ),
                    last_command=self._arm_replay_status_payload(
                        path, plan, tuning, approach_frame_count,
                        writes=writes, closed_loop=closed_loop,
                        hold_after_convergence=hold_after_convergence,
                        position_tolerance=position_tolerance,
                        final_error=final_error, converged=converged,
                        escalation=escalation, holding=False,
                        fault_reason=fault_reason,
                    ),
                )

        thread = threading.Thread(target=run_replay, name="arm-sdk-trajectory-replay", daemon=True)
        with self.command_lock:
            self.replay_cancel = cancel
            self.replay_thread = thread
        self._set_wrist_status(
            active=True,
            message="Publishing arm_sdk trajectory replay.",
            last_command={
                "mode": "trajectory",
                "recording": path.name,
                "command_scope": plan.get("command_scope"),
                "control_path": plan.get("control_path"),
                "direct_replay": not closed_loop,
                "hold_after_convergence": hold_after_convergence,
                "approach_frame_count": approach_frame_count,
                "xr_suspend": xr_suspend,
                "closed_loop": {
                    "enabled": closed_loop,
                    "tolerance_rad": position_tolerance,
                    "timeout_seconds": tuning["timeout_seconds"],
                    "settle_seconds": tuning["settle_seconds"],
                    "tuning": tuning,
                },
            },
        )
        thread.start()
        return 202, {
            "ok": True,
            "message": "arm_sdk trajectory replay started.",
            "recording": path.name,
            "plan": plan,
            "xr_suspend": xr_suspend,
            "direct_replay": not closed_loop,
            "hold_after_convergence": hold_after_convergence,
            "approach_frame_count": approach_frame_count,
            "closed_loop": {
                "enabled": closed_loop,
                "tolerance_rad": position_tolerance,
                "timeout_seconds": tuning["timeout_seconds"],
                "settle_seconds": tuning["settle_seconds"],
                "tuning": tuning,
            },
        }

    @staticmethod
    def _response_lerp(response: float, damped: float, balanced: float, responsive: float) -> float:
        if response <= ARM_REPLAY_RESPONSE_DEFAULT:
            ratio = response / ARM_REPLAY_RESPONSE_DEFAULT if ARM_REPLAY_RESPONSE_DEFAULT else 0.0
            return damped + (balanced - damped) * ratio
        ratio = (response - ARM_REPLAY_RESPONSE_DEFAULT) / (ARM_REPLAY_RESPONSE_MAX - ARM_REPLAY_RESPONSE_DEFAULT)
        return balanced + (responsive - balanced) * ratio

    def _arm_replay_tuning(self, payload: dict[str, Any] | None = None) -> dict[str, float]:
        payload = payload or {}
        try:
            response = float(payload.get("replay_response", ARM_REPLAY_RESPONSE_DEFAULT))
        except (TypeError, ValueError):
            response = ARM_REPLAY_RESPONSE_DEFAULT
        if not math.isfinite(response):
            response = ARM_REPLAY_RESPONSE_DEFAULT
        response = max(0.0, min(ARM_REPLAY_RESPONSE_MAX, response))
        tuning = {
            "response": response,
            "inner_kp_scale": self._response_lerp(response, 0.25, ARM_REPLAY_INNER_KP_SCALE, 0.6),
            "inner_kd_scale": self._response_lerp(response, 1.45, ARM_REPLAY_INNER_KD_SCALE, 0.95),
            "direct_kp_scale": self._response_lerp(response, 0.75, 1.0, 1.45),
            "direct_kd_scale": self._response_lerp(response, 1.2, 1.0, 0.9),
            "approach_kp_scale": self._response_lerp(response, 0.6, 0.75, 1.0),
            "approach_kd_scale": self._response_lerp(response, 1.2, 1.1, 1.0),
            "playback_speed": self._response_lerp(response, 0.75, 1.0, 2.0),
            "pid_kp_scale": self._response_lerp(response, 0.75, 1.0, 1.6),
            "pid_ki_scale": self._response_lerp(response, 0.75, 1.0, 1.4),
            "pid_kd_scale": self._response_lerp(response, 1.25, 1.0, 0.85),
            "gravity_tau_filter_seconds": ARM_REPLAY_GRAVITY_TAU_FILTER_SECONDS,
            "lock_tolerance_rad": ARM_REPLAY_LOCK_TOLERANCE_RAD,
            "lock_tolerance_m": ARM_REPLAY_LOCK_TOLERANCE_M,
            "max_pid_correction_rad": self._response_lerp(
                response,
                0.025,
                ARM_REPLAY_MAX_PID_CORRECTION_RAD,
                0.08,
            ),
            "smooth_approach_seconds": self._response_lerp(
                response,
                5.5,
                ARM_REPLAY_SMOOTH_APPROACH_SECONDS,
                1.5,
            ),
            "settle_seconds": self._response_lerp(response, 0.9, ARM_REPLAY_SETTLE_SECONDS, 0.35),
            "timeout_seconds": ARM_REPLAY_TIMEOUT_SECONDS,
            "gravity_hold_scale": ARM_REPLAY_GRAVITY_HOLD_SCALE,
            "gravity_move_scale": ARM_REPLAY_GRAVITY_MOVE_SCALE,
            "converge_velocity_rad_s": ARM_REPLAY_CONVERGE_VELOCITY_RAD_S,
            "cartesian_tolerance_m": ARM_REPLAY_CARTESIAN_TOLERANCE_M,
        }
        return {key: round(value, 6) for key, value in tuning.items()}

    def _smooth_arm_replay_frames(
        self,
        frames: list[dict[str, Any]],
        start_msg: Any,
        commanded_body_joints: set[int],
        smooth_approach_seconds: float = ARM_REPLAY_SMOOTH_APPROACH_SECONDS,
    ) -> list[dict[str, Any]]:
        if not frames:
            return []
        first_targets = self._arm_replay_frame_targets(frames[0], commanded_body_joints)
        if not first_targets:
            return frames
        start_by_index = {
            joint: float(getattr(start_msg.motor_state[joint], "q", 0.0) or 0.0)
            for joint in first_targets
        }
        # Velocity-bound the approach: stretch the ramp so the smootherstep peak
        # joint velocity (1.875x the average) stays under the cap, so the move to
        # the first frame is always smooth however far it is / whatever the dial.
        max_distance = max(
            (abs(target_q - start_by_index[joint]) for joint, target_q in first_targets.items()),
            default=0.0,
        )
        velocity_bounded_seconds = 1.875 * max_distance / ARM_REPLAY_APPROACH_PEAK_VEL_RAD_S
        approach_seconds = max(
            ARM_REPLAY_APPROACH_MIN_SECONDS,
            velocity_bounded_seconds,
            max(0.0, smooth_approach_seconds),
        )
        steps = max(1, math.ceil(approach_seconds / TRAJECTORY_DEFAULT_DT))
        smoothed: list[dict[str, Any]] = []
        for step in range(1, steps + 1):
            progress = step / steps
            eased = progress * progress * (3.0 - 2.0 * progress)
            smoothed.append(
                {
                    "motors": [
                        {
                            "index": joint,
                            "name": JOINT_NAMES.get(joint, f"Motor{joint}"),
                            "q": self._clamp_joint_target(
                                joint,
                                start_by_index[joint] + (target_q - start_by_index[joint]) * eased,
                            ),
                        }
                        for joint, target_q in first_targets.items()
                    ],
                }
            )
        for frame in frames[1:]:
            smoothed.append({"motors": frame.get("motors", [])})
        return smoothed

    def _arm_replay_frame_targets(self, frame: dict[str, Any], commanded_body_joints: set[int]) -> dict[int, float]:
        return {
            int(motor["index"]): self._clamp_joint_target(int(motor["index"]), float(motor.get("q", 0.0)))
            for motor in frame.get("motors", [])
            if (
                isinstance(motor, dict)
                and "index" in motor
                and int(motor["index"]) in ARM_SDK_JOINTS
                and int(motor["index"]) in commanded_body_joints
            )
        }

    def _arm_replay_status_payload(
        self,
        path: Path,
        plan: dict[str, Any],
        tuning: dict[str, float],
        approach_frame_count: int,
        *,
        writes: int,
        closed_loop: bool,
        hold_after_convergence: bool,
        position_tolerance: float,
        final_error: dict[str, Any],
        converged: bool,
        escalation: float = 1.0,
        holding: bool = False,
        ceiling_reached: bool = False,
        fault_reason: str | None = None,
    ) -> dict[str, Any]:
        return {
            "mode": "trajectory",
            "recording": path.name,
            "command_scope": plan.get("command_scope"),
            "control_path": plan.get("control_path"),
            "direct_replay": not closed_loop,
            "hold_after_convergence": hold_after_convergence,
            "holding_final_pose": holding,
            "approach_frame_count": approach_frame_count,
            "writes": writes,
            "replay_response": tuning["response"],
            "tuning": tuning,
            "closed_loop": {
                "enabled": closed_loop,
                "converged": converged,
                "ceiling_reached": ceiling_reached,
                "fault_reason": fault_reason,
                "escalation": round(float(escalation), 3),
                "tolerance_rad": position_tolerance,
                "cartesian_tolerance_m": tuning.get("cartesian_tolerance_m", ARM_REPLAY_CARTESIAN_TOLERANCE_M),
                "settle_seconds": tuning["settle_seconds"],
                "final_error": final_error,
            },
        }

    def _closed_loop_arm_targets(
        self,
        msg: Any,
        desired_by_index: dict[int, float],
        pid_state: dict[int, dict[str, float]],
        dt: float,
        tuning: dict[str, float] | None = None,
        escalation: float = 1.0,
    ) -> tuple[dict[int, float], dict[str, Any], dict[int, float]]:
        tuning = tuning or self._arm_replay_tuning()
        dt = max(0.001, dt)
        escalation = max(1.0, min(ARM_REPLAY_ESCALATION_MAX, float(escalation)))
        corrected: dict[int, float] = {}
        feedforward_tau: dict[int, float] = {}
        per_joint = []
        max_error = 0.0
        max_velocity = 0.0
        locked_count = 0
        stationary_count = 0
        settled_count = 0
        cartesian_sq = 0.0
        lock_tolerance = float(tuning.get("lock_tolerance_rad", ARM_REPLAY_LOCK_TOLERANCE_RAD))
        converge_velocity = float(tuning.get("converge_velocity_rad_s", ARM_REPLAY_CONVERGE_VELOCITY_RAD_S))
        hold_scale = float(tuning.get("gravity_hold_scale", ARM_REPLAY_GRAVITY_HOLD_SCALE))
        move_scale = float(tuning.get("gravity_move_scale", ARM_REPLAY_GRAVITY_MOVE_SCALE))
        learn_gain = ARM_REPLAY_GRAVITY_LEARN_GAIN * escalation
        for joint, desired_q in desired_by_index.items():
            motor_state = msg.motor_state[joint]
            actual_q = float(getattr(motor_state, "q", 0.0) or 0.0)
            actual_dq = float(getattr(motor_state, "dq", 0.0) or 0.0)
            actual_tau = float(getattr(motor_state, "tau_est", getattr(motor_state, "tau", 0.0)) or 0.0)
            error = desired_q - actual_q
            state = pid_state.setdefault(
                joint,
                {"integral": 0.0, "last_error": 0.0, "last_desired_q": math.nan, "gravity_tau": 0.0, "gravity_learn": 0.0},
            )
            last_desired_q = state.get("last_desired_q", math.nan)
            jump = (not math.isfinite(last_desired_q)) or abs(desired_q - last_desired_q) > 0.02
            if jump:
                state["integral"] = 0.0
                state["last_error"] = error
                state["gravity_tau"] = actual_tau
                state["gravity_learn"] = 0.0
            state["integral"] = max(
                -ARM_REPLAY_INTEGRAL_LIMIT,
                min(ARM_REPLAY_INTEGRAL_LIMIT, state.get("integral", 0.0) + error * dt),
            )
            tau_alpha = min(1.0, dt / max(0.001, tuning.get("gravity_tau_filter_seconds", ARM_REPLAY_GRAVITY_TAU_FILTER_SECONDS)))
            state["gravity_tau"] = state.get("gravity_tau", 0.0) + (actual_tau - state.get("gravity_tau", 0.0)) * tau_alpha
            kp, ki, kd = self._arm_replay_pid_gain(joint, tuning)
            derivative = (error - state.get("last_error", error)) / dt
            locked = abs(error) <= lock_tolerance
            stationary = abs(actual_dq) <= converge_velocity
            if locked:
                correction = 0.0
                locked_count += 1
            else:
                correction = kp * error + ki * state["integral"] + kd * derivative
            max_correction = tuning["max_pid_correction_rad"] * escalation
            correction = max(-max_correction, min(max_correction, correction))
            corrected[joint] = self._clamp_joint_target(joint, desired_q + correction)
            # Continuous gravity feed-forward scale, ramped by how STATIONARY the
            # joint is (not by lock state): a joint slowing toward its target gets
            # near-full gravity support, so it settles inside the band instead of a
            # couple of degrees short. No jump at the lock boundary.
            v_ramp = max(1e-6, converge_velocity * ARM_REPLAY_GRAVITY_RAMP_VEL_FACTOR)
            blend = max(0.0, min(1.0, 1.0 - abs(actual_dq) / v_ramp))
            gravity_scale = move_scale + (hold_scale - move_scale) * blend
            learned = state.get("gravity_learn", 0.0)
            base_gravity = state.get("gravity_tau", 0.0) * gravity_scale
            tau_limit = self._arm_replay_gravity_tau_limit(joint)
            gravity_tau = max(-tau_limit, min(tau_limit, base_gravity + learned))
            feedforward_tau[joint] = gravity_tau
            # Learn the residual holding torque while stationary and inside a small
            # band around the target, never on a jump/approach frame -- so it stays
            # exactly 0 during the reach (keeps the tau=0 approach path and the
            # exact-target lock contract intact) but erases the settling residual.
            near_band = abs(error) <= lock_tolerance * ARM_REPLAY_LEARN_BAND_FACTOR
            if near_band and stationary and not jump:
                new_learn = learned + learn_gain * error * dt
                state["gravity_learn"] = max(-ARM_REPLAY_GRAVITY_LEARN_LIMIT, min(ARM_REPLAY_GRAVITY_LEARN_LIMIT, new_learn))
            state["last_error"] = error
            state["last_desired_q"] = desired_q
            abs_error = abs(error)
            max_error = max(max_error, abs_error)
            max_velocity = max(max_velocity, abs(actual_dq))
            settled = locked and stationary
            if stationary:
                stationary_count += 1
            if settled:
                settled_count += 1
            lever = ARM_REPLAY_JOINT_LEVER_M.get(joint, 0.2)
            cartesian_sq += (lever * error) ** 2
            per_joint.append(
                {
                    "index": joint,
                    "name": JOINT_NAMES.get(joint, f"Motor{joint}"),
                    "target_q": round(desired_q, 6),
                    "actual_q": round(actual_q, 6),
                    "actual_dq": round(actual_dq, 6),
                    "actual_tau_est": round(actual_tau, 6),
                    "error_rad": round(error, 6),
                    "locked": locked,
                    "stationary": stationary,
                    "settled": settled,
                    "correction_rad": round(correction, 6),
                    "gravity_tau": round(gravity_tau, 6),
                    "command_q": round(corrected[joint], 6),
                }
            )
        joint_count = len(desired_by_index)
        return corrected, {
            "max_error_rad": round(max_error, 6),
            "max_velocity_rad_s": round(max_velocity, 6),
            "cartesian_proxy_error_m": round(math.sqrt(cartesian_sq), 6),
            "lock_tolerance_rad": round(lock_tolerance, 6),
            "lock_tolerance_m": tuning.get("lock_tolerance_m", ARM_REPLAY_LOCK_TOLERANCE_M),
            "cartesian_tolerance_m": tuning.get("cartesian_tolerance_m", ARM_REPLAY_CARTESIAN_TOLERANCE_M),
            "locked_joints": locked_count,
            "stationary_joints": stationary_count,
            "settled_joints": settled_count,
            "joint_count": joint_count,
            "all_locked": bool(desired_by_index) and locked_count == joint_count,
            "all_settled": bool(desired_by_index) and settled_count == joint_count,
            "escalation": round(escalation, 3),
            "per_joint": per_joint,
        }, feedforward_tau

    def _arm_replay_pid_gain(self, joint: int, tuning: dict[str, float] | None = None) -> tuple[float, float, float]:
        tuning = tuning or self._arm_replay_tuning()
        kp, ki, kd = ARM_REPLAY_PID_GAINS.get(self._joint_gain_group(joint), (0.1, 0.0, 0.004))
        return (
            kp * tuning["pid_kp_scale"],
            ki * tuning["pid_ki_scale"],
            kd * tuning["pid_kd_scale"],
        )

    def _arm_replay_gravity_tau_limit(self, joint: int) -> float:
        return ARM_REPLAY_GRAVITY_TAU_LIMITS.get(self._joint_gain_group(joint), 3.0)

    def _suspend_xr_motion_publishers(self) -> dict[str, Any]:
        if os.environ.get("RTW_SKIP_XR_SUSPEND") == "1":
            return {"ok": True, "skipped": True, "reason": "RTW_SKIP_XR_SUSPEND=1"}
        if shutil.which("systemctl") is None:
            return {"ok": True, "skipped": True, "reason": "systemctl unavailable"}

        actions: list[dict[str, Any]] = []

        def run_step(args: list[str], timeout: float) -> dict[str, Any]:
            try:
                result = subprocess.run(
                    args,
                    capture_output=True,
                    check=False,
                    text=True,
                    timeout=timeout,
                )
                return {
                    "cmd": args,
                    "returncode": result.returncode,
                    "stdout": result.stdout.strip(),
                    "stderr": result.stderr.strip(),
                }
            except subprocess.TimeoutExpired as exc:
                return {
                    "cmd": args,
                    "returncode": None,
                    "timeout": timeout,
                    "stdout": (exc.stdout or "").strip() if isinstance(exc.stdout, str) else "",
                    "stderr": (exc.stderr or "").strip() if isinstance(exc.stderr, str) else "",
                }
            except OSError as exc:
                return {"cmd": args, "returncode": None, "error": str(exc)}

        actions.append(run_step(["systemctl", "--user", "stop", "--no-block", *XR_MOTION_SERVICES], 2.0))
        actions.append(
            run_step(
                ["systemctl", "--user", "kill", "--kill-who=all", "--signal=KILL", *XR_MOTION_SERVICES],
                3.0,
            )
        )
        actions.append(run_step(["pkill", "-f", XR_TELEOP_PROCESS_PATTERN], 2.0))
        process_check = run_step(["pgrep", "-af", XR_TELEOP_PROCESS_PATTERN], 2.0)
        actions.append(process_check)

        remaining_processes = []
        if process_check.get("returncode") == 0:
            remaining_processes = [
                line
                for line in str(process_check.get("stdout", "")).splitlines()
                if XR_TELEOP_PROCESS_PATTERN in line
            ]
        return {
            "ok": not remaining_processes,
            "services": list(XR_MOTION_SERVICES),
            "remaining_processes": remaining_processes,
            "actions": actions,
        }

    @staticmethod
    def _clamp_joint_target(joint: int, q: float) -> float:
        low_high = JOINT_LIMITS.get(joint)
        if low_high is None:
            return q
        low, high = low_high
        return max(low, min(high, q))

    def plan_replay_control_path(self, path: Path, command_scope: str = "all") -> dict[str, Any]:
        if command_scope not in REPLAY_COMMAND_SCOPES:
            raise ValueError(f"command_scope must be one of {', '.join(sorted(REPLAY_COMMAND_SCOPES))}")
        scoped_joints = set(REPLAY_COMMAND_SCOPES[command_scope])
        frames = self._recording_trajectory_frames(path)
        current_q = self._current_body_q()
        current_lower = {joint: current_q[joint] for joint in LOWER_BODY_JOINTS}
        max_lower_delta = 0.0
        max_frame_delta = 0.0
        max_velocity = 0.0
        moving_lower_joints: set[int] = set()
        moving_joints: set[int] = set()
        violations: list[dict[str, Any]] = []
        joint_stats: dict[int, dict[str, float]] = {
            joint: {"max_step": 0.0, "max_velocity": 0.0}
            for joint in JOINT_NAMES
        }
        previous_q = dict(current_q)
        previous_timestamp: float | None = None

        for frame_index, frame in enumerate(frames):
            by_index = {
                int(motor["index"]): float(motor.get("q", 0.0))
                for motor in frame.get("motors", [])
                if isinstance(motor, dict) and "index" in motor
            }
            for joint in LOWER_BODY_JOINTS:
                if joint not in scoped_joints:
                    continue
                delta = abs(by_index.get(joint, current_lower[joint]) - current_lower[joint])
                max_lower_delta = max(max_lower_delta, delta)
                if delta > TRAJECTORY_ROUTE_EPSILON:
                    moving_lower_joints.add(joint)
            timestamp = frame.get("timestamp")
            dt = (
                max(TRAJECTORY_DEFAULT_DT, float(timestamp) - previous_timestamp)
                if isinstance(timestamp, (int, float)) and previous_timestamp is not None
                else TRAJECTORY_DEFAULT_DT
            )
            if isinstance(timestamp, (int, float)):
                previous_timestamp = float(timestamp)

            for joint, target_q in by_index.items():
                if joint not in JOINT_NAMES or joint not in scoped_joints:
                    continue
                delta = abs(target_q - previous_q.get(joint, 0.0))
                total_delta = abs(target_q - current_q.get(joint, 0.0))
                if delta > TRAJECTORY_ROUTE_EPSILON or total_delta > TRAJECTORY_ROUTE_EPSILON:
                    moving_joints.add(joint)
                if frame_index > 0:
                    velocity = delta / dt
                    max_frame_delta = max(max_frame_delta, delta)
                    max_velocity = max(max_velocity, velocity)
                    joint_stats[joint]["max_step"] = max(joint_stats[joint]["max_step"], delta)
                    joint_stats[joint]["max_velocity"] = max(joint_stats[joint]["max_velocity"], velocity)
                    if delta > TRAJECTORY_MAX_FRAME_DELTA_RAD:
                        self._append_trajectory_violation(
                            violations,
                            "frame_delta",
                            frame_index,
                            joint,
                            delta,
                            TRAJECTORY_MAX_FRAME_DELTA_RAD,
                        )
                    if velocity > TRAJECTORY_MAX_VELOCITY_RAD_S:
                        self._append_trajectory_violation(
                            violations,
                            "velocity",
                            frame_index,
                            joint,
                            velocity,
                            TRAJECTORY_MAX_VELOCITY_RAD_S,
                        )
                previous_q[joint] = target_q

        control_path = "lowcmd" if moving_lower_joints else "arm_sdk"
        duration = self._trajectory_duration(frames)
        hand_plan = self._plan_hand_trajectory(frames) if command_scope == "all" else self._disabled_hand_plan(
            "Hand/finger targets are ignored for scoped arm replay."
        )
        all_violations = [*violations, *hand_plan["violations"]]
        return {
            "command_scope": command_scope,
            "commanded_body_joints": [
                {"index": joint, "name": JOINT_NAMES[joint]} for joint in sorted(scoped_joints) if joint in JOINT_NAMES
            ],
            "control_path": control_path,
            "reason": (
                "lower body joints move during the planned trajectory"
                if control_path == "lowcmd"
                else "lower body joints stay within the stationary threshold"
            ),
            "frame_count": len(frames),
            "duration_seconds": round(duration, 3),
            "valid_for_execution": bool(frames) and not all_violations,
            "lower_body_threshold_rad": TRAJECTORY_ROUTE_EPSILON,
            "max_lower_body_delta_rad": round(max_lower_delta, 6),
            "max_frame_delta_rad": round(max_frame_delta, 6),
            "max_velocity_rad_s": round(max_velocity, 6),
            "moving_lower_body_joints": [
                {"index": joint, "name": JOINT_NAMES[joint]} for joint in sorted(moving_lower_joints)
            ],
            "moving_joints": [
                {"index": joint, "name": JOINT_NAMES[joint]} for joint in sorted(moving_joints)
            ],
            "gain_plan": self._select_trajectory_gains(control_path, moving_joints, joint_stats),
            "hand_plan": hand_plan,
            "limits": {
                "max_frame_delta_rad": TRAJECTORY_MAX_FRAME_DELTA_RAD,
                "max_velocity_rad_s": TRAJECTORY_MAX_VELOCITY_RAD_S,
                "hand_max_frame_delta": HAND_TRAJECTORY_MAX_FRAME_DELTA,
                "hand_max_velocity": HAND_TRAJECTORY_MAX_VELOCITY,
            },
            "violations": all_violations,
        }

    @staticmethod
    def _disabled_hand_plan(note: str) -> dict[str, Any]:
        return {
            "enabled": False,
            "state_topic": HAND_STATE_TOPIC,
            "command_topic": HAND_COMMAND_TOPIC,
            "command_type": "unitree_go.msg.dds_.MotorCmds_",
            "joint_count": len(HAND_JOINT_NAMES),
            "frame_count": 0,
            "stationary_threshold": HAND_TRAJECTORY_EPSILON,
            "max_frame_delta": 0.0,
            "max_velocity": 0.0,
            "valid_for_execution": True,
            "moving_hand_joints": [],
            "execution_note": note,
            "violations": [],
        }

    def _plan_hand_trajectory(self, frames: list[dict[str, Any]]) -> dict[str, Any]:
        current_q = self._current_hand_q()
        previous_q = dict(current_q)
        previous_timestamp: float | None = None
        moving_joints: set[int] = set()
        max_frame_delta = 0.0
        max_velocity = 0.0
        violations: list[dict[str, Any]] = []
        hand_frame_count = 0

        for frame_index, frame in enumerate(frames):
            hand_joints = frame.get("hands") or []
            by_index = {
                int(joint["index"]): float(joint.get("q", 0.0))
                for joint in hand_joints
                if isinstance(joint, dict) and "index" in joint
            }
            if not by_index:
                continue
            hand_frame_count += 1
            timestamp = frame.get("timestamp")
            dt = (
                max(TRAJECTORY_DEFAULT_DT, float(timestamp) - previous_timestamp)
                if isinstance(timestamp, (int, float)) and previous_timestamp is not None
                else TRAJECTORY_DEFAULT_DT
            )
            if isinstance(timestamp, (int, float)):
                previous_timestamp = float(timestamp)

            for joint, target_q in by_index.items():
                if joint not in HAND_JOINT_NAMES:
                    continue
                delta = abs(target_q - previous_q.get(joint, 0.0))
                total_delta = abs(target_q - current_q.get(joint, 0.0))
                if delta > HAND_TRAJECTORY_EPSILON or total_delta > HAND_TRAJECTORY_EPSILON:
                    moving_joints.add(joint)
                if hand_frame_count > 1:
                    velocity = delta / dt
                    max_frame_delta = max(max_frame_delta, delta)
                    max_velocity = max(max_velocity, velocity)
                    if delta > HAND_TRAJECTORY_MAX_FRAME_DELTA:
                        self._append_trajectory_violation(
                            violations,
                            "hand_frame_delta",
                            frame_index,
                            joint,
                            delta,
                            HAND_TRAJECTORY_MAX_FRAME_DELTA,
                            HAND_JOINT_NAMES,
                        )
                    if velocity > HAND_TRAJECTORY_MAX_VELOCITY:
                        self._append_trajectory_violation(
                            violations,
                            "hand_velocity",
                            frame_index,
                            joint,
                            velocity,
                            HAND_TRAJECTORY_MAX_VELOCITY,
                            HAND_JOINT_NAMES,
                        )
                previous_q[joint] = target_q

        return {
            "enabled": bool(moving_joints),
            "state_topic": HAND_STATE_TOPIC,
            "command_topic": HAND_COMMAND_TOPIC,
            "command_type": "unitree_go.msg.dds_.MotorCmds_",
            "joint_count": len(HAND_JOINT_NAMES),
            "frame_count": hand_frame_count,
            "stationary_threshold": HAND_TRAJECTORY_EPSILON,
            "max_frame_delta": round(max_frame_delta, 6),
            "max_velocity": round(max_velocity, 6),
            "valid_for_execution": hand_frame_count == 0 or not violations,
            "moving_hand_joints": [
                {"index": joint, "name": HAND_JOINT_NAMES[joint]} for joint in sorted(moving_joints)
            ],
            "execution_note": (
                "Finger targets should be published in parallel on rt/inspire/cmd while the body route "
                "continues through arm_sdk or lowcmd. Physical publish is still locked."
            ),
            "violations": violations,
        }

    def _select_trajectory_gains(
        self,
        control_path: str,
        moving_joints: set[int],
        joint_stats: dict[int, dict[str, float]],
    ) -> list[dict[str, Any]]:
        if control_path == "arm_sdk":
            commanded_joints = [joint for joint in ARM_SDK_JOINTS if joint in JOINT_NAMES]
        else:
            commanded_joints = list(JOINT_NAMES)

        plan = []
        for joint in commanded_joints:
            group = self._joint_gain_group(joint)
            base_kp, base_kd = self._base_gain(joint, control_path)
            stats = joint_stats.get(joint, {"max_step": 0.0, "max_velocity": 0.0})
            nominal = GAIN_NOMINALS[group]
            step_ratio = stats["max_step"] / nominal["step"] if nominal["step"] else 0.0
            velocity_ratio = stats["max_velocity"] / nominal["velocity"] if nominal["velocity"] else 0.0
            demand_score = max(step_ratio, velocity_ratio)
            scale = max(0.75, min(1.15, 0.75 + 0.25 * demand_score))
            if joint not in moving_joints:
                scale = 0.75
            kp = base_kp * scale
            kd = base_kd * math.sqrt(scale)
            plan.append(
                {
                    "index": joint,
                    "name": JOINT_NAMES[joint],
                    "group": group,
                    "moving": joint in moving_joints,
                    "base_kp": base_kp,
                    "base_kd": base_kd,
                    "scale": round(scale, 4),
                    "kp": round(kp, 4),
                    "kd": round(kd, 4),
                    "demand_score": round(demand_score, 4),
                }
            )
        return plan

    def _base_gain(self, joint: int, control_path: str) -> tuple[float, float]:
        if control_path == "arm_sdk" and joint in ARM_SDK_JOINTS:
            arm_index = ARM_SDK_JOINTS.index(joint)
            return float(ARM_SDK_KP[arm_index]), float(ARM_SDK_KD[arm_index])
        return LOWCMD_BASE_GAINS[self._joint_gain_group(joint)]

    @staticmethod
    def _joint_gain_group(joint: int) -> str:
        if joint in {0, 1, 2, 6, 7, 8}:
            return "hip"
        if joint in {3, 9}:
            return "knee"
        if joint in {4, 5, 10, 11}:
            return "ankle"
        if joint == 12:
            return "waist"
        if joint in {13, 14, 15, 20, 21, 22}:
            return "shoulder"
        if joint in {16, 23}:
            return "elbow"
        return "wrist"

    def _append_trajectory_violation(
        self,
        violations: list[dict[str, Any]],
        kind: str,
        frame_index: int,
        joint: int,
        value: float,
        limit: float,
        names: dict[int, str] | None = None,
    ) -> None:
        if len(violations) >= TRAJECTORY_MAX_REPORTED_VIOLATIONS:
            return
        joint_names = names or JOINT_NAMES
        violations.append(
            {
                "kind": kind,
                "frame": frame_index,
                "joint": {"index": joint, "name": joint_names.get(joint, f"Motor{joint}")},
                "value": round(value, 6),
                "limit": limit,
            }
        )

    def _trajectory_duration(self, frames: list[dict[str, Any]]) -> float:
        if not frames:
            return 0.0
        timestamps = [frame.get("timestamp") for frame in frames if isinstance(frame.get("timestamp"), (int, float))]
        if len(timestamps) >= 2:
            return TRAJECTORY_APPROACH_SECONDS + max(0.0, float(timestamps[-1]) - float(timestamps[0]))
        return TRAJECTORY_APPROACH_SECONDS + max(0.0, (len(frames) - 1) * TRAJECTORY_DEFAULT_DT)

    def _current_body_q(self) -> dict[int, float]:
        with self.command_lock:
            msg = self.lowstate_msg
        if msg is None:
            return {joint: 0.0 for joint in JOINT_NAMES}
        return {
            joint: float(getattr(msg.motor_state[joint], "q", 0.0) or 0.0)
            for joint in JOINT_NAMES
        }

    def _current_hand_q(self) -> dict[int, float]:
        with self.lock:
            hands = dict(self.latest.get("hands") or {})
        joints = hands.get("joints") or []
        current = {joint: 0.0 for joint in HAND_JOINT_NAMES}
        for joint in joints:
            if not isinstance(joint, dict):
                continue
            index = joint.get("index")
            q = numeric(joint.get("q"))
            if isinstance(index, int) and index in HAND_JOINT_NAMES and q is not None:
                current[index] = q
        return current

    def _recording_trajectory_frames(self, path: Path) -> list[dict[str, Any]]:
        raw_frames: list[dict[str, Any]] = []
        for record in self._recording_point_records(path):
            frame = self._recording_point_frame(record)
            if frame is not None:
                raw_frames.append(frame)
        return self._adaptive_trajectory_frames(raw_frames)

    def _recording_point_records(self, path: Path) -> list[dict[str, Any]]:
        if path.name.endswith(".jsonl"):
            records = []
            with path.open(encoding="utf-8") as source:
                for line in source:
                    line = line.strip()
                    if not line:
                        continue
                    record = json.loads(line)
                    if record.get("type") == "telemetry_sample":
                        records.append(record)
            return records

        data = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(data, list):
            return [record for record in data if isinstance(record, dict)]
        if not isinstance(data, dict):
            return []
        for key in ("points", "frames", "snapshots", "trajectory"):
            records = data.get(key)
            if isinstance(records, list):
                return [record for record in records if isinstance(record, dict)]
        if isinstance(data.get("snapshot"), dict):
            return [data["snapshot"]]
        return [data]

    def _recording_point_frame(self, record: dict[str, Any]) -> dict[str, Any] | None:
        snapshot = record.get("snapshot") if isinstance(record.get("snapshot"), dict) else record
        motors = snapshot.get("motors") or (snapshot.get("body") or {}).get("motors") or []
        hands = (snapshot.get("hands") or {}).get("joints") or snapshot.get("hand_joints") or []
        if not (motors or hands):
            return None
        return {
            "timestamp": snapshot.get("timestamp") or record.get("timestamp"),
            "motors": motors,
            "hands": hands,
        }

    def _adaptive_trajectory_frames(self, frames: list[dict[str, Any]]) -> list[dict[str, Any]]:
        if len(frames) <= 1:
            return frames

        result = [frames[0]]
        previous = frames[0]
        previous_timestamp = numeric(previous.get("timestamp")) or 0.0
        for target in frames[1:]:
            duration = self._recording_segment_duration(previous, target)
            max_delta = self._max_recording_frame_delta(previous, target)
            dense_enough = duration <= TRAJECTORY_DENSE_MAX_DT and max_delta <= TRAJECTORY_MAX_INTERPOLATED_STEP_RAD
            steps = (
                1
                if dense_enough
                else max(
                    2,
                    math.ceil(duration * TRAJECTORY_ADAPTIVE_SAMPLE_HZ),
                    math.ceil(max_delta / TRAJECTORY_MAX_INTERPOLATED_STEP_RAD),
                )
            )
            for step in range(1, steps + 1):
                t = step / steps
                timestamp = previous_timestamp + duration * t
                result.append(
                    {**target, "timestamp": timestamp}
                    if step == steps
                    else self._interpolate_recording_frame(previous, target, t, timestamp)
                )
            previous = target
            previous_timestamp += duration
        return result

    def _recording_segment_duration(self, start: dict[str, Any], target: dict[str, Any]) -> float:
        start_time = numeric(start.get("timestamp"))
        target_time = numeric(target.get("timestamp"))
        if start_time is not None and target_time is not None and target_time > start_time:
            return target_time - start_time
        return TRAJECTORY_DEFAULT_DT

    def _max_recording_frame_delta(self, start: dict[str, Any], target: dict[str, Any]) -> float:
        return max(
            self._max_recording_joint_delta(start.get("motors") or [], target.get("motors") or []),
            self._max_recording_joint_delta(start.get("hands") or [], target.get("hands") or []),
        )

    @staticmethod
    def _recording_joint_key(joint: dict[str, Any]) -> Any:
        return joint.get("index", joint.get("name", ""))

    def _max_recording_joint_delta(self, start_joints: list[dict[str, Any]], target_joints: list[dict[str, Any]]) -> float:
        start_by_key = {self._recording_joint_key(joint): joint for joint in start_joints if isinstance(joint, dict)}
        max_delta = 0.0
        for target in target_joints:
            if not isinstance(target, dict):
                continue
            start = start_by_key.get(self._recording_joint_key(target))
            start_q = numeric((start or {}).get("q"))
            target_q = numeric(target.get("q"))
            if start_q is not None and target_q is not None:
                max_delta = max(max_delta, abs(target_q - start_q))
        return max_delta

    def _interpolate_recording_frame(
        self,
        start: dict[str, Any],
        target: dict[str, Any],
        t: float,
        timestamp: float,
    ) -> dict[str, Any]:
        return {
            **target,
            "timestamp": timestamp,
            "motors": self._interpolate_recording_joints(start.get("motors") or [], target.get("motors") or [], t),
            "hands": self._interpolate_recording_joints(start.get("hands") or [], target.get("hands") or [], t),
        }

    def _interpolate_recording_joints(
        self,
        start_joints: list[dict[str, Any]],
        target_joints: list[dict[str, Any]],
        t: float,
    ) -> list[dict[str, Any]]:
        start_by_key = {self._recording_joint_key(joint): joint for joint in start_joints if isinstance(joint, dict)}
        interpolated = []
        for target in target_joints:
            if not isinstance(target, dict):
                continue
            start = start_by_key.get(self._recording_joint_key(target), target)
            item = dict(target)
            for field in ("q", "dq", "tau_est"):
                start_value = numeric(start.get(field))
                target_value = numeric(target.get(field))
                if start_value is not None and target_value is not None:
                    item[field] = start_value + (target_value - start_value) * t
            interpolated.append(item)
        return interpolated

    def record_command_event(self, name: str, payload: dict[str, Any]) -> None:
        self.recorder.write_event(name, payload)

    def wrist_snapshot(self) -> dict[str, Any]:
        snapshot = self.snapshot()
        motors = snapshot.get("motors") or []
        wrist = next((motor for motor in motors if motor.get("index") == RIGHT_WRIST_YAW), None)
        with self.command_lock:
            status = dict(self.wrist_status)
        return {
            **status,
            "joint": {
                "index": RIGHT_WRIST_YAW,
                "name": JOINT_NAMES[RIGHT_WRIST_YAW],
                "limits": {"min": WRIST_LIMITS[0], "max": WRIST_LIMITS[1]},
                "telemetry": wrist,
            },
        }

    def _set_wrist_status(self, **updates: Any) -> None:
        with self.command_lock:
            self.wrist_status = {**self.wrist_status, **updates, "updated_at": time.time()}

    def torso_snapshot(self) -> dict[str, Any]:
        snapshot = self.snapshot()
        motors = snapshot.get("motors") or []
        torso = next((motor for motor in motors if motor.get("index") == TORSO_YAW_JOINT), None)
        with self.command_lock:
            status = dict(self.torso_status)
            available = self.wrist_publisher is not None
        return {
            **status,
            "available": available,
            "joint": {
                "index": TORSO_YAW_JOINT,
                "name": JOINT_NAMES[TORSO_YAW_JOINT],
                "limits": {"min": TORSO_LIMITS[0], "max": TORSO_LIMITS[1]},
                "telemetry": torso,
            },
        }

    def _set_torso_status(self, **updates: Any) -> None:
        with self.command_lock:
            self.torso_status = {**self.torso_status, **updates, "updated_at": time.time()}

    def command_torso_twist(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        if not has_risk_ack(payload):
            return 400, {"ok": False, "error": "Command requires armed=true and i_understand_risk=true."}
        try:
            raw_target = float(payload.get("target_q", 0.0))
        except (TypeError, ValueError):
            return 400, {"ok": False, "error": "target_q must be a number."}
        if not math.isfinite(raw_target):
            return 400, {"ok": False, "error": "target_q must be a finite number."}
        target_q = max(TORSO_LIMITS[0], min(TORSO_LIMITS[1], raw_target))

        with self.command_lock:
            publisher = self.wrist_publisher
            msg = self.lowstate_msg
            running = self.torso_thread is not None and self.torso_thread.is_alive()
            self.torso_target_q = target_q
            wrist_cancel = self.wrist_cancel
            replay_cancel = self.replay_cancel
        if publisher is None:
            return 503, {"ok": False, "error": "DDS arm_sdk publisher is not available."}
        if msg is None:
            return 503, {"ok": False, "error": "No rt/lowstate sample is available yet."}
        if self.lowcmd_factory is None or self.crc is None:
            return 503, {"ok": False, "error": "DDS command factory is not available."}

        # Live update: a running twist just adopts the new shared target.
        if running:
            self._set_torso_status(
                active=True,
                message="Torso twisting.",
                target_q=round(target_q, 6),
                last_command={"target_q": round(target_q, 6)},
            )
            return 202, {"ok": True, "status": self.torso_snapshot()}

        # Start a fresh persistent twist: make sure no other arm_sdk writer runs.
        if wrist_cancel is not None:
            wrist_cancel.set()
        if replay_cancel is not None:
            replay_cancel.set()

        cancel = threading.Event()
        arm_hold = {
            joint: float(msg.motor_state[joint].q)
            for joint in ARM_SDK_JOINTS
            if joint != TORSO_YAW_JOINT
        }
        gains = {TORSO_YAW_JOINT: (TORSO_TWIST_KP, TORSO_TWIST_KD)}

        def run_torso() -> None:
            dt = 1.0 / TORSO_TWIST_RATE_HZ
            max_step = TORSO_TWIST_MAX_VEL_RAD_S * dt
            commanded = float(msg.motor_state[TORSO_YAW_JOINT].q)
            writes = 0
            self._set_torso_status(
                available=True, active=True, message="Torso twisting.",
                target_q=round(self.torso_target_q, 6),
                last_command={"target_q": round(self.torso_target_q, 6)},
            )
            try:
                while not cancel.is_set():
                    with self.command_lock:
                        latest_msg = self.lowstate_msg
                        latest_publisher = self.wrist_publisher
                        target = max(TORSO_LIMITS[0], min(TORSO_LIMITS[1], self.torso_target_q))
                    if latest_msg is None or latest_publisher is None:
                        break
                    if commanded < target:
                        commanded = min(target, commanded + max_step)
                    elif commanded > target:
                        commanded = max(target, commanded - max_step)
                    targets = {**arm_hold, TORSO_YAW_JOINT: commanded}
                    latest_publisher.Write(
                        self._build_arm_sdk_trajectory_cmd(latest_msg, targets, gains, {}, weight=1.0)
                    )
                    writes += 1
                    time.sleep(dt)
            finally:
                with self.command_lock:
                    if self.torso_cancel is cancel:
                        self.torso_cancel = None
                        self.torso_thread = None
                self._set_torso_status(
                    active=False,
                    message="Torso twist stopped." if cancel.is_set() else "Torso twist ended.",
                    last_command={"target_q": round(self.torso_target_q, 6), "writes": writes},
                )

        thread = threading.Thread(target=run_torso, name="torso-twist", daemon=True)
        with self.command_lock:
            self.torso_cancel = cancel
            self.torso_thread = thread
        thread.start()
        return 202, {"ok": True, "status": self.torso_snapshot()}

    def stop_torso(self) -> dict[str, Any]:
        with self.command_lock:
            cancel = self.torso_cancel
            publisher = self.wrist_publisher
            msg = self.lowstate_msg
        if cancel is not None:
            cancel.set()
        if publisher is not None and msg is not None:
            with contextlib.suppress(Exception):
                release = self._build_arm_sdk_cmd(msg, 0.0, 0.0, 0.0, weight=0.0)
                for _ in range(10):
                    publisher.Write(release)
                    time.sleep(0.01)
        self._set_torso_status(active=False, message="Torso released to onboard controller.")
        return self.torso_snapshot()

    def _append_loco_history(self, command: dict[str, Any]) -> None:
        history = [command, *list(self.loco_status.get("history") or [])]
        self.loco_status = {**self.loco_status, "history": history[:12]}

    def _set_loco_status(self, **updates: Any) -> None:
        with self.command_lock:
            self.loco_status = {**self.loco_status, **updates, "updated_at": time.time()}

    def _loco_status_payload(
        self, status: dict[str, Any], robot: dict[str, Any], available: bool, include_metadata: bool = True
    ) -> dict[str, Any]:
        motion_mode = status.get("motion_mode")
        check_code = None
        last = status.get("last_command") or {}
        if "motion_check_code" in last:
            check_code = last.get("motion_check_code")

        payload = {
            **status,
            "available": available,
            "motion_mode": motion_mode,
            "motion_check_code": check_code,
            "robot": {
                "mode_pr": robot.get("mode_pr"),
                "mode_machine": robot.get("mode_machine"),
                "tick": robot.get("tick"),
            },
        }
        if include_metadata:
            payload["limits"] = LOCO_LIMITS
            payload["actions"] = LOCO_ACTIONS
        return payload

    def loco_snapshot(self) -> dict[str, Any]:
        with self.lock:
            robot = dict(self.latest.get("robot") or {})
        with self.command_lock:
            status = dict(self.loco_status)
            loco_available = bool(self.loco_client)
        return self._loco_status_payload(status, robot, loco_available)

    def _build_arm_sdk_cmd(self, msg: Any, target_q: float, kp: float, kd: float, weight: float = 1.0) -> Any:
        if self.lowcmd_factory is None or self.crc is None:
            raise RuntimeError("LowCmd factory is not initialized")
        cmd = self.lowcmd_factory()
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

        cmd.motor_cmd[ARM_SDK_WEIGHT_SLOT].q = float(weight)
        if weight > 0:
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

        cmd.crc = self.crc.Crc(cmd)
        return cmd

    def _build_arm_sdk_trajectory_cmd(
        self,
        msg: Any,
        target_by_index: dict[int, float],
        gain_by_index: dict[int, tuple[float, float]],
        feedforward_tau_by_index: dict[int, float] | None = None,
        weight: float = 1.0,
    ) -> Any:
        if self.lowcmd_factory is None or self.crc is None:
            raise RuntimeError("LowCmd factory is not initialized")
        cmd = self.lowcmd_factory()
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

        cmd.motor_cmd[ARM_SDK_WEIGHT_SLOT].q = float(weight)
        if weight > 0:
            feedforward_tau_by_index = feedforward_tau_by_index or {}
            for joint, fallback_kp, fallback_kd in zip(ARM_SDK_JOINTS, ARM_SDK_KP, ARM_SDK_KD):
                motor = cmd.motor_cmd[joint]
                kp, kd = gain_by_index.get(joint, (float(fallback_kp), float(fallback_kd)))
                motor.mode = 1
                motor.q = float(target_by_index.get(joint, getattr(msg.motor_state[joint], "q", 0.0) or 0.0))
                motor.dq = 0.0
                motor.tau = float(feedforward_tau_by_index.get(joint, 0.0))
                motor.kp = kp
                motor.kd = kd

        cmd.crc = self.crc.Crc(cmd)
        return cmd

    def _build_lowcmd_wrist_cmd(self, msg: Any, hold_q: list[float], wrist_q: float, kp: float, kd: float) -> Any:
        if self.lowcmd_factory is None or self.crc is None:
            raise RuntimeError("LowCmd factory is not initialized")
        cmd = self.lowcmd_factory()
        cmd.mode_pr = 0
        cmd.mode_machine = int(getattr(msg, "mode_machine", 0) or 0)
        for i in range(27):
            motor = cmd.motor_cmd[i]
            motor.mode = 1
            motor.q = wrist_q if i == RIGHT_WRIST_YAW else hold_q[i]
            motor.dq = 0.0
            motor.tau = 0.0
            if i == RIGHT_WRIST_YAW:
                motor.kp = kp
                motor.kd = kd
            elif i < 13:
                motor.kp = 70.0
                motor.kd = 1.0
            else:
                motor.kp = 25.0
                motor.kd = 0.8
            motor.reserve = 0
        cmd.crc = self.crc.Crc(cmd)
        return cmd

    @staticmethod
    def _auto_wrist_gains(mode: str, start_q: float, target_q: float, delta: float, period: float) -> tuple[float, float]:
        if mode == "oscillate":
            amplitude = abs(delta)
            max_target_speed = (2.0 * math.pi * amplitude) / max(0.4, period)
            kp = 6.0 + 80.0 * amplitude + 3.0 * max_target_speed
            kd = 0.4 + 0.08 * math.sqrt(kp) + 0.8 * max_target_speed
        else:
            error = abs(delta) if mode == "relative" else abs(target_q - start_q)
            x = max(0.0, min(1.0, error / 0.2))
            kp = 4.0 + (18.0 - 4.0) * x
            kd = 0.28 * 2.0 * math.sqrt(kp)
        return max(4.0, min(22.0, kp)), max(0.35, min(2.0, kd))

    def stop_wrist(self) -> dict[str, Any]:
        with self.command_lock:
            cancel = self.wrist_cancel
            replay_cancel = self.replay_cancel
            torso_cancel = self.torso_cancel
            publisher = self.wrist_publisher
            msg = self.lowstate_msg
        if cancel is not None:
            cancel.set()
        if replay_cancel is not None:
            replay_cancel.set()
        if torso_cancel is not None:
            torso_cancel.set()
        if publisher is not None and msg is not None:
            try:
                current_q = float(msg.motor_state[RIGHT_WRIST_YAW].q)
                release = self._build_arm_sdk_cmd(msg, current_q, 0.0, 0.0, weight=0.0)
                for _ in range(10):
                    publisher.Write(release)
                    time.sleep(0.01)
            except Exception as exc:
                self._set_wrist_status(active=False, message=f"Stop publish failed: {exc}")
                return self.wrist_snapshot()
        self._set_wrist_status(active=False, message="Right wrist command stopped.")
        return self.wrist_snapshot()

    def chill_motors(self) -> tuple[int, dict[str, Any]]:
        return self.request_chill({"armed": True, "i_understand_risk": True})

    def request_home(self) -> tuple[int, dict[str, Any]]:
        return self._request_xr_ipc("CMD_STOP", "XR teleop stop requested. Arms should move home during clean shutdown.")

    def request_straight(self) -> tuple[int, dict[str, Any]]:
        return self._request_xr_ipc("CMD_STRAIGHT", "Straight arm hold requested. XR arm tracking is paused.")

    def _request_xr_ipc(self, command: str, success_message: str) -> tuple[int, dict[str, Any]]:
        script = """
import time
import os
from teleop.utils.ipc import IPC_Client

command = os.environ["RTW_XR_IPC_COMMAND"]
client = IPC_Client(hb_fps=10.0)
try:
    for _ in range(40):
        if client.is_online():
            break
        time.sleep(0.1)
    reply = client.send_data(command)
    print(reply)
    raise SystemExit(0 if reply.get("status") == "ok" else 2)
finally:
    client.stop()
"""
        env = os.environ.copy()
        python_paths = [str(path) for path in XR_TELEOP_PATHS if path.exists()]
        if env.get("PYTHONPATH"):
            python_paths.append(env["PYTHONPATH"])
        env["PYTHONPATH"] = os.pathsep.join(python_paths)
        env["RTW_XR_IPC_COMMAND"] = command
        try:
            result = subprocess.run(
                [sys.executable, "-c", script],
                capture_output=True,
                check=False,
                cwd=str(APP_DIR),
                env=env,
                text=True,
                timeout=8.0,
            )
        except subprocess.TimeoutExpired:
            return 504, {"ok": False, "error": "Timed out while sending XR home command."}
        except OSError as exc:
            return 500, {"ok": False, "error": f"Could not send XR home command: {exc}"}

        output = (result.stdout or result.stderr).strip()
        if result.returncode != 0:
            return 502, {"ok": False, "error": output or "XR home command was rejected."}
        return 202, {"ok": True, "message": success_message, "reply": output}

    def request_chill(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        with self.command_lock:
            cancel = self.wrist_cancel
            motion_switcher = self.motion_switcher
            loco_client = self.loco_client
        if cancel is not None:
            cancel.set()

        wrist_status = self.stop_wrist()
        select_code = None
        stop_code = None
        damp_code = None
        try:
            if motion_switcher is not None:
                select_code, _ = motion_switcher.SelectMode("ai")
                time.sleep(0.15)
            if loco_client is None:
                return 503, {"ok": False, "error": "H1 loco client is not available.", "wrist": wrist_status}
            stop_code = loco_client.SetVelocity(0.0, 0.0, 0.0, 0.2)
            damp_code = loco_client.SetFsmId(1)
        except Exception as exc:
            return 500, {
                "ok": False,
                "error": f"Could not request damp mode: {exc}",
                "select_mode_code": select_code,
                "stop_move_code": stop_code,
                "damp_code": damp_code,
                "wrist": wrist_status,
            }

        ok = damp_code == 0
        message = "Damp mode requested. Motors should stop actively pushing." if ok else f"Damp request returned code {damp_code}."
        self._set_wrist_status(active=False, message=message)
        return (200 if ok else 502), {
            "ok": ok,
            "message": message,
            "select_mode_code": select_code,
            "stop_move_code": stop_code,
            "damp_code": damp_code,
            "wrist": wrist_status,
        }

    @staticmethod
    def _coerce_float(payload: dict[str, Any], name: str, default: float, low: float, high: float) -> float:
        try:
            value = float(payload.get(name, default))
        except (TypeError, ValueError):
            raise ValueError(f"{name} must be a number")
        if not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
        if value < low or value > high:
            raise ValueError(f"{name} must be between {low} and {high}")
        return value

    def command_loco(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        action = str(payload.get("action", "")).strip()
        allowed_actions = {
            "ready",
            "balance_stand",
            "stand_up",
            "start",
            "stop_move",
            "damp",
            "zero_torque",
            "high_stand",
            "low_stand",
            "set_height",
            "set_swing_height",
            "velocity",
            "move",
            "continuous_gait_on",
            "continuous_gait_off",
            "next_foot_left",
            "next_foot_right",
            "wave_hand",
            "shake_hand",
            "shake_hand_start",
            "shake_hand_end",
            "enable_odom",
            "disable_odom",
            "get_odom",
            "set_target_position",
            "get_fsm_id",
            "get_fsm_mode",
            "get_balance_mode",
            "get_swing_height",
            "get_stand_height",
            "get_phase",
        }
        if action not in allowed_actions:
            return 400, {"ok": False, "error": f"Unsupported loco action: {action}"}

        try:
            vx = self._coerce_float(payload, "vx", 0.0, -1.0, 1.0)
            vy = self._coerce_float(payload, "vy", 0.0, -0.5, 0.5)
            vyaw = self._coerce_float(payload, "vyaw", 0.0, -1.0, 1.0)
            duration = self._coerce_float(payload, "duration", 1.0, 0.1, 10.0)
            stand_height = self._coerce_float(payload, "stand_height", 0.0, 0.0, 1.0)
            swing_height = self._coerce_float(payload, "swing_height", 0.05, 0.0, 0.3)
            target_x = self._coerce_float(payload, "target_x", 0.0, -2.0, 2.0)
            target_y = self._coerce_float(payload, "target_y", 0.0, -2.0, 2.0)
            target_yaw = self._coerce_float(payload, "target_yaw", 0.0, -3.14, 3.14)
        except ValueError as exc:
            return 400, {"ok": False, "error": str(exc)}
        continuous = bool(payload.get("continuous_move"))
        target_relative = bool(payload.get("target_relative", True))

        with self.command_lock:
            loco_client = self.loco_client
            motion_switcher = self.motion_switcher
            cancel = self.wrist_cancel
        if loco_client is None:
            return 503, {"ok": False, "error": "H1 loco client is not available."}
        if cancel is not None:
            cancel.set()

        command = {
            "action": action,
            "vx": round(vx, 4),
            "vy": round(vy, 4),
            "vyaw": round(vyaw, 4),
            "duration": round(duration, 4),
            "stand_height": round(stand_height, 4),
            "swing_height": round(swing_height, 4),
            "target_x": round(target_x, 4),
            "target_y": round(target_y, 4),
            "target_yaw": round(target_yaw, 4),
            "continuous_move": continuous,
            "target_relative": target_relative,
            "time": time.time(),
        }

        self._set_loco_status(active=True, message=f"Sending loco {action}.", last_command=command)
        select_code = None
        call_code = None
        stop_code = None
        motion_mode = None
        try:
            if motion_switcher is not None:
                with contextlib.suppress(Exception):
                    check_code, motion_mode = motion_switcher.CheckMode()
                    command["motion_check_code"] = check_code

            result_data = None
            if action in ("ready", "balance_stand"):
                call_code = loco_client.BalanceStand()
            elif action == "stand_up":
                call_code = loco_client.StandUp()
            elif action == "start":
                call_code = loco_client.Start()
            elif action == "stop_move":
                call_code = loco_client.SetVelocity(0.0, 0.0, 0.0, 0.4)
            elif action == "damp":
                stop_code = loco_client.SetVelocity(0.0, 0.0, 0.0, 0.2)
                call_code = loco_client.SetFsmId(1)
            elif action == "zero_torque":
                stop_code = loco_client.SetVelocity(0.0, 0.0, 0.0, 0.2)
                call_code = loco_client.SetFsmId(0)
            elif action == "high_stand":
                call_code = loco_client.HighStand()
            elif action == "low_stand":
                call_code = loco_client.LowStand()
            elif action == "set_height":
                call_code = loco_client.SetStandHeight(stand_height)
            elif action == "set_swing_height":
                call_code = loco_client.SetSwingHeight(swing_height)
            elif action == "velocity":
                call_code = loco_client.SetVelocity(vx, vy, vyaw, duration)
            elif action == "move":
                call_code = loco_client.Move(vx, vy, vyaw, continuous)
            elif action == "continuous_gait_on":
                call_code = loco_client.ContinuousGait(True)
            elif action == "continuous_gait_off":
                call_code = loco_client.ContinuousGait(False)
            elif action == "next_foot_left":
                call_code = loco_client.SetNextFoot(True)
            elif action == "next_foot_right":
                call_code = loco_client.SetNextFoot(False)
            elif action == "wave_hand":
                call_code = loco_client.WaveHand()
            elif action == "shake_hand":
                call_code = loco_client.ShakeHand()
            elif action == "shake_hand_start":
                call_code = loco_client.ShakeHand(0)
            elif action == "shake_hand_end":
                call_code = loco_client.ShakeHand(1)
            elif action == "enable_odom":
                call_code = loco_client.EnableOdom()
            elif action == "disable_odom":
                call_code = loco_client.DisableOdom()
            elif action == "get_odom":
                call_code, result_data = loco_client.GetOdom()
            elif action == "set_target_position":
                call_code = loco_client.SetTargetPos(target_x, target_y, target_yaw, target_relative)
            elif action == "get_fsm_id":
                call_code, result_data = loco_client.GetFsmId()
            elif action == "get_fsm_mode":
                call_code, result_data = loco_client.GetFsmMode()
            elif action == "get_balance_mode":
                call_code, result_data = loco_client.GetBalanceMode()
            elif action == "get_swing_height":
                call_code, result_data = loco_client.GetSwingHeight()
            elif action == "get_stand_height":
                call_code, result_data = loco_client.GetStandHeight()
            elif action == "get_phase":
                call_code, result_data = loco_client.GetPhase()

            command = {
                **command,
                "select_mode_code": select_code,
                "call_code": call_code,
                "stop_code": stop_code,
                "motion_mode": motion_mode,
                "result": result_data,
            }
            ok = call_code in (0, None)
            message = f"Loco {action} accepted." if ok else f"Loco {action} returned code {call_code}."
            with self.command_lock:
                self._append_loco_history(command)
            self._set_loco_status(
                available=True,
                active=False,
                message=message,
                last_command=command,
                motion_mode=motion_mode,
            )
            return (200 if ok else 502), {"ok": ok, "message": message, "result": result_data, "status": self.loco_snapshot()}
        except Exception as exc:
            command = {**command, "select_mode_code": select_code, "call_code": call_code, "error": str(exc)}
            with self.command_lock:
                self._append_loco_history(command)
            self._set_loco_status(active=False, message=f"Loco {action} failed: {exc}", last_command=command)
            return 500, {"ok": False, "error": str(exc), "status": self.loco_snapshot()}

    def switch_xr_mode(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        mode = str(payload.get("mode", "")).strip()
        modes = {
            "pad": {
                "label": "Floating VR Control Pad",
                "XR_ROOT_CHILDREN_VISUAL": "1",
                "XR_HEAD_TILT_LOCO": "0",
                "XR_POSITION_MATCH_LOCO": "0",
            },
            "head_tilt": {
                "label": "Head Rotation Control",
                "XR_ROOT_CHILDREN_VISUAL": "0",
                "XR_HEAD_TILT_LOCO": "1",
                "XR_POSITION_MATCH_LOCO": "0",
            },
            "position_match": {
                "label": "Position Matching",
                "XR_ROOT_CHILDREN_VISUAL": "0",
                "XR_HEAD_TILT_LOCO": "0",
                "XR_POSITION_MATCH_LOCO": "1",
            },
        }
        if mode not in modes:
            return 400, {"ok": False, "error": "mode must be one of: pad, head_tilt, position_match"}

        env = modes[mode]
        XR_TELEOP_MODE_DROPIN.parent.mkdir(parents=True, exist_ok=True)
        XR_TELEOP_MODE_DROPIN.write_text(
            "\n".join(
                [
                    "[Service]",
                    f"Environment=XR_ROOT_CHILDREN_VISUAL={env['XR_ROOT_CHILDREN_VISUAL']}",
                    f"Environment=XR_HEAD_TILT_LOCO={env['XR_HEAD_TILT_LOCO']}",
                    f"Environment=XR_POSITION_MATCH_LOCO={env['XR_POSITION_MATCH_LOCO']}",
                    "",
                ]
            ),
            encoding="utf-8",
        )

        try:
            subprocess.run(
                ["systemctl", "--user", "daemon-reload"],
                check=True,
                capture_output=True,
                text=True,
                timeout=15,
            )
            subprocess.run(
                ["systemctl", "--user", "kill", "--kill-who=all", "--signal=KILL", "xr-teleop.service"],
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )
            subprocess.run(
                ["systemctl", "--user", "restart", "--no-block", "xr-teleop.service"],
                check=True,
                capture_output=True,
                text=True,
                timeout=15,
            )
        except subprocess.CalledProcessError as exc:
            return 500, {
                "ok": False,
                "error": f"Could not switch XR mode: {exc.stderr.strip() or exc.stdout.strip() or exc}",
                "mode": mode,
            }
        except Exception as exc:
            return 500, {"ok": False, "error": f"Could not switch XR mode: {exc}", "mode": mode}

        return 200, {
            "ok": True,
            "mode": mode,
            "message": f"XR teleop switched to {env['label']}.",
            "env": {
                "XR_ROOT_CHILDREN_VISUAL": env["XR_ROOT_CHILDREN_VISUAL"],
                "XR_HEAD_TILT_LOCO": env["XR_HEAD_TILT_LOCO"],
                "XR_POSITION_MATCH_LOCO": env["XR_POSITION_MATCH_LOCO"],
            },
        }

    def command_wrist(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        if not has_risk_ack(payload):
            return 400, {"ok": False, "error": "Command requires armed=true and i_understand_risk=true."}

        def number(name: str, default: float, low: float, high: float) -> float:
            try:
                value = float(payload.get(name, default))
            except (TypeError, ValueError):
                raise ValueError(f"{name} must be a number")
            if not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number")
            if value < low or value > high:
                raise ValueError(f"{name} must be between {low} and {high}")
            return value

        try:
            mode = str(payload.get("mode", "absolute"))
            control_path = str(payload.get("control_path", "arm_sdk"))
            if mode not in {"absolute", "relative", "oscillate"}:
                raise ValueError("mode must be one of absolute, relative, oscillate")
            if control_path not in {"arm_sdk", "lowcmd"}:
                raise ValueError("control_path must be one of arm_sdk, lowcmd")
            if mode == "oscillate" and control_path != "lowcmd":
                raise ValueError("oscillate mode requires control_path=lowcmd")
            target = number("target_q", 0.0, WRIST_LIMITS[0], WRIST_LIMITS[1])
            delta = number("delta_q", 0.0, -0.25, 0.25)
            kp = number("kp", 4.0, 0.0, 30.0)
            kd = number("kd", 0.35, 0.0, 5.0)
            duration = number("duration", 0.35, 0.05, 12.0)
            rate = number("rate", 80.0, 20.0, 200.0)
            period = number("period", 2.0, 0.4, 8.0)
        except ValueError as exc:
            return 400, {"ok": False, "error": str(exc)}

        with self.command_lock:
            publisher = self.wrist_publisher
            lowcmd_publisher = self.lowcmd_publisher
            motion_switcher = self.motion_switcher
            msg = self.lowstate_msg
            previous_cancel = self.wrist_cancel
            torso_cancel = self.torso_cancel
        if self.lowcmd_factory is None or self.crc is None:
            return 503, {"ok": False, "error": "DDS command factory is not available."}
        if control_path == "lowcmd" and lowcmd_publisher is None:
            return 503, {"ok": False, "error": "DDS lowcmd publisher is not available."}
        if control_path != "lowcmd" and publisher is None:
            return 503, {"ok": False, "error": "DDS arm_sdk publisher is not available."}
        if msg is None:
            return 503, {"ok": False, "error": "No rt/lowstate sample is available yet."}
        if previous_cancel is not None:
            previous_cancel.set()
        if torso_cancel is not None:
            torso_cancel.set()

        start_q = float(msg.motor_state[RIGHT_WRIST_YAW].q)
        if mode == "relative":
            target_q = start_q + delta
        else:
            target_q = target
        target_q = max(WRIST_LIMITS[0], min(WRIST_LIMITS[1], target_q))
        if payload.get("auto_gains"):
            kp, kd = self._auto_wrist_gains(mode, start_q, target_q, delta, period)

        cancel = threading.Event()
        command = {
            "mode": mode,
            "start_q": round(start_q, 6),
            "target_q": round(target_q, 6),
            "kp": kp,
            "kd": kd,
            "duration": duration,
            "rate": rate,
            "period": period,
            "control_path": control_path,
        }

        def run_command() -> None:
            self._set_wrist_status(
                available=True,
                active=True,
                message=f"Publishing right wrist {control_path} command.",
                last_command=command,
            )
            publish_period = 1.0 / rate
            writes = 0
            try:
                hold_q = [float(msg.motor_state[i].q) for i in range(27)]
                center_q = hold_q[RIGHT_WRIST_YAW]
                if control_path == "lowcmd" and motion_switcher is not None:
                    code, result = motion_switcher.CheckMode()
                    if code == 0 and result and result.get("name"):
                        motion_switcher.ReleaseMode()
                        time.sleep(0.25)
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline and not cancel.is_set():
                    with self.command_lock:
                        latest_msg = self.lowstate_msg
                        latest_publisher = self.wrist_publisher
                        latest_lowcmd_publisher = self.lowcmd_publisher
                    if latest_msg is None:
                        break
                    if control_path == "lowcmd":
                        if latest_lowcmd_publisher is None:
                            break
                        elapsed = duration - max(0.0, deadline - time.monotonic())
                        wrist_q = (
                            center_q + delta * math.sin((2.0 * math.pi * elapsed) / command["period"])
                            if mode == "oscillate"
                            else target_q
                        )
                        latest_lowcmd_publisher.Write(self._build_lowcmd_wrist_cmd(latest_msg, hold_q, wrist_q, kp, kd))
                    else:
                        if latest_publisher is None:
                            break
                        latest_publisher.Write(self._build_arm_sdk_cmd(latest_msg, target_q, kp, kd, weight=1.0))
                    writes += 1
                    time.sleep(publish_period)
                if control_path == "lowcmd" and motion_switcher is not None:
                    with contextlib.suppress(Exception):
                        motion_switcher.SelectMode("ai")
                message = "Right wrist command cancelled." if cancel.is_set() else f"Right wrist command complete ({writes} writes)."
                self._set_wrist_status(active=False, message=message, last_command={**command, "writes": writes})
            except Exception as exc:
                self._set_wrist_status(active=False, message=f"Right wrist command failed: {exc}", last_command=command)

        thread = threading.Thread(target=run_command, name="right-wrist-command", daemon=True)
        with self.command_lock:
            self.wrist_cancel = cancel
            self.wrist_thread = thread
        thread.start()
        return 202, {"ok": True, "status": self.wrist_snapshot()}

    def set_camera_frame(self, frame: bytes) -> None:
        with self.camera_lock:
            self.camera_frame = frame
            self.camera_timestamp = time.time()
            self.camera_error = None
            self.camera_condition.notify_all()

    def set_camera_error(self, error: str | None) -> None:
        with self.camera_lock:
            self.camera_error = error

    def get_camera_frame(self) -> bytes | None:
        with self.camera_lock:
            return self.camera_frame

    def wait_for_camera_frame(self, last_timestamp: float | None, timeout: float = 1.0) -> tuple[bytes | None, float | None]:
        with self.camera_condition:
            if self.camera_frame is not None and self.camera_timestamp != last_timestamp:
                return self.camera_frame, self.camera_timestamp
            self.camera_condition.wait(timeout)
            return self.camera_frame, self.camera_timestamp

    def _set_error(self, error: str) -> None:
        with self.lock:
            self.latest = {
                **self.latest,
                "connected": False,
                "timestamp": time.time(),
                "error": error,
            }

    def _run(self) -> None:
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
            from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_
            from unitree_sdk2py.utils.crc import CRC
        except Exception as exc:
            self._set_error(f"Could not import Unitree SDK: {exc}")
            return

        hand_msg = None
        hand_samples = 0
        hand_timestamp = None
        last_snapshot_at = 0.0

        def on_hand(msg: Any) -> None:
            nonlocal hand_msg, hand_samples, hand_timestamp
            hand_msg = msg
            hand_samples += 1
            hand_timestamp = time.time()
            hands = handstate_to_dict(hand_msg, hand_samples, hand_timestamp)
            with self.lock:
                self.latest["hands"] = hands

        def on_lowstate(msg: Any) -> None:
            nonlocal last_snapshot_at
            try:
                now = time.time()
                self.samples += 1
                self.sample_times.append(now)
                hands = handstate_to_dict(hand_msg, hand_samples, hand_timestamp)
                self.recorder.write_sample(lowstate_record(msg, self.samples, hands, hand_samples, hand_timestamp))
                if now - last_snapshot_at < 1.0 / 30.0:
                    return
                last_snapshot_at = now
                if len(self.sample_times) > 1:
                    elapsed = self.sample_times[-1] - self.sample_times[0]
                    rate = (len(self.sample_times) - 1) / elapsed if elapsed > 0 else 0
                else:
                    rate = 0

                snapshot = lowstate_to_dict(msg, self.samples, rate, hands)
                with self.lock:
                    self.latest = snapshot
                with self.command_lock:
                    self.lowstate_msg = msg
            except Exception as exc:
                self._set_error(f"LowState callback failed: {exc}")

        try:
            ChannelFactoryInitialize(self.domain, self.camera_source or None)
            wrist_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            wrist_pub.Init()
            lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
            lowcmd_pub.Init()
            motion_switcher = MotionSwitcherClient()
            motion_switcher.SetTimeout(5.0)
            motion_switcher.Init()
            loco_client = LocoClient()
            loco_client.SetTimeout(5.0)
            loco_client.Init()
            with self.command_lock:
                self.wrist_publisher = wrist_pub
                self.lowcmd_publisher = lowcmd_pub
                self.motion_switcher = motion_switcher
                self.loco_client = loco_client
                self.lowcmd_factory = unitree_hg_msg_dds__LowCmd_
                self.lowcmd_type = LowCmd_
                self.crc = CRC()
                self.wrist_status = {
                    **self.wrist_status,
                    "available": True,
                    "message": "DDS arm_sdk publisher is ready.",
                    "updated_at": time.time(),
                }
                self.loco_status = {
                    **self.loco_status,
                    "available": True,
                    "message": "H1 loco client is ready.",
                    "updated_at": time.time(),
                }

            sub = ChannelSubscriber("rt/lowstate", LowState_)
            sub.Init(on_lowstate, 10)

            hand_sub = ChannelSubscriber("rt/inspire/state", MotorStates_)
            hand_sub.Init(on_hand, 10)
        except Exception as exc:
            self._set_error(f"Could not initialize DDS subscriber: {exc}")
            return

        while self.running:
            try:
                hands = handstate_to_dict(hand_msg, hand_samples, hand_timestamp)
                with self.lock:
                    self.latest["hands"] = hands
                time.sleep(0.25)
            except Exception as exc:
                self._set_error(f"Subscriber loop failed: {exc}")
                time.sleep(0.25)


class TelemetryHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    request_queue_size = 64


class TelemetryHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    store: TelemetryStore

    def log_message(self, fmt: str, *args: Any) -> None:
        sys.stdout.write("[%s] %s\n" % (self.log_date_time_string(), fmt % args))

    def do_GET(self) -> None:
        request_path = urlsplit(self.path).path
        if request_path in ("/", "/index.html"):
            self._send_file(STATIC_DIR / "index.html", "text/html; charset=utf-8")
        elif request_path == "/app.js":
            self._send_file(STATIC_DIR / "app.js", "application/javascript; charset=utf-8")
        elif request_path == "/viewer.js":
            self._send_file(STATIC_DIR / "viewer.js", "application/javascript; charset=utf-8")
        elif request_path == "/diagram.js":
            self._send_file(STATIC_DIR / "diagram.js", "application/javascript; charset=utf-8")
        elif request_path == "/torso.js":
            self._send_file(STATIC_DIR / "torso.js", "application/javascript; charset=utf-8")
        elif request_path == "/styles.css":
            self._send_file(STATIC_DIR / "styles.css", "text/css; charset=utf-8")
        elif request_path == "/api/state":
            self._send_json(self.store.snapshot())
        elif request_path == "/api/camera":
            self._send_json(self.store.camera_snapshot())
        elif request_path == "/api/ros-graph":
            self._send_json(self.store.ros_graph_snapshot())
        elif request_path == "/api/recording/status":
            self._send_json(self.store.recording_status())
        elif request_path == "/api/recording/files":
            self._send_json(self.store.recording_files())
        elif request_path.startswith("/api/recording/files/"):
            filename = unquote(request_path.removeprefix("/api/recording/files/"))
            try:
                path = self.store.recording_file_path(filename)
            except FileNotFoundError:
                self.send_error(HTTPStatus.NOT_FOUND, "Recording not found")
                return
            except ValueError as exc:
                self.send_error(HTTPStatus.BAD_REQUEST, str(exc))
                return
            self._send_file(path, "application/x-ndjson; charset=utf-8")
        elif request_path == "/api/diagrams":
            self._send_json(self.store.diagram_files())
        elif request_path.startswith("/api/diagrams/"):
            filename = unquote(request_path.removeprefix("/api/diagrams/"))
            try:
                path = self.store.diagram_file_path(filename)
            except FileNotFoundError:
                self.send_error(HTTPStatus.NOT_FOUND, "Diagram not found")
                return
            except ValueError as exc:
                self.send_error(HTTPStatus.BAD_REQUEST, str(exc))
                return
            self._send_file(path, "application/xml; charset=utf-8")
        elif request_path == "/api/wrist/status":
            self._send_json(self.store.wrist_snapshot())
        elif request_path == "/api/torso/status":
            self._send_json(self.store.torso_snapshot())
        elif request_path == "/api/loco/status":
            self._send_json(self.store.loco_snapshot())
        elif request_path == "/camera.mjpg":
            self._send_camera_stream()
        elif request_path == "/events":
            self._send_events()
        elif request_path.startswith("/models/") or request_path.startswith("/vendor/") or request_path.startswith("/assets/"):
            self._send_static_asset(request_path)
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def do_POST(self) -> None:
        request_path = urlsplit(self.path).path
        if request_path not in (
            "/api/wrist/command",
            "/api/wrist/stop",
            "/api/torso/twist",
            "/api/torso/stop",
            "/api/robot/chill",
            "/api/robot/home",
            "/api/robot/straight",
            "/api/loco/command",
            "/api/xr/mode",
            "/api/recording/start",
            "/api/recording/stop",
            "/api/recording/pose",
            "/api/recording/sequence",
            "/api/recording/replay/robot",
        ):
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")
            return

        payload: dict[str, Any] = {}
        if request_path in (
            "/api/wrist/command",
            "/api/torso/twist",
            "/api/robot/chill",
            "/api/loco/command",
            "/api/xr/mode",
            "/api/recording/start",
            "/api/recording/pose",
            "/api/recording/sequence",
            "/api/recording/replay/robot",
        ):
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length > MAX_JSON_BODY_BYTES:
                    self.close_connection = True
                    self._send_json_status(
                        {"ok": False, "error": f"JSON body must be at most {MAX_JSON_BODY_BYTES} bytes."},
                        HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
                    )
                    return
                body = self.rfile.read(length)
                decoded = json.loads(body.decode("utf-8")) if body else {}
                if not isinstance(decoded, dict):
                    raise ValueError("JSON body must be an object")
                payload = decoded
            except Exception as exc:
                self._send_json_status({"ok": False, "error": f"Invalid JSON body: {exc}"}, HTTPStatus.BAD_REQUEST)
                return

        if request_path == "/api/recording/start":
            status, response = self.store.start_recording(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/recording/stop":
            status, response = self.store.stop_recording()
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/recording/pose":
            status, response = self.store.capture_pose(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/recording/sequence":
            status, response = self.store.save_sequence(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/recording/replay/robot":
            status, response = self.store.request_robot_replay(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        self.store.record_command_event(request_path, payload)

        if request_path == "/api/wrist/stop":
            self._send_json(self.store.stop_wrist())
            return

        if request_path == "/api/torso/twist":
            status, response = self.store.command_torso_twist(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/torso/stop":
            self._send_json(self.store.stop_torso())
            return

        if request_path == "/api/robot/chill":
            status, response = self.store.request_chill(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/robot/home":
            status, response = self.store.request_home()
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/robot/straight":
            status, response = self.store.request_straight()
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/loco/command":
            status, response = self.store.command_loco(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/xr/mode":
            status, response = self.store.switch_xr_mode(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        status, response = self.store.command_wrist(payload)
        self._send_json_status(response, HTTPStatus(status))

    def _send_file(self, path: Path, content_type: str) -> None:
        try:
            body = path.read_bytes()
        except OSError:
            self.send_error(HTTPStatus.NOT_FOUND, "Missing asset")
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_static_asset(self, request_path: str) -> None:
        relative = Path(unquote(request_path.lstrip("/")))
        path = (STATIC_DIR / relative).resolve()
        try:
            path.relative_to(STATIC_DIR.resolve())
        except ValueError:
            self.send_error(HTTPStatus.FORBIDDEN, "Forbidden")
            return

        content_type = mimetypes.guess_type(path.name)[0]
        if content_type is None:
            if path.suffix.lower() == ".stl":
                content_type = "model/stl"
            elif path.suffix.lower() in (".urdf", ".xml"):
                content_type = "application/xml"
            elif path.suffix.lower() == ".js":
                content_type = "application/javascript"
            else:
                content_type = "application/octet-stream"

        self._send_file(path, content_type)

    def _send_json(self, data: dict[str, Any]) -> None:
        self._send_json_status(data, HTTPStatus.OK)

    def _send_json_status(self, data: dict[str, Any], status: HTTPStatus) -> None:
        body = json.dumps(data, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        while True:
            payload = json.dumps(self.store.snapshot(), separators=(",", ":"))
            try:
                self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                break
            time.sleep(0.2)

    def _send_camera_stream(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Pragma", "no-cache")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        last_timestamp: float | None = None
        while True:
            frame, timestamp = self.store.wait_for_camera_frame(last_timestamp, timeout=1.0)
            if frame is None:
                continue
            if timestamp == last_timestamp:
                continue
            last_timestamp = timestamp
            payload = (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                + f"Content-Length: {len(frame)}\r\n\r\n".encode("utf-8")
                + frame
                + b"\r\n"
            )
            try:
                self.wfile.write(payload)
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                break


def main() -> None:
    parser = argparse.ArgumentParser(description="Unitree telemetry web dashboard")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--robot-host", default="192.168.123.164")
    parser.add_argument("--camera-source", default=os.environ.get("CAMERA_SOURCE", ""))
    parser.add_argument("--camera-resolution", type=int, default=int(os.environ.get("CAMERA_RESOLUTION", "360")))
    parser.add_argument("--camera-output", default=str(CAMERA_JPEG_PATH))
    parser.add_argument(
        "--camera-backend",
        choices=("auto", "teleimager", "ros2"),
        default=os.environ.get("CAMERA_BACKEND", "auto"),
    )
    parser.add_argument("--camera-bridge", action="store_true")
    parser.add_argument("--disable-camera", action="store_true")
    args = parser.parse_args()

    if args.camera_bridge:
        camera_bridge_main(args.camera_source, args.camera_resolution, Path(args.camera_output))
        return

    store = TelemetryStore(domain=args.domain, robot_host=args.robot_host)
    store.camera_source = args.camera_source or route_interface(args.robot_host) or default_interface() or ""
    store.camera_resolution = args.camera_resolution
    store.camera_backend = args.camera_backend
    store.start()
    if args.disable_camera:
        store.set_camera_error("Camera worker disabled for this server run.")
    else:
        start_camera_bridge(store)

    TelemetryHandler.store = store
    server = TelemetryHTTPServer((args.host, args.port), TelemetryHandler)

    print("Unitree telemetry dashboard")
    print(f"Listening on http://{args.host}:{args.port}")
    print(f"Try from another machine: http://{public_host()}:{args.port}")
    print("Press Ctrl+C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        store.running = False
        if store.camera_process is not None:
            store.camera_process.terminate()
        server.server_close()


if __name__ == "__main__":
    main()
