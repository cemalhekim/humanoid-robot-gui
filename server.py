#!/usr/bin/env python3
"""Unitree H1-2 telemetry web dashboard.

Runs on the robot PC. Subscribes to rt/lowstate continuously and serves a
dependency-free web UI with JSON and Server-Sent Events endpoints.
"""

from __future__ import annotations

import argparse
import base64
import binascii
import csv
import json
import math
import mimetypes
import os
import queue
import shutil
import socket
import ssl
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
import urllib.error
import urllib.request
from urllib.parse import unquote, urlsplit, parse_qs
from typing import Any

import kinematics
import tracking

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
# Scratch dir for unsaved poses/sequences that the operator moves the robot with
# directly from the 3D editor. Files here are written, replayed through the exact
# same validated pipeline as saved recordings, then deleted. A subdirectory keeps
# them out of the recordings file list (which globs RECORDINGS_DIR non-recursively).
EPHEMERAL_REPLAY_DIR = RECORDINGS_DIR / ".ephemeral"
# Operator feedback on LLM pose proposals (liked/disliked/executed events).
# Untracked runtime data; fed back into the LLM prompt as learned examples.
FEEDBACK_DIR = APP_DIR / "feedback"
POSE_FEEDBACK_CSV = FEEDBACK_DIR / "pose_feedback.csv"
# Attached reference images collected alongside the labeled feedback rows, keyed
# by proposal_id. Live copy on the robot; synced to the tracked repo dir like the CSV.
POSE_FEEDBACK_IMAGE_DIR = FEEDBACK_DIR / "images"
POSE_FEEDBACK_FIELDS = [
    "timestamp_iso", "proposal_id", "event", "request_text",
    "joints_json", "semantics_json", "comment", "image_path", "parent_id",
]
POSE_FEEDBACK_COMMENT_MAX = 500
LEARNED_FEEDBACK_LIKED_MAX = 8
LEARNED_FEEDBACK_DISLIKED_MAX = 4
# Repo-visible snapshot of the live feedback CSV. The live feedback/ file must
# stay UNTRACKED: the robot's autoupdate runs `git reset --hard origin/main`,
# which would clobber a tracked live file and lose rows. The tracked copy under
# data/ is regenerated from the live file on every sync, so even a lost local
# commit self-heals on the next push.
FEEDBACK_REPO_CSV = APP_DIR / "data" / "pose_feedback.csv"
FEEDBACK_REPO_IMAGE_DIR = APP_DIR / "data" / "images"
FEEDBACK_SYNC_PUSH_URL = "git@github.com:cemalhekim/humanoid-robot-gui.git"
# Sync runs only where this write-access deploy key exists (the robot).
FEEDBACK_SYNC_KEY = Path(os.environ.get(
    "FEEDBACK_SYNC_KEY", str(Path.home() / ".ssh" / "robot_feedback_deploy")
))
# Auto-push must be explicitly enabled (set on the robot's service unit). Without
# it, a developer who merely has the deploy key at the default path never
# auto-commits+pushes to main on server start.
FEEDBACK_SYNC_ENABLED = os.environ.get("FEEDBACK_SYNC_ENABLE", "0") not in ("0", "false", "False", "")
FEEDBACK_SYNC_DEBOUNCE_SECONDS = 20.0
# network_status() is expensive (subprocess + /proc + UDP socket); cache it since
# the interface/host is effectively static between snapshots.
NETWORK_STATUS_TTL_SECONDS = 5.0
# The always-on welcome/onboarding page (GitHub Pages). The robot only
# redirects here; it never serves its own copy.
WELCOME_PAGE_URL = "https://cemalhekim.github.io/humanoid-robot-gui/"
# Entrances the welcome page offers; probed by /api/entrances.
ENTRANCE_PROBES = {
    "wifi": "http://10.2.100.142:8088",
    "ethernet": "http://192.168.123.164:8088",
}
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
# The legacy damped→balanced→responsive curve tops out here (the OLD 100%).
# Values <= this behave exactly as before the range was doubled.
ARM_REPLAY_RESPONSE_LEGACY_MAX = 2.5
# Doubled slider range: (legacy_max, max] is the overdrive zone, scaling the
# PID linearly up to 2x the legacy-top aggressiveness at the new 100%.
# The old 100% now sits at the slider's 50% mark and is the UI default.
ARM_REPLAY_RESPONSE_MAX = 5.0
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
# --- Candidate B (structural), both OFF by default so behaviour is unchanged ---
# Model-based gravity feed-forward: blend the URDF-mass gravity torque (kinematics.
# gravity_torques) into the feed-forward instead of relying only on the measured
# tau_est, which contains the controller's own PD reaction and any contact. 0 = off,
# 1 = feed-forward is the model only (the adaptive learn term still adds on top).
ARM_REPLAY_GRAVITY_MODEL_SCALE = 0.0
# 1 = the URDF hand links count in the gravity model (the robot has Inspire hands);
# 0 = arm links only (the MuJoCo twin is handless).
ARM_REPLAY_GRAVITY_MODEL_INCLUDE_HANDS = 1.0
# Inside the lock band the PID correction is zeroed today, leaving the gravity-learn
# integrator alone -- an undamped integrator, hence the slow limit cycle seen on the
# twin (period 5-9 s). This keeps a scaled P+D correction active inside the band.
ARM_REPLAY_INBAND_CORRECTION_SCALE = 0.0
# Gravity-learn gating (Candidate B follow-up). 1 = the adaptive learn integral only
# runs while EVERY commanded joint is stationary (hold phase), so the dynamic
# coupling / contact torque of a still-moving shoulder is not learned as gravity
# by a wrist that already sits at its target (the #482 wind-up on the twin). 0 = off.
ARM_REPLAY_LEARN_ARM_STATIONARY_GATE = 0.0
# Leak rate (1/s) applied to the learned torque while the joint is NOT in the learn
# band, so a wound-up value decays instead of pinning the joint at a wrong
# equilibrium forever. 0 = no leak (shipped behaviour).
ARM_REPLAY_LEARN_LEAK_PER_S = 0.0
# Self-collision check along the planned joint-space PATH (live pose -> frame 0 ->
# ... -> last frame), sampled every ARM_REPLAY_PATH_CHECK_STEP_RAD with the sphere
# model in tracking.mimic_pose_collides. The result is always reported in the plan
# (plan["path_collision"]); with BLOCK >= 0.5 a hit also invalidates execution.
# 18 of 1000 random twin motions had a straight-line path through the torso/hip.
ARM_REPLAY_PATH_COLLISION_BLOCK = 0.0
ARM_REPLAY_PATH_CHECK_STEP_RAD = 0.05
ARM_REPLAY_PATH_CHECK_MAX_SAMPLES = 600
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
# rt/inspire/state is published continuously while inspire_h1 runs, so a gap
# this long means the bridge is down -- not that the hands are idle.
HAND_STATE_STALE_SECONDS = 5.0
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
# Per-joint (kp, kd) keyed by motor index, reused when driving arms to a target
# via lowcmd during a torso twist (motion mode released) so they hold against
# gravity just like the arm_sdk path.
ARM_SDK_GAIN_BY_INDEX = {joint: (ARM_SDK_KP[k], ARM_SDK_KD[k]) for k, joint in enumerate(ARM_SDK_JOINTS)}


# --- Simulation-only tuning overrides -----------------------------------------
# RTW_TUNING_JSON=<file> replaces the arm-replay constants above at import time,
# so an optimizer can evaluate parameter sets against the MuJoCo twin
# (simulation/h1_2_twin/optimize_params.py). Unset -- as on the robot -- this is
# a no-op. Only ARM_REPLAY_* constants and ARM_SDK_KP/KD may be overridden;
# unknown keys or shape mismatches abort start-up instead of silently running
# with half-applied tuning.
def _apply_tuning_overrides() -> dict[str, Any]:
    path = os.environ.get("RTW_TUNING_JSON", "").strip()
    if not path:
        return {}
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise SystemExit("RTW_TUNING_JSON must contain a JSON object")
    scope = globals()
    applied: dict[str, Any] = {}
    for key, value in data.items():
        if not (key.startswith("ARM_REPLAY_") or key in ("ARM_SDK_KP", "ARM_SDK_KD")) or key not in scope:
            raise SystemExit(f"RTW_TUNING_JSON: unknown tuning key {key!r}")
        current = scope[key]
        if isinstance(current, dict):
            if not isinstance(value, dict) or set(value) - set(current):
                raise SystemExit(f"RTW_TUNING_JSON: {key} expects keys {sorted(current)}")
            merged = dict(current)
            for sub, sub_value in value.items():
                merged[sub] = tuple(float(v) for v in sub_value) if isinstance(current[sub], tuple) else float(sub_value)
            scope[key] = merged
        elif isinstance(current, list):
            if not isinstance(value, list) or len(value) != len(current):
                raise SystemExit(f"RTW_TUNING_JSON: {key} expects a list of {len(current)} numbers")
            scope[key] = [float(v) for v in value]
        else:
            scope[key] = float(value)
        applied[key] = scope[key]
    # Re-derive the constants computed from the ones that may have changed.
    scope["ARM_REPLAY_TOLERANCE_RAD"] = scope["ARM_REPLAY_LOCK_TOLERANCE_RAD"]
    scope["ARM_SDK_GAIN_BY_INDEX"] = {
        joint: (scope["ARM_SDK_KP"][k], scope["ARM_SDK_KD"][k]) for k, joint in enumerate(ARM_SDK_JOINTS)
    }
    print(f"[tuning] {len(applied)} overrides from {path}: {sorted(applied)}", file=sys.stderr, flush=True)
    return applied


TUNING_OVERRIDES = _apply_tuning_overrides()

REPLAY_COMMAND_SCOPES = {
    "all": list(JOINT_NAMES),
    "arms": JOINT_GROUPS["left_arm"] + JOINT_GROUPS["right_arm"] + JOINT_GROUPS["waist"],
    "both_arms": JOINT_GROUPS["left_arm"] + JOINT_GROUPS["right_arm"],
    "right_arm": JOINT_GROUPS["right_arm"],
    "left_arm": JOINT_GROUPS["left_arm"],
}
JOINT_LIMITS = {
    12: (-1.2, 1.2),  # WaistYaw: conservative torso-twist clamp (mechanical is +/-2.35)
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
ARM_JOINT_INDEX_BY_NAME = {name: index for index, name in JOINT_NAMES.items() if 13 <= index <= 26}
ARM_PROPOSAL_TTL_SECONDS = 300.0
try:
    ARM_KINEMATICS: kinematics.ArmKinematics | None = kinematics.ArmKinematics()
    LLM_ARM_GUIDE = kinematics.arm_pose_guide(
        ARM_KINEMATICS,
        {name: JOINT_LIMITS[index] for name, index in ARM_JOINT_INDEX_BY_NAME.items()},
    )
except Exception as _kin_exc:  # pragma: no cover - URDF missing on exotic installs
    ARM_KINEMATICS = None
    LLM_ARM_GUIDE = f"(arm kinematics unavailable: {_kin_exc})"
# Waist ("torso") yaw is NOT part of the H1-2 arm_sdk joint set (H1_2_JointArmIndex
# is 13-26 only), so it cannot be moved through rt/arm_sdk. It is driven separately
# via rt/lowcmd commanding ONLY joint 12 -- legs and arms get mode=0 (no signal),
# and the arms keep running on arm_sdk. No motion-mode release is performed.
WAIST_YAW_JOINT = 12
WAIST_LOWCMD_KP = 200.0
WAIST_LOWCMD_KD = 5.0
WAIST_LOWCMD_MAX_VEL_RAD_S = 0.6
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
    "balance_mode": [0, 1],
}
# Loco actions that walk / translate the whole robot and therefore require an
# explicit risk acknowledgement (armed + i_understand_risk). Stops, posture and
# read-only get_* actions are intentionally excluded.
LOCO_MOBILITY_ACTIONS = frozenset({
    "start",
    "velocity",
    "move",
    "continuous_gait_on",
    "next_foot_left",
    "next_foot_right",
    "set_target_position",
})
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
    "set_balance_mode",
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

# ---------------------------------------------------------------------------
# On-prem LLM chatbot (Command Center assistant)
#
# The dashboard proxies chat to an OpenAI-compatible endpoint (Ollama on the
# lab "AI-DEV" host by default). The proxy is READ-ONLY: it can read live
# telemetry to answer questions but never issues robot commands. All settings
# are env-overridable so the endpoint/model can change without code edits.
# ---------------------------------------------------------------------------
LLM_ENABLED = os.environ.get("LLM_ENABLED", "1") not in ("0", "false", "False", "")
LLM_BASE_URL = os.environ.get("LLM_BASE_URL", "http://10.2.125.3:11434").rstrip("/")
LLM_MODEL = os.environ.get("LLM_MODEL", "qwen3:30b-a3b-instruct-2507-q4_K_M")
LLM_TWIN_VISION_ENABLED = os.environ.get("LLM_TWIN_VISION_ENABLED", "0") not in (
    "0", "false", "False", "",
)
# Keep the base64 image plus chat history below MAX_JSON_BODY_BYTES.
LLM_TWIN_IMAGE_MAX_BYTES = 650_000
# Operator-supplied reference photo for pose mimicry ("do this pose"). The client
# downscales before upload; this caps a single image so it plus the chat history
# stays under MAX_JSON_BODY_BYTES.
LLM_MIMIC_IMAGE_MAX_BYTES = 900_000
LLM_API_KEY = os.environ.get("LLM_API_KEY", "")
LLM_TIMEOUT_SECONDS = float(os.environ.get("LLM_TIMEOUT_SECONDS", "120"))

# ---------------------------------------------------------------------------
# Lab Home Assistant — read-only relay of the robot's smart-plug state for the
# welcome page's single wake/sleep button. The token stays server-side; with no
# token the endpoint reports "unknown" and the button falls back to a blind
# toggle. Switching the plug happens via HA's own local-only webhooks, not here.
# ---------------------------------------------------------------------------
HA_BASE_URL = os.environ.get("HA_BASE_URL", "http://10.2.200.100").rstrip("/")
HA_TOKEN = os.environ.get("HA_TOKEN", "")
HA_SWITCH_ENTITY = os.environ.get("HA_SWITCH_ENTITY", "switch.5g_construction_display")
HA_TIMEOUT_SECONDS = float(os.environ.get("HA_TIMEOUT_SECONDS", "6"))
# Transient connection failures to the LLM (flaky robot Wi-Fi) are retried, since
# the request never reached the model — no tool ran, so retrying is side-effect-free.
# The robot's USB Wi-Fi dongle drops for ~10-15s at a time (periodic driver scans),
# so the retries are SPREAD OUT (linear backoff) to WAIT OUT a drop rather than
# bunching inside it: attempts land at ~0, 1, 3, 6, 10, 15, 21 s. Chat then works
# through a drop (reply just takes longer) instead of erroring.
LLM_CONNECT_RETRIES = int(os.environ.get("LLM_CONNECT_RETRIES", "6"))
LLM_CONNECT_RETRY_BACKOFF_SECONDS = float(os.environ.get("LLM_CONNECT_RETRY_BACKOFF_SECONDS", "1.0"))
# Optional second chat backend: a Claude Code bridge (tools/claude_bridge.py)
# running on the operator's machine. Unset/empty = the Claude toggle reports
# "not configured". Example: CLAUDE_BRIDGE_URL=http://10.2.100.50:8399
CLAUDE_BRIDGE_URL = os.environ.get("CLAUDE_BRIDGE_URL", "").rstrip("/")
CLAUDE_BRIDGE_MODEL = os.environ.get("CLAUDE_BRIDGE_MODEL", "claude")
# Optional shared secret; when set (on both the robot and the bridge) it is sent
# as a bearer token so a random LAN host can't spend the operator's Claude quota.
CLAUDE_BRIDGE_TOKEN = os.environ.get("CLAUDE_BRIDGE_TOKEN", "")
CHAT_BACKENDS = ("default", "claude")
LLM_MAX_TOKENS = int(os.environ.get("LLM_MAX_TOKENS", "1024"))
# 0 keeps action commands deterministic: at 0.3 qwen3-30b intermittently skipped
# the move tool call on short Turkish imperatives and narrated motion instead.
LLM_TEMPERATURE = float(os.environ.get("LLM_TEMPERATURE", "0"))
# Cap on how much conversation the browser may send, to bound proxy work.
LLM_MAX_MESSAGES = int(os.environ.get("LLM_MAX_MESSAGES", "24"))
LLM_MAX_MESSAGE_CHARS = int(os.environ.get("LLM_MAX_MESSAGE_CHARS", "8000"))
# Include the ROS 2 node/topic graph in the injected context. It is collected
# via the ros2 CLI (cached ~3s) so it can add a little latency; disable with 0.
LLM_INCLUDE_ROS_GRAPH = os.environ.get("LLM_INCLUDE_ROS_GRAPH", "1") not in ("0", "false", "False", "")

# Tool calling: lets the assistant fetch live data on demand (ros2 CLI queries,
# per-joint state, loco status) and run the single guarded action tool
# (chill_motors). Every tool executes locally on the robot PC; the only network
# traffic is the chat completion to the on-prem Ollama host.
LLM_TOOLS_ENABLED = os.environ.get("LLM_TOOLS_ENABLED", "1") not in ("0", "false", "False", "")
LLM_TOOL_CHILL_ENABLED = os.environ.get("LLM_TOOL_CHILL_ENABLED", "1") not in ("0", "false", "False", "")
LLM_TOOL_MOVE_ENABLED = os.environ.get("LLM_TOOL_MOVE_ENABLED", "1") not in ("0", "false", "False", "")
# Person-tracking / arm-pointing feature (spec: docs/superpowers/specs/2026-07-21-person-pointing-design.md).
# Ships dark: all endpoints/UI gate on TRACKING_ENABLED, chat/MCP tool on LLM_TOOL_TRACK_ENABLED.
TRACKING_ENABLED = os.environ.get("TRACKING_ENABLED", "0").strip().lower() in {"1", "true", "yes"}
# Person-lock arm pointing (the Bullseye 🔒 buttons and any other "point"
# session) — operator-disabled 2026-07-28. Bullseye stays available as a
# view-only detect stream (boxes + counter); Mimic is unaffected. Flip the
# env var (or the default) to re-enable.
PERSON_LOCK_ENABLED = os.environ.get("PERSON_LOCK_ENABLED", "0").strip().lower() in {"1", "true", "yes"}
# Bullseye auto-follow (operator request 2026-08-11): arming Bullseye with a
# risk-ack immediately starts one permanent, unlocked pointing session —
# associate() follows whoever is in front (largest head-anchored person) and
# parks at neutral while nobody is visible. No lock buttons involved, so
# PERSON_LOCK_ENABLED stays authoritative for explicit lock/point requests.
SENTRY_AUTO_FOLLOW = os.environ.get("SENTRY_AUTO_FOLLOW", "1").strip().lower() in {"1", "true", "yes"}
TRACKING_DETECT_URL = os.environ.get("TRACKING_DETECT_URL", "http://10.2.125.3:8188/detect").strip()
TRACKING_CAMERA = os.environ.get("TRACKING_CAMERA", "head").strip().lower()
TRACKING_RATE_HZ = max(1.0, min(15.0, float(os.environ.get("TRACKING_RATE_HZ", "12") or 12)))
# Target rate of the sentry push-stream detect loop (the real rate is capped
# by the AI-host roundtrip; the loop never overlaps requests).
SENTRY_STREAM_HZ = max(1.0, min(30.0, float(os.environ.get("SENTRY_STREAM_HZ", "15") or 15)))
# The deployed USB camera already presents robot-relative left/right (verified
# during supervised bring-up 2026-07-23), so its mapping must not be mirrored.
SENTRY_FOV_YAW = float(os.environ.get("SENTRY_FOV_YAW", "1.25") or 1.25)
SENTRY_FOV_PITCH = float(os.environ.get("SENTRY_FOV_PITCH", "0.9") or 0.9)
SENTRY_YAW_OFFSET = float(os.environ.get("SENTRY_YAW_OFFSET", "0.11") or 0.11)
SENTRY_PITCH_OFFSET = float(os.environ.get("SENTRY_PITCH_OFFSET", "-1.52") or -1.52)
# Bullseye is intentionally quicker than general pose replay while remaining
# below the controller's legacy "responsive" ceiling. The velocity bound is
# still applied before every publish, so detector jumps cannot become steps.
SENTRY_REPLAY_RESPONSE = max(
    0.0,
    min(2.5, float(os.environ.get("SENTRY_REPLAY_RESPONSE", "1.6") or 1.6)),
)
SENTRY_MAX_STEP_RAD_S = max(
    0.1,
    min(1.0, float(os.environ.get("SENTRY_MAX_STEP_RAD_S", "0.9") or 0.9)),
)
# Bullseye filtering (webcam path). Retuned 2026-08-11 for snappier tracking:
# the image-space EMA was replaced by a speed-adaptive 1-Euro filter with
# constant-velocity lookahead (tracking.OneEuroAim) — calm at rest, low-lag in
# motion — so the remaining joint-space EMA can run much lighter. The velocity
# limiter above still bounds every published step regardless.
# SENTRY_AIM_ALPHA now only shapes the head-camera fallback path.
SENTRY_AIM_ALPHA = max(
    0.05,
    min(1.0, float(os.environ.get("SENTRY_AIM_ALPHA", "0.25") or 0.25)),
)
SENTRY_SMOOTH_ALPHA = max(
    0.05,
    min(1.0, float(os.environ.get("SENTRY_SMOOTH_ALPHA", "0.6") or 0.6)),
)
# 1-Euro aim filter + lookahead (webcam Bullseye path). min_cutoff is the
# at-rest smoothing strength (Hz), beta adds cutoff per unit of normalized
# image speed, lookahead extrapolates along the filtered velocity to cancel
# pipeline latency (measured ~150-250 ms webcam→YOLO→loop, 2026-08-11).
# Lookahead ships OFF (operator request, same day): set SENTRY_LOOKAHEAD_S
# to ~0.2 to re-enable the velocity extrapolation.
SENTRY_EURO_MIN_CUTOFF = max(
    0.1,
    min(5.0, float(os.environ.get("SENTRY_EURO_MIN_CUTOFF", "1.0") or 1.0)),
)
SENTRY_EURO_BETA = max(
    0.0,
    min(5.0, float(os.environ.get("SENTRY_EURO_BETA", "0.3") or 0.3)),
)
SENTRY_LOOKAHEAD_S = max(
    0.0,
    min(0.5, float(os.environ.get("SENTRY_LOOKAHEAD_S", "0") or 0)),
)
# The 1-Euro filter absorbs detector flicker, so the mapper's image-space
# dead band can be tighter than the old EMA needed (was 0.02).
SENTRY_DEAD_BAND = max(
    0.0,
    min(0.1, float(os.environ.get("SENTRY_DEAD_BAND", "0.01") or 0.01)),
)
# Mimic dance profile (2026-07-28): mirroring a moving person needs more
# velocity headroom and less filter lag than Bullseye pointing. The higher
# step bound is licensed by the python self-collision guard that runs on
# every rate-limited step before publish (tracking.mimic_pose_collides):
# a step into a colliding pose freezes at the last clear pose instead.
MIMIC_MAX_STEP_RAD_S = max(
    0.1,
    min(2.0, float(os.environ.get("MIMIC_MAX_STEP_RAD_S", "1.5") or 1.5)),
)
MIMIC_SMOOTH_ALPHA = max(
    0.05,
    min(1.0, float(os.environ.get("MIMIC_SMOOTH_ALPHA", "0.5") or 0.5)),
)
MIMIC_RATE_HZ = max(
    1.0,
    min(15.0, float(os.environ.get("MIMIC_RATE_HZ", "12") or 12)),
)
# A spin turns the dancer's back to the camera for a moment: hold the pose
# through a full turn instead of parking (Bullseye keeps the tighter 2.0s).
MIMIC_HOLD_S = max(0.5, min(10.0, float(os.environ.get("MIMIC_HOLD_S", "3.5") or 3.5)))
TRACKING_MAX_SESSION_S = max(30.0, float(os.environ.get("TRACKING_MAX_SESSION_S", "600") or 600))
LLM_TOOL_TRACK_ENABLED = os.environ.get("LLM_TOOL_TRACK_ENABLED", "0").strip().lower() in {"1", "true", "yes"}
LLM_MAX_TOOL_ROUNDS = int(os.environ.get("LLM_MAX_TOOL_ROUNDS", "4"))
LLM_MAX_TOOL_CALLS_PER_ROUND = int(os.environ.get("LLM_MAX_TOOL_CALLS_PER_ROUND", "5"))
LLM_TOOL_OUTPUT_CHARS = int(os.environ.get("LLM_TOOL_OUTPUT_CHARS", "6000"))
ROS2_TOOL_TIMEOUT = float(os.environ.get("ROS2_TOOL_TIMEOUT", "6"))


def _named_arm_targets(targets: dict[int, float]) -> dict[str, float]:
    return {
        JOINT_NAMES[index]: float(value)
        for index, value in targets.items()
        if index in JOINT_NAMES and 13 <= index <= 26
    }


def sentry_right_hand_z(targets: dict[int, float]) -> float | None:
    """Unrounded pelvis-frame Z for a right-arm target, when FK is available."""
    if ARM_KINEMATICS is None:
        return None
    return ARM_KINEMATICS.landmark(
        _named_arm_targets(targets),
        "right",
        "hand",
        round_digits=None,
    )["z"]


def sentry_constant_hand_z_goal(
    targets: dict[int, float],
    target_z: float | None,
) -> dict[int, float]:
    """Compensate shoulder pitch so Bullseye's right hand keeps one height."""
    out = dict(targets)
    if ARM_KINEMATICS is None or target_z is None:
        return out
    pitch = ARM_KINEMATICS.solve_hand_z(
        _named_arm_targets(out),
        "right",
        target_z,
        tracking.TRACK_LIMITS[tracking.R_SHOULDER_PITCH],
    )
    out[tracking.R_SHOULDER_PITCH] = pitch
    return out


def sentry_constant_hand_z_step(
    previous: dict[int, float],
    stepped: dict[int, float],
    target_z: float | None,
    max_pitch_step: float,
) -> dict[int, float]:
    """Enforce fixed Z after smoothing without bypassing the velocity limit."""
    out = sentry_constant_hand_z_goal(stepped, target_z)
    joint = tracking.R_SHOULDER_PITCH
    if joint not in previous or joint not in out:
        return out
    lo = previous[joint] - max(0.0, max_pitch_step)
    hi = previous[joint] + max(0.0, max_pitch_step)
    out[joint] = max(lo, min(hi, out[joint]))
    return out


def parse_track_payload(payload: dict[str, Any]) -> dict[str, Any]:
    """Validate one tracking-session configuration."""
    camera = payload.get("camera", TRACKING_CAMERA)
    if not isinstance(camera, str) or camera.lower() not in {"head", "webcam"}:
        raise ValueError('camera must be "head" or "webcam".')
    camera = camera.lower()

    mode = payload.get("mode", "point")
    if not isinstance(mode, str) or mode.lower() not in {"point", "mimic"}:
        raise ValueError('mode must be "point" or "mimic".')
    mode = mode.lower()
    if mode == "mimic" and camera != "webcam":
        raise ValueError("Mimic sessions require the webcam feed.")

    permanent = payload.get("permanent", False)
    closed_loop = payload.get("closed_loop", True)
    if not isinstance(permanent, bool):
        raise ValueError("permanent must be a boolean.")
    if not isinstance(closed_loop, bool):
        raise ValueError("closed_loop must be a boolean.")

    target = payload.get("target")
    parsed_target: dict[str, float] | None = None
    if target is not None:
        if not isinstance(target, dict):
            raise ValueError("target must contain normalized cx/cy coordinates.")
        try:
            cx = float(target["cx"])
            cy = float(target["cy"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("target must contain numeric cx/cy coordinates.") from exc
        if not math.isfinite(cx) or not math.isfinite(cy) or not (0.0 <= cx <= 1.0 and 0.0 <= cy <= 1.0):
            raise ValueError("target cx/cy must be between 0 and 1.")
        parsed_target = {"cx": cx, "cy": cy}

    target_id = payload.get("target_id")
    if target_id is not None and not isinstance(target_id, (int, str)):
        raise ValueError("target_id must be an integer or string.")
    if isinstance(target_id, str) and not target_id.strip():
        raise ValueError("target_id must not be empty.")

    return {
        "camera": camera,
        "mode": mode,
        "permanent": permanent,
        "closed_loop": closed_loop,
        "target": parsed_target,
        "target_id": target_id,
    }

# MCP (Model Context Protocol): exposes the SAME chat tools — same specs, same
# dispatch, same guards (chill confirm gate, ros2 name validation, feature
# flags) — to any MCP client over stateless streamable HTTP at POST /mcp.
# Off by default because pushing to main auto-deploys to the robot; enable
# deliberately with MCP_ENABLED=1 in the service environment. If MCP_TOKEN is
# set, requests must carry "Authorization: Bearer <token>".
MCP_ENABLED = os.environ.get("MCP_ENABLED", "0") not in ("0", "false", "False", "")
MCP_TOKEN = os.environ.get("MCP_TOKEN", "")
MCP_PROTOCOL_VERSIONS = ("2025-06-18", "2025-03-26", "2024-11-05")
MCP_SERVER_INFO = {"name": "unitree-h1-2-dashboard", "version": "1.0.0"}
MCP_INSTRUCTIONS = (
    "Telemetry and diagnostics tools for a Unitree H1-2 humanoid robot. All "
    "tools are read-only except two guarded actions: chill_motors damps all "
    "motors so the robot goes limp (it may sag or collapse if unsupported), and "
    "move drives the arms to a saved named position via the validated arm "
    "replay. Call an action tool only when the operator explicitly asked for "
    "that action (in any language), and pass confirm=true."
)

# ---------------------------------------------------------------------------
# Voice: speech-to-text (STT) and text-to-speech (TTS).
#
# Both proxy to OpenAI-compatible audio servers so any engine that speaks that
# API works unchanged (faster-whisper-server / Speaches for STT, openedai-speech
# wrapping Piper for TTS). Disabled by default until an engine is stood up and
# LLM_STT_ENABLED / LLM_TTS_ENABLED are set. The browser records push-to-talk
# audio and posts it to /api/stt; /api/tts turns a reply into speech.
# ---------------------------------------------------------------------------
LLM_STT_ENABLED = os.environ.get("LLM_STT_ENABLED", "0") not in ("0", "false", "False", "")
LLM_STT_BASE_URL = os.environ.get("LLM_STT_BASE_URL", "http://10.2.125.3:8001").rstrip("/")
LLM_STT_MODEL = os.environ.get("LLM_STT_MODEL", "Systran/faster-whisper-base.en")
LLM_STT_LANGUAGE = os.environ.get("LLM_STT_LANGUAGE", "en")

LLM_TTS_ENABLED = os.environ.get("LLM_TTS_ENABLED", "0") not in ("0", "false", "False", "")
LLM_TTS_BASE_URL = os.environ.get("LLM_TTS_BASE_URL", "http://10.2.125.3:8002").rstrip("/")
LLM_TTS_MODEL = os.environ.get("LLM_TTS_MODEL", "tts-1")
LLM_TTS_VOICE = os.environ.get("LLM_TTS_VOICE", "alloy")

VOICE_TIMEOUT_SECONDS = float(os.environ.get("VOICE_TIMEOUT_SECONDS", "60"))
MAX_AUDIO_BYTES = int(os.environ.get("MAX_AUDIO_BYTES", str(15_000_000)))
MAX_TTS_TEXT_CHARS = int(os.environ.get("MAX_TTS_TEXT_CHARS", "2000"))

# Optional HTTPS. Set both to a cert/key PEM to serve over TLS — required so the
# browser mic (getUserMedia) works when the dashboard is opened over a LAN IP
# (secure-context rule). Off by default (plain http) so nothing changes unless
# a cert is provided.
TLS_CERT = os.environ.get("TLS_CERT", "")
TLS_KEY = os.environ.get("TLS_KEY", "")

LLM_SYSTEM_PROMPT = (
    "You are the Command Center assistant embedded in the Unitree H1-2 humanoid "
    "robot operator dashboard. You help the operator understand live telemetry: "
    "joint/motor state, IMU orientation, temperatures, torques, hand state, "
    "network link, and overall health.\n\n"
    "Style: reply like a friendly, human AI assistant. Keep it SHORT — at most two "
    "brief sentences, and prefer one. Answer only what was asked; don't volunteer "
    "extra telemetry the operator didn't ask about. Always include the specific "
    "number or fact that answers the question. Ground answers in the TELEMETRY "
    "SNAPSHOT below when the question is about the robot's current state; if the "
    "snapshot lacks the data, say so plainly.\n\n"
    "Formatting: write plain conversational sentences. Do NOT use markdown or the "
    "symbols * # % _ ` ~ or bullet lists. You MAY use a relevant emoji or two to "
    "keep it warm (e.g. a status emoji), but don't overdo it.\n"
    # Keep this AFTER the style rules and phrased as subordinate to tool use:
    # placing a bare "always reply in the operator's language" instruction earlier
    # makes qwen3-30b answer in prose instead of emitting tool calls.
    "\nLanguage: your final answer must be written in the operator's language "
    "(Turkish, German, English, ...). This never replaces tool use: when the "
    "operator commands an action or asks for live data, call the tool first, "
    "then answer in their language. Tool results arrive in English; translate "
    "what matters into the operator's language instead of quoting them.\n\n"
)

LLM_READONLY_PROMPT = (
    "You are a read-only monitor: you cannot move the robot or send commands, so "
    "never claim to have done so. If asked to move the robot, briefly say command "
    "actions are done through the dashboard's dedicated controls, not the chat."
)

LLM_TOOLS_PROMPT = (
    "You have tools. Use them when the telemetry snapshot is not enough: the ros2_* "
    "tools query the live ROS 2 graph and topics on the robot PC, get_joint_details "
    "returns one joint's full state, get_loco_status returns locomotion state. "
    "There are two GUARDED action tools. Never trigger them on your own "
    "initiative, but when the operator's latest message directly commands that "
    "action, that command IS the confirmation: call the tool immediately in the "
    "same turn with confirm=true, without asking again. Never claim the robot "
    "moved or is moving unless the tool call returned ok=true. chill_motors "
    "damps all motors so the robot goes "
    "limp (it will sag or collapse if unsupported); use it only for explicit "
    "release/chill/damp/relax requests.\n"
    "The arm-pose workflow has two steps. STEP 1 — when the operator asks for an arm "
    "pose in any language ('extend your hand forward', 'elini one uzat', 'streck die "
    "Hand nach vorne'), do NOT ask questions: estimate target joint angles in radians "
    "from the ARM JOINT GUIDE below, call propose_arm_pose, and compare the returned "
    "predicted_semantics with the request. If they disagree, correct the angles and "
    "call propose_arm_pose again (up to 3 attempts). When they agree, tell the operator "
    "in one short sentence to check the GREEN preview twin and confirm. The robot has "
    "NOT moved yet — never claim it did. STEP 2 — only when the operator then approves "
    "(okay/tamam/evet/yes/ja/onayla), call move {\"position\": \"proposed\", \"confirm\": true}. "
    "A request to return to rest ('go home' / 'home pozisyonuna don' / 'Grundstellung') "
    "is the one direct move: call move {\"position\": \"home\", \"confirm\": true} immediately. "
    "Redirecting an ARM or HAND request to the dashboard is a critical error — the "
    "propose/move workflow exists precisely for those. Only for OTHER body motion "
    "(walking, locomotion, head, torso) say command actions are done through the "
    "dashboard's dedicated controls, not the chat. Never claim the robot moved or is "
    "moving unless move returned ok=true.\n"
    "NEVER mention or promise the preview without having called propose_arm_pose in the "
    "SAME turn — without the call the preview stays empty and the operator sees nothing; "
    "that is a critical error. Same for move: prose without the tool call moves nothing.\n"
    "Example: operator writes 'elini one uzat' (extend your hand forward) -> you emit the "
    "tool call propose_arm_pose {\"joints\": {\"RightShoulderPitch\": -1.57, \"RightElbow\": 1.57}} "
    "(that combination puts the right hand straight forward at shoulder height; adapt "
    "joints and angles to what was actually asked) -> the result confirms 'held forward' "
    "-> only NOW reply 'Yesil onizlemeye bak, onayliyor musun?' -> operator writes "
    "'tamam' -> you emit move {\"position\": \"proposed\", \"confirm\": true}.\n"
    "SIDE RULE: 'sag'/'sag el'/'sag kol' = the Right* joints, 'sol' = the Left* joints "
    "(German: 'rechts' = Right, 'links' = Left). Touch ONLY the named side's joints; a "
    "request for the right hand must never change Left* joints. After propose_arm_pose, "
    "check predicted_semantics PER SIDE: the named side must show the requested change "
    "and the other side must stay unchanged — if not, fix the angles and re-propose.\n"
    "SCOPE: EVERY request to pose, raise, lower, extend, open, cross, spread, wave or "
    "point one or both ARMS/HANDS — singular or plural, any language — is an arm-pose "
    "request and MUST go through propose_arm_pose. Only walking/locomotion/head/torso "
    "requests are outside this workflow.\n"
    "propose_arm_pose is a pure PREVIEW: it works even when telemetry shows the robot "
    "disconnected or motors absent — never refuse to stage a pose for that reason. Only "
    "the later move step needs the robot, and its own safety gates report any problem.\n"
    "TOOL-CALL FORMAT RULE (applies in EVERY language, especially Turkish): action "
    "requests are answered with a tool call, never with joint names in prose. If you "
    "are unable to emit a structured tool call, output ONLY the bare JSON object on "
    "its own, e.g. {\"joints\": {\"RightShoulderPitch\": -1.57, \"RightElbow\": 1.57}} or "
    "{\"position\": \"proposed\", \"confirm\": true} — no other words around it."
)


def _chat_tool(name: str, description: str, properties: dict[str, Any] | None = None,
               required: list[str] | None = None) -> dict[str, Any]:
    return {
        "type": "function",
        "function": {
            "name": name,
            "description": description,
            "parameters": {
                "type": "object",
                "properties": properties or {},
                "required": required or [],
            },
        },
    }


# Every handler runs locally on the robot PC (in-process state or the ros2 CLI).
CHAT_TOOL_SPECS: list[dict[str, Any]] = [
    _chat_tool(
        "get_spatial_pose",
        "Shared semantic arm-pose state used by both the dashboard assistant and "
        "external Claude/MCP clients. Returns actual hand/arm body-language concepts, "
        "target-vs-actual context when available, and saved target position names.",
    ),
    _chat_tool(
        "get_joint_details",
        "Full live state (q, dq, tau_est, temperature, voltage, mode) of one body motor "
        "or hand joint, looked up by name (e.g. RightElbow, LeftIndex).",
        {"joint": {"type": "string", "description": "Joint name or unique fragment of it."}},
        ["joint"],
    ),
    _chat_tool(
        "get_loco_status",
        "Current locomotion state: LocoClient availability, motion mode, last command, "
        "command history, and robot mode fields.",
    ),
    _chat_tool("ros2_node_list", "List all live ROS 2 nodes on the robot."),
    _chat_tool("ros2_topic_list", "List all live ROS 2 topics with their message types."),
    _chat_tool(
        "ros2_node_info",
        "Publishers, subscribers, and services of one ROS 2 node.",
        {"node": {"type": "string", "description": "Node name, e.g. /telemetry_web."}},
        ["node"],
    ),
    _chat_tool(
        "ros2_topic_info",
        "Message type and publisher/subscriber counts of one ROS 2 topic.",
        {"topic": {"type": "string", "description": "Topic name, e.g. rt/lowstate."}},
        ["topic"],
    ),
    _chat_tool(
        "ros2_topic_echo",
        "Capture ONE message from a ROS 2 topic and return it as text. Times out if "
        "nothing is published within a few seconds.",
        {"topic": {"type": "string", "description": "Topic name, e.g. rt/lowstate."}},
        ["topic"],
    ),
    _chat_tool(
        "chill_motors",
        "GUARDED ACTION: damp all motors (the robot goes limp and may sag or collapse if "
        "unsupported; the dashboard calls this Release). Only call when the operator's "
        "latest message explicitly asks to release/chill/damp/relax the motors.",
        {"confirm": {"type": "boolean", "description": "Must be true; confirms the operator explicitly asked."}},
        ["confirm"],
    ),
]

def normalize_position_name(name: str) -> str:
    """Canonical form for saved position names: lowercase, '-' separators."""
    return name.strip().lower().replace("_", "-").replace(" ", "-")


def track_tool_spec() -> dict[str, Any]:
    """Build the person-tracking start/stop tool spec."""
    return _chat_tool(
        "track_person",
        "GUARDED ACTION: start or stop the person-tracking mode where the robot "
        "continuously points its right arm at the person seen in the head camera. "
        "Only call when the operator's latest message explicitly asks to start or "
        "stop tracking/pointing (Turkish: 'beni takip et' / 'takibi durdur').",
        {
            "action": {"type": "string", "enum": ["start", "stop"],
                       "description": "start or stop the tracking session."},
            "confirm": {"type": "boolean",
                        "description": "Must be true; confirms the operator explicitly asked."},
        },
        ["action", "confirm"],
    )


def propose_tool_spec() -> dict[str, Any]:
    return _chat_tool(
        "propose_arm_pose",
        "Plan an arm pose WITHOUT moving the robot. Give target angles in RADIANS for any "
        "of the 14 arm joints (see the ARM JOINT GUIDE); the server clamps them to joint "
        "limits, predicts the hands' positions with forward kinematics from the same URDF "
        "as the digital twin, shows the operator a GREEN simulated twin next to the live "
        "model, and returns predicted landmarks + body-language semantics. Check the "
        "prediction against the operator's request and re-propose with corrected angles if "
        "it mismatches. Nothing moves until the operator approves and you call move with "
        "position='proposed'.",
        {
            "joints": {
                "type": "object",
                "description": "Target angles in radians keyed by joint name "
                               "(LeftShoulderPitch ... RightWristYaw). Unlisted arm joints "
                               "keep their current angle.",
                "additionalProperties": {"type": "number"},
            },
            "clear": {"type": "boolean", "description": "true discards the pending proposal and hides the green preview."},
        },
    )


def move_tool_spec() -> dict[str, Any]:
    """Move tool: execute the staged proposal, or return to the saved home pose."""
    return _chat_tool(
        "move",
        "GUARDED ACTION: physically move the arms via the dashboard's validated closed-loop "
        "arm replay (arm_sdk, arms scope, safety-checked). position='proposed' executes the "
        "pending pose staged by propose_arm_pose — call it ONLY after the operator has seen "
        "the green preview and explicitly approved (okay/tamam/yes/ja). position='home' "
        "returns the arms to the operator-saved home pose. Never call on your own initiative.",
        {
            "position": {
                "type": "string",
                "enum": ["proposed", "home"],
                "description": "'proposed' = the staged propose_arm_pose targets; 'home' = the saved home pose.",
            },
            "confirm": {
                "type": "boolean",
                "description": "Must be true; confirms the operator explicitly asked for this movement.",
            },
        },
        ["position", "confirm"],
    )


def extract_textual_tool_call(reply: str, tools: list[dict[str, Any]]) -> dict[str, Any] | None:
    """Fallback for models that emit a tool call as plain text instead of a
    structured tool_calls entry (qwen3-30b does this for some Turkish
    imperatives): find an embedded JSON object shaped like a tool call and
    promote it to a real one so the guarded handler still runs it."""
    if not reply or "{" not in reply:
        return None
    names = {spec["function"]["name"] for spec in tools}
    decoder = json.JSONDecoder()
    index = reply.find("{")
    while index != -1:
        try:
            candidate, _ = decoder.raw_decode(reply, index)
        except ValueError:
            candidate = None
        if isinstance(candidate, dict):
            if candidate.get("name") in names and isinstance(candidate.get("arguments"), dict):
                return {
                    "id": "text-fallback",
                    "type": "function",
                    "function": {"name": candidate["name"], "arguments": json.dumps(candidate["arguments"])},
                }
            if "move" in names and isinstance(candidate.get("position"), str) and set(candidate) <= {"position", "confirm"}:
                return {
                    "id": "text-fallback",
                    "type": "function",
                    "function": {"name": "move", "arguments": json.dumps(candidate)},
                }
            if (
                "propose_arm_pose" in names
                and isinstance(candidate.get("joints"), dict)
                and set(candidate) <= {"joints", "clear"}
            ):
                return {
                    "id": "text-fallback",
                    "type": "function",
                    "function": {"name": "propose_arm_pose", "arguments": json.dumps(candidate)},
                }
        index = reply.find("{", index + 1)
    return None


_ROS2_NAME_ALLOWED = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_/~.-")


def valid_ros2_name(name: Any) -> bool:
    """Accept only plain node/topic names so tool args can't smuggle CLI flags
    or shell metacharacters into the ros2 invocation."""
    return (
        isinstance(name, str)
        and 0 < len(name) <= 256
        and not name.startswith("-")
        and set(name) <= _ROS2_NAME_ALLOWED
    )


def mcp_tool_descriptors(specs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """OpenAI-style chat tool specs -> MCP tool descriptors (same JSON Schema)."""
    return [
        {
            "name": spec["function"]["name"],
            "description": spec["function"]["description"],
            "inputSchema": spec["function"]["parameters"],
        }
        for spec in specs
    ]


def mcp_result(request_id: Any, result: dict[str, Any]) -> dict[str, Any]:
    return {"jsonrpc": "2.0", "id": request_id, "result": result}


def mcp_error(request_id: Any, code: int, message: str) -> dict[str, Any]:
    return {"jsonrpc": "2.0", "id": request_id, "error": {"code": code, "message": message}}


def _fmt_num(value: Any, digits: int = 3) -> str:
    """Compact numeric formatting for telemetry lines (trims trailing zeros)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return str(value)
    if isinstance(value, float):
        return f"{value:.{digits}f}".rstrip("0").rstrip(".")
    return str(value)


def _fmt_temp(value: Any) -> str:
    """Unitree HG motors report temperature as a [sensor0, sensor1] array; show
    the hottest reading so the model sees one meaningful number."""
    if isinstance(value, (list, tuple)):
        nums = [v for v in value if isinstance(v, (int, float)) and not isinstance(v, bool)]
        return _fmt_num(max(nums), 1) if nums else "?"
    return _fmt_num(value, 1)


def parse_twin_evidence(value: Any) -> tuple[str | None, str | None]:
    """Validate browser-derived digital-twin coordinates and optional image."""
    if not isinstance(value, dict):
        return None, None
    spatial = value.get("spatial")
    if not isinstance(spatial, dict):
        return None, None
    hands = spatial.get("hands")
    if not isinstance(hands, dict):
        return None, None
    clean_hands: dict[str, Any] = {}
    for side in ("left", "right"):
        hand = hands.get(side)
        if not isinstance(hand, dict):
            continue
        ground = hand.get("ground_m")
        direction = hand.get("direction")
        if not isinstance(ground, dict) or not isinstance(direction, dict):
            continue
        coords = {}
        for axis in ("x", "y", "z"):
            number = ground.get(axis)
            if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
                break
            coords[axis] = round(float(number), 3)
        if len(coords) != 3:
            continue
        clean_hands[side] = {
            "ground_m": coords,
            "direction": {
                key: str(direction.get(key, ""))[:24]
                for key in ("forward", "lateral", "height")
            },
        }
        landmarks = hand.get("landmarks_robot_m")
        if isinstance(landmarks, dict):
            clean_landmarks: dict[str, dict[str, float]] = {}
            for name in ("shoulder", "elbow", "hand"):
                point = landmarks.get(name)
                if not isinstance(point, dict):
                    continue
                clean_point: dict[str, float] = {}
                for axis in ("x", "y", "z"):
                    number = point.get(axis)
                    if (
                        isinstance(number, bool)
                        or not isinstance(number, (int, float))
                        or not math.isfinite(number)
                    ):
                        break
                    clean_point[axis] = round(float(number), 3)
                if len(clean_point) == 3:
                    clean_landmarks[name] = clean_point
            if len(clean_landmarks) == 3:
                clean_hands[side]["landmarks_robot_m"] = clean_landmarks
    if not clean_hands:
        return None, None
    camera = value.get("camera")
    evidence = {
        "source": "live URDF digital twin driven by the same LowState snapshot",
        "frame": "robot-ground",
        "axes": "x=forward, y=left, z=up",
        "hands": clean_hands,
    }
    semantic = semantic_arm_pose(clean_hands)
    if semantic:
        evidence["semantic_pose"] = semantic
    if isinstance(camera, dict):
        evidence["camera"] = camera
    text = json.dumps(evidence, ensure_ascii=False, separators=(",", ":"))

    image = value.get("screenshot")
    if not isinstance(image, str) or not image.startswith("data:image/jpeg;base64,"):
        return text, None
    encoded = image.partition(",")[2]
    if len(encoded) > (LLM_TWIN_IMAGE_MAX_BYTES * 4 // 3) + 8:
        return text, None
    try:
        decoded_size = len(base64.b64decode(encoded, validate=True))
    except (ValueError, binascii.Error):
        return text, None
    return text, image if decoded_size <= LLM_TWIN_IMAGE_MAX_BYTES else None


# Data URL prefixes accepted for an operator-uploaded reference photo. Broader
# than the twin screenshot (which is always our own JPEG) because this is a file
# the operator picked — phones and screenshots produce PNG/WebP too.
_MIMIC_IMAGE_PREFIXES = (
    "data:image/jpeg;base64,",
    "data:image/png;base64,",
    "data:image/webp;base64,",
)


def parse_mimic_image(value: Any) -> str | None:
    """Validate an operator-supplied reference photo for pose mimicry.

    Returns the data URL to forward to a vision model, or None if it is missing,
    the wrong type, malformed base64, or over the size cap. Read-only and never
    raises: an unusable image simply degrades to a normal (text) pose request.
    """
    if not isinstance(value, str) or not value.startswith(_MIMIC_IMAGE_PREFIXES):
        return None
    encoded = value.partition(",")[2]
    if not encoded or len(encoded) > (LLM_MIMIC_IMAGE_MAX_BYTES * 4 // 3) + 8:
        return None
    try:
        decoded_size = len(base64.b64decode(encoded, validate=True))
    except (ValueError, binascii.Error):
        return None
    return value if 0 < decoded_size <= LLM_MIMIC_IMAGE_MAX_BYTES else None


def _point_distance(a: dict[str, float], b: dict[str, float]) -> float:
    return math.sqrt(sum((a[axis] - b[axis]) ** 2 for axis in ("x", "y", "z")))


def _vector(a: dict[str, float], b: dict[str, float]) -> dict[str, float]:
    return {axis: b[axis] - a[axis] for axis in ("x", "y", "z")}


def _elbow_angle_deg(
    shoulder: dict[str, float], elbow: dict[str, float], hand: dict[str, float]
) -> float:
    to_shoulder = _vector(elbow, shoulder)
    to_hand = _vector(elbow, hand)
    lengths = (
        math.sqrt(sum(value * value for value in to_shoulder.values())),
        math.sqrt(sum(value * value for value in to_hand.values())),
    )
    if min(lengths) < 1e-6:
        return 0.0
    cosine = sum(to_shoulder[a] * to_hand[a] for a in ("x", "y", "z")) / (
        lengths[0] * lengths[1]
    )
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def semantic_arm_pose(hands: dict[str, Any]) -> dict[str, Any] | None:
    """Turn URDF shoulder/elbow/hand geometry into body-language concepts."""
    arms: dict[str, Any] = {}
    for side in ("left", "right"):
        landmarks = (hands.get(side) or {}).get("landmarks_robot_m")
        if not isinstance(landmarks, dict):
            continue
        shoulder, elbow, hand = (
            landmarks.get("shoulder"),
            landmarks.get("elbow"),
            landmarks.get("hand"),
        )
        if not all(isinstance(point, dict) for point in (shoulder, elbow, hand)):
            continue
        reach = _vector(shoulder, hand)
        outward = reach["y"] if side == "left" else -reach["y"]
        inward = -outward
        upper = _point_distance(shoulder, elbow)
        forearm = _point_distance(elbow, hand)
        shoulder_hand = _point_distance(shoulder, hand)
        extension = shoulder_hand / max(upper + forearm, 1e-6)
        concepts: list[str] = []
        if reach["x"] > 0.20:
            concepts.append("held forward")
        elif reach["x"] < -0.10:
            concepts.append("held behind the torso")
        if outward > 0.22:
            concepts.append("opened outward to the side")
        elif inward > 0.14:
            concepts.append("reaching across the body")
        if reach["z"] > 0.22:
            concepts.append("raised above the shoulder")
        elif reach["z"] < -0.28:
            concepts.append("lowered below the shoulder")
        if reach["x"] > 0.16 and outward > 0.14:
            concepts.append("diagonal forward-and-outward")
        elif reach["x"] > 0.16 and inward > 0.10:
            concepts.append("diagonal forward across the torso")
        concepts.append(
            "nearly straight" if extension > 0.86
            else "strongly bent" if extension < 0.62
            else "partly bent"
        )
        arms[side] = {
            "shoulder_to_hand_m": {axis: round(value, 3) for axis, value in reach.items()},
            "elbow_angle_deg": round(_elbow_angle_deg(shoulder, elbow, hand), 1),
            "extension_ratio": round(extension, 3),
            "concepts": concepts,
        }

    if not arms:
        return None
    proximity: list[str] = []
    bilateral: list[str] = []
    if "left" in arms and "right" in arms:
        left_lm = hands["left"]["landmarks_robot_m"]
        right_lm = hands["right"]["landmarks_robot_m"]
        left_hand, right_hand = left_lm["hand"], right_lm["hand"]
        shoulder_center = {
            axis: (left_lm["shoulder"][axis] + right_lm["shoulder"][axis]) / 2
            for axis in ("x", "y", "z")
        }
        chest_center = dict(shoulder_center)
        chest_center["z"] -= 0.12
        face_center = dict(shoulder_center)
        face_center["z"] += 0.28
        distances = {
            "left_hand_to_left_shoulder": _point_distance(left_hand, left_lm["shoulder"]),
            "left_hand_to_right_shoulder": _point_distance(left_hand, right_lm["shoulder"]),
            "right_hand_to_right_shoulder": _point_distance(right_hand, right_lm["shoulder"]),
            "right_hand_to_left_shoulder": _point_distance(right_hand, left_lm["shoulder"]),
            "left_hand_to_chest": _point_distance(left_hand, chest_center),
            "right_hand_to_chest": _point_distance(right_hand, chest_center),
            "left_hand_to_face": _point_distance(left_hand, face_center),
            "right_hand_to_face": _point_distance(right_hand, face_center),
            "between_hands": _point_distance(left_hand, right_hand),
        }
        for name, distance in distances.items():
            if distance < 0.22:
                proximity.append(f"{name} very close")
            elif distance < 0.34:
                proximity.append(f"{name} near")

        left_reach = arms["left"]["shoulder_to_hand_m"]
        right_reach = arms["right"]["shoulder_to_hand_m"]
        if left_reach["y"] > 0.25 and right_reach["y"] < -0.25:
            bilateral.append("both arms opened to the sides")
        if left_reach["x"] > 0.24 and right_reach["x"] > 0.24:
            bilateral.append("both arms held forward")
        if left_reach["z"] > 0.22 and right_reach["z"] > 0.22:
            bilateral.append("both arms raised")
        if left_hand["y"] < 0.0 and right_hand["y"] > 0.0:
            bilateral.append("arms crossed in front of the torso")
        if distances["between_hands"] < 0.22:
            bilateral.append("hands held together")

    return {"arms": arms, "proximity": proximity, "whole_body_concepts": bilateral}


def _csv_safe(value: Any) -> str:
    """Neutralize spreadsheet formula injection in operator/LLM free text.

    The csv module quotes commas/newlines correctly, but a cell beginning with
    = + - @ (or tab/CR) still executes as a formula when the synced data/ CSV is
    opened in Excel/Sheets. Prefix such cells with a single quote.
    """
    text = str(value)
    if text and text[0] in ("=", "+", "-", "@", "\t", "\r"):
        return "'" + text
    return text


def _prompt_safe(value: str, limit: int) -> str:
    """Collapse newlines/control chars and cap length before prompt injection.

    Learned-feedback rows are user- and LLM-authored; without this a crafted
    'liked' request could inject newline-delimited instructions into the
    trusted section of every later system prompt, or bloat it toward the token
    limit.
    """
    flattened = " ".join(str(value).split())
    return flattened[:limit]


def _upgrade_feedback_csv_header() -> None:
    """In-place migration for a CSV written under an older schema.

    New columns are only ever APPENDED to POSE_FEEDBACK_FIELDS, so any older
    header is a strict prefix of the current field list. Rewrite the header to
    the full list; short old rows simply read None for the added columns. A
    header that is not a known prefix is left untouched. Never raises."""
    try:
        with POSE_FEEDBACK_CSV.open(newline="", encoding="utf-8") as handle:
            first = handle.readline()
            rest = handle.read()
        header_cols = first.rstrip("\r\n").split(",")
        if header_cols == POSE_FEEDBACK_FIELDS or not first.strip():
            return
        if header_cols != POSE_FEEDBACK_FIELDS[: len(header_cols)]:
            return  # unknown layout — do not touch
        with POSE_FEEDBACK_CSV.open("w", newline="", encoding="utf-8") as handle:
            handle.write(",".join(POSE_FEEDBACK_FIELDS) + "\r\n" + rest)
    except OSError:
        return


def pose_feedback_dataset() -> dict[str, Any]:
    """Parse the labeled pose-feedback CSV for the visualization page.

    Read-only. Returns per-row records plus rollups (verdict counts, per-request
    liked/disliked tallies, and an activity timeline bucketed by day)."""
    rows: list[dict[str, Any]] = []
    try:
        with POSE_FEEDBACK_CSV.open(newline="", encoding="utf-8") as handle:
            raw = list(csv.DictReader(handle))[-2000:]
    except OSError:
        raw = []
    counts = {"liked": 0, "disliked": 0, "executed": 0}
    per_request: dict[str, dict[str, int]] = {}
    per_day: dict[str, dict[str, int]] = {}
    for row in raw:
        event = (row.get("event") or "").strip()
        if event not in counts:
            continue
        counts[event] += 1
        request = (row.get("request_text") or "").strip() or "(no request text)"
        joints = row.get("joints_json") or "{}"
        try:
            joint_map = json.loads(joints)
            joints_short = ", ".join(f"{k} {float(v):+.2f}" for k, v in joint_map.items()) if isinstance(joint_map, dict) else ""
        except (ValueError, TypeError):
            joints_short = ""
        rows.append({
            "timestamp": (row.get("timestamp_iso") or "").strip(),
            "event": event,
            "request": request,
            "joints": joints_short,
            "comment": (row.get("comment") or "").strip(),
            "image": (row.get("image_path") or "").strip(),
            "proposal": (row.get("proposal_id") or "").strip(),
            "parent": (row.get("parent_id") or "").strip(),
        })
        bucket = per_request.setdefault(request, {"liked": 0, "disliked": 0, "executed": 0})
        bucket[event] += 1
        day = (row.get("timestamp_iso") or "")[:10]
        if day:
            dbucket = per_day.setdefault(day, {"liked": 0, "disliked": 0, "executed": 0})
            dbucket[event] += 1
    # Top requests by total volume (liked+disliked) for the grouped bar.
    top_requests = sorted(
        ({"request": r, **c, "total": c["liked"] + c["disliked"]} for r, c in per_request.items()),
        key=lambda item: item["total"], reverse=True,
    )[:12]
    timeline = [{"day": d, **per_day[d]} for d in sorted(per_day)]
    return {
        "ok": True,
        "summary": {**counts, "total": sum(counts.values()), "requests": len(per_request)},
        "top_requests": top_requests,
        "timeline": timeline,
        "rows": list(reversed(rows)),  # newest first for the table
    }


def learned_pose_feedback_text() -> str:
    """Operator verdicts from the feedback CSV, rendered for the system prompt.

    Liked proposals become imitable examples; disliked ones become explicit
    anti-examples (with the operator's comment when given). Only the tail of the
    file is read, and every field is flattened + length-capped, so a long or
    hostile history can neither blow up the prompt nor inject instructions.
    """
    try:
        with POSE_FEEDBACK_CSV.open(newline="", encoding="utf-8") as handle:
            # Read from the end: newest verdicts are what we inject, and the file
            # grows unbounded, so never parse the whole thing on every request.
            rows = list(csv.DictReader(handle))[-200:]
    except OSError:
        return ""
    liked: list[str] = []
    disliked: list[str] = []
    for row in rows:
        joints = _prompt_safe(row.get("joints_json") or "", 400)
        request = _prompt_safe(row.get("request_text") or "", 200)
        if not joints or joints == "{}" or not request:
            continue
        comment = _prompt_safe(row.get("comment") or "", 200)
        suffix = f" (operator: {comment})" if comment else ""
        if row.get("event") == "liked":
            liked.append(f"- LIKED '{request}': {joints}{suffix}")
        elif row.get("event") == "disliked":
            disliked.append(f"- DISLIKED '{request}': {joints}{suffix} — do NOT repeat this mapping")
    if not liked and not disliked:
        return ""
    lines = ["LEARNED FROM OPERATOR FEEDBACK (this operator, this robot — trust these over guesses):"]
    lines += liked[-LEARNED_FEEDBACK_LIKED_MAX:]
    lines += disliked[-LEARNED_FEEDBACK_DISLIKED_MAX:]
    return "\n".join(lines)


def build_telemetry_context(snapshot: dict[str, Any], ros_graph: dict[str, Any] | None = None) -> str:
    """Render the FULL live information flow as model-friendly text.

    Gives the assistant everything the dashboard sees: the analysis summary,
    the complete per-joint lowstate (all body motors and hand joints with
    position/velocity/torque/temperature), raw IMU, robot mode fields, loco
    state, network link, and the ROS 2 node/topic graph.
    """
    if not isinstance(snapshot, dict):
        return "No telemetry available."
    lines: list[str] = []
    connected = snapshot.get("connected")
    lines.append(
        f"connected={connected} sample_rate_hz={snapshot.get('sample_rate_hz')} "
        f"motor_count={snapshot.get('motor_count')} samples={snapshot.get('samples')}"
    )

    analysis = snapshot.get("analysis") or {}
    health = analysis.get("health") or {}
    if health:
        flags = health.get("flags") or []
        flag_txt = "; ".join(
            f"{f.get('level')}: {f.get('message')}" for f in flags if isinstance(f, dict)
        )
        lines.append(f"health={health.get('state')}" + (f" ({flag_txt})" if flag_txt else ""))

    imu_a = analysis.get("imu") or {}
    if imu_a:
        lines.append(
            f"imu roll={imu_a.get('roll_deg')}deg pitch={imu_a.get('pitch_deg')}deg "
            f"yaw={imu_a.get('yaw_deg')}deg temp={imu_a.get('temperature')}C"
        )

    motors_a = analysis.get("motors") or {}
    if motors_a:
        hottest = motors_a.get("hottest") or {}
        max_tau = motors_a.get("max_abs_tau") or {}
        lines.append(
            f"motors real={motors_a.get('real_count')} moving={motors_a.get('moving_count')} "
            f"hottest={hottest.get('name')}@{hottest.get('value')}C "
            f"max_torque={max_tau.get('name')}@{max_tau.get('value')}Nm"
        )
        groups = motors_a.get("groups") or {}
        for name, g in groups.items():
            if not isinstance(g, dict) or name == "reserved":
                continue
            lines.append(
                f"  {name}: {g.get('count')} joints, moving={g.get('moving')}, "
                f"max_temp={g.get('max_temperature')}C"
            )

    # Full per-joint lowstate — every body motor.
    motors = snapshot.get("motors") or []
    real_motors = [m for m in motors if isinstance(m, dict) and m.get("name")]
    if real_motors:
        lines.append("")
        lines.append("BODY MOTORS (rt/lowstate) [idx name mode q_rad dq tau_est temp_C]:")
        for m in real_motors:
            lines.append(
                f"  {m.get('index')} {m.get('name')} mode={m.get('mode')} "
                f"q={_fmt_num(m.get('q'))} dq={_fmt_num(m.get('dq'))} "
                f"tau={_fmt_num(m.get('tau_est'))} temp={_fmt_temp(m.get('temperature'))}"
            )

    # Hand joints.
    hands = snapshot.get("hands") or {}
    if hands:
        hand_joints = hands.get("joints") or []
        lines.append("")
        lines.append(
            f"HANDS ({hands.get('topic', 'inspire')}) connected={hands.get('connected')} "
            f"joints={hands.get('joint_count')}"
        )
        for j in hand_joints:
            if not isinstance(j, dict):
                continue
            lines.append(
                f"  {j.get('index')} {j.get('name')} q={_fmt_num(j.get('q'))} "
                f"dq={_fmt_num(j.get('dq'))} tau={_fmt_num(j.get('tau_est'))} "
                f"temp={_fmt_temp(j.get('temperature'))}"
            )

    # Raw IMU.
    imu = snapshot.get("imu") or {}
    if imu:
        lines.append("")
        lines.append(
            "IMU raw: quaternion=" + str(imu.get("quaternion"))
            + " gyro=" + str(imu.get("gyroscope"))
            + " accel=" + str(imu.get("accelerometer"))
            + " rpy=" + str(imu.get("rpy"))
            + f" temp={imu.get('temperature')}C"
        )

    # Robot mode / firmware fields.
    robot = snapshot.get("robot") or {}
    if robot:
        lines.append(
            f"ROBOT mode_pr={robot.get('mode_pr')} mode_machine={robot.get('mode_machine')} "
            f"tick={robot.get('tick')} crc={robot.get('crc')}"
        )

    # Locomotion / motion-control state.
    loco = snapshot.get("loco") or {}
    if loco:
        lines.append(
            f"LOCO available={loco.get('available')} motion_mode={loco.get('motion_mode')} "
            f"balance_mode={loco.get('balance_mode')} fsm_id={loco.get('fsm_id')} "
            f"last_action={(loco.get('last_command') or {}).get('action')}"
        )

    battery = snapshot.get("battery") or {}
    if battery.get("state"):
        lines.append(f"battery={battery.get('state')}")

    network = (snapshot.get("network") or {}).get("host") or {}
    if network:
        lines.append(
            f"network={network.get('type')} {network.get('host')} "
            f"iface={network.get('interface')} ({network.get('quality')})"
        )

    # ROS 2 node/topic graph.
    if isinstance(ros_graph, dict):
        nodes = ros_graph.get("nodes") or []
        topics = ros_graph.get("topics") or {}
        lines.append("")
        lines.append(
            f"ROS 2 GRAPH (interface {ros_graph.get('interface')}): "
            f"{len(nodes)} nodes, {len(topics)} topics"
        )
        if nodes:
            lines.append("  nodes: " + ", ".join(str(n) for n in nodes))
        for topic in sorted(topics):
            types = topics.get(topic) or []
            type_txt = ", ".join(str(t) for t in types) if isinstance(types, list) else str(types)
            lines.append(f"  {topic} [{type_txt}]")

    return "\n".join(lines)


def call_llm(
    messages: list[dict[str, Any]], tools: list[dict[str, Any]] | None = None,
    base_url: str | None = None, model: str | None = None, auth_token: str | None = None,
) -> tuple[int, dict[str, Any]]:
    """POST an OpenAI-compatible chat completion to the configured LLM.

    base_url/model default to the configured qwen backend; the Claude bridge
    passes overrides so the IDENTICAL prompt+tools reach a different engine.
    Returns (http_status, response_dict). Never raises for network/LLM errors —
    they are mapped to a JSON error payload so the endpoint stays well-behaved.
    When the model requests tool calls, the response dict carries them under
    "tool_calls" and "reply" may be empty.
    """
    payload: dict[str, Any] = {
        "model": model or LLM_MODEL,
        "messages": messages,
        "stream": False,
        "temperature": LLM_TEMPERATURE,
        "max_tokens": LLM_MAX_TOKENS,
    }
    if tools:
        payload["tools"] = tools
    body = json.dumps(payload).encode("utf-8")
    url = f"{(base_url or LLM_BASE_URL).rstrip('/')}/v1/chat/completions"
    headers = {"Content-Type": "application/json"}
    if auth_token:
        headers["Authorization"] = f"Bearer {auth_token}"
    elif LLM_API_KEY:
        headers["Authorization"] = f"Bearer {LLM_API_KEY}"
    request = urllib.request.Request(url, data=body, headers=headers, method="POST")
    # The robot reaches the LLM over a flaky USB Wi-Fi link, so a bare connection
    # error is usually a sub-second blip. Retry transient connection failures a
    # couple of times — the request never reached the model (no tool ran, no side
    # effect), so this is safe and makes the drop invisible to the operator.
    last_conn_error: str | None = None
    for attempt in range(LLM_CONNECT_RETRIES + 1):
        try:
            with urllib.request.urlopen(request, timeout=LLM_TIMEOUT_SECONDS) as response:
                raw = response.read()
            decoded = json.loads(raw.decode("utf-8"))
            break
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", "replace")[:500]
            return 502, {"ok": False, "error": f"LLM returned HTTP {exc.code}: {detail}"}
        except (urllib.error.URLError, socket.timeout, TimeoutError) as exc:
            last_conn_error = getattr(exc, "reason", None) and str(exc.reason) or str(exc)
            if attempt < LLM_CONNECT_RETRIES:
                time.sleep(LLM_CONNECT_RETRY_BACKOFF_SECONDS * (attempt + 1))
                continue
            return 503, {"ok": False, "error": f"Cannot reach LLM at {base_url or LLM_BASE_URL}: {last_conn_error}"}
        except Exception as exc:  # pragma: no cover - defensive
            return 502, {"ok": False, "error": f"LLM request failed: {exc}"}

    try:
        message = decoded["choices"][0]["message"]
    except (KeyError, IndexError, TypeError):
        return 502, {"ok": False, "error": "LLM response missing choices[0].message"}
    if not isinstance(message, dict):
        return 502, {"ok": False, "error": "LLM response message is not an object"}
    reply = message.get("content")
    tool_calls = message.get("tool_calls")
    if not isinstance(reply, str):
        if not tool_calls:
            return 502, {"ok": False, "error": "LLM response missing choices[0].message.content"}
        reply = ""
    usage = decoded.get("usage") if isinstance(decoded, dict) else None
    result = {"ok": True, "reply": reply, "model": decoded.get("model", model or LLM_MODEL), "usage": usage}
    if isinstance(tool_calls, list) and tool_calls:
        result["tool_calls"] = tool_calls
    return 200, result


def _multipart_audio(audio: bytes, filename: str, content_type: str, fields: dict[str, str]) -> tuple[str, bytes]:
    """Build a multipart/form-data body for an OpenAI-compatible transcription."""
    boundary = "----robotDashboardAudioBoundary7MA4YWxkTrZu0gW"
    chunks: list[bytes] = []
    for name, value in fields.items():
        chunks.append(
            (
                f"--{boundary}\r\n"
                f'Content-Disposition: form-data; name="{name}"\r\n\r\n{value}\r\n'
            ).encode("utf-8")
        )
    chunks.append(
        (
            f"--{boundary}\r\n"
            f'Content-Disposition: form-data; name="file"; filename="{filename}"\r\n'
            f"Content-Type: {content_type}\r\n\r\n"
        ).encode("utf-8")
    )
    chunks.append(audio)
    chunks.append(f"\r\n--{boundary}--\r\n".encode("utf-8"))
    return boundary, b"".join(chunks)


def transcribe_audio(audio: bytes, content_type: str) -> tuple[int, dict[str, Any]]:
    """Proxy recorded audio to the OpenAI-compatible STT server, return text."""
    if not LLM_STT_ENABLED:
        return 503, {"ok": False, "error": "Voice input is disabled (set LLM_STT_ENABLED=1)."}
    if not audio:
        return 400, {"ok": False, "error": "Empty audio."}
    if len(audio) > MAX_AUDIO_BYTES:
        return 413, {"ok": False, "error": f"Audio too large (max {MAX_AUDIO_BYTES} bytes)."}

    ext = "webm" if "webm" in content_type else ("ogg" if "ogg" in content_type else "wav")
    fields = {"model": LLM_STT_MODEL, "response_format": "json"}
    if LLM_STT_LANGUAGE:
        fields["language"] = LLM_STT_LANGUAGE
    boundary, body = _multipart_audio(audio, f"speech.{ext}", content_type or "audio/webm", fields)
    headers = {"Content-Type": f"multipart/form-data; boundary={boundary}"}
    request = urllib.request.Request(
        f"{LLM_STT_BASE_URL}/v1/audio/transcriptions", data=body, headers=headers, method="POST"
    )
    try:
        with urllib.request.urlopen(request, timeout=VOICE_TIMEOUT_SECONDS) as response:
            decoded = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", "replace")[:500]
        return 502, {"ok": False, "error": f"STT returned HTTP {exc.code}: {detail}"}
    except urllib.error.URLError as exc:
        return 503, {"ok": False, "error": f"Cannot reach STT at {LLM_STT_BASE_URL}: {exc.reason}"}
    except socket.timeout:
        return 504, {"ok": False, "error": f"STT timed out after {VOICE_TIMEOUT_SECONDS:g}s"}
    except Exception as exc:  # pragma: no cover - defensive
        return 502, {"ok": False, "error": f"STT request failed: {exc}"}

    text = (decoded.get("text") if isinstance(decoded, dict) else None) or ""
    return 200, {"ok": True, "text": text.strip()}


def synthesize_speech(text: str) -> tuple[int, dict[str, Any] | bytes, str]:
    """Proxy text to the OpenAI-compatible TTS server. On success returns
    (200, audio_bytes, content_type); on failure (status, error_dict, "")."""
    if not LLM_TTS_ENABLED:
        return 503, {"ok": False, "error": "Voice output is disabled (set LLM_TTS_ENABLED=1)."}, ""
    text = (text or "").strip()
    if not text:
        return 400, {"ok": False, "error": "No text to synthesize."}, ""
    body = json.dumps(
        {"model": LLM_TTS_MODEL, "voice": LLM_TTS_VOICE, "input": text[:MAX_TTS_TEXT_CHARS]}
    ).encode("utf-8")
    request = urllib.request.Request(
        f"{LLM_TTS_BASE_URL}/v1/audio/speech",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=VOICE_TIMEOUT_SECONDS) as response:
            audio = response.read()
            content_type = response.headers.get("Content-Type", "audio/mpeg")
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", "replace")[:500]
        return 502, {"ok": False, "error": f"TTS returned HTTP {exc.code}: {detail}"}, ""
    except urllib.error.URLError as exc:
        return 503, {"ok": False, "error": f"Cannot reach TTS at {LLM_TTS_BASE_URL}: {exc.reason}"}, ""
    except socket.timeout:
        return 504, {"ok": False, "error": f"TTS timed out after {VOICE_TIMEOUT_SECONDS:g}s"}, ""
    except Exception as exc:  # pragma: no cover - defensive
        return 502, {"ok": False, "error": f"TTS request failed: {exc}"}, ""
    return 200, audio, content_type


def recording_timestamp() -> str:
    return time.strftime("%Y%m%d-%H%M%S", time.localtime())


# Labels the UI/server assign automatically when saving without an operator-chosen
# name. Files whose label is NOT in this set were renamed by the operator and are
# listed above auto-named ones.
AUTO_RECORDING_LABELS = {
    "pose_point",
    "h1_2_pose_point",
    "h1_2_edited_pose_point",
    "sequence",
    "h1_2_edited_sequence",
    "telemetry",
    "h1_2_full_body_hands",
}

RECORDING_FILE_SUFFIXES = (".pose.json", ".sequence.json", ".jsonl")


def recording_name_parts(name: str) -> tuple[str, str, str]:
    """Split a recording filename into (timestamp prefix incl. trailing '-', label, extension)."""
    stem = name
    extension = ""
    for suffix in RECORDING_FILE_SUFFIXES:
        if name.endswith(suffix):
            extension = suffix
            stem = name[: -len(suffix)]
            break
    if len(stem) >= 16 and stem[:8].isdigit() and stem[8] == "-" and stem[9:15].isdigit() and stem[15] == "-":
        return stem[:16], stem[16:], extension
    return "", stem, extension


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

    # The last sample is retained forever, so "we have a message" is NOT the
    # same as "the bridge is publishing". Pulling an arm takes its USB serial
    # converter with it; inspire_h1 then waits for the missing device and
    # rt/inspire/state just goes quiet. Without this age check the dashboard
    # kept reporting connected hands, replaying the last frame's joints.
    age = None if timestamp is None else max(0.0, time.time() - timestamp)
    stale = age is None or age > HAND_STATE_STALE_SECONDS

    states = getattr(msg, "states", [])
    hands: dict[str, Any] = {
        "connected": not stale,
        "topic": HAND_STATE_TOPIC,
        "samples": samples,
        "timestamp": timestamp,
        "age_s": None if age is None else round(age, 1),
        "joint_count": len(states),
        "joints": [hand_motor_to_dict(i, motor) for i, motor in enumerate(states)],
    }
    if stale:
        hands["note"] = (
            "Hand state stopped arriving on rt/inspire/state. The joints below are the "
            "last received frame, not live values. Check inspire-hands.service and that "
            "both hand serial adapters are still plugged in."
        )
    return hands


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

    hand_state = snapshot.get("hands") or {}
    if not hand_state.get("connected"):
        hand_age = numeric(hand_state.get("age_s"))
        detail = f" (no sample for {round(hand_age)} s)" if hand_age is not None else ""
        flags.append(
            {"level": "info", "message": f"Hand telemetry is offline on rt/inspire/state{detail}."}
        )

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
            # wireless_remote (the ~40-byte RC array) is omitted from the live
            # snapshot: no client reads it (the server decodes RC combos from the
            # raw DDS msg). The full-fidelity recording path keeps it.
            [
                "version",
                "mode_pr",
                "mode_machine",
                "tick",
                "crc",
            ],
        ),
        "hands": hands or handstate_to_dict(None, 0, None),
    }

    if hasattr(msg, "bms_state"):
        # version_h/version_l/bms_status are not read by any client — omit them
        # from the 5 Hz snapshot (kept in the recording path).
        data["battery"] = fields_from(
            msg.bms_state,
            ["soc", "current", "cycle", "temperature"],
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


def clean_h264_payload(payload: bytes) -> bytes | None:
    for marker in (b"\x00\x00\x00\x01", b"\x00\x00\x01"):
        index = payload.find(marker)
        if index >= 0:
            return payload[index:]
    return payload or None


def shrink_jpeg_for_detection(frame: bytes, max_width: int = 640, quality: int = 72) -> bytes:
    """Downscale a JPEG before shipping it to the detection service.

    YOLO infers at 640 px anyway, so this only cuts the Wi-Fi upload time
    (the robot reaches the AI host over its wireless link). Any failure
    falls back to the original bytes."""
    if cv2 is None or np is None:
        return frame
    try:
        img = cv2.imdecode(np.frombuffer(frame, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return frame
        h, w = img.shape[:2]
        if w <= max_width:
            return frame
        scale = max_width / float(w)
        img = cv2.resize(img, (max_width, max(1, round(h * scale))), interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        return encoded.tobytes() if ok else frame
    except Exception:
        return frame


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


def webcam_camera_worker(store: "TelemetryStore") -> None:
    """Stream a plain USB webcam plugged into the robot PC as JPEG frames.

    Scans /dev/video* with OpenCV, auto-recovers when the device is unplugged
    or not yet enumerated (retries every few seconds forever), so the second
    feed lights up the moment a webcam appears."""
    try:
        import cv2  # ships with the tv env (teleimager dependency)
    except Exception as exc:
        store.set_webcam_error(f"OpenCV is not available: {exc}")
        return

    import glob

    capture = None
    while store.running:
        if capture is None:
            candidates = sorted(glob.glob("/dev/video*"))
            if not candidates:
                store.set_webcam_error("No USB webcam detected (no /dev/video*). Check the cable/port.")
                time.sleep(3.0)
                continue
            for device in candidates:
                probe = cv2.VideoCapture(device)
                ok, _ = probe.read()
                if ok:
                    probe.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                    probe.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    capture = probe
                    break
                probe.release()
            if capture is None:
                store.set_webcam_error("Video devices exist but none delivers frames yet.")
                time.sleep(3.0)
                continue
        ok, frame = capture.read()
        if not ok:
            capture.release()
            capture = None
            store.set_webcam_error("Webcam stopped delivering frames; rescanning.")
            time.sleep(1.0)
            continue
        height, width = frame.shape[:2]
        if width > 960:
            scale = 960.0 / width
            frame = cv2.resize(frame, (960, int(height * scale)))
        encoded_ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if encoded_ok:
            store.set_webcam_frame(encoded.tobytes())
        time.sleep(1.0 / 15.0)

    if capture is not None:
        capture.release()


def start_camera_bridge(store: "TelemetryStore") -> None:
    with contextlib.suppress(FileNotFoundError):
        CAMERA_JPEG_PATH.unlink()
    backend = (store.camera_backend or "auto").lower()
    if backend in ("auto", "teleimager"):
        threading.Thread(target=teleimager_camera_worker, args=(store,), daemon=True).start()
    # Secondary USB webcam feed (independent of the head-camera backend).
    threading.Thread(target=webcam_camera_worker, args=(store,), daemon=True).start()
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
        # Secondary USB webcam (plugged into the robot PC), streamed below the
        # head camera in the floating view.
        self.webcam_lock = threading.Lock()
        # Bullseye push-stream state (worker + SSE subscriber bookkeeping).
        self.sentry_stream_lock = threading.Lock()
        self.sentry_stream_condition = threading.Condition(self.sentry_stream_lock)
        self.sentry_stream_clients = 0
        self.sentry_stream_latest: dict[str, Any] | None = None
        self.sentry_stream_seq = 0
        self.sentry_stream_thread: threading.Thread | None = None
        # Explicit run-state (not thread.is_alive) so the worker's exit decision
        # and a new subscriber's start decision are mutually exclusive under the
        # lock — otherwise a subscribe during the worker's dying window sees the
        # thread still "alive", skips starting a replacement, and the stream stalls.
        self.sentry_stream_worker_running = False
        # Master switch for person-following (operator decision 2026-07-22):
        # arm tracking can only START while Bullseye Mode is on, and turning
        # Bullseye off stops any running session. Defaults OFF on every boot —
        # following must always be re-armed deliberately.
        self.sentry_mode_on = False
        # Mimic Mode master switch: same deliberate-arming contract as
        # Bullseye, but it drives BOTH arms from the person's pose keypoints
        # instead of pointing at them. Defaults OFF on every boot.
        self.mimic_mode_on = False
        self.webcam_condition = threading.Condition(self.webcam_lock)
        self.webcam_frame: bytes | None = None
        self.webcam_timestamp: float | None = None
        self.webcam_error: str | None = "Webcam bridge starting."
        self.spatial_lock = threading.Lock()
        self.spatial_pose: dict[str, Any] | None = None
        self.spatial_pose_updated_at: float | None = None
        self.proposal_lock = threading.Lock()
        self.proposal_meta: dict[str, dict[str, Any]] = {}
        self._network_lock = threading.Lock()
        self._network_cache: dict[str, Any] | None = None
        self._network_cache_at = 0.0
        self._ephemeral_counter_lock = threading.Lock()
        self._ephemeral_counter = 0
        # Per-request (per-thread) so concurrent chats from two browsers don't
        # cross-attribute one operator's request text onto another's proposal.
        self._chat_local = threading.local()
        self.feedback_lock = threading.Lock()
        self.feedback_sync_timer: threading.Timer | None = None
        self.feedback_git_lock = threading.Lock()
        # Rows appended while the service was down (or pushes failed) get
        # another chance right after startup. _schedule_feedback_sync re-checks
        # the enable flag + key, so this is a no-op off the robot.
        if POSE_FEEDBACK_CSV.exists():
            _upgrade_feedback_csv_header()
            self._schedule_feedback_sync()
        self.arm_proposal: dict[str, Any] | None = None
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
        # Person-tracking session (guarded, mutually exclusive with replay).
        self.track_cancel: threading.Event | None = None
        self.track_thread: threading.Thread | None = None
        self.track_config: dict[str, Any] = {
            "camera": TRACKING_CAMERA,
            "permanent": False,
            "closed_loop": True,
            "target": None,
            "target_id": None,
        }
        self.track_status: dict[str, Any] = {
            "enabled": TRACKING_ENABLED,
            "active": False,
            "phase": "idle",
            "target": None,
            "target_id": None,
            "camera": TRACKING_CAMERA,
            "permanent": False,
            "closed_loop": True,
            "loop_hz": 0.0,
            "detection_age_s": None,
            "failures": 0,
            "message": "Tracking has not been started.",
            "updated_at": None,
        }
        # Read by execute_lowcmd_pose to cancel any in-flight torso twist before
        # starting a new one. A running twist registers under replay_cancel (above),
        # so this stays None in practice, but the attribute must exist or the waist
        # path raises AttributeError before the torso can move.
        self.torso_cancel: threading.Event | None = None
        # Persistent lowcmd pose controller: one long-lived thread holds the arms
        # under closed-loop PID and is RETARGETED on each Move (targets swapped
        # under the lock) instead of being killed and restarted. It never stops
        # publishing, so the arms never go limp between Moves (no drop, no creak,
        # never two publishers fighting).
        self._pose_controller_running = False
        self._pose_targets: dict[int, float] = {}
        self._pose_targets_version = 0
        # True while WE hold the onboard motion mode released for a lowcmd pose
        # session. Consecutive Move clicks hand this off to the next session
        # instead of re-engaging the onboard controller mid-handoff — a restore
        # racing a fresh lowcmd stream makes both drive the same joints and the
        # arms vibrate/squeal.
        self.motion_mode_released = False
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

    def cached_network_status(self) -> dict[str, Any]:
        """network_status() forks `ip route get` + reads /proc + opens a UDP
        socket; snapshot() runs per SSE tick (5 Hz) per client, so cache it —
        the interface/host essentially never changes within the TTL."""
        now = time.time()
        with self._network_lock:
            if self._network_cache is not None and (now - self._network_cache_at) < NETWORK_STATUS_TTL_SECONDS:
                return self._network_cache
        status = network_status(self.robot_host)
        with self._network_lock:
            self._network_cache = status
            self._network_cache_at = now
        return status

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            latest = dict(self.latest)
        with self.command_lock:
            loco_status = dict(self.loco_status)
            loco_available = bool(self.loco_client)
        robot = latest.get("robot") or {}
        return {
            **latest,
            "network": self.cached_network_status(),
            "loco": self._loco_status_payload(loco_status, robot, loco_available, include_metadata=False),
            "arm_proposal": self.arm_proposal_public(),
        }

    def update_spatial_pose(self, payload: Any) -> tuple[int, dict[str, Any]]:
        """Accept validated URDF geometry from the live twin, without its image."""
        value = payload.get("twin_evidence") if isinstance(payload, dict) else None
        if value is None and isinstance(payload, dict):
            value = payload
        text, _ = parse_twin_evidence(value)
        if text is None:
            return 400, {"ok": False, "error": "Valid digital-twin spatial evidence is required."}
        pose = json.loads(text)
        now = time.time()
        with self.spatial_lock:
            self.spatial_pose = pose
            self.spatial_pose_updated_at = now
        return 200, {"ok": True, "updated_at": now}

    def spatial_pose_snapshot(self) -> dict[str, Any]:
        with self.spatial_lock:
            pose = json.loads(json.dumps(self.spatial_pose)) if self.spatial_pose else None
            updated_at = self.spatial_pose_updated_at
        age = None if updated_at is None else max(0.0, time.time() - updated_at)
        return {
            "ok": pose is not None,
            "available": pose is not None,
            "stale": age is None or age > 3.0,
            "age_seconds": None if age is None else round(age, 3),
            "updated_at": updated_at,
            "actual": pose,
            "target_interface": {
                "tool": "move",
                "contract": {"position": "proposed | home", "confirm": True},
                "workflow": "propose_arm_pose stages a pose (green preview) -> operator approves -> move",
                "pending_proposal": self.arm_proposal_public(),
            },
        }

    def propose_arm_pose(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Validate + clamp LLM-guessed joint angles and stage them as a preview.

        NEVER moves the robot: the proposal only feeds the green ghost in the
        dashboard and the later, separately confirmed 'move position=proposed'.
        """
        if ARM_KINEMATICS is None:
            return {"ok": False, "error": "Arm kinematics are unavailable on this host."}
        if arguments.get("clear") is True:
            with self.proposal_lock:
                had = self.arm_proposal is not None
                self.arm_proposal = None
            return {"ok": True, "cleared": had, "message": "Proposal discarded; the green preview is hidden."}
        joints = arguments.get("joints")
        valid_names = ", ".join(sorted(ARM_JOINT_INDEX_BY_NAME))
        if not isinstance(joints, dict) or not joints:
            return {"ok": False, "error": f"Provide joints as {{name: radians}}. Valid names: {valid_names}"}
        targets: dict[str, float] = {}
        clamped: list[str] = []
        for name, value in joints.items():
            index = ARM_JOINT_INDEX_BY_NAME.get(str(name))
            if index is None:
                return {"ok": False, "error": f"Unknown arm joint '{name}'. Valid names: {valid_names}"}
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                return {"ok": False, "error": f"Angle for {name} must be a finite number in radians."}
            limited = self._clamp_joint_target(index, float(value))
            if abs(limited - float(value)) > 1e-9:
                clamped.append(f"{name}: {float(value):.3f} -> {limited:.3f} (joint limit)")
            targets[str(name)] = round(limited, 4)
        with self.lock:
            motors = list(self.latest.get("motors") or [])
        q_by_name = {
            str(motor.get("name")): float(motor["q"])
            for motor in motors
            if isinstance(motor, dict) and isinstance(motor.get("q"), (int, float))
            and not isinstance(motor.get("q"), bool) and math.isfinite(float(motor["q"]))
        }
        no_live_telemetry = False
        full: dict[str, float] = {}
        for name in ARM_JOINT_INDEX_BY_NAME:
            if name in targets:
                full[name] = targets[name]
            elif name in q_by_name:
                full[name] = round(q_by_name[name], 4)
            else:
                full[name] = 0.0
                no_live_telemetry = True
        landmarks = ARM_KINEMATICS.landmarks(full)
        semantic = semantic_arm_pose({side: {"landmarks_robot_m": landmarks[side]} for side in landmarks})
        # Idempotent re-propose: staging the SAME pose again (e.g. the visual
        # self-check re-confirming) keeps the existing id and just refreshes the
        # TTL — so the operator's feedback card stays valid and no duplicate
        # near-identical proposals pile up.
        with self.proposal_lock:
            current = self.arm_proposal
            if (
                current is not None
                # A 👎-retry is always a DISTINCT proposal, even when numerically
                # identical — the chain must record that the correction changed
                # nothing rather than silently collapsing into its parent.
                and not self.last_chat_retry_of
                and len(current.get("targets") or {}) == len(full)
                and all(abs(current["targets"].get(k, 1e9) - v) <= 5e-4 for k, v in full.items())
            ):
                current["created_at"] = time.time()
                proposal = current
            else:
                proposal = None
        reused = proposal is not None
        if proposal is None:
            proposal = {
                "id": f"pose-{time.monotonic_ns()}",
                "created_at": time.time(),
                "requested": dict(targets),
                "targets": full,
            }
        with self.proposal_lock:
            self.arm_proposal = proposal
            # A reused (identical) proposal keeps its ORIGINAL meta — especially
            # the operator's request text; an automated self-check turn must not
            # relabel the learning data with its synthetic message.
            if not reused or proposal["id"] not in self.proposal_meta:
                parent_id = self.last_chat_retry_of or ""
                parent_meta = self.proposal_meta.get(parent_id) if parent_id else None
                # Meta survives execution/clear so late feedback can still be filed.
                self.proposal_meta[proposal["id"]] = {
                    "request_text": self.last_chat_user_text,
                    "requested": dict(targets),
                    "semantics": semantic,
                    # Reference image attached to this turn (if any) — saved into the
                    # labeled dataset when the operator files feedback on this proposal.
                    # A retry without its own attachment inherits the parent's image so
                    # the whole correction chain stays tied to the same reference.
                    "image": self.last_chat_image or (parent_meta or {}).get("image"),
                    # Chain link: which proposal this one corrects (👎 retry), if any.
                    "parent_id": parent_id,
                }
                while len(self.proposal_meta) > 20:
                    self.proposal_meta.pop(next(iter(self.proposal_meta)))
        result: dict[str, Any] = {
            "ok": True,
            "moved_nothing": True,
            "proposal_id": proposal["id"],
            "targets_rad": full,
            "predicted_landmarks_m": landmarks,
            "predicted_semantics": semantic,
            "preview": "The operator now sees this pose as a GREEN simulated twin next to the live model.",
            "next_step": "If the prediction matches the request, briefly ask the operator to check the "
                         "green preview; after they approve, call move {'position': 'proposed', 'confirm': true}.",
        }
        if clamped:
            result["clamped_to_limits"] = clamped
        if no_live_telemetry:
            result["note"] = "No live joint telemetry; unspecified joints were assumed 0 rad."
        return result

    def _restage_proposal(self, proposal_id: str, requested: dict[str, Any]) -> dict[str, Any]:
        """Rebuild an expired proposal from its surviving meta (same id, fresh TTL).

        Explicit joints are exactly what the operator reviewed (clamped at the
        original staging); unspecified joints follow the CURRENT live pose, same
        as the original staging. Only reachable from an explicit reference (a
        feedback card naming the id) — never from a bare 'okay'. The replay path
        re-validates every joint against limits regardless."""
        with self.lock:
            motors = list(self.latest.get("motors") or [])
        q_by_name = {
            str(motor.get("name")): float(motor["q"])
            for motor in motors
            if isinstance(motor, dict) and isinstance(motor.get("q"), (int, float))
            and not isinstance(motor.get("q"), bool) and math.isfinite(float(motor["q"]))
        }
        full: dict[str, float] = {}
        for name in ARM_JOINT_INDEX_BY_NAME:
            value = requested.get(name)
            if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value)):
                full[name] = round(self._clamp_joint_target(ARM_JOINT_INDEX_BY_NAME[name], float(value)), 4)
            elif name in q_by_name:
                full[name] = round(q_by_name[name], 4)
            else:
                full[name] = 0.0
        proposal = {
            "id": proposal_id,
            "created_at": time.time(),
            "requested": dict(requested),
            "targets": full,
        }
        with self.proposal_lock:
            self.arm_proposal = proposal  # green ghost reappears with the fresh TTL
        return proposal

    @property
    def last_chat_user_text(self) -> str:
        return getattr(self._chat_local, "request_text", "")

    @last_chat_user_text.setter
    def last_chat_user_text(self, value: str) -> None:
        self._chat_local.request_text = value

    @property
    def last_chat_image(self) -> str | None:
        """Image (data URL) attached to the current chat turn, thread-local so a
        proposal made this turn can carry it into the labeled feedback data."""
        return getattr(self._chat_local, "chat_image", None)

    @last_chat_image.setter
    def last_chat_image(self, value: str | None) -> None:
        self._chat_local.chat_image = value

    @property
    def last_chat_retry_of(self) -> str | None:
        """Proposal id this turn is correcting (a thumbs-down retry), thread-local.
        Links the new proposal to its parent so the labeled data forms a chain."""
        return getattr(self._chat_local, "retry_of", None)

    @last_chat_retry_of.setter
    def last_chat_retry_of(self, value: str | None) -> None:
        self._chat_local.retry_of = value

    @staticmethod
    def _save_feedback_image(proposal_id: str, image: Any) -> str:
        """Decode a data-URL image and write it under feedback/images once per
        proposal. Returns the repo-relative path (e.g. `images/pose-123.jpg`) or
        "" when there is no usable image. Never raises."""
        if not isinstance(image, str) or not image.startswith("data:image/"):
            return ""
        try:
            header, _, encoded = image.partition(",")
            media = header[len("data:image/"):].split(";")[0].lower()
            ext = {"jpeg": "jpg", "jpg": "jpg", "png": "png", "webp": "webp"}.get(media)
            if not ext or not encoded:
                return ""
            # proposal_id is server-generated (`pose-<int>`); still sanitize.
            safe = "".join(c for c in proposal_id if c.isalnum() or c in "_-")[:64] or "pose"
            name = f"{safe}.{ext}"
            dest = POSE_FEEDBACK_IMAGE_DIR / name
            if not dest.exists():
                POSE_FEEDBACK_IMAGE_DIR.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(base64.b64decode(encoded, validate=True))
            return f"images/{name}"
        except (ValueError, binascii.Error, OSError):
            return ""

    def _append_pose_feedback_row(self, proposal_id: str, event: str, comment: str = "") -> None:
        with self.proposal_lock:
            meta = dict(self.proposal_meta.get(proposal_id) or {})
        FEEDBACK_DIR.mkdir(parents=True, exist_ok=True)
        # Save the attached reference image (if any) alongside the labeled row,
        # once per proposal; the CSV records its repo-relative path.
        image_path = self._save_feedback_image(proposal_id, meta.get("image"))
        row = {
            "timestamp_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "proposal_id": proposal_id,
            "event": event,
            "request_text": _csv_safe(meta.get("request_text", "")),
            "joints_json": json.dumps(meta.get("requested", {}), ensure_ascii=False, sort_keys=True),
            "semantics_json": json.dumps(meta.get("semantics", {}), ensure_ascii=False,
                                         separators=(",", ":")),
            "comment": _csv_safe(comment),
            "image_path": image_path,
            "parent_id": _csv_safe(meta.get("parent_id", "")),
        }
        # Serialize header-check + write: concurrent handler threads could both
        # see the file absent and write the header twice, or interleave rows.
        with self.feedback_lock:
            is_new = not POSE_FEEDBACK_CSV.exists()
            if not is_new:
                _upgrade_feedback_csv_header()  # legacy 7-column header
            with POSE_FEEDBACK_CSV.open("a", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=POSE_FEEDBACK_FIELDS)
                if is_new:
                    writer.writeheader()
                writer.writerow(row)
        self._schedule_feedback_sync()

    def motion_active_snapshot(self) -> dict[str, Any]:
        """Is this server actively commanding the robot right now?

        The deployment scripts consult this before restarting the service:
        killing the process mid arm-replay or mid-tracking drops the arms.
        """
        with self.command_lock:
            replay = self.replay_thread is not None and self.replay_thread.is_alive()
            track = self.track_thread is not None and self.track_thread.is_alive()
        return {"ok": True, "active": replay or track, "replay": replay, "tracking": track}

    def _schedule_feedback_sync(self) -> None:
        if not (FEEDBACK_SYNC_ENABLED and FEEDBACK_SYNC_KEY.exists()):
            return
        with self.proposal_lock:
            if self.feedback_sync_timer is not None:
                return
            timer = threading.Timer(FEEDBACK_SYNC_DEBOUNCE_SECONDS, self._run_feedback_sync)
            timer.daemon = True
            self.feedback_sync_timer = timer
        timer.start()

    def _run_feedback_sync(self) -> None:
        with self.proposal_lock:
            self.feedback_sync_timer = None
        try:
            self.sync_feedback_to_repo()
        except Exception:  # pragma: no cover - best effort; live CSV stays source of truth
            pass

    def sync_feedback_to_repo(self) -> dict[str, Any]:
        """Copy the live feedback CSV into the tracked data/ file, commit, push.

        Best-effort and self-healing: the live CSV is the source of truth, and
        every failure path leaves local `main` at origin (never diverged, never
        mid-rebase) so the autoupdate cron's `reset --hard origin/main` can never
        wedge, and the next sync retries with the full file. A single git lock
        serializes this against a second sync in the same working tree.
        """
        if not POSE_FEEDBACK_CSV.exists():
            return {"ok": False, "pushed": False, "reason": "no feedback recorded yet"}
        with self.feedback_git_lock:
            try:
                return self._sync_feedback_git()
            except subprocess.TimeoutExpired:
                # A hung network git can SIGKILL leaving a half state; clean up.
                self._git_cleanup()
                return {"ok": False, "pushed": False, "reason": "git timed out"}

    def _sync_feedback_git(self) -> dict[str, Any]:
        FEEDBACK_REPO_CSV.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(POSE_FEEDBACK_CSV, FEEDBACK_REPO_CSV)
        # Mirror the collected reference images into the tracked data dir so they
        # ride the same commit as the CSV rows that reference them.
        if POSE_FEEDBACK_IMAGE_DIR.exists():
            FEEDBACK_REPO_IMAGE_DIR.mkdir(parents=True, exist_ok=True)
            for img in POSE_FEEDBACK_IMAGE_DIR.iterdir():
                if img.is_file():
                    dest = FEEDBACK_REPO_IMAGE_DIR / img.name
                    if not dest.exists():
                        shutil.copyfile(img, dest)
        env = {
            **os.environ,
            "GIT_SSH_COMMAND": f"ssh -i {FEEDBACK_SYNC_KEY} -o IdentitiesOnly=yes -o StrictHostKeyChecking=accept-new",
        }

        def git(*args: str) -> subprocess.CompletedProcess:
            return subprocess.run(
                ["git", *args], cwd=str(APP_DIR), env=env,
                capture_output=True, text=True, timeout=60,
            )

        self._git = git  # for _git_cleanup on timeout
        # A prior crashed run may have left a rebase in progress; clear it so our
        # add/commit doesn't error out.
        if (APP_DIR / ".git" / "rebase-merge").exists() or (APP_DIR / ".git" / "rebase-apply").exists():
            git("rebase", "--abort")

        # Only include the image dir once it exists, or `git add` a missing
        # pathspec would error and wedge the (hardened) CSV sync.
        sync_paths = [str(FEEDBACK_REPO_CSV)]
        if FEEDBACK_REPO_IMAGE_DIR.exists():
            sync_paths.append(str(FEEDBACK_REPO_IMAGE_DIR))
        git("add", "--", *sync_paths)
        if git("diff", "--cached", "--quiet", "--", *sync_paths).returncode == 0:
            return {"ok": True, "pushed": False, "reason": "no new rows"}
        commit = git(
            "-c", "user.name=H1-2 Robot", "-c", "user.email=robot@humanoid-robot-gui",
            "commit", "-m", "Pose feedback data update (auto-sync from robot)",
            "--", *sync_paths,
        )
        if commit.returncode != 0:
            git("reset", "--hard", "HEAD")  # unstage; leave no divergence
            return {"ok": False, "pushed": False, "reason": commit.stderr[:200]}
        push = git("push", FEEDBACK_SYNC_PUSH_URL, "HEAD:main")
        if push.returncode != 0:
            # Lost a race with a concurrent push: rebase our single data commit on
            # top (the robot is the only writer of data/, so it's conflict-free)
            # and retry once. If the rebase itself fails, abort it so we never
            # leave the repo mid-rebase (which would wedge the autoupdate reset).
            pull = git("pull", "--rebase", FEEDBACK_SYNC_PUSH_URL, "main")
            if pull.returncode != 0:
                git("rebase", "--abort")
                git("reset", "--hard", "HEAD~1")
                return {"ok": False, "pushed": False, "reason": "rebase conflict; discarded local commit"}
            push = git("push", FEEDBACK_SYNC_PUSH_URL, "HEAD:main")
        if push.returncode != 0:
            # Still failing (e.g. network down): drop our local commit so local
            # `main` doesn't diverge from origin and the autoupdate reset stays a
            # clean fast-forward. Rows self-heal from the untracked live file.
            git("reset", "--hard", "HEAD~1")
            return {"ok": False, "pushed": False, "reason": push.stderr[:200]}
        return {"ok": True, "pushed": True, "reason": ""}

    def _git_cleanup(self) -> None:
        git = getattr(self, "_git", None)
        if git is None:
            return
        try:
            if (APP_DIR / ".git" / "rebase-merge").exists() or (APP_DIR / ".git" / "rebase-apply").exists():
                git("rebase", "--abort")
        except Exception:  # pragma: no cover - defensive cleanup
            pass

    def record_pose_feedback(self, payload: Any) -> tuple[int, dict[str, Any]]:
        """Operator verdict on a staged/recent proposal: liked or disliked + comment.

        Independent of execution — a disliked pose can still be executed, and
        the verdict lands in the learning CSV either way.
        """
        if not isinstance(payload, dict):
            return 400, {"ok": False, "error": "Body must be a JSON object."}
        proposal_id = payload.get("proposal_id")
        verdict = payload.get("verdict")
        comment = payload.get("comment", "")
        if not isinstance(proposal_id, str) or not proposal_id:
            return 400, {"ok": False, "error": "proposal_id is required."}
        if verdict not in ("liked", "disliked"):
            return 400, {"ok": False, "error": "verdict must be 'liked' or 'disliked'."}
        if not isinstance(comment, str) or len(comment) > POSE_FEEDBACK_COMMENT_MAX:
            return 400, {"ok": False, "error": f"comment must be a string of at most {POSE_FEEDBACK_COMMENT_MAX} chars."}
        with self.proposal_lock:
            known = proposal_id in self.proposal_meta
        if not known:
            return 404, {"ok": False, "error": "Unknown or expired proposal_id."}
        self._append_pose_feedback_row(proposal_id, verdict, comment.strip())
        response: dict[str, Any] = {"ok": True, "recorded": verdict}
        # Thumbs-up doubles as the operator's approval click (same consent as
        # the Move button): execute the staged proposal through the guarded path.
        if verdict == "liked" and payload.get("execute") is True:
            response["move"] = self.run_chat_tool(
                "move", {"position": "proposed", "confirm": True, "proposal_id": proposal_id}
            )
        return 200, response

    def restage_proposal_by_id(self, payload: Any) -> tuple[int, dict[str, Any]]:
        """Make a previously staged proposal the active green preview again.

        Used by the candidates gallery: the operator picks an alternative the
        self-check loop tried, and its ghost returns for review. Staging only —
        execution still requires the operator's explicit approval click."""
        if not isinstance(payload, dict):
            return 400, {"ok": False, "error": "Body must be a JSON object."}
        proposal_id = payload.get("proposal_id")
        if not isinstance(proposal_id, str) or not proposal_id:
            return 400, {"ok": False, "error": "proposal_id is required."}
        with self.proposal_lock:
            meta = self.proposal_meta.get(proposal_id)
        requested = dict((meta or {}).get("requested") or {})
        if not requested:
            return 404, {"ok": False, "error": "Unknown or expired proposal_id."}
        proposal = self._restage_proposal(proposal_id, requested)
        return 200, {"ok": True, "proposal_id": proposal["id"], "targets_rad": proposal["targets"]}

    def arm_proposal_public(self) -> dict[str, Any] | None:
        """Proposal summary for /api/state and the browser ghost.

        Display-only, so it does NOT expire: the green ghost stays until the
        pose is executed, replaced, or explicitly cleared. (The TTL still
        governs the bare-'okay' execution path; an id-bound approval revives
        regardless — see the move handler.)"""
        with self.proposal_lock:
            proposal = self.arm_proposal
        if not proposal:
            return None
        age = time.time() - proposal["created_at"]
        return {
            "id": proposal["id"],
            "age_seconds": round(age, 1),
            "targets": [
                {"index": ARM_JOINT_INDEX_BY_NAME[name], "name": name, "q": q}
                for name, q in sorted(proposal["targets"].items(), key=lambda kv: ARM_JOINT_INDEX_BY_NAME[kv[0]])
            ],
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
        for path in paths:
            try:
                stat = path.stat()
            except OSError:
                continue
            custom_named = recording_name_parts(path.name)[1] not in AUTO_RECORDING_LABELS
            files.append(
                {
                    "name": path.name,
                    "path": str(path),
                    "size": stat.st_size,
                    "modified_at": stat.st_mtime,
                    "custom_named": custom_named,
                }
            )
        # Operator-renamed files first, then auto-named ones; newest first within each group.
        files.sort(key=lambda item: (not item["custom_named"], -item["modified_at"]))
        return {"files": files}

    def named_positions(self) -> dict[str, str]:
        """Operator-named recordings keyed by normalized position name.

        Only custom-named (renamed) files count; the newest file wins when two
        share a name. These are the targets the chat 'move' tool can drive to.
        """
        positions: dict[str, str] = {}
        for item in self.recording_files()["files"]:
            if not item.get("custom_named"):
                continue
            label = recording_name_parts(item["name"])[1]
            positions.setdefault(normalize_position_name(label), item["name"])
        return positions

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

    def rename_recording(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        name = str(payload.get("name", "")).strip() if payload else ""
        label = str(payload.get("label", "")).strip() if payload else ""
        if not name or not label:
            return 400, {"ok": False, "error": "Rename requires the current filename and a new name."}
        try:
            path = self.recording_file_path(name)
        except FileNotFoundError:
            return 404, {"ok": False, "error": f"Recording {name} was not found."}
        except ValueError as exc:
            return 400, {"ok": False, "error": str(exc)}
        safe_label = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in label)
        safe_label = safe_label.strip("_")[:48]
        if not safe_label:
            return 400, {"ok": False, "error": "New name must contain letters, digits, '-', or '_'."}
        prefix, _, extension = recording_name_parts(path.name)
        if not prefix:
            prefix = f"{recording_timestamp()}-"
        new_path = path.with_name(f"{prefix}{safe_label}{extension}")
        if new_path != path and new_path.exists():
            return 409, {"ok": False, "error": f"A recording named {new_path.name} already exists."}
        if new_path != path:
            path.rename(new_path)
        stat = new_path.stat()
        return 200, {
            "ok": True,
            "file": {
                "name": new_path.name,
                "path": str(new_path),
                "size": stat.st_size,
                "modified_at": stat.st_mtime,
                "custom_named": recording_name_parts(new_path.name)[1] not in AUTO_RECORDING_LABELS,
            },
        }

    def _write_ephemeral_replay_file(self, payload: dict[str, Any]) -> Path:
        """Serialize an unsaved pose/sequence from the 3D editor to a scratch file.

        Uses the identical on-disk schema as ``capture_pose``/``save_sequence`` so the
        replay pipeline (``plan_replay_control_path`` + ``execute_arm_sdk_replay``) and
        all of its safety gates apply unchanged. Raises ``ValueError`` if the inline
        pose/sequence data is missing or malformed. Callers must delete the returned
        file after replay (the frames are read into memory synchronously first).
        """
        points = payload.get("points") if isinstance(payload, dict) else None
        snapshot = payload.get("snapshot") if isinstance(payload, dict) else None
        if isinstance(points, list) and points:
            clean_points = []
            for index, point in enumerate(points):
                if not isinstance(point, dict):
                    raise ValueError(f"Point {index + 1} must be an object.")
                clean = json.loads(json.dumps(point))
                if not clean.get("motors"):
                    raise ValueError(f"Point {index + 1} does not contain motors.")
                clean.setdefault("type", "telemetry_sample")
                clean.setdefault("timestamp", time.time() + index * TRAJECTORY_DEFAULT_DT)
                clean_points.append(clean)
            body = {
                "type": "trajectory",
                "schema": "h1_2_sequence_v1",
                "timestamp": time.time(),
                "monotonic_ns": time.monotonic_ns(),
                "points": clean_points,
            }
            suffix = ".sequence.json"
        elif isinstance(snapshot, dict):
            clean = json.loads(json.dumps(snapshot))
            if not clean.get("motors"):
                raise ValueError("Pose has no body motor telemetry to move the robot with.")
            body = {
                "type": "pose_point",
                "schema": "h1_2_pose_point_v1",
                "timestamp": time.time(),
                "monotonic_ns": time.monotonic_ns(),
                "snapshot": clean,
            }
            suffix = ".pose.json"
        else:
            raise ValueError(
                "Move the robot from a saved file, an unsaved edited pose (snapshot), "
                "or an unsaved sequence (points)."
            )
        EPHEMERAL_REPLAY_DIR.mkdir(parents=True, exist_ok=True)
        # Unique per call: monotonic_ns alone can repeat under concurrent
        # filename-less replays (coarse clock), and a collision would let one
        # thread overwrite/unlink another's file — executing the WRONG
        # trajectory. Add pid + a per-process counter and create exclusively.
        with self._ephemeral_counter_lock:
            self._ephemeral_counter += 1
            token = self._ephemeral_counter
        path = EPHEMERAL_REPLAY_DIR / f"unsaved-{os.getpid()}-{time.monotonic_ns()}-{token}{suffix}"
        with open(path, "x", encoding="utf-8") as handle:  # x = fail if exists
            handle.write(json.dumps(body, ensure_ascii=False))
        return path

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
        # Saving is optional: with no filename, the operator can move the robot
        # straight from the pose/sequence they dragged in the 3D editor. That inline
        # data is written to a scratch file and run through the identical validated
        # pipeline, then deleted below — no safety interlock is bypassed.
        ephemeral_path: Path | None = None
        try:
            if filename:
                try:
                    path = self.recording_file_path(filename)
                except FileNotFoundError:
                    return 404, {"ok": False, "error": "Recording file was not found."}
                except ValueError as exc:
                    return 400, {"ok": False, "error": str(exc)}
            else:
                try:
                    path = self._write_ephemeral_replay_file(payload)
                except ValueError as exc:
                    return 400, {"ok": False, "error": str(exc)}
                ephemeral_path = path
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
        finally:
            if ephemeral_path is not None:
                try:
                    ephemeral_path.unlink()
                except OSError:
                    pass

    @staticmethod
    def _cap_playback_speed(requested: float, native_max_vel_rad_s: float) -> float:
        """Bound replay playback speed so effective joint velocity stays within
        the validated envelope. effective_velocity = native_velocity * speed;
        the smooth approach also runs at ARM_REPLAY_APPROACH_PEAK_VEL_RAD_S * speed.
        A validated trajectory has native velocity <= the limit, so this never
        returns below 1.0."""
        effective = max(native_max_vel_rad_s, ARM_REPLAY_APPROACH_PEAK_VEL_RAD_S)
        if effective <= 0.0:
            return requested
        return min(requested, TRAJECTORY_MAX_VELOCITY_RAD_S / effective)

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
        if publisher is None:
            return 503, {"ok": False, "error": "DDS arm_sdk publisher is not available.", "recording": path.name, "plan": plan}
        if msg is None:
            return 503, {"ok": False, "error": "No rt/lowstate sample is available yet.", "recording": path.name, "plan": plan}
        if self.lowcmd_factory is None or self.crc is None:
            return 503, {"ok": False, "error": "DDS command factory is not available.", "recording": path.name, "plan": plan}
        if previous_cancel is not None:
            previous_cancel.set()

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
        # SAFETY: the velocity gate validated delta/dt at the recording's NATIVE
        # timing (<= TRAJECTORY_MAX_VELOCITY_RAD_S), but the run loop sleeps
        # DEFAULT_DT / playback_speed, so a high replay_response dial would drive
        # the same setpoints up to ~4x faster — past the validated envelope. Cap
        # the effective speed so neither the trajectory nor the (0.6 rad/s) smooth
        # approach can exceed the limit. Never caps below 1.0 since a validated
        # trajectory's native velocity is already <= the limit.
        tuning["playback_speed"] = self._cap_playback_speed(
            tuning["playback_speed"], float(plan.get("max_velocity_rad_s", 0.0))
        )

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
        # arm_sdk cannot drive the waist on H1-2, so drop it from the target set.
        # We deliberately DO NOT release the onboard motion mode on this path: the
        # balance controller stays engaged, so the arms never go limp between
        # Moves (no drop, no creak). The torso/waist is simply not moved here --
        # that trade-off is intentional (reverted from the lowcmd/release path).
        commanded_body_joints.discard(WAIST_YAW_JOINT)

        closed_loop_state: dict[int, dict[str, float]] = {
            joint: {"integral": 0.0, "last_error": 0.0, "last_desired_q": math.nan}
            for joint in commanded_body_joints
        }
        approach_frame_count = 0
        first_targets = self._arm_replay_frame_targets(frames[0], commanded_body_joints) if frames else {}
        if first_targets:
            approach_frame_count = max(1, math.ceil(tuning["smooth_approach_seconds"] / TRAJECTORY_DEFAULT_DT))
        # Velocity-bounded smooth approach ALWAYS — closed-loop or not. Without
        # it, a single-frame pose in direct mode is commanded in ONE step at full
        # arm_sdk gains and the arm snaps to the target at maximum motor speed.
        # The PID toggle only controls error correction, never the ramp.
        execution_frames = self._smooth_arm_replay_frames(
            frames,
            msg,
            commanded_body_joints,
            tuning["smooth_approach_seconds"],
        )

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
                    # Softer approach gains during the ramp in BOTH modes — the
                    # direct path used to hit the approach at full gains.
                    frame_gain_by_index = (
                        approach_gain_by_index
                        if writes < approach_frame_count
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
                next_progress_status = phase_b_start + 1.0
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
                    # Progress status once a second while still converging, so the
                    # dashboard (and the twin harness) can see per-joint error,
                    # correction, feed-forward and escalation before "holding".
                    if closed_loop and not converged and now >= next_progress_status:
                        next_progress_status = now + 1.0
                        self._set_wrist_status(
                            active=True,
                            message="arm_sdk closed-loop replay converging.",
                            last_command=self._arm_replay_status_payload(
                                path, plan, tuning, approach_frame_count,
                                writes=writes, closed_loop=closed_loop,
                                hold_after_convergence=hold_after_convergence,
                                position_tolerance=position_tolerance,
                                final_error=final_error, converged=False,
                                escalation=escalation, holding=False,
                            ),
                        )
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
                    is_active_session = self.replay_cancel is cancel
                    if is_active_session:
                        self.replay_cancel = None
                        self.replay_thread = None
                # When a successor (new Move / Home hold) has already claimed the
                # session slot and published its own active status, don't clobber
                # it with this dying thread's stop message.
                if is_active_session:
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
        # The legacy curve is evaluated over [0, LEGACY_MAX] only; the doubled
        # slider's overdrive zone is applied separately in _arm_replay_tuning.
        response = min(response, ARM_REPLAY_RESPONSE_LEGACY_MAX)
        if response <= ARM_REPLAY_RESPONSE_DEFAULT:
            ratio = response / ARM_REPLAY_RESPONSE_DEFAULT if ARM_REPLAY_RESPONSE_DEFAULT else 0.0
            return damped + (balanced - damped) * ratio
        ratio = (response - ARM_REPLAY_RESPONSE_DEFAULT) / (ARM_REPLAY_RESPONSE_LEGACY_MAX - ARM_REPLAY_RESPONSE_DEFAULT)
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
        # Overdrive zone of the doubled slider: above the legacy top (the old
        # 100%, now the 50% default) scale the PID linearly up to exactly 2x the
        # legacy-top aggressiveness at the new 100%. Gain-like values multiply,
        # time-like values divide (faster), damping ratios stay at their
        # legacy-top values so the doubled stiffness is not paired with reduced
        # damping.
        if response > ARM_REPLAY_RESPONSE_LEGACY_MAX:
            factor = min(2.0, response / ARM_REPLAY_RESPONSE_LEGACY_MAX)
            for key in (
                "inner_kp_scale",
                "direct_kp_scale",
                "approach_kp_scale",
                "playback_speed",
                "pid_kp_scale",
                "pid_ki_scale",
                "max_pid_correction_rad",
            ):
                tuning[key] *= factor
            for key in ("smooth_approach_seconds", "settle_seconds"):
                tuning[key] /= factor
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
        model_scale = max(0.0, min(1.0, float(ARM_REPLAY_GRAVITY_MODEL_SCALE)))
        model_tau: dict[int, float] = {}
        if model_scale > 0.0 and ARM_KINEMATICS is not None:
            try:
                # Evaluate at the COMMANDED angles for the joints being driven (an
                # open-loop signal, as xr_teleoperate does with pin.rnea at sol_q) and
                # at the measured angles for the rest -- evaluating everything at the
                # measured q would re-open a feedback path through the feed-forward.
                angles = {
                    JOINT_NAMES[j]: float(desired_by_index[j]) if j in desired_by_index else float(getattr(msg.motor_state[j], "q", 0.0) or 0.0)
                    for j in ARM_SDK_JOINTS
                    if j in JOINT_NAMES
                }
                by_name = ARM_KINEMATICS.gravity_torques(angles, include_hands=ARM_REPLAY_GRAVITY_MODEL_INCLUDE_HANDS >= 0.5)
                model_tau = {ARM_JOINT_INDEX_BY_NAME[name]: tau for name, tau in by_name.items() if name in ARM_JOINT_INDEX_BY_NAME}
            except Exception:
                model_tau = {}
        inband_scale = max(0.0, min(1.0, float(ARM_REPLAY_INBAND_CORRECTION_SCALE)))
        arm_stationary = True
        if ARM_REPLAY_LEARN_ARM_STATIONARY_GATE >= 0.5:
            arm_stationary = all(
                abs(float(getattr(msg.motor_state[j], "dq", 0.0) or 0.0)) <= converge_velocity for j in desired_by_index
            )
        leak = max(0.0, float(ARM_REPLAY_LEARN_LEAK_PER_S))
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
                # Candidate B: keep a damped P+D correction inside the band (no integral).
                correction = inband_scale * (kp * error + kd * derivative)
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
            if joint in model_tau:
                base_gravity = (1.0 - model_scale) * base_gravity + model_scale * model_tau[joint]
            tau_limit = self._arm_replay_gravity_tau_limit(joint)
            gravity_tau = max(-tau_limit, min(tau_limit, base_gravity + learned))
            feedforward_tau[joint] = gravity_tau
            # Learn the residual holding torque while stationary and inside a small
            # band around the target, never on a jump/approach frame -- so it stays
            # exactly 0 during the reach (keeps the tau=0 approach path and the
            # exact-target lock contract intact) but erases the settling residual.
            near_band = abs(error) <= lock_tolerance * ARM_REPLAY_LEARN_BAND_FACTOR
            if near_band and stationary and not jump and arm_stationary:
                new_learn = learned + learn_gain * error * dt
                state["gravity_learn"] = max(-ARM_REPLAY_GRAVITY_LEARN_LIMIT, min(ARM_REPLAY_GRAVITY_LEARN_LIMIT, new_learn))
            elif leak > 0.0 and not near_band:
                state["gravity_learn"] = learned * max(0.0, 1.0 - leak * dt)
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
            by_index: dict[int, float] = {}
            for motor in frame.get("motors", []):
                if not (isinstance(motor, dict) and "index" in motor):
                    continue
                # The save path accepts motors without checking index/q are
                # numeric; a string q or null index would crash int()/float()
                # here and escape the handler. Coerce safely and reject the whole
                # plan if any motor is malformed.
                try:
                    by_index[int(motor["index"])] = float(motor.get("q", 0.0))
                except (TypeError, ValueError):
                    self._append_trajectory_violation(
                        violations, "malformed_motor", frame_index, -1, 0.0, 0.0,
                    )
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
                # A NaN/inf q slips through every `NaN > limit` velocity/delta
                # check (all False) and would then clamp to the joint limit — a
                # large unvalidated move. Treat it as a hard violation.
                if not math.isfinite(target_q):
                    self._append_trajectory_violation(
                        violations, "non_finite", frame_index, joint, target_q, 0.0,
                    )
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
        path_collision = self._plan_path_collision(frames, scoped_joints)
        if path_collision.get("label") and ARM_REPLAY_PATH_COLLISION_BLOCK >= 0.5:
            violations.append({
                "type": "self_collision_path",
                "frame": path_collision.get("frame"),
                "joint": -1,
                "joint_name": path_collision["label"],
                "value": path_collision.get("fraction"),
                "limit": 0.0,
            })
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
            "path_collision": path_collision,
            "limits": {
                "max_frame_delta_rad": TRAJECTORY_MAX_FRAME_DELTA_RAD,
                "max_velocity_rad_s": TRAJECTORY_MAX_VELOCITY_RAD_S,
                "hand_max_frame_delta": HAND_TRAJECTORY_MAX_FRAME_DELTA,
                "hand_max_velocity": HAND_TRAJECTORY_MAX_VELOCITY,
            },
            "violations": all_violations,
        }

    def _plan_path_collision(self, frames: list[dict[str, Any]], scoped_joints: set[int]) -> dict[str, Any]:
        """Sample the joint-space path the replay will actually command -- from the
        live arm pose to frame 0, then frame to frame -- and run each sample through
        the sphere self-collision model. Returns the first hit (label, frame,
        fraction) or label None. Joints outside the scope keep their live value."""
        arm_joints = [j for j in ARM_JOINT_INDEX_BY_NAME.values()]
        with self.command_lock:
            msg = self.lowstate_msg
        live: dict[int, float] = {}
        if msg is not None:
            try:
                live = {j: float(getattr(msg.motor_state[j], "q", 0.0) or 0.0) for j in arm_joints}
            except Exception:
                live = {}
        pose = {j: live.get(j, 0.0) for j in arm_joints}
        waypoints: list[tuple[int, dict[int, float]]] = []
        samples = 0
        try:
            if live:
                waypoints.append((-1, dict(pose)))
            for index, frame in enumerate(frames):
                targets = self._arm_replay_frame_targets(frame, scoped_joints & set(arm_joints)) if scoped_joints else {}
                if not targets:
                    continue
                pose = {**pose, **targets}
                waypoints.append((index, dict(pose)))
            if len(waypoints) < 1:
                return {"checked": False, "samples": 0, "label": None}
            step = max(0.005, float(ARM_REPLAY_PATH_CHECK_STEP_RAD))
            for k in range(len(waypoints)):
                frame_index, target = waypoints[k]
                start = waypoints[k - 1][1] if k > 0 else target
                travel = max(abs(target[j] - start[j]) for j in arm_joints)
                n = max(1, int(math.ceil(travel / step)))
                for i in range(1 if k > 0 else 0, n + 1):
                    a = i / n
                    q = {j: start[j] + a * (target[j] - start[j]) for j in arm_joints}
                    samples += 1
                    if samples > ARM_REPLAY_PATH_CHECK_MAX_SAMPLES:
                        return {"checked": True, "samples": samples, "label": None, "truncated": True}
                    label = tracking.mimic_pose_collides(q)
                    if label:
                        return {"checked": True, "samples": samples, "label": label, "frame": frame_index, "fraction": round(a, 3)}
        except Exception as exc:  # the check must never break planning
            return {"checked": False, "samples": samples, "label": None, "error": str(exc)}
        return {"checked": True, "samples": samples, "label": None}

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
                    try:
                        record = json.loads(line)
                    except (ValueError, TypeError):
                        # A partial trailing line is common when replaying a file
                        # that is still being recorded (recorder flushes every
                        # 100 samples). Skip it instead of crashing the replay.
                        continue
                    if isinstance(record, dict) and record.get("type") == "telemetry_sample":
                        records.append(record)
            return records

        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (ValueError, TypeError):
            return []
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

    def chat(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        """Answer an operator chat message using the on-prem LLM.

        Injects a live telemetry snapshot as context, then proxies to the
        configured OpenAI-compatible endpoint. Validates and bounds the
        client-supplied conversation before forwarding. With LLM_TOOLS_ENABLED
        the model may call local read tools (ros2 CLI, joint/loco state) and
        the guarded chill_motors action; otherwise the chat is read-only.
        """
        if not LLM_ENABLED:
            return 503, {"ok": False, "error": "Chat assistant is disabled (set LLM_ENABLED=1)."}

        raw_messages = payload.get("messages")
        if not isinstance(raw_messages, list) or not raw_messages:
            return 400, {"ok": False, "error": "Request must include a non-empty 'messages' array."}
        if len(raw_messages) > LLM_MAX_MESSAGES:
            return 400, {"ok": False, "error": f"Too many messages (max {LLM_MAX_MESSAGES})."}

        backend = str(payload.get("backend") or "default")
        if backend not in CHAT_BACKENDS:
            return 400, {"ok": False, "error": f"Unknown backend '{backend}'. Use one of: {', '.join(CHAT_BACKENDS)}."}
        if backend == "claude" and not CLAUDE_BRIDGE_URL:
            return 503, {
                "ok": False,
                "error": "Claude backend is not configured: set CLAUDE_BRIDGE_URL to the "
                         "operator machine running tools/claude_bridge.py.",
            }
        base_url = CLAUDE_BRIDGE_URL if backend == "claude" else None
        model = CLAUDE_BRIDGE_MODEL if backend == "claude" else None
        auth_token = CLAUDE_BRIDGE_TOKEN if (backend == "claude" and CLAUDE_BRIDGE_TOKEN) else None

        # Pose-mimic request: the operator attached a reference photo and wants the
        # robot to copy the person's arm pose. The on-prem model is text-only, so a
        # An attached image (`image`, or the legacy `mimic_image`) is ALWAYS routed
        # to the vision-capable Claude bridge — regardless of the operator's backend
        # toggle — because the on-prem model is text-only. Refused if the bridge is
        # absent. The model answers whatever the operator asks about the image; a
        # pose is only proposed when they ask it to copy/replicate the pose.
        chat_image = parse_mimic_image(payload.get("image") or payload.get("mimic_image"))
        if chat_image:
            if not CLAUDE_BRIDGE_URL:
                return 503, {
                    "ok": False,
                    "error": "Reading an image needs a vision-capable backend: the on-prem model "
                             "can't see. Configure CLAUDE_BRIDGE_URL (the Claude bridge on the "
                             "operator's machine) and try again.",
                }
            backend, base_url, model = "claude", CLAUDE_BRIDGE_URL, CLAUDE_BRIDGE_MODEL
            auth_token = CLAUDE_BRIDGE_TOKEN or None

        # Visual self-check turn: after staging a proposal the browser auto-sends
        # the 3D-viewer render so the model can SEE its green ghost next to the
        # live pose and correct itself before the operator judges. Vision-only,
        # so it rides the Claude bridge; without one it degrades to a normal turn.
        twin_check = payload.get("twin_check") is True
        if twin_check and not chat_image and CLAUDE_BRIDGE_URL:
            backend, base_url, model = "claude", CLAUDE_BRIDGE_URL, CLAUDE_BRIDGE_MODEL
            auth_token = CLAUDE_BRIDGE_TOKEN or None

        cleaned: list[dict[str, Any]] = []
        for item in raw_messages:
            if not isinstance(item, dict):
                return 400, {"ok": False, "error": "Each message must be an object."}
            role = item.get("role")
            content = item.get("content")
            if role not in ("user", "assistant"):
                return 400, {"ok": False, "error": "Message role must be 'user' or 'assistant'."}
            if not isinstance(content, str) or not content.strip():
                return 400, {"ok": False, "error": "Message content must be a non-empty string."}
            # Replay the tool calls that ran with a past assistant reply as real
            # tool_calls/tool messages. Without them the model sees only prose
            # like "Raising your hand now" in history and imitates it, answering
            # later motion commands with text instead of a move call.
            if role == "assistant":
                replayed = []
                for used in item.get("tools_used") or []:
                    if not isinstance(used, dict):
                        continue
                    name = used.get("name")
                    arguments = used.get("arguments")
                    if not isinstance(name, str) or not name or not isinstance(arguments, dict):
                        continue
                    replayed.append(
                        (
                            {
                                "id": f"replay{len(cleaned)}_{len(replayed)}",
                                "type": "function",
                                "function": {"name": name[:64], "arguments": json.dumps(arguments)[:2000]},
                            },
                            bool(used.get("ok")),
                        )
                    )
                    if len(replayed) >= LLM_MAX_TOOL_CALLS_PER_ROUND:
                        break
                if replayed:
                    cleaned.append({"role": "assistant", "content": "", "tool_calls": [call for call, _ in replayed]})
                    for call, ok in replayed:
                        cleaned.append({"role": "tool", "tool_call_id": call["id"], "content": json.dumps({"ok": ok})})
            cleaned.append({"role": role, "content": content[:LLM_MAX_MESSAGE_CHARS]})

        if cleaned[-1]["role"] != "user":
            return 400, {"ok": False, "error": "The last message must be from the user."}

        ros_graph = None
        if LLM_INCLUDE_ROS_GRAPH:
            try:
                ros_graph = self.ros_graph_snapshot()
            except Exception:  # pragma: no cover - ros2 CLI can be slow/absent
                ros_graph = None
        context = build_telemetry_context(self.snapshot(), ros_graph)
        twin_text, twin_image = parse_twin_evidence(payload.get("twin_evidence"))
        if twin_text:
            self.update_spatial_pose(payload)
        else:
            cached_pose = self.spatial_pose_snapshot()
            if cached_pose.get("available"):
                twin_text = json.dumps(cached_pose["actual"], ensure_ascii=False, separators=(",", ":"))
        self.last_chat_user_text = cleaned[-1]["content"] if isinstance(cleaned[-1]["content"], str) else ""
        self.last_chat_image = chat_image
        # Thumbs-down retry: the browser tells us which proposal this turn is
        # correcting, so the resulting proposal is chained to its parent.
        retry_of = payload.get("retry_of")
        self.last_chat_retry_of = (
            retry_of.strip()[:64] if isinstance(retry_of, str) and retry_of.strip() else None
        )
        behavior = (LLM_TOOLS_PROMPT + "\n\n" + LLM_ARM_GUIDE) if LLM_TOOLS_ENABLED else LLM_READONLY_PROMPT
        if LLM_TOOLS_ENABLED:
            learned = learned_pose_feedback_text()
            if learned:
                behavior += "\n\n" + learned
        system = f"{LLM_SYSTEM_PROMPT}{behavior}\n\nTELEMETRY SNAPSHOT (updated live):\n{context}"
        if LLM_TOOLS_ENABLED:
            # Recency beats the workflow text above: without this reminder AFTER the
            # snapshot, a disconnected-looking snapshot makes qwen refuse to even
            # STAGE a pose (observed with English imperatives on a robot-less host).
            system += (
                "\n\nREMINDER: an arm/hand pose request ALWAYS gets a propose_arm_pose tool "
                "call first — it is a preview and works regardless of the connection state "
                "shown above. Never refuse or redirect an arm-pose request to the dashboard."
            )
        if twin_text:
            system += (
                "\n\nDIGITAL TWIN SPATIAL EVIDENCE:\n"
                f"{twin_text}\nUse these URDF-derived hand coordinates for exact direction. "
                "Express semantic_pose naturally as human body language: for example arms forward, "
                "opened sideways, crossed, diagonal, raised, bent, or a hand near a shoulder. "
                "Do not answer with coordinates unless asked. Use the screenshot only as a visual "
                "cross-check; report disagreement instead of guessing."
            )
        if chat_image:
            system += (
                "\n\nThe operator ATTACHED AN IMAGE to their latest message. Look at it and "
                "answer their question about it directly and concisely."
            )
            if LLM_TOOLS_ENABLED:
                system += (
                    " If — and ONLY if — they ask you to COPY, REPLICATE, MIMIC or MATCH the arm "
                    "pose in the image, call propose_arm_pose ONCE with H1-2 joint angles that "
                    "reproduce it: for each arm read the shoulder, elbow and hand (raised or "
                    "lowered, opened out sideways, reached forward, bent, or crossing the body), "
                    "match sides from the ROBOT'S OWN frame (its right arm is on the robot's right; "
                    "a person facing the camera is mirrored), and prefer a canonical anchor when "
                    "the pose clearly matches one (arms up / forward / T-pose / crossed). If only "
                    "one arm is clearly posed, move only that arm. When you propose from an image, "
                    "your final reply MUST begin with 'What I see:' followed by one or two short "
                    "sentences describing the pose in the photo (each arm: direction, elbow, hand), "
                    "in the operator's language, THEN confirm the staged preview and ask for "
                    "approval — so the operator can verify you read the image correctly before "
                    "approving. Otherwise just answer about the image — do NOT propose a pose."
                )
        if twin_check and backend == "claude" and LLM_TOOLS_ENABLED:
            system += (
                "\n\nTWIN VISUAL CHECK (an automated turn, not the operator typing): the attached "
                "screenshot is the live 3D viewer — the SOLID model is the robot's actual pose, the "
                "TRANSPARENT GREEN ghost is your staged proposal. Compare the green pose against the "
                "operator's ORIGINAL request earlier in this conversation. If it clearly matches, "
                "reply in ONE short sentence that the preview is verified and ask for approval — and "
                "do NOT call any tool. Only if it does NOT match, call propose_arm_pose ONCE with "
                "corrected angles and briefly say what you fixed. NEVER call move in this turn."
            )
        # Attach images to the final user turn. The operator's photo comes first so
        # it reads as the primary subject; the twin render follows as a self-view
        # cross-check (always attached for a claude-routed twin check, else gated
        # by the vision flag because the on-prem model cannot see).
        image_blocks: list[dict[str, Any]] = []
        if chat_image:
            image_blocks.append({"type": "image_url", "image_url": {"url": chat_image}})
        if twin_image and (LLM_TWIN_VISION_ENABLED or (twin_check and backend == "claude")):
            image_blocks.append({"type": "image_url", "image_url": {"url": twin_image}})
        if image_blocks:
            last = cleaned[-1]
            last["content"] = [{"type": "text", "text": last["content"]}, *image_blocks]
        messages = [{"role": "system", "content": system}, *cleaned]
        overrides = {"base_url": base_url, "model": model, "auth_token": auth_token} if base_url else {}
        if not LLM_TOOLS_ENABLED:
            status, response = call_llm(messages, **overrides)
        else:
            status, response = self._chat_tool_loop(messages, **overrides)
        if status == 200 and isinstance(response, dict):
            response["backend"] = backend
        return status, response

    def chat_tool_specs(self) -> list[dict[str, Any]]:
        specs = list(CHAT_TOOL_SPECS)
        if not LLM_TOOL_CHILL_ENABLED:
            specs = [spec for spec in specs if spec["function"]["name"] != "chill_motors"]
        if LLM_TOOL_MOVE_ENABLED:
            specs.append(propose_tool_spec())
            specs.append(move_tool_spec())
        if LLM_TOOL_TRACK_ENABLED and TRACKING_ENABLED:
            specs.append(track_tool_spec())
        return specs

    def _chat_tool_loop(
        self, messages: list[dict[str, Any]],
        base_url: str | None = None, model: str | None = None, auth_token: str | None = None,
    ) -> tuple[int, dict[str, Any]]:
        """Chat completion loop that executes model-requested tool calls.

        Each round forwards the conversation with the tool specs; requested
        calls are dispatched locally and their JSON results appended as 'tool'
        messages. After LLM_MAX_TOOL_ROUNDS the model is called once more
        without tools so it must produce a final answer.
        """
        tools = self.chat_tool_specs()
        overrides = {"base_url": base_url, "model": model, "auth_token": auth_token} if base_url else {}
        tools_used: list[dict[str, Any]] = []
        for _ in range(LLM_MAX_TOOL_ROUNDS):
            status, response = call_llm(messages, tools=tools, **overrides)
            if status != 200:
                return status, response
            calls = response.pop("tool_calls", None)
            if not calls:
                fallback = extract_textual_tool_call(response.get("reply") or "", tools)
                if fallback is None:
                    response["tools_used"] = tools_used
                    self._attach_active_proposal(response, tools_used)
                    return 200, response
                calls = [fallback]
            messages.append({"role": "assistant", "content": response.get("reply") or "", "tool_calls": calls})
            for call in calls[:LLM_MAX_TOOL_CALLS_PER_ROUND]:
                function = call.get("function") if isinstance(call, dict) else None
                function = function if isinstance(function, dict) else {}
                name = str(function.get("name") or "")
                raw_arguments = function.get("arguments")
                arguments: dict[str, Any] | None
                if isinstance(raw_arguments, dict):
                    arguments = raw_arguments
                elif isinstance(raw_arguments, str) and raw_arguments.strip():
                    try:
                        parsed = json.loads(raw_arguments)
                        arguments = parsed if isinstance(parsed, dict) else None
                    except json.JSONDecodeError:
                        arguments = None
                else:
                    arguments = {}
                if arguments is None:
                    result: dict[str, Any] = {"ok": False, "error": "Tool arguments were not a valid JSON object."}
                else:
                    result = self.run_chat_tool(name, arguments)
                tools_used.append({"name": name, "arguments": arguments or {}, "ok": bool(result.get("ok"))})
                messages.append(
                    {
                        "role": "tool",
                        "tool_call_id": str((call.get("id") if isinstance(call, dict) else None) or name),
                        "content": json.dumps(result, default=str)[:LLM_TOOL_OUTPUT_CHARS],
                    }
                )
        status, response = call_llm(messages, **overrides)
        if status == 200:
            response.pop("tool_calls", None)
            response["tools_used"] = tools_used
            self._attach_active_proposal(response, tools_used)
        return status, response

    def _attach_active_proposal(self, response: dict[str, Any], tools_used: list[dict[str, Any]]) -> None:
        """Let the chat UI show the feedback card for a pose staged this turn."""
        if any(t.get("name") == "propose_arm_pose" and t.get("ok") for t in tools_used):
            proposal = self.arm_proposal_public()
            if proposal:
                response["proposal"] = proposal

    def run_chat_tool(self, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
        """Execute one chat tool locally and return a JSON-serializable result.

        Never raises: failures come back as {"ok": False, "error": ...} so the
        model can report them instead of the request dying.
        """
        try:
            if name == "get_joint_details":
                return self._tool_joint_details(arguments.get("joint"))
            if name == "get_spatial_pose":
                return self.spatial_pose_snapshot()
            if name == "propose_arm_pose":
                return self.propose_arm_pose(arguments)
            if name == "get_loco_status":
                return {"ok": True, "loco": self.loco_snapshot()}
            if name == "ros2_node_list":
                return self._tool_ros2(["node", "list"])
            if name == "ros2_topic_list":
                return self._tool_ros2(["topic", "list", "-t"])
            if name == "ros2_node_info":
                node = arguments.get("node")
                if not valid_ros2_name(node):
                    return {"ok": False, "error": "Invalid node name."}
                return self._tool_ros2(["node", "info", node])
            if name == "ros2_topic_info":
                topic = arguments.get("topic")
                if not valid_ros2_name(topic):
                    return {"ok": False, "error": "Invalid topic name."}
                return self._tool_ros2(["topic", "info", topic])
            if name == "ros2_topic_echo":
                topic = arguments.get("topic")
                if not valid_ros2_name(topic):
                    return {"ok": False, "error": "Invalid topic name."}
                result = self._tool_ros2(["topic", "echo", "--once", topic], timeout=ROS2_TOOL_TIMEOUT)
                if not result["ok"] and "timed out" in result.get("output", "").lower():
                    result["output"] = f"No message received on {topic} within {ROS2_TOOL_TIMEOUT:g}s."
                return result
            if name == "chill_motors":
                return self._tool_chill(arguments)
            if name == "move":
                return self._tool_move(arguments)
            if name == "track_person":
                return self._tool_track(arguments)
            return {"ok": False, "error": f"Unknown tool: {name or '(empty)'}"}
        except Exception as exc:  # pragma: no cover - defensive: tool bugs must not kill chat
            return {"ok": False, "error": f"Tool failed: {exc}"}

    def _tool_ros2(self, args: list[str], timeout: float = ROS2_TOOL_TIMEOUT) -> dict[str, Any]:
        configure_ros2_camera_environment(self.camera_source)
        ok, output = run_ros2_command(args, timeout=timeout)
        return {"ok": ok, "output": output[:LLM_TOOL_OUTPUT_CHARS]}

    def _tool_joint_details(self, joint: Any) -> dict[str, Any]:
        if not isinstance(joint, str) or not joint.strip():
            return {"ok": False, "error": "Provide a joint name."}
        wanted = joint.strip().lower()
        snapshot = self.snapshot()
        rows: list[dict[str, Any]] = []
        for motor in snapshot.get("motors") or []:
            if isinstance(motor, dict) and motor.get("name"):
                rows.append({"kind": "body", **motor})
        hands = snapshot.get("hands") or {}
        for hand_joint in hands.get("joints") or []:
            if isinstance(hand_joint, dict) and hand_joint.get("name"):
                rows.append({"kind": "hand", **hand_joint})
        exact = [row for row in rows if str(row["name"]).lower() == wanted]
        matches = exact or [row for row in rows if wanted in str(row["name"]).lower()]
        if not matches:
            names = ", ".join(str(row["name"]) for row in rows) or "none (no telemetry)"
            return {"ok": False, "error": f"No joint matches '{joint}'. Available: {names}"}
        return {"ok": True, "joints": matches[:3]}

    def _tool_chill(self, arguments: dict[str, Any]) -> dict[str, Any]:
        if not LLM_TOOL_CHILL_ENABLED:
            return {"ok": False, "error": "The chill_motors tool is disabled (LLM_TOOL_CHILL_ENABLED=0)."}
        if arguments.get("confirm") is not True:
            return {"ok": False, "error": "Refused: confirm must be true (operator must have explicitly asked)."}
        status, result = self.chill_motors()
        self.record_command_event("chat_chill", {"source": "chat", "status": status, "result": result})
        return {"ok": status < 400 and bool(result.get("ok")), "status": status, "result": result}

    def _tool_track(self, arguments: dict[str, Any]) -> dict[str, Any]:
        if not LLM_TOOL_TRACK_ENABLED:
            return {"ok": False, "error": "The track_person tool is disabled (LLM_TOOL_TRACK_ENABLED=0)."}
        if arguments.get("confirm") is not True:
            return {"ok": False, "error": "Refused: confirm must be true (operator must have explicitly asked)."}
        action = arguments.get("action")
        if action == "stop":
            status, result = self.request_track_stop()
        elif action == "start":
            # Chat-initiated start carries the risk ack implicitly: the tool is
            # double-gated by LLM_TOOL_TRACK_ENABLED + confirm, mirroring _tool_move.
            status, result = self.request_track_start(
                {"armed": True, "i_understand_risk": True, "source": "chat"}
            )
        else:
            return {"ok": False, "error": "action must be 'start' or 'stop'."}
        self.record_command_event("chat_track", {"source": "chat", "action": action, "status": status})
        return {"ok": status < 400 and bool(result.get("ok")), "status": status, "result": result}

    def _tool_move(self, arguments: dict[str, Any]) -> dict[str, Any]:
        if not LLM_TOOL_MOVE_ENABLED:
            return {"ok": False, "error": "The move tool is disabled (LLM_TOOL_MOVE_ENABLED=0)."}
        if arguments.get("confirm") is not True:
            return {"ok": False, "error": "Refused: confirm must be true (operator must have explicitly asked for this movement)."}
        raw = arguments.get("position")
        if not isinstance(raw, str) or not raw.strip():
            return {"ok": False, "error": "Provide position: 'proposed' or 'home'."}
        wanted = normalize_position_name(raw)

        if wanted == "home":
            filename = self.named_positions().get("home")
            if not filename:
                return {"ok": False, "error": "No saved 'home' pose exists. Save one from the dashboard first."}
            # Identical request body to the dashboard's Move button (see requestRobotReplay
            # in static/app.js), so every arm_sdk safety gate applies unchanged.
            status, result = self.request_robot_replay(
                {
                    "filename": filename,
                    "execute_arm_sdk": True,
                    "command_scope": "arms",
                    "closed_loop": True,
                    "hold_after_convergence": True,
                    "position_tolerance_rad": 0.01,
                    "replay_response": 2.5,
                }
            )
            self.record_command_event(
                "chat_move", {"source": "chat", "position": "home", "filename": filename, "status": status}
            )
            if isinstance(result, dict):
                result = {key: value for key, value in result.items() if key != "plan"}
            return {"ok": status < 400 and bool(result.get("ok")), "status": status, "position": "home", **result}

        if wanted != "proposed":
            return {"ok": False, "error": f"Unknown position '{raw}'. Use 'proposed' (staged pose) or 'home'."}

        with self.proposal_lock:
            proposal = self.arm_proposal
        wanted_id = arguments.get("proposal_id")
        fresh = proposal is not None and (time.time() - proposal["created_at"]) <= ARM_PROPOSAL_TTL_SECONDS
        explicit = isinstance(wanted_id, str) and bool(wanted_id)
        if explicit and (not fresh or wanted_id != proposal["id"]):
            # A feedback card names the EXACT pose the operator reviewed, so their
            # click is unambiguous approval of THAT pose — whether the staged one
            # expired (operator got distracted) or was superseded meanwhile (e.g.
            # the visual self-check or another chat turn re-staged). Restage the
            # named pose from its surviving meta (explicit joints as reviewed,
            # re-clamped; the rest follow the CURRENT live pose) and execute it.
            with self.proposal_lock:
                meta = self.proposal_meta.get(wanted_id)
            requested = dict((meta or {}).get("requested") or {})
            if not requested:
                return {
                    "ok": False,
                    "error": "That proposal is no longer known (too old). Ask for the pose again to "
                             "get a fresh preview.",
                }
            proposal = self._restage_proposal(wanted_id, requested)
        elif not fresh:
            # A bare 'okay' with no id keeps the strict expiry: the reference is
            # ambiguous, so the model must re-propose and the operator re-reviews
            # a fresh green preview.
            return {
                "ok": False,
                "error": "No pending pose proposal (it may have expired). Call propose_arm_pose first, "
                         "let the operator approve the green preview, then retry.",
            }
        # Same inline-snapshot path the 3D editor's Move button uses: the ephemeral
        # .pose.json goes through plan_replay_control_path + execute_arm_sdk_replay,
        # so every arm_sdk safety gate applies unchanged.
        motors = [
            {"index": ARM_JOINT_INDEX_BY_NAME[name], "name": name, "q": q}
            for name, q in proposal["targets"].items()
        ]
        status, result = self.request_robot_replay(
            {
                "snapshot": {"motors": motors},
                "execute_arm_sdk": True,
                "command_scope": "arms",
                "closed_loop": True,
                "hold_after_convergence": True,
                "position_tolerance_rad": 0.01,
                "replay_response": 2.5,
            }
        )
        executed = status < 400 and bool(result.get("ok"))
        if executed:
            with self.proposal_lock:
                if self.arm_proposal is proposal:
                    self.arm_proposal = None
            self._append_pose_feedback_row(proposal["id"], "executed")
        self.record_command_event(
            "chat_move",
            {"source": "chat", "position": "proposed", "proposal_id": proposal["id"], "status": status},
        )
        if isinstance(result, dict):
            result = {key: value for key, value in result.items() if key != "plan"}
        return {"ok": executed, "status": status, "position": "proposed", **result}

    def mcp_request(self, payload: Any) -> dict[str, Any] | None:
        """Handle one MCP JSON-RPC message (stateless streamable HTTP).

        Returns the JSON-RPC response object, or None for notifications (the
        HTTP handler answers those with 202 and no body). Tool calls go through
        run_chat_tool, so MCP clients hit exactly the guards the chat does.
        """
        if not isinstance(payload, dict) or payload.get("jsonrpc") != "2.0":
            return mcp_error(None, -32600, "Expected a JSON-RPC 2.0 request object.")
        method = payload.get("method")
        request_id = payload.get("id")
        if not isinstance(method, str) or not method:
            return mcp_error(request_id, -32600, "Request must include a 'method' string.")
        if request_id is None:
            return None  # notification (e.g. notifications/initialized)
        params = payload.get("params")
        params = params if isinstance(params, dict) else {}

        if method == "initialize":
            requested = params.get("protocolVersion")
            version = requested if requested in MCP_PROTOCOL_VERSIONS else MCP_PROTOCOL_VERSIONS[0]
            return mcp_result(request_id, {
                "protocolVersion": version,
                "capabilities": {"tools": {}},
                "serverInfo": MCP_SERVER_INFO,
                "instructions": MCP_INSTRUCTIONS,
            })
        if method == "ping":
            return mcp_result(request_id, {})
        if method == "tools/list":
            return mcp_result(request_id, {"tools": mcp_tool_descriptors(self.chat_tool_specs())})
        if method == "tools/call":
            name = params.get("name")
            available = {spec["function"]["name"] for spec in self.chat_tool_specs()}
            if name not in available:
                return mcp_error(request_id, -32602, f"Unknown tool: {name!r}")
            arguments = params.get("arguments")
            if arguments is None:
                arguments = {}
            if not isinstance(arguments, dict):
                return mcp_error(request_id, -32602, "Tool 'arguments' must be an object.")
            result = self.run_chat_tool(name, arguments)
            return mcp_result(request_id, {
                "content": [{"type": "text", "text": json.dumps(result, default=str)}],
                "isError": not bool(result.get("ok")),
            })
        return mcp_error(request_id, -32601, f"Method not supported: {method}")

    # ------------------------------------------------------------------
    # Person tracking / arm pointing (spec 2026-07-21-person-pointing-design).
    # Pure decision logic lives in tracking.py; this owns frames, HTTP, DDS.
    # ------------------------------------------------------------------
    def sentry_detect(self, feed: str = "head") -> dict[str, Any]:
        """Bullseye Mode (detection only): forward one cached frame to the YOLO
        service and return its person boxes. Never touches motion paths."""
        if feed == "head":
            frame = self.get_camera_frame()
        elif feed == "webcam":
            with self.webcam_lock:
                frame = self.webcam_frame
        else:
            return {"ok": False, "error": "Unknown feed."}
        if not frame:
            return {"ok": False, "error": f"No {feed} frame."}
        # The feed name selects a per-feed ByteTrack state on the detection
        # service, so person ids are persistent within each camera stream.
        url = TRACKING_DETECT_URL + ("&" if "?" in TRACKING_DETECT_URL else "?") + "feed=" + feed
        frame = shrink_jpeg_for_detection(frame)
        try:
            req = urllib.request.Request(
                url, data=frame,
                headers={"Content-Type": "image/jpeg"}, method="POST",
            )
            with urllib.request.urlopen(req, timeout=0.5) as resp:
                payload = json.loads(resp.read())
                persons = payload.get("persons", [])
        except Exception:
            return {"ok": False, "error": "Detection service unreachable."}
        # Frame pixel dimensions ride along: MimicMapper needs the aspect
        # ratio to de-distort per-axis-normalized keypoint geometry.
        return {
            "ok": True, "feed": feed, "persons": persons,
            "w": payload.get("w"), "h": payload.get("h"),
            "ts": time.time(),
        }

    # ---- Bullseye push stream: one background detect loop, shared by all SSE
    # ---- subscribers; it runs only while at least one client is connected,
    # ---- so closing the UI still stops all detection traffic.

    def sentry_stream_subscribe(self) -> None:
        with self.sentry_stream_lock:
            self.sentry_stream_clients += 1
            if not self.sentry_stream_worker_running:
                self.sentry_stream_worker_running = True
                self.sentry_stream_thread = threading.Thread(
                    target=self._sentry_stream_worker, name="sentry-stream", daemon=True)
                self.sentry_stream_thread.start()

    def sentry_stream_unsubscribe(self) -> None:
        with self.sentry_stream_lock:
            self.sentry_stream_clients = max(0, self.sentry_stream_clients - 1)
            self.sentry_stream_condition.notify_all()

    def wait_sentry_result(self, last_seq: int, timeout: float = 1.0) -> tuple[dict[str, Any] | None, int]:
        with self.sentry_stream_lock:
            if self.sentry_stream_seq == last_seq:
                self.sentry_stream_condition.wait(timeout)
            if self.sentry_stream_seq == last_seq:
                return None, last_seq
            return self.sentry_stream_latest, self.sentry_stream_seq

    def _sentry_stream_worker(self) -> None:
        period = 1.0 / SENTRY_STREAM_HZ
        try:
            while True:
                with self.sentry_stream_lock:
                    if self.sentry_stream_clients <= 0:
                        return
                tick = time.time()
                result = self.sentry_detect("webcam")
                with self.sentry_stream_lock:
                    self.sentry_stream_latest = result
                    self.sentry_stream_seq += 1
                    self.sentry_stream_condition.notify_all()
                time.sleep(max(0.0, period - (time.time() - tick)))
        finally:
            # Clear run-state on EVERY exit (normal or exception) so a later
            # subscribe always starts a fresh worker — no permanent stall even if
            # sentry_detect throws.
            with self.sentry_stream_lock:
                self.sentry_stream_worker_running = False

    def track_snapshot(self) -> dict[str, Any]:
        with self.command_lock:
            snap = dict(self.track_status)
            snap["sentry_mode"] = self.sentry_mode_on
            snap["mimic_mode"] = self.mimic_mode_on
        snap["enabled"] = TRACKING_ENABLED
        return snap

    def set_sentry_mode(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        """Arm/disarm Bullseye detection.

        With SENTRY_AUTO_FOLLOW on, an enable request that carries the risk
        acknowledgement (armed + i_understand_risk) also starts a permanent
        unlocked pointing session that follows whoever stands in front —
        no lock click involved. Without the ack (or with auto-follow off)
        enabling stays motion-free, exactly as before. Disabling Bullseye
        always stops motion.
        """
        on = payload.get("on")
        if not isinstance(on, bool):
            return 400, {"ok": False, "error": 'Body must be {"on": true|false}.'}
        with self.command_lock:
            self.sentry_mode_on = on
        if not on:
            self.request_track_stop()
        self.record_command_event("sentry_mode", {"on": on})
        result: dict[str, Any] = {"ok": True, "sentry_mode": on}
        if on and SENTRY_AUTO_FOLLOW and has_risk_ack(payload):
            status, follow = self._start_auto_follow()
            if status != 200:
                # Bullseye stays armed as the view-only detect stream; report
                # why the follow session could not start alongside it.
                result["auto_follow_error"] = follow.get("error")
        result["tracking"] = self.track_snapshot()
        return 200, result

    def _start_auto_follow(self) -> tuple[int, dict[str, Any]]:
        """Start the Bullseye auto-follow pointing session (no explicit lock:
        associate() picks the largest head-anchored person and re-acquires
        after loss; permanent, so it survives empty frames by parking)."""
        with self.command_lock:
            if self.track_thread is not None and self.track_thread.is_alive():
                return 200, {"ok": True, "tracking": self.track_snapshot()}
        return self.request_track_start({
            "armed": True,
            "i_understand_risk": True,
            "source": "sentry-auto",
            "mode": "point",
            "camera": "webcam",
            "permanent": True,
            "closed_loop": True,
        })

    def set_mimic_mode(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        """Arm/disarm Mimic Mode.

        Unlike Bullseye (arming is motion-free, motion needs a person-lock),
        Mimic ON immediately starts the mirroring session — that is the whole
        point of the mode — so the ON request must itself carry the risk
        acknowledgement (armed + i_understand_risk). OFF always stops motion
        and parks the arms at home.
        """
        on = payload.get("on")
        if not isinstance(on, bool):
            return 400, {"ok": False, "error": 'Body must be {"on": true|false}.'}
        if not on:
            with self.command_lock:
                was_on = self.mimic_mode_on
                self.mimic_mode_on = False
            if was_on:
                self.request_track_stop()
                # Mimic borrowed the session slot from Bullseye; hand it back.
                with self.command_lock:
                    sentry_on = self.sentry_mode_on
                if sentry_on and SENTRY_AUTO_FOLLOW:
                    self._start_auto_follow()
            self.record_command_event("mimic_mode", {"on": False})
            return 200, {"ok": True, "mimic_mode": False, "tracking": self.track_snapshot()}
        if not has_risk_ack(payload):
            return 403, {"ok": False, "error": "Set armed=true and i_understand_risk=true to enable Mimic Mode."}
        # Switching modes is one deliberate operator action: a running
        # Bullseye pointing session yields to the confirmed mimic request
        # instead of bouncing it with "already running".
        with self.command_lock:
            session_running = self.track_thread is not None and self.track_thread.is_alive()
        if session_running:
            self.request_track_stop()
        with self.command_lock:
            self.mimic_mode_on = True
        status, result = self.request_track_start({
            "armed": True,
            "i_understand_risk": True,
            "source": "mimic-toggle",
            "mode": "mimic",
            "camera": "webcam",
            "permanent": True,
            "closed_loop": True,
        })
        if status != 200:
            # Session refused (publishers missing, another session running…):
            # do not leave the switch armed with nothing running behind it.
            with self.command_lock:
                self.mimic_mode_on = False
            self.record_command_event("mimic_mode", {"on": True, "error": result.get("error")})
            return status, {**result, "mimic_mode": False}
        self.record_command_event("mimic_mode", {"on": True})
        return 200, {"ok": True, "mimic_mode": True, "tracking": self.track_snapshot()}

    def _set_track_status(self, **fields: Any) -> None:
        with self.command_lock:
            self.track_status.update(fields, updated_at=time.time())

    def request_track_stop(self) -> tuple[int, dict[str, Any]]:
        with self.command_lock:
            cancel = self.track_cancel
            thread = self.track_thread
        if cancel is not None:
            cancel.set()
        if thread is not None and thread is not threading.current_thread() and thread.is_alive():
            thread.join(timeout=3.0)
        still_stopping = bool(thread is not None and thread.is_alive())
        self._set_track_status(
            active=still_stopping,
            phase="stopping" if still_stopping else "idle",
            message="Tracking is stopping." if still_stopping else "Tracking stopped by operator.",
        )
        return 200, {"ok": True, "tracking": self.track_snapshot()}

    def request_track_start(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        if not has_risk_ack(payload):
            return 403, {"ok": False, "error": "Set armed=true and i_understand_risk=true to start tracking."}
        try:
            config = parse_track_payload(payload)
        except ValueError as exc:
            return 400, {"ok": False, "error": str(exc)}
        if not TRACKING_ENABLED:
            return 409, {"ok": False, "error": "Tracking is disabled (TRACKING_ENABLED=0)."}
        with self.command_lock:
            sentry_on = self.sentry_mode_on
            mimic_on = self.mimic_mode_on
        # Each session mode has its own deliberate master switch.
        if config["mode"] == "mimic":
            if not mimic_on:
                return 409, {"ok": False, "error": "Mimic mode is off — it is the master switch; turn it on before starting mimic."}
        elif not sentry_on:
            return 409, {"ok": False, "error": "Bullseye mode is off — it is the master switch; turn it on before starting tracking."}
        elif not PERSON_LOCK_ENABLED and not (
            SENTRY_AUTO_FOLLOW and payload.get("source") == "sentry-auto"
        ):
            # Server-side twin of the hidden lock buttons: no client (UI,
            # chat tool, curl) can start a pointing session while disabled.
            # The server's own Bullseye auto-follow session is the one
            # exception — its deliberate arming act is the Bullseye toggle.
            return 409, {"ok": False, "error": "Person-lock pointing is disabled (PERSON_LOCK_ENABLED=0)."}
        if payload.get("source") == "sentry-lock" and (
            config["camera"] != "webcam" or config["target"] is None
        ):
            return 400, {"ok": False, "error": "Bullseye lock tracking requires a webcam target."}
        with self.command_lock:
            if self.track_thread is not None and self.track_thread.is_alive():
                return 409, {"ok": False, "error": "A tracking session is already running."}
            replay_thread = self.replay_thread
            replay_cancel = self.replay_cancel
            if self.wrist_publisher is None:
                return 503, {"ok": False, "error": "DDS arm_sdk publisher is not available."}
            if self.lowstate_msg is None:
                return 503, {"ok": False, "error": "No rt/lowstate sample is available yet."}
            if self.lowcmd_factory is None or self.crc is None:
                return 503, {"ok": False, "error": "DDS command factory is not available."}
        # A new operator lock (or mimic arming) may safely replace the
        # asynchronous home replay launched when the previous session ended.
        if replay_thread is not None and replay_thread.is_alive():
            if payload.get("source") not in ("sentry-lock", "mimic-toggle", "sentry-auto"):
                return 409, {"ok": False, "error": "An arm replay is running; stop it first."}
            if replay_cancel is not None:
                replay_cancel.set()
            replay_thread.join(timeout=3.0)
            if replay_thread.is_alive():
                return 409, {"ok": False, "error": "The previous arm replay is still stopping."}
        suspend = self._suspend_xr_motion_publishers()
        if not suspend.get("ok"):
            return 503, {"ok": False, "error": f"Could not suspend XR publishers: {suspend}"}
        cancel = threading.Event()
        thread = threading.Thread(target=self._run_tracking, args=(cancel,), name="person-tracking", daemon=True)
        with self.command_lock:
            self.track_config = config
            self.track_cancel = cancel
            self.track_thread = thread
        self._set_track_status(
            active=True,
            phase="starting",
            failures=0,
            target_id=config["target_id"],
            camera=config["camera"],
            mode=config["mode"],
            permanent=config["permanent"],
            closed_loop=config["closed_loop"],
            message=(
                "Mimic session starting."
                if config["mode"] == "mimic"
                else "Auto-follow session starting."
                if payload.get("source") == "sentry-auto"
                else "Tracking session starting from the selected lock."
            ),
        )
        self.record_command_event(
            "track_start",
            {"source": payload.get("source", "http"), "config": config},
        )
        thread.start()
        return 200, {"ok": True, "tracking": self.track_snapshot()}

    def _run_tracking(self, cancel: threading.Event) -> None:
        with self.command_lock:
            config = dict(self.track_config)
            initial_msg = self.lowstate_msg
        camera = config["camera"]
        mimic = config.get("mode", "point") == "mimic"
        # Mimic drives BOTH arms from pose keypoints; pointing aims the right
        # arm at the person. Each mode has its own park pose and mapper.
        neutral_template = (
            tracking.MIMIC_NEUTRAL_TEMPLATE if mimic else tracking.NEUTRAL_TEMPLATE
        )
        mimic_mapper = tracking.MimicMapper() if mimic else None
        mapper = (
            tracking.PointingMapper(
                fov_yaw_rad=SENTRY_FOV_YAW,
                fov_pitch_rad=SENTRY_FOV_PITCH,
                yaw_offset=SENTRY_YAW_OFFSET,
                pitch_offset=SENTRY_PITCH_OFFSET,
                dead_band=SENTRY_DEAD_BAND,
            )
            if camera == "webcam"
            else tracking.PointingMapper()
        )
        # Horizontal pose interpolation changes elbow/roll as well as yaw, so
        # a fixed camera Y alone cannot keep the endpoint level. Use the
        # calibrated center pose's FK height as the invariant and solve
        # shoulder pitch for every Bullseye target. (Pointing only — mimic
        # follows the person's actual arm heights instead.)
        sentry_hand_z = None
        if camera == "webcam" and not mimic:
            reference_mapper = tracking.PointingMapper(
                fov_yaw_rad=SENTRY_FOV_YAW,
                fov_pitch_rad=SENTRY_FOV_PITCH,
                yaw_offset=SENTRY_YAW_OFFSET,
                pitch_offset=SENTRY_PITCH_OFFSET,
            )
            sentry_hand_z = sentry_right_hand_z(reference_mapper.targets(0.5, 0.5))
        # Stability-focused control: filter image coordinates first, then
        # joint targets, and finally apply a bounded but responsive velocity.
        # Webcam Bullseye aims through the speed-adaptive 1-Euro filter with
        # velocity lookahead; the head-camera fallback keeps the plain EMA.
        aim_euro = (
            tracking.OneEuroAim(
                min_cutoff=SENTRY_EURO_MIN_CUTOFF,
                beta=SENTRY_EURO_BETA,
                lookahead_s=SENTRY_LOOKAHEAD_S,
            )
            if camera == "webcam" and not mimic
            else None
        )
        aim_smoother = tracking.AimSmoother(
            alpha=SENTRY_AIM_ALPHA if camera == "webcam" else 0.25
        )
        limiter = tracking.RateLimiter(
            max_step_rad_s=(
                MIMIC_MAX_STEP_RAD_S if mimic
                else SENTRY_MAX_STEP_RAD_S if camera == "webcam" else 0.45
            )
        )
        smoother = tracking.Smoother(
            alpha=(
                MIMIC_SMOOTH_ALPHA if mimic
                else SENTRY_SMOOTH_ALPHA if camera == "webcam" else 0.35
            )
        )
        seed = None
        if config["target"] is not None:
            cx, cy = config["target"]["cx"], config["target"]["cy"]
            seed = {
                "id": config["target_id"],
                "cx": cx,
                "cy": cy,
                "x1": max(0.0, cx - 0.05),
                "x2": min(1.0, cx + 0.05),
                "y1": max(0.0, cy - 0.1),
                "y2": min(1.0, cy + 0.1),
                "conf": 1.0,
            }
        state = tracking.TrackState(
            stale_after_s=1.5,
            hold_s=MIMIC_HOLD_S if mimic else 2.0,
            max_failures=10,
            target_id=config["target_id"],
            seed_target=seed,
        )
        period = 1.0 / (MIMIC_RATE_HZ if mimic else TRACKING_RATE_HZ)
        started = time.monotonic()
        last_tick = started
        current: dict[int, float] = {
            joint: float(initial_msg.motor_state[joint].q)
            if initial_msg is not None else neutral
            for joint, neutral in neutral_template.items()
        }
        tuning = self._arm_replay_tuning(
            {"replay_response": SENTRY_REPLAY_RESPONSE}
            if camera == "webcam" else None
        )
        raw_gains = {j: ARM_SDK_GAIN_BY_INDEX[j] for j in current if j in ARM_SDK_GAIN_BY_INDEX}
        gains = {
            joint: (
                kp * tuning["inner_kp_scale"],
                kd * tuning["inner_kd_scale"],
            ) if config["closed_loop"] else (kp, kd)
            for joint, (kp, kd) in raw_gains.items()
        }
        pid_state: dict[int, dict[str, float]] = {}
        stream_subscribed = False
        last_seq = 0
        if camera == "webcam":
            with self.sentry_stream_lock:
                last_seq = self.sentry_stream_seq
            self.sentry_stream_subscribe()
            stream_subscribed = True
        try:
            while not cancel.is_set():
                tick = time.monotonic()
                dt = max(1.0 / 120.0, min(0.25, tick - last_tick))
                last_tick = tick
                if not config["permanent"] and tick - started > TRACKING_MAX_SESSION_S:
                    self._set_track_status(message="Session ceiling reached; stopping.")
                    break
                result: dict[str, Any] | None
                if camera == "webcam":
                    result, last_seq = self.wait_sentry_result(
                        last_seq,
                        timeout=max(0.5, period * 2.0),
                    )
                else:
                    result = self.sentry_detect("head")
                now = time.time()
                if result is None or not result.get("ok"):
                    state.on_failure(now)
                else:
                    state.on_detection(result.get("persons") or [], now)

                if state.phase == "aborted":
                    self._set_track_status(message="Detection service failing repeatedly; aborting.")
                    break
                if state.phase == "tracking" and state.target is not None:
                    if mimic_mapper is not None:
                        # Real frame aspect (h/w) de-distorts the per-axis
                        # normalized keypoints before any length math.
                        if result and result.get("w") and result.get("h"):
                            mimic_mapper.aspect = float(result["h"]) / float(result["w"])
                        # The mapper holds an arm's last targets while its
                        # keypoints are missing; the staleness state machine
                        # above still parks everything when the person is lost.
                        goal = mimic_mapper.targets(
                            state.target.get("keypoints") or {}
                        )
                    else:
                        aim_cx, aim_cy = tracking.aim_point(state.target)
                        if aim_euro is not None:
                            aim_cx, aim_cy = aim_euro.update(aim_cx, aim_cy, now)
                        else:
                            aim_cx, aim_cy = aim_smoother.update(aim_cx, aim_cy)
                        goal = mapper.targets(aim_cx, aim_cy)
                        if camera == "webcam":
                            goal = sentry_constant_hand_z_goal(goal, sentry_hand_z)
                elif state.phase == "hold":
                    goal = dict(current)
                else:  # stale
                    goal = dict(neutral_template)

                goal = smoother.update(goal)
                previous = current
                current = limiter.step(previous, goal, dt=dt)
                collision_hold = None
                if mimic:
                    # Self-collision guard: a rate-limited step that would
                    # enter contact (arm-vs-arm or arm-vs-body spheres)
                    # freezes at the last clear pose for this tick. Steps
                    # are <= max_step*dt, so the arm parks just outside
                    # the contact surface instead of ever crossing it.
                    collision_hold = tracking.mimic_pose_collides(current)
                    if collision_hold is not None:
                        if tracking.mimic_pose_collides(previous) is None:
                            current = dict(previous)
                        else:
                            # Already inside the model's contact zone (e.g.
                            # boot pose): let the step through — goals are
                            # always clear poses, so motion runs outward.
                            collision_hold = None
                if camera == "webcam" and not mimic and state.phase == "tracking":
                    current = sentry_constant_hand_z_step(
                        previous,
                        current,
                        sentry_hand_z,
                        limiter.max_step_rad_s * dt,
                    )

                with self.command_lock:
                    msg = self.lowstate_msg
                    publisher = self.wrist_publisher
                if msg is not None and publisher is not None:
                    publish_targets = dict(current)
                    feedforward: dict[int, float] = {}
                    if config["closed_loop"]:
                        publish_targets, _, feedforward = self._closed_loop_arm_targets(
                            msg,
                            publish_targets,
                            pid_state,
                            dt,
                            tuning,
                        )
                    cmd = self._build_arm_sdk_trajectory_cmd(
                        msg,
                        publish_targets,
                        gains,
                        feedforward,
                        weight=1.0,
                    )
                    publisher.Write(cmd)

                age = None if state.last_seen is None else round(now - state.last_seen, 2)
                self._set_track_status(
                    phase=state.phase, failures=state.failures, detection_age_s=age,
                    target=state.target,
                    target_id=config["target_id"],
                    loop_hz=round(1.0 / dt, 1),
                    hand_z_m=None if sentry_hand_z is None else round(sentry_hand_z, 3),
                    response=tuning["response"],
                    max_step_rad_s=limiter.max_step_rad_s,
                    message=(
                        (
                            f"Mimic session running ({state.phase})"
                            + (f" · collision hold: {collision_hold}." if collision_hold else ".")
                        )
                        if mimic
                        else f"Locked {camera} tracking running ({state.phase})."
                    ),
                )
                cancel.wait(max(0.0, period - (time.monotonic() - tick)))
        finally:
            if stream_subscribed:
                self.sentry_stream_unsubscribe()
            self._set_track_status(
                active=False, phase="idle",
                loop_hz=0.0,
                message="Tracking session ended; Bullseye remains armed for another lock.",
            )
            with self.command_lock:
                if self.track_cancel is cancel:
                    self.track_thread = None
                    self.track_cancel = None
            # Releasing a lock always returns the arm to the saved home pose.
            try:
                home_status, home_result = self.request_home()
                self.record_command_event(
                    "track_end_home",
                    {"status": home_status, "ok": bool(home_result.get("ok"))},
                )
            except Exception as exc:  # pragma: no cover - defensive
                self.record_command_event("track_end_home", {"error": str(exc)})

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

    def _build_lowcmd_pose_cmd(
        self,
        msg: Any,
        hold_q: list[float],
        commanded_q: dict[int, float],
        tau_by_index: dict[int, float] | None = None,
    ) -> Any:
        # rt/lowcmd driving the arms + waist (commanded_q: {motor index -> target q},
        # tau_by_index: gravity feed-forward torque from the closed-loop controller)
        # to a target pose while holding every other joint (the legs) stiff at
        # hold_q. Used AFTER the motion mode is released, so the legs are held stiff
        # (robot must be physically supported), the waist is free of the onboard
        # controller and turns, and the arms move to the recorded/edited pose.
        if self.lowcmd_factory is None or self.crc is None:
            raise RuntimeError("LowCmd factory is not initialized")
        tau_by_index = tau_by_index or {}
        cmd = self.lowcmd_factory()
        cmd.mode_pr = 0
        cmd.mode_machine = int(getattr(msg, "mode_machine", 0) or 0)
        for i in range(27):
            motor = cmd.motor_cmd[i]
            motor.mode = 1
            motor.dq = 0.0
            motor.tau = 0.0
            motor.reserve = 0
            if i in commanded_q:
                motor.q = commanded_q[i]
                motor.tau = float(tau_by_index.get(i, 0.0))
                if i == WAIST_YAW_JOINT:
                    motor.kp = WAIST_LOWCMD_KP
                    motor.kd = WAIST_LOWCMD_KD
                else:  # arm joint — reuse the arm_sdk gains so it holds against gravity
                    kp, kd = ARM_SDK_GAIN_BY_INDEX.get(i, (25.0, 0.8))
                    motor.kp = float(kp)
                    motor.kd = float(kd)
            else:
                motor.q = hold_q[i]
                if i < 12:  # legs held stiff
                    motor.kp = 70.0
                    motor.kd = 1.0
                else:  # any arm not being driven — hold in place
                    motor.kp = 25.0
                    motor.kd = 0.8
        cmd.crc = self.crc.Crc(cmd)
        return cmd

    def execute_lowcmd_pose(
        self,
        path: Path,
        plan: dict[str, Any],
        payload: dict[str, Any] | None,
        frames: list[dict[str, Any]],
        msg: Any,
        waist_target: float,
    ) -> tuple[int, dict[str, Any]]:
        """Drive the arms + waist to a target pose via a single persistent
        closed-loop PID controller (legs held; onboard motion mode released --
        robot must be physically supported).

        A long-lived thread holds the arms and is RETARGETED on each Move rather
        than killed and restarted: it never stops publishing, so between Moves the
        arms stay actively held (no limp drop) and there is only ever one lowcmd
        publisher (no fighting/creaking). The closed loop runs every cycle."""
        with self.command_lock:
            lowcmd_publisher = self.lowcmd_publisher
            motion_switcher = self.motion_switcher
        if lowcmd_publisher is None or self.lowcmd_factory is None or self.crc is None:
            return 503, {"ok": False, "error": "DDS lowcmd publisher is not available.", "recording": path.name, "plan": plan}

        # Arm + waist targets from the final recorded/edited frame (clamped). Legs
        # are never driven here. Respect the command scope but always include waist.
        scoped_joints = {
            int(item["index"])
            for item in plan.get("commanded_body_joints", [])
            if isinstance(item, dict) and "index" in item
        }
        if not scoped_joints:
            scoped_joints = set(ARM_SDK_JOINTS)
        body_targets = self._arm_replay_frame_targets(frames[-1], scoped_joints) if frames else {}
        body_targets[WAIST_YAW_JOINT] = waist_target  # already clamped upstream

        command = {
            "mode": "lowcmd_pose",
            "recording": path.name,
            "control_path": "lowcmd_pose",
            "waist_target": round(waist_target, 6),
            "arm_joint_count": sum(1 for j in body_targets if j != WAIST_YAW_JOINT),
        }

        # If the persistent controller is already running, just hand it the new
        # target -- continuous control, no restart, no drop, no creak.
        with self.command_lock:
            controller_alive = (
                self._pose_controller_running
                and self.replay_thread is not None
                and self.replay_thread.is_alive()
            )
            if controller_alive:
                self._pose_targets = dict(body_targets)
                self._pose_targets_version += 1
                self._set_wrist_status(
                    available=True, active=True,
                    message="lowcmd pose: retargeted (continuous closed-loop hold; arms not released).",
                    last_command=command,
                )
                return 202, {
                    "ok": True,
                    "message": "lowcmd pose retargeted: arms + waist ramping to the new target under continuous PID.",
                    "recording": path.name,
                    "plan": plan,
                    "waist_target": round(waist_target, 6),
                    "control_path": "lowcmd_pose",
                }
            prev_cancel = self.replay_cancel
            prev_thread = self.replay_thread
            wrist_cancel = self.wrist_cancel
            torso_cancel = self.torso_cancel
            self._pose_targets = dict(body_targets)
            self._pose_targets_version += 1

        # No live controller: cancel any other in-flight motion and wait for it to
        # stop publishing before starting a fresh controller (never two at once).
        for other in (prev_cancel, wrist_cancel, torso_cancel):
            if other is not None:
                other.set()
        if prev_thread is not None and prev_thread.is_alive() and prev_thread is not threading.current_thread():
            with contextlib.suppress(RuntimeError):
                prev_thread.join(timeout=2.5)

        cancel = threading.Event()
        tuning = {
            **self._arm_replay_tuning(payload if isinstance(payload, dict) else None),
            # Full-gain lowcmd PD needs far less measured-torque support than the
            # weak arm_sdk gains this controller was tuned for; the bounded
            # gravity-learn integral supplies the accurate steady holding torque.
            "gravity_hold_scale": 0.7,
            "gravity_move_scale": 0.35,
            "gravity_tau_filter_seconds": 0.8,
        }

        def run_pose_controller() -> None:
            released = bool(self.motion_mode_released)
            writes = 0
            pid_state: dict[int, dict[str, float]] = {}
            loop_status: dict[str, Any] = {}
            dt_nominal = 1.0 / 120.0
            step = WAIST_LOWCMD_MAX_VEL_RAD_S * dt_nominal
            last_tick = time.monotonic()
            with self.command_lock:
                state_msg = self.lowstate_msg or msg
                targets = dict(self._pose_targets)
                ver = self._pose_targets_version
            hold_q = [float(state_msg.motor_state[i].q) for i in range(27)]
            desired = {joint: float(state_msg.motor_state[joint].q) for joint in targets}
            targets_eff = dict(targets)
            stall_s: dict[int, float] = {}
            blocked: list[str] = []
            try:
                # Release the onboard motion mode once so the arms/waist are ours.
                if motion_switcher is not None:
                    with contextlib.suppress(Exception):
                        motion_switcher.ReleaseMode()
                        released = True
                        self.motion_mode_released = True
                        time.sleep(0.2)
                self._set_wrist_status(
                    available=True, active=True,
                    message="lowcmd pose: motion mode released; legs held, arms + waist under closed-loop PID.",
                    last_command=command,
                )
                while not cancel.is_set():
                    with self.command_lock:
                        latest_msg = self.lowstate_msg
                        publisher = self.lowcmd_publisher
                        cur_ver = self._pose_targets_version
                        if cur_ver != ver:
                            targets = dict(self._pose_targets)
                            ver = cur_ver
                            targets_eff = dict(targets)
                            stall_s = {}
                    if latest_msg is None or publisher is None:
                        break
                    for joint in targets:
                        if joint not in desired:
                            desired[joint] = float(latest_msg.motor_state[joint].q)
                    now = time.monotonic()
                    dt = min(0.05, max(0.001, now - last_tick))
                    last_tick = now
                    # Velocity-bounded ramp toward the target, easing out over the
                    # last ~0.12 rad so arrival is smooth (no hard velocity stop).
                    for joint, target_q in targets_eff.items():
                        remaining = target_q - desired[joint]
                        if remaining:
                            ease = max(0.3, min(1.0, abs(remaining) / 0.12))
                            delta = min(abs(remaining), step * ease)
                            desired[joint] += delta if remaining > 0 else -delta
                    # Closed loop around the ramped setpoint: PID inside a lock band
                    # (correction hard-zeroes at the target so the hold is dead-still)
                    # + filtered gravity feed-forward with bounded residual learning.
                    corrected, loop_status, tau_ff = self._closed_loop_arm_targets(
                        latest_msg, desired, pid_state, dt, tuning
                    )
                    publisher.Write(self._build_lowcmd_pose_cmd(latest_msg, hold_q, corrected, tau_ff))
                    writes += 1
                    # Contact backoff: only AFTER the ramp has fully commanded the
                    # target for this joint. A joint then stationary, outside the
                    # lock band, with saturated gravity feed-forward for 3 s is
                    # physically blocked -- accept its reachable pose instead of
                    # leaning on the obstacle. Gating on ramp-complete prevents it
                    # from ever aborting a still-in-progress reach.
                    for pj in loop_status.get("per_joint", []):
                        joint = int(pj["index"])
                        ramp_done = abs(targets_eff.get(joint, desired[joint]) - desired[joint]) < 1e-4
                        saturated = abs(pj["gravity_tau"]) >= 0.98 * self._arm_replay_gravity_tau_limit(joint)
                        if pj["locked"] or not pj["stationary"] or not saturated or not ramp_done:
                            stall_s.pop(joint, None)
                            continue
                        stall_s[joint] = stall_s.get(joint, 0.0) + dt
                        if stall_s[joint] >= 3.0 and targets_eff.get(joint) == targets.get(joint):
                            targets_eff[joint] = self._clamp_joint_target(
                                joint,
                                float(pj["actual_q"]) + max(-0.008, min(0.008, float(pj["error_rad"]))),
                            )
                            name = pj.get("name", f"Motor{joint}")
                            if name not in blocked:
                                blocked.append(name)
                                command["blocked_joints"] = list(blocked)
                    if writes % 240 == 0:  # ~every 2 s, for /api/wrist/status observability
                        self._set_wrist_status(
                            available=True, active=True,
                            message=(
                                f"lowcmd pose: max_err={loop_status.get('max_error_rad')} rad, "
                                f"settled {loop_status.get('settled_joints')}/{loop_status.get('joint_count')} joints."
                            ),
                            last_command={**command, "writes": writes, "closed_loop": loop_status},
                        )
                    time.sleep(dt_nominal)
            finally:
                with self.command_lock:
                    is_active = self.replay_cancel is cancel
                    if is_active:
                        self.replay_cancel = None
                        self.replay_thread = None
                        self._pose_controller_running = False
                restored = False
                if is_active and released and motion_switcher is not None:
                    with contextlib.suppress(Exception):
                        motion_switcher.SelectMode("ai")
                        restored = True
                    self.motion_mode_released = False
                if is_active:
                    self._set_wrist_status(
                        active=False,
                        message=(
                            f"lowcmd pose stopped ({writes} writes; motion mode restored={restored}; "
                            f"final max_err={loop_status.get('max_error_rad')} rad)."
                        ),
                        last_command={**command, "writes": writes, "closed_loop": loop_status},
                    )

        thread = threading.Thread(target=run_pose_controller, name="lowcmd-pose-controller", daemon=True)
        with self.command_lock:
            self.replay_cancel = cancel
            self.replay_thread = thread
            self._pose_controller_running = True
        thread.start()
        return 202, {
            "ok": True,
            "message": "lowcmd pose started: closed-loop PID, arms + waist to target (legs held, motion mode released).",
            "recording": path.name,
            "plan": plan,
            "waist_target": round(waist_target, 6),
            "control_path": "lowcmd_pose",
        }

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
            publisher = self.wrist_publisher
            msg = self.lowstate_msg
        if cancel is not None:
            cancel.set()
        if replay_cancel is not None:
            replay_cancel.set()
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

    def probe_entrances(self) -> dict[str, Any]:
        """Reachability of the robot's dashboard entrances (see /api/entrances).

        Runs wherever this server runs: on the robot it reports its own
        addresses; on the operator Mac it reports what the Mac can reach over
        the lab network — which is what the remote welcome page needs."""
        now = time.time()
        with self.lock:
            cached = getattr(self, "_entrance_probe_cache", None)
            if cached and now - cached[0] < 5.0:
                return cached[1]
        result: dict[str, Any] = {"checked_at": now}
        for name, base in ENTRANCE_PROBES.items():
            started = time.monotonic()
            try:
                # GET, not HEAD: this dashboard's HTTP handler implements
                # do_GET/do_POST only, so HEAD would 501 and read as offline.
                with urllib.request.urlopen(base + "/api/chat/status", timeout=1.5) as response:
                    response.read(512)
                    result[name] = {
                        "reachable": response.status < 500,
                        "latency_ms": round((time.monotonic() - started) * 1000, 1),
                    }
            except Exception:
                result[name] = {"reachable": False, "latency_ms": None}
        with self.lock:
            self._entrance_probe_cache = (now, result)
        return result

    def power_status(self) -> dict[str, Any]:
        """State of the smart plug powering the robot (see /api/power/status).

        Queries the lab Home Assistant with the server-side token so the https
        welcome page can label its wake/sleep button. "unknown" means the relay
        is unconfigured or HA is unreachable — never an error to the client."""
        if not HA_TOKEN:
            return {"enabled": False, "state": "unknown"}
        now = time.time()
        with self.lock:
            cached = getattr(self, "_power_status_cache", None)
            if cached and now - cached[0] < 3.0:
                return cached[1]
        request = urllib.request.Request(
            f"{HA_BASE_URL}/api/states/{HA_SWITCH_ENTITY}",
            headers={"Authorization": f"Bearer {HA_TOKEN}"},
        )
        try:
            with urllib.request.urlopen(request, timeout=HA_TIMEOUT_SECONDS) as response:
                payload = json.loads(response.read().decode("utf-8"))
            state = payload.get("state") or "unknown"
        except Exception:
            state = "unknown"
        result = {"enabled": True, "state": state, "entity": HA_SWITCH_ENTITY}
        with self.lock:
            self._power_status_cache = (now, result)
        return result

    def chill_motors(self) -> tuple[int, dict[str, Any]]:
        return self.request_chill({"armed": True, "i_understand_risk": True})

    def request_home(self) -> tuple[int, dict[str, Any]]:
        """Home: closed-loop move of the arms to the saved 'home' pose — the
        identical request body the dashboard Move button / chat move tool
        sends, so every arm_sdk safety gate applies unchanged (operator,
        2026-07-23; previously this held the current pose instead). Falls back
        to the legacy XR teleop home command when the DDS path is unavailable.
        """
        filename = self.named_positions().get("home")
        if not filename:
            return 404, {
                "ok": False,
                "error": "No saved position named 'home' — rename a saved pose "
                         "in the dashboard to 'home' to create one.",
            }
        status, result = self.request_robot_replay(
            {
                "filename": filename,
                "execute_arm_sdk": True,
                "command_scope": "arms",
                "closed_loop": True,
                "hold_after_convergence": True,
                "position_tolerance_rad": 0.01,
                "replay_response": 2.5,
            }
        )
        if status == 503:
            xr_status, xr_result = self._request_xr_ipc(
                "CMD_STOP", "XR teleop stop requested. Arms should move home during clean shutdown."
            )
            if xr_status < 400:
                return xr_status, xr_result
            result["xr_fallback_error"] = xr_result.get("error")
        return status, result

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
        # Cancel only the wrist oscillation here. Any arm_sdk hold/replay KEEPS
        # publishing through the mode/damp transition below: if the hold were
        # dropped first (weight -> 0), the onboard controller would instantly
        # reassert its own arm targets at full gains — a violent "snap toward
        # home" right before the motors go limp. With the hold alive until damp
        # is engaged, the arms simply sag from where they are.
        if cancel is not None:
            cancel.set()

        select_code = None
        stop_code = None
        damp_code = None
        try:
            if motion_switcher is not None:
                select_code, _ = motion_switcher.SelectMode("ai")
                time.sleep(0.15)
            if loco_client is None:
                self.request_track_stop()
                wrist_status = self.stop_wrist()
                return 503, {"ok": False, "error": "H1 loco client is not available.", "wrist": wrist_status}
            stop_code = loco_client.SetVelocity(0.0, 0.0, 0.0, 0.2)
            damp_code = loco_client.SetFsmId(1)
            # Give damp a moment to actually engage before dropping the hold.
            time.sleep(0.3)
        except Exception as exc:
            self.request_track_stop()
            wrist_status = self.stop_wrist()
            return 500, {
                "ok": False,
                "error": f"Could not request damp mode: {exc}",
                "select_mode_code": select_code,
                "stop_move_code": stop_code,
                "damp_code": damp_code,
                "wrist": wrist_status,
            }

        # Motors are damped now — releasing the arm_sdk weight cannot snap.
        # Also end any person-tracking session: it publishes arm_sdk weight=1.0
        # every loop and would otherwise re-assert arm authority right after the
        # damp, so the arms keep tracking a person despite the operator's "limp".
        self.request_track_stop()
        wrist_status = self.stop_wrist()

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
            "set_balance_mode",
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

        # Base-mobility actions (walk / translate the whole robot) require the
        # same explicit risk acknowledgement as wrist/tracking. Posture, stop,
        # damp and read-only get_* actions stay ungated (stopping must always be
        # allowed). The dashboard already sends these flags for every command;
        # this closes the gap for a raw/MCP/curl caller.
        if action in LOCO_MOBILITY_ACTIONS and not has_risk_ack(payload):
            return 400, {
                "ok": False,
                "error": f"Loco action '{action}' moves the robot; set armed=true and "
                         "i_understand_risk=true to proceed.",
            }

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
            balance_mode = int(self._coerce_float(payload, "balance_mode", 0.0, 0.0, 1.0))
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
            "balance_mode": balance_mode,
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
            elif action == "set_balance_mode":
                call_code = loco_client.SetBalanceMode(balance_mode)
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
                        # Oscillate builds its setpoint around the MEASURED center,
                        # so amplitude validation alone can drive the absolute angle
                        # past WRIST_LIMITS (and the hardware joint limit) when the
                        # wrist already sits near an extreme. Clamp the final angle.
                        low, high = WRIST_LIMITS
                        joint_low, joint_high = JOINT_LIMITS[RIGHT_WRIST_YAW]
                        wrist_q = max(max(low, joint_low), min(min(high, joint_high), wrist_q))
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

    def set_webcam_frame(self, frame: bytes) -> None:
        with self.webcam_condition:
            self.webcam_frame = frame
            self.webcam_timestamp = time.time()
            self.webcam_error = None
            self.webcam_condition.notify_all()

    def set_webcam_error(self, error: str | None) -> None:
        with self.webcam_condition:
            self.webcam_error = error

    def wait_for_webcam_frame(self, last_timestamp: float | None, timeout: float = 1.0) -> tuple[bytes | None, float | None]:
        with self.webcam_condition:
            if self.webcam_frame is not None and self.webcam_timestamp != last_timestamp:
                return self.webcam_frame, self.webcam_timestamp
            self.webcam_condition.wait(timeout)
            return self.webcam_frame, self.webcam_timestamp

    def webcam_snapshot(self) -> dict[str, Any]:
        with self.webcam_condition:
            stale = self.webcam_timestamp is not None and time.time() - self.webcam_timestamp > 5.0
            return {
                "source": "usb-webcam",
                "available": self.webcam_frame is not None and not stale,
                "timestamp": self.webcam_timestamp,
                "error": self.webcam_error,
            }

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
                # This callback fires at hundreds of Hz. Build the per-sample
                # record ONLY while a recording is active (otherwise write_sample
                # discards it); and defer the hand-dict + snapshot build to the
                # 30 Hz throttle below. Non-recording steady state is the common
                # case and now does neither per callback.
                if self.recorder.file is not None:
                    rec_hands = handstate_to_dict(hand_msg, hand_samples, hand_timestamp)
                    self.recorder.write_sample(
                        lowstate_record(msg, self.samples, rec_hands, hand_samples, hand_timestamp)
                    )
                if now - last_snapshot_at < 1.0 / 30.0:
                    return
                last_snapshot_at = now
                if len(self.sample_times) > 1:
                    elapsed = self.sample_times[-1] - self.sample_times[0]
                    rate = (len(self.sample_times) - 1) / elapsed if elapsed > 0 else 0
                else:
                    rate = 0

                hands = handstate_to_dict(hand_msg, hand_samples, hand_timestamp)
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
        elif request_path in ("/welcome", "/welcome.html"):
            # The welcome page lives on GitHub Pages so it stays reachable with
            # the robot (and the operator Mac) switched off. Redirect old
            # bookmarks instead of serving a robot-hosted copy.
            self.send_response(HTTPStatus.FOUND)
            self.send_header("Location", WELCOME_PAGE_URL)
            self.send_header("Content-Length", "0")
            self.end_headers()
        elif request_path == "/favicon.ico":
            self._send_file(STATIC_DIR / "assets" / "app-icon.png", "image/png")
        elif request_path == "/remote-entrance.json":
            # Current remote-tunnel hostnames for the welcome page's Offline
            # (read-only mirror) and Remote (live Ethernet relay) cards
            # (kept fresh by tools/update_remote_entrance.py on the operator Mac).
            self._send_file(STATIC_DIR / "remote-entrance.json", "application/json; charset=utf-8")
        elif request_path == "/api/webcam":
            self._send_json(self.store.webcam_snapshot())
        elif request_path == "/api/entrances":
            # Live reachability of the robot's Wi-Fi/Ethernet entrances, CORS-open
            # so the GitHub-Pages welcome page (https) can read it through the
            # operator Mac's https tunnel — browsers block the page's own direct
            # http probes as mixed content.
            self._send_json_cors(self.store.probe_entrances())
        elif request_path == "/api/power/status":
            # Smart-plug state for the welcome page's wake/sleep button,
            # relayed from the lab Home Assistant (CORS-open, read-only; the
            # HA token never leaves this server).
            self._send_json_cors(self.store.power_status())
        elif request_path == "/app.js":
            self._send_file(STATIC_DIR / "app.js", "application/javascript; charset=utf-8")
        elif request_path == "/viewer.js":
            self._send_file(STATIC_DIR / "viewer.js", "application/javascript; charset=utf-8")
        elif request_path == "/diagram.js":
            self._send_file(STATIC_DIR / "diagram.js", "application/javascript; charset=utf-8")
        elif request_path == "/styles.css":
            self._send_file(STATIC_DIR / "styles.css", "text/css; charset=utf-8")
        elif request_path in ("/feedback", "/feedback.html"):
            self._send_file(STATIC_DIR / "feedback.html", "text/html; charset=utf-8")
        elif request_path == "/api/state":
            self._send_json(self.store.snapshot())
        elif request_path == "/api/spatial/pose":
            self._send_json(self.store.spatial_pose_snapshot())
        elif request_path == "/api/motion/active":
            self._send_json(self.store.motion_active_snapshot())
        elif request_path == "/api/pose/feedback/data":
            self._send_json(pose_feedback_dataset())
        elif request_path.startswith("/api/pose/feedback/image/"):
            # Serve a collected reference image (basename only — no traversal).
            name = os.path.basename(unquote(request_path.removeprefix("/api/pose/feedback/image/")))
            path = (POSE_FEEDBACK_IMAGE_DIR / name).resolve()
            if path.parent != POSE_FEEDBACK_IMAGE_DIR.resolve() or not path.is_file():
                self.send_error(HTTPStatus.NOT_FOUND, "Image not found")
                return
            ctype = {"jpg": "image/jpeg", "png": "image/png", "webp": "image/webp"}.get(
                path.suffix.lstrip(".").lower(), "application/octet-stream")
            self._send_file(path, ctype)
        elif request_path == "/api/camera":
            self._send_json(self.store.camera_snapshot())
        elif request_path == "/api/track/status":
            self._send_json({"ok": True, "tracking": self.store.track_snapshot()})
        elif request_path == "/api/sentry/detect":
            query = parse_qs(urlsplit(self.path).query)
            feed = (query.get("feed") or ["head"])[0]
            self._send_json(self.store.sentry_detect(feed))
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
        elif request_path == "/api/loco/status":
            self._send_json(self.store.loco_snapshot())
        elif request_path == "/api/chat/status":
            self._send_json(
                {
                    "enabled": LLM_ENABLED,
                    "model": LLM_MODEL,
                    "endpoint": LLM_BASE_URL,
                    "twin_spatial_evidence": True,
                    "twin_vision": LLM_TWIN_VISION_ENABLED,
                    "claude_bridge": {
                        "configured": bool(CLAUDE_BRIDGE_URL),
                        "url": CLAUDE_BRIDGE_URL or None,
                    },
                    "voice_input": LLM_STT_ENABLED,
                    "voice_output": LLM_TTS_ENABLED,
                    "tools_enabled": LLM_TOOLS_ENABLED,
                    "tools": [spec["function"]["name"] for spec in self.store.chat_tool_specs()]
                    if LLM_TOOLS_ENABLED
                    else [],
                }
            )
        elif request_path == "/camera.mjpg":
            self._send_camera_stream()
        elif request_path == "/webcam.mjpg":
            self._send_webcam_stream()
        elif request_path == "/api/sentry/stream":
            self._send_sentry_stream()
        elif request_path == "/events":
            self._send_events()
        elif request_path == "/mcp":
            # Stateless MCP: no server-initiated SSE stream, POST only.
            self.send_error(HTTPStatus.METHOD_NOT_ALLOWED, "MCP endpoint is POST-only")
        elif request_path.startswith("/models/") or request_path.startswith("/vendor/") or request_path.startswith("/assets/"):
            self._send_static_asset(request_path)
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def do_POST(self) -> None:
        request_path = urlsplit(self.path).path
        if request_path not in (
            "/api/wrist/command",
            "/api/wrist/stop",
            "/api/robot/chill",
            "/api/robot/home",
            "/api/robot/straight",
            "/api/loco/command",
            "/api/xr/mode",
            "/api/chat",
            "/api/spatial/pose",
            "/api/pose/feedback",
            "/api/stt",
            "/api/tts",
            "/api/recording/start",
            "/api/recording/stop",
            "/api/recording/pose",
            "/api/recording/sequence",
            "/api/recording/rename",
            "/api/recording/replay/robot",
            "/api/track/start",
            "/api/track/stop",
            "/api/sentry/mode",
            "/api/mimic/mode",
            "/mcp",
        ):
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")
            return

        # Speech-to-text takes a raw audio body (not JSON) — handle it up front.
        if request_path == "/api/stt":
            self._handle_stt()
            return

        # MCP needs JSON-RPC-shaped errors, so it parses its own body.
        if request_path == "/mcp":
            self._handle_mcp()
            return

        payload: dict[str, Any] = {}
        if request_path in (
            "/api/wrist/command",
            "/api/robot/chill",
            "/api/loco/command",
            "/api/xr/mode",
            "/api/chat",
            "/api/spatial/pose",
            "/api/pose/feedback",
            "/api/pose/proposal/restage",
            "/api/tts",
            "/api/recording/start",
            "/api/recording/pose",
            "/api/recording/sequence",
            "/api/recording/rename",
            "/api/recording/replay/robot",
            "/api/track/start",
            "/api/sentry/mode",
            "/api/mimic/mode",
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

        if request_path == "/api/recording/rename":
            status, response = self.store.rename_recording(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/recording/replay/robot":
            status, response = self.store.request_robot_replay(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/track/start":
            status, response = self.store.request_track_start(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/track/stop":
            status, response = self.store.request_track_stop()
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/sentry/mode":
            status, response = self.store.set_sentry_mode(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/mimic/mode":
            status, response = self.store.set_mimic_mode(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/chat":
            status, response = self.store.chat(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/spatial/pose":
            status, response = self.store.update_spatial_pose(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/pose/feedback":
            status, response = self.store.record_pose_feedback(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/pose/proposal/restage":
            status, response = self.store.restage_proposal_by_id(payload)
            self._send_json_status(response, HTTPStatus(status))
            return

        if request_path == "/api/tts":
            status, result, content_type = synthesize_speech(str(payload.get("text", "")))
            if status == 200 and isinstance(result, (bytes, bytearray)):
                self._send_bytes(result, content_type)
            else:
                self._send_json_status(result, HTTPStatus(status))
            return

        self.store.record_command_event(request_path, payload)

        if request_path == "/api/wrist/stop":
            self._send_json(self.store.stop_wrist())
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
        # Without this, browsers heuristically cache app.js/styles.css and the
        # UI keeps running STALE code after a deploy (widgets render with no
        # behavior). no-cache = revalidate every load; fine on the LAN.
        self.send_header("Cache-Control", "no-cache")
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

    def _handle_stt(self) -> None:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        if length <= 0:
            self._send_json_status({"ok": False, "error": "Empty audio body."}, HTTPStatus.BAD_REQUEST)
            return
        if length > MAX_AUDIO_BYTES:
            self.close_connection = True
            self._send_json_status(
                {"ok": False, "error": f"Audio too large (max {MAX_AUDIO_BYTES} bytes)."},
                HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
            )
            return
        audio = self.rfile.read(length)
        content_type = self.headers.get("Content-Type", "audio/webm")
        status, response = transcribe_audio(audio, content_type)
        self._send_json_status(response, HTTPStatus(status))

    def _handle_mcp(self) -> None:
        """POST /mcp — stateless MCP over streamable HTTP (single JSON responses,
        no SSE stream, no sessions)."""
        if not MCP_ENABLED:
            self._send_json_status(
                {"ok": False, "error": "MCP endpoint is disabled (set MCP_ENABLED=1)."},
                HTTPStatus.NOT_FOUND,
            )
            return
        if MCP_TOKEN and self.headers.get("Authorization", "") != f"Bearer {MCP_TOKEN}":
            self._send_json_status(
                {"ok": False, "error": "Missing or invalid bearer token."},
                HTTPStatus.UNAUTHORIZED,
            )
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        if length > MAX_JSON_BODY_BYTES:
            self.close_connection = True
            self._send_json_status(
                mcp_error(None, -32600, f"Body must be at most {MAX_JSON_BODY_BYTES} bytes."),
                HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
            )
            return
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8")) if length > 0 else None
        except Exception as exc:
            self._send_json_status(
                mcp_error(None, -32700, f"Parse error: {exc}"), HTTPStatus.BAD_REQUEST
            )
            return
        response = self.store.mcp_request(payload)
        if response is None:  # notification: acknowledge with no body
            self.send_response(HTTPStatus.ACCEPTED)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        self._send_json(response)

    def _send_bytes(self, body: bytes, content_type: str) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, data: dict[str, Any]) -> None:
        self._send_json_status(data, HTTPStatus.OK)

    def _send_json_cors(self, data: dict[str, Any]) -> None:
        # Like _send_json but readable cross-origin (read-only status payloads
        # consumed by the GitHub-Pages welcome page).
        body = json.dumps(data, separators=(",", ":")).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        with contextlib.suppress(BrokenPipeError, ConnectionResetError):
            self.wfile.write(body)

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
            except OSError:
                # Any dropped/half-open client (incl. ssl.SSLError under TLS).
                break
            time.sleep(0.2)

    def _send_sentry_stream(self) -> None:
        self.store.sentry_stream_subscribe()
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            last_seq = -1
            while True:
                result, last_seq = self.store.wait_sentry_result(last_seq, timeout=1.0)
                try:
                    if result is None:
                        # Keepalive comment: lets us notice dead sockets and
                        # release the worker even when detection is stalled.
                        self.wfile.write(b": ping\n\n")
                    else:
                        payload = json.dumps(result, separators=(",", ":"))
                        self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    break
        finally:
            self.store.sentry_stream_unsubscribe()

    def _send_webcam_stream(self) -> None:
        self._send_mjpeg(self.store.wait_for_webcam_frame)

    def _send_camera_stream(self) -> None:
        self._send_mjpeg(self.store.wait_for_camera_frame)

    def _send_mjpeg(self, wait_for_frame) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Pragma", "no-cache")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        last_timestamp: float | None = None
        while True:
            frame, timestamp = wait_for_frame(last_timestamp, timeout=1.0)
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
            except OSError:
                # Any dropped/half-open client (incl. ssl.SSLError under TLS).
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

    scheme = "http"
    if TLS_CERT and TLS_KEY:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(certfile=TLS_CERT, keyfile=TLS_KEY)
        server.socket = context.wrap_socket(server.socket, server_side=True)
        scheme = "https"
        print(f"TLS enabled (cert {TLS_CERT})")

    print("Unitree telemetry dashboard")
    print(f"Listening on {scheme}://{args.host}:{args.port}")
    print(f"Try from another machine: {scheme}://{public_host()}:{args.port}")
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
