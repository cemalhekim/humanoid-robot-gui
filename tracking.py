"""Pure math and policy for the person-tracking / arm-pointing feature.

No imports from server.py and no I/O: everything here is unit-testable
offline. server.py owns frames, HTTP, DDS; this module owns the decisions.
Spec: docs/superpowers/specs/2026-07-21-person-pointing-design.md
"""
from __future__ import annotations

import math
from typing import Any

# H1-2 arm joint indices (must match server.py JOINT_LIMITS indices).
L_SHOULDER_PITCH = 13
L_SHOULDER_ROLL = 14
L_SHOULDER_YAW = 15
L_ELBOW = 16
L_WRIST_ROLL = 17
L_WRIST_PITCH = 18
L_WRIST_YAW = 19
R_SHOULDER_PITCH = 20
R_SHOULDER_ROLL = 21
R_SHOULDER_YAW = 22
R_ELBOW = 23
R_WRIST_ROLL = 24
R_WRIST_PITCH = 25
R_WRIST_YAW = 26

# Fixed "pointing" arm shape: the arm's pose when the person is dead-center
# in the camera. Operator-calibrated: mirrored from the authored left-arm
# pose recordings/20260722-153832 (L pitch -1.523, roll 0.045, yaw -0.114,
# elbow 1.347, wrist roll 0.01 / pitch -0.343 / yaw 0.39). Left->right
# mirror negates roll and yaw joints, keeps pitch and elbow.
# H1-2 sign convention: NEGATIVE shoulder pitch raises the arm forward.
# Shoulder pitch/yaw are the two aimed joints; the rest stay fixed.
POINTING_TEMPLATE: dict[int, float] = {
    R_SHOULDER_PITCH: -1.52,
    R_SHOULDER_ROLL: -0.05,
    R_SHOULDER_YAW: 0.11,
    R_ELBOW: 1.35,
    R_WRIST_ROLL: -0.01,
    R_WRIST_PITCH: -0.34,
    R_WRIST_YAW: -0.39,
}

# Where the arm parks when tracking is stale/stopped (relaxed at the side).
# Same key set as POINTING_TEMPLATE so gains/rate-limiting cover every
# commanded joint in both phases.
NEUTRAL_TEMPLATE: dict[int, float] = {
    R_SHOULDER_PITCH: 0.0,
    R_SHOULDER_ROLL: -0.05,
    R_SHOULDER_YAW: 0.0,
    R_ELBOW: 0.3,
    R_WRIST_ROLL: 0.0,
    R_WRIST_PITCH: 0.0,
    R_WRIST_YAW: 0.0,
}

# Horizontal aiming: with the upper arm pitched forward, the shoulder-yaw
# axis runs ALONG the arm (it spins the arm in place, no lateral sweep —
# field-observed 2026-07-22), while the roll axis stands vertical and does
# the actual left/right swing. Aim therefore interpolates whole poses:
# person at the image's right edge adds RIGHT_EDGE_DELTA to the template
# (derived from the operator's right-hand IK target X 0.523 Y -0.527
# Z 1.454 — a ~48deg outward swing with a straighter elbow); the left
# side mirrors the sweep and the clamp caps how far the right arm can
# cross the chest.
RIGHT_EDGE_DELTA: dict[int, float] = {
    R_SHOULDER_ROLL: -0.85,
    R_SHOULDER_YAW: -0.30,
    R_ELBOW: -0.35,
}

# Conservative aiming envelope, intentionally tighter than server.py
# JOINT_LIMITS. server.py re-clamps against JOINT_LIMITS anyway.
TRACK_LIMITS: dict[int, tuple[float, float]] = {
    R_SHOULDER_PITCH: (-2.0, -0.8),
    R_SHOULDER_ROLL: (-1.2, 0.3),
    R_SHOULDER_YAW: (-1.0, 1.0),
    R_ELBOW: (0.1, 1.6),
    R_WRIST_ROLL: (-0.3, 0.3),
    R_WRIST_PITCH: (-0.45, 0.45),
    R_WRIST_YAW: (-1.0, 1.0),
}


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class PointingMapper:
    """Map a normalized image position (cx, cy in 0..1) to joint targets.

    Linear map across the camera field of view; a dead band keeps the arm
    still while the person barely moves. No IK: only shoulder pitch/yaw aim.
    """

    def __init__(
        self,
        fov_yaw_rad: float = 1.25,
        fov_pitch_rad: float = 0.9,
        yaw_offset: float = 0.11,
        pitch_offset: float = -1.52,
        # A 2% image-space band rejects pose-keypoint flicker while preserving
        # deliberate person motion. The old 0.5% band made the arm chase
        # detector noise frame-by-frame.
        dead_band: float = 0.02,
        # Wrist fine-aim amplified head-keypoint jitter at the end effector.
        # Keep it opt-in until the coarse shoulder/elbow tracking is stable.
        wrist_yaw_gain: float = 0.0,
        wrist_pitch_gain: float = 0.0,
    ) -> None:
        self.fov_yaw_rad = fov_yaw_rad
        self.fov_pitch_rad = fov_pitch_rad
        self.yaw_offset = yaw_offset
        self.pitch_offset = pitch_offset
        self.dead_band = dead_band
        self.wrist_yaw_gain = wrist_yaw_gain
        self.wrist_pitch_gain = wrist_pitch_gain
        self._last_cx: float | None = None
        self._last_cy: float | None = None

    def targets(self, cx: float, cy: float) -> dict[int, float]:
        if (
            self._last_cx is not None
            and abs(cx - self._last_cx) <= self.dead_band
            and abs(cy - self._last_cy) <= self.dead_band
        ):
            cx, cy = self._last_cx, self._last_cy
        else:
            self._last_cx, self._last_cy = cx, cy

        # Horizontal: blend the calibrated center pose toward the right-edge
        # anchor pose. s=+1 at the image's right edge (person on the robot's
        # right, non-mirrored camera), s=-1 mirrors the sweep to the left,
        # where TRACK_LIMITS caps the cross-chest reach of the right arm.
        # The FOV sign selects camera orientation. A negative yaw FOV mirrors
        # horizontal motion for an external webcam facing the robot; magnitude
        # scales the calibrated edge delta around the 1.25-rad baseline.
        yaw_scale = self.fov_yaw_rad / 1.25
        s = (cx - 0.5) * 2.0 * yaw_scale
        out = {
            joint: value + s * RIGHT_EDGE_DELTA.get(joint, 0.0)
            for joint, value in POINTING_TEMPLATE.items()
        }
        out[R_SHOULDER_YAW] += self.yaw_offset - POINTING_TEMPLATE[R_SHOULDER_YAW]
        # Vertical: shoulder pitch raises the arm with NEGATIVE values
        # (verified live 2026-07-22), so a higher person (smaller cy) must
        # drive pitch more negative.
        out[R_SHOULDER_PITCH] = self.pitch_offset - (0.5 - cy) * self.fov_pitch_rad
        # Wrist fine-aim on top of the coarse pose: the hand itself keeps
        # turning toward the person (both axes), following the same sign
        # conventions as the shoulder (negative pitch = up, negative yaw
        # delta toward the image's right).
        out[R_WRIST_YAW] += s * self.wrist_yaw_gain
        out[R_WRIST_PITCH] -= (0.5 - cy) * self.wrist_pitch_gain
        return {
            joint: _clamp(value, *TRACK_LIMITS[joint])
            for joint, value in out.items()
        }


class RateLimiter:
    """Bound setpoint changes to max_step_rad_s, below controller caps."""

    def __init__(self, max_step_rad_s: float = 0.35) -> None:
        self.max_step_rad_s = max_step_rad_s

    def step(
        self,
        current: dict[int, float],
        target: dict[int, float],
        dt: float,
    ) -> dict[int, float]:
        max_step = self.max_step_rad_s * max(dt, 0.0)
        out: dict[int, float] = {}
        for joint, goal in target.items():
            now = current.get(joint, goal)
            out[joint] = now + _clamp(goal - now, -max_step, max_step)
        return out


class Smoother:
    """Exponential moving average over joint target dicts."""

    def __init__(self, alpha: float = 0.35) -> None:
        self.alpha = alpha
        self._value: dict[int, float] | None = None

    def update(self, value: dict[int, float]) -> dict[int, float]:
        if self._value is None:
            self._value = dict(value)
        else:
            for joint, v in value.items():
                prev = self._value.get(joint, v)
                self._value[joint] = prev + self.alpha * (v - prev)
        return dict(self._value)


class AimSmoother:
    """Low-pass noisy image-space aim points before joint mapping."""

    def __init__(self, alpha: float = 0.25) -> None:
        self.alpha = alpha
        self._value: tuple[float, float] | None = None

    def update(self, cx: float, cy: float) -> tuple[float, float]:
        cx = _clamp(float(cx), 0.0, 1.0)
        cy = _clamp(float(cy), 0.0, 1.0)
        if self._value is None:
            self._value = (cx, cy)
        else:
            px, py = self._value
            self._value = (
                px + self.alpha * (cx - px),
                py + self.alpha * (cy - py),
            )
        return self._value


def aim_point(target: dict[str, Any]) -> tuple[float, float]:
    """Where to aim on a person: the detector's head anchor (nose, else ear
    midpoint) when present, else near the top of the box as a head-height
    fallback. Operator request 2026-07-22: aim for the head as much as
    possible, even as it moves slightly."""
    head = target.get("head")
    if isinstance(head, dict) and "x" in head and "y" in head:
        return head["x"], head["y"]
    return target["cx"], target["y1"] + (target["y2"] - target["y1"]) * 0.15


def _area(person: dict[str, Any]) -> float:
    return max(0.0, person["x2"] - person["x1"]) * max(0.0, person["y2"] - person["y1"])


def has_head(person: dict[str, Any]) -> bool:
    """True when the detector anchored a head (nose/eye/ear keypoints).

    The robot's own raised arm reads as a 'person' box to the detector but
    never gets face keypoints — requiring a head makes it invisible to
    tracking (operator, 2026-07-23). The detect service also drops headless
    boxes at the source; this is the robot-side defense in depth."""
    head = person.get("head")
    return isinstance(head, dict) and "x" in head and "y" in head


def associate(
    persons: list[dict[str, Any]],
    prev_cx: float | None,
    prev_cy: float | None,
    target_id: Any | None = None,
) -> dict[str, Any] | None:
    """Pick a stable target.

    A UI lock is authoritative: when ``target_id`` is set, only that
    detector/ByteTrack identity can be selected, even on frames where its
    face keypoints disappear. Without an explicit lock we still prefer
    head-anchored people so a head-camera view of the robot's own arm cannot
    become the initial target.
    """
    if target_id is not None:
        return next((person for person in persons if person.get("id") == target_id), None)
    # Unlocked/manual head-camera starts retain the defensive head gate. The
    # important regression fix is that an explicit identity lock above can
    # survive temporary loss of face keypoints.
    persons = [person for person in persons if has_head(person)]
    if not persons:
        return None
    if prev_cx is None or prev_cy is None:
        return max(persons, key=_area)
    return min(
        persons,
        key=lambda p: (p["cx"] - prev_cx) ** 2 + (p["cy"] - prev_cy) ** 2,
    )


class TrackState:
    """Staleness / failure state machine for one tracking session.

    Phases: tracking (fresh target), hold (target briefly lost, keep pose),
    stale (lost too long, arm should go neutral), aborted (too many
    consecutive detect-service failures — session must end).
    """

    def __init__(
        self,
        stale_after_s: float = 1.5,
        hold_s: float = 2.0,
        max_failures: int = 10,
        target_id: Any | None = None,
        seed_target: dict[str, Any] | None = None,
    ) -> None:
        self.stale_after_s = stale_after_s
        self.hold_s = hold_s
        self.max_failures = max_failures
        self.phase = "stale"
        self.failures = 0
        self.target_id = target_id
        self.target: dict[str, Any] | None = dict(seed_target) if seed_target else None
        self.last_seen: float | None = None

    def on_detection(self, persons: list[dict[str, Any]], now: float) -> None:
        if self.phase == "aborted":
            return
        self.failures = 0
        prev_cx = self.target["cx"] if self.target else None
        prev_cy = self.target["cy"] if self.target else None
        picked = associate(persons, prev_cx, prev_cy, self.target_id)
        if picked is not None:
            self.target = picked
            self.last_seen = now
            self.phase = "tracking"
            return
        if self.last_seen is None:
            self.phase = "stale"
        elif now - self.last_seen <= self.hold_s:
            self.phase = "hold"
        else:
            self.phase = "stale"
            self.target = None

    def on_failure(self, now: float) -> None:
        if self.phase == "aborted":
            return
        self.failures += 1
        if self.failures >= self.max_failures:
            self.phase = "aborted"
            return
        if self.last_seen is None or now - self.last_seen > self.stale_after_s:
            self.phase = "stale"


# ---------------------------------------------------------------------------
# Mimic Mode: retarget a person's 2D pose keypoints onto both H1-2 arms.
# ---------------------------------------------------------------------------

# Both arms relaxed at the side — where mimic parks while no pose is visible.
MIMIC_NEUTRAL_TEMPLATE: dict[int, float] = {
    L_SHOULDER_PITCH: 0.0,
    L_SHOULDER_ROLL: 0.05,
    L_SHOULDER_YAW: 0.0,
    L_ELBOW: 0.3,
    L_WRIST_ROLL: 0.0,
    L_WRIST_PITCH: 0.0,
    L_WRIST_YAW: 0.0,
    R_SHOULDER_PITCH: 0.0,
    R_SHOULDER_ROLL: -0.05,
    R_SHOULDER_YAW: 0.0,
    R_ELBOW: 0.3,
    R_WRIST_ROLL: 0.0,
    R_WRIST_PITCH: 0.0,
    R_WRIST_YAW: 0.0,
}

# Conservative mimic envelope, intentionally tighter than server.py
# JOINT_LIMITS (which re-clamps anyway). Roll conventions per JOINT_LIMITS:
# NEGATIVE roll abducts the RIGHT arm outward, POSITIVE the LEFT.
MIMIC_LIMITS: dict[int, tuple[float, float]] = {
    L_SHOULDER_PITCH: (-1.6, 0.4),
    L_SHOULDER_ROLL: (-0.2, 2.6),
    L_SHOULDER_YAW: (-0.8, 0.8),
    L_ELBOW: (0.0, 2.3),
    L_WRIST_ROLL: (-0.3, 0.3),
    L_WRIST_PITCH: (-0.45, 0.45),
    L_WRIST_YAW: (-1.0, 1.0),
    R_SHOULDER_PITCH: (-1.6, 0.4),
    R_SHOULDER_ROLL: (-2.6, 0.2),
    R_SHOULDER_YAW: (-0.8, 0.8),
    R_ELBOW: (0.0, 2.3),
    R_WRIST_ROLL: (-0.3, 0.3),
    R_WRIST_PITCH: (-0.45, 0.45),
    R_WRIST_YAW: (-1.0, 1.0),
}


def has_upper_body(person: dict[str, Any]) -> bool:
    """True when at least one full upper-arm chain (shoulder+elbow) is visible."""
    kp = person.get("keypoints")
    if not isinstance(kp, dict):
        return False
    return any(
        isinstance(kp.get(f"{side}_shoulder"), dict)
        and isinstance(kp.get(f"{side}_elbow"), dict)
        for side in ("l", "r")
    )


class MimicMapper:
    """Map detector pose keypoints (normalized image space, y down) to
    both-arm joint targets, mirror-style.

    The deployed webcam presents robot-relative left/right (not mirrored,
    verified 2026-07-23), so a person facing the robot has their LEFT arm on
    the robot's RIGHT side: person-left keypoints drive the robot's RIGHT
    arm and vice versa — the robot behaves like a mirror.

    Frontal-plane retarget from a single 2D view:
    - upper-arm elevation (angle from hanging-down, positive = outward)
      drives shoulder ROLL (abduction);
    - the shoulder→elbow→wrist bend angle drives the ELBOW joint;
    - upper-arm FORESHORTENING (projected length vs the expected length,
      0.75 × shoulder width) drives shoulder PITCH: an arm swung toward the
      camera shrinks on screen, and acos(length ratio) is the out-of-plane
      angle. Depth sign is unobservable in 2D, so shrinkage is always read
      as FORWARD (people don't raise arms backward); a threshold below
      acos(pitch_start_ratio) swallows keypoint length noise around the
      frontal plane, and the output is capped at max_pitch;
    - shoulder yaw and wrists stay neutral (yaw plane is a hardware-tuning
      follow-up).
    An arm whose keypoints are missing this frame HOLDS its previous
    targets; the caller's staleness state machine decides when to park.
    """

    # (person keypoint prefix, outward x sign in image space,
    #  robot roll joint, roll outward sign, robot elbow joint,
    #  robot pitch joint)
    _ARMS = (
        ("l", 1.0, R_SHOULDER_ROLL, -1.0, R_ELBOW, R_SHOULDER_PITCH),
        ("r", -1.0, L_SHOULDER_ROLL, 1.0, L_ELBOW, L_SHOULDER_PITCH),
    )

    def __init__(
        self,
        min_segment: float = 0.015,
        dead_band_rad: float = 0.04,
        arm_ratio: float = 0.75,
        pitch_start_ratio: float = 0.9,
        max_pitch: float = 1.2,
        min_shoulder_width: float = 0.05,
    ) -> None:
        # Segments shorter than ~1.5% of the image are direction-noise
        # (foreshortened arm pointing at the camera); hold instead of jitter.
        self.min_segment = min_segment
        self.dead_band_rad = dead_band_rad
        # Expected full upper-arm length is arm_ratio × shoulder width
        # (human biacromial proportions; matches the deployed camera view).
        self.arm_ratio = arm_ratio
        # No pitch until the arm has visibly shortened below this fraction
        # of expected (~26° out of plane at 0.9) — keypoint jitter around
        # full length would otherwise thrash acos where its slope blows up.
        self._pitch_threshold = math.acos(_clamp(pitch_start_ratio, 0.0, 1.0))
        self.max_pitch = max_pitch
        # Rescale so a fully camera-pointing arm (raw 90°) hits max_pitch.
        self._pitch_gain = max_pitch / (math.pi / 2 - self._pitch_threshold)
        self.min_shoulder_width = min_shoulder_width
        self._last: dict[int, float] = dict(MIMIC_NEUTRAL_TEMPLATE)

    @staticmethod
    def _vec(a: dict[str, Any], b: dict[str, Any]) -> tuple[float, float]:
        return float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])

    def targets(self, keypoints: dict[str, Any]) -> dict[int, float]:
        out = dict(self._last)
        l_shoulder = keypoints.get("l_shoulder")
        r_shoulder = keypoints.get("r_shoulder")
        shoulder_width = (
            math.hypot(*self._vec(l_shoulder, r_shoulder))
            if isinstance(l_shoulder, dict) and isinstance(r_shoulder, dict)
            else 0.0
        )
        for prefix, out_sign, roll_joint, roll_sign, elbow_joint, pitch_joint in self._ARMS:
            shoulder = keypoints.get(f"{prefix}_shoulder")
            elbow = keypoints.get(f"{prefix}_elbow")
            wrist = keypoints.get(f"{prefix}_wrist")
            if not (isinstance(shoulder, dict) and isinstance(elbow, dict)):
                continue
            ux, uy = self._vec(shoulder, elbow)
            upper_len = math.hypot(ux, uy)
            # Pitch from foreshortening. Unlike roll/elbow this stays valid
            # for a near-zero segment — a vanishing upper arm IS the signal
            # (arm at the camera), only its direction is meaningless. Needs
            # both shoulders for scale; otherwise the pitch holds.
            if shoulder_width >= self.min_shoulder_width:
                ratio = _clamp(upper_len / (self.arm_ratio * shoulder_width), 0.0, 1.0)
                raw = math.acos(ratio)
                pitch = (
                    0.0 if raw <= self._pitch_threshold
                    # H1-2 sign convention: NEGATIVE pitch raises the arm forward.
                    else -min(self.max_pitch, (raw - self._pitch_threshold) * self._pitch_gain)
                )
                if abs(pitch - out[pitch_joint]) > self.dead_band_rad:
                    out[pitch_joint] = pitch
            if upper_len < self.min_segment:
                continue
            # 0 = arm hanging down (image y grows downward), +pi/2 = out
            # horizontal, ~pi = overhead. Slightly negative = crossed inward.
            elevation = math.atan2(out_sign * ux, uy)
            roll = roll_sign * elevation
            if abs(roll - out[roll_joint]) > self.dead_band_rad:
                out[roll_joint] = roll
            if isinstance(wrist, dict):
                fx, fy = self._vec(elbow, wrist)
                if math.hypot(fx, fy) >= self.min_segment:
                    # Interior bend angle: 0 = straight arm, pi = folded.
                    dot = ux * fx + uy * fy
                    cross = ux * fy - uy * fx
                    bend = abs(math.atan2(cross, dot))
                    if abs(bend - out[elbow_joint]) > self.dead_band_rad:
                        out[elbow_joint] = bend
        out = {
            joint: _clamp(value, *MIMIC_LIMITS[joint])
            for joint, value in out.items()
        }
        self._last = dict(out)
        return out
