"""Pure math and policy for the person-tracking / arm-pointing feature.

No imports from server.py and no I/O: everything here is unit-testable
offline. server.py owns frames, HTTP, DDS; this module owns the decisions.
Spec: docs/superpowers/specs/2026-07-21-person-pointing-design.md
"""
from __future__ import annotations

from typing import Any

# H1-2 right-arm joint indices (must match server.py JOINT_LIMITS indices).
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
        # Operator wants the arm to answer small target motion; the band only
        # swallows sub-pixel detector jitter, the Smoother handles the rest.
        dead_band: float = 0.005,
        # Wrist fine-aim: the hand keeps turning toward the person on top of
        # the coarse shoulder sweep, so the "hand end" of the arm stays aimed
        # even where the shoulder clamps (cross-chest left side).
        wrist_yaw_gain: float = -0.5,
        wrist_pitch_gain: float = 0.5,
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
        s = (cx - 0.5) * 2.0
        out = {
            joint: value + s * RIGHT_EDGE_DELTA.get(joint, 0.0)
            for joint, value in POINTING_TEMPLATE.items()
        }
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
) -> dict[str, Any] | None:
    """Pick the person to track among detections WITH a head anchor:
    nearest to previous target, else largest."""
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
    ) -> None:
        self.stale_after_s = stale_after_s
        self.hold_s = hold_s
        self.max_failures = max_failures
        self.phase = "stale"
        self.failures = 0
        self.target: dict[str, Any] | None = None
        self.last_seen: float | None = None

    def on_detection(self, persons: list[dict[str, Any]], now: float) -> None:
        if self.phase == "aborted":
            return
        self.failures = 0
        prev_cx = self.target["cx"] if self.target else None
        prev_cy = self.target["cy"] if self.target else None
        picked = associate(persons, prev_cx, prev_cy)
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
