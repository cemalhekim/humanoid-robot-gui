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

# Fixed "pointing" arm shape: arm raised well forward, elbow bent so shoulder
# yaw swings the forearm laterally. Mirrored from the operator's authored
# left-arm pointing pose (recordings/20260722-143533, L pitch -1.315,
# roll -0.169, yaw 0.138, elbow 1.447; left->right mirror negates roll/yaw).
# H1-2 sign convention: NEGATIVE shoulder pitch raises the arm forward.
# Shoulder pitch/yaw are the two aimed joints; roll and elbow stay fixed.
POINTING_TEMPLATE: dict[int, float] = {
    R_SHOULDER_PITCH: -1.3,
    R_SHOULDER_ROLL: 0.17,
    R_SHOULDER_YAW: -0.14,
    R_ELBOW: 1.45,
}

# Where the arm parks when tracking is stale/stopped (relaxed at the side).
NEUTRAL_TEMPLATE: dict[int, float] = {
    R_SHOULDER_PITCH: 0.0,
    R_SHOULDER_ROLL: -0.05,
    R_SHOULDER_YAW: 0.0,
    R_ELBOW: 0.3,
}

# Conservative aiming envelope, intentionally tighter than server.py
# JOINT_LIMITS. server.py re-clamps against JOINT_LIMITS anyway.
TRACK_LIMITS: dict[int, tuple[float, float]] = {
    R_SHOULDER_PITCH: (-1.8, -0.6),
    R_SHOULDER_ROLL: (-0.6, 0.3),
    R_SHOULDER_YAW: (-1.0, 1.0),
    R_ELBOW: (0.1, 1.6),
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
        yaw_offset: float = -0.14,
        pitch_offset: float = -1.3,
        dead_band: float = 0.03,
    ) -> None:
        self.fov_yaw_rad = fov_yaw_rad
        self.fov_pitch_rad = fov_pitch_rad
        self.yaw_offset = yaw_offset
        self.pitch_offset = pitch_offset
        self.dead_band = dead_band
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

        # Both signs are flipped vs the naive map, verified live 2026-07-22:
        # the arm mirrored the person horizontally until yaw was negated, and
        # H1-2 shoulder pitch raises the arm with NEGATIVE values, so a higher
        # person (smaller cy) must drive pitch more negative.
        yaw = self.yaw_offset - (cx - 0.5) * self.fov_yaw_rad
        pitch = self.pitch_offset - (0.5 - cy) * self.fov_pitch_rad

        out = dict(POINTING_TEMPLATE)
        out[R_SHOULDER_YAW] = yaw
        out[R_SHOULDER_PITCH] = pitch
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


def _area(person: dict[str, Any]) -> float:
    return max(0.0, person["x2"] - person["x1"]) * max(0.0, person["y2"] - person["y1"])


def associate(
    persons: list[dict[str, Any]],
    prev_cx: float | None,
    prev_cy: float | None,
) -> dict[str, Any] | None:
    """Pick the person to track: nearest to previous target, else largest."""
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
