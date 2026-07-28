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

# Both arms in the H1-2's natural standby — where mimic parks while no pose
# is visible. Values read from the real robot's relaxed stance (rt/lowstate,
# 2026-07-28). NOTE elbow semantics (verified against h1_2.urdf + the live
# digital twin, which feeds motor q straight into the URDF): elbow q=0 is a
# 90-degree bend with the forearm FORWARD; q=+pi/2 is the straight arm.
MIMIC_NEUTRAL_TEMPLATE: dict[int, float] = {
    L_SHOULDER_PITCH: 0.15,
    L_SHOULDER_ROLL: 0.2,
    L_SHOULDER_YAW: 0.0,
    L_ELBOW: 0.7,
    L_WRIST_ROLL: 0.0,
    L_WRIST_PITCH: 0.0,
    L_WRIST_YAW: 0.0,
    R_SHOULDER_PITCH: 0.15,
    R_SHOULDER_ROLL: -0.2,
    R_SHOULDER_YAW: 0.0,
    R_ELBOW: 0.7,
    R_WRIST_ROLL: 0.0,
    R_WRIST_PITCH: 0.0,
    R_WRIST_YAW: 0.0,
}

# Conservative mimic envelope, intentionally tighter than server.py
# JOINT_LIMITS (which re-clamps anyway). Roll conventions per JOINT_LIMITS:
# NEGATIVE roll abducts the RIGHT arm outward, POSITIVE the LEFT.
# Elbow range is in URDF q semantics: -0.9 = deep human curl, 1.6 = straight.
MIMIC_LIMITS: dict[int, tuple[float, float]] = {
    L_SHOULDER_PITCH: (-1.6, 0.4),
    L_SHOULDER_ROLL: (-0.2, 2.6),
    L_SHOULDER_YAW: (-0.8, 0.8),
    L_ELBOW: (-0.9, 1.6),
    L_WRIST_ROLL: (-0.3, 0.3),
    L_WRIST_PITCH: (-0.45, 0.45),
    L_WRIST_YAW: (-1.0, 1.0),
    R_SHOULDER_PITCH: (-1.6, 0.4),
    R_SHOULDER_ROLL: (-2.6, 0.2),
    R_SHOULDER_YAW: (-0.8, 0.8),
    R_ELBOW: (-0.9, 1.6),
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

def _rot_x(v: tuple[float, float, float], a: float) -> tuple[float, float, float]:
    c, s = math.cos(a), math.sin(a)
    return (v[0], c * v[1] - s * v[2], s * v[1] + c * v[2])


def _rot_y(v: tuple[float, float, float], a: float) -> tuple[float, float, float]:
    c, s = math.cos(a), math.sin(a)
    return (c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2])


# ---------------------------------------------------------------------------
# Arm forward kinematics + self-collision guard (h1_2.urdf ground truth).
# Pure python 3x3 math, torso_link frame (x forward, y left, z up).
# ---------------------------------------------------------------------------

def _mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat_vec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def _rot_rpy(rpy):
    r, p, y = rpy
    Rx = [[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]]
    Ry = [[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]]
    Rz = [[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]]
    return _mat_mul(Rz, _mat_mul(Ry, Rx))


def _axis_rot(axis, a):
    x, y, z = axis
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return [
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


# (joint index, origin xyz, origin rpy, axis) per arm, torso -> wrist_yaw,
# transcribed from h1_2.urdf. Wrist origins chain the forearm geometry so
# collision points are exact even with wrists commanded (mimic pins them 0).
_ARM_CHAINS: dict[str, tuple] = {
    "right": (
        (R_SHOULDER_PITCH, (0, -0.14806, 0.42333), (-0.2618, 0, 0), (0, 1, 0)),
        (R_SHOULDER_ROLL, (0.0342, -0.061999, -0.0060011), (0.2618, 0, 0), (1, 0, 0)),
        (R_SHOULDER_YAW, (-0.0342, 0, -0.1456), (0, 0, 0), (0, 0, 1)),
        (R_ELBOW, (0.006, -0.0329, -0.182), (0, 0, 0), (0, 1, 0)),
        (R_WRIST_ROLL, (0.121, 0.0329, -0.011), (0, 0, 0), (1, 0, 0)),
        (R_WRIST_PITCH, (0.087, 0, 0), (0, 0, 0), (0, 1, 0)),
        (R_WRIST_YAW, (0.020, 0, 0), (0, 0, 0), (0, 0, 1)),
    ),
    "left": (
        (L_SHOULDER_PITCH, (0, 0.14806, 0.42333), (0.2618, 0, 0), (0, 1, 0)),
        (L_SHOULDER_ROLL, (0.0342, 0.061999, -0.0060011), (-0.2618, 0, 0), (1, 0, 0)),
        (L_SHOULDER_YAW, (-0.0342, 0, -0.1456), (0, 0, 0), (0, 0, 1)),
        (L_ELBOW, (0.006, 0.0329, -0.182), (0, 0, 0), (0, 1, 0)),
        (L_WRIST_ROLL, (0.121, -0.0329, -0.011), (0, 0, 0), (1, 0, 0)),
        (L_WRIST_PITCH, (0.087, 0, 0), (0, 0, 0), (0, 1, 0)),
        (L_WRIST_YAW, (0.020, 0, 0), (0, 0, 0), (0, 0, 1)),
    ),
}

# Collision sphere radii per arm point (matches the 3D editor's JS model)
# plus the hand ball sitting ~0.10 m beyond the wrist-yaw frame along +x.
_ARM_SPHERES = (("elbow", 0.08), ("wrist_pitch", 0.075), ("wrist_yaw", 0.075), ("hand", 0.085))
_HAND_OFFSET = (0.10, 0.0, 0.0)
_ARM_POINT_JOINT = {"elbow": 3, "wrist_pitch": 5, "wrist_yaw": 6}

# Body spheres in the torso frame: centers hand-placed against the URDF
# meshes (camera/lidar are their exact fixed-joint origins), radii tuned so
# every legitimate mimic pose (standby, hang, T-pose, forward reach) keeps
# >2 cm clearance while chest-contact and crossed-arm poses collide — see
# MimicCollisionTests.
_BODY_SPHERES = (
    ((0.0, 0.0, 0.32), 0.16, "torso"),
    ((0.0, 0.0, -0.08), 0.14, "pelvis"),
    ((0.11109, 0.0175, 0.68789), 0.12, "camera"),
    ((0.04874, 0.0, 0.6798), 0.12, "lidar"),
)


def arm_points(side: str, targets: dict[int, float]) -> dict[str, tuple[float, float, float]]:
    """FK: joint targets -> named collision-point positions (torso frame)."""
    R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    p = [0.0, 0.0, 0.0]
    frames = []
    for joint, xyz, rpy, axis in _ARM_CHAINS[side]:
        step = _mat_vec(R, list(xyz))
        p = [p[i] + step[i] for i in range(3)]
        R = _mat_mul(R, _rot_rpy(rpy))
        R = _mat_mul(R, _axis_rot(axis, float(targets.get(joint, 0.0))))
        frames.append((tuple(p), R))
    points = {name: frames[idx][0] for name, idx in _ARM_POINT_JOINT.items()}
    wrist_p, wrist_R = frames[6]
    hand = _mat_vec(wrist_R, list(_HAND_OFFSET))
    points["hand"] = tuple(wrist_p[i] + hand[i] for i in range(3))
    return points


def mimic_pose_collides(targets: dict[int, float]) -> str | None:
    """Self-collision check for a both-arm pose. Returns a short label for
    the first colliding sphere pair, or None when the pose is clear. Pure
    sphere-vs-sphere: arm points vs body spheres, and arm vs arm."""
    arms = {side: arm_points(side, targets) for side in ("right", "left")}
    for side, points in arms.items():
        for name, radius in _ARM_SPHERES:
            px, py, pz = points[name]
            for (bx, by, bz), bradius, blabel in _BODY_SPHERES:
                if math.dist((px, py, pz), (bx, by, bz)) < radius + bradius:
                    return f"{side}-{name} vs {blabel}"
    for rname, rradius in _ARM_SPHERES:
        for lname, lradius in _ARM_SPHERES:
            if math.dist(arms["right"][rname], arms["left"][lname]) < rradius + lradius:
                return f"right-{rname} vs left-{lname}"
    return None


class MimicMapper:
    """Map detector pose keypoints (normalized image space, y down) to
    both-arm joint targets, mirror-style, via deterministic single-view 3D
    lifting (Taylor 2000: known bone length + projected length -> the
    out-of-plane angle) and an analytic inverse of the H1-2 URDF arm chain.

    The deployed webcam presents robot-relative left/right (not mirrored,
    verified 2026-07-23), so person-left keypoints drive the robot's RIGHT
    arm and vice versa — the robot behaves like a mirror.

    Per frame:
    1. Torso frame: shoulder line = lateral axis, shoulder-mid -> hip-mid
       = down axis (in-image perpendicular fallback when hips are
       missing); shoulder width = the scale reference.
    2. Lift each bone to 3D: lateral/down components are measured in the
       torso frame, the forward component is
       sqrt(max(0, L_eff^2 - measured^2)) with L_eff = depth_start_ratio
       * expected bone length — a projection can only SHRINK a bone, so
       shrinkage is the out-of-plane angle. The depth sign is unobservable
       in a single view and is always read as FORWARD (people do not
       raise arms behind their back).
    3. Invert the URDF chain analytically (pitch about y, then roll about
       x, with the ±15° shoulder-mount tilt folded in): the upper-arm
       vector yields shoulder pitch+roll; the forearm vector, expressed in
       the post-roll local frame, yields shoulder yaw (atan2 of its
       in-plane components) and elbow q. URDF elbow semantics (verified
       against h1_2.urdf + the live digital twin): q=0 is a 90° bend with
       the forearm forward, q=+pi/2 is the straight arm, negative q curls.
    4. Degeneracies are explicit deterministic rules: no shoulder width ->
       planar fallback (roll+elbow from 2D directions, pitch/yaw hold);
       near-zero projected bone WITH scale -> depth dominates (the limb
       points at the camera); forearm nearly parallel to the upper arm ->
       yaw holds (gimbal); missing keypoints -> the arm holds and the
       caller's staleness machine decides when to park.
    """

    # (person keypoint prefix, robot pitch/roll/yaw/elbow joints,
    #  robot-outward y sign, shoulder-mount tilt). Mirror mapping:
    # person "l" drives the robot RIGHT arm (outward = -y, tilt -15°).
    _ARMS = (
        ("l", R_SHOULDER_PITCH, R_SHOULDER_ROLL, R_SHOULDER_YAW, R_ELBOW, -1.0, -0.2618),
        ("r", L_SHOULDER_PITCH, L_SHOULDER_ROLL, L_SHOULDER_YAW, L_ELBOW, 1.0, 0.2618),
    )

    def __init__(
        self,
        min_segment: float = 0.015,
        dead_band_rad: float = 0.04,
        upper_ratio: float = 0.75,
        fore_ratio: float = 0.65,
        depth_start_ratio: float = 0.94,
        max_pitch: float = 1.2,
        min_shoulder_width: float = 0.05,
        min_yaw_cos: float = 0.25,
        aspect: float = 1.0,
        depth_ramp: float = 0.08,
        yaw_hysteresis: float = 0.05,
    ) -> None:
        self.min_segment = min_segment
        self.dead_band_rad = dead_band_rad
        # Frame height/width ratio. Detector keypoints are normalized 0..1
        # PER AXIS, so on a 16:9 frame a vertical distance is 16/9x longer
        # than the same numeric horizontal distance — which silently breaks
        # every length ratio (arms are mostly vertical, shoulder width is
        # mostly horizontal). All geometry below multiplies y by this
        # factor to restore isotropic width-units. The session owner sets
        # it from the detect result's real w/h (h1 webcam bridge frames);
        # 1.0 keeps square-coordinate behavior when w/h is unknown.
        self.aspect = aspect
        # Expected bone lengths in units of shoulder width (human
        # anthropometry: upper arm ~0.75x, elbow->wrist ~0.65x biacromial).
        self.upper_ratio = upper_ratio
        self.fore_ratio = fore_ratio
        # Depth dead zone: a bone must shrink below this fraction of its
        # expected length before any forward depth is inferred — length
        # jitter around full extension would otherwise thrash the sqrt
        # where its slope blows up.
        self.depth_start_ratio = depth_start_ratio
        self.max_pitch = max_pitch
        self.min_shoulder_width = min_shoulder_width
        # cos(elbow q) below this = forearm nearly parallel to the upper
        # arm = the flex plane (yaw) is unobservable; yaw holds.
        self.min_yaw_cos = min_yaw_cos
        # URDF link-offset compensation (h1_2.urdf, exact numbers):
        # - the elbow->wrist offset (0.121, ±0.0329, -0.011) is NOT along
        #   the elbow frame's x axis: it leans 15° inward and 5° down.
        #   The forearm equation is solved against this true axis
        #   (a, ±b, c), A=|az-plane|, delta=atan2(a, c);
        # - the shoulder->elbow line at all-zero leans ~0.10 rad inward
        #   from the rotated -z axis (yaw/elbow origin offsets), a
        #   constant roll bias removed per arm.
        self._fore_a, self._fore_b, self._fore_c = 0.958, 0.26, -0.087
        self._fore_A = math.hypot(self._fore_a, self._fore_c)
        self._fore_delta = math.atan2(self._fore_a, self._fore_c)
        self.roll_bias = 0.10
        # Smoothness (dance profile 2026-07-28):
        # - depth_ramp: instead of a hard dead-zone edge (where sqrt slope
        #   is infinite and a hovering arm chatters), depth fades in with
        #   blend^2 over this ratio band below depth_start_ratio;
        # - yaw hysteresis: yaw starts updating above
        #   min_yaw_cos+yaw_hysteresis of in-plane magnitude and keeps
        #   updating until it drops below min_yaw_cos-yaw_hysteresis, so a
        #   slowly straightening arm cannot flicker update/hold.
        self.depth_ramp = depth_ramp
        self.yaw_hysteresis = yaw_hysteresis
        self._yaw_active: dict[int, bool] = {}
        # Per-person bone calibration: rolling MAX of the observed
        # projected-length/width ratio, clamped, seeded at the
        # anthropometric default. Projection can only shrink a bone, so
        # the max converges to the person's true ratio from below the
        # moment they extend the arm in the frontal plane (T-pose at
        # session start). Growing the expected length only ever REDUCES
        # inferred depth — calibration cannot create false motion.
        self._cal: dict[str, float] = {}
        self._last: dict[int, float] = dict(MIMIC_NEUTRAL_TEMPLATE)

    def _vec(self, a: dict[str, Any], b: dict[str, Any]) -> tuple[float, float]:
        # y scaled by the frame aspect -> isotropic width-units.
        return (
            float(b["x"]) - float(a["x"]),
            (float(b["y"]) - float(a["y"])) * self.aspect,
        )

    def _lift(
        self, lat: float, down: float, expected: float, y_sign: float
    ) -> tuple[float, float, float]:
        """2D bone (lateral/down, image units) -> unit 3D robot-frame
        vector (x forward, y left, z up). Outward lateral maps to the
        robot arm's outward y via y_sign; depth is forward-only."""
        planar_sq = lat * lat + down * down
        eff = self.depth_start_ratio * expected
        depth = math.sqrt(max(0.0, eff * eff - planar_sq))
        if expected > 1e-9 and self.depth_ramp > 1e-9:
            # Linear fade-in over the ramp band below the dead-zone
            # edge: kills the sqrt's infinite slope at the edge while
            # keeping the worst spatial gradient ~0.08 rad per 1% of
            # bone length anywhere in the band.
            ratio = math.sqrt(planar_sq) / expected
            depth *= _clamp((self.depth_start_ratio - ratio) / self.depth_ramp, 0.0, 1.0)
        norm = math.sqrt(planar_sq + depth * depth)
        if norm < 1e-9:
            return (1.0, 0.0, 0.0)
        return (depth / norm, y_sign * lat / norm, -down / norm)

    def _calibrated(self, key: str, default: float, observed_ratio: float, hi: float) -> float:
        cal = max(self._cal.get(key, default), min(observed_ratio, hi))
        self._cal[key] = cal
        return cal

    def _yaw_gate(self, joint: int, in_plane: float) -> bool:
        active = self._yaw_active.get(joint, False)
        if active:
            active = in_plane >= self.min_yaw_cos - self.yaw_hysteresis
        else:
            active = in_plane >= self.min_yaw_cos + self.yaw_hysteresis
        self._yaw_active[joint] = active
        return active

    def targets(self, keypoints: dict[str, Any]) -> dict[int, float]:
        out = dict(self._last)
        l_sh = keypoints.get("l_shoulder")
        r_sh = keypoints.get("r_shoulder")
        width = 0.0
        lat_axis = down_axis = None
        if isinstance(l_sh, dict) and isinstance(r_sh, dict):
            wx, wy = self._vec(r_sh, l_sh)  # person-right -> person-left
            width = math.hypot(wx, wy)
        if width >= self.min_shoulder_width:
            lat_axis = (wx / width, wy / width)  # toward person-LEFT side
            # Down axis: shoulder-mid -> hip-mid when hips are visible,
            # else the in-image perpendicular that points to image-bottom.
            l_hip, r_hip = keypoints.get("l_hip"), keypoints.get("r_hip")
            if isinstance(l_hip, dict) and isinstance(r_hip, dict):
                sx = (float(l_sh["x"]) + float(r_sh["x"])) / 2.0
                sy = (float(l_sh["y"]) + float(r_sh["y"])) / 2.0 * self.aspect
                hx = (float(l_hip["x"]) + float(r_hip["x"])) / 2.0
                hy = (float(l_hip["y"]) + float(r_hip["y"])) / 2.0 * self.aspect
                dn = math.hypot(hx - sx, hy - sy)
                if dn > 1e-6:
                    down_axis = ((hx - sx) / dn, (hy - sy) / dn)
            if down_axis is None:
                down_axis = (-lat_axis[1], lat_axis[0])
            if down_axis[1] < 0.0:
                down_axis = (-down_axis[0], -down_axis[1])
        for prefix, pitch_j, roll_j, yaw_j, elbow_j, y_sign, tilt in self._ARMS:
            shoulder = keypoints.get(f"{prefix}_shoulder")
            elbow = keypoints.get(f"{prefix}_elbow")
            wrist = keypoints.get(f"{prefix}_wrist")
            if not (isinstance(shoulder, dict) and isinstance(elbow, dict)):
                continue
            u2 = self._vec(shoulder, elbow)
            upper_len = math.hypot(*u2)
            # Outward lateral: person-left arm's outward is +lat_axis,
            # person-right arm's is -lat_axis (a mirror keeps outwardness).
            out_sign = 1.0 if prefix == "l" else -1.0

            if lat_axis is None:
                # Planar fallback (no scale reference): frontal-plane roll
                # and elbow bend from 2D directions; pitch and yaw hold.
                if upper_len < self.min_segment:
                    continue
                elevation = math.atan2(out_sign * u2[0], u2[1])
                # y_sign -1 (robot right): outward elevation -> negative
                # roll, matching JOINT_LIMITS conventions.
                self._apply(out, roll_j, y_sign * elevation)
                if isinstance(wrist, dict):
                    f2 = self._vec(elbow, wrist)
                    if math.hypot(*f2) >= self.min_segment:
                        dot = u2[0] * f2[0] + u2[1] * f2[1]
                        cross = u2[0] * f2[1] - u2[1] * f2[0]
                        bend = abs(math.atan2(cross, dot))
                        self._apply(out, elbow_j, math.pi / 2 - bend)
                continue

            u_lat = (u2[0] * lat_axis[0] + u2[1] * lat_axis[1]) * out_sign
            u_down = u2[0] * down_axis[0] + u2[1] * down_axis[1]
            upper_cal = self._calibrated(
                f"{prefix}_u", self.upper_ratio, math.hypot(u_lat, u_down) / width, 1.0
            )
            u3 = self._lift(u_lat, u_down, upper_cal * width, y_sign)
            # Analytic chain inverse. v = R_x(-tilt)·u3; then
            # v = R_y(pitch)·R_x(phi)·(0,0,-1) with joint roll = phi+tilt.
            v = _rot_x(u3, -tilt)
            planar = math.hypot(v[0], v[2])
            phi = math.atan2(v[1], planar)  # branch A: |phi| <= pi/2
            pitch = math.atan2(-v[0], -v[2]) if planar > 1e-9 else 0.0
            if abs(pitch) > 1.65:
                # Overhead branch: the arm crossed the shoulder line; fold
                # roll past 90° and re-derive a near-sagittal pitch.
                phi = math.copysign(math.pi, phi) - phi
                pitch = math.atan2(v[0], v[2])
            pitch = _clamp(pitch, -self.max_pitch, 0.4)
            roll = phi + tilt - y_sign * self.roll_bias
            self._apply(out, pitch_j, pitch)
            self._apply(out, roll_j, roll)

            if isinstance(wrist, dict):
                f2 = self._vec(elbow, wrist)
                f_lat = (f2[0] * lat_axis[0] + f2[1] * lat_axis[1]) * out_sign
                f_down = f2[0] * down_axis[0] + f2[1] * down_axis[1]
                fore_cal = self._calibrated(
                    f"{prefix}_f", self.fore_ratio, math.hypot(f_lat, f_down) / width, 0.9
                )
                f3 = self._lift(f_lat, f_down, fore_cal * width, y_sign)
                # Forearm in the post-roll local frame:
                # f_local = R_x(tilt-roll)·R_y(-pitch)·R_x(-tilt)·f3
                #         = R_z(yaw)·R_y(q)·(1,0,0)
                # -> elbow q from its z, shoulder yaw from its x/y.
                f_local = _rot_x(
                    _rot_y(_rot_x(f3, -tilt), -pitch), tilt - roll
                )
                # Solve f_local = R_z(yaw)·R_y(q)·(a, b, c) with (a, b, c)
                # the true forearm axis: its z gives q via
                # f_z = A·cos(q + delta); its xy phase minus the axis's own
                # phase after R_y(q) gives yaw.
                b = -y_sign * self._fore_b
                q_elbow = (
                    math.acos(_clamp(f_local[2] / self._fore_A, -1.0, 1.0))
                    - self._fore_delta
                )
                self._apply(out, elbow_j, q_elbow)
                in_plane = math.hypot(f_local[0], f_local[1])
                if self._yaw_gate(yaw_j, in_plane):
                    axis_x = (
                        self._fore_a * math.cos(q_elbow)
                        + self._fore_c * math.sin(q_elbow)
                    )
                    yaw = math.atan2(f_local[1], f_local[0]) - math.atan2(b, axis_x)
                    self._apply(out, yaw_j, yaw)
        out = {
            joint: _clamp(value, *MIMIC_LIMITS[joint])
            for joint, value in out.items()
        }
        self._last = dict(out)
        return out

    def _apply(self, out: dict[int, float], joint: int, value: float) -> None:
        if abs(value - out[joint]) > self.dead_band_rad:
            out[joint] = value
