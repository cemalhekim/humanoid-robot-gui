"""Forward kinematics for the Unitree H1-2 arm chains.

Parses the SAME URDF the browser digital twin renders
(static/models/h1_2_description/h1_2.urdf) and computes shoulder/elbow/hand
landmark positions in the pelvis frame (x=forward, y=left, z=up, meters).
Pure stdlib (xml.etree + math) so the server gains no dependencies. Used to
DRY-RUN LLM-proposed joint angles before anything is allowed to move.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

URDF_PATH = Path(__file__).resolve().parent / "static" / "models" / "h1_2_description" / "h1_2.urdf"

TELEMETRY_TO_URDF_JOINT = {
    "WaistYaw": "torso_joint",
    "LeftShoulderPitch": "left_shoulder_pitch_joint",
    "LeftShoulderRoll": "left_shoulder_roll_joint",
    "LeftShoulderYaw": "left_shoulder_yaw_joint",
    "LeftElbow": "left_elbow_joint",
    "LeftWristRoll": "left_wrist_roll_joint",
    "LeftWristPitch": "left_wrist_pitch_joint",
    "LeftWristYaw": "left_wrist_yaw_joint",
    "RightShoulderPitch": "right_shoulder_pitch_joint",
    "RightShoulderRoll": "right_shoulder_roll_joint",
    "RightShoulderYaw": "right_shoulder_yaw_joint",
    "RightElbow": "right_elbow_joint",
    "RightWristRoll": "right_wrist_roll_joint",
    "RightWristPitch": "right_wrist_pitch_joint",
    "RightWristYaw": "right_wrist_yaw_joint",
}
ARM_JOINT_NAMES = tuple(name for name in TELEMETRY_TO_URDF_JOINT if name != "WaistYaw")
_LANDMARK_LINKS = {"shoulder": "shoulder_pitch_link", "elbow": "elbow_link", "hand": "wrist_yaw_link"}
_ROOT_LINK = "pelvis"

Matrix = list[list[float]]
_IDENTITY: Matrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
# A bisected pitch root lands far below this z-residual; anything under it counts
# as a real solution when choosing the branch nearest the current pitch.
_Z_SOLVE_TOLERANCE_M = 1e-3


def _mat_mul(a: Matrix, b: Matrix) -> Matrix:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat_vec(m: Matrix, v: list[float]) -> list[float]:
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def _rot_rpy(r: float, p: float, y: float) -> Matrix:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # URDF fixed-axis convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _rot_axis(axis: list[float], angle: float) -> Matrix:
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def _parse_floats(text: str | None, count: int, default: float = 0.0) -> list[float]:
    parts = (text or "").split()
    values = [float(p) for p in parts] if parts else []
    return values if len(values) == count else [default] * count


def _ARM_LINK_NAMES(side: str) -> set[str]:
    """The seven arm links proper; everything else below the shoulder is hand."""
    return {f"{side}_{part}_link" for part in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")}


class ArmKinematics:
    """Chain FK from the pelvis to both arms' shoulder/elbow/hand links."""

    def __init__(self, urdf_path: Path = URDF_PATH) -> None:
        xml_root = ET.parse(urdf_path).getroot()
        parent_of: dict[str, tuple[str, dict[str, Any]]] = {}
        for joint in xml_root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            origin = joint.find("origin")
            axis_el = joint.find("axis")
            axis = _parse_floats(axis_el.get("xyz") if axis_el is not None else "1 0 0", 3)
            norm = math.sqrt(sum(a * a for a in axis)) or 1.0
            parent_of[child.get("link")] = (
                parent.get("link"),
                {
                    "name": joint.get("name"),
                    "type": joint.get("type"),
                    "xyz": _parse_floats(origin.get("xyz") if origin is not None else None, 3),
                    "rpy": _parse_floats(origin.get("rpy") if origin is not None else None, 3),
                    "axis": [a / norm for a in axis],
                },
            )
        # Link masses and centres of mass (link frame) for the gravity model.
        self._inertial: dict[str, tuple[float, list[float]]] = {}
        for link in xml_root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                continue
            mass_el = inertial.find("mass")
            origin_el = inertial.find("origin")
            mass = float(mass_el.get("value", "0")) if mass_el is not None else 0.0
            if mass > 0.0:
                self._inertial[link.get("name")] = (
                    mass,
                    _parse_floats(origin_el.get("xyz") if origin_el is not None else None, 3),
                )
        self._parent_of = parent_of
        # Every massive link hanging from each shoulder, with its joint chain from the pelvis.
        self._mass_chains: dict[str, list[tuple[float, list[float], list[dict[str, Any]]]]] = {}
        for side in ("left", "right"):
            root = f"{side}_shoulder_pitch_link"
            entries = []
            for link, (mass, com) in self._inertial.items():
                chain: list[dict[str, Any]] = []
                cursor = link
                in_arm = False
                while cursor != _ROOT_LINK and cursor in parent_of:
                    if cursor == root:
                        in_arm = True
                    cursor, joint = parent_of[cursor][0], parent_of[cursor][1]
                    chain.append(joint)
                if in_arm and cursor == _ROOT_LINK:
                    chain.reverse()
                    entries.append((mass, com, chain, link in _ARM_LINK_NAMES(side)))
            self._mass_chains[side] = entries
        self._chains: dict[str, list[dict[str, Any]]] = {}
        for side in ("left", "right"):
            for part, suffix in _LANDMARK_LINKS.items():
                link = f"{side}_{suffix}"
                chain: list[dict[str, Any]] = []
                cursor = link
                while cursor != _ROOT_LINK:
                    if cursor not in parent_of:
                        raise ValueError(f"URDF link {link} does not chain back to {_ROOT_LINK}")
                    cursor, joint = parent_of[cursor][0], parent_of[cursor][1]
                    chain.append(joint)
                chain.reverse()
                self._chains[f"{side}:{part}"] = chain

    @staticmethod
    def _joint_angles_by_urdf(angles_rad: dict[str, float]) -> dict[str, float]:
        unknown = set(angles_rad) - set(TELEMETRY_TO_URDF_JOINT)
        if unknown:
            raise ValueError(f"Unknown arm joints: {', '.join(sorted(unknown))}")
        by_urdf: dict[str, float] = {}
        for name, value in angles_rad.items():
            number = float(value)
            # A NaN/inf angle would silently poison every landmark (and make the
            # guide mislabel the joint as "reorients in place"); fail loudly.
            if not math.isfinite(number):
                raise ValueError(f"Angle for {name} must be finite, got {value!r}")
            by_urdf[TELEMETRY_TO_URDF_JOINT[name]] = number
        return by_urdf

    def _landmark_position(
        self,
        by_urdf: dict[str, float],
        side: str,
        part: str,
    ) -> dict[str, float]:
        rotation, position = _IDENTITY, [0.0, 0.0, 0.0]
        for joint in self._chains[f"{side}:{part}"]:
            position = [p + o for p, o in zip(position, _mat_vec(rotation, joint["xyz"]))]
            rotation = _mat_mul(rotation, _rot_rpy(*joint["rpy"]))
            if joint["type"] in ("revolute", "continuous"):
                rotation = _mat_mul(
                    rotation,
                    _rot_axis(joint["axis"], by_urdf.get(joint["name"], 0.0)),
                )
        return {"x": position[0], "y": position[1], "z": position[2]}

    @staticmethod
    def _rounded_position(position: dict[str, float], digits: int | None) -> dict[str, float]:
        if digits is None:
            return position
        return {axis: round(value, digits) for axis, value in position.items()}

    def landmark(
        self,
        angles_rad: dict[str, float],
        side: str,
        part: str,
        *,
        round_digits: int | None = 3,
    ) -> dict[str, float]:
        """Return one FK landmark, optionally without display rounding."""
        if side not in ("left", "right") or part not in _LANDMARK_LINKS:
            raise ValueError(f"Unknown landmark: {side}:{part}")
        by_urdf = self._joint_angles_by_urdf(angles_rad)
        return self._rounded_position(
            self._landmark_position(by_urdf, side, part),
            round_digits,
        )

    def landmarks(
        self,
        angles_rad: dict[str, float],
        *,
        round_digits: int | None = 3,
    ) -> dict[str, dict[str, dict[str, float]]]:
        by_urdf = self._joint_angles_by_urdf(angles_rad)
        result: dict[str, dict[str, dict[str, float]]] = {}
        for side in ("left", "right"):
            result[side] = {}
            for part in _LANDMARK_LINKS:
                result[side][part] = self._rounded_position(
                    self._landmark_position(by_urdf, side, part),
                    round_digits,
                )
        return result

    def arm_mass(self, side: str, *, include_hands: bool = True) -> float:
        return sum(mass for mass, _, _, is_arm in self._mass_chains[side] if include_hands or is_arm)

    def _potential_energy(self, by_urdf: dict[str, float], side: str, include_hands: bool, g: float = 9.81) -> float:
        energy = 0.0
        for mass, com, chain, is_arm in self._mass_chains[side]:
            if not include_hands and not is_arm:
                continue
            rotation, position = _IDENTITY, [0.0, 0.0, 0.0]
            for joint in chain:
                position = [p + o for p, o in zip(position, _mat_vec(rotation, joint["xyz"]))]
                rotation = _mat_mul(rotation, _rot_rpy(*joint["rpy"]))
                if joint["type"] in ("revolute", "continuous"):
                    rotation = _mat_mul(rotation, _rot_axis(joint["axis"], by_urdf.get(joint["name"], 0.0)))
            world_com = [p + o for p, o in zip(position, _mat_vec(rotation, com))]
            energy += mass * g * world_com[2]
        return energy

    def gravity_torques(self, angles_rad: dict[str, float], *, include_hands: bool = True, g: float = 9.81) -> dict[str, float]:
        """Joint torque (Nm) each arm motor must apply to hold the arms still against
        gravity at ``angles_rad`` (telemetry joint names): one FK pass per massive
        link, then tau_j = sum_i m_i g (a_j x (c_i - p_j))_z over the links distal to
        joint j (Jacobian transpose), which equals dU/dq_j. Pelvis assumed upright.
        ``include_hands=False`` drops the hand links (the MuJoCo twin is handless; the
        robot carries Inspire hands). ~1 ms for 14 joints, fit for the 120 Hz loop."""
        by_urdf = self._joint_angles_by_urdf(angles_rad)
        wanted = {TELEMETRY_TO_URDF_JOINT[name]: name for name in angles_rad if name != "WaistYaw"}
        torques = {name: 0.0 for name in wanted.values()}
        for side in ("left", "right"):
            for mass, com, chain, is_arm in self._mass_chains[side]:
                if not include_hands and not is_arm:
                    continue
                rotation, position = _IDENTITY, [0.0, 0.0, 0.0]
                pivots: list[tuple[str, list[float], list[float]]] = []
                for joint in chain:
                    position = [p + o for p, o in zip(position, _mat_vec(rotation, joint["xyz"]))]
                    rotation = _mat_mul(rotation, _rot_rpy(*joint["rpy"]))
                    if joint["type"] in ("revolute", "continuous"):
                        if joint["name"] in wanted:
                            pivots.append((joint["name"], list(position), _mat_vec(rotation, joint["axis"])))
                        rotation = _mat_mul(rotation, _rot_axis(joint["axis"], by_urdf.get(joint["name"], 0.0)))
                world_com = [p + o for p, o in zip(position, _mat_vec(rotation, com))]
                weight = mass * g
                for urdf_name, pivot, axis in pivots:
                    rx, ry = world_com[0] - pivot[0], world_com[1] - pivot[1]
                    # z-component of axis x r
                    torques[wanted[urdf_name]] += weight * (axis[0] * ry - axis[1] * rx)
        return torques

    def gravity_torques_fd(self, angles_rad: dict[str, float], *, step: float = 1e-4, include_hands: bool = True) -> dict[str, float]:
        """Reference implementation of gravity_torques by central finite difference of
        the potential energy (slow, ~10 ms); kept for tests."""
        by_urdf = self._joint_angles_by_urdf(angles_rad)
        torques: dict[str, float] = {}
        for name in angles_rad:
            urdf_name = TELEMETRY_TO_URDF_JOINT[name]
            if name == "WaistYaw":
                continue
            side = "left" if name.startswith("Left") else "right"
            plus = dict(by_urdf)
            minus = dict(by_urdf)
            plus[urdf_name] = by_urdf.get(urdf_name, 0.0) + step
            minus[urdf_name] = by_urdf.get(urdf_name, 0.0) - step
            torques[name] = (self._potential_energy(plus, side, include_hands) - self._potential_energy(minus, side, include_hands)) / (2.0 * step)
        return torques

    def solve_hand_z(
        self,
        angles_rad: dict[str, float],
        side: str,
        target_z: float,
        pitch_limits: tuple[float, float],
        *,
        samples: int = 24,
        iterations: int = 18,
    ) -> float:
        """Solve shoulder pitch so the selected hand stays at ``target_z``.

        A coarse bracket scan followed by bisection is robust to the arm
        chain's trigonometric shape and keeps the real-time solve dependency
        free. If the requested height is unreachable, the closest safe pitch
        sample is returned.
        """
        if side not in ("left", "right"):
            raise ValueError(f"Unknown arm side: {side}")
        if not math.isfinite(float(target_z)):
            raise ValueError("target_z must be finite")
        lo, hi = sorted((float(pitch_limits[0]), float(pitch_limits[1])))
        if not math.isfinite(lo) or not math.isfinite(hi) or lo == hi:
            raise ValueError("pitch_limits must contain two distinct finite values")
        samples = max(2, int(samples))
        iterations = max(1, int(iterations))

        pitch_name = f"{side.capitalize()}ShoulderPitch"
        pitch_urdf = TELEMETRY_TO_URDF_JOINT[pitch_name]
        by_urdf = self._joint_angles_by_urdf(angles_rad)
        preferred = max(lo, min(hi, float(angles_rad.get(pitch_name, (lo + hi) * 0.5))))

        def error_at(pitch: float) -> tuple[float, float]:
            by_urdf[pitch_urdf] = pitch
            z = self._landmark_position(by_urdf, side, "hand")["z"]
            return z - target_z, z

        points: list[tuple[float, float]] = []
        best = (math.inf, math.inf, preferred)
        for index in range(samples + 1):
            pitch = lo + (hi - lo) * index / samples
            error, _ = error_at(pitch)
            points.append((pitch, error))
            candidate = (abs(error), abs(pitch - preferred), pitch)
            if candidate < best:
                best = candidate

        roots: list[tuple[float, float, float]] = []
        for (left, left_error), (right, right_error) in zip(points, points[1:]):
            if left_error == 0.0:
                roots.append((0.0, abs(left - preferred), left))
                continue
            if left_error * right_error > 0.0:
                continue
            for _ in range(iterations):
                mid = (left + right) * 0.5
                mid_error, _ = error_at(mid)
                candidate = (abs(mid_error), abs(mid - preferred), mid)
                if candidate < best:
                    best = candidate
                if left_error * mid_error <= 0.0:
                    right, right_error = mid, mid_error
                else:
                    left, left_error = mid, mid_error
            pitch = (left + right) * 0.5
            error, _ = error_at(pitch)
            roots.append((abs(error), abs(pitch - preferred), pitch))

        # When the target height has two valid pitch solutions (a near and a far
        # branch ~pi apart), pick the one CLOSEST to the current pitch. Sorting
        # by residual first made the far branch win on a negligible numeric edge,
        # swinging the shoulder ~180 deg away from where it already was.
        reachable = [root for root in roots if root[0] <= _Z_SOLVE_TOLERANCE_M]
        if reachable:
            return min(reachable, key=lambda root: (root[1], root[0]))[2]
        if roots:
            return min(roots, key=lambda root: root[0])[2]
        return best[2]


def arm_pose_guide(kin: ArmKinematics, limits_by_name: dict[str, tuple[float, float]]) -> str:
    """LLM cheat sheet: probe each arm joint via FK and describe the hand motion."""
    zero = kin.landmarks({})
    lines = [
        "ARM JOINT GUIDE (derived from the URDF; angles in radians; frame: x=forward, y=left, z=up):",
    ]
    for name in ARM_JOINT_NAMES:
        side = "left" if name.startswith("Left") else "right"
        low, high = limits_by_name.get(name, (-math.pi, math.pi))
        # Probe into whichever side has the MOST travel, not just any positive
        # range. RightShoulderRoll limits are [-3.4, 0.38]: the old `high >= 0.1`
        # test probed the 0.38-rad minority side and described the right arm's
        # useful (negative) direction backwards, inconsistent with the left arm.
        if abs(low) > abs(high):
            probe = max(-0.5, low)
        else:
            probe = min(0.5, high)
        hand0, hand1 = zero[side]["hand"], kin.landmarks({name: probe})[side]["hand"]
        delta = {axis: hand1[axis] - hand0[axis] for axis in ("x", "y", "z")}
        parts = []
        if abs(delta["x"]) >= 0.02:
            parts.append("forward" if delta["x"] > 0 else "backward")
        if abs(delta["y"]) >= 0.02:
            parts.append("to the robot's left" if delta["y"] > 0 else "to the robot's right")
        if abs(delta["z"]) >= 0.02:
            parts.append("up" if delta["z"] > 0 else "down")
        if parts:
            effect = f"moves the {side} hand " + " and ".join(parts)
        else:
            effect = f"mostly reorients the {side} hand in place"
        lines.append(f"- {name}: {probe:+.1f} rad {effect}; limits [{low}, {high}]")
    right = zero["right"]
    lines.append(
        "Zero pose (all angles 0): upper arms hang down, elbows are bent ~90 deg so the "
        f"forearms point forward; each hand starts near x={right['hand']['x']:.2f}, "
        f"z={right['hand']['z']:.2f} relative to the pelvis, shoulders at z={right['shoulder']['z']:.2f}. "
        "ELBOW CONVENTION: Elbow 0 = the natural ~90 deg bend, Elbow +1.57 = arm fully "
        "STRAIGHT (use it whenever the operator wants open/straight/unbent arms). "
        "Combine joints; unspecified joints keep their current angle."
    )
    lines.append("CANONICAL POSES (right-arm angles; MIRROR the sign of Roll and Yaw for the "
                 "left arm, Pitch and Elbow keep the same sign). Interpolate/combine from "
                 "these anchors instead of guessing:")
    anchors = [
        ("hand at rest beside the hip", {}),
        ("hand extended straight forward at shoulder height",
         {"RightShoulderPitch": -1.57, "RightElbow": 1.57}),
        ("hands raised high in the air (elbows keep their natural bend)",
         {"RightShoulderPitch": -2.2, "RightShoulderRoll": -0.35}),
        ("arm raised STRAIGHT up above the head",
         {"RightShoulderPitch": -2.6, "RightShoulderRoll": -0.35, "RightElbow": 1.57}),
        ("arm opened sideways STRAIGHT at shoulder height (true T-pose, no elbow bend)",
         {"RightShoulderRoll": -1.57, "RightElbow": 1.57}),
        ("arm crossed in front of the chest (hand on the opposite side)",
         {"RightShoulderPitch": -1.6, "RightShoulderRoll": 0.38,
          "RightShoulderYaw": 1.5, "RightElbow": 1.2}),
    ]
    for label, angles in anchors:
        hand = kin.landmarks(angles)["right"]["hand"]
        joint_text = ", ".join(f"{name}: {value:+.2f}" for name, value in angles.items()) or "all joints 0"
        lines.append(
            f"- {label}: {{{joint_text}}} -> hand at x={hand['x']:.2f}, y={hand['y']:.2f}, z={hand['z']:.2f}"
        )
        if angles:
            both = {}
            for name, value in angles.items():
                left = name.replace("Right", "Left")
                both[left] = -value if ("Roll" in name or "Yaw" in name) else value
                both[name] = value
            both_text = ", ".join(f"{name}: {value:+.2f}" for name, value in sorted(both.items()))
            lines.append(f"  BOTH arms version (note the mirrored Roll/Yaw signs): {{{both_text}}}")
    return "\n".join(lines)
