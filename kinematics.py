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
        return {TELEMETRY_TO_URDF_JOINT[name]: float(value) for name, value in angles_rad.items()}

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

        return min(roots, default=best)[2]


def arm_pose_guide(kin: ArmKinematics, limits_by_name: dict[str, tuple[float, float]]) -> str:
    """LLM cheat sheet: probe each arm joint via FK and describe the hand motion."""
    zero = kin.landmarks({})
    lines = [
        "ARM JOINT GUIDE (derived from the URDF; angles in radians; frame: x=forward, y=left, z=up):",
    ]
    for name in ARM_JOINT_NAMES:
        side = "left" if name.startswith("Left") else "right"
        low, high = limits_by_name.get(name, (-math.pi, math.pi))
        probe = min(0.5, high) if high >= 0.1 else max(-0.5, low)
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
        "Combine joints; unspecified joints keep their current angle."
    )
    lines.append("CANONICAL POSES (right-arm angles; MIRROR the sign of Roll and Yaw for the "
                 "left arm, Pitch and Elbow keep the same sign). Interpolate/combine from "
                 "these anchors instead of guessing:")
    anchors = [
        ("hand at rest beside the hip", {}),
        ("hand extended straight forward at shoulder height",
         {"RightShoulderPitch": -1.57, "RightElbow": 1.57}),
        ("hands raised high in the air (above the head)",
         {"RightShoulderPitch": -2.2, "RightShoulderRoll": -0.35}),
        ("arm opened sideways at shoulder height (T-pose)",
         {"RightShoulderRoll": -1.57}),
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
