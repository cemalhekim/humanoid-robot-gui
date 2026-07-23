# LLM Arm-Pose Proposals with Digital-Twin Preview — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The dashboard LLM (qwen3:30b) turns "extend your hand forward" into concrete arm joint angles, previews them as a green simulated twin next to the live model without moving the robot, and executes them through the existing safety-gated closed-loop arm replay only after the operator says "okay".

**Architecture:** A new pure-python forward-kinematics module (`kinematics.py`) parses the same URDF the browser twin uses and predicts hand positions for proposed angles. A new guarded-adjacent chat tool `propose_arm_pose` clamps LLM-guessed angles to `JOINT_LIMITS`, runs FK + the existing `semantic_arm_pose` interpreter, and stores the proposal in `TelemetryStore`; the proposal rides the existing `/api/state`→`/events` snapshot into the browser, where the live viewer shows it as a green ("trajectory"-tone) ghost. The `move` tool loses its saved-position catalog: it now accepts only `"home"` (saved recording) or `"proposed"` (executes the pending proposal as an inline ephemeral pose through `request_robot_replay`, so every arm_sdk safety gate applies unchanged).

**Tech Stack:** Python 3 stdlib only (xml.etree, math, unittest), vanilla JS + three.js viewer, Ollama qwen3:30b on AI-DEV.

## Global Constraints

- **NO pushes to `origin/main`** — the robot auto-pulls main. All work on branch `feature/llm-pose-proposals`; push only that branch. (User order 2026-07-23: local testing only for a long while.)
- Commit after every task (CLAUDE.md), trailer: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
- Test command: `python3 -m unittest discover -s tests -p 'test_*.py'`. Release gate at the end: `make production-gate`.
- All safety interlocks stay intact: proposals NEVER move the robot; execution goes exclusively through `request_robot_replay` (`execute_arm_sdk`, `command_scope="arms"`, closed-loop) with `confirm=true`, exactly like the old `move`.
- Real-LLM integration tests run against `http://10.2.125.3:11434` (qwen3:30b-a3b-instruct-2507-q4_K_M) from the local machine; the local server has no DDS, so execution attempts must end at the "DDS arm_sdk publisher is not available" gate — that is the expected local result.
- No new dependencies. No numpy.

## File Structure

- Create `kinematics.py` — URDF parse + FK for both arm chains + auto-derived joint-convention guide. Root-level module like `tracking.py`/`distance.py`.
- Create `tests/test_kinematics.py`.
- Modify `server.py` — proposal state + `propose_arm_pose` tool + snapshot field + `move` rework + prompt update.
- Modify `tests/test_chat.py`, `tests/test_mcp.py` — adapt move-tool tests, add proposal tests.
- Modify `static/viewer.js` — green proposal ghost in the live viewer.
- Modify `static/index.html` — cache-bust `?v=` for app.js/viewer.js.

---

### Task 1: `kinematics.py` — URDF forward kinematics

**Files:**
- Create: `kinematics.py`
- Test: `tests/test_kinematics.py`

**Interfaces:**
- Produces: `ArmKinematics()` (parses `static/models/h1_2_description/h1_2.urdf` by default), method `landmarks(angles_rad: dict[str, float]) -> dict` returning `{"left": {"shoulder": {x,y,z}, "elbow": {...}, "hand": {...}}, "right": {...}}` in the pelvis frame (x=forward, y=left, z=up, meters, rounded to 3 decimals). Keys of `angles_rad` are telemetry joint names (`LeftShoulderPitch` … `RightWristYaw`, plus `WaistYaw`); unknown keys raise `ValueError`; unspecified joints default to 0. Also `TELEMETRY_TO_URDF_JOINT: dict[str, str]` and `ARM_JOINT_NAMES: tuple[str, ...]` (the 14 arm joints, no waist).

- [ ] **Step 1: Write the failing tests**

`tests/test_kinematics.py`:

```python
from __future__ import annotations

import math
import unittest

import kinematics


def dist(a: dict, b: dict) -> float:
    return math.sqrt(sum((a[k] - b[k]) ** 2 for k in ("x", "y", "z")))


class ArmKinematicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.kin = kinematics.ArmKinematics()

    def test_zero_pose_is_left_right_symmetric(self) -> None:
        marks = self.kin.landmarks({})
        for part in ("shoulder", "elbow", "hand"):
            left, right = marks["left"][part], marks["right"][part]
            self.assertAlmostEqual(left["x"], right["x"], delta=0.02, msg=part)
            self.assertAlmostEqual(left["y"], -right["y"], delta=0.02, msg=part)
            self.assertAlmostEqual(left["z"], right["z"], delta=0.02, msg=part)
        self.assertGreater(marks["left"]["shoulder"]["y"], 0.05)   # left = +y
        self.assertLess(marks["right"]["shoulder"]["y"], -0.05)

    def test_zero_pose_hand_hangs_below_shoulder(self) -> None:
        marks = self.kin.landmarks({})
        for side in ("left", "right"):
            self.assertLess(marks[side]["hand"]["z"], marks[side]["shoulder"]["z"])

    def test_elbow_bend_moves_hand_but_not_elbow(self) -> None:
        straight = self.kin.landmarks({})["right"]
        bent = self.kin.landmarks({"RightElbow": 1.2})["right"]
        self.assertGreater(dist(straight["hand"], bent["hand"]), 0.10)
        self.assertLess(dist(straight["elbow"], bent["elbow"]), 0.001)

    def test_link_lengths_are_rigid(self) -> None:
        a = self.kin.landmarks({})["right"]
        b = self.kin.landmarks({"RightShoulderPitch": -1.0, "RightElbow": 1.3})["right"]
        self.assertAlmostEqual(dist(a["shoulder"], a["elbow"]), dist(b["shoulder"], b["elbow"]), delta=0.001)
        self.assertAlmostEqual(dist(a["elbow"], a["hand"]), dist(b["elbow"], b["hand"]), delta=0.001)

    def test_mirrored_pitch_elbow_gives_mirrored_hands(self) -> None:
        left = self.kin.landmarks({"LeftShoulderPitch": -0.8, "LeftElbow": 0.9})["left"]["hand"]
        right = self.kin.landmarks({"RightShoulderPitch": -0.8, "RightElbow": 0.9})["right"]["hand"]
        self.assertAlmostEqual(left["x"], right["x"], delta=0.02)
        self.assertAlmostEqual(left["y"], -right["y"], delta=0.02)
        self.assertAlmostEqual(left["z"], right["z"], delta=0.02)

    def test_unknown_or_non_arm_joint_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.kin.landmarks({"LeftKnee": 0.2})
        with self.assertRaises(ValueError):
            self.kin.landmarks({"NoSuchJoint": 0.2})

    def test_waist_yaw_swings_both_hands_laterally(self) -> None:
        base = self.kin.landmarks({})
        turned = self.kin.landmarks({"WaistYaw": 0.6})
        self.assertGreater(dist(base["left"]["hand"], turned["left"]["hand"]), 0.05)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m unittest tests.test_kinematics -v`
Expected: FAIL/ERROR with `ModuleNotFoundError: No module named 'kinematics'`

- [ ] **Step 3: Implement `kinematics.py`**

```python
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

    def landmarks(self, angles_rad: dict[str, float]) -> dict[str, dict[str, dict[str, float]]]:
        unknown = set(angles_rad) - set(TELEMETRY_TO_URDF_JOINT)
        if unknown:
            raise ValueError(f"Unknown arm joints: {', '.join(sorted(unknown))}")
        by_urdf = {TELEMETRY_TO_URDF_JOINT[name]: float(value) for name, value in angles_rad.items()}
        result: dict[str, dict[str, dict[str, float]]] = {}
        for side in ("left", "right"):
            result[side] = {}
            for part in _LANDMARK_LINKS:
                rotation, position = _IDENTITY, [0.0, 0.0, 0.0]
                for joint in self._chains[f"{side}:{part}"]:
                    position = [p + o for p, o in zip(position, _mat_vec(rotation, joint["xyz"]))]
                    rotation = _mat_mul(rotation, _rot_rpy(*joint["rpy"]))
                    if joint["type"] in ("revolute", "continuous"):
                        rotation = _mat_mul(rotation, _rot_axis(joint["axis"], by_urdf.get(joint["name"], 0.0)))
                result[side][part] = {
                    "x": round(position[0], 3), "y": round(position[1], 3), "z": round(position[2], 3),
                }
        return result
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m unittest tests.test_kinematics -v`
Expected: all 7 tests PASS. If symmetry fails, print `ArmKinematics().landmarks({})` and inspect — a sign error in `_rot_rpy` is the usual culprit; do NOT loosen the deltas.

- [ ] **Step 5: Run the full suite, commit**

Run: `python3 -m unittest discover -s tests -p 'test_*.py'` — all pass.

```bash
git add kinematics.py tests/test_kinematics.py
git commit -m "Add pure-python URDF forward kinematics for the arm chains"
```

---

### Task 2: Auto-derived joint-convention guide for the LLM

**Files:**
- Modify: `kinematics.py` (append function)
- Test: `tests/test_kinematics.py` (append class)

**Interfaces:**
- Produces: `arm_pose_guide(kin: ArmKinematics, limits_by_name: dict[str, tuple[float, float]]) -> str` — human/LLM-readable cheat sheet, one line per arm joint, describing which way a positive angle moves that hand, derived by probing FK (never hand-written, so it can't be wrong).

- [ ] **Step 1: Write the failing tests** (append to `tests/test_kinematics.py`)

```python
class ArmPoseGuideTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.kin = kinematics.ArmKinematics()
        cls.limits = {name: (-1.0, 1.0) for name in kinematics.ARM_JOINT_NAMES}
        cls.guide = kinematics.arm_pose_guide(cls.kin, cls.limits)

    def test_every_arm_joint_is_documented_with_limits(self) -> None:
        for name in kinematics.ARM_JOINT_NAMES:
            self.assertIn(name, self.guide)
        self.assertIn("[-1.0, 1.0]", self.guide)

    def test_axes_convention_is_stated(self) -> None:
        self.assertIn("x=forward, y=left, z=up", self.guide)

    def test_shoulder_pitch_lines_mention_a_direction(self) -> None:
        for line in self.guide.splitlines():
            if "ShoulderPitch" in line:
                self.assertTrue(any(w in line for w in ("forward", "backward", "up", "down")), line)
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m unittest tests.test_kinematics.ArmPoseGuideTest -v`
Expected: ERROR `AttributeError: module 'kinematics' has no attribute 'arm_pose_guide'`

- [ ] **Step 3: Implement** (append to `kinematics.py`)

```python
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
        effect = " and ".join(parts) or "mostly reorients the hand in place"
        lines.append(f"- {name}: {probe:+.1f} rad moves the {side} hand {effect}; limits [{low}, {high}]")
    lines.append(
        "At all-zero angles both arms hang straight down at the robot's sides. "
        "Combine joints; unspecified joints keep their current angle."
    )
    return "\n".join(lines)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m unittest tests.test_kinematics -v` — all pass.

- [ ] **Step 5: Sanity-read the guide, commit**

Run: `python3 -c "import kinematics, json; k=kinematics.ArmKinematics(); print(kinematics.arm_pose_guide(k, {n: (-3.14, 3.14) for n in kinematics.ARM_JOINT_NAMES}))"`
Read the output: LeftShoulderRoll/RightShoulderRoll must describe opposite lateral directions; ShoulderPitch lines must agree with each other. Then:

```bash
git add kinematics.py tests/test_kinematics.py
git commit -m "Auto-derive the LLM arm-joint convention guide from URDF kinematics"
```

---

### Task 3: `propose_arm_pose` tool + shared proposal state in the server

**Files:**
- Modify: `server.py`
- Test: `tests/test_chat.py`

**Interfaces:**
- Consumes: `kinematics.ArmKinematics.landmarks`, `kinematics.arm_pose_guide`, existing `semantic_arm_pose(hands)` (expects `hands[side]["landmarks_robot_m"]`), `TelemetryStore._clamp_joint_target(index, q)`.
- Produces: module globals `ARM_JOINT_INDEX_BY_NAME: dict[str, int]` (names of joints 13–26), `ARM_PROPOSAL_TTL_SECONDS = 300.0`, `ARM_KINEMATICS`, `LLM_ARM_GUIDE: str`; `propose_tool_spec() -> dict`; store methods `propose_arm_pose(arguments) -> dict` and `arm_proposal_public() -> dict | None`; snapshot key `"arm_proposal"` shaped `{"id", "age_seconds", "targets": [{"index", "name", "q"}]}` or `None`.

- [ ] **Step 1: Write the failing tests** (append to `tests/test_chat.py`)

```python
class ProposeArmPoseTest(unittest.TestCase):
    def setUp(self) -> None:
        self.store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_propose_clamps_stores_and_predicts(self) -> None:
        result = self.store.run_chat_tool(
            "propose_arm_pose", {"joints": {"RightShoulderPitch": -1.4, "RightElbow": 9.0}}
        )
        self.assertTrue(result["ok"])
        self.assertTrue(result["moved_nothing"])
        self.assertEqual(result["targets_rad"]["RightElbow"], 3.18)  # clamped to JOINT_LIMITS
        self.assertTrue(any("RightElbow" in c for c in result["clamped_to_limits"]))
        self.assertIn("right", result["predicted_landmarks_m"])
        self.assertIn("arms", result["predicted_semantics"])
        public = self.store.arm_proposal_public()
        self.assertIsNotNone(public)
        by_name = {t["name"]: t for t in public["targets"]}
        self.assertEqual(by_name["RightShoulderPitch"]["q"], -1.4)
        self.assertEqual(by_name["RightShoulderPitch"]["index"], 20)

    def test_propose_rejects_unknown_and_non_finite(self) -> None:
        self.assertFalse(self.store.run_chat_tool("propose_arm_pose", {"joints": {"LeftKnee": 0.2}})["ok"])
        self.assertFalse(self.store.run_chat_tool("propose_arm_pose", {"joints": {"LeftElbow": float("nan")}})["ok"])
        self.assertFalse(self.store.run_chat_tool("propose_arm_pose", {"joints": {}})["ok"])
        self.assertIsNone(self.store.arm_proposal_public())

    def test_clear_discards_proposal(self) -> None:
        self.store.run_chat_tool("propose_arm_pose", {"joints": {"LeftElbow": 1.0}})
        result = self.store.run_chat_tool("propose_arm_pose", {"clear": True})
        self.assertTrue(result["ok"])
        self.assertIsNone(self.store.arm_proposal_public())

    def test_snapshot_exposes_active_proposal_and_ttl_expiry(self) -> None:
        self.store.run_chat_tool("propose_arm_pose", {"joints": {"LeftElbow": 1.0}})
        self.assertIsNotNone(self.store.snapshot()["arm_proposal"])
        with self.store.proposal_lock:
            self.store.arm_proposal["created_at"] -= server.ARM_PROPOSAL_TTL_SECONDS + 1
        self.assertIsNone(self.store.snapshot()["arm_proposal"])

    def test_tool_spec_offered_when_move_enabled(self) -> None:
        names = [spec["function"]["name"] for spec in self.store.chat_tool_specs()]
        self.assertIn("propose_arm_pose", names)
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m unittest tests.test_chat.ProposeArmPoseTest -v`
Expected: failures — `run_chat_tool` returns `{"ok": False, "error": "Unknown tool: propose_arm_pose"}`.

- [ ] **Step 3: Implement in `server.py`**

3a. Import next to the other local-module imports (`tracking`, `distance` are imported around the top; add alongside):

```python
import kinematics
```

3b. Constants after `JOINT_LIMITS` (below line ~320):

```python
ARM_JOINT_INDEX_BY_NAME = {name: index for index, name in JOINT_NAMES.items() if 13 <= index <= 26}
ARM_PROPOSAL_TTL_SECONDS = 300.0
try:
    ARM_KINEMATICS: kinematics.ArmKinematics | None = kinematics.ArmKinematics()
    LLM_ARM_GUIDE = kinematics.arm_pose_guide(
        ARM_KINEMATICS,
        {name: JOINT_LIMITS[index] for name, index in ARM_JOINT_INDEX_BY_NAME.items()},
    )
except Exception as _kin_exc:  # URDF missing on exotic installs: chat works, pose flow reports it
    ARM_KINEMATICS = None
    LLM_ARM_GUIDE = f"(arm kinematics unavailable: {_kin_exc})"
```

3c. Tool spec, next to `move_tool_spec`:

```python
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
```

3d. `TelemetryStore.__init__` — next to the `spatial_lock` block added on 2026-07-23:

```python
        self.proposal_lock = threading.Lock()
        self.arm_proposal: dict[str, Any] | None = None
```

3e. Store methods, next to `update_spatial_pose`/`spatial_pose_snapshot`:

```python
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
        proposal = {
            "id": f"pose-{time.monotonic_ns()}",
            "created_at": time.time(),
            "requested": dict(targets),
            "targets": full,
        }
        with self.proposal_lock:
            self.arm_proposal = proposal
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

    def arm_proposal_public(self) -> dict[str, Any] | None:
        """Proposal summary for /api/state and the browser ghost; expires by TTL."""
        with self.proposal_lock:
            proposal = self.arm_proposal
        if not proposal:
            return None
        age = time.time() - proposal["created_at"]
        if age > ARM_PROPOSAL_TTL_SECONDS:
            with self.proposal_lock:
                if self.arm_proposal is proposal:
                    self.arm_proposal = None
            return None
        return {
            "id": proposal["id"],
            "age_seconds": round(age, 1),
            "targets": [
                {"index": ARM_JOINT_INDEX_BY_NAME[name], "name": name, "q": q}
                for name, q in sorted(proposal["targets"].items(), key=lambda kv: ARM_JOINT_INDEX_BY_NAME[kv[0]])
            ],
        }
```

3f. `snapshot()` return dict gains one line:

```python
            "arm_proposal": self.arm_proposal_public(),
```

3g. `run_chat_tool` dispatch, next to `get_spatial_pose`:

```python
            if name == "propose_arm_pose":
                return self.propose_arm_pose(arguments)
```

3h. `chat_tool_specs()` — inside the existing `if LLM_TOOL_MOVE_ENABLED:` branch add `specs.append(propose_tool_spec())` (the `positions` handling changes in Task 4; for now keep the rest as is).

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m unittest tests.test_chat.ProposeArmPoseTest -v` — 5 PASS.

- [ ] **Step 5: Full suite, commit**

Run: `python3 -m unittest discover -s tests -p 'test_*.py'` — all pass (existing suites must not regress; `snapshot()` gained a key, which is additive).

```bash
git add server.py tests/test_chat.py
git commit -m "Add propose_arm_pose: clamp LLM joint guesses, FK dry-run, staged preview state"
```

---

### Task 4: `move` rework — only `proposed` and `home`

**Files:**
- Modify: `server.py` (`move_tool_spec`, `_tool_move`, `chat_tool_specs`, `spatial_pose_snapshot`)
- Test: `tests/test_chat.py`, `tests/test_mcp.py`

**Interfaces:**
- Consumes: `arm_proposal_public()`/`self.arm_proposal` from Task 3; existing `request_robot_replay(payload)` inline-snapshot path (`payload["snapshot"]` with a `motors` list of `{"index", "name", "q"}` dicts → ephemeral `.pose.json` → full safety pipeline).
- Produces: `move_tool_spec() -> dict` (NO arguments — signature change; enum `["proposed", "home"]`); `_tool_move` behavior: `home` replays the saved recording named `home` (the only surviving saved-position path), `proposed` executes the pending proposal and clears it on success.

- [ ] **Step 1: Adapt/extend the failing tests**

In `tests/test_chat.py`, find the existing move-tool tests (search `"move"` / `named_positions`); update them to the new contract and add:

```python
class MoveProposedTest(unittest.TestCase):
    def setUp(self) -> None:
        self.store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_move_requires_confirm_and_known_position(self) -> None:
        self.assertFalse(self.store.run_chat_tool("move", {"position": "proposed"})["ok"])
        result = self.store.run_chat_tool("move", {"position": "wave", "confirm": True})
        self.assertFalse(result["ok"])
        self.assertIn("proposed", result["error"])
        self.assertIn("home", result["error"])

    def test_move_proposed_without_proposal_fails(self) -> None:
        result = self.store.run_chat_tool("move", {"position": "proposed", "confirm": True})
        self.assertFalse(result["ok"])
        self.assertIn("propose_arm_pose", result["error"])

    def test_move_proposed_executes_inline_snapshot_and_clears(self) -> None:
        self.store.run_chat_tool("propose_arm_pose", {"joints": {"RightElbow": 1.5, "RightShoulderPitch": -1.0}})
        with mock.patch.object(
            self.store, "request_robot_replay", return_value=(200, {"ok": True})
        ) as replay:
            result = self.store.run_chat_tool("move", {"position": "proposed", "confirm": True})
        self.assertTrue(result["ok"])
        payload = replay.call_args[0][0]
        self.assertTrue(payload["execute_arm_sdk"])
        self.assertEqual(payload["command_scope"], "arms")
        self.assertNotIn("filename", payload)
        q_by_index = {m["index"]: m["q"] for m in payload["snapshot"]["motors"]}
        self.assertEqual(q_by_index[23], 1.5)   # RightElbow
        self.assertEqual(q_by_index[20], -1.0)  # RightShoulderPitch
        self.assertIsNone(self.store.arm_proposal_public())  # consumed on success

    def test_move_proposed_keeps_proposal_on_failure(self) -> None:
        self.store.run_chat_tool("propose_arm_pose", {"joints": {"RightElbow": 1.5}})
        with mock.patch.object(
            self.store, "request_robot_replay", return_value=(503, {"ok": False, "error": "no DDS"})
        ):
            result = self.store.run_chat_tool("move", {"position": "proposed", "confirm": True})
        self.assertFalse(result["ok"])
        self.assertIsNotNone(self.store.arm_proposal_public())  # operator can retry

    def test_move_tool_spec_offers_only_proposed_and_home(self) -> None:
        specs = {s["function"]["name"]: s for s in self.store.chat_tool_specs()}
        self.assertIn("move", specs)  # offered even with zero saved positions now
        enum = specs["move"]["function"]["parameters"]["properties"]["position"]["enum"]
        self.assertEqual(enum, ["proposed", "home"])
```

In `tests/test_mcp.py`, extend the tools-list test: descriptor names must include `propose_arm_pose` and `move`, and the `move` descriptor's `inputSchema` enum must equal `["proposed", "home"]`.

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m unittest tests.test_chat.MoveProposedTest tests.test_mcp -v`
Expected: FAIL (old enum built from saved positions; `move` absent without saved positions; proposed unknown).

- [ ] **Step 3: Implement in `server.py`**

3a. Replace `move_tool_spec(positions)` entirely:

```python
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
```

3b. In `chat_tool_specs()` replace the move branch:

```python
        if LLM_TOOL_MOVE_ENABLED:
            specs.append(propose_tool_spec())
            specs.append(move_tool_spec())
```

(The `positions = sorted(self.named_positions())` lookup and the `if positions:` gate are deleted.)

3c. Replace the body of `_tool_move` after the confirm gate:

```python
        raw = arguments.get("position")
        if not isinstance(raw, str) or not raw.strip():
            return {"ok": False, "error": "Provide position: 'proposed' or 'home'."}
        wanted = normalize_position_name(raw)

        if wanted == "home":
            filename = self.named_positions().get("home")
            if not filename:
                return {"ok": False, "error": "No saved 'home' pose exists. Save one from the dashboard first."}
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
        if not proposal or (time.time() - proposal["created_at"]) > ARM_PROPOSAL_TTL_SECONDS:
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
        self.record_command_event(
            "chat_move",
            {"source": "chat", "position": "proposed", "proposal_id": proposal["id"], "status": status},
        )
        if isinstance(result, dict):
            result = {key: value for key, value in result.items() if key != "plan"}
        return {"ok": executed, "status": status, "position": "proposed", **result}
```

3d. In `spatial_pose_snapshot()` update `target_interface` to the new contract:

```python
            "target_interface": {
                "tool": "move",
                "contract": {"position": "proposed | home", "confirm": True},
                "workflow": "propose_arm_pose stages a pose (green preview) -> operator approves -> move",
                "pending_proposal": self.arm_proposal_public(),
            },
```

3e. Grep for now-dead references: `move_tool_spec(` callers, tests asserting `available_positions`, and the `extract_textual_tool_call` move fallback (it still works — shape `{"position", "confirm"}` is unchanged). Update any test asserting the old enum/description.

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m unittest tests.test_chat tests.test_mcp -v` — all pass, including adapted legacy move tests.

- [ ] **Step 5: Full suite, commit**

```bash
git add server.py tests/test_chat.py tests/test_mcp.py
git commit -m "move tool now executes the staged proposal or home only"
```

---

### Task 5: Prompt preparation for qwen

**Files:**
- Modify: `server.py` (`LLM_TOOLS_PROMPT`, chat handler system-prompt assembly)
- Test: `tests/test_chat.py`

**Interfaces:**
- Consumes: `LLM_ARM_GUIDE` (Task 3).
- Produces: rewritten `LLM_TOOLS_PROMPT` describing the propose→preview→okay→move workflow; the tools-enabled system prompt ends with the ARM JOINT GUIDE.

- [ ] **Step 1: Write the failing tests** (append to `tests/test_chat.py`)

```python
class PosePromptTest(unittest.TestCase):
    def test_tools_prompt_teaches_the_proposal_workflow(self) -> None:
        for phrase in ("propose_arm_pose", "green", "position': 'proposed'", "okay"):
            self.assertIn(phrase.replace("': '", "':'").replace("'", "'"), server.LLM_TOOLS_PROMPT.replace('"', "'"))
        self.assertNotIn("saved named position", server.LLM_TOOLS_PROMPT)

    def test_arm_guide_reaches_the_system_prompt(self) -> None:
        self.assertIn("ARM JOINT GUIDE", server.LLM_TOOLS_PROMPT + server.LLM_ARM_GUIDE)
        self.assertIn("RightShoulderPitch", server.LLM_ARM_GUIDE)
```

(Keep assertions loose on wording; pin only the load-bearing tokens: tool names, 'green', 'proposed'.)

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m unittest tests.test_chat.PosePromptTest -v` — FAIL (old prompt has the saved-position text).

- [ ] **Step 3: Rewrite the move paragraph of `LLM_TOOLS_PROMPT`**

Replace from "move drives the arms to a saved named position…" through the closing example with:

```python
    "The arm-pose workflow has two steps. STEP 1 — when the operator asks for an arm "
    "pose in any language ('extend your hand forward', 'elini one uzat', 'streck die "
    "Hand nach vorne'), do NOT ask questions: estimate target joint angles in radians "
    "from the ARM JOINT GUIDE below, call propose_arm_pose, and check the returned "
    "predicted_semantics against the request. If they disagree, correct the angles and "
    "call propose_arm_pose again (up to 3 attempts). When they agree, tell the operator "
    "in one short sentence to check the GREEN preview twin and confirm. The robot has "
    "NOT moved yet — never claim it did. STEP 2 — only when the operator then approves "
    "(okay/tamam/evet/yes/ja/onayla), call move {\"position\": \"proposed\", \"confirm\": true}. "
    "A request to go back to rest ('go home' / 'home pozisyonuna don' / 'Grundstellung') "
    "is the one direct move: call move {\"position\": \"home\", \"confirm\": true} immediately. "
    "Never claim the robot moved or is moving unless move returned ok=true.\n"
    "Example: operator says 'elini one uzat' -> propose_arm_pose with your angle "
    "estimate -> reply 'Yesil onizlemeye bak, onayliyor musun?' -> operator says "
    "'tamam' -> move {\"position\": \"proposed\", \"confirm\": true}."
```

Then append the guide where the behavior prompt is assembled in the chat handler (the `behavior = LLM_TOOLS_PROMPT if LLM_TOOLS_ENABLED else LLM_READONLY_PROMPT` line):

```python
        behavior = (LLM_TOOLS_PROMPT + "\n\n" + LLM_ARM_GUIDE) if LLM_TOOLS_ENABLED else LLM_READONLY_PROMPT
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m unittest tests.test_chat -v` — all pass.

- [ ] **Step 5: Full suite, commit**

```bash
git add server.py tests/test_chat.py
git commit -m "Teach qwen the propose->preview->okay->move workflow with the URDF-derived joint guide"
```

---

### Task 6: Green proposal ghost in the live viewer

**Files:**
- Modify: `static/viewer.js`
- Modify: `static/index.html` (cache-bust)

**Interfaces:**
- Consumes: snapshot key `arm_proposal` (`{targets: [{name, q}]}` or null) arriving via the existing `telemetry-state` event; existing `buildRobot(xml, {name, tone, targetGroups})` with tone `"trajectory"` (green transparent, 0x1dff75); `BODY_JOINTS` name map; `setJointValueIn(groups, urdfJoint, value)`.

- [ ] **Step 1: Build the ghost model in the live viewer**

In the `RobotViewer` constructor (near `this.referenceJointGroups = new Map();`):

```js
    this.proposalJointGroups = new Map();
    this.proposalRoot = null;
```

In `loadRobot()`, in the non-compare `else` branch (live/main viewer):

```js
    } else {
      this.robotRoot = this.buildRobot(xml, { name: "h1_2", tone: "default", targetGroups: this.jointGroups });
      if (this.live) {
        // LLM pose proposals: green simulated twin, hidden until a proposal is staged.
        this.proposalRoot = this.buildRobot(xml, {
          name: "h1_2_proposal",
          tone: "trajectory",
          targetGroups: this.proposalJointGroups,
        });
        this.proposalRoot.visible = false;
      }
    }
```

- [ ] **Step 2: Drive the ghost from snapshots**

At the end of `applyTelemetry(snapshot, source)` (inside, before `setFields`), add:

```js
    if (this.live && this.proposalRoot) {
      const proposal = snapshot.arm_proposal;
      const active = Boolean(proposal && Array.isArray(proposal.targets) && proposal.targets.length);
      this.proposalRoot.visible = active;
      if (active) {
        // Ghost mirrors the live body, then the proposed arm targets override.
        for (const motor of snapshot.motors || []) {
          const urdfJoint = BODY_JOINTS[motor.name];
          if (urdfJoint) this.setJointValueIn(this.proposalJointGroups, urdfJoint, motor.q);
        }
        for (const target of proposal.targets) {
          const urdfJoint = BODY_JOINTS[target.name];
          if (urdfJoint) this.setJointValueIn(this.proposalJointGroups, urdfJoint, target.q);
        }
      }
    }
```

- [ ] **Step 3: Cache-bust**

In `static/index.html` change `app.js?v=twin-evidence-v1` → `app.js?v=pose-proposal-v1` and `viewer.js?v=shared-spatial-v1` → `viewer.js?v=pose-proposal-v1`.

- [ ] **Step 4: Verify in a local browser**

Run `python3 server.py` locally (no robot needed), open `http://localhost:<port>/`, then in a second terminal:

```bash
curl -s -X POST localhost:<port>/api/chat -H 'Content-Type: application/json' \
  -d '{"messages": [{"role": "user", "content": "propose your right arm straight forward"}]}'
```

(or stage directly for a UI-only check via a tiny python: `store` isn't reachable — instead use the chat; the LLM path is Task 7. For a pure UI check, temporarily POST a proposal by calling propose through `/api/chat` tool flow, or run the unittest then screenshot.) Confirm: green transparent H1-2 appears overlapping the gray live model with the right arm forward; disappears after `propose_arm_pose {clear: true}` or TTL. Check the browser console for errors.

- [ ] **Step 5: Full suite, commit**

Run: `python3 -m unittest discover -s tests -p 'test_*.py'`

```bash
git add static/viewer.js static/index.html
git commit -m "Live viewer renders staged LLM pose proposals as a green simulated twin"
```

---

### Task 7: Local end-to-end verification against the real qwen (NO robot)

**Files:**
- Create: `tools/test_llm_pose_flow.py` (manual integration script, not part of the unit suite)

**Interfaces:**
- Consumes: running local server (`python3 server.py`), AI-DEV Ollama at `http://10.2.125.3:11434`.

- [ ] **Step 1: Write the manual integration script**

```python
"""Manual integration test: LLM pose-proposal flow against the real qwen.

Run with a local server up (python3 server.py). Requires AI-DEV reachable.
Verifies the full chain WITHOUT any robot: propose staged via natural language,
snapshot exposes it, and execution stops at the expected no-DDS gate.

Usage: python3 tools/test_llm_pose_flow.py [http://localhost:8000]
"""
from __future__ import annotations

import json
import sys
import urllib.request

BASE = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:8000"


def post_chat(messages):
    body = json.dumps({"messages": messages}).encode()
    req = urllib.request.Request(f"{BASE}/api/chat", data=body, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=180) as response:
        return json.load(response)


def get(path):
    with urllib.request.urlopen(f"{BASE}{path}", timeout=10) as response:
        return json.load(response)


def check(label, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {label}  {detail}")
    return ok


def main() -> int:
    failures = 0
    history = [{"role": "user", "content": "elini one dogru uzat"}]
    reply = post_chat(history)
    tools = [t.get("tool") for t in reply.get("tools_used", [])]
    failures += not check("qwen called propose_arm_pose", "propose_arm_pose" in tools, str(tools))
    failures += not check("qwen did NOT move yet", "move" not in tools, str(tools))
    proposal = get("/api/state").get("arm_proposal")
    failures += not check("snapshot exposes proposal", bool(proposal), json.dumps(proposal)[:120])

    history += [{"role": "assistant", "content": reply.get("reply", "")},
                {"role": "user", "content": "okay"}]
    reply2 = post_chat(history)
    tools2 = [t.get("tool") for t in reply2.get("tools_used", [])]
    failures += not check("okay triggers move(proposed)", "move" in tools2, str(tools2))
    # No DDS locally: the ONLY acceptable outcome is the publisher gate.
    move_results = [t for t in reply2.get("tools_used", []) if t.get("tool") == "move"]
    gate = move_results and "DDS" in json.dumps(move_results[-1])
    failures += not check("execution stopped at the no-DDS gate (robot untouched)", bool(gate),
                          json.dumps(move_results)[-200:])
    print("ALL PASS" if not failures else f"{failures} FAILURES")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
```

Note: confirm the actual `/api/chat` response field names (`reply`, `tools_used`, per-tool result shape) by reading the chat handler before running; adjust the script to match reality, not the other way around.

- [ ] **Step 2: Run the flow repeatedly against real qwen**

Start `python3 server.py` locally. Run `python3 tools/test_llm_pose_flow.py` at least 3 times (qwen is stochastic). Also probe variants by editing the first message: "sag kolunu yukari kaldir", "iki kolunu yana ac", "raise both arms". Every run must end "ALL PASS". If qwen guesses wildly wrong angles (semantics mismatch its own target), tighten the ARM JOINT GUIDE wording or the LLM_TOOLS_PROMPT example — prompt-only fixes, then rerun.

- [ ] **Step 3: Unit suite + production gate**

Run: `python3 -m unittest discover -s tests -p 'test_*.py'` — all pass.
Run: `make production-gate` — passes.

- [ ] **Step 4: Commit (branch only!)**

```bash
git add tools/test_llm_pose_flow.py
git commit -m "Add manual LLM pose-flow integration script (local, no robot)"
git push -u origin feature/llm-pose-proposals   # NEVER main during the local-testing freeze
```

---

## Self-Review

- Spec coverage: qwen prepared (Task 5 prompt + Task 2 guide); hardcoded saved-position move deleted except home (Task 4); "bring your hand forward" → generated positions (Task 3 propose); green simulated twin next to blue/live one (Task 6; the live model keeps its real colors — the green 'trajectory' tone matches the existing preview convention, and the replay panel's blue reference remains the 'correct' twin); "okay" → move with the guessed positions (Task 4 + Task 5 STEP 2). Local-only testing with real AI server (Task 7 + global constraints). ✓
- Placeholders: none; every step carries code or an exact command. ✓
- Type consistency: `landmarks()` shape `{side: {part: {x,y,z}}}` is consumed by `semantic_arm_pose` via the `landmarks_robot_m` wrapper (Task 3) and by `arm_pose_guide` (Task 2); proposal `targets` dict[str, float] → snapshot list form `{index, name, q}` (Task 3) → consumed by viewer (Task 6) and `_tool_move` motors list (Task 4). `move_tool_spec()` signature change is propagated in `chat_tool_specs()` (Task 4/3h). ✓
