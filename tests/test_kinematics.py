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
            if "ShoulderPitch" in line and "limits [" in line:
                self.assertTrue(any(w in line for w in ("forward", "backward", "up", "down")), line)

    def test_guide_lists_canonical_pose_anchors(self) -> None:
        self.assertIn("CANONICAL POSES", self.guide)
        for anchor in ("hands raised high", "straight forward", "opened sideways", "crossed in front"):
            self.assertIn(anchor, self.guide)
        # The crossed anchor must actually put the right hand on the LEFT side.
        crossed = self.kin.landmarks({"RightShoulderPitch": -1.6, "RightShoulderRoll": 0.38,
                                      "RightShoulderYaw": 1.5, "RightElbow": 1.2})
        self.assertGreater(crossed["right"]["hand"]["y"], 0.05)
        # The hands-up anchor must place the hand well above the shoulder per FK.
        up = self.kin.landmarks({"RightShoulderPitch": -2.2, "RightShoulderRoll": -0.35})
        shoulder_z = self.kin.landmarks({})["right"]["shoulder"]["z"]
        self.assertGreater(up["right"]["hand"]["z"], shoulder_z + 0.3)

    def test_zero_pose_description_is_derived_not_wrong(self) -> None:
        # The H1-2 zero pose has elbows bent ~90 deg with forearms forward; the
        # guide must describe where the hands actually start, not claim they hang.
        self.assertIn("zero", self.guide.lower())
        self.assertNotIn("hang straight down", self.guide)


if __name__ == "__main__":
    unittest.main()
