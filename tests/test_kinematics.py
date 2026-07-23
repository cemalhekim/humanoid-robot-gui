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
