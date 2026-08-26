import math
import unittest

import kinematics


class GravityModelTest(unittest.TestCase):
    def setUp(self) -> None:
        self.kin = kinematics.ArmKinematics()

    def test_arm_mass_is_plausible_and_symmetric(self) -> None:
        left, right = self.kin.arm_mass("left"), self.kin.arm_mass("right")
        self.assertAlmostEqual(left, right, places=3)
        self.assertTrue(4.0 < right < 9.0, right)

    def test_hanging_arm_needs_little_shoulder_pitch_torque(self) -> None:
        tau = self.kin.gravity_torques({"RightShoulderPitch": 0.0, "RightElbow": 0.0})
        self.assertLess(abs(tau["RightShoulderPitch"]), 4.5)

    def test_horizontal_arm_needs_roughly_mass_times_lever(self) -> None:
        tau = self.kin.gravity_torques({"RightShoulderPitch": -math.pi / 2, "RightElbow": 0.0})
        # ~6.4 kg arm, centre of mass ~0.17 m from the shoulder when extended -> ~10-11 Nm
        self.assertTrue(8.0 < abs(tau["RightShoulderPitch"]) < 14.0, tau)
        self.assertLess(tau["RightShoulderPitch"], 0.0)  # sign matches MuJoCo qfrc_bias on the twin

    def test_mirrored_pose_gives_mirrored_torques(self) -> None:
        right = self.kin.gravity_torques({"RightShoulderPitch": -1.0, "RightShoulderRoll": -0.5, "RightElbow": 0.8})
        left = self.kin.gravity_torques({"LeftShoulderPitch": -1.0, "LeftShoulderRoll": 0.5, "LeftElbow": 0.8})
        # The URDF hand links are not perfectly mirrored; the arm links are.
        self.assertAlmostEqual(right["RightShoulderPitch"], left["LeftShoulderPitch"], delta=0.05 * abs(right["RightShoulderPitch"]))
        self.assertAlmostEqual(right["RightElbow"], left["LeftElbow"], delta=0.1 * abs(right["RightElbow"]))
        self.assertAlmostEqual(right["RightShoulderRoll"], -left["LeftShoulderRoll"], delta=0.05 * abs(right["RightShoulderRoll"]) + 1e-3)
        r_arm = self.kin.gravity_torques({"RightShoulderPitch": -1.0, "RightShoulderRoll": -0.5, "RightElbow": 0.8}, include_hands=False)
        l_arm = self.kin.gravity_torques({"LeftShoulderPitch": -1.0, "LeftShoulderRoll": 0.5, "LeftElbow": 0.8}, include_hands=False)
        self.assertAlmostEqual(r_arm["RightShoulderPitch"], l_arm["LeftShoulderPitch"], places=1)
        self.assertLess(abs(r_arm["RightShoulderPitch"]), abs(right["RightShoulderPitch"]))

    def test_handless_model_matches_the_twin_within_ten_percent(self) -> None:
        # MuJoCo (handless H1-2 scene) needs -11.11 Nm on the shoulder pitch at this pose.
        tau = self.kin.gravity_torques({"RightShoulderPitch": -1.0, "RightShoulderRoll": -0.5, "RightElbow": 0.8}, include_hands=False)
        self.assertTrue(10.0 < abs(tau["RightShoulderPitch"]) < 12.2, tau)


if __name__ == "__main__":
    unittest.main()
