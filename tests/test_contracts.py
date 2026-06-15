from __future__ import annotations

import unittest

import server


class TelemetryContractsTest(unittest.TestCase):
    def test_body_joint_contract_matches_h1_2_slots(self) -> None:
        self.assertEqual(len(server.JOINT_NAMES), 27)
        self.assertEqual(server.JOINT_NAMES[0], "LeftHipYaw")
        self.assertEqual(server.JOINT_NAMES[12], "WaistYaw")
        self.assertEqual(server.JOINT_NAMES[26], "RightWristYaw")
        self.assertEqual(server.RIGHT_WRIST_YAW, 26)
        self.assertEqual(server.ARM_SDK_WEIGHT_SLOT, 27)

    def test_hand_joint_contract_matches_inspire_bridge_order(self) -> None:
        self.assertEqual(len(server.HAND_JOINT_NAMES), 12)
        self.assertEqual(server.HAND_JOINT_NAMES[0], "RightPinky")
        self.assertEqual(server.HAND_JOINT_NAMES[5], "RightThumbRotation")
        self.assertEqual(server.HAND_JOINT_NAMES[6], "LeftPinky")
        self.assertEqual(server.HAND_JOINT_NAMES[11], "LeftThumbRotation")

    def test_wrist_commands_require_explicit_risk_flags(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")

        wrist_status, wrist_response = store.command_wrist({})
        self.assertEqual(wrist_status, 400)
        self.assertFalse(wrist_response["ok"])
        self.assertIn("armed=true", wrist_response["error"])

    def test_loco_and_chill_do_not_require_risk_flags(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")

        loco_status, loco_response = store.command_loco({"action": "ready"})
        self.assertEqual(loco_status, 503)
        self.assertFalse(loco_response["ok"])
        self.assertIn("loco client", loco_response["error"])

        chill_status, chill_response = store.request_chill({})
        self.assertEqual(chill_status, 503)
        self.assertFalse(chill_response["ok"])
        self.assertIn("loco client", chill_response["error"])

    def test_command_limits_fail_closed(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        payload = {
            "armed": True,
            "i_understand_risk": True,
            "target_q": server.WRIST_LIMITS[1] + 1.0,
        }

        status, response = store.command_wrist(payload)
        self.assertEqual(status, 400)
        self.assertFalse(response["ok"])
        self.assertIn("target_q", response["error"])

    def test_command_numbers_must_be_finite(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        payload = {
            "armed": True,
            "i_understand_risk": True,
            "target_q": float("nan"),
        }

        status, response = store.command_wrist(payload)
        self.assertEqual(status, 400)
        self.assertFalse(response["ok"])
        self.assertIn("finite", response["error"])

    def test_wrist_mode_and_control_path_are_explicitly_validated(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        base = {"armed": True, "i_understand_risk": True}

        mode_status, mode_response = store.command_wrist({**base, "mode": "surprise"})
        self.assertEqual(mode_status, 400)
        self.assertFalse(mode_response["ok"])
        self.assertIn("mode", mode_response["error"])

        path_status, path_response = store.command_wrist({**base, "control_path": "unknown"})
        self.assertEqual(path_status, 400)
        self.assertFalse(path_response["ok"])
        self.assertIn("control_path", path_response["error"])

        oscillate_status, oscillate_response = store.command_wrist(
            {**base, "mode": "oscillate", "control_path": "arm_sdk"}
        )
        self.assertEqual(oscillate_status, 400)
        self.assertFalse(oscillate_response["ok"])
        self.assertIn("lowcmd", oscillate_response["error"])

    def test_loco_actions_are_kept_in_sync_with_declared_actions(self) -> None:
        self.assertIn("ready", server.LOCO_ACTIONS)
        self.assertIn("zero_torque", server.LOCO_ACTIONS)
        self.assertIn("velocity", server.LOCO_ACTIONS)
        self.assertIn("set_target_position", server.LOCO_ACTIONS)


if __name__ == "__main__":
    unittest.main()
