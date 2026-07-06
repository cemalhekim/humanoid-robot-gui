from __future__ import annotations

import unittest
from tempfile import TemporaryDirectory

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

    def test_telemetry_recorder_writes_ordered_jsonl_records(self) -> None:
        with TemporaryDirectory() as directory:
            recorder = server.TelemetryRecorder(server.Path(directory))
            started = recorder.start("unit-test")
            self.assertTrue(started["active"])

            recorder.write_sample({"type": "telemetry_sample", "timestamp": 1.0, "monotonic_ns": 10})
            recorder.write_event("test-command", {"ok": True})
            stopped = recorder.stop()

            self.assertFalse(stopped["active"])
            self.assertEqual(stopped["samples"], 1)
            self.assertEqual(stopped["events"], 1)
            path = server.Path(stopped["path"])
            records = [server.json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual([record["type"] for record in records], [
                "recording_start",
                "telemetry_sample",
                "command_event",
                "recording_stop",
            ])

    def test_replay_planner_routes_stationary_lower_body_to_arm_sdk(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            path = server.Path(directory) / "arm-only.jsonl"
            motors = [
                {"index": index, "name": name, "q": 0.0}
                for index, name in server.JOINT_NAMES.items()
            ]
            motors[13]["q"] = 0.25
            path.write_text(
                server.json.dumps({"type": "telemetry_sample", "body": {"motors": motors}}) + "\n",
                encoding="utf-8",
            )

            plan = store.plan_replay_control_path(path)

        self.assertEqual(plan["control_path"], "arm_sdk")
        self.assertEqual(plan["moving_lower_body_joints"], [])
        self.assertEqual(len(plan["gain_plan"]), len(server.ARM_SDK_JOINTS))
        self.assertTrue(all(joint["index"] in server.ARM_SDK_JOINTS for joint in plan["gain_plan"]))

    def test_replay_planner_routes_lower_body_motion_to_lowcmd(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            path = server.Path(directory) / "leg-motion.jsonl"
            motors = [
                {"index": index, "name": name, "q": 0.0}
                for index, name in server.JOINT_NAMES.items()
            ]
            motors[3]["q"] = 0.08
            path.write_text(
                server.json.dumps({"type": "telemetry_sample", "body": {"motors": motors}}) + "\n",
                encoding="utf-8",
            )

            plan = store.plan_replay_control_path(path)

        self.assertEqual(plan["control_path"], "lowcmd")
        self.assertEqual(plan["moving_lower_body_joints"], [{"index": 3, "name": "LeftKnee"}])
        self.assertEqual(len(plan["gain_plan"]), len(server.JOINT_NAMES))
        knee_gain = next(joint for joint in plan["gain_plan"] if joint["index"] == 3)
        self.assertEqual(knee_gain["group"], "knee")
        self.assertGreaterEqual(knee_gain["kp"], server.LOWCMD_BASE_GAINS["knee"][0] * 0.75)

    def test_replay_planner_includes_parallel_hand_plan_for_finger_motion(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            path = server.Path(directory) / "finger-motion.jsonl"
            motors = [
                {"index": index, "name": name, "q": 0.0}
                for index, name in server.JOINT_NAMES.items()
            ]
            first_hands = [
                {"index": index, "name": name, "q": 0.0}
                for index, name in server.HAND_JOINT_NAMES.items()
            ]
            second_hands = [dict(joint) for joint in first_hands]
            second_hands[3]["q"] = 0.08
            path.write_text(
                "\n".join(
                    [
                        server.json.dumps(
                            {
                                "type": "telemetry_sample",
                                "timestamp": 1.0,
                                "body": {"motors": motors},
                                "hands": {"joints": first_hands},
                            }
                        ),
                        server.json.dumps(
                            {
                                "type": "telemetry_sample",
                                "timestamp": 1.1,
                                "body": {"motors": motors},
                                "hands": {"joints": second_hands},
                            }
                        ),
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            plan = store.plan_replay_control_path(path)

        self.assertEqual(plan["control_path"], "arm_sdk")
        self.assertTrue(plan["hand_plan"]["enabled"])
        self.assertEqual(plan["hand_plan"]["command_topic"], server.HAND_COMMAND_TOPIC)
        self.assertEqual(plan["hand_plan"]["state_topic"], server.HAND_STATE_TOPIC)
        self.assertEqual(
            plan["hand_plan"]["moving_hand_joints"],
            [{"index": 3, "name": "RightIndex"}],
        )

    def test_replay_planner_disables_hand_plan_when_fingers_are_stationary(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            path = server.Path(directory) / "no-finger-motion.jsonl"
            motors = [
                {"index": index, "name": name, "q": 0.0}
                for index, name in server.JOINT_NAMES.items()
            ]
            hands = [
                {"index": index, "name": name, "q": 0.0}
                for index, name in server.HAND_JOINT_NAMES.items()
            ]
            path.write_text(
                server.json.dumps(
                    {
                        "type": "telemetry_sample",
                        "timestamp": 1.0,
                        "body": {"motors": motors},
                        "hands": {"joints": hands},
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            plan = store.plan_replay_control_path(path)

        self.assertFalse(plan["hand_plan"]["enabled"])
        self.assertEqual(plan["hand_plan"]["frame_count"], 1)
        self.assertEqual(plan["hand_plan"]["moving_hand_joints"], [])

    def test_pose_json_is_read_as_single_trajectory_point(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            path = server.Path(directory) / "single.pose.json"
            motors = [{"index": 13, "name": "LeftShoulderPitch", "q": 0.2}]
            path.write_text(
                server.json.dumps({"type": "pose_point", "snapshot": {"timestamp": 10.0, "motors": motors}}),
                encoding="utf-8",
            )

            frames = store._recording_trajectory_frames(path)
            plan = store.plan_replay_control_path(path)

        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0]["timestamp"], 10.0)
        self.assertEqual(frames[0]["motors"], motors)
        self.assertEqual(plan["frame_count"], 1)
        self.assertEqual(plan["duration_seconds"], server.TRAJECTORY_APPROACH_SECONDS)

    def test_capture_pose_accepts_client_edited_snapshot(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            original_dir = server.RECORDINGS_DIR
            server.RECORDINGS_DIR = server.Path(directory)
            try:
                motors = [{"index": 20, "name": "RightShoulderPitch", "q": 0.42}]
                status, response = store.capture_pose({"label": "edited", "snapshot": {"motors": motors}})
                path = server.Path(response["file"]["path"])
                saved = server.json.loads(path.read_text(encoding="utf-8"))
            finally:
                server.RECORDINGS_DIR = original_dir

        self.assertEqual(status, 200)
        self.assertEqual(saved["snapshot"]["motors"], motors)

    def test_save_sequence_writes_sequence_json_points(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            original_dir = server.RECORDINGS_DIR
            server.RECORDINGS_DIR = server.Path(directory)
            try:
                points = [
                    {"timestamp": 1.0, "motors": [{"index": 20, "name": "RightShoulderPitch", "q": 0.1}]},
                    {"timestamp": 2.0, "motors": [{"index": 20, "name": "RightShoulderPitch", "q": 0.2}]},
                ]
                status, response = store.save_sequence({"label": "edited-sequence", "points": points})
                path = server.Path(response["file"]["path"])
                saved = server.json.loads(path.read_text(encoding="utf-8"))
            finally:
                server.RECORDINGS_DIR = original_dir

        self.assertEqual(status, 200)
        self.assertTrue(response["file"]["name"].endswith(".sequence.json"))
        self.assertEqual(saved["type"], "trajectory")
        self.assertEqual(len(saved["points"]), 2)
        self.assertEqual(saved["points"][0]["motors"], points[0]["motors"])
        self.assertEqual(saved["points"][0]["type"], "telemetry_sample")

    def test_sparse_sequence_json_is_resampled_between_trajectory_points(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            path = server.Path(directory) / "multi.sequence.json"
            points = [
                {"timestamp": 10.0, "motors": [{"index": 13, "name": "LeftShoulderPitch", "q": 0.1}]},
                {"timestamp": 11.0, "motors": [{"index": 13, "name": "LeftShoulderPitch", "q": 0.2}]},
            ]
            path.write_text(server.json.dumps({"type": "trajectory", "points": points}), encoding="utf-8")

            frames = store._recording_trajectory_frames(path)

        self.assertGreater(len(frames), 2)
        self.assertEqual(frames[0]["timestamp"], 10.0)
        self.assertEqual(frames[-1]["timestamp"], 11.0)
        self.assertEqual(frames[0]["motors"][0]["q"], 0.1)
        self.assertEqual(frames[-1]["motors"][0]["q"], 0.2)

    def test_dense_sequence_json_keeps_recorded_trajectory_points(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            path = server.Path(directory) / "dense.sequence.json"
            points = [
                {"timestamp": 10.0, "motors": [{"index": 13, "name": "LeftShoulderPitch", "q": 0.1}]},
                {"timestamp": 10.01, "motors": [{"index": 13, "name": "LeftShoulderPitch", "q": 0.11}]},
                {"timestamp": 10.02, "motors": [{"index": 13, "name": "LeftShoulderPitch", "q": 0.12}]},
            ]
            path.write_text(server.json.dumps({"type": "trajectory", "points": points}), encoding="utf-8")

            frames = store._recording_trajectory_frames(path)

        self.assertEqual(len(frames), 3)
        self.assertEqual([frame["timestamp"] for frame in frames], [10.0, 10.01, 10.02])
        self.assertEqual([frame["motors"][0]["q"] for frame in frames], [0.1, 0.11, 0.12])


if __name__ == "__main__":
    unittest.main()
