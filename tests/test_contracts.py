from __future__ import annotations

import os
import time
import unittest
from unittest import mock
from tempfile import TemporaryDirectory

import server


class FakeMotorState:
    def __init__(self, q: float = 0.0, dq: float = 0.0, tau_est: float = 0.0) -> None:
        self.q = q
        self.dq = dq
        self.tau_est = tau_est


class FakeLowState:
    def __init__(self) -> None:
        self.motor_state = [FakeMotorState() for _ in range(35)]


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

    def test_hand_state_goes_offline_when_the_bridge_stops_publishing(self) -> None:
        # Detaching an arm unplugs its USB serial converter, inspire_h1 stops,
        # and rt/inspire/state goes quiet -- but the last message is retained.
        # The snapshot must report that as offline instead of replaying the
        # last frame as live hand positions.
        msg = mock.Mock(states=[FakeMotorState() for _ in range(12)])

        fresh = server.handstate_to_dict(msg, 100, time.time())
        self.assertTrue(fresh["connected"])
        self.assertNotIn("note", fresh)

        stale_at = time.time() - server.HAND_STATE_STALE_SECONDS - 1.0
        stale = server.handstate_to_dict(msg, 100, stale_at)
        self.assertFalse(stale["connected"])
        self.assertEqual(len(stale["joints"]), 12)
        self.assertIn("last received frame", stale["note"])

        flags = server.health_flags({"hands": stale}, {})
        self.assertTrue(
            any("Hand telemetry is offline" in flag["message"] for flag in flags),
            flags,
        )

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

    def test_loco_mobility_actions_require_risk_ack(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        # Walking/translating unarmed is rejected (400) before touching the robot.
        for action in ("velocity", "move", "set_target_position", "start"):
            status, response = store.command_loco({"action": action})
            self.assertEqual(status, 400, action)
            self.assertIn("armed", response["error"])
        # Armed mobility passes validation and only then hits the no-loco-client 503.
        status, response = store.command_loco(
            {"action": "velocity", "armed": True, "i_understand_risk": True, "vx": 0.2}
        )
        self.assertEqual(status, 503)
        # Read-only / posture actions remain ungated.
        self.assertEqual(store.command_loco({"action": "ready"})[0], 503)

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

    def test_stop_wrist_cancels_active_replay_hold(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        replay_cancel = server.threading.Event()
        store.replay_cancel = replay_cancel

        status = store.stop_wrist()

        self.assertTrue(replay_cancel.is_set())
        self.assertFalse(status.get("active"))

    def test_loco_actions_are_kept_in_sync_with_declared_actions(self) -> None:
        self.assertIn("ready", server.LOCO_ACTIONS)
        self.assertIn("zero_torque", server.LOCO_ACTIONS)
        self.assertIn("velocity", server.LOCO_ACTIONS)
        self.assertIn("set_target_position", server.LOCO_ACTIONS)

    def test_both_arms_replay_scope_excludes_waist(self) -> None:
        self.assertEqual(
            server.REPLAY_COMMAND_SCOPES["both_arms"],
            server.JOINT_GROUPS["left_arm"] + server.JOINT_GROUPS["right_arm"],
        )
        self.assertNotIn(12, server.REPLAY_COMMAND_SCOPES["both_arms"])

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

    def test_replay_planner_rejects_non_finite_joint_angle(self) -> None:
        # A NaN q passes every `NaN > limit` velocity/delta check (all False) and
        # would clamp to the joint limit — validation must fail closed instead.
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            path = server.Path(directory) / "nan.jsonl"
            motors = [{"index": i, "name": n, "q": 0.0} for i, n in server.JOINT_NAMES.items()]
            motors[16]["q"] = float("nan")  # LeftElbow
            path.write_text(
                server.json.dumps({"type": "telemetry_sample", "body": {"motors": motors}}) + "\n",
                encoding="utf-8",
            )
            plan = store.plan_replay_control_path(path)
        self.assertFalse(plan["valid_for_execution"])
        self.assertTrue(any(v.get("kind") == "non_finite" for v in plan.get("violations", [])))

    def test_playback_speed_capped_to_validated_velocity_envelope(self) -> None:
        cap = server.TelemetryStore._cap_playback_speed
        limit = server.TRAJECTORY_MAX_VELOCITY_RAD_S
        peak = server.ARM_REPLAY_APPROACH_PEAK_VEL_RAD_S
        # Fast recording (native at the limit): must not be sped up at all.
        self.assertAlmostEqual(cap(4.0, limit), 1.0)
        # Recording at half the limit: may be sped up to 2x, not 4x.
        self.assertAlmostEqual(cap(4.0, limit / 2), 2.0)
        # Slow recording (below approach peak): capped by the approach envelope.
        self.assertAlmostEqual(cap(4.0, 0.01), limit / peak)
        # A modest requested speed under the cap is unchanged.
        self.assertAlmostEqual(cap(1.5, limit / 4), 1.5)
        # Never returns below 1.0 for a validated (<= limit) native velocity.
        self.assertGreaterEqual(cap(1.0, limit), 1.0)

    def test_replay_planner_skips_partial_jsonl_line(self) -> None:
        # A truncated trailing line (concurrent record+replay) must not crash.
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            path = server.Path(directory) / "partial.jsonl"
            motors = [{"index": i, "name": n, "q": 0.0} for i, n in server.JOINT_NAMES.items()]
            good = server.json.dumps({"type": "telemetry_sample", "body": {"motors": motors}})
            path.write_text(good + "\n" + '{"type": "telemetry_sample", "body": {"mot', encoding="utf-8")
            plan = store.plan_replay_control_path(path)  # must not raise
        self.assertTrue(plan["frame_count"] >= 1)

    def test_replay_planner_rejects_non_numeric_motor(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            path = server.Path(directory) / "bad.jsonl"
            motors = [{"index": i, "name": n, "q": 0.0} for i, n in server.JOINT_NAMES.items()]
            motors[16]["q"] = "boom"  # non-numeric q
            path.write_text(
                server.json.dumps({"type": "telemetry_sample", "body": {"motors": motors}}) + "\n",
                encoding="utf-8",
            )
            plan = store.plan_replay_control_path(path)  # must not raise
        self.assertFalse(plan["valid_for_execution"])
        self.assertTrue(any(v.get("kind") == "malformed_motor" for v in plan.get("violations", [])))

    def test_ephemeral_replay_paths_are_unique(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        motors = [{"index": 13, "name": "LeftShoulderPitch", "q": 0.1}]
        payload = {"snapshot": {"motors": motors}}
        paths = set()
        try:
            for _ in range(50):
                p = store._write_ephemeral_replay_file(payload)
                self.assertNotIn(p, paths)  # no collision
                paths.add(p)
        finally:
            for p in paths:
                try:
                    p.unlink()
                except OSError:
                    pass

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

    def test_right_arm_replay_scope_ignores_full_body_pose_lower_joints(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            path = server.Path(directory) / "full-body-pose.pose.json"
            motors = [
                {"index": index, "name": name, "q": 0.0}
                for index, name in server.JOINT_NAMES.items()
            ]
            motors[3]["q"] = 0.5
            motors[20]["q"] = 0.25
            path.write_text(
                server.json.dumps({"type": "pose_point", "snapshot": {"timestamp": 10.0, "motors": motors}}),
                encoding="utf-8",
            )

            full_plan = store.plan_replay_control_path(path)
            right_arm_plan = store.plan_replay_control_path(path, command_scope="right_arm")

        self.assertEqual(full_plan["control_path"], "lowcmd")
        self.assertEqual(right_arm_plan["control_path"], "arm_sdk")
        self.assertEqual(right_arm_plan["moving_lower_body_joints"], [])
        self.assertEqual(
            [joint["index"] for joint in right_arm_plan["commanded_body_joints"]],
            server.JOINT_GROUPS["right_arm"],
        )
        self.assertEqual(
            [joint["index"] for joint in right_arm_plan["moving_joints"]],
            [20],
        )

    def test_arm_sdk_replay_rejects_active_xr_motion_publisher(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        store.wrist_publisher = mock.Mock()
        store.lowstate_msg = mock.Mock()
        store.lowcmd_factory = mock.Mock()
        store.crc = mock.Mock()
        with TemporaryDirectory() as directory:
            path = server.Path(directory) / "right-arm.pose.json"
            motors = [
                {"index": index, "name": name, "q": 0.0}
                for index, name in server.JOINT_NAMES.items()
            ]
            motors[20]["q"] = 0.25
            path.write_text(
                server.json.dumps({"type": "pose_point", "snapshot": {"timestamp": 10.0, "motors": motors}}),
                encoding="utf-8",
            )
            plan = {
                "command_scope": "right_arm",
                "control_path": "arm_sdk",
                "valid_for_execution": True,
                "hand_plan": {"enabled": False},
                "commanded_body_joints": [
                    {"index": index, "name": server.JOINT_NAMES[index]}
                    for index in server.JOINT_GROUPS["right_arm"]
                ],
                "gain_plan": [
                    {"index": index, "kp": 12.0, "kd": 1.0}
                    for index in server.JOINT_GROUPS["right_arm"]
                ],
            }
            with mock.patch.object(
                store,
                "_suspend_xr_motion_publishers",
                return_value={"ok": False, "remaining_processes": ["123 teleop_hand_and_arm.py --motion"]},
            ):
                status, response = store.execute_arm_sdk_replay(path, plan)

        self.assertEqual(status, 409)
        self.assertFalse(response["ok"])
        self.assertIn("XR teleop motion publisher", response["error"])

    def test_closed_loop_arm_targets_correct_toward_desired_position(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        msg = FakeLowState()
        joint = 20
        msg.motor_state[joint].q = 0.2
        msg.motor_state[joint].dq = 0.0
        target = {joint: 0.5}
        state: dict[int, dict[str, float]] = {}

        corrected, error, feedforward_tau = store._closed_loop_arm_targets(msg, target, state, 1.0 / 60.0)

        self.assertGreater(corrected[joint], target[joint])
        self.assertLessEqual(corrected[joint] - target[joint], server.ARM_REPLAY_MAX_PID_CORRECTION_RAD)
        self.assertEqual(error["max_error_rad"], 0.3)
        self.assertGreater(error["per_joint"][0]["correction_rad"], 0.0)
        self.assertEqual(feedforward_tau[joint], 0.0)

    def test_closed_loop_arm_targets_drive_lagging_joint_to_target(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        msg = FakeLowState()
        joint = 20
        target = {joint: 0.5}
        state: dict[int, dict[str, float]] = {}
        dt = 1.0 / 60.0
        max_error = 1.0

        for _ in range(240):
            corrected, error, _feedforward_tau = store._closed_loop_arm_targets(msg, target, state, dt)
            command_q = corrected[joint]
            previous_q = msg.motor_state[joint].q
            msg.motor_state[joint].q += (command_q - previous_q) * 0.12
            msg.motor_state[joint].dq = (msg.motor_state[joint].q - previous_q) / dt
            max_error = error["max_error_rad"]

        self.assertLessEqual(max_error, server.ARM_REPLAY_TOLERANCE_RAD)
        self.assertTrue(error["all_locked"])
        self.assertAlmostEqual(msg.motor_state[joint].q, target[joint], delta=server.ARM_REPLAY_TOLERANCE_RAD)

    def test_closed_loop_arm_targets_reset_integral_when_target_changes(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        msg = FakeLowState()
        joint = 20
        state: dict[int, dict[str, float]] = {}
        dt = 1.0 / 60.0

        for _ in range(120):
            store._closed_loop_arm_targets(msg, {joint: 0.5}, state, dt)

        self.assertGreater(state[joint]["integral"], 0.0)
        store._closed_loop_arm_targets(msg, {joint: -0.2}, state, dt)

        self.assertLessEqual(abs(state[joint]["integral"]), abs(-0.2 - msg.motor_state[joint].q) * dt)

    def test_closed_loop_arm_targets_lock_exact_target_with_gravity_feedforward(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        msg = FakeLowState()
        joint = 20
        msg.motor_state[joint].q = 0.496
        msg.motor_state[joint].dq = 0.0
        msg.motor_state[joint].tau_est = 8.0
        target = {joint: 0.5}

        corrected, error, feedforward_tau = store._closed_loop_arm_targets(msg, target, {}, 1.0 / 60.0)

        self.assertEqual(corrected[joint], target[joint])
        self.assertTrue(error["all_locked"])
        self.assertTrue(error["per_joint"][0]["locked"])
        self.assertGreater(feedforward_tau[joint], 0.0)
        self.assertLessEqual(feedforward_tau[joint], server.ARM_REPLAY_GRAVITY_TAU_LIMITS["shoulder"])
        self.assertGreater(error["per_joint"][0]["gravity_tau"], 0.0)

    def test_arm_replay_response_tuning_preserves_balanced_default(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")

        tuning = store._arm_replay_tuning({})

        self.assertTrue(server.ARM_REPLAY_HOLD_AFTER_CONVERGENCE_DEFAULT)
        self.assertEqual(tuning["response"], server.ARM_REPLAY_RESPONSE_DEFAULT)
        self.assertEqual(tuning["inner_kp_scale"], server.ARM_REPLAY_INNER_KP_SCALE)
        self.assertEqual(tuning["inner_kd_scale"], server.ARM_REPLAY_INNER_KD_SCALE)
        self.assertEqual(tuning["direct_kp_scale"], 1.0)
        self.assertEqual(tuning["direct_kd_scale"], 1.0)
        self.assertEqual(tuning["approach_kp_scale"], 0.75)
        self.assertEqual(tuning["approach_kd_scale"], 1.1)
        self.assertEqual(tuning["playback_speed"], 1.0)
        self.assertEqual(tuning["lock_tolerance_rad"], server.ARM_REPLAY_LOCK_TOLERANCE_RAD)
        self.assertEqual(tuning["lock_tolerance_m"], server.ARM_REPLAY_LOCK_TOLERANCE_M)
        self.assertEqual(tuning["max_pid_correction_rad"], server.ARM_REPLAY_MAX_PID_CORRECTION_RAD)
        self.assertEqual(tuning["smooth_approach_seconds"], server.ARM_REPLAY_SMOOTH_APPROACH_SECONDS)
        self.assertEqual(tuning["settle_seconds"], server.ARM_REPLAY_SETTLE_SECONDS)

    def test_arm_replay_response_tuning_clamps_and_increases_responsiveness(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")

        damped = store._arm_replay_tuning({"replay_response": -1})
        responsive = store._arm_replay_tuning({"replay_response": 99})

        self.assertEqual(damped["response"], 0.0)
        self.assertEqual(responsive["response"], server.ARM_REPLAY_RESPONSE_MAX)
        self.assertGreater(responsive["inner_kp_scale"], damped["inner_kp_scale"])
        self.assertLess(responsive["inner_kd_scale"], damped["inner_kd_scale"])
        self.assertGreater(responsive["direct_kp_scale"], damped["direct_kp_scale"])
        self.assertLess(responsive["direct_kd_scale"], damped["direct_kd_scale"])
        self.assertGreater(responsive["approach_kp_scale"], damped["approach_kp_scale"])
        self.assertLess(responsive["approach_kd_scale"], damped["approach_kd_scale"])
        self.assertGreater(responsive["playback_speed"], damped["playback_speed"])
        self.assertGreater(responsive["max_pid_correction_rad"], damped["max_pid_correction_rad"])
        self.assertLess(responsive["smooth_approach_seconds"], damped["smooth_approach_seconds"])

    def test_smooth_arm_replay_frames_ramp_from_current_position(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        msg = FakeLowState()
        joint = 20
        msg.motor_state[joint].q = 0.0
        frames = [{"motors": [{"index": joint, "name": server.JOINT_NAMES[joint], "q": 1.0}]}]

        smoothed = store._smooth_arm_replay_frames(frames, msg, {joint})
        first_q = smoothed[0]["motors"][0]["q"]
        final_q = smoothed[-1]["motors"][0]["q"]

        self.assertGreater(len(smoothed), 100)
        self.assertGreater(first_q, 0.0)
        self.assertLess(first_q, 0.001)
        self.assertAlmostEqual(final_q, 1.0)

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

    def test_rename_recording_keeps_timestamp_prefix_and_extension(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            original_dir = server.RECORDINGS_DIR
            server.RECORDINGS_DIR = server.Path(directory)
            try:
                path = server.RECORDINGS_DIR / "20260714-101500-h1_2_pose_point.pose.json"
                path.write_text("{}", encoding="utf-8")
                status, response = store.rename_recording({"name": path.name, "label": "wave hello!"})
            finally:
                server.RECORDINGS_DIR = original_dir

        self.assertEqual(status, 200)
        self.assertEqual(response["file"]["name"], "20260714-101500-wave_hello.pose.json")
        self.assertTrue(response["file"]["custom_named"])

    def test_rename_recording_rejects_missing_file_and_existing_target(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            original_dir = server.RECORDINGS_DIR
            server.RECORDINGS_DIR = server.Path(directory)
            try:
                source = server.RECORDINGS_DIR / "20260714-101500-pose_point.pose.json"
                source.write_text("{}", encoding="utf-8")
                (server.RECORDINGS_DIR / "20260714-101500-taken.pose.json").write_text("{}", encoding="utf-8")
                missing_status, _ = store.rename_recording({"name": "20260714-000000-gone.pose.json", "label": "x"})
                conflict_status, _ = store.rename_recording({"name": source.name, "label": "taken"})
            finally:
                server.RECORDINGS_DIR = original_dir

        self.assertEqual(missing_status, 404)
        self.assertEqual(conflict_status, 409)

    def test_recording_files_lists_custom_named_before_auto_named(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            original_dir = server.RECORDINGS_DIR
            server.RECORDINGS_DIR = server.Path(directory)
            try:
                old_custom = server.RECORDINGS_DIR / "20260101-000000-wave_hello.pose.json"
                new_auto = server.RECORDINGS_DIR / "20260714-101500-h1_2_pose_point.pose.json"
                old_custom.write_text("{}", encoding="utf-8")
                new_auto.write_text("{}", encoding="utf-8")
                os.utime(old_custom, (1_700_000_000, 1_700_000_000))
                os.utime(new_auto, (1_800_000_000, 1_800_000_000))
                names = [item["name"] for item in store.recording_files()["files"]]
            finally:
                server.RECORDINGS_DIR = original_dir

        self.assertEqual(names, [old_custom.name, new_auto.name])

    def test_named_positions_maps_custom_labels_to_files(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            original_dir = server.RECORDINGS_DIR
            server.RECORDINGS_DIR = server.Path(directory)
            try:
                for name in (
                    "20260714-112404-raise-hand.pose.json",
                    "20260714-112354-hand_forward.pose.json",
                    "20260714-111543-home.pose.json",
                    "20260714-101500-h1_2_pose_point.pose.json",
                ):
                    (server.RECORDINGS_DIR / name).write_text("{}", encoding="utf-8")
                positions = store.named_positions()
            finally:
                server.RECORDINGS_DIR = original_dir

        self.assertEqual(
            positions,
            {
                "raise-hand": "20260714-112404-raise-hand.pose.json",
                "hand-forward": "20260714-112354-hand_forward.pose.json",
                "home": "20260714-111543-home.pose.json",
            },
        )

    def test_chat_tool_specs_include_move_with_position_enum(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        # move no longer depends on saved recordings: proposed + home only.
        with TemporaryDirectory() as directory:
            original_dir = server.RECORDINGS_DIR
            server.RECORDINGS_DIR = server.Path(directory)
            try:
                specs = {spec["function"]["name"]: spec for spec in store.chat_tool_specs()}
            finally:
                server.RECORDINGS_DIR = original_dir

        move = specs["move"]["function"]
        self.assertEqual(move["parameters"]["properties"]["position"]["enum"], ["proposed", "home"])
        self.assertEqual(move["parameters"]["required"], ["position", "confirm"])
        self.assertIn("propose_arm_pose", specs)

    def test_move_tool_requires_confirm_and_known_position(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            original_dir = server.RECORDINGS_DIR
            server.RECORDINGS_DIR = server.Path(directory)
            try:
                (server.RECORDINGS_DIR / "20260714-111543-home.pose.json").write_text("{}", encoding="utf-8")
                no_confirm = store.run_chat_tool("move", {"position": "home"})
                unknown = store.run_chat_tool("move", {"position": "backflip", "confirm": True})
            finally:
                server.RECORDINGS_DIR = original_dir

        self.assertFalse(no_confirm["ok"])
        self.assertIn("confirm", no_confirm["error"])
        self.assertFalse(unknown["ok"])
        self.assertIn("home", unknown["error"])

    def test_move_tool_replays_home_like_move_button(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            original_dir = server.RECORDINGS_DIR
            server.RECORDINGS_DIR = server.Path(directory)
            try:
                (server.RECORDINGS_DIR / "20260714-111543-home.pose.json").write_text("{}", encoding="utf-8")
                with mock.patch.object(
                    store, "request_robot_replay", return_value=(200, {"ok": True, "message": "moving", "plan": {"x": 1}})
                ) as replay:
                    result = store.run_chat_tool("move", {"position": "home", "confirm": True})
            finally:
                server.RECORDINGS_DIR = original_dir

        self.assertTrue(result["ok"])
        self.assertEqual(result["position"], "home")
        self.assertNotIn("plan", result)
        payload = replay.call_args[0][0]
        self.assertEqual(payload["filename"], "20260714-111543-home.pose.json")
        self.assertIs(payload["execute_arm_sdk"], True)
        self.assertEqual(payload["command_scope"], "arms")

    def test_chat_replays_history_tools_used_as_tool_calls(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        captured: dict[str, list] = {}

        def fake_call_llm(messages, tools=None):
            captured["messages"] = messages
            return 200, {"ok": True, "reply": "done"}

        payload = {
            "messages": [
                {"role": "user", "content": "raise your hand"},
                {
                    "role": "assistant",
                    "content": "Raising your hand now.",
                    "tools_used": [{"name": "move", "arguments": {"position": "raise-hand", "confirm": True}, "ok": True}],
                },
                {"role": "user", "content": "go home"},
            ]
        }
        with mock.patch.object(server, "call_llm", side_effect=fake_call_llm):
            with mock.patch.object(server, "LLM_TOOLS_ENABLED", True):
                status, _ = store.chat(payload)

        self.assertEqual(status, 200)
        roles = [m["role"] for m in captured["messages"]]
        self.assertEqual(roles, ["system", "user", "assistant", "tool", "assistant", "user"])
        replayed = captured["messages"][2]["tool_calls"][0]["function"]
        self.assertEqual(replayed["name"], "move")
        self.assertEqual(server.json.loads(replayed["arguments"])["position"], "raise-hand")
        self.assertEqual(server.json.loads(captured["messages"][3]["content"]), {"ok": True})

    def test_extract_textual_tool_call_promotes_propose_json(self) -> None:
        # qwen sometimes narrates the proposal as text; a bare {"joints": {...}}
        # blob must still become a real propose_arm_pose call.
        tools = [server.propose_tool_spec(), server.move_tool_spec()]
        for reply in (
            '{"joints": {"RightShoulderPitch": -1.57, "RightElbow": 1.57}}',
            'Staging: {"joints": {"RightShoulderPitch": -1.57}} now',
            '<tool_call>{"name": "propose_arm_pose", "arguments": {"joints": {"LeftElbow": 1.0}}}</tool_call>',
        ):
            call = server.extract_textual_tool_call(reply, tools)
            self.assertIsNotNone(call, reply)
            self.assertEqual(call["function"]["name"], "propose_arm_pose")
            self.assertIn("joints", server.json.loads(call["function"]["arguments"]))

    def test_extract_textual_tool_call_promotes_move_json(self) -> None:
        tools = [server.move_tool_spec()]
        for reply in (
            '{"position": "proposed", "confirm": true}',
            'Sure: {"position": "proposed", "confirm": true} done',
            '<tool_call>{"name": "move", "arguments": {"position": "proposed", "confirm": true}}</tool_call>',
        ):
            call = server.extract_textual_tool_call(reply, tools)
            self.assertIsNotNone(call, reply)
            self.assertEqual(call["function"]["name"], "move")
            self.assertEqual(server.json.loads(call["function"]["arguments"])["position"], "proposed")

    def test_extract_textual_tool_call_ignores_plain_answers(self) -> None:
        tools = [server.move_tool_spec()]
        for reply in (
            "Motor sayısı: 0",
            "raising your hand now.",
            'The config is {"port": 8088} on eth0',
            "",
        ):
            self.assertIsNone(server.extract_textual_tool_call(reply, tools), reply)
        self.assertIsNone(server.extract_textual_tool_call('{"position": "home", "confirm": true}', []))

    def test_robot_replay_accepts_inline_edited_pose_without_saving(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            original_dir = server.RECORDINGS_DIR
            original_ephemeral = server.EPHEMERAL_REPLAY_DIR
            server.RECORDINGS_DIR = server.Path(directory)
            server.EPHEMERAL_REPLAY_DIR = server.Path(directory) / ".ephemeral"
            try:
                motors = [{"index": 13, "name": "LeftShoulderPitch", "q": 0.2}]
                status, response = store.request_robot_replay(
                    {"snapshot": {"timestamp": 5.0, "motors": motors}, "dry_run": True}
                )
                # No filename provided: the unsaved pose is planned like a saved file.
                self.assertEqual(status, 200)
                self.assertTrue(response["ok"])
                self.assertTrue(response["recording"].endswith(".pose.json"))
                self.assertEqual(response["plan"]["frame_count"], 1)
                # The scratch file is deleted, never left behind in recordings.
                leftover = (
                    list(server.EPHEMERAL_REPLAY_DIR.glob("*"))
                    if server.EPHEMERAL_REPLAY_DIR.exists()
                    else []
                )
                self.assertEqual(leftover, [])
                self.assertEqual(list(server.Path(directory).glob("*.pose.json")), [])
            finally:
                server.RECORDINGS_DIR = original_dir
                server.EPHEMERAL_REPLAY_DIR = original_ephemeral

    def test_robot_replay_accepts_inline_sequence_points(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with TemporaryDirectory() as directory:
            original_dir = server.RECORDINGS_DIR
            original_ephemeral = server.EPHEMERAL_REPLAY_DIR
            server.RECORDINGS_DIR = server.Path(directory)
            server.EPHEMERAL_REPLAY_DIR = server.Path(directory) / ".ephemeral"
            try:
                points = [
                    {"timestamp": 1.0, "motors": [{"index": 20, "name": "RightShoulderPitch", "q": 0.1}]},
                    {"timestamp": 2.0, "motors": [{"index": 20, "name": "RightShoulderPitch", "q": 0.2}]},
                ]
                status, response = store.request_robot_replay({"points": points, "dry_run": True})
                self.assertEqual(status, 200)
                self.assertTrue(response["recording"].endswith(".sequence.json"))
                self.assertGreaterEqual(response["plan"]["frame_count"], 2)
            finally:
                server.RECORDINGS_DIR = original_dir
                server.EPHEMERAL_REPLAY_DIR = original_ephemeral

    def test_robot_replay_rejects_empty_inline_payload(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        # No filename, no snapshot, no points -> nothing to move.
        status, response = store.request_robot_replay({"dry_run": True})
        self.assertEqual(status, 400)
        self.assertFalse(response["ok"])


    def test_response_overdrive_doubles_legacy_top_pid(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        legacy_top = store._arm_replay_tuning({"replay_response": server.ARM_REPLAY_RESPONSE_LEGACY_MAX})
        new_top = store._arm_replay_tuning({"replay_response": server.ARM_REPLAY_RESPONSE_MAX})
        mid = store._arm_replay_tuning({"replay_response": 1.25})
        # Doubled slider: the new 100% drives the PID at exactly 2x the old top.
        self.assertAlmostEqual(new_top["pid_kp_scale"], legacy_top["pid_kp_scale"] * 2, places=5)
        self.assertAlmostEqual(new_top["inner_kp_scale"], legacy_top["inner_kp_scale"] * 2, places=5)
        self.assertAlmostEqual(new_top["max_pid_correction_rad"], legacy_top["max_pid_correction_rad"] * 2, places=5)
        # Time-like values speed up; damping ratios stay at legacy-top values.
        self.assertAlmostEqual(new_top["smooth_approach_seconds"], legacy_top["smooth_approach_seconds"] / 2, places=5)
        self.assertEqual(new_top["pid_kd_scale"], legacy_top["pid_kd_scale"])
        # Values inside the legacy range are untouched by the range doubling.
        self.assertAlmostEqual(mid["pid_kp_scale"], 1.225, places=3)

    def test_store_initializes_motion_cancel_handles(self) -> None:
        # execute_lowcmd_pose reads all three to cancel in-flight motion before a
        # torso twist. A missing handle crashes the waist path with AttributeError
        # before the torso can move (regression: torso_cancel was never initialized).
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        self.assertIsNone(store.wrist_cancel)
        self.assertIsNone(store.replay_cancel)
        self.assertIsNone(store.torso_cancel)
        # Handoff flag for consecutive lowcmd pose sessions must start False.
        self.assertFalse(store.motion_mode_released)

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

class HomeMoveContractTest(unittest.TestCase):
    """Home = closed-loop move to the saved 'home' pose (2026-07-23; the old
    hold-current-position behavior is gone)."""

    def setUp(self) -> None:
        self.store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_home_without_dds_falls_back_and_reports_unavailable(self) -> None:
        with TemporaryDirectory() as directory:
            (server.Path(directory) / "20260714-111543-home.pose.json").write_text(
                "{}", encoding="utf-8")
            with mock.patch.object(server, "RECORDINGS_DIR", server.Path(directory)), \
                 mock.patch.object(
                     self.store, "request_robot_replay",
                     return_value=(503, {"ok": False, "error": "DDS arm_sdk publisher is not available."}),
                 ), \
                 mock.patch.object(
                     self.store, "_request_xr_ipc",
                     return_value=(502, {"ok": False, "error": "xr down"}),
                 ) as xr_ipc:
                status, response = self.store.request_home()
        self.assertEqual(status, 503)
        self.assertFalse(response["ok"])
        self.assertIn("arm_sdk", response["error"])
        self.assertEqual(response["xr_fallback_error"], "xr down")
        xr_ipc.assert_called_once()
