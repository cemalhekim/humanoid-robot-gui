import unittest
from unittest import mock

import server


def _motors(targets: dict) -> list:
    return [{"index": i, "name": server.JOINT_NAMES.get(i, f"Motor{i}"), "q": targets.get(i, 0.0)} for i in range(27)]


class PathCollisionPlanTest(unittest.TestCase):
    def setUp(self) -> None:
        self.store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def _plan(self, targets: dict) -> dict:
        status, body = self.store.request_robot_replay({"snapshot": {"motors": _motors(targets)}, "dry_run": True, "command_scope": "arms"})
        self.assertEqual(status, 200, body)
        return body["plan"]

    def test_benign_pose_reports_no_collision_and_stays_valid(self) -> None:
        plan = self._plan({20: -0.8, 21: -0.3, 23: 0.8})
        self.assertTrue(plan["path_collision"]["checked"])
        self.assertIsNone(plan["path_collision"]["label"])
        self.assertTrue(plan["valid_for_execution"])

    def test_arms_crossing_in_front_are_reported_but_not_blocked_by_default(self) -> None:
        # both arms straight forward with no abduction: the sphere model flags wrist vs wrist
        plan = self._plan({13: -1.57, 20: -1.57})
        self.assertIsNotNone(plan["path_collision"]["label"], plan["path_collision"])
        self.assertTrue(plan["valid_for_execution"])
        self.assertFalse(any(v["type"] == "self_collision_path" for v in plan["violations"]))

    def test_block_flag_invalidates_execution(self) -> None:
        with mock.patch.object(server, "ARM_REPLAY_PATH_COLLISION_BLOCK", 1.0):
            plan = self._plan({13: -1.57, 20: -1.57})
        self.assertFalse(plan["valid_for_execution"])
        self.assertTrue(any(v["type"] == "self_collision_path" for v in plan["violations"]))


class LearnGatingTest(unittest.TestCase):
    def _fake_state(self, moving_joint: int | None) -> object:
        from tests.test_contracts import FakeLowState  # reuse the contract fixture
        msg = FakeLowState()
        if moving_joint is not None:
            msg.motor_state[moving_joint].dq = 1.0
        return msg

    def _learn_after_steps(self, msg, steps: int = 50) -> float:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        state: dict = {}
        target = {23: 0.02, 20: 0.0}  # elbow sits 0.02 rad (inside the learn band) off target
        msg.motor_state[23].q = 0.0
        for _ in range(steps):
            store._closed_loop_arm_targets(msg, target, state, 1.0 / 120.0)
        return state[23]["gravity_learn"]

    def test_learn_runs_when_the_arm_is_still(self) -> None:
        self.assertNotEqual(self._learn_after_steps(self._fake_state(None)), 0.0)

    def test_gate_stops_learning_while_another_joint_moves(self) -> None:
        with mock.patch.object(server, "ARM_REPLAY_LEARN_ARM_STATIONARY_GATE", 1.0):
            self.assertEqual(self._learn_after_steps(self._fake_state(20)), 0.0)
        # without the gate the same situation does learn
        self.assertNotEqual(self._learn_after_steps(self._fake_state(20)), 0.0)

    def test_leak_decays_a_wound_up_value_outside_the_band(self) -> None:
        from tests.test_contracts import FakeLowState
        msg = FakeLowState()
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        state = {23: {"integral": 0.0, "last_error": 0.0, "last_desired_q": 0.5, "gravity_tau": 0.0, "gravity_learn": 2.0}}
        target = {23: 0.5}  # error 0.5 rad: far outside the learn band
        with mock.patch.object(server, "ARM_REPLAY_LEARN_LEAK_PER_S", 2.0):
            for _ in range(120):
                store._closed_loop_arm_targets(msg, target, state, 1.0 / 120.0)
        self.assertLess(state[23]["gravity_learn"], 0.5)
        self.assertGreater(state[23]["gravity_learn"], 0.0)


if __name__ == "__main__":
    unittest.main()
