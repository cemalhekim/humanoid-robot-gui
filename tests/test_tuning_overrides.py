import json
import os
import tempfile
import unittest
from pathlib import Path

import server


class TuningOverridesTest(unittest.TestCase):
    def setUp(self) -> None:
        self.saved = {k: getattr(server, k) for k in (
            "ARM_REPLAY_PID_GAINS", "ARM_REPLAY_STALL_SECONDS", "ARM_SDK_KP",
            "ARM_SDK_GAIN_BY_INDEX", "ARM_REPLAY_LOCK_TOLERANCE_RAD", "ARM_REPLAY_TOLERANCE_RAD",
        )}
        self.env = os.environ.get("RTW_TUNING_JSON")

    def tearDown(self) -> None:
        for k, v in self.saved.items():
            setattr(server, k, v)
        if self.env is None:
            os.environ.pop("RTW_TUNING_JSON", None)
        else:
            os.environ["RTW_TUNING_JSON"] = self.env

    def _with_file(self, payload: dict) -> dict:
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as handle:
            json.dump(payload, handle)
        os.environ["RTW_TUNING_JSON"] = handle.name
        try:
            return server._apply_tuning_overrides()
        finally:
            Path(handle.name).unlink()

    def test_unset_is_a_no_op(self) -> None:
        os.environ.pop("RTW_TUNING_JSON", None)
        self.assertEqual(server._apply_tuning_overrides(), {})
        self.assertEqual(server.ARM_REPLAY_STALL_SECONDS, self.saved["ARM_REPLAY_STALL_SECONDS"])

    def test_overrides_scalars_dicts_lists_and_derived_values(self) -> None:
        kp = list(self.saved["ARM_SDK_KP"])
        kp[0] = 99
        applied = self._with_file({
            "ARM_REPLAY_STALL_SECONDS": 4.0,
            "ARM_REPLAY_PID_GAINS": {"shoulder": [0.5, 0.01, 0.02]},
            "ARM_SDK_KP": kp,
            "ARM_REPLAY_LOCK_TOLERANCE_RAD": 0.02,
        })
        self.assertEqual(set(applied), {"ARM_REPLAY_STALL_SECONDS", "ARM_REPLAY_PID_GAINS", "ARM_SDK_KP", "ARM_REPLAY_LOCK_TOLERANCE_RAD"})
        self.assertEqual(server.ARM_REPLAY_STALL_SECONDS, 4.0)
        self.assertEqual(server.ARM_REPLAY_PID_GAINS["shoulder"], (0.5, 0.01, 0.02))
        self.assertEqual(server.ARM_REPLAY_PID_GAINS["elbow"], self.saved["ARM_REPLAY_PID_GAINS"]["elbow"])
        self.assertEqual(server.ARM_SDK_GAIN_BY_INDEX[13][0], 99.0)
        self.assertEqual(server.ARM_REPLAY_TOLERANCE_RAD, 0.02)

    def test_unknown_key_aborts(self) -> None:
        with self.assertRaises(SystemExit):
            self._with_file({"LOCO_LIMITS": {}})
        with self.assertRaises(SystemExit):
            self._with_file({"ARM_REPLAY_DOES_NOT_EXIST": 1})

    def test_shape_mismatch_aborts(self) -> None:
        with self.assertRaises(SystemExit):
            self._with_file({"ARM_SDK_KP": [1, 2, 3]})
        with self.assertRaises(SystemExit):
            self._with_file({"ARM_REPLAY_PID_GAINS": {"knee": [1, 2, 3]}})


if __name__ == "__main__":
    unittest.main()
