import unittest
from unittest import mock

import server


class LocoSpeedScaleParsingTest(unittest.TestCase):
    def test_unset_or_blank_defaults_to_one(self) -> None:
        self.assertEqual(server.loco_speed_scale_from_env(None), 1.0)
        self.assertEqual(server.loco_speed_scale_from_env(""), 1.0)
        self.assertEqual(server.loco_speed_scale_from_env("   "), 1.0)

    def test_scale_is_applied(self) -> None:
        self.assertEqual(server.loco_speed_scale_from_env("1.5"), 1.5)
        self.assertEqual(server.loco_speed_scale_from_env("2.0"), 2.0)
        self.assertEqual(server.loco_speed_scale_from_env("0.5"), 0.5)

    def test_scale_is_capped_at_two(self) -> None:
        self.assertEqual(server.LOCO_SPEED_SCALE_MAX, 2.0)
        self.assertEqual(server.loco_speed_scale_from_env("2.01"), 2.0)
        self.assertEqual(server.loco_speed_scale_from_env("10"), 2.0)
        self.assertEqual(server.loco_speed_scale_from_env("inf"), 1.0)

    def test_garbage_falls_back_to_default(self) -> None:
        for raw in ("abc", "nan", "0", "-1", "2x"):
            self.assertEqual(server.loco_speed_scale_from_env(raw), 1.0, raw)


class LocoLimitsBuildTest(unittest.TestCase):
    def test_default_limits_are_the_shipped_envelope(self) -> None:
        limits = server.build_loco_limits(1.0)
        self.assertEqual(limits["vx"], [-1.0, 1.0])
        self.assertEqual(limits["vy"], [-0.5, 0.5])
        self.assertEqual(limits["vyaw"], [-1.0, 1.0])
        self.assertEqual(limits["duration"], [0.1, 10.0])

    def test_scale_doubles_only_the_velocity_fields(self) -> None:
        limits = server.build_loco_limits(2.0)
        self.assertEqual(limits["vx"], [-2.0, 2.0])
        self.assertEqual(limits["vy"], [-1.0, 1.0])
        self.assertEqual(limits["vyaw"], [-2.0, 2.0])
        for name, bounds in server.LOCO_FIXED_LIMITS.items():
            self.assertEqual(limits[name], list(bounds), name)

    def test_module_limits_match_the_env_scale(self) -> None:
        self.assertEqual(server.LOCO_LIMITS, server.build_loco_limits(server.LOCO_SPEED_SCALE))
        self.assertLessEqual(server.LOCO_SPEED_SCALE, server.LOCO_SPEED_SCALE_MAX)


class LocoCommandUsesScaledLimitsTest(unittest.TestCase):
    """command_loco clamps against LOCO_LIMITS; the gates around it are untouched."""

    def _velocity(self, **fields):
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        payload = {"action": "velocity", "armed": True, "i_understand_risk": True, **fields}
        return store.command_loco(payload)

    def test_one_x_rejects_above_shipped_envelope(self) -> None:
        with mock.patch.object(server, "LOCO_LIMITS", server.build_loco_limits(1.0)):
            status, response = self._velocity(vx=1.5)
            self.assertEqual(status, 400)
            self.assertIn("vx must be between -1.0 and 1.0", response["error"])
            # In range passes validation and only then hits the no-loco-client 503.
            self.assertEqual(self._velocity(vx=1.0, vy=0.5, vyaw=1.0)[0], 503)

    def test_two_x_accepts_doubled_envelope_and_rejects_beyond(self) -> None:
        with mock.patch.object(server, "LOCO_LIMITS", server.build_loco_limits(2.0)):
            self.assertEqual(self._velocity(vx=2.0, vy=1.0, vyaw=2.0)[0], 503)
            self.assertEqual(self._velocity(vx=-2.0, vy=-1.0, vyaw=-2.0)[0], 503)
            for field, value in (("vx", 2.01), ("vy", 1.01), ("vyaw", -2.01)):
                status, response = self._velocity(**{field: value})
                self.assertEqual(status, 400, field)
                self.assertIn(field, response["error"])

    def test_scale_does_not_touch_duration_or_the_risk_gate(self) -> None:
        with mock.patch.object(server, "LOCO_LIMITS", server.build_loco_limits(2.0)):
            status, response = self._velocity(vx=0.5, duration=10.5)
            self.assertEqual(status, 400)
            self.assertIn("duration", response["error"])
            store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
            status, response = store.command_loco({"action": "velocity", "vx": 0.5})
            self.assertEqual(status, 400)
            self.assertIn("armed", response["error"])
            # Stops stay ungated (503 = passed validation, no loco client).
            self.assertEqual(store.command_loco({"action": "stop_move"})[0], 503)

    def test_status_payload_echoes_limits_and_scale(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        payload = store.loco_snapshot()
        self.assertEqual(payload["limits"], server.LOCO_LIMITS)
        self.assertEqual(payload["speed_scale"], server.LOCO_SPEED_SCALE)


if __name__ == "__main__":
    unittest.main()
