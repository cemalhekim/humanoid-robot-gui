from __future__ import annotations

import unittest

import server


class XrResumeDecisionTest(unittest.TestCase):
    def test_idle_when_nothing_was_suspended(self) -> None:
        self.assertEqual(server.xr_resume_action(pending=False, age_s=99, replay_alive=False, track_alive=False, holding=False), "idle")

    def test_waits_through_the_grace_window_even_if_arms_look_free(self) -> None:
        # the replay thread starts a few ms after the suspend; never race it
        self.assertEqual(server.xr_resume_action(pending=True, age_s=1.0, replay_alive=False, track_alive=False, holding=False), "wait")

    def test_waits_while_a_replay_is_still_moving(self) -> None:
        self.assertEqual(server.xr_resume_action(pending=True, age_s=10, replay_alive=True, track_alive=False, holding=False), "wait")

    def test_releases_a_converged_hold(self) -> None:
        self.assertEqual(server.xr_resume_action(pending=True, age_s=10, replay_alive=True, track_alive=False, holding=True), "release")

    def test_releases_a_replay_stuck_converging_past_the_max_wait(self) -> None:
        self.assertEqual(server.xr_resume_action(pending=True, age_s=31, replay_alive=True, track_alive=False, holding=False, max_wait_s=30), "release")

    def test_never_cuts_person_tracking_short(self) -> None:
        self.assertEqual(server.xr_resume_action(pending=True, age_s=500, replay_alive=False, track_alive=True, holding=False), "wait")

    def test_starts_xr_once_the_arms_are_free(self) -> None:
        self.assertEqual(server.xr_resume_action(pending=True, age_s=5, replay_alive=False, track_alive=False, holding=False), "start")


class XrResumeStoreTest(unittest.TestCase):
    def test_snapshot_defaults_and_toggle(self) -> None:
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        snap = store.xr_resume_snapshot()
        self.assertFalse(snap["pending"])
        self.assertIsNone(snap["suspended_at"])
        status, body = store.set_xr_resume({"enabled": False})
        self.assertEqual(status, 200)
        self.assertFalse(body["enabled"])
        self.assertEqual(store.set_xr_resume({"enabled": "no"})[0], 400)
        self.assertIn("xr", store.snapshot())


if __name__ == "__main__":
    unittest.main()
