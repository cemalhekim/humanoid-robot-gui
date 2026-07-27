"""Mimic Mode: keypoint→joint retargeting math and endpoint gating."""
import math
import unittest

import server
import tracking


def kp(x, y):
    return {"x": x, "y": y, "conf": 0.9}


def person_pose(**keypoints):
    """Build a detector keypoints dict (image space, y down)."""
    return keypoints


class MimicMapperTests(unittest.TestCase):
    def test_hanging_arms_stay_near_neutral(self):
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.6, 0.45), l_wrist=kp(0.6, 0.6),
            r_shoulder=kp(0.4, 0.3), r_elbow=kp(0.4, 0.45), r_wrist=kp(0.4, 0.6),
        ))
        self.assertAlmostEqual(out[tracking.R_SHOULDER_ROLL], 0.0, delta=0.1)
        self.assertAlmostEqual(out[tracking.L_SHOULDER_ROLL], 0.0, delta=0.1)
        self.assertAlmostEqual(out[tracking.R_ELBOW], 0.0, delta=0.1)
        self.assertAlmostEqual(out[tracking.L_ELBOW], 0.0, delta=0.1)

    def test_person_left_arm_out_drives_robot_right_arm_out(self):
        # Person's LEFT arm horizontal outward (their left = image right =
        # robot's right side, camera is robot-relative). Mirror mimic: the
        # robot RIGHT arm abducts, which is NEGATIVE right-shoulder roll.
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.75, 0.3), l_wrist=kp(0.9, 0.3),
        ))
        self.assertAlmostEqual(out[tracking.R_SHOULDER_ROLL], -math.pi / 2, delta=0.1)
        # Straight arm: elbow stays extended.
        self.assertAlmostEqual(out[tracking.R_ELBOW], 0.0, delta=0.1)
        # The other arm was never seen: it holds neutral.
        self.assertAlmostEqual(
            out[tracking.L_SHOULDER_ROLL],
            tracking.MIMIC_NEUTRAL_TEMPLATE[tracking.L_SHOULDER_ROLL],
            delta=1e-9,
        )

    def test_person_right_arm_out_drives_robot_left_arm_out(self):
        # Person's RIGHT arm outward points toward image LEFT (-x).
        # Robot LEFT arm abducts with POSITIVE left-shoulder roll.
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            r_shoulder=kp(0.4, 0.3), r_elbow=kp(0.25, 0.3), r_wrist=kp(0.1, 0.3),
        ))
        self.assertAlmostEqual(out[tracking.L_SHOULDER_ROLL], math.pi / 2, delta=0.1)

    def test_bent_elbow_maps_to_flexed_elbow_joint(self):
        # Upper arm horizontal out, forearm straight up: a ~90° interior bend.
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.4), l_elbow=kp(0.75, 0.4), l_wrist=kp(0.75, 0.25),
        ))
        self.assertAlmostEqual(out[tracking.R_ELBOW], math.pi / 2, delta=0.1)

    def test_overhead_arm_is_clamped_to_mimic_limits(self):
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.4), l_elbow=kp(0.61, 0.2), l_wrist=kp(0.62, 0.05),
        ))
        lo, hi = tracking.MIMIC_LIMITS[tracking.R_SHOULDER_ROLL]
        self.assertGreaterEqual(out[tracking.R_SHOULDER_ROLL], lo)
        self.assertLessEqual(out[tracking.R_SHOULDER_ROLL], hi)

    def test_missing_keypoints_hold_previous_targets(self):
        mapper = tracking.MimicMapper()
        first = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.75, 0.3), l_wrist=kp(0.9, 0.3),
        ))
        second = mapper.targets(person_pose())  # occluded frame
        self.assertEqual(first, second)

    def test_foreshortened_segment_is_ignored(self):
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.601, 0.301),
        ))
        self.assertAlmostEqual(
            out[tracking.R_SHOULDER_ROLL],
            tracking.MIMIC_NEUTRAL_TEMPLATE[tracking.R_SHOULDER_ROLL],
            delta=1e-9,
        )

    def test_all_outputs_stay_inside_mimic_limits(self):
        mapper = tracking.MimicMapper()
        for lx in (0.0, 0.3, 0.6, 0.9, 1.0):
            for ly in (0.0, 0.4, 0.8, 1.0):
                out = mapper.targets(person_pose(
                    l_shoulder=kp(0.6, 0.4), l_elbow=kp(lx, ly), l_wrist=kp(1.0 - lx, 1.0 - ly),
                    r_shoulder=kp(0.4, 0.4), r_elbow=kp(1.0 - lx, ly), r_wrist=kp(lx, 1.0 - ly),
                ))
                for joint, value in out.items():
                    lo, hi = tracking.MIMIC_LIMITS[joint]
                    self.assertGreaterEqual(value, lo - 1e-9, msg=f"joint {joint}")
                    self.assertLessEqual(value, hi + 1e-9, msg=f"joint {joint}")

    def test_has_upper_body(self):
        self.assertTrue(tracking.has_upper_body(
            {"keypoints": person_pose(l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.7, 0.3))}
        ))
        self.assertFalse(tracking.has_upper_body({"keypoints": {"nose": kp(0.5, 0.1)}}))
        self.assertFalse(tracking.has_upper_body({}))

    def test_neutral_template_covers_both_arms_within_limits(self):
        for joint, value in tracking.MIMIC_NEUTRAL_TEMPLATE.items():
            lo, hi = tracking.MIMIC_LIMITS[joint]
            self.assertGreaterEqual(value, lo)
            self.assertLessEqual(value, hi)
        self.assertIn(tracking.L_SHOULDER_PITCH, tracking.MIMIC_NEUTRAL_TEMPLATE)
        self.assertIn(tracking.R_SHOULDER_PITCH, tracking.MIMIC_NEUTRAL_TEMPLATE)


class MimicPayloadTests(unittest.TestCase):
    def test_mode_defaults_to_point(self):
        parsed = server.parse_track_payload({})
        self.assertEqual(parsed["mode"], "point")

    def test_mimic_mode_requires_webcam(self):
        with self.assertRaises(ValueError):
            server.parse_track_payload({"mode": "mimic", "camera": "head"})
        parsed = server.parse_track_payload({"mode": "mimic", "camera": "webcam"})
        self.assertEqual(parsed["mode"], "mimic")

    def test_unknown_mode_is_rejected(self):
        with self.assertRaises(ValueError):
            server.parse_track_payload({"mode": "dance"})


class MimicLoopTests(unittest.TestCase):
    def test_mimic_session_commands_both_arms_from_keypoints(self):
        import threading
        from types import SimpleNamespace
        from unittest import mock

        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        motors = [SimpleNamespace(q=0.0, dq=0.0, tau_est=0.0) for _ in range(35)]
        store.lowstate_msg = SimpleNamespace(motor_state=motors)
        cancel = threading.Event()
        publisher = mock.MagicMock()
        publisher.Write.side_effect = lambda _cmd: cancel.set()
        store.wrist_publisher = publisher
        store.track_config = {
            "camera": "webcam",
            "mode": "mimic",
            "permanent": True,
            "closed_loop": True,
            "target": None,
            "target_id": None,
        }
        person = {
            "id": 3,
            "cx": 0.5, "cy": 0.5,
            "x1": 0.2, "x2": 0.8, "y1": 0.1, "y2": 0.9,
            "conf": 0.9,
            "head": {"x": 0.5, "y": 0.2},
            "keypoints": person_pose(
                l_shoulder=kp(0.6, 0.35), l_elbow=kp(0.75, 0.35), l_wrist=kp(0.9, 0.35),
                r_shoulder=kp(0.4, 0.35), r_elbow=kp(0.4, 0.5), r_wrist=kp(0.4, 0.65),
            ),
        }
        captured = {}

        def capture_targets(_msg, desired, _state, _dt, _tuning):
            captured.update(desired)
            return desired, {}, {}

        with mock.patch.object(store, "sentry_stream_subscribe"), \
                mock.patch.object(store, "sentry_stream_unsubscribe"), \
                mock.patch.object(
                    store, "wait_sentry_result",
                    return_value=({"ok": True, "persons": [person]}, 1),
                ), \
                mock.patch.object(
                    store, "_closed_loop_arm_targets", side_effect=capture_targets,
                ), \
                mock.patch.object(
                    store, "_build_arm_sdk_trajectory_cmd", return_value=object()
                ), \
                mock.patch.object(
                    store, "request_home", return_value=(200, {"ok": True})
                ):
            store._run_tracking(cancel)

        publisher.Write.assert_called_once()
        # Both arms are commanded: the pointing template never includes the
        # left arm, so left joints prove the mimic path was taken.
        self.assertIn(tracking.L_SHOULDER_ROLL, captured)
        self.assertIn(tracking.R_SHOULDER_ROLL, captured)
        self.assertIn(tracking.L_ELBOW, captured)


class MimicEndpointTests(unittest.TestCase):
    def make_store(self):
        return server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_mimic_on_requires_risk_ack(self):
        store = self.make_store()
        status, response = store.set_mimic_mode({"on": True})
        self.assertEqual(status, 403)
        self.assertFalse(response["ok"])
        self.assertFalse(store.track_snapshot()["mimic_mode"])

    def test_mimic_on_requires_boolean(self):
        store = self.make_store()
        status, _ = store.set_mimic_mode({"on": "yes"})
        self.assertEqual(status, 400)

    def test_mimic_on_without_dds_fails_and_disarms(self):
        store = self.make_store()
        old = server.TRACKING_ENABLED
        server.TRACKING_ENABLED = True
        try:
            status, response = store.set_mimic_mode(
                {"on": True, "armed": True, "i_understand_risk": True}
            )
        finally:
            server.TRACKING_ENABLED = old
        self.assertEqual(status, 503)  # no DDS publisher offline
        # The switch must not stay armed with no session running behind it.
        self.assertFalse(response["mimic_mode"])
        self.assertFalse(store.track_snapshot()["mimic_mode"])

    def test_mimic_start_refused_when_feature_disabled(self):
        store = self.make_store()
        old = server.TRACKING_ENABLED
        server.TRACKING_ENABLED = False
        try:
            status, _ = store.set_mimic_mode(
                {"on": True, "armed": True, "i_understand_risk": True}
            )
        finally:
            server.TRACKING_ENABLED = old
        self.assertEqual(status, 409)
        self.assertFalse(store.track_snapshot()["mimic_mode"])

    def test_mimic_session_refused_while_mimic_switch_off(self):
        # Direct /api/track/start with mode=mimic must respect the master
        # switch exactly like Bullseye locks respect theirs.
        store = self.make_store()
        old = server.TRACKING_ENABLED
        server.TRACKING_ENABLED = True
        try:
            status, response = store.request_track_start({
                "armed": True,
                "i_understand_risk": True,
                "mode": "mimic",
                "camera": "webcam",
            })
        finally:
            server.TRACKING_ENABLED = old
        self.assertEqual(status, 409)
        self.assertIn("Mimic", response["error"])

    def test_mimic_off_is_always_accepted(self):
        store = self.make_store()
        status, response = store.set_mimic_mode({"on": False})
        self.assertEqual(status, 200)
        self.assertFalse(response["mimic_mode"])

    def test_snapshot_reports_mimic_mode(self):
        store = self.make_store()
        self.assertIn("mimic_mode", store.track_snapshot())


class MimicUiSourceTests(unittest.TestCase):
    def test_skeleton_overlay_draws_the_chains_mimic_consumes(self):
        with open("static/app.js") as fh:
            src = fh.read()
        # The overlay must draw exactly the retargeted chains: both
        # shoulder→elbow→wrist arms plus the shoulder line.
        self.assertIn('["l_shoulder", "l_elbow"], ["l_elbow", "l_wrist"]', src)
        self.assertIn('["r_shoulder", "r_elbow"], ["r_elbow", "r_wrist"]', src)
        self.assertIn('["l_shoulder", "r_shoulder"]', src)

    def test_mimic_state_reaches_the_bullseye_panel(self):
        with open("static/app.js") as fh:
            src = fh.read()
        # setupMimic publishes, setupBullseye consumes: stream gating, box
        # gating and the skeleton pass all read the shared dataset flag.
        self.assertIn('document.body.dataset.mimicOn = serverOn ? "1" : "0"', src)
        self.assertIn('document.body.dataset.mimicOn === "1"', src)
        self.assertIn("(isOn() || isMimicOn())", src)
        self.assertIn("(!isBoxesOn() && !isMimicOn())", src)


if __name__ == "__main__":
    unittest.main()
