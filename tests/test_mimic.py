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
    """3D-lift retargeting (2026-07-28 rewrite). Elbow values are URDF q
    semantics verified against h1_2.urdf + the live digital twin: q=0 is a
    90° bend with the forearm forward, q≈+1.5 is the straight arm."""

    def test_hanging_arms_stay_near_neutral(self):
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.6, 0.45), l_wrist=kp(0.6, 0.6),
            r_shoulder=kp(0.4, 0.3), r_elbow=kp(0.4, 0.45), r_wrist=kp(0.4, 0.6),
        ))
        # Hanging straight arms: pitch ~0, elbow ~straight (q ~1.5), and
        # roll at the small link-offset compensation that makes the robot
        # arm hang visually straight (+0.10 right / -0.10 left).
        self.assertAlmostEqual(out[tracking.R_SHOULDER_PITCH], 0.0, delta=0.1)
        self.assertAlmostEqual(out[tracking.L_SHOULDER_PITCH], 0.0, delta=0.1)
        self.assertAlmostEqual(out[tracking.R_SHOULDER_ROLL], 0.0, delta=0.15)
        self.assertAlmostEqual(out[tracking.L_SHOULDER_ROLL], 0.0, delta=0.15)
        self.assertGreater(out[tracking.R_ELBOW], 1.3)
        self.assertGreater(out[tracking.L_ELBOW], 1.3)

    def test_person_left_arm_out_drives_robot_right_arm_out(self):
        # Person's LEFT arm horizontal outward -> robot RIGHT arm abducts
        # (NEGATIVE right-shoulder roll), elbow stays straight (q ~1.5).
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.75, 0.3), l_wrist=kp(0.9, 0.3),
            r_shoulder=kp(0.4, 0.3), r_elbow=kp(0.4, 0.45), r_wrist=kp(0.4, 0.6),
        ))
        self.assertAlmostEqual(out[tracking.R_SHOULDER_ROLL], -math.pi / 2, delta=0.15)
        self.assertGreater(out[tracking.R_ELBOW], 1.3)

    def test_person_right_arm_out_drives_robot_left_arm_out(self):
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.6, 0.45), l_wrist=kp(0.6, 0.6),
            r_shoulder=kp(0.4, 0.3), r_elbow=kp(0.25, 0.3), r_wrist=kp(0.1, 0.3),
        ))
        self.assertAlmostEqual(out[tracking.L_SHOULDER_ROLL], math.pi / 2, delta=0.15)

    def test_bent_elbow_maps_to_flexed_elbow_joint(self):
        # Upper arm horizontal out, forearm straight up: ~90° interior
        # bend -> robot elbow q near 0 (URDF 90°-bend zero).
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.4), l_elbow=kp(0.75, 0.4), l_wrist=kp(0.75, 0.25),
            r_shoulder=kp(0.4, 0.4), r_elbow=kp(0.4, 0.55), r_wrist=kp(0.4, 0.7),
        ))
        self.assertAlmostEqual(out[tracking.R_ELBOW], 0.0, delta=0.35)
        # The flex plane rotated from sagittal to the frontal plane: a
        # strongly negative yaw (clamped by MIMIC_LIMITS).
        self.assertLess(out[tracking.R_SHOULDER_YAW], -0.7)

    def test_forearm_at_camera_gives_forward_flex(self):
        # The user's benchmark case: elbow bent 90° with the forearm dead
        # forward — the wrist keypoint collapses onto the elbow. Robot
        # elbow q ~0 (90° bend forward); the yaw compensates the URDF's
        # inward forearm offset so the actual wrist direction is forward.
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.6, 0.45), l_wrist=kp(0.605, 0.452),
            r_shoulder=kp(0.4, 0.3), r_elbow=kp(0.4, 0.45), r_wrist=kp(0.4, 0.6),
        ))
        self.assertAlmostEqual(out[tracking.R_ELBOW], 0.0, delta=0.25)
        self.assertAlmostEqual(out[tracking.R_SHOULDER_YAW], -0.26, delta=0.2)
        # The hanging right arm is untouched: elbow stays straight.
        self.assertGreater(out[tracking.L_ELBOW], 1.3)

    def test_arm_at_camera_pitches_forward(self):
        # Whole upper arm collapses on screen: max forward pitch, capped.
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.605, 0.302),
            r_shoulder=kp(0.4, 0.3), r_elbow=kp(0.4, 0.45),
        ))
        self.assertAlmostEqual(out[tracking.R_SHOULDER_PITCH], -mapper.max_pitch, delta=0.05)
        self.assertAlmostEqual(out[tracking.L_SHOULDER_PITCH], 0.0, delta=0.1)

    def test_half_forward_raise_gives_partial_pitch(self):
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.6, 0.375),
            r_shoulder=kp(0.4, 0.3), r_elbow=kp(0.4, 0.45),
        ))
        self.assertLess(out[tracking.R_SHOULDER_PITCH], -0.5)
        self.assertGreater(out[tracking.R_SHOULDER_PITCH], -1.2)

    def test_slight_foreshortening_stays_in_dead_zone(self):
        # 93% of expected upper-arm length is inside the 90% depth dead
        # zone: pitch must stay exactly zero (keypoint jitter immunity).
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.6, 0.4395),
            r_shoulder=kp(0.4, 0.3), r_elbow=kp(0.4, 0.45),
        ))
        self.assertAlmostEqual(out[tracking.R_SHOULDER_PITCH], 0.0, delta=1e-9)

    def test_pitch_holds_without_both_shoulders(self):
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.605, 0.302),
        ))
        self.assertAlmostEqual(
            out[tracking.R_SHOULDER_PITCH],
            tracking.MIMIC_NEUTRAL_TEMPLATE[tracking.R_SHOULDER_PITCH],
            delta=1e-9,
        )

    def test_planar_fallback_without_scale(self):
        # One shoulder missing -> no scale -> 2D-only fallback: roll from
        # the exact elevation, elbow from the interior bend, pitch/yaw hold.
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.75, 0.3), l_wrist=kp(0.9, 0.3),
        ))
        self.assertAlmostEqual(out[tracking.R_SHOULDER_ROLL], -math.pi / 2, delta=0.05)
        self.assertGreater(out[tracking.R_ELBOW], 1.4)
        self.assertAlmostEqual(
            out[tracking.R_SHOULDER_YAW],
            tracking.MIMIC_NEUTRAL_TEMPLATE[tracking.R_SHOULDER_YAW],
            delta=1e-9,
        )

    def test_missing_keypoints_hold_previous_targets(self):
        mapper = tracking.MimicMapper()
        first = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.75, 0.3), l_wrist=kp(0.9, 0.3),
        ))
        second = mapper.targets(person_pose())  # occluded frame
        self.assertEqual(first, second)

    def test_straight_arm_holds_yaw(self):
        # Gimbal rule: a straight arm has no observable flex plane, so a
        # frame cannot rewrite yaw no matter what the forearm noise says.
        mapper = tracking.MimicMapper()
        out = mapper.targets(person_pose(
            l_shoulder=kp(0.6, 0.3), l_elbow=kp(0.6, 0.45), l_wrist=kp(0.6, 0.6),
            r_shoulder=kp(0.4, 0.3), r_elbow=kp(0.4, 0.45), r_wrist=kp(0.4, 0.6),
        ))
        self.assertAlmostEqual(
            out[tracking.R_SHOULDER_YAW],
            tracking.MIMIC_NEUTRAL_TEMPLATE[tracking.R_SHOULDER_YAW],
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

    def test_aspect_correction_matches_isotropic_geometry(self):
        # The same physical pose, once in isotropic coordinates (aspect 1)
        # and once as 16:9 per-axis-normalized detector output with
        # aspect=h/w — the joint targets must be identical. Guards the
        # 2026-07-28 live bug: vertical distances 16/9x inflated broke all
        # length ratios (arms read as always-long -> robot stuck straight).
        aspect = 0.5625  # 720/1280
        iso = {
            "l_shoulder": kp(0.6, 0.2), "l_elbow": kp(0.66, 0.31), "l_wrist": kp(0.7, 0.42),
            "r_shoulder": kp(0.4, 0.2), "r_elbow": kp(0.4, 0.31), "r_wrist": kp(0.4, 0.42),
        }
        norm = {
            name: kp(point["x"], point["y"] / aspect) for name, point in iso.items()
        }
        out_iso = tracking.MimicMapper(dead_band_rad=0.0).targets(iso)
        mapper = tracking.MimicMapper(dead_band_rad=0.0, aspect=aspect)
        out_norm = mapper.targets(norm)
        for joint, value in out_iso.items():
            self.assertAlmostEqual(out_norm[joint], value, places=6, msg=f"joint {joint}")

    def test_live_hanging_pose_reads_straight_arms(self):
        # Real detector output captured live 2026-07-28 (person standing,
        # arms relaxed, 16:9 frame). With aspect correction both elbows
        # must read near-straight — without it they read q~0.7-bent and the
        # robot looked frozen in one pose regardless of the person's arms.
        keypoints = {
            "l_shoulder": kp(0.17578125, 0.5541666666666667),
            "r_shoulder": kp(0.086083984375, 0.5722222222222222),
            "l_elbow": kp(0.212890625, 0.6756944444444445),
            "r_elbow": kp(0.07412109375, 0.6875),
            "l_wrist": kp(0.21484375, 0.8076388888888889),
            "r_wrist": kp(0.0775390625, 0.7569444444444444),
            "l_hip": kp(0.1669921875, 0.8215277777777777),
            "r_hip": kp(0.10576171875, 0.8277777777777777),
        }
        mapper = tracking.MimicMapper(dead_band_rad=0.0, aspect=0.5625)
        out = mapper.targets(keypoints)
        self.assertGreater(out[tracking.R_ELBOW], 1.0)
        # The person's right forearm was genuinely a bit foreshortened in
        # this frame (reaching toward the desk): mildly bent is correct,
        # fully-bent (the pre-fix reading) is not.
        self.assertGreater(out[tracking.L_ELBOW], 0.6)
        self.assertGreater(out[tracking.R_SHOULDER_PITCH], -0.4)
        self.assertGreater(out[tracking.L_SHOULDER_PITCH], -0.4)

    def test_neutral_template_covers_both_arms_within_limits(self):
        for joint, value in tracking.MIMIC_NEUTRAL_TEMPLATE.items():
            lo, hi = tracking.MIMIC_LIMITS[joint]
            self.assertGreaterEqual(value, lo)
            self.assertLessEqual(value, hi)
        self.assertIn(tracking.L_SHOULDER_PITCH, tracking.MIMIC_NEUTRAL_TEMPLATE)
        self.assertIn(tracking.R_SHOULDER_PITCH, tracking.MIMIC_NEUTRAL_TEMPLATE)


def _fk_mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _fk_mat_vec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def _fk_rot_rpy(rpy):
    r, p, y = rpy
    Rx = [[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]]
    Ry = [[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]]
    Rz = [[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]]
    return _fk_mat_mul(Rz, _fk_mat_mul(Ry, Rx))


def _fk_axis_rot(axis, a):
    x, y, z = axis
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return [
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


# h1_2.urdf arm chains (name, origin xyz, origin rpy, axis), torso->elbow,
# plus the elbow->wrist offset. These are the ground truth the mapper's
# analytic inverse is validated against.
_FK_CHAINS = {
    "right": (
        (((0, -0.14806, 0.42333), (-0.2618, 0, 0), (0, 1, 0)),
         ((0.0342, -0.061999, -0.0060011), (0.2618, 0, 0), (1, 0, 0)),
         ((-0.0342, 0, -0.1456), (0, 0, 0), (0, 0, 1)),
         ((0.006, -0.0329, -0.182), (0, 0, 0), (0, 1, 0))),
        (0.121, 0.0329, -0.011),
    ),
    "left": (
        (((0, 0.14806, 0.42333), (0.2618, 0, 0), (0, 1, 0)),
         ((0.0342, 0.061999, -0.0060011), (-0.2618, 0, 0), (1, 0, 0)),
         ((-0.0342, 0, -0.1456), (0, 0, 0), (0, 0, 1)),
         ((0.006, 0.0329, -0.182), (0, 0, 0), (0, 1, 0))),
        (0.121, -0.0329, -0.011),
    ),
}


class MimicRoundTripTests(unittest.TestCase):
    """Gold test: URDF forward kinematics -> orthographic projection ->
    MimicMapper -> the commanded joints must come back within tolerance.
    Tolerances reflect the deliberate depth dead zone (safety) and the
    small link-offset approximations."""

    TOL = {"pitch": 0.35, "roll": 0.10, "yaw": 0.30, "elbow": 0.45}
    POSES = (
        ("standby", {"pitch": 0.15, "roll": -0.2, "yaw": 0.0, "elbow": 0.7}),
        ("forward raise", {"pitch": -0.9, "roll": -0.15, "yaw": 0.0, "elbow": 1.2}),
        ("out sideways", {"pitch": -0.2, "roll": -1.1, "yaw": 0.3, "elbow": 0.5}),
        ("curl", {"pitch": -0.3, "roll": -0.35, "yaw": -0.5, "elbow": -0.4}),
        ("straight hang", {"pitch": 0.0, "roll": -0.1, "yaw": 0.0, "elbow": 1.5}),
    )

    @staticmethod
    def _fk_dirs(side, q):
        chain, forearm = _FK_CHAINS[side]
        R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        p = [0.0, 0.0, 0.0]
        joints = ("pitch", "roll", "yaw", "elbow")
        pos = {}
        for (xyz, rpy, axis), name in zip(chain, joints):
            step = _fk_mat_vec(R, list(xyz))
            p = [p[i] + step[i] for i in range(3)]
            R = _fk_mat_mul(R, _fk_rot_rpy(rpy))
            R = _fk_mat_mul(R, _fk_axis_rot(axis, q[name]))
            pos[name] = (list(p), [row[:] for row in R])
        sh = pos["roll"][0]
        el, elR = pos["elbow"]
        wr = [el[i] + c for i, c in enumerate(_fk_mat_vec(elR, list(forearm)))]

        def unit(v):
            n = math.sqrt(sum(c * c for c in v))
            return [c / n for c in v]

        return unit([el[i] - sh[i] for i in range(3)]), unit([wr[i] - el[i] for i in range(3)])

    def test_fk_round_trip_both_arms(self):
        width = 0.2
        joint_map = {
            "right": (tracking.R_SHOULDER_PITCH, tracking.R_SHOULDER_ROLL,
                      tracking.R_SHOULDER_YAW, tracking.R_ELBOW),
            "left": (tracking.L_SHOULDER_PITCH, tracking.L_SHOULDER_ROLL,
                     tracking.L_SHOULDER_YAW, tracking.L_ELBOW),
        }
        for side in ("right", "left"):
            y_sign = -1.0 if side == "right" else 1.0
            prefix = "l" if side == "right" else "r"
            out_sign = 1.0 if prefix == "l" else -1.0
            for label, base in self.POSES:
                q = dict(base)
                if side == "left":
                    q["roll"], q["yaw"] = -q["roll"], -q["yaw"]
                u, f = self._fk_dirs(side, q)
                mapper = tracking.MimicMapper(dead_band_rad=0.0)
                lu = mapper.upper_ratio * width
                lf = mapper.fore_ratio * width

                def img(v, length):
                    lat, down = v[1] / y_sign, -v[2]
                    return (out_sign * lat * length, down * length)

                sx = 0.6 if prefix == "l" else 0.4
                du, df = img(u, lu), img(f, lf)
                pose = {
                    "l_shoulder": kp(0.6, 0.3), "r_shoulder": kp(0.4, 0.3),
                    f"{prefix}_elbow": kp(sx + du[0], 0.3 + du[1]),
                    f"{prefix}_wrist": kp(sx + du[0] + df[0], 0.3 + du[1] + df[1]),
                }
                got = mapper.targets(pose)
                for name, joint in zip(("pitch", "roll", "yaw", "elbow"), joint_map[side]):
                    expected = tracking._clamp(q[name], *tracking.MIMIC_LIMITS[joint])
                    self.assertLessEqual(
                        abs(got[joint] - expected), self.TOL[name],
                        msg=f"{side} {label} {name}: got {got[joint]:+.2f}, "
                            f"want {expected:+.2f}",
                    )
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

    def test_mimic_on_replaces_running_pointing_session(self):
        # Mode switch is one deliberate action: with a Bullseye pointing
        # session live, a confirmed mimic request stops it and proceeds to
        # the start gates instead of failing with "already running".
        import threading

        store = self.make_store()
        cancel = threading.Event()
        fake_session = threading.Thread(target=cancel.wait, daemon=True)
        fake_session.start()
        store.track_thread = fake_session
        store.track_cancel = cancel
        old = server.TRACKING_ENABLED
        server.TRACKING_ENABLED = True
        try:
            status, response = store.set_mimic_mode(
                {"on": True, "armed": True, "i_understand_risk": True}
            )
        finally:
            server.TRACKING_ENABLED = old
        # The old session was stopped…
        fake_session.join(timeout=1.0)
        self.assertFalse(fake_session.is_alive())
        # …and the request reached the DDS gate (503 offline), NOT the
        # "already running" 409.
        self.assertEqual(status, 503)
        self.assertFalse(response["mimic_mode"])

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
