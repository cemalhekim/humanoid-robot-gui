import unittest

import tracking


class MapperTests(unittest.TestCase):
    def setUp(self):
        self.mapper = tracking.PointingMapper()

    def test_center_of_image_gives_template_pose(self):
        targets = self.mapper.targets(0.5, 0.5)
        self.assertEqual(set(targets), set(tracking.POINTING_TEMPLATE))
        self.assertAlmostEqual(
            targets[tracking.R_SHOULDER_YAW],
            tracking.POINTING_TEMPLATE[tracking.R_SHOULDER_YAW],
            places=5,
        )

    def test_person_left_of_center_swings_yaw_toward_them(self):
        # Field-verified 2026-07-22: the arm mirrored the person until the yaw
        # sign was flipped — image-left (small cx) must give LARGER joint-22 yaw.
        left = self.mapper.targets(0.2, 0.5)[tracking.R_SHOULDER_YAW]
        right = self.mapper.targets(0.8, 0.5)[tracking.R_SHOULDER_YAW]
        self.assertNotEqual(left, right)
        self.assertGreater(left, right)

    def test_person_higher_in_image_raises_arm(self):
        # H1-2 shoulder pitch: NEGATIVE raises the arm forward/up (operator's
        # authored pointing pose has LeftShoulderPitch -1.315). Higher person
        # (smaller cy) must therefore give a MORE NEGATIVE pitch.
        high = self.mapper.targets(0.5, 0.2)[tracking.R_SHOULDER_PITCH]
        low = self.mapper.targets(0.5, 0.8)[tracking.R_SHOULDER_PITCH]
        self.assertLess(high, low)

    def test_center_gives_exact_calibrated_pose(self):
        # Operator calibration 2026-07-22 (recordings/20260722-153832): person
        # dead-center in the camera must produce exactly the authored pose.
        targets = self.mapper.targets(0.5, 0.5)
        for joint, value in tracking.POINTING_TEMPLATE.items():
            self.assertAlmostEqual(targets[joint], value, places=5, msg=f"joint {joint}")

    def test_template_extends_arm_forward(self):
        # The pointing template must hold the arm raised well forward
        # (mirrored from the operator's saved pose), not near the side.
        self.assertLess(tracking.POINTING_TEMPLATE[tracking.R_SHOULDER_PITCH], -1.0)

    def test_targets_always_inside_track_limits(self):
        for cx, cy in [(0.0, 0.0), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0)]:
            for joint, value in self.mapper.targets(cx, cy).items():
                lo, hi = tracking.TRACK_LIMITS[joint]
                self.assertGreaterEqual(value, lo, f"joint {joint} cx={cx} cy={cy}")
                self.assertLessEqual(value, hi, f"joint {joint} cx={cx} cy={cy}")

    def test_dead_band_freezes_small_motion(self):
        a = self.mapper.targets(0.500, 0.500)
        b = self.mapper.targets(0.515, 0.510)  # inside +-0.03 dead band
        self.assertEqual(a, b)


class RateLimiterTests(unittest.TestCase):
    def test_step_is_velocity_bounded(self):
        limiter = tracking.RateLimiter(max_step_rad_s=0.35)
        current = {20: 0.0}
        target = {20: 1.0}
        stepped = limiter.step(current, target, dt=0.1)
        self.assertAlmostEqual(stepped[20], 0.035, places=6)

    def test_step_does_not_overshoot(self):
        limiter = tracking.RateLimiter(max_step_rad_s=0.35)
        stepped = limiter.step({20: 0.99}, {20: 1.0}, dt=1.0)
        self.assertAlmostEqual(stepped[20], 1.0, places=6)


class AssociateTests(unittest.TestCase):
    def person(self, cx, cy, area):
        half = area ** 0.5 / 2
        return {"cx": cx, "cy": cy, "x1": cx - half, "x2": cx + half,
                "y1": cy - half, "y2": cy + half, "conf": 0.9}

    def test_no_previous_target_picks_largest(self):
        big = self.person(0.7, 0.5, 0.20)
        small = self.person(0.2, 0.5, 0.05)
        self.assertEqual(tracking.associate([small, big], None, None), big)

    def test_sticks_with_nearest_to_previous(self):
        near = self.person(0.32, 0.5, 0.05)
        far_big = self.person(0.9, 0.5, 0.20)
        self.assertEqual(tracking.associate([near, far_big], 0.30, 0.5), near)

    def test_empty_returns_none(self):
        self.assertIsNone(tracking.associate([], 0.5, 0.5))


class TrackStateTests(unittest.TestCase):
    def test_fresh_detection_keeps_tracking(self):
        state = tracking.TrackState(stale_after_s=1.5, hold_s=2.0, max_failures=10)
        state.on_detection([{"cx": 0.5, "cy": 0.5, "x1": 0.4, "x2": 0.6,
                             "y1": 0.3, "y2": 0.7, "conf": 0.9}], now=100.0)
        self.assertEqual(state.phase, "tracking")

    def test_target_lost_holds_then_goes_stale(self):
        state = tracking.TrackState(stale_after_s=1.5, hold_s=2.0, max_failures=10)
        state.on_detection([{"cx": 0.5, "cy": 0.5, "x1": 0.4, "x2": 0.6,
                             "y1": 0.3, "y2": 0.7, "conf": 0.9}], now=100.0)
        state.on_detection([], now=101.0)
        self.assertEqual(state.phase, "hold")
        state.on_detection([], now=103.5)
        self.assertEqual(state.phase, "stale")

    def test_consecutive_failures_abort(self):
        state = tracking.TrackState(stale_after_s=1.5, hold_s=2.0, max_failures=3)
        for i in range(3):
            state.on_failure(now=100.0 + i)
        self.assertEqual(state.phase, "aborted")

    def test_success_resets_failure_count(self):
        state = tracking.TrackState(stale_after_s=1.5, hold_s=2.0, max_failures=3)
        state.on_failure(now=100.0)
        state.on_failure(now=100.1)
        state.on_detection([{"cx": 0.5, "cy": 0.5, "x1": 0.4, "x2": 0.6,
                             "y1": 0.3, "y2": 0.7, "conf": 0.9}], now=100.2)
        self.assertEqual(state.failures, 0)
        self.assertEqual(state.phase, "tracking")


if __name__ == "__main__":
    unittest.main()
