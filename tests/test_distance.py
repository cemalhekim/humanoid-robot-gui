import math
import unittest

from distance import (
    DistanceEstimator,
    REAL_HEIGHT_M,
    REAL_SHOULDER_WIDTH_M,
    REAL_TORSO_M,
    _Smoother,
)


# ---- synthetic-geometry helpers ---------------------------------------------
#
# These mirror the pinhole formulas documented in distance.py's module
# docstring, computed independently here (not by calling estimator
# internals) so the tests are a real check of the implementation, not a
# tautology.

def focal_h(fov_h_deg: float, aspect: float = 4 / 3) -> float:
    return 0.5 / math.tan(math.radians(fov_h_deg) / 2.0)


def azimuth(x_norm: float, f_h: float) -> float:
    return math.atan2(x_norm - 0.5, f_h)


def bbox_person_at_distance(distance_m, fov_h_deg=70.0, aspect=4 / 3, cx=0.5, conf=0.9):
    """A bbox-only person whose full-height apparent size implies distance_m."""
    f_h = focal_h(fov_h_deg, aspect)
    size = REAL_HEIGHT_M * f_h / distance_m
    dy = size * aspect
    half = dy / 2.0
    return {
        "x1": cx - 0.1, "x2": cx + 0.1,
        "y1": 0.5 - half, "y2": 0.5 + half,
        "cx": cx, "cy": 0.5, "conf": conf,
    }


def torso_keypoints_at_distance(distance_m, fov_h_deg=70.0, aspect=4 / 3, cx=0.5, cy=0.5):
    """Purely-vertical torso (mid-shoulder/mid-hip) implying distance_m."""
    f_h = focal_h(fov_h_deg, aspect)
    size = REAL_TORSO_M * f_h / distance_m
    dy = size * aspect
    half = dy / 2.0
    return {
        "l_shoulder": {"x": cx - 0.05, "y": cy - half, "conf": 0.9},
        "r_shoulder": {"x": cx + 0.05, "y": cy - half, "conf": 0.9},
        "l_hip": {"x": cx - 0.05, "y": cy + half, "conf": 0.9},
        "r_hip": {"x": cx + 0.05, "y": cy + half, "conf": 0.9},
    }


def shoulder_keypoints_at_distance(distance_m, fov_h_deg=70.0, aspect=4 / 3, cx=0.5, cy=0.5):
    """Purely-horizontal shoulder pair implying distance_m."""
    f_h = focal_h(fov_h_deg, aspect)
    size = REAL_SHOULDER_WIDTH_M * f_h / distance_m
    half = size / 2.0
    return {
        "l_shoulder": {"x": cx - half, "y": cy, "conf": 0.9},
        "r_shoulder": {"x": cx + half, "y": cy, "conf": 0.9},
    }


class PinholeSanityTests(unittest.TestCase):
    def test_bbox_height_cue_recovers_known_distance(self):
        est = DistanceEstimator()
        person = bbox_person_at_distance(3.0)
        result = est.estimate("p1", person, now=0.0)
        self.assertIsNotNone(result)
        self.assertEqual(result["cue"], "bbox_height")
        self.assertAlmostEqual(result["camera_m"], 3.0, delta=0.05 * 3.0)

    def test_torso_cue_recovers_known_distance(self):
        est = DistanceEstimator()
        person = {
            "x1": 0.4, "x2": 0.6, "y1": 0.2, "y2": 0.8,
            "cx": 0.5, "cy": 0.5, "conf": 0.9,
            "keypoints": torso_keypoints_at_distance(2.4),
        }
        result = est.estimate("p2", person, now=0.0)
        self.assertIsNotNone(result)
        self.assertEqual(result["cue"], "torso")
        self.assertAlmostEqual(result["camera_m"], 2.4, delta=0.05 * 2.4)


class CuePriorityTests(unittest.TestCase):
    def test_torso_used_when_all_cues_present(self):
        est = DistanceEstimator()
        kp = torso_keypoints_at_distance(2.0)
        bbox = bbox_person_at_distance(8.0)
        person = {**bbox, "keypoints": kp}
        result = est.estimate("p1", person, now=0.0)
        self.assertEqual(result["cue"], "torso")
        self.assertAlmostEqual(result["camera_m"], 2.0, delta=0.05 * 2.0)

    def test_shoulder_used_when_no_hip_keypoints(self):
        est = DistanceEstimator()
        kp = shoulder_keypoints_at_distance(4.0)
        bbox = bbox_person_at_distance(9.0)
        person = {**bbox, "keypoints": kp}
        result = est.estimate("p1", person, now=0.0)
        self.assertEqual(result["cue"], "shoulder")
        self.assertAlmostEqual(result["camera_m"], 4.0, delta=0.05 * 4.0)

    def test_low_confidence_keypoints_fall_back_to_bbox(self):
        est = DistanceEstimator()
        kp = torso_keypoints_at_distance(2.0)
        for point in kp.values():
            point["conf"] = 0.1  # below MIN_KEYPOINT_CONF
        bbox = bbox_person_at_distance(5.0)
        person = {**bbox, "keypoints": kp}
        result = est.estimate("p1", person, now=0.0)
        self.assertEqual(result["cue"], "bbox_height")
        self.assertAlmostEqual(result["camera_m"], 5.0, delta=0.05 * 5.0)

    def test_cropped_bbox_without_keypoints_returns_none(self):
        est = DistanceEstimator()
        person = {
            "x1": 0.3, "x2": 0.7, "y1": 0.1, "y2": 0.99,  # cropped: y2 > 0.98
            "cx": 0.5, "cy": 0.6, "conf": 0.9,
        }
        result = est.estimate("p1", person, now=0.0)
        self.assertIsNone(result)

    def test_cropped_bbox_falls_back_to_keypoints_when_available(self):
        est = DistanceEstimator()
        kp = shoulder_keypoints_at_distance(3.0)
        person = {
            "x1": 0.3, "x2": 0.7, "y1": 0.1, "y2": 0.99,  # cropped
            "cx": 0.5, "cy": 0.6, "conf": 0.9, "keypoints": kp,
        }
        result = est.estimate("p1", person, now=0.0)
        self.assertEqual(result["cue"], "shoulder")
        self.assertAlmostEqual(result["camera_m"], 3.0, delta=0.05 * 3.0)


class LawOfCosinesTests(unittest.TestCase):
    def test_person_behind_robot_same_azimuth(self):
        # robot_cx default 0.35, robot_distance_m default 2.5.
        # Person on the exact same ray, 1 m farther away -> robot_m ~= 1 m.
        est = DistanceEstimator()
        person = bbox_person_at_distance(3.5, cx=0.35)
        result = est.estimate("p1", person, now=0.0)
        self.assertAlmostEqual(result["camera_m"], 3.5, delta=0.05 * 3.5)
        self.assertAlmostEqual(result["robot_m"], 1.0, delta=0.05)
        self.assertFalse(result["smoothed"])  # first sample for this track

    def test_person_same_distance_thirty_degrees_off_matches_chord(self):
        fov_h_deg = 70.0
        robot_cx = 0.35
        robot_distance_m = 2.5
        f_h = focal_h(fov_h_deg)
        theta_r = azimuth(robot_cx, f_h)
        theta_p = theta_r + math.radians(30.0)
        person_cx = 0.5 + f_h * math.tan(theta_p)

        est = DistanceEstimator(fov_h_deg=fov_h_deg, robot_cx=robot_cx, robot_distance_m=robot_distance_m)
        person = bbox_person_at_distance(robot_distance_m, fov_h_deg=fov_h_deg, cx=person_cx)
        result = est.estimate("p1", person, now=0.0)

        # Isosceles chord: for two rays of equal length r separated by
        # angle theta, the chord length is 2 * r * sin(theta / 2).
        expected_robot_m = 2.0 * robot_distance_m * math.sin(math.radians(30.0) / 2.0)
        self.assertAlmostEqual(result["robot_m"], expected_robot_m, delta=0.03)


class SmootherTests(unittest.TestCase):
    def test_stable_readings_pass_through_with_ema(self):
        sm = _Smoother(alpha=0.4)
        v, smoothed0 = sm.update(3.0, now=0.0)
        self.assertAlmostEqual(v, 3.0)
        self.assertFalse(smoothed0)
        v, smoothed1 = sm.update(3.1, now=0.1)
        self.assertTrue(smoothed1)
        self.assertAlmostEqual(v, 3.0 + 0.4 * (3.1 - 3.0), places=6)

    def test_large_outlier_is_rejected(self):
        sm = _Smoother(alpha=0.4)
        sm.update(3.0, now=0.0)
        sm.update(3.0, now=0.1)
        sm.update(3.0, now=0.2)
        stable = sm.value
        v, smoothed = sm.update(10.0, now=0.3)  # 7 m / 233% jump -> rejected
        self.assertTrue(smoothed)
        self.assertAlmostEqual(v, stable, places=6)
        self.assertAlmostEqual(sm.value, stable, places=6)

    def test_three_consecutive_similar_outliers_are_accepted(self):
        sm = _Smoother(alpha=0.4)
        sm.update(3.0, now=0.0)
        sm.update(3.0, now=0.1)

        v1, _ = sm.update(1.5, now=0.2)  # outlier #1: rejected
        self.assertAlmostEqual(v1, 3.0, places=6)

        v2, _ = sm.update(1.5, now=0.3)  # outlier #2: rejected
        self.assertAlmostEqual(v2, 3.0, places=6)

        v3, _ = sm.update(1.5, now=0.4)  # outlier #3: trusted, snap to raw
        self.assertAlmostEqual(v3, 1.5, places=6)

        # streak resets: the next sample now blends normally around 1.5.
        v4, _ = sm.update(1.5, now=0.5)
        self.assertAlmostEqual(v4, 1.5, places=6)

    def test_small_relative_jump_is_not_treated_as_outlier(self):
        # 50% relative jump but well under the 1.0 m absolute threshold.
        sm = _Smoother(alpha=0.4)
        sm.update(1.0, now=0.0)
        v, _ = sm.update(1.5, now=0.1)
        self.assertAlmostEqual(v, 1.0 + 0.4 * (1.5 - 1.0), places=6)


class LifecycleTests(unittest.TestCase):
    def test_forget_removes_track_state(self):
        est = DistanceEstimator()
        person = bbox_person_at_distance(3.0)
        est.estimate("p1", person, now=0.0)
        self.assertIn("p1", est._smoothers)
        est.forget("p1")
        self.assertNotIn("p1", est._smoothers)

    def test_forget_unknown_track_is_a_noop(self):
        est = DistanceEstimator()
        est.forget("does-not-exist")  # must not raise

    def test_forget_stale_removes_only_expired_tracks(self):
        est = DistanceEstimator()
        person = bbox_person_at_distance(3.0)
        est.estimate("old", person, now=0.0)
        est.estimate("fresh", person, now=8.0)

        est.forget_stale(now=10.0, ttl_s=5.0)

        self.assertNotIn("old", est._smoothers)   # 10 - 0 = 10 > 5
        self.assertIn("fresh", est._smoothers)    # 10 - 8 = 2 <= 5

    def test_forgotten_track_starts_fresh_unsmoothed(self):
        est = DistanceEstimator()
        person = bbox_person_at_distance(3.0)
        est.estimate("p1", person, now=0.0)
        est.forget("p1")
        result = est.estimate("p1", person, now=1.0)
        self.assertFalse(result["smoothed"])


if __name__ == "__main__":
    unittest.main()
