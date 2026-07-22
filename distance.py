"""Real-time person-to-robot distance estimation from a fixed monocular camera.

Pure math, stdlib only (no numpy), no I/O — mirrors the style of tracking.py:
server.py owns frames/HTTP/DDS, this module owns the geometry and filtering
decisions so it can be unit-tested offline.

-------------------------------------------------------------------------
THE MODEL, STEP BY STEP
-------------------------------------------------------------------------

1) Pinhole camera / focal length in normalized units
   ---------------------------------------------------
   All detections are normalized to the image (0..1 in x, 0..1 in y), so we
   derive a focal length in the same normalized units rather than pixels.

   Treat the sensor width as 1.0 "unit". For a horizontal field of view
   fov_h, basic pinhole trigonometry gives:

       tan(fov_h / 2) = (half sensor width) / focal_length = 0.5 / f_h

   so

       f_h = 0.5 / tan(fov_h / 2)                                   (units: sensor-widths)

   This f_h is the one focal length we use for every size cue. A vertical
   (y) pixel delta is only comparable to a horizontal (x) pixel delta once
   it is re-expressed in "sensor width" units. Since aspect = width/height,
   a normalized y-delta dy (0..1 of image height) corresponds to a physical
   sensor extent of dy / aspect sensor-widths (a taller aspect number means
   the sensor is relatively wider than it is tall, so the same normalized
   dy covers less width-equivalent distance). We therefore convert every
   apparent size to this common "width-normalized" frame before dividing by
   f_h — see `_apparent_size()`.

   Standard pinhole distance-from-size relation, once size cue and focal
   length share units:

       distance_m = real_size_m * f_h / apparent_size_normalized

2) Size cues, in priority order (most to least reliable)
   -------------------------------------------------------
   a) Torso length (mid-shoulder -> mid-hip), real ~0.52 m. Most stable:
      unaffected by arm pose, works whether the person faces the camera or
      is in profile-ish, and is rarely occluded by frame edges.
   b) Shoulder width (l_shoulder -> r_shoulder), real ~0.41 m. Good, but
      degrades as the person turns away from the camera (foreshortening).
   c) Full bbox height, real ~1.70 m (adult standing height). Least
      reliable: breaks down for anything but a fully upright, fully framed
      person, and is meaningless once the detector's box is clipped by the
      top or bottom of the frame — a cropped person is not full height, so
      that cue is skipped whenever y1 < 0.02 or y2 > 0.98.
   Keypoints (torso/shoulder cues) additionally require conf >= 0.3 for
   every keypoint involved; otherwise that cue is skipped.
   If no cue is usable (e.g. a bbox cropped top/bottom, no keypoints, or
   low-confidence keypoints) `estimate()` returns None for that frame.

3) Robot-relative distance via the law of cosines
   -------------------------------------------------
   The camera is the apex of a triangle. One side is the ray to the person
   (length = camera_m from step 1/2, direction = the person's azimuth
   angle theta_p). The other side is the ray to the robot (length =
   robot_distance_m, a fixed calibrated constant; direction = the robot's
   azimuth theta_r, also fixed/calibrated). Both azimuths are derived from
   normalized x position the same way, using the same pinhole geometry:

       theta = atan2(x_norm - 0.5, f_h)

   (x_norm - 0.5 is the horizontal offset from the principal point/optical
   axis, in the same "sensor width" units as f_h.)

   The person-to-robot distance is then the third side of the triangle:

       robot_m = sqrt(camera_m^2 + robot_distance_m^2
                       - 2 * camera_m * robot_distance_m * cos(theta_p - theta_r))

   Assumptions (must hold or the estimate degrades):
     - The camera, the robot's base, and people's feet are all
       approximately on the same floor plane; the model is 2D (top-down),
       so all vertical offsets (camera height, person height differences)
       are ignored. This is fine for a room-scale wide shot where the
       vertical parallax error is small compared to horizontal spacing.
     - The robot does not move (fixed azimuth + fixed distance, both
       calibrated once at setup time).
     - People are standing/upright enough that their apparent size cue
       reflects the priority-list real-world sizes above.

4) Temporal smoothing per track id
   ----------------------------------
   Raw per-frame robot_m is noisy (detector jitter, keypoint jitter). We
   keep one exponential-moving-average (EMA) smoother per track id:

       smoothed_new = smoothed_old + alpha * (raw - smoothed_old)

   with outlier rejection: a raw sample is treated as a probable bad frame
   (and NOT blended in) if it differs from the current smoothed value by
   more than both 40% (relative) AND 1.0 m (absolute) -- both conditions
   must hold, so e.g. a 50% jump that is only 0.3 m is accepted normally.
   To avoid permanently ignoring a real fast approach (person jogging
   toward the robot), a small consecutive-outlier counter tracks repeated
   "outlier-looking" samples; the 3rd consecutive one is trusted outright
   (snapped to) instead of rejected, and the counter resets.

-------------------------------------------------------------------------
"""
from __future__ import annotations

import math
from typing import Any

# Real-world size priors (meters). Averaged adult proportions.
REAL_TORSO_M = 0.52          # mid-shoulder to mid-hip
REAL_SHOULDER_WIDTH_M = 0.41  # l_shoulder to r_shoulder
REAL_HEIGHT_M = 1.70          # full standing height (bbox top to bottom)

# A keypoint below this confidence is treated as missing.
MIN_KEYPOINT_CONF = 0.3

# A bbox this close to the top/bottom edge is probably cropped, so its
# height no longer reflects full standing height.
CROP_EDGE_MARGIN = 0.02

# Outlier-rejection thresholds for the per-track smoother.
OUTLIER_REL_THRESHOLD = 0.40
OUTLIER_ABS_THRESHOLD_M = 1.0
OUTLIER_STREAK_TO_ACCEPT = 3


def _keypoint_ok(kp: dict[str, Any] | None) -> bool:
    return kp is not None and kp.get("conf", 0.0) >= MIN_KEYPOINT_CONF


class _Smoother:
    """EMA + outlier rejection for one scalar (one track's robot_m).

    See module docstring section 4 for the algorithm. Kept as a small
    standalone class (like tracking.Smoother) so it is independently
    testable.
    """

    def __init__(self, alpha: float = 0.4) -> None:
        self.alpha = alpha
        self.value: float | None = None
        self.last_update: float | None = None
        self._outlier_streak = 0

    def update(self, raw: float, now: float) -> tuple[float, bool]:
        """Feed one raw sample. Returns (value_to_report, was_smoothed)."""
        self.last_update = now

        if self.value is None:
            # First sample for this track: nothing to blend against yet.
            self.value = raw
            self._outlier_streak = 0
            return self.value, False

        diff = abs(raw - self.value)
        rel = diff / self.value if self.value > 1e-9 else float("inf")
        is_outlier = diff > OUTLIER_ABS_THRESHOLD_M and rel > OUTLIER_REL_THRESHOLD

        if is_outlier:
            self._outlier_streak += 1
            if self._outlier_streak < OUTLIER_STREAK_TO_ACCEPT:
                # Probable bad frame: ignore it, keep reporting the last
                # smoothed value.
                return self.value, True
            # 3rd consecutive similar "outlier": trust it as real fast
            # motion rather than noise. Snap to it and reset the streak.
            self._outlier_streak = 0
            self.value = raw
            return self.value, True

        self._outlier_streak = 0
        self.value = self.value + self.alpha * (raw - self.value)
        return self.value, True


class DistanceEstimator:
    """Monocular distance-from-camera + robot-relative distance estimator."""

    def __init__(
        self,
        fov_h_deg: float = 70.0,
        aspect: float = 4 / 3,
        robot_cx: float = 0.35,
        robot_distance_m: float = 2.5,
        alpha: float = 0.4,
    ) -> None:
        self.fov_h_deg = fov_h_deg
        self.aspect = aspect
        self.robot_cx = robot_cx
        self.robot_distance_m = robot_distance_m
        self.alpha = alpha

        # Focal length in "sensor width" normalized units. See module
        # docstring section 1.
        self._f_h = 0.5 / math.tan(math.radians(fov_h_deg) / 2.0)

        self._smoothers: dict[Any, _Smoother] = {}

    # ---- camera model helpers -------------------------------------------------

    def _apparent_size(self, dx: float, dy: float) -> float:
        """Euclidean size of a (dx, dy) normalized delta, re-expressed in
        common "sensor width" units (dy is scaled by 1/aspect first)."""
        return math.hypot(dx, dy / self.aspect)

    def _azimuth(self, x_norm: float) -> float:
        """Ray angle (radians) for a normalized x position, relative to the
        camera's optical axis. See module docstring section 3."""
        return math.atan2(x_norm - 0.5, self._f_h)

    # ---- size cues (priority order: torso > shoulder > bbox height) -----------

    def _torso_cue(self, keypoints: dict[str, Any]) -> tuple[float, str] | None:
        ls = keypoints.get("l_shoulder")
        rs = keypoints.get("r_shoulder")
        lh = keypoints.get("l_hip")
        rh = keypoints.get("r_hip")
        if not all(_keypoint_ok(p) for p in (ls, rs, lh, rh)):
            return None
        mid_sh_x = (ls["x"] + rs["x"]) / 2.0
        mid_sh_y = (ls["y"] + rs["y"]) / 2.0
        mid_hip_x = (lh["x"] + rh["x"]) / 2.0
        mid_hip_y = (lh["y"] + rh["y"]) / 2.0
        size = self._apparent_size(mid_sh_x - mid_hip_x, mid_sh_y - mid_hip_y)
        if size <= 0:
            return None
        return REAL_TORSO_M * self._f_h / size, "torso"

    def _shoulder_cue(self, keypoints: dict[str, Any]) -> tuple[float, str] | None:
        ls = keypoints.get("l_shoulder")
        rs = keypoints.get("r_shoulder")
        if not all(_keypoint_ok(p) for p in (ls, rs)):
            return None
        size = self._apparent_size(ls["x"] - rs["x"], ls["y"] - rs["y"])
        if size <= 0:
            return None
        return REAL_SHOULDER_WIDTH_M * self._f_h / size, "shoulder"

    def _bbox_cue(self, person: dict[str, Any]) -> tuple[float, str] | None:
        y1, y2 = person["y1"], person["y2"]
        if y1 < CROP_EDGE_MARGIN or y2 > (1.0 - CROP_EDGE_MARGIN):
            # Cropped by top/bottom of frame: height no longer represents
            # full standing height, so this cue is unusable.
            return None
        dy = y2 - y1
        if dy <= 0:
            return None
        size = self._apparent_size(0.0, dy)
        return REAL_HEIGHT_M * self._f_h / size, "bbox_height"

    def _camera_distance(self, person: dict[str, Any]) -> tuple[float, str] | None:
        keypoints = person.get("keypoints") or {}
        for cue_fn in (self._torso_cue, self._shoulder_cue):
            result = cue_fn(keypoints)
            if result is not None:
                return result
        return self._bbox_cue(person)

    # ---- robot-relative distance -----------------------------------------------

    def _robot_distance(self, camera_m: float, person_cx: float) -> float:
        theta_p = self._azimuth(person_cx)
        theta_r = self._azimuth(self.robot_cx)
        angle = theta_p - theta_r
        d2 = (
            camera_m * camera_m
            + self.robot_distance_m * self.robot_distance_m
            - 2.0 * camera_m * self.robot_distance_m * math.cos(angle)
        )
        return math.sqrt(max(d2, 0.0))

    # ---- public API ---------------------------------------------------------

    def estimate(self, track_id: Any, person: dict[str, Any], now: float) -> dict[str, Any] | None:
        """person: {x1,y1,x2,y2,cx,cy,conf, keypoints?: {nose,l_shoulder,
        r_shoulder,l_hip,r_hip}}. Returns {"camera_m", "robot_m", "cue",
        "smoothed"} or None when no usable size cue is available."""
        cue_result = self._camera_distance(person)
        if cue_result is None:
            return None
        camera_m, cue_name = cue_result

        raw_robot_m = self._robot_distance(camera_m, person["cx"])
        smoother = self._smoothers.setdefault(track_id, _Smoother(self.alpha))
        robot_m, smoothed = smoother.update(raw_robot_m, now)

        return {
            "camera_m": camera_m,
            "robot_m": robot_m,
            "cue": cue_name,
            "smoothed": smoothed,
        }

    def forget(self, track_id: Any) -> None:
        self._smoothers.pop(track_id, None)

    def forget_stale(self, now: float, ttl_s: float = 5.0) -> None:
        stale = [
            tid
            for tid, smoother in self._smoothers.items()
            if smoother.last_update is not None and now - smoother.last_update > ttl_s
        ]
        for tid in stale:
            del self._smoothers[tid]
