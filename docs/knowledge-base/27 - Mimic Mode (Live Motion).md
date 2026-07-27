---
tags: [mimic, computer-vision, keypoints, arm-control, safety]
summary: Mimic Mode — the robot mirrors the operator's upper-body pose (both arms) live from webcam pose keypoints. Master-switch toggle, keypoint→joint retargeting math, safety envelope, and what still needs hardware verification.
---

# 27 - Mimic Mode (Live Motion)

Added 2026-07-27. The topbar **Mimic Mode** button (`#mimicModeToggle`, blue
accent — the red one is [[19 - Sentry Mode & Head-Lock|Bullseye]]) makes the
robot mirror the person in front of the webcam with **both arms**, live.

Not to be confused with [[20 - LLM Arm Pose Proposals & Mimic|photo-pose
mimic]], which copies one static pose from an attached image via the LLM.

## How it works (chain)

1. **Toggle ON** → `POST /api/mimic/mode {on:true, armed:true,
   i_understand_risk:true}` after a browser confirm. ON immediately starts
   the session (that is the point of the mode), so the risk ack rides on the
   toggle itself. OFF always stops and parks both arms at `home`.
2. The endpoint arms `mimic_mode_on` and calls `request_track_start` with
   `{mode:"mimic", camera:"webcam", permanent:true, closed_loop:true,
   source:"mimic-toggle"}`. If the session is refused (DDS missing, another
   session running, `TRACKING_ENABLED=0`) the switch **disarms itself** —
   it never stays on with nothing running.
3. The session is a regular `_run_tracking` loop sharing ALL of Bullseye's
   plumbing: the shared sentry detect stream (`SENTRY_STREAM_HZ`), the
   `TrackState` staleness machine (lost person → hold 2 s → park), the EMA
   smoothing (`SENTRY_SMOOTH_ALPHA`), the `RateLimiter`
   (`SENTRY_MAX_STEP_RAD_S`), the closed-loop PID + gravity feed-forward
   corrector (`SENTRY_REPLAY_RESPONSE` tuning), and the arm_sdk publish with
   `weight=1.0`.
4. **Detection**: the AI-host service ([[07 - Detection Service (YOLO)]],
   `yolo11m-pose`) now also returns `l_elbow`, `r_elbow`, `l_wrist`,
   `r_wrist` keypoints (COCO 7–10, conf ≥ 0.3) — **the AI-host copy must be
   redeployed** (copy `deployment/ai_host/detect_service.py` and restart
   `person-detect.service`) before mimic sees arms.
5. **Retargeting** (`tracking.MimicMapper`, pure/unit-tested): the deployed
   webcam is robot-relative (not mirrored), so the person's LEFT keypoints
   drive the robot's RIGHT arm and vice versa — the robot behaves like a
   mirror. Frontal-plane math per arm:
   - upper-arm **elevation** (angle of shoulder→elbow from hanging-down;
     positive = outward) → shoulder **ROLL** (abduction), sign per arm
     (negative abducts the right arm, positive the left);
   - shoulder→elbow→wrist interior **bend** angle → **ELBOW** joint;
   - shoulder pitch/yaw + wrists stay neutral (forward raise is invisible
     to a frontal 2D view).
   Segments < 1.5 % of the image (foreshortened, pointing at the camera) and
   changes < 0.04 rad (keypoint flicker) are ignored; an arm with missing
   keypoints **holds** its last targets while the staleness machine decides.

## Safety envelope

- Own master switch (`mimic_mode_on`), default OFF every boot; `POST
  /api/track/start {mode:"mimic"}` without it → 409, exactly like Bullseye
  locks without Bullseye.
- Bullseye and Mimic sessions are mutually exclusive (single
  `track_thread`); either master switch going OFF stops whatever runs.
- `MIMIC_LIMITS` in `tracking.py` is a conservative both-arm envelope
  (roll ±2.6, elbow 0–2.3, pitch/yaw/wrists pinned near neutral); server
  `JOINT_LIMITS` re-clamps after it, and the rate limiter bounds every
  published step, so detector jumps cannot become arm jumps.
- `MIMIC_NEUTRAL_TEMPLATE` (both arms relaxed) is the park/stale pose.

## Status / hardware TODO

⚠️ Signs and planes are verified against joint-limit tables and unit tests
(`tests/test_mimic.py`, 21 tests), **not yet on real arms**. First live run:
spotter present, robot in open space, be ready on the toggle. Known
limitation to tune on hardware: with shoulder yaw pinned at 0, elbow flexion
moves the forearm in the robot's natural (sagittal-ish) plane, not
necessarily the human's frontal plane — a yaw follow-up is the planned fix
if it looks wrong live.

## Related

[[19 - Sentry Mode & Head-Lock]] · [[07 - Detection Service (YOLO)]] ·
[[20 - LLM Arm Pose Proposals & Mimic]] · [[03 - Safety Interlocks]] ·
[[24 - Control Gains, PID & Shared Mechanisms]]
