---
tags: [mimic, computer-vision, keypoints, arm-control, safety]
summary: Mimic Mode — the robot mirrors the operator's upper-body pose (both arms) live from webcam pose keypoints. Master-switch toggle, keypoint→joint retargeting math, safety envelope, and what still needs hardware verification.
---

# 27 - Mimic Mode (Live Motion)

Added 2026-07-27. The topbar **Mimic Mode** button (`#mimicModeToggle`, blue
accent — the red one is [[19 - Sentry Mode & Head-Lock|Bullseye]]) makes the
robot mirror the person in front of the webcam with **both arms**, live.

Not to be confused with [[20 - LLM Arm Pose Proposals & Mimic|photo-pose mimic]],
which copies one static pose from an attached image via the LLM.

## How it works (chain)

1. **Toggle ON** → `POST /api/mimic/mode {on:true, armed:true,
   i_understand_risk:true}` — no browser confirm since 2026-07-28
   (operator request: "mimic never asks, just works"); the click is the
   deliberate action and the risk ack rides on the request. ON immediately
   starts the session (that is the point of the mode). OFF always stops
   and parks both arms at `home`.
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
5. **Retargeting** (`tracking.MimicMapper`, rewritten 2026-07-28 as full
   deterministic **single-view 3D lifting** — Taylor 2000): the deployed
   webcam is robot-relative (not mirrored), so the person's LEFT keypoints
   drive the robot's RIGHT arm and vice versa — the robot behaves like a
   mirror. Per frame:
   0. **Aspect de-distortion** (fix 2026-07-28 evening): detector
      keypoints are normalized 0..1 PER AXIS, so on a 16:9 frame vertical
      distances are 16/9× inflated vs horizontal — which broke every
      length ratio (arms are vertical, shoulder width horizontal: arms
      read as always-long → depth never fired → the robot sat frozen in
      one straight-arm pose, both arms alike). `sentry_detect` now passes
      the frame's real `w`/`h` through and the session sets
      `MimicMapper.aspect = h/w`; all geometry multiplies y by it.
   1. **Torso frame**: shoulder line = lateral axis, shoulder-mid→hip-mid =
      down axis (image-down fallback); shoulder width = the scale.
   2. **3D lift per bone**: lateral/down measured in that frame, forward =
      `sqrt(max(0, L_eff² − measured²))` with L_eff = 0.9 × expected length
      (upper arm 0.75 ×, forearm 0.65 × shoulder width) — projection can
      only *shrink* a bone, so shrinkage IS the out-of-plane angle. Depth
      sign is unobservable in one view: always read as **forward**. The
      0.9 factor is a ~26° dead zone absorbing keypoint length jitter.
   3. **Analytic inverse of the URDF chain** (h1_2.urdf, incl. the exact
      ±15° shoulder-mount tilt): upper-arm vector → shoulder **PITCH +
      ROLL**; forearm vector in the post-roll local frame → shoulder
      **YAW** (atan2 of its in-plane components) + **ELBOW q**, solved
      against the URDF's true forearm axis (15° inward / 5° down offsets
      compensated, plus a 0.10 rad constant roll bias from link offsets).
   ⚠️ **Elbow q semantics** (verified against h1_2.urdf + the live digital
   twin, which feeds motor q straight into URDF joints): **q=0 is a 90°
   bend forearm-FORWARD; q≈+1.5 is the straight arm; negative q curls
   deeper.** The pre-2026-07-28 mapper commanded the interior bend angle
   directly — 90° off; fixed by the rewrite.
   Degeneracy rules (all deterministic): no shoulder width → planar 2D
   fallback (roll+elbow only); vanishing bone WITH scale → depth dominates
   (limb points at the camera); forearm ≈ parallel to upper arm → yaw
   holds (gimbal); missing keypoints → arm holds, staleness machine parks.
   Validated by an **FK round-trip gold test** (`MimicRoundTripTests`:
   URDF FK → project → map → compare; worst error 0.39 rad, roll ≤ 0.01,
   from the deliberate safety dead zone). Changes < 0.04 rad are ignored
   (flicker dead band). `MIMIC_NEUTRAL_TEMPLATE` now equals the robot's
   real standby stance (read from rt/lowstate 2026-07-28: pitch 0.15,
   roll ±0.2, elbow 0.7); `MIMIC_LIMITS` elbow is (−0.9, 1.6) in q-space.

## Dance profile + self-collision guard (2026-07-28)

Built for the dance-demonstration case: smooth, fast enough to mirror a
dancer, and safe. All env-overridable (`deployment` drop-ins):

| Knob | Mimic default | Bullseye keeps | Why |
| --- | --- | --- | --- |
| `MIMIC_MAX_STEP_RAD_S` | **1.5** | 0.65 | a 90° raise completes in ~1 s instead of 2.4 s |
| `MIMIC_SMOOTH_ALPHA` | **0.5** | 0.35 | ~120 ms filter lag instead of ~300 ms |
| `MIMIC_RATE_HZ` | **12** | 8 | detect stream runs 15 Hz; the loop was the bottleneck |
| `MIMIC_HOLD_S` | **3.5** | 2.0 | a spin (back to camera) holds through instead of parking |

**What licenses the speed: the python self-collision guard.** Every
rate-limited step is checked by `tracking.mimic_pose_collides()` — exact
URDF FK (`tracking.arm_points`, torso frame) + sphere model (elbow/wrist/
hand per arm vs torso/pelvis/camera/lidar, plus arm-vs-arm; radii match
the 3D editor's JS model). A step that would enter contact **freezes at
the last clear pose** for that tick (steps are ≤ `max_step·dt`, so the arm
parks just outside the contact surface); if the robot ever *starts*
inside the model's zone, outward motion is allowed so it can escape. The
track-status message shows `collision hold: <pair>` while frozen.
Validated by `MimicCollisionTests`: 7 legitimate demo poses keep ≥2 cm
margin, 3 contact poses (hands-meet-midline, hand-into-chest,
curl-into-face) are all caught.

**Smoothness/accuracy (same day, `MimicMapper`):**
- **Linear depth ramp** (`depth_ramp` 0.08) replaces the hard dead-zone
  edge — the sqrt's infinite slope there made a hovering arm chatter;
  worst spatial gradient is now ~0.08 rad per 1 % of bone length.
- **Yaw-gate hysteresis** (`min_yaw_cos` 0.25 ± 0.05): yaw starts
  updating above 0.30 in-plane magnitude and holds below 0.20 — a slowly
  straightening arm can't flicker update/hold.
- **Per-person bone calibration**: rolling **max** of observed
  length/shoulder-width per bone, seeded at the anthropometric defaults
  (0.75 upper / 0.65 forearm), clamped (≤1.0 / ≤0.9). Projection only
  shrinks bones, so the max converges to the dancer's true proportions
  the moment they extend an arm frontally — **have the dancer hold a
  2 s T-pose right after Mimic ON**. Calibration only grows the expected
  length, which only *reduces* inferred depth: it can never invent
  motion. Depth edge raised 0.90 → **0.94** (about half the old
  mid-range under-response).

### Dance-demo day checklist

1. **One person in the camera's view** — `associate()` prefers the
   largest box; a bystander walking close can steal the target.
2. Dancer stands ~3–4 m back: **elbows + wrists must stay in frame**.
3. Robot in open floor, spotter at the **Release** button the whole time.
4. On AI-DEV verify the detect service is the real one:
   `ss -ltnp | grep 8188` PID == `ExecMainPID` (the rogue-:8188 gotcha
   has struck twice).
5. Prefer the wired network path; check `/api/track/status` `loop_hz`
   ≈ 12 during a warm-up run.
6. After Mimic ON: dancer T-poses ~2 s (calibration), then dances.
7. Spins are fine (3.5 s hold); leaving the frame parks the arms to
   standby — that's the safe default, not a bug.

## Camera-view overlay (what mimic sees)

While Mimic is ON, the floating **webcam** panel shows, per person (added
2026-07-27, same day as the mode):

- the detection **bounding box** with confidence label (green/red exactly as
  in Bullseye), even if the "boxes" toggle is off — mimic forces it on;
- a blue (#2f6fed, the mimic accent) **skeleton**: the shoulder–shoulder
  line plus both shoulder→elbow→wrist chains — precisely the segments the
  retargeting consumes, with dots on the six arm joints (wrists slightly
  bigger). A keypoint the detector drops leaves its bone undrawn — the
  overlay never invents a limb, so it is an honest debug view.

Plumbing: `setupMimic` publishes the server-confirmed state on
`document.body.dataset.mimicOn`; the Bullseye panel reads it to keep the
`/api/sentry/stream` SSE open while mimic runs (even with Bullseye off),
track keypoints per person (`track.kp`, lightly smoothed, stale points
dropped), and draw the skeleton in `renderBoxes`. Person-lock 🔓 buttons are
hidden in a mimic-only view (a lock would only error while Bullseye is off);
the counter chip reads `Mimic: N • MIRRORING`.

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

## Troubleshooting (from the 2026-07-27 bring-up)

First live attempt "did nothing"; two independent causes, both fixed:

1. **No elbow/wrist keypoints reaching the robot** — the AI-host service
   was serving OLD code from a **rogue process squatting on :8188** (boot-era
   PID) while the freshly restarted systemd unit crash-looped on bind — the
   exact known gotcha. Diagnosis:
   hips present but elbows absent (hips come AFTER elbows in COCO order →
   impossible with new code), `ss -ltnp | grep 8188` PID ≠ `ExecMainPID`.
   Fix: kill the rogue PID, restart `person-detect.service`, verify the
   listener PID matches. Without arm keypoints mimic starts but holds
   neutral — box visible, no skeleton arms, no motion.
2. **Mimic refused while a Bullseye pointing session ran** (409). Since
   2026-07-27, Mimic ON **stops a running pointing session and takes over**
   — a mode switch is one deliberate operator action (risk-acked + browser
   confirm). Bullseye's toggle stays armed; a new lock works after mimic
   turns off.

## Status / hardware TODO

✅ Pitch (foreshortening v1) live-verified 2026-07-28 13:10: standing
straight → 0.00 both arms; forward raise tracked smoothly to −0.5 rad;
clean return; occlusion → hold → stale → re-acquire all behaved.

⚠️ The full 3D rewrite (yaw + corrected elbow-q semantics, same day) is
verified against the URDF FK round-trip + 30 unit tests, **not yet on real
arms**. First-run live checks: arms hanging → robot arms hang straight
(elbow ~1.5, NOT bent); elbow bent 90° forearm forward (wrist hidden
behind elbow on camera) → robot matches with forearm forward; forearm
sweep right↔front↔left with bent elbow → yaw follows, capped ±0.8; arm
straight out sideways → no yaw jitter (gimbal hold). Tunables in
`MimicMapper.__init__`: `depth_start_ratio` (0.9 dead zone),
`max_pitch` (1.2), `min_yaw_cos` (0.25 gimbal guard), `fore_ratio`
(0.65 forearm anthropometry). First live run:
spotter present, robot in open space, be ready on the toggle. Known
limitation to tune on hardware: with shoulder yaw pinned at 0, elbow flexion
moves the forearm in the robot's natural (sagittal-ish) plane, not
necessarily the human's frontal plane — a yaw follow-up is the planned fix
if it looks wrong live.

## Related

[[19 - Sentry Mode & Head-Lock]] · [[07 - Detection Service (YOLO)]] ·
[[20 - LLM Arm Pose Proposals & Mimic]] · [[03 - Safety Interlocks]] ·
[[24 - Control Gains, PID & Shared Mechanisms]]
