# Sentry Mode Phase 3 — permanent closed-loop pointing at the locked person

**Date:** 2026-07-22
**Status:** Implemented 2026-07-23 (operator decisions recorded below). Builds on
Phase 1 (`/api/sentry/detect`) and Phase 2 (head-lock buttons).

## Operator decisions (2026-07-22)

1. **Camera:** the **webcam** feed (same feed the lock buttons live on) drives
   the pointing. The webcam faces the robot from outside, so the yaw mapping
   is mirrored and needs live calibration (see Bring-up).
2. **Arming model:** pointing starts **automatically from the lock click**
   while Sentry Mode is on — no separate risk-ack dialog. The operator was
   explicitly warned this weakens the motion interlock and chose it; the
   lock-button click is treated as the operator's arming action, and the
   frontend sends the `armed`/`i_understand_risk` flags with the start
   request. **All other interlocks stay intact:** `TRACKING_ENABLED` kill
   switch, XR publisher suspension, lowstate+publisher guard, rate limiter,
   staleness → neutral pose, Release/Damp/Stand Up overrides, one session at
   a time.
3. **Permanence:** a sentry-started session has **no session time ceiling**
   (`permanent: true` skips `TRACKING_MAX_SESSION_S`). It ends only on
   unlock, Sentry toggle off, floating cam close, operator stop
   (`/api/track/stop`, Release), or repeated detection failures (existing
   abort logic).
4. **Controller:** the arm is driven through the **closed-loop PID +
   gravity feed-forward** corrector used by the High Level Controller's arm
   replay (`_closed_loop_arm_targets`), not raw position targets.

## Server (`server.py`)

- **New module-level payload parser** `parse_track_payload(payload) -> dict`
  (pure, unit-testable): validates and returns
  `{"camera": "head"|"webcam", "permanent": bool, "closed_loop": bool
  (default True), "target": {"cx","cy"} | None}`. Invalid camera or target →
  `ValueError`; `request_track_start` maps that to HTTP 400. Risk-ack and
  all existing 403/409/503 guards unchanged.
- **`request_track_start`** stores the parsed config on the store
  (`self.track_config`) before spawning the session thread; snapshot gains
  `camera`/`permanent` fields for the UI.
- **`_run_tracking`** upgrades:
  - Frame source per-session: `head` → `get_camera_frame()`, `webcam` →
    `webcam_frame` under `webcam_lock` (replaces the global
    `TRACKING_CAMERA` env branch; env stays as the default camera).
  - Webcam sessions build the mapper from new env-tunable calibration
    constants (defined next to the other `TRACKING_*` constants):
    `SENTRY_FOV_YAW` (default **-1.25** — negative mirrors yaw because the
    webcam faces the robot), `SENTRY_FOV_PITCH` (0.9), `SENTRY_YAW_OFFSET`
    (0.0), `SENTRY_PITCH_OFFSET` (0.35). Head sessions keep
    `PointingMapper()` defaults.
  - Target seeding: when the payload carries `target`, pre-set
    `state.target` to a synthetic person box at that cx/cy so the first
    association latches onto the clicked person, not the largest one.
  - Ceiling skip: `permanent` sessions bypass the `TRACKING_MAX_SESSION_S`
    check; everything else in the loop (stale → neutral template, failure
    abort, smoothing, rate limiting) is unchanged.
  - Closed-loop publish: when `closed_loop`, initialize the per-joint PID
    state exactly as `execute_arm_sdk_replay` does, then each tick run
    `_closed_loop_arm_targets(msg, current, pid_state, period)` and publish
    its corrected targets + feed-forward torques via
    `_build_arm_sdk_trajectory_cmd(msg, publish_targets, gains, ff,
    weight=1.0)`. Non-closed-loop falls back to today's direct command.

## Frontend (`static/app.js` sentry module)

- Lock click (while Sentry on) → `POST /api/track/start` with
  `{armed: true, i_understand_risk: true, source: "sentry-lock",
  camera: "webcam", permanent: true, closed_loop: true,
  target: {cx, cy}, target_id}` (center and persistent detector identity of
  the clicked track's smoothed box).
- Unlock → `POST /api/track/stop`. Switching person → stop, then start with
  the new seed. Toggle off / floating cam close / locked track aged out →
  stop (fire-and-forget; `clearAllTracks` is the chokepoint).
- Counter shows ` • POINTING` instead of ` • LOCKED` while the server
  confirms an active session (checked via `/api/track/status` every 2 s
  while locked); start failures surface in the counter tooltip and the lock
  stays visual-only.

## Bring-up / calibration (manual, operator present)

1. Sentry on, lock on a person, observe the right arm: it must track
   horizontally toward the person. If yaw moves the wrong way, flip the
   sign of `SENTRY_FOV_YAW`; then tune magnitude and
   `SENTRY_YAW_OFFSET`/`SENTRY_PITCH_OFFSET` until pointing lines up.
   Constants are set via `Environment=` in the robot's
   `robot-telemetry-web.service` (restart between tweaks).
2. Verify every stop path with the arm live: unlock, Sentry off, cam close,
   Release button, person leaving frame (stale → neutral within ~1.5 s).

## Testing

- Unit: `parse_track_payload` validation matrix; start still 403 without
  risk flags and 400 on bad camera/target; snapshot carries new fields.
- Contract: frontend needles (`/api/track/start`, `sentry-lock`,
  `POINTING`).
- Full suite + `make production-gate`; motion behavior itself is validated
  in the supervised bring-up, not in CI.
