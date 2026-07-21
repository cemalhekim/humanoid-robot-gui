# Person tracking / pointing — design

Date: 2026-07-21
Status: draft, pending operator review

## Goal

The H1-2 continuously points at the operator with its right arm: a computer
vision loop detects the person in the head-camera image and steers the right
shoulder so the extended arm follows them, smoothly and inside all existing
safety interlocks. This is a showcase feature: it must look alive (smooth,
responsive) and fail safe (arm returns to a neutral pose whenever tracking
degrades).

Out of scope for v1: finger/hand gestures (Inspire hand pointing pose),
waist rotation (joint 12 is lowcmd-only), locomotion, multi-person choreography.

## Architecture

Three pieces, two of which already exist in some form:

```
head camera (TeleImager ZMQ)          AI host 10.2.125.3
        │ JPEG frames                 ┌──────────────────────────┐
        ▼                             │ detect_service.py :8188  │
robot PC server.py ── POST /detect ──▶│ YOLOv8n on A40 GPU       │
  tracking loop   ◀── person bboxes ──│ ~7 ms/frame              │
        │ joint targets (retarget)    └──────────────────────────┘
        ▼
persistent arm_sdk pose controller (120 Hz, velocity-bounded, PID+gravity)
```

1. **Detection service (exists, prototyped)** — `~/person-tracking/detect_service.py`
   on 10.2.125.3, port 8188. POST a JPEG to `/detect`, get normalized person
   boxes `{x1,y1,x2,y2,cx,cy,conf}` sorted by area, plus inference ms.
   Runs YOLOv8n (person class only, conf ≥ 0.4) on the A40 GPU. Needs a
   systemd unit for production. Verified end-to-end 2026-07-21.

2. **Tracking loop (new)** — a daemon thread in `server.py` on the robot PC,
   following the retargetable-controller pattern of `execute_lowcmd_pose`
   (server.py:4479-4604). At ~8-10 Hz it:
   - reads the freshest head-camera JPEG from `store.get_camera_frame()`
     (fallback: webcam buffer via `TRACKING_CAMERA=webcam`),
   - POSTs it to the detection service (`TRACKING_DETECT_URL`, timeout 0.5 s),
   - selects the target person (see policy below),
   - maps image coordinates to right-arm joint targets (see mapping below),
   - smooths (EMA, alpha ≈ 0.35) and swaps targets into the arm controller
     under `command_lock`, exactly like `_pose_targets` retargeting.
   The existing 120 Hz controller does interpolation, velocity bounding,
   PID and gravity feed-forward — the tracking loop only ever moves the
   *setpoint*.

3. **Control surface (new)** — HTTP + UI + chat/MCP:
   - `POST /api/track/start` — guarded by `has_risk_ack` (`armed` +
     `i_understand_risk`), goes through `_suspend_xr_motion_publishers`,
     refuses to start if a replay/wrist/home session is active (same
     mutual-exclusion cancel-Event scheme).
   - `POST /api/track/stop` — always available, ramps to neutral then releases.
   - `GET /api/track/status` — state, target bbox, detection age, fps,
     consecutive-failure count.
   - Dashboard card with Start/Stop, live status, and the risk-ack checkboxes
     (mirror the wrist card).
   - Chat/MCP tool `track_person` (`action: start|stop`, `confirm: true`
     required) — spec appended in `chat_tool_specs` behind
     `LLM_TOOL_TRACK_ENABLED`, dispatched in `run_chat_tool`, MCP exposure is
     automatic.

## Image → joint mapping (no IK)

Pointing is a 2-DOF problem; full IK is unnecessary. The right arm is held in
a fixed "pointing template" (elbow ~15° short of straight, wrist neutral) and
only the shoulder aims it:

- horizontal: person `cx` ∈ [0,1] → right shoulder yaw (joint 22), linear map
  across the camera's horizontal FOV, clamped well inside `JOINT_LIMITS[22]`.
- vertical: person `cy` (use bbox upper-third midpoint ≈ chest height, not
  box center) → shoulder pitch (joint 20), same linear treatment.

Calibration constants (FOV span → radian span, plus per-axis offsets) live in
env vars with conservative defaults and get tuned live on the robot. Dead-band
of ±0.03 normalized units so the arm doesn't jitter when the person stands
still. All targets pass through `_clamp_joint_target`; setpoint changes are
additionally rate-limited to ≤ 0.35 rad/s at the tracking layer, below the
controller's own caps.

## Target selection policy

- Lock onto the **largest** person box (nearest visitor) on start.
- While locked, prefer the detection whose center is nearest the previous
  target (IoU/center-distance association) — so the robot doesn't jump between
  people when a second visitor walks through.
- If the locked target vanishes: hold pose for 2 s, then re-acquire largest;
  if nobody is visible, go to stale-fallback (below).

## Failure handling (Wi-Fi is known to be flaky)

Detection staleness is the central hazard: the robot must never keep aiming
with dead data.

- Every loop tick checks detection age. Age > 1.5 s → ramp arm to the neutral
  template pose (velocity-bounded), state becomes `stale`.
- 10 consecutive detect-service failures → auto-stop the session entirely
  (state `aborted`, reason surfaced in `/api/track/status` and the UI card).
- Hard session ceiling `TRACKING_MAX_SESSION_S` (default 600 s) → auto-stop,
  matching the spirit of the replay 90 s ceiling.
- `POST /api/track/stop`, chat `track_person stop`, and any start of a
  replay/wrist/home session cancel tracking first (shared cancel-Event
  convention).
- DDS lowstate older than 0.5 s → treat as stale (no blind publishing).

## Configuration (env, robot service)

| Var | Default | Meaning |
| --- | --- | --- |
| `TRACKING_ENABLED` | `0` | feature flag for endpoints + UI card |
| `TRACKING_DETECT_URL` | `http://10.2.125.3:8188/detect` | detection service |
| `TRACKING_CAMERA` | `head` | `head` or `webcam` frame source |
| `TRACKING_RATE_HZ` | `8` | detection loop rate |
| `TRACKING_MAX_SESSION_S` | `600` | hard session ceiling |
| `LLM_TOOL_TRACK_ENABLED` | `0` | expose chat/MCP tool |

## Testing

- Unit tests (offline, no DDS — same style as `tests/test_chat.py`):
  mapping math (cx/cy → clamped joint targets, dead-band, rate limit),
  target-association policy, staleness state machine, endpoint gating
  (risk-ack required, 409 when replay active), tool spec/dispatch.
- Detection service: contract test with a canned image (already proven live).
- `make production-gate` before every push (CLAUDE.md requirement).
- On-robot bring-up (robot is currently powered off; ordered checklist):
  1. verify robot→AI-host reachability + measure real frame round-trip,
  2. dry-run: tracking loop with arm publishing disabled, status only,
  3. arms enabled at reduced rate limits, operator physically present,
  4. tune FOV calibration constants, then normal limits.

## Risks

- **Wi-Fi stalls mid-session** → covered by staleness ramp-to-neutral; worst
  case the arm holds neutral, never a stale aim.
- **Fighting controllers** → reuse `_suspend_xr_motion_publishers` + the
  cancel-Event mutual exclusion; tracking is just another guarded session.
- **Camera FOV/mounting unknowns** → calibration constants are env-tunable;
  bring-up step 4 exists precisely for this.
- **Robot PC CPU cost** → loop only JPEG-forwards frames (no local inference);
  at 8 Hz and ~50 KB frames this is negligible.
