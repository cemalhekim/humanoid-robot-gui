---
tags: [api, http, reference, routes, sse]
summary: Complete table of server.py HTTP routes (read, motion, recording, chat/voice, smartplug, MCP) with method and purpose.
---

# 04 - HTTP API Reference

All routes are served by `server.py` on `:8088`. Route strings below were
extracted directly from `server.py` (dispatch around lines 5487–5618). There is
**no authentication** except optional `MCP_TOKEN` on `/mcp` — see
[[03 - Safety Interlocks]].

## Read / GET endpoints

| Path | Type | Purpose |
| --- | --- | --- |
| `/` , `/index.html` | HTML | Main dashboard |
| `/welcome` , `/welcome.html` | HTML | Entry page (choose Wi-Fi or Ethernet) |
| `/api/state` | JSON | Normalized live robot snapshot |
| `/api/camera` | JSON | Camera worker + backend state |
| `/api/webcam` | JSON | Webcam source state |
| `/api/ros-graph` | JSON | ROS 2 graph snapshot (when available) |
| `/api/entrances` | JSON | Remote-entrance list (`remote-entrance.json`) |
| `/api/wrist/status` | JSON | Right-wrist control state |
| `/api/loco/status` | JSON | LocoClient / locomotion state |
| `/api/recording/status` | JSON | Telemetry recorder state |
| `/api/recording/files` | JSON | Available JSONL/pose recording files |
| `/api/recording/files/<name>` | JSONL | A single recording file for replay |
| `/api/diagrams` , `/api/diagrams/<name>` | JSON | `.drawio` docs for the diagram viewer |
| `/api/chat/status` | JSON | Chat assistant availability/config (see [[05 - Chat & MCP Tools]]) |
| `/api/spatial/pose` | JSON | Shared digital-twin spatial pose (hand coords) — see [[20 - LLM Arm Pose Proposals & Mimic]] |
| `/api/motion/active` | JSON | Is a replay/track running? (gates deploy restarts) |
| `/api/pose/feedback/data` | JSON | Pose-feedback rollup for the plot page |
| `/api/pose/feedback/image/<name>` | image | A collected reference image (basename-guarded) |
| `/feedback` , `/feedback.html` | HTML | Pose-feedback plot page (liked/disliked/executed) |
| `/events` | **SSE** | State stream, ~5 Hz (`_send_events` sleeps 0.2 s) |
| `/camera.mjpg` | MJPEG | Camera bridge stream |
| `/models/...` | Static | URDF, XML, STL model assets |
| `/vendor/...` | Static | Vendored Three.js modules |
| `/assets/...` | Static | Dashboard visual assets |

## Motion / command POST endpoints

| Path | Purpose | Guard |
| --- | --- | --- |
| `/api/wrist/command` | Right-wrist target or oscillation command | **`has_risk_ack`** (400 without) |
| `/api/wrist/stop` | Stop active wrist command | — |
| `/api/robot/chill` | Damp/chill ("Release") — arms limp | fails closed if LocoClient unavailable |
| `/api/robot/home` | Hold arms at current pose (arm_sdk; XR fallback) | XR suspend, arm_sdk |
| `/api/robot/straight` | Straight posture preset | XR suspend |
| `/api/loco/command` | H1 `LocoClient` command (see actions below) | validation, fails closed |
| `/api/xr/mode` | Request an XR teleop mode change | — |

`/api/loco/command` accepts `LOCO_ACTIONS`: `ready`, `balance_stand`,
`stand_up`, `start`, `stop_move`, `damp`, `zero_torque`, `high_stand`,
`low_stand`, `set_height`, `set_swing_height`, `set_balance_mode`, `velocity`,
`move`, `continuous_gait_on/off`, `next_foot_left/right`, `wave_hand`,
`shake_hand[_start/_end]`, `enable/disable/get_odom`, `set_target_position`, and
several `get_*` state queries. Bounds are in `LOCO_LIMITS` (e.g. `vx ∈ [-1,1]`,
`vy ∈ [-0.5,0.5]`, `vyaw ∈ [-1,1]`, `duration ∈ [0.1,10]`).

## Recording POST endpoints

| Path | Purpose |
| --- | --- |
| `/api/recording/start` | Start a JSONL telemetry recording (`{"label": ...}`) |
| `/api/recording/stop` | Stop the active recording |
| `/api/recording/pose` | Capture current full-body pose as one target point (`.pose.json`) |
| `/api/recording/sequence` | Sequence-mode recording control |
| `/api/recording/rename` | Rename a recording file |
| `/api/recording/replay/robot` | **Locked** robot playback — dry-run planning only; refuses physical playback (see [[03 - Safety Interlocks]]) |

`/api/recording/replay/robot` with `dry_run=true` reports the planned command
path (`arm_sdk` or `lowcmd`), trajectory validity, moving joints, velocity/delta
limits, and per-joint `kp`/`kd` plan (plus a `hand_plan` if fingers move). It
does **not** publish motor commands.

## Chat / voice / MCP

> [!note] Smart plug removed
> The `/api/smartplug/*` Home Assistant proxy was **removed** on 2026-07-23 and
> no longer exists on the current build — see [[23 - Smart Plug & Home Assistant]]
> for the historical record.

| Path | Method | Purpose |
| --- | --- | --- |
| `/api/chat` | POST | Command Center LLM chat + tool loop; optional `image` (attach an image to ask about it / mimic — routed to the vision bridge, see [[20 - LLM Arm Pose Proposals & Mimic]]; legacy `mimic_image` accepted) and `backend` (`default`/`claude`) |
| `/api/spatial/pose` | POST | Update the shared digital-twin spatial pose |
| `/api/pose/feedback` | POST | Record a pose verdict (`proposal_id`, `event`, `comment`); 👍 also executes the staged pose |
| `/api/pose/proposal/restage` | POST | Restage an earlier proposal as the green preview (candidates gallery; staging only) |
| `/api/stt` | POST | Speech-to-text proxy (off unless `LLM_STT_ENABLED`) |
| `/api/tts` | POST | Text-to-speech proxy (off unless `LLM_TTS_ENABLED`) |
| `/mcp` | POST | MCP endpoint (off unless `MCP_ENABLED`); `GET /mcp` → 405 |

## Person tracking (live, dark by default)

> [!note] Implemented 2026-07-21
> `POST /api/track/start` (risk-ack body → 403/409/503 gates), `POST /api/track/stop`
> (idempotent), and `GET /api/track/status` (`{"ok": true, "tracking": {...}}`) are
> now wired in `server.py`. They are gated by `TRACKING_ENABLED` (default `0`), so
> the feature ships dark until enabled on the robot. See
> [[06 - Person Tracking (CV Feature)#Implementation status]].

| Method | Route | Purpose |
| --- | --- | --- |
| POST | `/api/track/start` | Start the pointing session (needs `armed`+`i_understand_risk`) |
| POST | `/api/track/stop` | Stop / ramp to neutral (always available) |
| GET | `/api/track/status` | Phase, target bbox, detection age, failure count |

## Quick curl checks

```bash
curl -sS http://10.2.100.142:8088/api/state
curl -sS http://10.2.100.142:8088/api/loco/status
curl -sS http://10.2.100.142:8088/api/recording/status
curl -sS -X POST http://10.2.100.142:8088/api/recording/start \
  -H 'Content-Type: application/json' -d '{"label":"h1_2_full_body_hands"}'
```

## Related

[[01 - Architecture]] · [[03 - Safety Interlocks]] · [[05 - Chat & MCP Tools]] · [[06 - Person Tracking (CV Feature)]]
</content>
