---
tags: [overview, h1-2, dashboard, telemetry]
summary: What the humanoid-robot-gui system is, the H1-2 robot it drives, and the tech stack.
---

# 00 - Project Overview

## What this is

`humanoid-robot-gui` is a **local web dashboard + telemetry server + guarded
robot command surface + Vision Pro / XR service deployment** for the
**Unitree H1-2** humanoid robot. See [[01 - Architecture]] for how the pieces fit.

The project is designed to:

- Read Unitree **DDS** telemetry from the robot PC (see [[09 - Glossary#DDS]]).
- Show H1-2 robot state — joints, IMU, battery, hands, camera, command history — in a browser dashboard.
- Expose **guarded** right-wrist and H1 locomotion commands with explicit operator intent and runtime limits (see [[03 - Safety Interlocks]]).
- Run Vision Pro / XR teleoperation services on the robot PC as user-level systemd services (see [[02 - Network & Hosts]]).
- Let a development laptop test the UI and static assets **without** live robot telemetry.

> [!warning] Not a read-only application
> Some HTTP endpoints can move the robot. Run it only on trusted networks and
> only with an operator who can physically observe the robot and understands
> the risk. There is **no authentication** — anyone who can reach the dashboard
> host can send HTTP requests to exposed endpoints. See [[03 - Safety Interlocks]]
> and [[04 - HTTP API Reference]].

## The Unitree H1-2 robot

The H1-2 is a full-size humanoid. The dashboard models it as **27 known body
motors** (indices 0–26) plus reserved slots, driven/observed over DDS.

Body joint order (`rt/lowstate`), abbreviated:

| Range | Joints |
| --- | --- |
| 0–5 | Left leg: HipYaw, HipPitch, HipRoll, Knee, AnklePitch, AnkleRoll |
| 6–11 | Right leg: same order |
| 12 | `WaistYaw` — torso twist (lowcmd-only; **not** on arm_sdk) |
| 13–19 | Left arm: ShoulderPitch, ShoulderRoll, ShoulderYaw, Elbow, WristRoll, WristPitch, WristYaw |
| 20–26 | Right arm: same order |

The **RH56BFX / Inspire hands** are a separate 12-joint system (6 per hand)
reported on `rt/inspire/state`. Body telemetry and hand telemetry are
independent — the body can be connected while the hand panel is disconnected.

See the full joint tables in [[09 - Glossary]] and the arm subset used for
motion in [[03 - Safety Interlocks#arm_sdk joint set]].

## The dashboard's purpose

The browser dashboard (`static/index.html` + `app.js` + `viewer.js`) shows:

- Live body telemetry from `rt/lowstate` (position `q`, velocity `dq`, estimated torque `tau_est`, temperature, voltage, mode per motor).
- IMU (quaternion, gyro, accel, roll/pitch/yaw, temperature).
- Robot metadata (`version`, `mode_pr`, `mode_machine`, `tick`, `crc`, `wireless_remote`).
- Battery/BMS values (when firmware exposes them), foot forces, hand telemetry.
- A live **Three.js H1-2 URDF viewer** (`static/models/h1_2_description/h1_2.urdf`) with view-cube presets, grid, and auto-rotate.
- A **TeleImager camera** panel and links to the XR / Vuer page.
- **Command surfaces**: Release (damp), Home (arm_sdk hold), Stand Up, Loco control, wrist commands.
- A **telemetry recorder** (sequence JSONL + pose-point) with a 3-model replay digital twin.
- An on-prem **LLM chat** ("Command Center assistant") with guarded tools — see [[05 - Chat & MCP Tools]].
- A **Smart Plug** card that toggles a showcase Sonoff plug via Home Assistant.

## Command surfaces (dashboard buttons)

| Button | Behavior |
| --- | --- |
| `Release` (red, formerly "Chill Motors") | Requests damp behavior — arms go limp (`/api/robot/chill`). |
| `Home` (green) | Engages an arm_sdk hold at the arms' measured position at press time; closed-loop PID + gravity feed-forward corrector (disable with `HOME_HOLD_CLOSED_LOOP=0`). Falls back to legacy XR home when DDS unavailable. |
| `Stand Up` (formerly "Straight") | LocoClient `StandUp()` lock-stand — recovery after Release. |
| `Loco Control` | ready, stand, start, damp, zero torque, velocity, target position, odometry. |
| `Camera` | TeleImager WebRTC preview + XR entry points. |

> [!note] Right Wrist page archived
> The dedicated Right Wrist page has been archived, but its guarded backend API
> (`/api/wrist/*`) is retained for tool/integration compatibility.

## Tech stack

| Layer | Technology |
| --- | --- |
| Backend | Python 3.10+ **stdlib** HTTP server (`http.server`), single file `server.py` |
| Robot comms | Unitree SDK2 Python + CycloneDDS (`unitree_sdk2py`) |
| Frontend | Plain HTML/CSS/JS — **no build step, no npm**, vendored Three.js |
| 3D viewer | Three.js + STLLoader + OrbitControls, H1-2 URDF |
| On-prem LLM | Ollama (`qwen3:30b-a3b-instruct-2507-q4_K_M`) via OpenAI-compatible API |
| CV feature | YOLOv8n (ultralytics) detection microservice on the AI host GPU |
| Deployment | user-level **systemd** services + a 60 s auto-update timer |
| Tests | Python `unittest` (offline) + `production_gate.py` |

The frontend intentionally has **no build system**: plain HTML/CSS/browser JS
with vendored Three.js, so deployment on robot PCs needs no extra tooling.

## Related

[[01 - Architecture]] · [[03 - Safety Interlocks]] · [[06 - Person Tracking (CV Feature)]] · [[09 - Glossary]]
</content>
