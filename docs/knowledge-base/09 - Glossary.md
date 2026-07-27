---
tags: [glossary, reference, terminology]
summary: Definitions of the project's core terms — H1-2, DDS, arm_sdk, lowcmd/lowstate, retargeting, TeleImager, XR, IK, EMA, FOV, bbox, and more.
---

# 09 - Glossary

## Robot & platform

- **H1-2** — Unitree's full-size humanoid robot driven by this project. Modeled
  as 27 body motors (indices 0–26) plus reserved slots, plus the RH56BFX hands.
  See 00 - Project Overview.
- **RH56BFX / Inspire hands** — the robot's dexterous hands. 12 joints (6 per
  hand) on `rt/inspire/state`; bridged by `inspire-hands.service`.
- **LocoClient** — Unitree's high-level locomotion client. Preferred layer for
  walking (`/api/loco/command` → `LocoClient.Move` etc.). Do **not** start with
  low-level joint trajectories; they fight the onboard locomotion controller.
- **URDF** — Unified Robot Description Format; the H1-2 model
  (`static/models/h1_2_description/h1_2.urdf`) rendered by the Three.js viewer.

## Communication

- **DDS** — Data Distribution Service; the pub/sub middleware (CycloneDDS /
  Unitree DDS) carrying robot telemetry and commands. Default domain `0`.
- **`rt/lowstate`** — DDS topic (`unitree_hg.msg.dds_.LowState_`): H1-2 body
  telemetry — motors, IMU, forces, battery.
- **`rt/inspire/state`** — DDS topic (`unitree_go.msg.dds_.MotorStates_`): hand state.
- **`rt/inspire/cmd`** — DDS topic (`MotorCmds_`): planned hand command path.
- **`rt/arm_sdk`** — DDS topic the arm_sdk controller publishes arm/waist commands to.
- **`rt/lowcmd`** — lowest-level body control topic (highest risk). Used here
  only to drive waist yaw (joint 12). See 03 - Safety Interlocks.
- **`lowstate` / `lowcmd`** — Unitree's low-level state/command message pair.
- **SSE (Server-Sent Events)** — one-way HTTP stream; `/events` pushes the state
  snapshot ~every 100 ms. See 04 - HTTP API Reference.
- **MJPEG** — Motion JPEG stream; `/camera.mjpg` bridges the head camera.

## Control paths

- **arm_sdk** — Unitree's arm SDK command path. Drives joints 13–26 (+ waist 12
  via a separate lowcmd hack). Runs a **closed-loop 120 Hz PID + gravity
  feed-forward** controller here. Joint set / gains in
  03 - Safety Interlocks.
- **arm_sdk weight slot** — motor slot 27 (`ARM_SDK_WEIGHT_SLOT`); the enable
  weight (1.0 = full authority) blending arm_sdk into the controller.
- **Retargeting** — mapping one representation to another; here, mapping desired
  joint targets (`_pose_targets`) into the running arm controller's setpoints.
  The tracking loop retargets image position → shoulder setpoints.
- **IK (Inverse Kinematics)** — solving joint angles for a desired end-effector
  pose. **Deliberately avoided** in person-tracking: pointing is 2-DOF, so only
  shoulder yaw/pitch are aimed (see 06 - Person Tracking (CV Feature)).
- **Closed-loop hold** — driving a joint to a target while continuously
  correcting error with PID + gravity feed-forward (vs. open-loop position command).
- **Gravity feed-forward** — a torque term (built from measured torque here) that
  offsets gravity so the arm holds against sag; bounded per joint
  (`ARM_REPLAY_GRAVITY_TAU_LIMITS`).

## Teleoperation & camera

- **XR** — Extended Reality (VR/AR/WebXR). Vision Pro teleoperation via the Vuer
  page on `:8012` (`xr-teleop.service`).
- **Vuer** — the web framework serving the XR teleop page.
- **TeleImager** — Unitree's WebRTC camera server (`:60001`, `teleimager.service`);
  the head-camera source (ZMQ/JPEG) for the tracking loop.
- **Deadman / watchdog** — a required continuous input (or heartbeat) that stops
  motion when released/lost. `xr-home-watchdog.service` watches for lost XR
  home/pose packets.

## CV / math terms

- **YOLO / YOLOv8n** — "You Only Look Once", a single-shot object detector;
  `n` = nano (smallest/fastest). Used for person detection — see
  07 - Detection Service (YOLO).
- **bbox (bounding box)** — rectangle around a detected object: `x1,y1,x2,y2`
  (corners), `cx,cy` (center), normalized 0..1 here.
- **FOV (Field of View)** — the camera's angular coverage; `fov_yaw_rad` /
  `fov_pitch_rad` map normalized image position to a radian span for shoulder aim.
- **EMA (Exponential Moving Average)** — smoothing filter
  `v ← v + α·(new − v)`; `Smoother(alpha=0.35)` reduces setpoint jitter.
- **Dead band** — a small input range (±0.03 here) where output doesn't change,
  so the arm stays still while the person barely moves.
- **Rate limiter** — bounds per-step setpoint change (≤ 0.35 rad/s) below controller caps.
- **Association** — deciding which new detection continues the tracked target
  (largest, then nearest-to-previous center). See `associate()`.
- **Staleness** — detection/telemetry too old to act on; triggers the ramp-to-neutral fail-safe.

## Ops

- **Auto-update timer** — systemd timer pulling `origin/main` every ~60 s to
  deploy. See 08 - Development Workflow.
- **Production gate** — offline release check (`scripts/production_gate.py`).
- **risk-ack** — `armed=true` + `i_understand_risk=true` operator acknowledgement
  required for guarded motion. See 03 - Safety Interlocks.
- **MCP (Model Context Protocol)** — standard for exposing tools to LLM clients;
  `POST /mcp` exposes the chat tools. See 05 - Chat & MCP Tools.
- **Ollama** — on-prem LLM server (OpenAI-compatible) on the AI host.

