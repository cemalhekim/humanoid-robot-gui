---
tags: [moc, index, h1-2, humanoid-robot-gui]
summary: Map-of-content index for the Unitree H1-2 telemetry + XR operator dashboard knowledge base.
---

# H1-2 Knowledge Base

Knowledge vault for the **`humanoid-robot-gui`** project — a local web dashboard,
telemetry server, guarded robot command surface, and Vision Pro / XR service
deployment for the **Unitree H1-2** humanoid robot.

> [!info] Source of truth
> This vault was mined from the repo at `/Users/vodafone/Workspace/humanoid-robot-gui`
> (README.md, CLAUDE.md, `server.py`, `tracking.py`, the `docs/superpowers/`
> spec + plan, `deployment/`, and `tests/`). Where a claim could not be fully
> verified it is flagged with a `> [!warning]` callout.

## Start here

1. [[00 - Project Overview]] — what the system is and why it exists.
2. [[01 - Architecture]] — components and data flow (with diagram).
3. [[03 - Safety Interlocks]] — **read before touching any motion path**. This code moves a real robot.
4. [[06 - Person Tracking (CV Feature)]] — the newest feature (YOLO person-pointing).

## Map of content

| Note | Topic |
| --- | --- |
| [[00 - Project Overview]] | System purpose, the H1-2 robot, tech stack |
| [[01 - Architecture]] | Components, data flow, mermaid diagram |
| [[02 - Network & Hosts]] | Hosts, ports, auto-update timer |
| [[03 - Safety Interlocks]] | risk-ack, JOINT_LIMITS, XR suspend, cancel-Events, ceilings |
| [[04 - HTTP API Reference]] | Every `/api/*` route with method + purpose |
| [[05 - Chat & MCP Tools]] | LLM tool system, MCP endpoint, feature flags |
| [[06 - Person Tracking (CV Feature)]] | YOLO person-pointing feature, 7-task plan |
| [[07 - Detection Service (YOLO)]] | The YOLOv8n microservice on the AI host |
| [[08 - Development Workflow]] | Git workflow, auto-deploy, production gate |
| [[09 - Glossary]] | H1-2, DDS, arm_sdk, retargeting, EMA, FOV, bbox, ... |
| [[10 - Testing]] | Test suite layout, offline vs on-robot coverage |
| [[11 - Teleoperation (Vision Pro & XR)]] | Vision Pro / WebXR teleop service |
| [[12 - Camera & Video Streaming]] | Head camera / webcam pipeline, MJPEG stream |
| [[13 - Telemetry Recording & Pose Editor]] | Recording schema, `/api/recording/*`, 6-DOF pose editor |
| [[14 - Recording Replay & Digital Twin]] | Closed-loop arm replay, red/green twin, dry-run planning |
| [[15 - Locomotion Control]] | LocoClient actions, bounds, risk-ack gate |
| [[16 - Arm Control & Command Surfaces]] | Wrist / home / straight / chill / arm_sdk replay |
| [[17 - 3D URDF Viewer]] | Three.js viewer, blue/green/red models, IK editing |
| [[18 - Body, IMU, Battery & Hand Telemetry]] | DDS lowstate → `/api/state` + SSE, hand state |
| [[19 - Sentry Mode & Head-Lock]] | Bullseye Mode (UI name since 2026-07-27): server-enforced person-following master switch |
| [[20 - LLM Arm Pose Proposals & Mimic]] | Propose → green twin → 👍/👎 → move; photo→pose mimic; feedback CSV + plots |
| [[21 - Semantic Teleoperation Pipeline]] | Monorepo teleop/sim/execution subsystems |
| [[22 - Deployment & Runtime Services]] | systemd units, auto-update timer, runtime services |
| [[23 - Smart Plug & Home Assistant]] | Sonoff plug showcase (⚠️ **removed** 2026-07-23 — historical) |
| [[24 - Control Gains, PID & Shared Mechanisms]] | kp/kd tables + the shared arm-replay PID |
| [[25 - Known Issues & Optimization Audit]] | Fixed + deferred perf/safety findings |
| [[27 - Mimic Mode (Live Motion)]] | Robot mirrors the operator's upper-body pose live (both arms, webcam keypoints) |
| [[28 - Pepper Robot (LLM Assistant)]] | Second lab robot: SoftBank Pepper at `10.2.100.187` with on-board German voice-assistant (`pepper_llm`) |
| [[29 - Jetson AGX Orin]] | Edge GPU box: AGX Orin 64 GB at `192.168.123.100` (static, robot subnet), JetPack 5.1.2, `ssh jetson` |
| [[26 - Repository Docs Index]] | Every `docs/` file mapped to its KB note |

## Key facts at a glance

- **Dashboard**: `http://10.2.100.142:8088` (Wi-Fi) / `http://192.168.123.164:8088` (Ethernet) — see [[02 - Network & Hosts]].
- **This is NOT read-only.** Some HTTP endpoints move the robot — see [[03 - Safety Interlocks]].
- **Pushing to `main` auto-deploys to the robot** via a 60 s systemd timer — see [[08 - Development Workflow]].
- **On-prem LLM** (Ollama `qwen3:30b-a3b-instruct-2507-q4_K_M`) drives a guarded chat/[[05 - Chat & MCP Tools|MCP]] tool surface.
- **New CV feature**: right-arm points at the detected person — see [[06 - Person Tracking (CV Feature)]] and [[07 - Detection Service (YOLO)]].
- **LLM arm-pose proposals + photo mimic**: ask (or attach a photo) → green digital-twin preview → 👍/👎 → guarded move — see [[20 - LLM Arm Pose Proposals & Mimic]].

## Repository landmarks

- `server.py` (~5983 lines) — HTTP server, DDS telemetry, guarded command endpoints, closed-loop arm replay, chat/MCP.
- `tracking.py` — pure person-tracking math/policy (no I/O).
- `static/` — dashboard (`index.html`, `app.js`, `viewer.js`, `styles.css`, `diagram.js`).
- `deployment/` — robot systemd units, XR patch scripts, AI-host detection service.
- `docs/superpowers/` — the person-pointing [[06 - Person Tracking (CV Feature)|spec + plan]].
- `tests/` — offline unittest suite (see [[10 - Testing]]).
</content>
</invoke>
