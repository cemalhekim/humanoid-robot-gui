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

## Key facts at a glance

- **Dashboard**: `http://10.2.100.142:8088` (Wi-Fi) / `http://192.168.123.164:8088` (Ethernet) — see [[02 - Network & Hosts]].
- **This is NOT read-only.** Some HTTP endpoints move the robot — see [[03 - Safety Interlocks]].
- **Pushing to `main` auto-deploys to the robot** via a 60 s systemd timer — see [[08 - Development Workflow]].
- **On-prem LLM** (Ollama `qwen3:30b-a3b-instruct-2507-q4_K_M`) drives a guarded chat/[[05 - Chat & MCP Tools|MCP]] tool surface.
- **New CV feature**: right-arm points at the detected person — see [[06 - Person Tracking (CV Feature)]] and [[07 - Detection Service (YOLO)]].

## Repository landmarks

- `server.py` (~5983 lines) — HTTP server, DDS telemetry, guarded command endpoints, closed-loop arm replay, chat/MCP.
- `tracking.py` — pure person-tracking math/policy (no I/O).
- `static/` — dashboard (`index.html`, `app.js`, `viewer.js`, `styles.css`, `diagram.js`).
- `deployment/` — robot systemd units, XR patch scripts, AI-host detection service.
- `docs/superpowers/` — the person-pointing [[06 - Person Tracking (CV Feature)|spec + plan]].
- `tests/` — offline unittest suite (see [[10 - Testing]]).
</content>
</invoke>
