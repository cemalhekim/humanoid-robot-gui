---
tags: [index, docs, meta, navigation, presentations, diagrams, superpowers]
summary: The master index of every file under docs/ (outside this knowledge base) — markdown notes, .drawio diagrams, PDF/PPTX decks, images, the reference C++ example, and the superpowers specs/plans — each mapped to the KB note that covers it.
---

# 26 - Repository Docs Index

> [!abstract] Goal
> Guarantee that **every artifact under `docs/` is represented in the knowledge
> base**. This note enumerates each file (excluding `docs/knowledge-base/` itself
> and the top-level `README.md`) with a one-line description and the canonical KB
> note that explains it. If a new doc is added to the repo, add a row here.

> [!note] How this list was generated
> Enumerated with `find docs -type f -not -path 'docs/knowledge-base/*'`. Paths
> are relative to the repo root. "Covered by" points at the KB note whose subject
> the file belongs to; some files legitimately map to more than one.

## Top-level design & research notes (`docs/*.md`)

| Path | What it is | Covered by |
| --- | --- | --- |
| `docs/2026-07-24-optimization-and-bug-audit.md` | The unattended optimization + safety-bug audit (fixed / deferred / open findings) | [[25 - Known Issues & Optimization Audit]] |
| `docs/production_refinement.md` | The behavior-preserving production-hardening loop + `make production-gate` rules | [[25 - Known Issues & Optimization Audit]] · [[08 - Development Workflow]] |
| `docs/gain_selection_research.md` | Conservative gain-selection algorithm for future trajectory execution | [[24 - Control Gains, PID & Shared Mechanisms]] |
| `docs/trajectory_executor_integration.md` | The safe path from replay preview to supervised robot execution | [[14 - Recording Replay & Digital Twin]] · [[16 - Arm Control & Command Surfaces]] |
| `docs/robot_control_paths.md` | Inventory of live ROS2/DDS motion & control topics visible from this machine | [[01 - Architecture]] · [[16 - Arm Control & Command Surfaces]] |
| `docs/workspace_inventory.md` | Consolidated layout of the wider local robot workspace around this repo | [[01 - Architecture]] · [[00 - Project Overview]] |
| `docs/h1_2_robot_platform_presentation.md` | Executive presentation draft (telemetry, XR, digital twin) for leadership review | [[00 - Project Overview]] |

## Diagrams (`docs/*.drawio`)

Rendered in-app by the docs `.drawio` viewer (`static/diagram.js`).

| Path | What it is | Covered by |
| --- | --- | --- |
| `docs/arm_sdk_replay_flow.drawio` | Flow of physical arm replay via `rt/arm_sdk` (`request_robot_replay` → `execute_arm_sdk_replay` → `run_replay` thread), incl. guard decisions | [[14 - Recording Replay & Digital Twin]] · [[16 - Arm Control & Command Surfaces]] · [[03 - Safety Interlocks]] |
| `docs/robot_telemetry_xr_workflow.drawio` | End-to-end repo + XR teleop workflow (dev repo → deploy → robot runtime services → external users/devices) | [[01 - Architecture]] · [[22 - Deployment & Runtime Services]] · [[11 - Teleoperation (Vision Pro & XR)]] |

## Reference source (`docs/reference/`)

| Path | What it is | Covered by |
| --- | --- | --- |
| `docs/reference/h1_2_arm_sdk_dds_example.cpp` | Vendor Unitree arm_sdk CycloneDDS C++ example — reference for the `rt/arm_sdk` command path | [[16 - Arm Control & Command Surfaces]] |

## Presentations (`docs/presentations/`, `docs/*.pdf`)

| Path | What it is | Covered by |
| --- | --- | --- |
| `docs/h1_2_robot_platform_presentation.pdf` | PDF export of the platform presentation draft | [[00 - Project Overview]] |
| `docs/presentations/Unitree-H1-Feature-Deck.pptx` | Feature deck (v1) source slides | [[00 - Project Overview]] |
| `docs/presentations/Unitree-H1-Feature-Deck.pdf` | Feature deck (v1) PDF export | [[00 - Project Overview]] |
| `docs/presentations/Unitree-H1-Feature-Deck.pptx.inspect.ndjson` | Slide-content inspection dump of the v1 deck | [[00 - Project Overview]] |
| `docs/presentations/Unitree-H1-Feature-Deck-v2.pptx` | Feature deck (v2) source slides | [[00 - Project Overview]] |
| `docs/presentations/Unitree-H1-Feature-Deck-v2.pdf` | Feature deck (v2) PDF export | [[00 - Project Overview]] |
| `docs/presentations/Unitree-H1-Feature-Deck-v2.pptx.inspect.ndjson` | Slide-content inspection dump of the v2 deck | [[00 - Project Overview]] |
| `docs/presentations/telemetry-recorder-feature-slide.pptx` | Single feature slide on the telemetry recorder | [[13 - Telemetry Recording & Pose Editor]] |
| `docs/presentations/telemetry-recorder-feature-slide.pptx.inspect.ndjson` | Slide-content inspection dump of the recorder slide | [[13 - Telemetry Recording & Pose Editor]] |
| `docs/presentations/teleoperation-hands-collage.png` | Collage image of XR hand teleoperation used in the deck | [[11 - Teleoperation (Vision Pro & XR)]] |

## Images (`docs/images/`)

| Path | What it is | Covered by |
| --- | --- | --- |
| `docs/images/humanoid-robot-resting.jpg` | Photo of the H1-2 in its resting pose | [[00 - Project Overview]] |
| `docs/images/vodafone_logo.png` | Vodafone branding used in presentations | [[00 - Project Overview]] |
| `docs/images/xr-control-methods.jpg` | Diagram of the XR / Vision Pro control methods | [[11 - Teleoperation (Vision Pro & XR)]] |
| `docs/images/telemetry-recorder-sequence.jpg` | Illustration of the telemetry-recorder sequence flow | [[13 - Telemetry Recording & Pose Editor]] |
| `docs/images/telegram-robot-reply.jpg` | Screenshot of a Telegram-channel robot reply | [[05 - Chat & MCP Tools]] |
| `docs/images/Bildschirmfoto 2026-07-07 um 14.42.13.png` | Dashboard UI screenshot (2026-07-07) | [[00 - Project Overview]] |
| `docs/images/Bildschirmfoto 2026-07-07 um 14.43.56.png` | Dashboard UI screenshot (2026-07-07) | [[00 - Project Overview]] |

## Superpowers specs (`docs/superpowers/specs/`)

Approved design specs that fed the feature builds.

| Path | What it is | Covered by |
| --- | --- | --- |
| `docs/superpowers/specs/2026-07-21-person-pointing-design.md` | Design for continuous arm-pointing at the detected person | [[06 - Person Tracking (CV Feature)]] |
| `docs/superpowers/specs/2026-07-22-sentry-mode-person-detection-design.md` | Sentry Mode Phase 1 — passive person-detection overlay | [[19 - Sentry Mode & Head-Lock]] |
| `docs/superpowers/specs/2026-07-22-sentry-head-lock-buttons-design.md` | Sentry Mode Phase 2 — head-tracked glowing lock buttons (UI-only) | [[19 - Sentry Mode & Head-Lock]] |
| `docs/superpowers/specs/2026-07-22-sentry-pointing-phase3-design.md` | Sentry Mode Phase 3 — closed-loop pointing at the locked person | [[19 - Sentry Mode & Head-Lock]] · [[06 - Person Tracking (CV Feature)]] |

## Superpowers plans (`docs/superpowers/plans/`)

Task-by-task implementation plans (checkbox-tracked) that executed the specs.

| Path | What it is | Covered by |
| --- | --- | --- |
| `docs/superpowers/plans/2026-07-21-person-pointing.md` | Implementation plan for person-tracking / arm pointing | [[06 - Person Tracking (CV Feature)]] |
| `docs/superpowers/plans/2026-07-22-sentry-mode-phase1.md` | Implementation plan for Sentry Phase 1 (detection overlay) | [[19 - Sentry Mode & Head-Lock]] |
| `docs/superpowers/plans/2026-07-22-sentry-head-lock-buttons.md` | Implementation plan for Sentry Phase 2 (head-lock buttons) | [[19 - Sentry Mode & Head-Lock]] |
| `docs/superpowers/plans/2026-07-23-llm-pose-proposals.md` | Implementation plan for LLM arm-pose proposals with digital-twin preview | [[20 - LLM Arm Pose Proposals & Mimic]] · [[21 - Semantic Teleoperation Pipeline]] |
| `docs/superpowers/plans/2026-07-23-pose-feedback.md` | Implementation plan for the pose feedback loop (like/dislike → CSV → prompt anchors) | [[20 - LLM Arm Pose Proposals & Mimic]] · [[21 - Semantic Teleoperation Pipeline]] |

> [!note] Coverage check
> The 34 files returned by `find docs -type f -not -path 'docs/knowledge-base/*'`
> are all listed above. The excluded set — everything under
> `docs/knowledge-base/` (this KB) and the top-level `README.md` — is documented
> by the KB notes themselves and [[00 - Project Overview]].

## Related

[[00 - Project Overview]] · [[01 - Architecture]] · [[08 - Development Workflow]] · [[25 - Known Issues & Optimization Audit]] · [[09 - Glossary]]
