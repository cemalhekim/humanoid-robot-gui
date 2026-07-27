---
tags: [testing, unittest, production-gate, ci, offline]
summary: Test suite layout, what's covered offline vs on-robot, and the production gate.
---

# 10 - Testing

## Running the suite

```bash
cd /Users/vodafone/Workspace/humanoid-robot-gui
python3 -m unittest discover -s tests -p 'test_*.py'   # or: make test
```

All tests run **fully offline** — no DDS, no robot, no network. This is a hard
requirement (the tracking plan and
CLAUDE.md both state it). See also 08 - Development Workflow.

## Test files

| File | Classes | ~Tests | Covers |
| --- | --- | --- | --- |
| `tests/test_contracts.py` | `TelemetryContractsTest`, `HomeHoldTest` | 46 | Joint ordering, command limits, `JOINT_LIMITS` clamping, API behavior, Home hold (`HomeHoldTest`) |
| `tests/test_chat.py` | `TelemetryContextTest`, `ChatValidationTest`, `CallLlmTest`, `VoiceTest`, `ChatToolsTest` | 28 | Chat context building, validation, `call_llm`, STT/TTS, tool specs/dispatch |
| `tests/test_mcp.py` | `McpDescriptorTest`, `McpRequestTest` | 15 | MCP tool descriptors + `/mcp` request handling |
| `tests/test_mcp_agent.py` | `SpecConversionTest`, `McpClientTest`, `RunTurnTest` | 8 | The stdlib `tools/mcp_agent.py` client (spec conversion, tool loop) |
| `tests/test_smartplug.py` | `SmartplugTest` | 5 | Home Assistant smart-plug proxy (`/api/smartplug/*`) |
| `tests/test_tracking_math.py` | `MapperTests`, `RateLimiterTests`, `AssociateTests`, `TrackStateTests` | 13 | Pure `tracking.py` logic (see below) |
| `tests/test_tracking_endpoints.py` | `TrackingGatingTests`, `TrackingRouteTests`, `TrackToolTests` | 9 | `/api/track/*` gating, route presence, `track_person` tool spec/dispatch |

`tests/test_contracts.py` is the canonical **contract test** for joint ordering,
command limits, and API behavior (called out in the README as the key test).

## `test_tracking_math.py` coverage

Exercises `tracking.py` (the implemented half of the
person-pointing feature):

- **`MapperTests`** — center of image → template pose; person left/center/right
  swings yaw monotonically; targets **always inside `TRACK_LIMITS`** at all four
  corners; dead band freezes small motion.
- **`RateLimiterTests`** — step is velocity-bounded (`0.35 rad/s · dt`); no overshoot.
- **`AssociateTests`** — no previous target → largest; sticks with nearest-to-previous; empty → `None`.
- **`TrackStateTests`** — fresh detection → `tracking`; target lost → `hold` then
  `stale`; consecutive failures → `aborted`; success resets failure count.

## `test_tracking_endpoints.py` coverage

Added with plan Tasks 3–5 (now implemented):

- **`TrackingGatingTests`** — `request_track_start` returns 403 without risk-ack,
  409 when `TRACKING_ENABLED=0`, 503 when no DDS arm_sdk publisher; `request_track_stop`
  is idempotent (200); `track_snapshot` shape.
- **`TrackingRouteTests`** — `/api/track/start|stop|status` are dispatched in `server.py`.
- **`TrackToolTests`** — `track_person` hidden when disabled, requires `confirm`,
  and stop works offline.

The suite runs fully offline (`domain=0, robot_host="127.0.0.1"`); the whole
project suite is **124 tests** with `make production-gate` green. Only the
dashboard card (Task 6) and on-robot bring-up (Task 7) remain.

## The production gate

```bash
make production-gate        # → python3 scripts/production_gate.py
```

Offline gate: compiles owned Python (`server.py`, `tests/`, `deployment/`,
`tools/`), `node --check` on `static/app.js` + `static/viewer.js` (the only JS
syntax check — there is **no JS test harness**), excludes `execution/`,
`simulation/`, external teleop, and `vendor/`. `--live` adds an explicit robot
reachability check. Must pass **before every push**. Details in
08 - Development Workflow.

## Offline vs on-robot

| Verifiable offline (dev machine / CI) | Requires the robot |
| --- | --- |
| All unittest suite + production gate | Live DDS telemetry (`/api/state` connected) |
| UI loads, static assets, 3D model render | arm_sdk motion, replay, home hold |
| `tracking.py` math/policy/state machine | Detection round-trip latency, FOV calibration |
| Chat validation, tool dispatch shape, MCP protocol | LocoClient walking, wrist commands |
| Route-presence assertions | Failure drills (Wi-Fi stall, walk out of frame, e-stop) |

Person-tracking on-robot bring-up (plan Task 7) is a deliberately ordered,
operator-present checklist: reachability + latency → status-only dry run → arms
at reduced limits → FOV calibration → failure drills → enable chat tool.

