# Person Tracking / Arm Pointing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The H1-2 continuously points its right arm at the person detected in the head-camera image, inside all existing safety interlocks.

**Architecture:** A GPU detection microservice on the AI host (10.2.125.3:8188, already prototyped) returns normalized person boxes for posted JPEGs. A new tracking session thread in `server.py` reads head-camera frames from the in-memory buffer, calls the service at ~8 Hz, maps the target person's image position to right-shoulder joint setpoints through pure functions in a new `tracking.py`, and publishes velocity-bounded arm_sdk commands. Guarded HTTP endpoints, a dashboard card, and a chat/MCP tool control it.

**Tech Stack:** Python 3 stdlib (robot side — no new robot deps), ultralytics YOLOv8n + OpenCV (AI host only), existing unittest suite, vanilla JS dashboard.

**Spec:** `docs/superpowers/specs/2026-07-21-person-pointing-design.md` — read it first.

## Global Constraints

- Robot is a REAL H1-2. Never weaken existing interlocks: `has_risk_ack` gating, `_suspend_xr_motion_publishers`, `JOINT_LIMITS`, cancel-Event mutual exclusion.
- `make production-gate` must pass before every push (CLAUDE.md).
- Commit + push after every task; every commit message ends with:
  `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`
- Pushing to `main` auto-deploys to the robot (60 s autoupdate timer). All new behavior ships dark behind `TRACKING_ENABLED=0` default.
- No new Python dependencies on the robot side (stdlib + existing imports only).
- Tests must run offline (no DDS, no robot, no network): `python3 -m unittest discover -s tests -p 'test_*.py'`.

## File Structure

- Create `tracking.py` — pure logic: image→joint mapping, smoothing, rate limiting, target association, staleness state machine. Zero imports from server.py; fully unit-testable.
- Create `deployment/ai_host/detect_service.py` — the detection microservice (canonical copy of the prototype already running on 10.2.125.3).
- Create `deployment/ai_host/person-detect.service` + `deployment/ai_host/README.md` — systemd unit and install steps for the AI host.
- Modify `server.py` — TrackingController session (new methods on `TelemetryStore`), three HTTP routes, chat tool spec + dispatch, env config block.
- Modify `static/index.html`, `static/app.js` — Tracking card.
- Create `tests/test_tracking_math.py`, `tests/test_tracking_endpoints.py`.

---

### Task 1: Pure tracking math module (`tracking.py`)

**Files:**
- Create: `tracking.py`
- Test: `tests/test_tracking_math.py`

**Interfaces:**
- Produces (used by Tasks 3-4):
  - `PointingMapper(fov_yaw_rad=1.25, fov_pitch_rad=0.9, yaw_offset=0.0, pitch_offset=0.35, dead_band=0.03).targets(cx, cy) -> dict[int, float]` — right-arm joint targets keyed by joint index, clamped to `TRACK_LIMITS`.
  - `RateLimiter(max_step_rad_s=0.35).step(current: dict[int, float], target: dict[int, float], dt: float) -> dict[int, float]`
  - `Smoother(alpha=0.35).update(value: dict[int, float]) -> dict[int, float]`
  - `associate(persons: list[dict], prev_cx: float | None, prev_cy: float | None) -> dict | None` — pick the tracked person.
  - `TrackState` state machine: `.on_detection(persons, now)`, `.on_failure(now)`, `.phase` in `{"tracking", "hold", "stale", "aborted"}`.
  - Constants: `R_SHOULDER_PITCH = 20`, `R_SHOULDER_ROLL = 21`, `R_SHOULDER_YAW = 22`, `R_ELBOW = 23`, `POINTING_TEMPLATE`, `NEUTRAL_TEMPLATE`, `TRACK_LIMITS`.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_tracking_math.py
import unittest

import tracking


class MapperTests(unittest.TestCase):
    def setUp(self):
        self.mapper = tracking.PointingMapper()

    def test_center_of_image_gives_template_pose(self):
        targets = self.mapper.targets(0.5, 0.5)
        self.assertEqual(set(targets), set(tracking.POINTING_TEMPLATE))
        self.assertAlmostEqual(
            targets[tracking.R_SHOULDER_YAW],
            tracking.POINTING_TEMPLATE[tracking.R_SHOULDER_YAW],
            places=5,
        )

    def test_person_left_of_center_swings_yaw_left(self):
        left = self.mapper.targets(0.2, 0.5)[tracking.R_SHOULDER_YAW]
        right = self.mapper.targets(0.8, 0.5)[tracking.R_SHOULDER_YAW]
        self.assertNotEqual(left, right)
        self.assertLess(left, right)

    def test_targets_always_inside_track_limits(self):
        for cx, cy in [(0.0, 0.0), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0)]:
            for joint, value in self.mapper.targets(cx, cy).items():
                lo, hi = tracking.TRACK_LIMITS[joint]
                self.assertGreaterEqual(value, lo, f"joint {joint} cx={cx} cy={cy}")
                self.assertLessEqual(value, hi, f"joint {joint} cx={cx} cy={cy}")

    def test_dead_band_freezes_small_motion(self):
        a = self.mapper.targets(0.500, 0.500)
        b = self.mapper.targets(0.515, 0.510)  # inside +-0.03 dead band
        self.assertEqual(a, b)


class RateLimiterTests(unittest.TestCase):
    def test_step_is_velocity_bounded(self):
        limiter = tracking.RateLimiter(max_step_rad_s=0.35)
        current = {20: 0.0}
        target = {20: 1.0}
        stepped = limiter.step(current, target, dt=0.1)
        self.assertAlmostEqual(stepped[20], 0.035, places=6)

    def test_step_does_not_overshoot(self):
        limiter = tracking.RateLimiter(max_step_rad_s=0.35)
        stepped = limiter.step({20: 0.99}, {20: 1.0}, dt=1.0)
        self.assertAlmostEqual(stepped[20], 1.0, places=6)


class AssociateTests(unittest.TestCase):
    def person(self, cx, cy, area):
        half = area ** 0.5 / 2
        return {"cx": cx, "cy": cy, "x1": cx - half, "x2": cx + half,
                "y1": cy - half, "y2": cy + half, "conf": 0.9}

    def test_no_previous_target_picks_largest(self):
        big = self.person(0.7, 0.5, 0.20)
        small = self.person(0.2, 0.5, 0.05)
        self.assertEqual(tracking.associate([small, big], None, None), big)

    def test_sticks_with_nearest_to_previous(self):
        near = self.person(0.32, 0.5, 0.05)
        far_big = self.person(0.9, 0.5, 0.20)
        self.assertEqual(tracking.associate([near, far_big], 0.30, 0.5), near)

    def test_empty_returns_none(self):
        self.assertIsNone(tracking.associate([], 0.5, 0.5))


class TrackStateTests(unittest.TestCase):
    def test_fresh_detection_keeps_tracking(self):
        state = tracking.TrackState(stale_after_s=1.5, hold_s=2.0, max_failures=10)
        state.on_detection([{"cx": 0.5, "cy": 0.5, "x1": 0.4, "x2": 0.6,
                             "y1": 0.3, "y2": 0.7, "conf": 0.9}], now=100.0)
        self.assertEqual(state.phase, "tracking")

    def test_target_lost_holds_then_goes_stale(self):
        state = tracking.TrackState(stale_after_s=1.5, hold_s=2.0, max_failures=10)
        state.on_detection([{"cx": 0.5, "cy": 0.5, "x1": 0.4, "x2": 0.6,
                             "y1": 0.3, "y2": 0.7, "conf": 0.9}], now=100.0)
        state.on_detection([], now=101.0)
        self.assertEqual(state.phase, "hold")
        state.on_detection([], now=103.5)
        self.assertEqual(state.phase, "stale")

    def test_consecutive_failures_abort(self):
        state = tracking.TrackState(stale_after_s=1.5, hold_s=2.0, max_failures=3)
        for i in range(3):
            state.on_failure(now=100.0 + i)
        self.assertEqual(state.phase, "aborted")

    def test_success_resets_failure_count(self):
        state = tracking.TrackState(stale_after_s=1.5, hold_s=2.0, max_failures=3)
        state.on_failure(now=100.0)
        state.on_failure(now=100.1)
        state.on_detection([{"cx": 0.5, "cy": 0.5, "x1": 0.4, "x2": 0.6,
                             "y1": 0.3, "y2": 0.7, "conf": 0.9}], now=100.2)
        self.assertEqual(state.failures, 0)
        self.assertEqual(state.phase, "tracking")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /Users/vodafone/Workspace/humanoid-robot-gui && python3 -m unittest tests.test_tracking_math -v`
Expected: FAIL / ERROR with `ModuleNotFoundError: No module named 'tracking'`

- [ ] **Step 3: Implement `tracking.py`**

```python
"""Pure math and policy for the person-tracking / arm-pointing feature.

No imports from server.py and no I/O: everything here is unit-testable
offline. server.py owns frames, HTTP, DDS; this module owns the decisions.
Spec: docs/superpowers/specs/2026-07-21-person-pointing-design.md
"""
from __future__ import annotations

from typing import Any

# H1-2 right-arm joint indices (must match server.py JOINT_LIMITS indices).
R_SHOULDER_PITCH = 20
R_SHOULDER_ROLL = 21
R_SHOULDER_YAW = 22
R_ELBOW = 23

# Fixed "pointing" arm shape: elbow slightly bent, arm raised forward.
# Shoulder pitch/yaw are the two aimed joints; roll and elbow stay fixed.
POINTING_TEMPLATE: dict[int, float] = {
    R_SHOULDER_PITCH: 0.35,
    R_SHOULDER_ROLL: -0.10,
    R_SHOULDER_YAW: 0.0,
    R_ELBOW: 0.25,
}

# Where the arm parks when tracking is stale/stopped (relaxed at the side).
NEUTRAL_TEMPLATE: dict[int, float] = {
    R_SHOULDER_PITCH: 0.0,
    R_SHOULDER_ROLL: -0.05,
    R_SHOULDER_YAW: 0.0,
    R_ELBOW: 0.3,
}

# Conservative aiming envelope, intentionally tighter than server.py
# JOINT_LIMITS. server.py re-clamps against JOINT_LIMITS anyway.
TRACK_LIMITS: dict[int, tuple[float, float]] = {
    R_SHOULDER_PITCH: (-0.6, 1.2),
    R_SHOULDER_ROLL: (-0.6, 0.2),
    R_SHOULDER_YAW: (-1.0, 1.0),
    R_ELBOW: (0.1, 1.2),
}


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class PointingMapper:
    """Map a normalized image position (cx, cy in 0..1) to joint targets.

    Linear map across the camera field of view; a dead band keeps the arm
    still while the person barely moves. No IK: only shoulder pitch/yaw aim.
    """

    def __init__(
        self,
        fov_yaw_rad: float = 1.25,
        fov_pitch_rad: float = 0.9,
        yaw_offset: float = 0.0,
        pitch_offset: float = 0.35,
        dead_band: float = 0.03,
    ) -> None:
        self.fov_yaw_rad = fov_yaw_rad
        self.fov_pitch_rad = fov_pitch_rad
        self.yaw_offset = yaw_offset
        self.pitch_offset = pitch_offset
        self.dead_band = dead_band
        self._last_cx: float | None = None
        self._last_cy: float | None = None

    def targets(self, cx: float, cy: float) -> dict[int, float]:
        if (
            self._last_cx is not None
            and abs(cx - self._last_cx) <= self.dead_band
            and abs(cy - self._last_cy) <= self.dead_band
        ):
            cx, cy = self._last_cx, self._last_cy
        else:
            self._last_cx, self._last_cy = cx, cy

        yaw = self.yaw_offset + (cx - 0.5) * self.fov_yaw_rad
        # Image y grows downward; higher person position = raise the arm.
        pitch = self.pitch_offset + (0.5 - cy) * self.fov_pitch_rad

        out = dict(POINTING_TEMPLATE)
        out[R_SHOULDER_YAW] = yaw
        out[R_SHOULDER_PITCH] = pitch
        return {
            joint: _clamp(value, *TRACK_LIMITS[joint])
            for joint, value in out.items()
        }


class RateLimiter:
    """Bound setpoint changes to max_step_rad_s, below controller caps."""

    def __init__(self, max_step_rad_s: float = 0.35) -> None:
        self.max_step_rad_s = max_step_rad_s

    def step(
        self,
        current: dict[int, float],
        target: dict[int, float],
        dt: float,
    ) -> dict[int, float]:
        max_step = self.max_step_rad_s * max(dt, 0.0)
        out: dict[int, float] = {}
        for joint, goal in target.items():
            now = current.get(joint, goal)
            out[joint] = now + _clamp(goal - now, -max_step, max_step)
        return out


class Smoother:
    """Exponential moving average over joint target dicts."""

    def __init__(self, alpha: float = 0.35) -> None:
        self.alpha = alpha
        self._value: dict[int, float] | None = None

    def update(self, value: dict[int, float]) -> dict[int, float]:
        if self._value is None:
            self._value = dict(value)
        else:
            for joint, v in value.items():
                prev = self._value.get(joint, v)
                self._value[joint] = prev + self.alpha * (v - prev)
        return dict(self._value)


def _area(person: dict[str, Any]) -> float:
    return max(0.0, person["x2"] - person["x1"]) * max(0.0, person["y2"] - person["y1"])


def associate(
    persons: list[dict[str, Any]],
    prev_cx: float | None,
    prev_cy: float | None,
) -> dict[str, Any] | None:
    """Pick the person to track: nearest to previous target, else largest."""
    if not persons:
        return None
    if prev_cx is None or prev_cy is None:
        return max(persons, key=_area)
    return min(
        persons,
        key=lambda p: (p["cx"] - prev_cx) ** 2 + (p["cy"] - prev_cy) ** 2,
    )


class TrackState:
    """Staleness / failure state machine for one tracking session.

    Phases: tracking (fresh target), hold (target briefly lost, keep pose),
    stale (lost too long, arm should go neutral), aborted (too many
    consecutive detect-service failures — session must end).
    """

    def __init__(
        self,
        stale_after_s: float = 1.5,
        hold_s: float = 2.0,
        max_failures: int = 10,
    ) -> None:
        self.stale_after_s = stale_after_s
        self.hold_s = hold_s
        self.max_failures = max_failures
        self.phase = "stale"
        self.failures = 0
        self.target: dict[str, Any] | None = None
        self.last_seen: float | None = None

    def on_detection(self, persons: list[dict[str, Any]], now: float) -> None:
        if self.phase == "aborted":
            return
        self.failures = 0
        prev_cx = self.target["cx"] if self.target else None
        prev_cy = self.target["cy"] if self.target else None
        picked = associate(persons, prev_cx, prev_cy)
        if picked is not None:
            self.target = picked
            self.last_seen = now
            self.phase = "tracking"
            return
        if self.last_seen is None:
            self.phase = "stale"
        elif now - self.last_seen <= self.hold_s:
            self.phase = "hold"
        else:
            self.phase = "stale"
            self.target = None

    def on_failure(self, now: float) -> None:
        if self.phase == "aborted":
            return
        self.failures += 1
        if self.failures >= self.max_failures:
            self.phase = "aborted"
            return
        if self.last_seen is None or now - self.last_seen > self.stale_after_s:
            self.phase = "stale"
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m unittest tests.test_tracking_math -v`
Expected: all tests PASS

- [ ] **Step 5: Commit**

```bash
git add tracking.py tests/test_tracking_math.py
git commit -m "Add tracking.py: pure mapping/association/staleness logic for person pointing

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git push
```

---

### Task 2: Version the AI-host detection service in the repo

The prototype already runs on 10.2.125.3 (`~/person-tracking/detect_service.py`, port 8188, venv with ultralytics — verified 2026-07-21, 6.8 ms/frame on the A40). This task makes the repo the canonical source and adds a systemd unit.

**Files:**
- Create: `deployment/ai_host/detect_service.py` (copy of the running prototype — content below)
- Create: `deployment/ai_host/person-detect.service`
- Create: `deployment/ai_host/README.md`

**Interfaces:**
- Produces: HTTP contract used by Task 3 — `POST http://10.2.125.3:8188/detect` with JPEG body → `{"persons": [{"x1","y1","x2","y2","cx","cy","conf"}...], "ms": float, "w": int, "h": int}` (coords normalized 0..1, persons sorted by area desc, conf ≥ 0.4, person class only); `GET /health` → `{"ok": true}`.

- [ ] **Step 1: Add `deployment/ai_host/detect_service.py`**

Copy the exact file that is running on the AI host (fetch it, do not retype):

```bash
sshpass -p "$AI_HOST_PW" scp -O vodafone@10.2.125.3:person-tracking/detect_service.py deployment/ai_host/detect_service.py
```

(`AI_HOST_PW` is in the operator's memory notes; never commit it.)

- [ ] **Step 2: Add the systemd unit `deployment/ai_host/person-detect.service`**

```ini
[Unit]
Description=YOLOv8n person-detection HTTP service (port 8188)
After=network-online.target

[Service]
Type=simple
WorkingDirectory=/home/vodafone/person-tracking
ExecStart=/home/vodafone/person-tracking/venv/bin/python /home/vodafone/person-tracking/detect_service.py
Restart=always
RestartSec=3

[Install]
WantedBy=default.target
```

- [ ] **Step 3: Add `deployment/ai_host/README.md`**

```markdown
# AI host (10.2.125.3) person-detection service

One-time install (as user `vodafone` on the AI host):

    mkdir -p ~/person-tracking && cd ~/person-tracking
    python3 -m venv venv
    ./venv/bin/pip install ultralytics opencv-python-headless
    # copy detect_service.py here from this repo, then:
    mkdir -p ~/.config/systemd/user
    cp person-detect.service ~/.config/systemd/user/
    systemctl --user daemon-reload
    systemctl --user enable --now person-detect.service
    loginctl enable-linger vodafone

Smoke test:

    curl -s http://localhost:8188/health          # {"ok": true}
    curl -s -X POST --data-binary @some.jpg http://localhost:8188/detect

The robot reaches this service at http://10.2.125.3:8188/detect
(TRACKING_DETECT_URL). First request after start is slow (~2 s model load);
steady state is <10 ms inference on the A40 GPU.
```

- [ ] **Step 4: Run the production gate**

Run: `make production-gate`
Expected: `Production gate passed.` (the new py file must compile)

- [ ] **Step 5: Commit**

```bash
git add deployment/ai_host/
git commit -m "Version the AI-host person-detection service + systemd unit

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git push
```

- [ ] **Step 6: Install the unit on the AI host (manual, robot not required)**

Run the README install block over SSH; verify `systemctl --user status person-detect.service` is `active (running)` and `/health` responds after a reboot-safe restart (`systemctl --user restart person-detect.service`).

---

### Task 3: TrackingController session in `server.py`

**Files:**
- Modify: `server.py` — env config near the other `TRACKING`-style flags (around line 419 where `LLM_TOOL_MOVE_ENABLED` lives), new `TelemetryStore` state in `__init__` (near line 2250 where `replay_cancel` lives), new methods after `request_robot_replay`'s helpers (near line 3300).
- Test: `tests/test_tracking_endpoints.py` (gating logic; extended in Task 4)

**Interfaces:**
- Consumes: `tracking.PointingMapper/RateLimiter/Smoother/TrackState/NEUTRAL_TEMPLATE/POINTING_TEMPLATE` (Task 1); detect-service HTTP contract (Task 2); existing `has_risk_ack`, `_suspend_xr_motion_publishers`, `_build_arm_sdk_trajectory_cmd`, `store.get_camera_frame()`, `ARM_SDK_KP/ARM_SDK_KD`, `command_lock`, cancel-Event conventions.
- Produces (used by Tasks 4-6):
  - `TelemetryStore.request_track_start(payload) -> tuple[int, dict]`
  - `TelemetryStore.request_track_stop() -> tuple[int, dict]`
  - `TelemetryStore.track_snapshot() -> dict` — `{"enabled", "active", "phase", "target", "detection_age_s", "failures", "message", "updated_at"}`

- [ ] **Step 1: Add env config**

Next to the other feature flags (grep for `LLM_TOOL_MOVE_ENABLED =` and match the existing env-parsing idiom in that block — reuse the repo's helper if one exists):

```python
TRACKING_ENABLED = os.environ.get("TRACKING_ENABLED", "0").strip().lower() in {"1", "true", "yes"}
TRACKING_DETECT_URL = os.environ.get("TRACKING_DETECT_URL", "http://10.2.125.3:8188/detect").strip()
TRACKING_CAMERA = os.environ.get("TRACKING_CAMERA", "head").strip().lower()
TRACKING_RATE_HZ = max(1.0, min(15.0, float(os.environ.get("TRACKING_RATE_HZ", "8") or 8)))
TRACKING_MAX_SESSION_S = max(30.0, float(os.environ.get("TRACKING_MAX_SESSION_S", "600") or 600))
LLM_TOOL_TRACK_ENABLED = os.environ.get("LLM_TOOL_TRACK_ENABLED", "0").strip().lower() in {"1", "true", "yes"}
```

Add `import tracking` with the other local imports at the top of server.py.

- [ ] **Step 2: Write failing gating tests**

```python
# tests/test_tracking_endpoints.py
import unittest

import server


class TrackingGatingTests(unittest.TestCase):
    def make_store(self):
        # Same offline construction the existing tests use — copy the
        # TelemetryStore setup idiom from tests/test_contracts.py verbatim.
        return server.TelemetryStore(robot_host="127.0.0.1")

    def test_start_requires_risk_ack(self):
        store = self.make_store()
        status, response = store.request_track_start({})
        self.assertEqual(status, 403)
        self.assertFalse(response["ok"])

    def test_start_refused_when_feature_disabled(self):
        store = self.make_store()
        old = server.TRACKING_ENABLED
        server.TRACKING_ENABLED = False
        try:
            status, response = store.request_track_start(
                {"armed": True, "i_understand_risk": True}
            )
        finally:
            server.TRACKING_ENABLED = old
        self.assertEqual(status, 409)

    def test_start_without_dds_returns_503(self):
        store = self.make_store()
        old = server.TRACKING_ENABLED
        server.TRACKING_ENABLED = True
        try:
            status, response = store.request_track_start(
                {"armed": True, "i_understand_risk": True}
            )
        finally:
            server.TRACKING_ENABLED = old
        self.assertEqual(status, 503)  # no wrist_publisher offline

    def test_stop_is_idempotent(self):
        store = self.make_store()
        status, response = store.request_track_stop()
        self.assertEqual(status, 200)
        self.assertTrue(response["ok"])

    def test_snapshot_shape(self):
        store = self.make_store()
        snap = store.track_snapshot()
        for key in ("enabled", "active", "phase", "message", "updated_at"):
            self.assertIn(key, snap)


if __name__ == "__main__":
    unittest.main()
```

NOTE for the implementer: if `TelemetryStore(robot_host=...)` is not how
`tests/test_contracts.py` builds a store, copy that file's construction
exactly — the assertion targets stay the same.

- [ ] **Step 3: Run tests to verify they fail**

Run: `python3 -m unittest tests.test_tracking_endpoints -v`
Expected: ERROR — `AttributeError: 'TelemetryStore' object has no attribute 'request_track_start'`

- [ ] **Step 4: Implement the session on `TelemetryStore`**

In `__init__`, next to `self.replay_cancel`:

```python
self.track_cancel: threading.Event | None = None
self.track_thread: threading.Thread | None = None
self.track_status: dict[str, Any] = {
    "enabled": TRACKING_ENABLED,
    "active": False,
    "phase": "idle",
    "target": None,
    "detection_age_s": None,
    "failures": 0,
    "message": "Tracking has not been started.",
    "updated_at": None,
}
```

New methods (place after the replay helpers; follow `run_replay`'s structure for DDS publishing and `_suspend_xr_motion_publishers` usage):

```python
def track_snapshot(self) -> dict[str, Any]:
    with self.command_lock:
        snap = dict(self.track_status)
    snap["enabled"] = TRACKING_ENABLED
    return snap

def _set_track_status(self, **fields: Any) -> None:
    with self.command_lock:
        self.track_status.update(fields, updated_at=time.time())

def request_track_stop(self) -> tuple[int, dict[str, Any]]:
    with self.command_lock:
        cancel = self.track_cancel
    if cancel is not None:
        cancel.set()
    self._set_track_status(active=False, phase="idle", message="Tracking stopped by operator.")
    return 200, {"ok": True, "tracking": self.track_snapshot()}

def request_track_start(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    if not has_risk_ack(payload):
        return 403, {"ok": False, "error": "Set armed=true and i_understand_risk=true to start tracking."}
    if not TRACKING_ENABLED:
        return 409, {"ok": False, "error": "Tracking is disabled (TRACKING_ENABLED=0)."}
    with self.command_lock:
        if self.track_thread is not None and self.track_thread.is_alive():
            return 409, {"ok": False, "error": "A tracking session is already running."}
        if self.replay_thread is not None and self.replay_thread.is_alive():
            return 409, {"ok": False, "error": "An arm replay is running; stop it first."}
        if self.wrist_publisher is None:
            return 503, {"ok": False, "error": "DDS arm_sdk publisher is not available."}
    ok, error = self._suspend_xr_motion_publishers()
    if not ok:
        return 503, {"ok": False, "error": f"Could not suspend XR publishers: {error}"}
    cancel = threading.Event()
    thread = threading.Thread(target=self._run_tracking, args=(cancel,), name="person-tracking", daemon=True)
    with self.command_lock:
        self.track_cancel = cancel
        self.track_thread = thread
    self._set_track_status(active=True, phase="starting", failures=0, message="Tracking session starting.")
    self.record_command_event("track_start", {"source": payload.get("source", "http")})
    thread.start()
    return 200, {"ok": True, "tracking": self.track_snapshot()}
```

NOTE: check `_suspend_xr_motion_publishers`'s real return shape at
server.py:3324 before writing the `ok, error` unpack — adapt to what it
actually returns.

- [ ] **Step 5: Implement `_run_tracking` (the loop)**

```python
def _run_tracking(self, cancel: threading.Event) -> None:
    import urllib.request

    mapper = tracking.PointingMapper()
    limiter = tracking.RateLimiter(max_step_rad_s=0.35)
    smoother = tracking.Smoother(alpha=0.35)
    state = tracking.TrackState(stale_after_s=1.5, hold_s=2.0, max_failures=10)
    period = 1.0 / TRACKING_RATE_HZ
    started = time.time()
    current: dict[int, float] = dict(tracking.NEUTRAL_TEMPLATE)
    try:
        while not cancel.is_set():
            tick = time.time()
            if tick - started > TRACKING_MAX_SESSION_S:
                self._set_track_status(message="Session ceiling reached; stopping.")
                break
            frame = self.get_camera_frame() if TRACKING_CAMERA == "head" else self.get_webcam_frame()
            # get_camera_frame returns the latest JPEG bytes (see
            # server.py:5299-5352 for the exact accessor names/shape —
            # adapt if the accessor returns (bytes, ts)).
            persons: list[dict[str, Any]] | None = None
            if frame:
                try:
                    req = urllib.request.Request(
                        TRACKING_DETECT_URL, data=frame,
                        headers={"Content-Type": "image/jpeg"}, method="POST",
                    )
                    with urllib.request.urlopen(req, timeout=0.5) as resp:
                        persons = json.loads(resp.read()).get("persons", [])
                except Exception:
                    persons = None
            now = time.time()
            if persons is None:
                state.on_failure(now)
            else:
                state.on_detection(persons, now)

            if state.phase == "aborted":
                self._set_track_status(message="Detection service failing repeatedly; aborting.")
                break
            if state.phase == "tracking" and state.target is not None:
                # Aim at the upper third of the box (chest height).
                aim_cy = state.target["y1"] + (state.target["y2"] - state.target["y1"]) / 3.0
                goal = mapper.targets(state.target["cx"], aim_cy)
            elif state.phase == "hold":
                goal = dict(current)
            else:  # stale
                goal = dict(tracking.NEUTRAL_TEMPLATE)

            goal = smoother.update(goal)
            current = limiter.step(current, goal, dt=period)

            with self.command_lock:
                msg = self.lowstate_msg
                publisher = self.wrist_publisher
            # Spec: never publish against stale robot state. on_lowstate
            # stores snapshots at 30 Hz — find the timestamp it records
            # (server.py:5386-5407) and skip publishing when it is older
            # than 0.5 s.
            if msg is not None and publisher is not None:
                gains = {j: (float(kp), float(kd)) for j, kp, kd in zip(ARM_SDK_JOINTS, ARM_SDK_KP, ARM_SDK_KD)}
                cmd = self._build_arm_sdk_trajectory_cmd(msg, dict(current), gains)
                publisher.Write(cmd)

            age = None if state.last_seen is None else round(now - state.last_seen, 2)
            self._set_track_status(
                phase=state.phase, failures=state.failures, detection_age_s=age,
                target=state.target, message=f"Tracking loop running ({state.phase}).",
            )
            cancel.wait(max(0.0, period - (time.time() - tick)))
    finally:
        # Ramp handled by publishing NEUTRAL_TEMPLATE above on stale; on exit
        # just mark inactive — arms hold last commanded pose under arm_sdk
        # weight, same convention as replay end.
        self._set_track_status(active=False, phase="idle", message="Tracking session ended.")
        with self.command_lock:
            self.track_thread = None
            self.track_cancel = None
```

IMPORTANT adaptation notes for the implementer (verify against real code,
do not guess): (a) the exact accessor for the head-camera JPEG buffer
(server.py:5299-5352), (b) whether `publisher.Write(cmd)` is the arm_sdk
publish call used by `run_replay` (server.py:2740-2956) — mirror the replay
loop's publish idiom exactly, including any weight ramp-in it performs,
(c) `_suspend_xr_motion_publishers` return shape. Publishing at 8 Hz with
smoothed setpoints is intentional: arm_sdk holds position between commands.

- [ ] **Step 6: Run the tests**

Run: `python3 -m unittest tests.test_tracking_endpoints tests.test_tracking_math -v`
Expected: all PASS

- [ ] **Step 7: Commit**

```bash
git add server.py tests/test_tracking_endpoints.py
git commit -m "Add guarded person-tracking session (TrackingController) to TelemetryStore

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git push
```

---

### Task 4: HTTP endpoints `/api/track/*`

**Files:**
- Modify: `server.py` — POST dispatch block (near `/api/robot/home` at line ~5713) and the GET dispatch (near `/api/camera` at ~5513).
- Test: extend `tests/test_tracking_endpoints.py`

**Interfaces:**
- Consumes: Task 3 methods.
- Produces: `POST /api/track/start` (risk-ack body), `POST /api/track/stop`, `GET /api/track/status` → `{"ok": true, "tracking": {...track_snapshot...}}` — consumed by the UI card (Task 6).

- [ ] **Step 1: Write the failing HTTP-level test**

Follow the idiom of the existing endpoint tests (see how `tests/test_smartplug.py` or `tests/test_contracts.py` exercise the handler — reuse their helper; if they call store methods directly, test the new dispatch by asserting the route constants exist):

```python
class TrackingRouteTests(unittest.TestCase):
    def test_routes_are_dispatched(self):
        src = open("server.py").read()
        self.assertIn('"/api/track/start"', src)
        self.assertIn('"/api/track/stop"', src)
        self.assertIn('"/api/track/status"', src)
```

(Byte-level route assertion is deliberately crude but offline-safe; the
store-level behavior is already covered by Task 3's tests.)

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.test_tracking_endpoints -v`
Expected: FAIL on the three `assertIn`s

- [ ] **Step 3: Wire the routes**

In the POST dispatcher, after the `/api/robot/home` block:

```python
if request_path == "/api/track/start":
    status, response = self.store.request_track_start(payload)
    self._send_json_status(response, HTTPStatus(status))
    return

if request_path == "/api/track/stop":
    status, response = self.store.request_track_stop()
    self._send_json_status(response, HTTPStatus(status))
    return
```

In the GET dispatcher, next to `/api/camera`:

```python
if request_path == "/api/track/status":
    self._send_json({"ok": True, "tracking": self.store.track_snapshot()})
    return
```

(Match the surrounding code's exact `_send_json` / `_send_json_status` usage.)

- [ ] **Step 4: Run tests + gate**

Run: `python3 -m unittest discover -s tests -p 'test_*.py' && make production-gate`
Expected: all PASS, `Production gate passed.`

- [ ] **Step 5: Commit**

```bash
git add server.py tests/test_tracking_endpoints.py
git commit -m "Expose /api/track/start|stop|status endpoints

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git push
```

---

### Task 5: Chat/MCP tool `track_person`

**Files:**
- Modify: `server.py` — `CHAT_TOOL_SPECS` area (line ~610), `chat_tool_specs` (~3945), `run_chat_tool` (~4046).
- Test: extend `tests/test_tracking_endpoints.py` (tool tests mirror `tests/test_chat.py` idioms)

**Interfaces:**
- Consumes: Task 3 methods; existing `_chat_tool` builder, `chat_tool_specs` assembly, `run_chat_tool` dispatch (MCP exposure is automatic via `mcp_tool_descriptors`).
- Produces: chat/MCP tool `track_person(action: "start"|"stop", confirm: true)`.

- [ ] **Step 1: Write failing tests**

```python
class TrackToolTests(unittest.TestCase):
    def make_store(self):
        return server.TelemetryStore(robot_host="127.0.0.1")

    def test_tool_hidden_when_disabled(self):
        old = server.LLM_TOOL_TRACK_ENABLED
        server.LLM_TOOL_TRACK_ENABLED = False
        try:
            specs = self.make_store().chat_tool_specs()
        finally:
            server.LLM_TOOL_TRACK_ENABLED = old
        names = [s["function"]["name"] for s in specs]
        self.assertNotIn("track_person", names)

    def test_tool_requires_confirm(self):
        old = server.LLM_TOOL_TRACK_ENABLED
        server.LLM_TOOL_TRACK_ENABLED = True
        try:
            result = self.make_store().run_chat_tool("track_person", {"action": "start"})
        finally:
            server.LLM_TOOL_TRACK_ENABLED = old
        self.assertFalse(result["ok"])
        self.assertIn("confirm", result["error"])

    def test_tool_stop_works_offline(self):
        old = server.LLM_TOOL_TRACK_ENABLED
        server.LLM_TOOL_TRACK_ENABLED = True
        try:
            result = self.make_store().run_chat_tool(
                "track_person", {"action": "stop", "confirm": True}
            )
        finally:
            server.LLM_TOOL_TRACK_ENABLED = old
        self.assertTrue(result["ok"])
```

- [ ] **Step 2: Run to verify failure**

Run: `python3 -m unittest tests.test_tracking_endpoints -v`
Expected: FAIL (`track_person` unknown tool / spec missing)

- [ ] **Step 3: Implement**

Spec builder next to `move_tool_spec` (server.py:617):

```python
def track_tool_spec() -> dict[str, Any]:
    return _chat_tool(
        "track_person",
        "GUARDED ACTION: start or stop the person-tracking mode where the robot "
        "continuously points its right arm at the person seen in the head camera. "
        "Only call when the operator's latest message explicitly asks to start or "
        "stop tracking/pointing (Turkish: 'beni takip et' / 'takibi durdur').",
        {
            "action": {"type": "string", "enum": ["start", "stop"],
                        "description": "start or stop the tracking session."},
            "confirm": {"type": "boolean",
                         "description": "Must be true; confirms the operator explicitly asked."},
        },
        ["action", "confirm"],
    )
```

In `chat_tool_specs` (server.py:3945), after the `move` append:

```python
if LLM_TOOL_TRACK_ENABLED and TRACKING_ENABLED:
    specs.append(track_tool_spec())
```

In `run_chat_tool` (server.py:4046), before the unknown-tool fallthrough:

```python
if name == "track_person":
    return self._tool_track(arguments)
```

Handler next to `_tool_move` (server.py:4088):

```python
def _tool_track(self, arguments: dict[str, Any]) -> dict[str, Any]:
    if not LLM_TOOL_TRACK_ENABLED:
        return {"ok": False, "error": "The track_person tool is disabled (LLM_TOOL_TRACK_ENABLED=0)."}
    if arguments.get("confirm") is not True:
        return {"ok": False, "error": "Refused: confirm must be true (operator must have explicitly asked)."}
    action = arguments.get("action")
    if action == "stop":
        status, result = self.request_track_stop()
    elif action == "start":
        # Chat-initiated start carries the risk ack implicitly: the tool is
        # double-gated by LLM_TOOL_TRACK_ENABLED + confirm, mirroring _tool_move.
        status, result = self.request_track_start(
            {"armed": True, "i_understand_risk": True, "source": "chat"}
        )
    else:
        return {"ok": False, "error": "action must be 'start' or 'stop'."}
    self.record_command_event("chat_track", {"source": "chat", "action": action, "status": status})
    return {"ok": status < 400 and bool(result.get("ok")), "status": status, "result": result}
```

- [ ] **Step 4: Run tests + gate**

Run: `python3 -m unittest discover -s tests -p 'test_*.py' && make production-gate`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add server.py tests/test_tracking_endpoints.py
git commit -m "Add track_person chat/MCP tool (start/stop person tracking)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git push
```

---

### Task 6: Dashboard "Person Tracking" card

**Files:**
- Modify: `static/index.html` — new card next to the wrist/loco cards
- Modify: `static/app.js` — status polling + start/stop handlers

**Interfaces:**
- Consumes: `/api/track/start|stop|status` (Task 4).
- Produces: operator UI. No JS module exports.

There is no JS test harness; `node --check` in the production gate is the
syntax check. Mirror the existing wrist-card idioms exactly (same checkbox
risk-ack pattern, same fetch wrapper, same status-badge classes) — open the
wrist card markup/JS first and copy its structure.

- [ ] **Step 1: Add the card markup to `static/index.html`**

Adapt class names to the wrist card's exact classes:

```html
<section class="card" id="tracking-card" hidden>
  <h2>Person Tracking</h2>
  <p class="card-note">The robot points its right arm at the person seen in the
  head camera. Detection runs on the AI host; if detections stall the arm
  returns to neutral.</p>
  <div class="status-row">
    <span id="tracking-phase" class="badge">idle</span>
    <span id="tracking-detail"></span>
  </div>
  <label><input type="checkbox" id="tracking-armed"> Armed</label>
  <label><input type="checkbox" id="tracking-risk"> I understand the robot will move</label>
  <div class="button-row">
    <button id="tracking-start" disabled>Start tracking</button>
    <button id="tracking-stop">Stop</button>
  </div>
</section>
```

- [ ] **Step 2: Add the logic to `static/app.js`**

```javascript
// --- Person tracking card -------------------------------------------------
function initTrackingCard() {
  const card = document.getElementById("tracking-card");
  if (!card) return;
  const phase = document.getElementById("tracking-phase");
  const detail = document.getElementById("tracking-detail");
  const armed = document.getElementById("tracking-armed");
  const risk = document.getElementById("tracking-risk");
  const startBtn = document.getElementById("tracking-start");
  const stopBtn = document.getElementById("tracking-stop");

  function syncButtons() {
    startBtn.disabled = !(armed.checked && risk.checked);
  }
  armed.addEventListener("change", syncButtons);
  risk.addEventListener("change", syncButtons);

  async function refresh() {
    try {
      const res = await fetch("/api/track/status");
      const data = await res.json();
      const t = data.tracking || {};
      card.hidden = !t.enabled;
      phase.textContent = t.phase || "idle";
      const age = t.detection_age_s == null ? "-" : `${t.detection_age_s}s`;
      detail.textContent = `${t.message || ""} (detection age: ${age})`;
    } catch (err) {
      detail.textContent = "Status unavailable.";
    }
  }

  startBtn.addEventListener("click", async () => {
    await fetch("/api/track/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ armed: true, i_understand_risk: true }),
    });
    refresh();
  });
  stopBtn.addEventListener("click", async () => {
    await fetch("/api/track/stop", { method: "POST" });
    refresh();
  });

  refresh();
  setInterval(refresh, 2000);
}
initTrackingCard();
```

Match the file's existing fetch/error idioms; if app.js has a shared
`postJson` helper or visibility-aware polling (it does — the performance
passes added idle-tab throttling), use those instead of raw `fetch`/
`setInterval`.

- [ ] **Step 3: Syntax check + gate**

Run: `node --check static/app.js && make production-gate`
Expected: clean, `Production gate passed.`

- [ ] **Step 4: Commit**

```bash
git add static/index.html static/app.js
git commit -m "Add Person Tracking card (start/stop + live phase) to the dashboard

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git push
```

---

### Task 7: On-robot bring-up (manual, operator present)

**Files:**
- Modify: `docs/superpowers/specs/2026-07-21-person-pointing-design.md` — record calibration results
- Modify (robot, not repo): `robot-telemetry-web.service` environment

Robot was powered off during planning — every step here needs the robot on,
an operator physically present, and clear space around the right arm.

- [ ] **Step 1: Network + latency check**

From the robot PC (`ssh unitree@192.168.123.164`):

```bash
curl -s http://10.2.125.3:8188/health           # expect {"ok": true}
# round-trip with a real head-camera frame:
curl -s -o /dev/null -w '%{time_total}\n' -X POST \
  --data-binary @<(curl -s http://127.0.0.1:8088/camera.mjpg | head -c 200000) \
  http://10.2.125.3:8188/detect
```

Expect total time well under 0.25 s over Wi-Fi. Record the number.

- [ ] **Step 2: Enable dark, verify status-only**

Add to the service environment (`systemctl --user edit robot-telemetry-web.service`): `TRACKING_ENABLED=1`, leave `LLM_TOOL_TRACK_ENABLED=0`. Restart, open the dashboard, start tracking **with the arm area clear**: verify `/api/track/status` cycles `tracking/hold/stale` phases correctly as people enter/leave the camera view. The arm WILL move in this step — that is the point; keep the e-stop/chill button at hand.

- [ ] **Step 3: Calibrate FOV constants**

Stand at known positions (image left edge, center, right edge); adjust `fov_yaw_rad` / `yaw_offset` (and pitch equivalents) via env overrides until the arm visibly points at the person at all three positions. Commit the calibrated defaults into `tracking.py`.

- [ ] **Step 4: Failure drills**

With tracking running: (a) unplug the AI-host path (block port 8188) → arm must ramp to neutral within ~2 s and session must abort after 10 failures; (b) walk out of frame → hold 2 s then neutral; (c) `POST /api/track/stop` mid-motion → immediate stop. Record all three outcomes in the spec doc.

- [ ] **Step 5: Enable the chat tool + final commit**

Set `LLM_TOOL_TRACK_ENABLED=1`, test "beni takip et" / "takibi durdur" in chat, then commit the calibration + bring-up notes:

```bash
git add tracking.py docs/superpowers/specs/2026-07-21-person-pointing-design.md
git commit -m "Calibrate tracking FOV constants from on-robot bring-up; record failure drills

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git push
```
