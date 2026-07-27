---
tags: [audit, known-issues, safety, optimization, performance, toctou, technical-debt, production]
summary: The living known-issues / audit page — what the 2026-07-24 optimization + bug audit fixed (with safety rationale), what stays open pending hardware, plus the production-refinement loop that governs how fixes land.
---

# 25 - Known Issues & Optimization Audit

> [!abstract] Goal
> One place to track the robot's outstanding correctness and performance issues:
> what has been **fixed** and why it mattered for safety, what is **still open**
> and needs the physical robot to verify, and the disciplined loop we use to land
> each fix without regressing behavior. This is a *living* page — update it as
> deferred findings get resolved on hardware.

Sources: `docs/2026-07-24-optimization-and-bug-audit.md` (the unattended
optimization + correctness audit on `feature/llm-pose-proposals`),
`docs/production_refinement.md` (the production-hardening loop + gate).

> [!warning] Staged, not shipped
> The entire 2026-07-24 audit lives on branch `feature/llm-pose-proposals`.
> **Nothing from it was merged to `main`, so nothing has reached the robot** — it
> is staged for review. Every audit commit passed the full unit suite (273 tests)
> and `make production-gate`, but the *open* findings below still need a hardware
> smoke test before they are safe to merge. Treat all "FIXED" rows as
> *fixed-in-branch, pending merge*.

## The production-refinement loop (how fixes land)

Per `docs/production_refinement.md`, every improvement follows a
behavior-preserving, one-slice-at-a-time loop backed by an offline gate:

```bash
make production-gate        # offline: py syntax, tests/, shell + JS syntax
python3 scripts/production_gate.py --live   # only when the robot PC is reachable
```

| Rule | Why |
| --- | --- |
| Run the gate **before and after** every edit; baseline first | Catch regressions per slice, not per session |
| One narrow slice per commit (one function / script / panel) | Reviewable, revertable, matches the repo's commit-per-change rule (08 - Development Workflow) |
| Offline gate must **never** need robot access or publish commands | Keeps CI safe; live checks stay separate |
| Never make command endpoints easier to trigger | Preserve the interlocks (03 - Safety Interlocks) |
| Keep `armed=true` + `i_understand_risk=true` in front of command execution | The core motion gate (16 - Arm Control & Command Surfaces) |
| Add/update **contract tests before** changing command, DDS, systemd, or teleop behavior | Characterize current behavior before touching it (10 - Testing) |

## Performance / resource optimizations (applied)

Behavior-preserving cuts to per-tick and per-client cost across the three hot
files:

| Area | Change | Effect |
| --- | --- | --- |
| `static/viewer.js` | Render-on-demand (redraw only on joint/camera/visibility change, 400 ms freeze-proof backstop); reused a scratch quaternion; skip idle joints; no shadow pass on transparent ghosts; dropped a wasted 2 Hz render+JPEG encode | ~100 allocations/tick removed; no idle redraws (17 - 3D URDF Viewer) |
| `server.py` | Cached `network_status()` (was forking `ip route get` + reading `/proc` + opening a UDP socket **per SSE tick × client**); build the DDS record only while recording; trimmed `wireless_remote` + unused battery fields from the 5 Hz snapshot; removed ~156 lines of dead camera code | Big drop in per-client SSE cost (01 - Architecture, 13 - Telemetry Recording & Pose Editor) |
| `static/app.js` | Skip 5 Hz DOM rebuilds when values unchanged; gate background polls when the tab is hidden; friendlier "AI server unreachable" message; removed dead `setFields` | No 200 ms HTML reparse when nothing changed |

## Safety / correctness bugs FIXED (with tests)

Each row shipped with a regression test. The **safety rationale** is why it
mattered on a machine that can move.

| # | Bug | Safety rationale | Fix |
| --- | --- | --- | --- |
| 1 | **Non-finite replay angle** — a `NaN`/`inf` `q` passed every velocity/delta check (`NaN > x` is `False`) then clamped to the joint limit | A single bad sample became a large **unvalidated arm move** | Flagged as a `non_finite` violation → not executable (14 - Recording Replay & Digital Twin) |
| 2 | **Wrist oscillate past limits** — setpoint `center + amp·sin` validated only on amplitude, published unclamped | Near an extreme it **exceeded `WRIST_LIMITS`** / the hardware joint limit | Final angle clamped before publish (16 - Arm Control & Command Surfaces) |
| 3 | **Emergency limp didn't stop tracking** — `chill_motors` never cancelled a person-tracking session | Arms **kept tracking after "go limp"** — the opposite of an e-stop | Stops tracking on every exit path (03 - Safety Interlocks, 06 - Person Tracking (CV Feature)) |
| 4 | **Replay 4× over-speed** — velocity gate validated native timing (≤2 rad/s) but execution sleeps `dt/playback_speed` | `replay_response` dial drove setpoints up to **~4× faster (~8 rad/s)** | Effective speed capped so `native_velocity·speed` (and the 0.6 rad/s approach) stay within limit (14 - Recording Replay & Digital Twin, 24 - Control Gains, PID & Shared Mechanisms) |
| 5 | **Loco mobility unarmed** — `command_loco` had no risk-ack gate | A raw/MCP/curl caller could **walk/translate the robot unarmed** | Mobility actions now require `armed`+`i_understand_risk`; stops/posture/read-only stay ungated (UI already sends flags) (03 - Safety Interlocks) |
| 6 | **`read_only_proxy` SSRF** — `urljoin(upstream, self.path)` let an absolute-form / protocol-relative request override the upstream host | On the internet-facing tunnel, a request could **target an arbitrary host** | Only a plain absolute path is forwarded to the fixed upstream (02 - Network & Hosts) |
| 7 | **`claude_bridge` silent LAN exposure** | Unauthenticated LAN access to the operator's **billed CLI** | Loud warning when binding a non-loopback host with no token |
| 8 | **Malformed/partial recording crashed replay** — a truncated trailing JSONL line (common when replaying a file still being recorded) or invalid JSON threw out of the handler | Dropped connection mid-operation | Skipped/handled gracefully (13 - Telemetry Recording & Pose Editor) |
| 9 | **Non-numeric motor `q`/`index` crashed replay planning** — save path accepted data the read path couldn't survive (`int(None)`) | Crash instead of clean rejection | Coerced safely; plan rejected as `malformed_motor` violation |
| 10 | **Ephemeral temp-file collision** — two filename-less replays could share a `monotonic_ns` temp path | One replay could **execute the other's trajectory** | Unique path (pid + counter + `O_EXCL`) |
| 11 | **Sentry-stream worker race** — `is_alive()` window could leave clients>0 with no worker | Detection stalled until another client connected | Explicit run flag + `try/finally` (19 - Sentry Mode & Head-Lock) |
| 12 | **SSE/MJPEG loops** didn't catch `OSError`/`ssl.SSLError` | A dropped TLS client caused an **unhandled-thread crash** | Loops now exit cleanly on `OSError` |

> [!note] Path containment audited clean
> The recording path-containment check (`../`, absolute paths, symlink, null-byte)
> was audited and confirmed **airtight** — no traversal bug in the
> `/api/recording/files/<name>` download path (13 - Telemetry Recording & Pose Editor).

## Still OPEN — needs hardware to verify

> [!warning] These are real findings deliberately left unfixed
> They were judged too risky to change unattended without being able to test on
> the robot. Recommend fixing them **together, on hardware, next session**.

### Concurrency / TOCTOU races

| Finding | Risk | Proposed fix |
| --- | --- | --- |
| **Two concurrent replays / two concurrent track-starts** | In `execute_arm_sdk_replay` and `request_track_start`, the "already running?" check and the register-new-thread step are under **separate** `command_lock` acquisitions with slow work (XR suspend, thread creation) between them. Two near-simultaneous requests (double-click / two clients) can both start a thread → **two arm_sdk publishers fight the arms**, and the first becomes an un-cancellable orphan | Atomic "starting" claim set under the existing lock at the check, cleared in a `try/finally` around the whole start — but **every early-return path must clear it** or the feature is permanently blocked; needs careful review + a hardware smoke test |
| **Sentry↔tracking desync race** | `request_track_start` reads `sentry_mode_on` then releases the lock; a concurrent `sentry off` slips in, leaving a **tracking session running with sentry off** — violates the sentry-is-the-master-switch invariant (19 - Sentry Mode & Head-Lock) | Same lock-window root cause; fix together with the above |

### Editor / front-end consistency

| Finding | Risk | Proposed fix |
| --- | --- | --- |
| **Editor torso clamp mismatch** (cosmetic/safety-adjacent) | 3D editor lets `WaistYaw` reach URDF ±2.35 while the server re-clamps to ±1.2 → the torso goes to a **different angle than the editor shows**. Robot is *safe* (server clamps); display-only inconsistency | Clamp `WaistYaw` to ±1.2 in the editor emit path (17 - 3D URDF Viewer) |
| **Editor "sync" overwrites a drag** | After dragging an arm, a `"sync"` re-emit (tab switch / viewer re-ready) can replace `state.editedPose` with the loaded-file frame → **Move sends the file pose, not your edit** | Front-end state-management fix; needs a browser to verify (13 - Telemetry Recording & Pose Editor) |

### MCP authorization

| Finding | Risk | Proposed fix |
| --- | --- | --- |
| **MCP guarded-action auth is caller-asserted** | Over `/mcp`, `confirm:true` is supplied by the **untrusted client** and `MCP_TOKEN` defaults empty — so if MCP is enabled without a token, `/mcp` is an **unauthenticated remote motion interface** (`move home`, `chill`, staged `move proposed`). MCP is **off by default**, so this is latent | When `MCP_ENABLED` and no `MCP_TOKEN`: refuse to start, or expose only read-only tools (strip move/chill/track from the MCP tool list). A design decision (05 - Chat & MCP Tools) |

### Camera bridge robustness

| Finding | Risk | Proposed fix |
| --- | --- | --- |
| **Camera bridge never restarted/reaped on crash** | If the ROS2 camera subprocess dies, nothing polls/restarts it — the feed **permanently stops** ("Waiting for camera bridge frame") and the child isn't reaped until the parent exits. Camera-only, **not a safety path** | In the camera file-watcher, `poll()` the process and restart with exponential backoff (+ a failure ceiling to avoid a restart-loop on a persistent bad config); `wait()` after `terminate()` on shutdown. Left undone: subprocess-restart logic is risky without a real camera to test the loop |
| **Decoder temp `.h264` file** (`/tmp/…_bridge_<pid>.h264`) not unlinked on exit | Bounded, up-to-4 MB stale file per bridge PID | Trivial — unlink in a `finally` |

## Deferred (operator's call)

| Item | Detail |
| --- | --- |
| **~41 MB unused vendor model files** under `static/models/h1_2_description/` | 96 meshes + variant `.urdf`/`.xml` the web app never loads. It's a *monorepo* (other subsystems keep their own copies), disk-only not runtime, and untestable with the robot off — left for operator approval. Exact removal command available on request |

