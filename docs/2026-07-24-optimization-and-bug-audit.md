# 2026-07-24 — Optimization & correctness audit (unattended session)

All work below is on branch `feature/llm-pose-proposals` only. **Nothing was merged
to `main`, so nothing has reached the robot** — this is staged for review. Every
commit passed the full unit suite (273 tests) + `make production-gate`.

## Part 1 — Performance / resource optimizations (applied)

- **viewer.js:** render-on-demand (only redraw the 3–4 models when a joint/camera/
  visibility changed; a 400 ms backstop guarantees it can never freeze), reused a
  scratch quaternion (was ~100 allocations/tick), skip idle joints, no shadow pass
  on the transparent ghosts, and dropped a wasted render+JPEG-encode running 2×/s.
- **server.py:** cached `network_status()` (was forking `ip route get` + reading
  /proc + opening a UDP socket on *every* SSE tick × client); build the DDS record
  only while recording (was building it hundreds of times/s and discarding it);
  trimmed `wireless_remote` + unused battery fields from the 5 Hz snapshot; removed
  ~156 lines of dead camera code + an unused constant.
- **app.js:** skip 5 Hz DOM rebuilds when values are unchanged (dashboard/motor/
  network panels were reparsing HTML every 200 ms); gated background polls when the
  tab is hidden; friendlier "AI server unreachable" message; removed dead `setFields`.

## Part 2 — Safety / correctness bugs FIXED (with tests)

1. **Non-finite replay angle** — a NaN/inf q in a recording passed every velocity/
   delta check (`NaN > x` is False) then clamped to the joint limit (a large
   unvalidated move). Now flagged as a `non_finite` violation → not executable.
2. **Wrist oscillate past limits** — the oscillate setpoint (`center + amp·sin`)
   was validated only on amplitude and published unclamped; near an extreme it
   exceeded `WRIST_LIMITS` / the hardware joint limit. Final angle now clamped.
3. **Emergency limp didn't stop tracking** — `chill_motors` never cancelled a
   person-tracking session, so the arms kept tracking after "go limp". It now
   stops tracking on every exit path.
4. **Replay 4× over-speed** — the velocity gate validated native timing (≤2 rad/s)
   but execution sleeps `dt/playback_speed`, so the `replay_response` dial drove
   setpoints up to ~4× faster (~8 rad/s). Effective speed is now capped so
   `native_velocity·speed` (and the 0.6 rad/s approach) stay within the limit.
5. **Loco mobility unarmed** — `command_loco` had no risk-ack gate, so a raw/MCP/
   curl caller could walk/translate the robot unarmed. Mobility actions now require
   `armed`+`i_understand_risk`; stops/posture/read-only stay ungated. (UI already
   sends the flags, so it's unaffected.)
6. **read_only_proxy SSRF** — `urljoin(upstream, self.path)` let an absolute-form or
   protocol-relative request target override the upstream host on the internet-
   facing tunnel. Now only a plain absolute path is forwarded to the fixed upstream.
7. **claude_bridge** prints a loud warning when it binds a non-loopback host with no
   token (unauthenticated LAN access to the operator's billed CLI).

## Part 3 — Findings NOT fixed (need your review / hardware testing)

These are real but I judged them too risky to change unattended without being able
to test on the robot. Recommend fixing together, on hardware, next session.

- **TOCTOU: two concurrent replays / two concurrent track-starts.** In both
  `execute_arm_sdk_replay` and `request_track_start`, the "already running?" check
  and the register-new-thread step are under *separate* `command_lock` acquisitions
  with slow work (XR suspend, thread creation) in between. Two near-simultaneous
  requests (double-click / two clients) can both start a thread → two arm_sdk
  publishers fight the arms, and the first becomes an un-cancellable orphan.
  *Proposed fix:* an atomic "starting" claim set under the existing lock at the
  check, cleared in a `try/finally` around the whole start — but every early-return
  path must clear it or the feature is permanently blocked, so it needs careful
  review + a hardware smoke test.
- **Sentry↔tracking desync race.** `request_track_start` reads `sentry_mode_on`
  then releases the lock; a concurrent `sentry off` can slip in, leaving a tracking
  session running with sentry off. Same lock-window root cause as above; fix
  together.
- **Editor torso clamp mismatch (cosmetic/safety-adjacent).** The 3D editor lets
  `WaistYaw` reach the URDF ±2.35 while the server re-clamps to ±1.2, so the robot's
  torso goes to a different angle than the editor shows. The robot is *safe* (server
  clamps); this is a display/consistency fix — clamp `WaistYaw` to ±1.2 in the
  editor emit path.
- **Editor "sync" overwrites a drag.** After dragging an arm, a `"sync"` re-emit
  (tab switch / viewer re-ready) can replace `state.editedPose` with the loaded file
  frame, so Move sends the file pose instead of your edit. Front-end state-management
  fix; needs a browser to verify.
- **MCP guarded-action authorization is caller-asserted.** Over `/mcp`, `confirm:true`
  is supplied by the (untrusted) client, and `MCP_TOKEN` defaults empty — so if MCP
  is enabled without a token, `/mcp` is an unauthenticated remote motion interface
  (`move home`, `chill`, staged `move proposed`). MCP is **off by default**, so this
  is latent. *Recommend:* when `MCP_ENABLED` and no `MCP_TOKEN`, either refuse to
  start or expose only read-only tools (strip move/chill/track from the MCP tool
  list). A design decision — left for you.

## Part 4 — Deferred (your call)

- **~41 MB unused vendor model files** under `static/models/h1_2_description/`
  (96 meshes + variant .urdf/.xml the web app never loads). It's a *monorepo* (other
  subsystems have their own copies), it's disk-only not runtime, and untestable with
  the robot off — so left for you to approve. Exact command available on request.
