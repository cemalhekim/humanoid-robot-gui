---
tags: [safety, interlocks, critical, joint-limits, arm_sdk, risk-ack]
summary: Safety-critical layers — risk-ack gating, JOINT_LIMITS clamping, XR suspension, cancel-Event mutual exclusion, session ceilings, staleness→neutral.
---

# 03 - Safety Interlocks

> [!danger] Safety-critical
> This code moves a **real** Unitree H1-2. CLAUDE.md is explicit: *"keep all
> safety interlocks intact and run the tests before pushing."* Never weaken
> these layers. The person-tracking plan repeats the same rule as a global
> constraint. All symbol names below are from `server.py` unless noted.

## Layer 1 — Operator intent: `has_risk_ack`

Motion-related endpoints require **explicit dual acknowledgement** in the request body:

```python
def has_risk_ack(payload):
    return payload.get("armed") is True and payload.get("i_understand_risk") is True
```

- Both `armed=true` **and** `i_understand_risk=true` are required (strict `is True`).
- `command_wrist` (`/api/wrist/command`) returns **400** without it.
- The dashboard mirrors this as two checkboxes on the wrist card; the planned
  [[06 - Person Tracking (CV Feature)|tracking card]] reuses the same pattern and gates
  `/api/track/start` on `has_risk_ack` (**403** without it).
- Chat/[[05 - Chat & MCP Tools|MCP]] actions carry the ack implicitly but are
  double-gated by their feature flag **and** a required `confirm=true`.

## Layer 2 — `JOINT_LIMITS` clamping

Every commanded joint target is clamped by `_clamp_joint_target(joint, q)`
against `JOINT_LIMITS` before publishing. Arm/waist limits (radians):

| Joint | Name | Limit (lo, hi) |
| --- | --- | --- |
| 12 | WaistYaw | (-1.2, 1.2) — conservative; mechanical is ±2.35 |
| 13 | LeftShoulderPitch | (-3.14, 1.57) |
| 14 | LeftShoulderRoll | (-0.38, 3.4) |
| 15 | LeftShoulderYaw | (-2.66, 3.01) |
| 16 | LeftElbow | (-0.95, 3.18) |
| 17 | LeftWristRoll | (-3.01, 2.75) |
| 18 | LeftWristPitch | (-0.4625, 0.4625) |
| 19 | LeftWristYaw | (-1.27, 1.27) |
| 20 | RightShoulderPitch | (-3.14, 1.57) |
| 21 | RightShoulderRoll | (-3.4, 0.38) |
| 22 | RightShoulderYaw | (-3.01, 2.66) |
| 23 | RightElbow | (-0.95, 3.18) |
| 24 | RightWristRoll | (-2.75, 3.01) |
| 25 | RightWristPitch | (-0.4625, 0.4625) |
| 26 | RightWristYaw | (-1.27, 1.27) |

- `WRIST_LIMITS = (-1.2, 1.2)` for the standalone wrist path.
- The [[06 - Person Tracking (CV Feature)|tracking module]] `tracking.py` uses a
  **tighter** `TRACK_LIMITS` envelope (e.g. RightShoulderYaw `(-1.0, 1.0)` vs
  JOINT_LIMITS `(-3.01, 2.66)`) — and `server.py` **re-clamps against
  `JOINT_LIMITS` anyway** (defense in depth).

## arm_sdk joint set

The H1-2 **arm_sdk** path can only drive joints **13–26 plus 12**:

```python
ARM_SDK_JOINTS = [13,14,15,16,17,18,19, 20,21,22,23,24,25,26, 12]
ARM_SDK_KP     = [120,120,80,50,50,50,50, 120,120,80,50,50,50,50, 200]
ARM_SDK_KD     = [2.0,2.0,1.5,1.0,1.0,1.0,1.0, 2.0,2.0,1.5,1.0,1.0,1.0,1.0, 2.0]
ARM_SDK_WEIGHT_SLOT = 27   # arm_sdk enable weight
```

> [!note] Waist is special
> `WaistYaw` (joint 12) is **not** part of the real H1-2 arm_sdk set
> (`H1_2_JointArmIndex` is 13–26 only). It is driven separately via `rt/lowcmd`
> commanding **only** joint 12 (legs + arms get `mode=0` / no signal, arms keep
> running on arm_sdk). This avoids a motion-mode release. Waist lowcmd caps:
> `WAIST_LOWCMD_KP=200`, `WAIST_LOWCMD_KD=5`, `WAIST_LOWCMD_MAX_VEL_RAD_S=0.6`.

The arm controller runs **closed-loop at 120 Hz** with PID + gravity
feed-forward (`ARM_REPLAY_HOLD_HZ = 120.0`). Gravity feed-forward is built from
**measured** torque and bounded per joint (`ARM_REPLAY_GRAVITY_TAU_LIMITS`:
shoulder 15, elbow 10, wrist 4, waist 6 Nm) so a contact spike cannot command
an unbounded push.

> [!warning] Known follow-up
> `server.py` notes feeding measured torque forward is **unsafe on hard
> contact** — model-based gravity + collision detection is the recommended
> hardware-validated follow-up.

## Layer 3 — XR publisher suspension

Before any dashboard motion (arm replay, home hold, and the planned tracking),
`_suspend_xr_motion_publishers()` stops the XR teleop motion services so they
cannot fight or overwrite the arm_sdk command:

- `systemctl --user stop --no-block` then `kill --signal=KILL` of
  `XR_MOTION_SERVICES = ("xr-home-watchdog.service", "xr-teleop.service")`.
- `pkill -f teleop_hand_and_arm.py` (`XR_TELEOP_PROCESS_PATTERN`) and `pgrep` verification.
- Returns a **dict** (`{"ok": ..., "skipped": ..., "actions": [...]}`), not a
  tuple — skippable via `RTW_SKIP_XR_SUSPEND=1` or when `systemctl` is absent (dev machines).

Replay refuses to publish if the XR teleop motion publisher is still active
("arm_sdk replay would be overwritten").

## Layer 4 — Cancel-Event mutual exclusion

Only one motion session may own the arm path at a time. `TelemetryStore` holds
`threading.Event` cancel flags under `command_lock`:

- `replay_cancel` + `replay_thread` — arm replay / home hold / torso twist session.
- `wrist_cancel` — wrist command session.
- `torso_cancel` — waist twist.

Starting a new session **sets the previous session's cancel Event first**, then
swaps in the new `cancel`/`thread`. The planned tracking session follows the
same convention: it refuses to start if a replay/wrist/home session is active
(**409**), and any new replay/wrist/home start cancels tracking first (shared
cancel-Event scheme). This prevents two controllers writing `rt/arm_sdk`
simultaneously.

## Layer 5 — Session ceilings & convergence

- **Arm replay absolute ceiling**: `ARM_REPLAY_ABSOLUTE_CEILING_SECONDS = 90.0`.
  If not converged by then it enters a **FLAGGED safe-hold** (keeps holding,
  reports `converged=false`) — it **never** claims success or releases at the
  wrong pose. Convergence is the only success exit.
- Convergence requires each joint inside a band **and** nearly stationary
  (`ARM_REPLAY_CONVERGE_VELOCITY_RAD_S = 0.05`), plus a Cartesian "silhouette"
  proxy tolerance (`ARM_REPLAY_CARTESIAN_TOLERANCE_M = 0.006`).
- Planned tracking ceiling: `TRACKING_MAX_SESSION_S` default **600 s** → auto-stop.

## Layer 6 — Staleness → neutral (fail-safe)

The robot must **never keep aiming with dead data**. In the person-tracking
design (see [[06 - Person Tracking (CV Feature)]]):

- Detection age **> 1.5 s** → ramp arm to the **neutral** template pose (velocity-bounded), state `stale`.
- DDS `rt/lowstate` older than **0.5 s** → treat as stale, do **not** publish blindly.
- **10 consecutive** detect-service failures → auto-stop the session (state `aborted`).
- Setpoint changes rate-limited to **≤ 0.35 rad/s** at the tracking layer (below controller caps).

The `TrackState` state machine (`tracking.py`) enforces phases
`tracking → hold → stale → aborted`. See [[10 - Testing]] for the offline tests.

## Command surface risk levels

| Path | Risk | Guard |
| --- | --- | --- |
| `/api/state`, `/api/camera`, `/events`, `/api/*/status` | Read-only | none (but exposes state) |
| `/api/loco/command` | Motion | fails closed if LocoClient unavailable / validation fails |
| `/api/robot/chill` | Motion (limp) | fails closed if LocoClient unavailable |
| `/api/robot/home`, `/api/robot/straight` | Motion | arm_sdk hold / preset, XR suspend |
| `/api/wrist/command` | Motion | **`has_risk_ack` (400 without)** |
| `/api/recording/replay/robot` | Highest arm path | dry-run/validation; **refuses physical playback** until a safety controller exists |
| `/lowcmd` body control | **Highest** | prefer high-level LocoClient; simulate first |

> [!info] Raw trajectory replay is intentionally disabled
> Replaying raw joint trajectories back onto the robot is not implemented:
> it requires interpolation, joint/velocity/torque limits, controller ownership
> checks, emergency-stop supervision, and simulation validation first.

## No authentication

There is **no auth** on the dashboard. Anyone who can reach the host can read
state and hit endpoints. Use only on a trusted robot network. (MCP can add a
bearer token via `MCP_TOKEN` — see [[05 - Chat & MCP Tools]].)

## Related

[[01 - Architecture]] · [[04 - HTTP API Reference]] · [[05 - Chat & MCP Tools]] · [[06 - Person Tracking (CV Feature)]] · [[10 - Testing]]
</content>
