<!-- Written 2026-08-26 from the vendored code in this repo; line numbers refer to that snapshot. -->

# How the reference implementations drive H1-2 arms accurately

Scope: everything below is read out of the vendored code in this repo. Line numbers are absolute. Where I extrapolate, I say so explicitly.

---

## 0. Key negative finding first

There is **no official Unitree `arm_sdk` example for the H1-2** in either vendored SDK copy.

- `/home/ch/Workspace/robot_telemetry_web/vendor/unitree_sdk2_python/example/h1_2/low_level/h1_2_low_level_example.py` is the only H1-2 example, and it uses `rt/lowcmd`, not `rt/arm_sdk`.
- `/home/ch/Workspace/robot_telemetry_web/docs/reference/h1_2_arm_sdk_dds_example.cpp` is a **0-byte placeholder** (`ls -la` shows size 0).
- The closest official `arm_sdk` references are G1 (`example/g1/high_level/g1_arm7_sdk_dds_example.py`, `g1_arm5_sdk_dds_example.py`), H2 (`example/h2/high_level/h2_arm_sdk_dds_example.py`), and the ROS2 C++ one (`execution/semantic_teleoperation/external/unitree_ros2/example/src/src/g1/high_level/g1_arm_sdk_dds_example.cpp`).

So the only H1-2-specific `arm_sdk` control law we can study is **xr_teleoperate's `H1_2_ArmController`** — which is Unitree's own teleop repo, and is therefore the authoritative H1-2 arm_sdk reference.

---

## 1. Unitree's own SDK examples

### 1a. `h1_2_low_level_example.py` (rt/lowcmd, whole body)

| Item | Value | Line |
|---|---|---|
| Control rate | `control_dt_ = 0.002` → **500 Hz** | 62 |
| Ramp duration | `duration_ = 3.0 s` | 63 |
| kp | `100.0 if i < 13 else 50.0` (legs+waist 100, **all arm joints 50**) | 123, 136 |
| kd | `1.0` for every motor | 124, 137 |
| dq | always `0.0` | 122, 135 |
| tau feed-forward | always `0.0` — **no gravity compensation at all** | 120, 133 |
| Interpolation | linear ratio `clip(t/duration, 0, 1)`, target `q = (1-ratio) * low_state.motor_state[i].q` | 118, 134 |

Two things worth copying and one worth noting:

1. **`mode_machine` is latched from the first `LowState`, never invented** (lines 104-106): `self.mode_machine_ = self.low_state.mode_machine` on the first callback, and a hard spin-wait `while self.update_mode_machine_ == False: sleep(1)` before the control thread starts (95-99). Our `_build_arm_sdk_trajectory_cmd` copies `mode_pr`/`mode_machine` from live lowstate (server.py:6331-6332) — same practice, verified correct.
2. `Mode.PR = 0` = series control for pitch/roll joints; `Mode.AB = 1` = parallel control for the A/B (ankle) joints (lines 55-57). PR is what both the example and we use.
3. The "interpolate toward zero" idiom `q = (1 - ratio) * measured_q` re-reads the **measured** q every tick, so it is a *decaying* target, not a fixed spline. That makes the commanded setpoint always within a motor-step of reality → no torque spike. This idiom is repeated in every Unitree example.

The example also gets the arms out of the onboard controller's way with `MotionSwitcherClient.ReleaseMode()` in a loop until `CheckMode()` returns an empty name (lines 77-81).

### 1b. `g1_arm7_sdk_dds_example.py` (rt/arm_sdk — the canonical weight-slot pattern)

| Item | Value | Line |
|---|---|---|
| Rate | `control_dt_ = 0.02` → **50 Hz** | 69 |
| kp / kd | **`kp = 60.0`, `kd = 1.5`, identical for all 17 arm+waist joints** | 74-75 |
| dq | `0.` always | 140, 150, 160 |
| tau | `0.` always — **no feed-forward, no gravity comp** | 138, 148, 158 |
| Weight slot | `kNotUsedJoint = 29` for G1, "NOTE: Weight" | 64 |
| Weight enable | `motor_cmd[kNotUsedJoint].q = 1` set once at stage 1 | 135 |
| Weight release | `(1 - ratio)` ramped down over `duration_ = 3 s` | 167-168 |
| `weight_rate` | declared `0.2` at line 73 but **never used** in this file | 73 |

Stage structure (130-171): 3 s ramp to zero pose → 6 s to `target_pos` → 9 s back to zero → 3 s weight ramp-down → done. Every stage uses the same `ratio * target + (1 - ratio) * measured_q` blend (149), i.e. always re-based on the measured state.

`h2_arm_sdk_dds_example.py` is the same code with `kp = 80.0, kd = 1.5` (73-74) and `kNotUsedJoint = 31` (65).

### 1c. `g1_arm_sdk_dds_example.cpp` (ROS2) — the most sophisticated official one

This one adds three ideas the Python examples lack:

- **Per-joint velocity clamp on the setpoint** (159-173):
  ```cpp
  const float max_delta = max_joint_velocity_ * control_dt_;   // 0.5 * 0.02 = 0.01 rad/tick
  current[j] += std::clamp(diff, -max_delta, max_delta);
  ```
  with `max_joint_velocity_ = 0.5F` (line 99) — a *hard 0.5 rad/s setpoint slew rate*, far more conservative than xr_teleoperate's 20 rad/s.
- **Gain tiering**: the last 3 joints in the array (the waist yaw/roll/pitch) get `kp * 4.0F` and `kd * 4.0F` — i.e. **240 / 6.0 for waist vs 60 / 1.5 for arms** (189-195).
- **Explicit weight ramp-down in `StopControl()`** (203-233): `delta_w = 0.2F * control_dt_` per tick over `2.0F / control_dt_` steps, holding `current_jpos_` while the weight decays, then a final explicit `weight = 0.0F` publish. `dq = 0`, `tau = 0` throughout (187-188, 219, 222).

**Summary of Unitree's own examples: zero feed-forward torque, zero velocity feed-forward, one flat kp/kd, always re-base the setpoint on measured q, ramp the weight in/out.** They are demos, not precision controllers.

---

## 2. xr_teleoperate `H1_2_ArmController`

File: `/home/ch/Workspace/robot_telemetry_web/teleoperation/vision_pro_control/external/xr_teleoperate/teleop/robot_control/robot_arm.py`

### Gains (622-645, 677-694)

```
kp_high = 300.0 / kd_high = 5.0     # non-arm, non-weak joints (legs)
kp_low  = 140.0 / kd_low  = 3.0     # shoulder pitch/roll/yaw + elbow pitch (13,14,15,16 / 20,21,22,23)
kp_wrist=  50.0 / kd_wrist = 2.0    # elbow roll + wrist pitch + wrist yaw (17,18,19 / 24,25,26)
```

The tier assignment is by `_Is_wrist_motor` (828-837) and `_Is_weak_motor` (811-826). Note for comparison: G1_29 uses `kp_low = 80.0, kd_low = 3.0, kp_wrist = 40.0, kd_wrist = 1.5` — **H1-2 is deliberately given the stiffest arm gains of the family (140)** because it is a heavier arm.

Crucially, `msg.motor_cmd[id].kp/kd` are written **once at construction** (679-694) and never touched in the control loop. There is no per-phase gain scheduling at all.

### The control loop `_ctrl_motor_state` (723-756)

- Rate: `control_dt = 1.0 / 250.0` → **250 Hz** (640), with a self-correcting sleep `max(0, control_dt - elapsed)` (751-753).
- Weight: `motor_cmd[kNotUsedJoint0].q = 1.0` set **once, before the loop** (724-725), only when `motion_mode` (i.e. publishing to `rt/arm_sdk`, line 647-650). `kNotUsedJoint0 = 27` for H1-2 (895).
- Setpoint: `cliped_arm_q_target = clip_arm_q_target(q_target, arm_velocity_limit)` (737).
- Per motor (739-742): `q = clipped target`, **`dq = 0`**, `tau = arm_tauff_target[idx]` ← the RNEA gravity torque from IK.
- CRC then `Write` every tick (744-745).

### `clip_arm_q_target` — the velocity limiter (716-721)

```python
current_q = self.get_current_dual_arm_q()          # measured
delta = target_q - current_q
motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
cliped_arm_q_target = current_q + delta / max(motion_scale, 1.0)
```

Three properties that matter a great deal:

1. It is **re-based on measured q every tick** — the commanded setpoint is never more than `velocity_limit * control_dt` away from reality. At 20 rad/s and 4 ms that is **0.08 rad max command-vs-measured gap**, so with kp=140 the position term can never exceed ~11 Nm. This is a *torque limiter disguised as a velocity limiter*.
2. The scaling is **uniform across all 14 joints** (`np.max` over the whole vector), so the arm's *shape* is preserved while it slows — it never distorts the pose.
3. Because the target is re-based on measured q, the loop behaves as a first-order lag toward the IK target, which is inherently stable and never winds up.

### `speed_gradual_max` (747-749, 801-805)

```python
self.arm_velocity_limit = 20.0 + (10.0 * min(1.0, t_elapsed / 5.0))
```
20 → 30 rad/s over 5 s from `_gradual_start_time`. Note `self._gradual_time = t` is stored (804) but **never read** — the `5.0` is hard-coded. `speed_instant_max()` (807-809) jumps straight to 30.0. In this repo's fork, `teleop_hand_and_arm.py:711-717` defaults `XR_ARM_SPEED_INSTANT` to **True**, so the gradual ramp is bypassed.

### Transition in / out

- **In**: there is no explicit ramp from the current pose. Safety comes entirely from `clip_arm_q_target` re-basing on measured q plus the `speed_gradual_max` ramp. The constructor also captures `all_motor_q` and pre-loads every `motor_cmd[id].q` with the measured value (694), so the first published frame is a no-op hold.
- **Out**: `ctrl_dual_arm_go_home` (780-799) sets `q_target = zeros(14)`, polls at 20 Hz until `all(|q| < 0.05)`, then ramps the weight `np.linspace(1, 0, num=101)` with `sleep(0.02)` → **a 2.02 s weight ramp-down** (793-795). Note the ramp only mutates `self.msg`; the 250 Hz publisher thread is what actually transmits it.

### Threading

Three threads: DDS read loop at ~500 Hz (`_subscribe_motor_state`, 705-714, `sleep(0.002)`), the 250 Hz publisher, and the main teleop loop at **30 Hz** (`--frequency` default 30.0, `teleop_hand_and_arm.py:512`, sleep at 949-950). A `ctrl_lock` guards only the `q_target`/`tauff_target` handoff (730-732, 760-762). **The setpoint is updated at 30 Hz and interpolated toward at 250 Hz purely by the velocity clamp** — there is no spline, no filter in the fast loop.

---

## 3. xr_teleoperate IK (`robot_arm_ik.py`, class `H1_2_ArmIK` at 608)

**Library**: Pinocchio + CasADi symbolic (`cpin.Model`, 705-706), solved by **IPOPT**.

- Model reduced by locking 12 leg joints + torso + 24 finger joints (633-676), so `nq = 14`. `L_ee`/`R_ee` frames offset `[0.05, 0, 0]` from the wrist-yaw joints (678-692). Model is pickled to `h1_2_model_cache.pkl` (616, 819-841).
- Errors: translational = `oMf.translation - Tf[:3,3]`; rotational = `cpin.log3(R_current @ R_target.T)` (718-737).
- **Cost (756)**:
  ```python
  self.opti.minimize(50 * self.translational_cost + self.rotation_cost
                     + 0.02 * self.regularization_cost + 0.1 * self.smooth_cost)
  ```
  i.e. **position:orientation = 50:1**, regularisation toward `q = 0` weighted 0.02, and a smoothness term `sumsqr(var_q - var_q_last)` weighted 0.1 where `var_q_last` is set to the previous solution (864). (G1_23 and H1 use `0.5 *` rotation; H2 uses `0.8 *`.)
- **Joint limits**: hard box constraint from the reduced model's `lowerPositionLimit`/`upperPositionLimit` (751-755).
- **Solver opts (758-776)**: `max_iter = 30`, `tol = 1e-4`, `acceptable_tol = 5e-4`, `acceptable_iter = 5`, `warm_start_init_point = 'yes'`, `expand = True`, exact Jacobian. Warm start comes from `self.init_data`, which is set to the **measured** `current_lr_arm_motor_q` each call (853-855).
- **Rate**: called once per main-loop tick → **30 Hz**.

### `sol_tauff` — the gravity feed-forward (verbatim, 874-881)

```python
if current_lr_arm_motor_dq is not None:
    v = current_lr_arm_motor_dq * 0.0
else:
    v = (sol_q - self.init_data) * 0.0

self.init_data = sol_q

sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data,
                     sol_q, v, np.zeros(self.reduced_robot.model.nv))
```

So: **RNEA at the *commanded* configuration with velocity forced to zero and acceleration zero** → `tau = g(q)`, pure model gravity from the URDF masses. The `* 0.0` is deliberate: they kept the plumbing for a velocity term but zeroed it (presumably it destabilised). The identical block is repeated in the exception handler (895-902), but the failure path then **returns `current_lr_arm_motor_q, zeros(nv)`** (909) — on non-convergence it freezes at the measured pose with zero torque, it does not push a bad solution.

### `WeightedMovingFilter` (`teleop/utils/weighted_moving_filter.py`)

FIR moving average on the 14-vector, weights must sum to 1.0 (line 9), duplicate samples skipped (28-29), and before the window fills it passes the latest sample through (15-16). Applied to `sol_q` *before* it becomes the target (871-872).

Weights: upstream G1/H1/H2 all use `[0.4, 0.3, 0.2, 0.1]` (4-tap). **This repo's H1-2 fork overrides it to `[0.75, 0.25]`** (2-tap, line 779, env `XR_H1_2_IK_SMOOTH_WEIGHTS`) — a local "snappy" patch that halves the filter lag.

---

## 4. How Unitree's onboard controller holds the arms

- **`arm_sdk` weight semantics.** No source in the vendored trees states the blending law numerically; the only labels are `kNotUsedJoint = 29 # NOTE: Weight` (g1_arm7:64) and `1:Enable arm_sdk, 0:Disable arm_sdk` (g1_arm7:135). The *modelled* semantics in this repo's own twin (`simulation/h1_2_twin/h1_2_mujoco_sim.py:186`) is a torque blend:
  ```python
  tau = weight * self._pd(armcmd.motor_cmd[i], q, dq) + (1.0 - weight) * hold
  ```
  where `hold = kp*(hold_q[i] - q) - kd*dq`, `--arm-hold-kp` default **120.0** (261). And crucially, when external authority ends the twin **latches** `hold_q[i] = q` (193-194) — "like the onboard controller holding the arms where arm_sdk left them". *This is our model of the onboard behaviour, not a Unitree statement.*
- **Onboard arm PD gains are not exposed anywhere** in the vendored code. The only handle is FSM/task level.
- **Arm tasks**: `SetTaskId` = API id `7106` (`g1_loco_client.py:67-73`), `WaveHand → 0/1`, `ShakeHand → 2/3` (114-125). H2 adds `SetArmSdkStatus` = API `7109` with `EnableArmSDK()/DisableArmSDK()` (`h2_loco_client.py:106-111, 172-176`) — an *explicit RPC toggle* which the H1-2 does not have; H1-2 must use the weight slot.
- **`G1ArmActionClient`** (`g1/arm/g1_arm_action_client.py`) exposes a canned action map (`"release arm": 99`, `"hands up": 15`, …) via API `7106`/`7107` — G1 only.
- **Damping/release**: `Damp()` = `SetFsmId(1)`, `ZeroTorque()` = `SetFsmId(0)` (`g1_loco_client.py:75-94`). xr_teleoperate binds `loco_wrapper.Damp()` to both thumbsticks as a soft e-stop (`teleop_hand_and_arm.py:785-786`).
- **Isaac Lab** (`unitree_sim_isaaclab/robots/unitree.py:1109` `H12_CFG_WITH_INSPIRE_HAND`): the arm actuator group uses **shoulder kp 25 / elbow kp 50 / wrist kp 40, damping 2.0 everywhere** (1224-1233), `soft_joint_pos_limit_factor = 0.9` (1194). These are *simulation* implicit-actuator gains for RL, an order of magnitude below the real `arm_sdk` numbers — do not treat them as a hardware reference.

---

## 5. Our controller, and the concrete deltas

Ours (`server.py`): `execute_arm_sdk_replay` (3842) → playback loop at `TRAJECTORY_DEFAULT_DT = 1/60` (181, 3993, 4021) → hold loop at `ARM_REPLAY_HOLD_HZ = 120.0` (247, 4033). `_closed_loop_arm_targets` (4408) is the outer PID; `_build_arm_sdk_trajectory_cmd` (6320) emits the frame.

### Comparison table

| Aspect | **Ours** | **Unitree example (g1_arm7 / ROS2 cpp)** | **xr_teleoperate H1_2** | **Onboard / Isaac** |
|---|---|---|---|---|
| Publish rate | 60 Hz playback, 120 Hz hold | 50 Hz | **250 Hz** | n/a |
| Setpoint source rate | recording frames @60 Hz | scripted ramp | IK @30 Hz | n/a |
| Arm kp (shoulder) | `ARM_SDK_KP` 120 (339) × plan scale 0.75-1.15 × phase scale (0.35 play / 0.55 hold / 0.75 approach) ⇒ **≈32-50 play, ≈50-76 hold** | 60 (flat) | **140 (flat)** | Isaac 25 (sim) |
| Arm kd (shoulder) | 2.0 × 1.2 = **2.4** | 1.5 | **3.0** | Isaac 2.0 |
| Wrist kp/kd | 50 / 1.0 × scales ⇒ ≈17-28 / 1.2 | 60 / 1.5 | **50 / 2.0** | Isaac 40 / 2.0 |
| Waist | in `ARM_SDK_JOINTS` (200/2.0) but **discarded** at 3943 | kp×4 = 240 (ROS2) | not driven | — |
| `dq` command | **always 0** (6351) | 0 | **0** (741) | — |
| `tau` feed-forward | **filtered measured `tau_est`** (4452, 4470) × stationarity blend + learned integral (4490-4496), clamped 15/10/4/6 Nm (330-335) | **0** | **`pin.rnea(q, 0, 0)` = model gravity** (881) | — |
| Outer position loop | **yes** — P+I+D on joint error, gains 0.28/0.035/0.018 (shoulder) (318-322), correction clamped ±0.12 rad (202) | none | **none** | — |
| Setpoint re-based on measured q? | **No** — absolute `desired_q + correction` | **Yes** (`(1-ratio)*measured_q`) | **Yes** (`current_q + delta/scale`) | — |
| Velocity limiting | offline gate 2.0 rad/s (180) + smootherstep approach peak 0.6 rad/s (200) | ROS2: 0.5 rad/s hard slew | **20→30 rad/s per-tick clamp, re-based** (719, 749) | — |
| Target filtering | none in the loop (offline smootherstep only, 4333) | none | **WeightedMovingFilter on IK output** (871-872) | — |
| Weight slot | `weight=1.0` hard-coded at every call site (4013, 4099, 6173, 7307); `weight=0.0` only in `stop_wrist` × 10 writes (6686-6688) | ramp in implicit, **ramp out over 3 s / 2 s** | ramp out `linspace(1,0,101)` @50 Hz = 2.02 s | twin blends torque by weight |
| Gain scheduling | **yes**, 3 phases (approach/move/hold) + response dial (4244-4298) | none | none | — |
| Failure behaviour | escalation to 2.0× (286-287), 90 s ceiling then flagged safe-hold (291, 4121-4142) | none | IK failure → hold measured pose, tau=0 (909) | — |

### The five substantive differences

1. **Feed-forward source.** We differentiate the plant: `state["gravity_tau"]` is a 0.4 s low-pass of `tau_est` (209, 4469-4470), which by construction *contains our own PD reaction*. The code comments already name this hazard twice (204-208, 236-240). xr_teleoperate feeds forward `g(q)` from the URDF — an *open-loop, causally clean* signal that cannot form a feedback loop with kd. Our own `ARM_REPLAY_GRAVITY_MODEL_SCALE = 0.0` (266) and `kinematics.gravity_torques` (kinematics.py:247) are the same idea, already built and shipped **off by default**.
2. **No re-basing.** Every Unitree reference recomputes the command from the *measured* q each tick. We command `desired_q + correction` absolutely. This is why we need the integral, the anti-windup limit (203), the jump detector (4459-4464), the stall escalation (4086-4088) and the 90 s ceiling — machinery that simply does not exist upstream.
3. **Effective stiffness.** At the default response 0.5, our shoulder inner kp lands around 32-50 during playback (120 × 0.75-1.15 × 0.35) versus xr_teleoperate's constant **140**. We are running roughly **3-4× softer** and compensating with an outer loop + measured-torque feed-forward. That trade is what the twin sweep shows: `reports/20260825T122142Z-sweep-ai-dev.md` reports trajectory-tracking **lag 0.55-0.8 s** and raw RMS **0.24 rad** (final static error is fine at ~0.003 rad).
4. **No velocity feed-forward, anywhere.** All four sources set `dq = 0`. For us that is the single largest tracking-lag contributor, because unlike the references our setpoint is *not* re-based on measured q — so lag accumulates as real position error instead of being absorbed.
5. **Weight handling.** We never ramp in or out on the replay path; both Unitree examples and xr_teleoperate ramp out over ~2-3 s. Our only weight-0 write is 10 packets over 0.1 s in `stop_wrist` (6687-6689).

---

## 6. What a comprehensive control law could look like

Marked **[V]** = verified pattern from code above, **[I]** = my inference/synthesis.

**a. Replace measured-torque feed-forward with model gravity. [V]**
Set `ARM_REPLAY_GRAVITY_MODEL_SCALE` toward 1.0 and use `ARM_KINEMATICS.gravity_torques` (server.py:4436-4446, kinematics.py:247) — or better, `pin.rnea(model, data, q_cmd, 0, 0)` exactly as `robot_arm_ik.py:881`, which is the same quantity computed analytically rather than by finite-differencing potential energy. **[I]** Evaluate the model at the *commanded* q like xr_teleoperate does, not at measured q as we currently do (4439) — evaluating at measured q reintroduces a feedback path. Keep the adaptive `gravity_learn` integral (4504-4505) on top: xr_teleoperate has no such term, and it is what absorbs URDF mass error and payload; but with a clean feed-forward it should be able to run with a *smaller* gain than the current 22.0.

**b. Re-base the setpoint on measured q with a uniform velocity clamp. [V]**
Port `clip_arm_q_target` (robot_arm.py:716-721) verbatim: uniform `np.max` scaling across the 14 joints so pose shape is preserved, applied at the publish rate. **[I]** Pick the limit from the ROS2 example's spirit rather than xr's: `0.5-2.0 rad/s` is a real motion limit; 20 rad/s at 250 Hz is really a *torque* limiter. With a re-based setpoint you can drop `ARM_REPLAY_INTEGRAL_LIMIT`, the jump detector and most of the stall machinery, or keep them purely as diagnostics.

**c. Add velocity feed-forward `dq`. [I]**
Every reference sets `dq = 0`, so this is *not* verified from code — but xr_teleoperate's `v = current_dq * 0.0` (875) shows they wired it and disabled it. The safe version: `dq_cmd = (q_target[k] - q_target[k-1]) / dt`, clamped, filtered, and only during the playback phase (force to 0 during hold, otherwise the kd term fights the settle). Expected payoff is directly on the measured 0.55-0.8 s tracking lag.

**d. Per-joint kp/kd tiers closer to Unitree's H1-2 numbers. [V]**
xr_teleoperate's tiering is *shoulder+elbow-pitch = 140/3.0, elbow-roll+wrists = 50/2.0* (robot_arm.py:631-636 with 811-837). Ours splits shoulder yaw down to 80 and elbow pitch down to 50 (server.py:339). **[I]** Once the feed-forward is model-based (which removes the kd/feed-forward positive-feedback coupling that motivated `ARM_REPLAY_HOLD_KD_SCALE` staying at 1.2, per the comment at 236-240), raising the arm kd toward 3.0 and the hold kp toward 0.8-1.0× nominal becomes safe and would sharply cut settle time.

**e. Filter the target, not the feed-forward. [V]**
Add a `WeightedMovingFilter`-style FIR on the commanded joint vector (weighted_moving_filter.py:14-35). We currently low-pass the *torque* at 0.4 s (209) which is a phase-lagged inner signal; filtering the *target* is phase-honest and is what Unitree does. **[I]** 2-3 taps at 60-120 Hz, e.g. `[0.6, 0.3, 0.1]`.

**f. Weight ramps in and out. [V]**
On session start ramp `motor_cmd[27].q` 0→1 while commanding measured q (the ROS2 `StopControl` pattern reversed, g1_arm_sdk cpp:203-233); on clean finish ramp 1→0 over ~2 s holding the final pose, then one explicit `weight = 0.0`. xr_teleoperate uses `linspace(1, 0, 101)` at 50 Hz (robot_arm.py:793-795). This removes the authority-handover transient at both ends and lets the onboard controller latch the pose smoothly.

**g. Raise the publish rate. [I]**
250 Hz (xr_teleoperate) vs our 60/120 Hz. With a re-based setpoint the publish rate *is* the effective interpolation rate, so 200-250 Hz for the whole session (playback and hold) would replace the current two-rate split entirely.

**h. Keep what is ours and better. [V]**
Nothing upstream has: the outer PID with per-group gains (318-322), the Cartesian lever-arm proxy convergence test (277-282, 4516-4517), the per-joint settle latch with hysteresis (260, 4062-4066), stall escalation (4086-4088), the safe-hold ceiling (291), the response dial (4235-4298), and the `RTW_TUNING_JSON` + CMA-ES optimisation rig against the MuJoCo twin. Those are genuine additions — the synthesis is to keep them and swap the *substrate* (feed-forward source, setpoint re-basing, dq, rate) to the reference implementations' proven form.

**Suggested order:** (a) model gravity → (b) re-basing + velocity clamp → (g) rate → (c) dq → (d) gain retune → (e)(f) polish. (a) and (b) are the two that the code's own comments already identify as the root causes of the observed bob/limit-cycle and lag.
