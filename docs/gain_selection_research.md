# Gain Selection Research Notes

This note defines a conservative gain-selection algorithm for future trajectory
execution on Unitree H1-2.

## Source Signals

Unitree SDK2 examples use PD-style low-level commands:

```text
q target
dq target
kp
kd
tau
```

Unitree's SDK2 Python README describes a low-level safety example with
`kp=10`, `kd=1` for a single joint hold. Unitree low-level examples also publish
interpolated `q` targets with fixed `Kp`/`Kd`, `dq=0`, and `tau=0`.

The Unitree RL deployment code keeps gains in robot config arrays and applies
them per motor group rather than deriving them from a recorded trajectory.

Humanoid control literature also supports gain switching by mode/state instead
of one static universal gain table. For our use case, the first integration
should use conservative preset gains and switch/scale them by command path and
motion class.

## Initial Gain Policy

Do not store or replay gains from recordings.

Use this runtime policy:

1. Select command path.
   - `arm_sdk` when lower body is stationary.
   - `lowcmd` when lower body moves.
2. Select base gain table by command path and joint group.
3. Scale gains only within conservative bounds.
4. Slow or reject trajectories instead of increasing gains aggressively.
5. Use live telemetry tracking error to abort, not to chase with larger gains.

## Base Tables

### Arm SDK

Use Unitree-style arm/waist presets already present in this repo:

| Joint group | Kp | Kd |
| --- | ---: | ---: |
| Shoulder pitch/roll | 120 | 2.0 |
| Shoulder yaw | 80 | 1.5 |
| Elbow | 50 | 1.0 |
| Wrist | 50 | 1.0 |
| Waist yaw | 200 | 2.0 |

### LowCmd Conservative Start

Use lower values for direct full-body low-level playback:

| Joint group | Kp | Kd |
| --- | ---: | ---: |
| Hip | 40 | 1.0 |
| Knee | 45 | 1.1 |
| Ankle | 30 | 0.8 |
| Waist | 35 | 1.0 |
| Shoulder | 25 | 0.8 |
| Elbow | 20 | 0.7 |
| Wrist | 12 | 0.5 |

These should be treated as starting points, not final robot-tuned values.

## Scaling Algorithm

For each joint, compute planned max velocity and max position step:

```text
max_step = max(abs(q[i] - q[i-1]))
max_velocity = max_step / dt
```

Select a gain scale:

```text
step_scale = clamp(max_step / nominal_step, 0.7, 1.15)
velocity_scale = clamp(max_velocity / nominal_velocity, 0.7, 1.15)
scale = min(step_scale, velocity_scale)
```

Then:

```text
kp = base_kp * scale
kd = base_kd * sqrt(scale)
```

Limits:

```text
0.6 * base_kp <= kp <= 1.2 * base_kp
0.6 * base_kd <= kd <= 1.2 * base_kd
```

If a trajectory requires more than `1.2x` base gains, do not increase gains.
Instead slow/interpolate the trajectory.

## Aggressiveness Decision Algorithm

The executor should not ask "what is the strongest gain that works?" It should
ask "is this trajectory asking too much from the current control path?"

Use two scores:

1. Offline demand score, computed before publishing.
2. Online tracking score, computed while publishing.

### Offline Demand Score

For each joint:

```text
step_ratio = max_step / nominal_step[joint_group]
velocity_ratio = max_velocity / nominal_velocity[joint_group]
accel_ratio = max_acceleration / nominal_acceleration[joint_group]
range_ratio = max(abs(q - neutral_q)) / allowed_range[joint]
```

Then:

```text
demand_score = max(step_ratio, velocity_ratio, accel_ratio, range_ratio)
```

Interpretation:

| Demand score | Action |
| ---: | --- |
| `< 0.6` | use soft gains |
| `0.6-1.0` | use base gains |
| `1.0-1.2` | use upper bounded gains and slower playback |
| `> 1.2` | reject or retime trajectory |

Do not solve a bad trajectory with stronger gains. First retime it.

### Online Tracking Score

During execution, compute:

```text
position_error = abs(actual_q - target_q)
velocity_error = abs(actual_dq - target_dq)
phase_lag = sign-consistent delay between target and actual motion
overshoot = actual_q crosses target_q with high velocity
oscillation = repeated sign changes in position error
saturation = tau_est/current/temperature near limits
```

Rules:

| Observation | Meaning | Action |
| --- | --- | --- |
| high position error, low velocity, no overshoot | sluggish tracking | slightly increase `kp`, keep/increase `kd` |
| overshoot or oscillation | too aggressive or underdamped | reduce `kp`, increase damping only within limit, or slow trajectory |
| high velocity error | trajectory too fast | slow trajectory, do not chase with gains |
| high tau/current/temp | actuator stress | abort or lower gains |
| stale telemetry | unsafe feedback | abort |

This is intentionally conservative. The robot should never "argue harder" with
the trajectory when the safer answer is to move slower.

### Gain Scale Formula

Use a bounded scale:

```text
demand_scale = clamp(0.75 + 0.25 * demand_score, 0.75, 1.15)
tracking_scale = clamp(1.0 + 0.15 * sluggish_score - 0.25 * oscillation_score, 0.75, 1.15)
scale = min(demand_scale, tracking_scale)
```

Then:

```text
kp = base_kp * scale
kd = base_kd * sqrt(scale)
```

If oscillation is present:

```text
kp = kp * 0.85
kd = min(base_kd * 1.15, kd * 1.10)
```

If saturation is present:

```text
abort
```

No gain should exceed `1.2x` the base table without a robot-specific tuning
session.

## Tracking Error Policy

During execution, compare live telemetry against target:

```text
position_error = abs(actual_q - target_q)
velocity_error = abs(actual_dq - target_dq)
```

Abort on:

- stale telemetry
- repeated publish deadline misses
- position error above per-joint threshold
- unexpected lower-body movement during `arm_sdk`
- motor temperature/current warnings

Do not use online gain increases as the first response to error. Prefer abort
or slower trajectory playback.

## Integration Steps

1. Add `GainSelector` with base tables and scaling. Done.
2. Add dry-run output for selected per-joint `kp`/`kd`. Done.
3. Test with right wrist only.
4. Test arm-only `arm_sdk`.
5. Test lowcmd in hanging/supported state only.
6. Tune base tables from observed tracking, not from one-off visual success.
