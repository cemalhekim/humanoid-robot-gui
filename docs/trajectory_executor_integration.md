# Trajectory Executor Integration

This document defines the safe path from dashboard replay preview to supervised
robot execution.

## Current State

The dashboard can preview a selected pose or sequence:

- Blue: current robot/reference pose.
- Red: target pose or current sequence frame.
- Green: simulated trajectory.

The backend can now dry-run the selected recording and report:

- selected command path: `arm_sdk` or `lowcmd`
- frame count and duration
- moving joints
- moving lower-body joints
- max frame delta
- max joint velocity
- per-joint `kp`/`kd` gain plan
- optional Inspire/RH56 finger command plan
- validation violations

Physical execution remains locked.

## Command Path Rule

Use `arm_sdk` when the planned trajectory does not move lower-body joints.

Use `lowcmd` when any lower-body joint moves beyond the stationary threshold.

Lower body currently means H1-2 leg joints:

- `0-5`: left leg
- `6-11`: right leg

`WaistYaw` is not treated as lower body because the current arm SDK path already
includes the waist slot.

Finger movement does not change this body route decision. RH56BFX / Inspire hand
targets are planned as a parallel hand lane and should be sent through
`rt/inspire/cmd`, not through `rt/arm_sdk` or `rt/lowcmd`.

## Required Execution Stages

### 1. Dry Run

`POST /api/recording/replay/robot` with:

```json
{
  "filename": "recording.jsonl",
  "preview_complete": true,
  "dry_run": true
}
```

The server returns the plan without publishing motor commands.

The plan includes:

```text
control_path
valid_for_execution
moving_joints
moving_lower_body_joints
max_frame_delta_rad
max_velocity_rad_s
gain_plan[]
hand_plan
violations[]
```

### 2. Right-Wrist Execution

First live execution should be a single right-wrist trajectory.

Requirements:

- live `rt/lowstate` available
- local operator present
- explicit `armed=true`
- explicit `i_understand_risk=true`
- cancel flag wired to stop command
- telemetry following error checked during publish

### 3. Arm SDK Execution

Use `rt/arm_sdk` for arm/waist trajectories when lower body remains stationary.

The executor should:

- interpolate from live pose to frame 0
- publish at a fixed rate
- set the arm SDK weight slot
- hold non-target arm/waist joints at live or planned positions
- release arm SDK weight on stop

### 4. LowCmd Execution

Use `rt/lowcmd` only when lower-body joints move.

The executor must:

- publish all 27 real H1-2 body joints every tick
- preserve `mode_machine` from live lowstate
- compute CRC for every command
- apply per-joint `kp` and `kd`
- clamp frame-to-frame deltas and velocities
- abort if telemetry is stale or tracking error grows
- restore/stop safely on cancellation

### 5. Finger Execution

Use `rt/inspire/cmd` for RH56BFX / Inspire finger trajectories when the selected
recording contains hand joint movement.

The executor should:

- read current hand state from `rt/inspire/state`
- interpolate from live hand state to frame 0
- publish `MotorCmds_` hand targets on the same timeline as the body executor
- keep the hand route parallel to the selected body route
- abort together with the body executor on any stop condition

The dashboard dry-run response reports this as `hand_plan`. If no finger joint
moves beyond the stationary threshold, `hand_plan.enabled` is false.

## Stop Conditions

Execution must stop immediately when:

- operator presses stop
- telemetry is stale
- DDS publish fails
- command loop misses timing budget repeatedly
- actual-vs-target joint error exceeds threshold
- any requested joint exceeds configured limits
- motion switcher/controller ownership changes unexpectedly

## Not Yet Implemented

- command publishing from selected replay files
- `rt/inspire/cmd` publishing for selected finger trajectories
- operator arming UI for replay execution
- per-joint position limits
- per-joint tracking error thresholds
- actual executor thread/status endpoint
- robot-side E-stop integration
