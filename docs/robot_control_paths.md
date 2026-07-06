# Robot Control Paths

This document lists the robot motion/control paths visible from this machine.
It is written for the Unitree H1-2 setup currently monitored by this dashboard.

## Current Live Topics

Discovered through `/api/ros-graph` on the robot link `enp0s31f6`.

### Low-Level Body Control

| ROS 2 topic | SDK/DDS topic | Type | Direction | Notes |
|---|---|---|---|---|
| `/lowstate` | `rt/lowstate` | `unitree_hg/msg/LowState` and `unitree_go/msg/LowState` | robot to PC | Main body state. Use `unitree_hg` for H1-2 body data. |
| `/lf/lowstate` | likely `rt/lf/lowstate` or low-frequency bridge | `unitree_hg/msg/LowState` and `unitree_go/msg/LowState` | robot to PC | Low-frequency state variant. |
| `/lowcmd` | `rt/lowcmd` | `unitree_hg/msg/LowCmd` | PC to robot | Full low-level body command. High risk. Can fight onboard controllers. |
| `/arm_sdk` | `rt/arm_sdk` through SDK2 Python | `unitree_hg/msg/LowCmd` | PC to robot arm SDK bridge | Arm/waist command path used by the dashboard wrist command experiments. Lower body should remain owned by higher-level controllers. |
| `/loco_sdk` | unknown bridge name | `unitree_hg/msg/LowState` | robot/service to PC | Likely loco state/SDK bridge. Read-only unless paired with API requests. |

### High-Level Robot APIs

These use `unitree_api/msg/Request` and `unitree_api/msg/Response`.

| Request topic | Response topic | Likely control area |
|---|---|---|
| `/api/loco/request` | `/api/loco/response` | H1 locomotion service: stand, start, velocity movement. |
| `/api/sport/request` | `/api/sport/response` | G1/Go-style sport/loco service. Present on the graph, but model compatibility must be verified. |
| `/api/motion_switcher/request` | `/api/motion_switcher/response` | Switches active motion/control service. Important before taking low-level control. |
| `/api/robot_state/request` | `/api/robot_state/response` | Robot state service/config reporting. |
| `/api/config/request` | `/api/config/response` | Robot configuration service. |
| `/api/bashrunner/request` | `/api/bashrunner/response` | Remote command service. Avoid for motion unless explicitly understood. |
| `/api/videohub/request` | `/api/videohub/response` | Video service, not motion. |

### State, Sensors, And Non-Motion Topics

| Topic | Type | Notes |
|---|---|---|
| `/lf/bmsstate` | `unitree_hg/msg/BmsState` | Battery/BMS state. |
| `/lf/mainboardstate` | `unitree_hg/msg/MainBoardState` | Main board state. |
| `/lf/odommodestate` | `unitree_go/msg/SportModeState` | Odometry/motion state. |
| `/odommodestate` | `unitree_go/msg/SportModeState` | Odometry/motion state. |
| `/wirelesscontroller` | `unitree_go/msg/WirelessController` | Wireless remote inputs. |
| `/inspire/state` | `unitree_go/msg/MotorStates` | Inspire/RH56 hand state. |
| `/frontvideostream` | `unitree_go/msg/Go2FrontVideoData` | Front camera H264 payload. |
| `/EstimatorData` | `unitree_go/msg/EstimatorData` | Estimator output. |
| `/SymState`, `/SymState_back` | `unitree_go/msg/SymState` | System/symbolic state. |

## SDK Function Paths

### H1 Loco Client

Local SDK file:
`/home/ch/Workspace/robot_telemetry_web/execution/semantic_teleoperation/external/unitree_sdk2_python/unitree_sdk2py/h1/loco/h1_loco_client.py`

Service name:
`loco`

Likely DDS API topics:
`rt/api/loco/request`, `rt/api/loco/response`

Functions:

| Function | API ID | Meaning |
|---|---:|---|
| `SetFsmId(fsm_id)` | `8101` | Set locomotion FSM/mode. |
| `SetStandHeight(height)` | `8104` | Set standing height. |
| `SetVelocity(vx, vy, omega, duration)` | `8105` | Command body velocity. |
| `Damp()` | via `SetFsmId(1)` | Damping/passive mode. |
| `Start()` | via `SetFsmId(204)` | Start locomotion controller. |
| `StandUp()` | via `SetFsmId(2)` | Stand up. |
| `ZeroTorque()` | via `SetFsmId(0)` | Zero torque/passive. |
| `StopMove()` | via `SetVelocity(0,0,0)` | Stop movement. |
| `HighStand()` | via `SetStandHeight(UINT32_MAX)` | High stand preset. |
| `LowStand()` | via `SetStandHeight(0)` | Low stand preset. |
| `Move(vx, vy, vyaw, continuous_move=False)` | via `SetVelocity` | Convenience movement command. |

This is the best first path for whole-body movement because it uses the onboard
controller instead of directly fighting individual motors.

### Low-Level H1-2 Body Command

Message:
`unitree_hg/msg/LowCmd`

Command topic:
`/lowcmd` in ROS 2, `rt/lowcmd` through SDK2 Python.

State topic:
`/lowstate` in ROS 2, `rt/lowstate` through SDK2 Python.

Message shape:

```text
LowCmd
  mode_pr
  mode_machine
  motor_cmd[35]
    mode
    q
    dq
    tau
    kp
    kd
    reserve
  reserve[4]
  crc
```

Real H1-2 body motors seen in live state:

| Index | Joint |
|---:|---|
| 0 | LeftHipYaw |
| 1 | LeftHipPitch |
| 2 | LeftHipRoll |
| 3 | LeftKnee |
| 4 | LeftAnklePitch |
| 5 | LeftAnkleRoll |
| 6 | RightHipYaw |
| 7 | RightHipPitch |
| 8 | RightHipRoll |
| 9 | RightKnee |
| 10 | RightAnklePitch |
| 11 | RightAnkleRoll |
| 12 | WaistYaw |
| 13 | LeftShoulderPitch |
| 14 | LeftShoulderRoll |
| 15 | LeftShoulderYaw |
| 16 | LeftElbow |
| 17 | LeftWristRoll |
| 18 | LeftWristPitch |
| 19 | LeftWristYaw |
| 20 | RightShoulderPitch |
| 21 | RightShoulderRoll |
| 22 | RightShoulderYaw |
| 23 | RightElbow |
| 24 | RightWristRoll |
| 25 | RightWristPitch |
| 26 | RightWristYaw |
| 27-34 | ReservedMotorSlot |

Direct low-level control must:

1. Subscribe to `rt/lowstate`.
2. Build a full `unitree_hg_msg_dds__LowCmd_()`.
3. Fill all 35 motor command slots.
4. Set target joint `q`, `dq`, `tau`, `kp`, `kd`.
5. Set safe values for non-commanded joints.
6. Compute CRC with `unitree_sdk2py.utils.crc.CRC`.
7. Publish repeatedly to `rt/lowcmd`.

Observed result from the right-wrist test: isolated `rt/lowcmd` wrist control
vibrated and did not move meaningfully. That means the active controller is
probably not releasing that joint to this command path, or the correct command
path is `/arm_sdk` or a motion-switcher/API sequence.

### Motion Switcher API

Topics:
`/api/motion_switcher/request`, `/api/motion_switcher/response`

Purpose:
Switch which onboard service owns motion control.

This is probably required before reliable low-level/arm control, but we have not
identified the exact API IDs or safe call sequence yet. Do not guess here on a
live robot.

### Arm SDK Topic

Topic:
`/arm_sdk`

Type:
`unitree_hg/msg/LowCmd`

The dashboard now initializes an SDK2 Python publisher on `rt/arm_sdk`.
Arm SDK commands still use the 35-slot `LowCmd` shape, but the command enables
the arm SDK weight slot at index `27` and fills only the waist/arm joints:

```text
12 WaistYaw
13-19 left arm
20-26 right arm
27 arm SDK weight / enable slot
```

Use this path for replay plans whose lower-body joints remain stationary.
If any leg joint `0-11` moves beyond the stationary threshold, route to
`rt/lowcmd` instead.

### Inspire / RH56 Finger Command

State topic:
`/inspire/state`, DDS `rt/inspire/state`

Command topic:
DDS `rt/inspire/cmd`

Message shape:
`unitree_go.msg.dds_.MotorCmds_` with 12 target values in the Inspire bridge
order:

```text
0 RightPinky
1 RightRing
2 RightMiddle
3 RightIndex
4 RightThumbBend
5 RightThumbRotation
6 LeftPinky
7 LeftRing
8 LeftMiddle
9 LeftIndex
10 LeftThumbBend
11 LeftThumbRotation
```

Finger commands are planned in parallel with body replay. They do not choose
between `arm_sdk` and `lowcmd`; the body route still follows the lower-body
movement rule, while finger targets are sent through the Inspire bridge.

## Practical Control Functions To Build

### Read-Only/Safe

```python
probe_lowstate(interface) -> LowState
list_motor_modes(lowstate) -> dict[int, int]
get_joint_q(lowstate, joint_index) -> float
get_robot_modes(lowstate) -> tuple[int, int]
watch_joint(interface, joint_index)
```

### High-Level Movement

Use `LocoClient`, not lowcmd:

```python
stand_up(interface)
start_loco(interface)
stop_move(interface)
move_velocity(interface, vx, vy, vyaw, duration)
damp(interface)
zero_torque(interface)
```

### Low-Level Diagnostics

Use only for tiny tests:

```python
nudge_joint_lowcmd(interface, joint_index, delta, seconds)
hold_joint_lowcmd(interface, joint_index, q, seconds)
relax_joint_lowcmd(interface, joint_index)
```

These need a controller ownership check before real motion.

### Arm/Wrist Control Investigation

Before writing more wrist motion:

```python
probe_arm_sdk_topic()
probe_motion_switcher_api()
try_arm_sdk_nudge(joint_index, delta)
compare_lowstate_before_after()
```

## Recommended Order

1. Use `LocoClient` for standing/walking/body movement.
2. Use dashboard lowstate analysis for monitoring.
3. Investigate `/arm_sdk` and `/api/motion_switcher` before commanding wrists.
4. Avoid direct `/lowcmd` continuous motion unless the robot is explicitly in a
   mode that accepts low-level control.
