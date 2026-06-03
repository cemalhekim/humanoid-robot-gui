# Unitree MuJoCo Integration

`unitree_mujoco` is a Unitree-maintained simulator built around MuJoCo and Unitree SDK2 DDS messages. It is a better fit than our quick Gazebo spawn when the goal is sim-to-real behavior because the simulator consumes the same style of low-level Unitree messages used by `unitree_ros2`, `unitree_sdk2`, and `unitree_sdk2_python`.

The checkout lives under `external/unitree_mujoco`, which is ignored by Git.

## Why It Helps

- Includes official H1 MJCF assets at `unitree_robots/h1`.
- Publishes simulated Unitree state topics.
- Consumes Unitree low-level command topics.
- Supports loopback DDS simulation on interface `lo`.
- Provides a more realistic bridge toward real Unitree low-level control than the simple Gazebo visual animation.

Important limitation: the current upstream simulator mainly supports low-level development. It does not magically provide Unitree's proprietary high-level walking controller. For walking, we still need either a provided low-level controller, a recorded trajectory, or our own controller.

## Fetch

```bash
cd "/home/ch/Workspace/semantic-teleoperation"
./scripts/fetch_unitree_mujoco.sh
```

## Install Python Dependencies

```bash
./scripts/install_unitree_mujoco_python_deps.sh
```

This installs:

- `mujoco`
- `pygame`
- editable `unitree_sdk2_python`

The run scripts also prepend `external/unitree_sdk2_python` to `PYTHONPATH` so the SDK can load Unitree's bundled CRC shared library from the source checkout.

If `unitree_sdk2_python` fails because CycloneDDS cannot be located, install/build CycloneDDS as described in the upstream SDK README.

## Run H1 MuJoCo

```bash
./scripts/run_unitree_mujoco_h1.sh
```

This configures `external/unitree_mujoco/simulate_python/config.py` for:

- `ROBOT = "h1"`
- loopback interface `lo`
- DDS domain `1`
- joystick disabled
- elastic band enabled for humanoid startup

## Stand Then Handshake Demo

```bash
./scripts/run_h1_mujoco_handshake_demo.sh
```

This starts the H1 MuJoCo simulator, waits for `rt/lowstate`, then publishes low-level H1 commands on `rt/lowcmd`:

- stand still for a few seconds
- raise the right arm
- perform a short handshake-like arm oscillation
- return to the standing pose

This is a simulation-only SDK2 low-level command demo. It is useful for testing the Unitree DDS command path, but it is not a certified real-robot behavior and should not be sent to hardware without proper safety checks, limits, and an emergency stop path.

## Two-Step Walk Demo

```bash
./scripts/run_h1_mujoco_two_step_walk.sh
```

This starts the H1 MuJoCo simulator, moves to the model's crouched home pose, performs two slow scripted steps, then returns to home and keeps publishing the hold command. This is a visual/quasi-static stepping demo that relies on the simulator's elastic band; it is not a dynamic walking controller.

## Jump Demo

```bash
./scripts/run_h1_mujoco_jump.sh
```

This starts the H1 MuJoCo simulator and executes a scripted jump-like motion from the home pose. It is a conservative simulator-only behavior, not a dynamic on-hardware jump.

## H1 Loco SetVelocity Client

```bash
./scripts/run_h1_loco_set_velocity_walk.sh <network-interface>
```

This sends Unitree's high-level H1 `loco` RPC command `SetVelocity(vx, vy, vyaw, duration)`. It is for a real H1, or a runtime that implements the `loco` service. The Python MuJoCo bridge used above does not implement that RPC service, so this client will not make the current MuJoCo simulator walk.

To test that client API against MuJoCo, use the local `loco` shim:

```bash
./scripts/run_h1_mujoco_loco_set_velocity_test.sh
```

The shim implements a minimal `loco` RPC server and translates positive forward `SetVelocity` calls into the scripted two-step low-level MuJoCo walk.

## ROS 2 / DDS Context

The MuJoCo simulator is not a ROS launch file. It talks DDS using Unitree SDK2 Python. To interact with it from ROS 2, source the Unitree ROS2 environment:

```bash
source scripts/source_unitree_ros2.sh lo
```

Then inspect topics:

```bash
ros2 topic list
```

## Relation To Gazebo

`h1_gazebo` is useful for basic ROS/Gazebo visualization. `unitree_mujoco` is the path to more realistic Unitree DDS sim-to-real experiments.
