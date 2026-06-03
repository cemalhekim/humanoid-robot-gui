# Unitree ROS2 Integration

This project uses `unitreerobotics/unitree_ros2` as an external dependency, not vendored source. The checkout lives under `external/unitree_ros2`, which is ignored by Git.

## Why It Helps

`h1_description` and `h1_moveit_config` let us visualize and plan for the robot model. `unitree_ros2` adds the Unitree ROS 2 message packages and examples needed to communicate with real Unitree robots over DDS.

For H1 work, the useful pieces are:

- `unitree_hg`: H1/G1/H1-2 style low-level message definitions.
- `unitree_api`: service/client API messages used by Unitree examples.
- `read_low_state_hg`: example for reading humanoid low-level state.
- `low_level_ctrl_hg`: example for low-level humanoid control.

This can eventually bridge natural-language or MoveIt plans into real Unitree state/control topics, but it is not the same thing as the URDF or MoveIt config.

## Install System Dependencies

Run once:

```bash
sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl libyaml-cpp-dev
```

## Fetch And Build

```bash
cd "/home/ch/Workspace/semantic-teleoperation"
./scripts/fetch_unitree_ros2.sh
./scripts/build_unitree_ros2.sh
```

## Source Environment

For local/off-robot testing:

```bash
source scripts/source_unitree_ros2.sh lo
```

For a real robot over Ethernet, replace `enp3s0` with the network interface connected to the robot:

```bash
source scripts/source_unitree_ros2.sh enp3s0
```

For default CycloneDDS interface selection:

```bash
source scripts/source_unitree_ros2.sh default
```

## Example Checks

```bash
ros2 pkg list | grep unitree
ros2 topic list
```

With a real H1/H1-2/G1-compatible robot connected, try:

```bash
external/unitree_ros2/example/install/unitree_ros2_example/bin/read_low_state_hg
```
