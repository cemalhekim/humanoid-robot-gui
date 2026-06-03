# H1 Gazebo Simulation

This package launches the H1 URDF in Gazebo Classic through ROS 2 Humble.

## Install Dependencies

```bash
sudo apt-get install -y gazebo ros-humble-gazebo-ros-pkgs
```

## Build

```bash
cd "/home/ch/Workspace/semantic-teleoperation"
source /opt/ros/humble/setup.bash
colcon build --packages-select h1_gazebo --symlink-install
```

## Launch

```bash
source install/setup.bash
ros2 launch h1_gazebo sim.launch.py
```

By default the robot is spawned with `fixed_base:=true`, which anchors the pelvis to the world for stable visualization. Without controllers, a free-base humanoid will fall under gravity.

For a free physics model:

```bash
ros2 launch h1_gazebo sim.launch.py fixed_base:=false paused:=true
```

This is a visualization/physics spawn, not a controlled walking simulation yet. Real control needs a `ros2_control` setup, Gazebo controller plugins, and tuned controllers.

## Scripted Walking Animation

For a fixed-base walking-style animation:

```bash
ros2 launch h1_gazebo sim.launch.py fixed_base:=true paused:=false enable_walk:=true
```

This drives Gazebo joint position targets with a simple alternating gait. It is useful for visualizing motion, but it is not a 1:1 replica of Unitree's real onboard walking controller.
