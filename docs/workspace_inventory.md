# Consolidated Workspace Inventory

This repository keeps the telemetry web dashboard at the root and organizes the rest of the local robot workspace into source-focused folders.

## Layout

| Path | Source | Contents |
| --- | --- | --- |
| `teleoperation/vision_pro_control` | `/home/ch/Workspace/h1_2_vision_pro_control` | H1-2 Vision Pro control scripts, docs, XR teleoperation source, and Isaac Lab simulation source. |
| `execution/semantic_teleoperation` | `/home/ch/Workspace/semantic-teleoperation/execution` | Execution-side external sources, including Unitree MuJoCo, Unitree ROS 2, and SDK2 Python snapshots used by the semantic teleoperation workspace. |
| `simulation/semantic_teleoperation` | `/home/ch/Workspace/semantic-teleoperation/simulation` | H1 simulation docs, scripts, Gazebo packages, MoveIt config, visualization config, URDF/MJCF, and meshes. |
| `robot_models/unitree_h1_2` | `/home/ch/Workspace/unitree_ros_tmp/robots/h1_2_description` | Unitree H1-2 robot model assets from the temporary ROS checkout. |
| `vendor/unitree_sdk2_python` | `/home/ch/Workspace/unitree_sdk2_python` | Unitree SDK2 Python source and examples. |
| `tools/rh56` | `/home/ch/Workspace/rh56_tools` | RH56 port probing and quick finger test utilities. |

## Excluded Generated Artifacts

The consolidation intentionally omits generated/runtime files that should not be versioned:

- Nested `.git` repositories and submodule metadata.
- Python caches such as `__pycache__` and `*.pyc`.
- Python virtual environments such as `.venv`.
- ROS/colcon and package build outputs such as `build`, `install`, and `log`.
- Package metadata generated locally, such as `unitree_sdk2py.egg-info`.

Original workspace folders were copied into this repo; they were not moved or deleted.
