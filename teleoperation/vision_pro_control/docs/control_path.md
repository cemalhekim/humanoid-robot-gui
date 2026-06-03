# H1-2 XR Teleop Control Path

This repo keeps Unitree's XR teleoperation code as upstream code and stores local launch/configuration around it.

Relevant upstream files:

- `external/xr_teleoperate/teleop/teleop_hand_and_arm.py`
- `external/xr_teleoperate/teleop/robot_control/robot_arm.py`
- `external/xr_teleoperate/teleop/robot_control/robot_arm_ik.py`

Important H1-2 details from upstream:

- `H1_2_ArmController` commands 14 arm joints.
- Right arm joint order is:
  - `20`: right shoulder pitch
  - `21`: right shoulder roll
  - `22`: right shoulder yaw
  - `23`: right elbow pitch
  - `24`: right elbow roll
  - `25`: right wrist pitch
  - `26`: right wrist yaw
- `--motion` publishes to `rt/arm_sdk`.
- Without `--motion`, it publishes to `rt/lowcmd`.
- `rt/lowstate` is used for feedback.
- In `--motion` mode, upstream writes `q = 1.0` to reserved slot `27`.

This matches the wrist-control work in `robot_telemetry_web`: the right wrist yaw target is index `26`, and direct low command takeover requires releasing the active Unitree motion mode first.

