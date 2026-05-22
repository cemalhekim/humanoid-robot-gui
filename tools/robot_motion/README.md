# Robot Motion Tools

Small local scripts for running guarded Unitree H1-2 motion utilities.

## Run Directly On The Robot PC

Probe first. This does not publish motor commands:

```bash
cd /home/ch/Workspace/robot_telemetry_web/tools/robot_motion
python3 h1_2_right_wrist_spin.py enp0s31f6 --joint yaw --probe
```

Then run the motion:

```bash
cd /home/ch/Workspace/robot_telemetry_web/tools/robot_motion
python3 h1_2_right_wrist_spin.py enp0s31f6 --joint yaw --execute --i-understand-risk
```

Stop with `Ctrl-C`.

## Run From Laptop Over SSH

```bash
cd /home/ch/Workspace/robot_telemetry_web/tools/robot_motion
python3 run_robot_py.py h1_2_right_wrist_spin.py -- enp0s31f6 --joint yaw --execute --i-understand-risk
```

The runner copies the script to `~/robot_motion_tools` on the robot and runs it
with `unitree_sdk2_python` on `PYTHONPATH`.

## Useful Options

```bash
--joint yaw
--joint pitch
--joint roll
--min -0.5 --max 0.5
--period 3.0
--rate 200
--kp 18 --kd 1
```
