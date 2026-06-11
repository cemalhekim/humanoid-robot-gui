# H1-2 Vision Pro Control

Local integration repo for Unitree H1-2 arm teleoperation with Apple Vision Pro / XR devices.

The upstream Unitree implementation is kept as a git submodule at:

```text
external/xr_teleoperate
```

Unitree's Isaac Lab simulator is kept as a second submodule at:

```text
external/unitree_sim_isaaclab
```

## Current Defaults

- Robot: Unitree H1-2
- Arm profile: `H1_2`
- Host/image server IP: `192.168.123.164`
- DDS network interface: `eth0`
- XR input mode: `hand`
- Display mode: `pass-through`

## Launch

From this repo:

```bash
./scripts/run_h1_2_vision_pro.sh
```

The script runs Unitree's:

```bash
python teleop_hand_and_arm.py --input-mode hand --display-mode pass-through --arm H1_2 --img-server-ip 192.168.123.164 --network-interface eth0 --motion
```

For robot-hosted teleoperation, keep DDS on the robot Ethernet interface and set
the image/Vuer address to the robot's Wi-Fi address. On the current robot this
means `--network-interface eth0` and `--img-server-ip 10.2.100.142`.

Open Vision Pro Safari to:

```text
https://10.2.100.142:8012/?ws=wss://10.2.100.142:8012
```

Use `DISPLAY_MODE=immersive` only when you intentionally want the robot head
camera to fill the headset. The default `pass-through` mode keeps the same
hand/controller teleoperation path but uses the headset's real environment view
instead of streaming robot camera video into the XR scene.

If the Vuer page loads but the XR session does not stay connected, verify that
Vision Pro trusts the `televuer` root CA, then retry through:

```text
https://vuer.ai?ws=wss://10.2.100.142:8012
```

Use `--no-motion` if you only want arm control without entering Unitree's motion/debug coexistence path:

```bash
./scripts/run_h1_2_vision_pro.sh --no-motion
```

## Simulation

Full simulation requires Isaac Sim / Isaac Lab and Unitree's simulator assets. After that environment is installed, start the H1-2 simulator:

```bash
./scripts/run_h1_2_sim.sh
```

Then start XR teleop in simulation mode:

```bash
./scripts/run_h1_2_xr_sim.sh
```

Current local blocker: this host does not expose `conda` or `nvidia-smi`, so the full Isaac Lab simulation cannot be launched yet from the default shell.

## Upstream Control Notes

Unitree's H1-2 arm controller uses `unitree_hg` low command messages and these topics:

- `rt/arm_sdk` when launched with `--motion`
- `rt/lowcmd` when launched without `--motion`
- `rt/lowstate` for motor state feedback

For H1-2, right wrist yaw is motor index `26`. Upstream also sets motor slot `27` to `q = 1.0` while publishing through `rt/arm_sdk`; that appears to act as the arm SDK weight/enabler.

## Install Reminder

Follow upstream setup in `external/xr_teleoperate/README.md`, including:

- Python/conda environment
- `unitree_sdk2_python`
- `teleop/teleimager`
- `teleop/televuer`
- HTTPS certificates for Apple Vision Pro
- Isaac Sim / Isaac Lab for simulation
