# Unitree Telemetry Web Dashboard

Local web dashboard for Unitree H1-2 telemetry.

## Features

- Live `rt/lowstate` telemetry stream
- Motor table for H1-2 body motors
- IMU, robot mode, battery availability, force, and hand-state panels
- RH56BFX hand-state subscription via `rt/inspire/state`
- Browser-native Three.js H1-2 URDF viewer
- View cube controls for front, back, left, right, top, and bottom camera presets
- Collapsible raw JSON telemetry drawer

## Run

```bash
cd ~/Workspace/robot_telemetry_web
python3 -u server.py --host 127.0.0.1 --port 8090
```

Open:

```text
http://127.0.0.1:8090
```

The app is read-only. It subscribes to telemetry topics and does not publish robot commands.
