# Unitree Telemetry Web Dashboard

A local, read-only web dashboard for monitoring Unitree H1-2 robot telemetry from a browser. The app runs a small Python HTTP server on the robot PC, subscribes to Unitree DDS telemetry topics through the Unitree SDK, and streams the latest robot state to a browser UI with a live motor table, status panels, raw JSON, and a browser-native Three.js URDF viewer.

The dashboard is intended for observation, debugging, and operator situational awareness. It does not publish robot commands.

## What It Shows

- Live `rt/lowstate` telemetry from the Unitree H1-2 body controller.
- Motor state for all known H1-2 body joints, including position, velocity, estimated torque, temperature, voltage, and mode.
- IMU data, including quaternion, gyroscope, accelerometer, roll/pitch/yaw, and temperature when exposed by the firmware.
- Robot metadata fields such as version, controller mode, machine mode, tick, CRC, and wireless remote state when available.
- Battery/BMS information when the active LowState firmware exposes it.
- Foot force and estimated foot force arrays when present in the LowState message.
- RH56BFX hand telemetry from `rt/inspire/state`, including per-finger joint positions when the Inspire hand service is running.
- A live Three.js H1-2 URDF viewer that moves body and hand joints from the incoming telemetry.
- View cube controls for front, back, left, right, top, and bottom camera presets.
- Grid and auto-rotate controls for the 3D view.
- Collapsible raw JSON output for direct inspection of the current normalized telemetry snapshot.

## Safety Model

This application is intentionally read-only.

- It subscribes to telemetry topics.
- It serves HTTP, JSON, static assets, and Server-Sent Events.
- It does not send motion commands.
- It does not write to Unitree DDS command topics.
- It does not change robot mode, gait, balance state, arm state, hand state, or motor targets.

Even though the app is read-only, run it only on trusted networks. The dashboard exposes live robot state over HTTP to any client that can reach the configured host and port.

## Repository Layout

```text
.
├── README.md
├── server.py
└── static
    ├── index.html
    ├── app.js
    ├── viewer.js
    ├── styles.css
    ├── vendor/three
    │   ├── three.module.js
    │   ├── OrbitControls.js
    │   └── STLLoader.js
    └── models/h1_2_description
        ├── h1_2.urdf
        ├── h1_2_with_FTP_hand.urdf
        ├── h1_2.xml
        ├── h1_2_handless.xml
        └── meshes/*.STL
```

Important files:

- `server.py` starts the Unitree DDS subscribers and HTTP server.
- `static/index.html` defines the dashboard layout.
- `static/app.js` connects to `/api/state` and `/events`, then renders the status panels, motor table, and raw JSON.
- `static/viewer.js` loads the H1-2 URDF, maps telemetry joint names to URDF joints, and animates the Three.js model.
- `static/styles.css` contains the dashboard styling.
- `static/vendor/three/*` contains vendored Three.js browser modules so the page can run without a package manager.
- `static/models/h1_2_description/*` contains the H1-2 robot model assets used by the viewer.

## Consolidated Workspace

Additional robot workspace projects are organized in this repo under:

- `teleoperation/vision_pro_control` for Vision Pro / XR teleoperation control code.
- `execution/semantic_teleoperation` for real/sim execution dependencies and Unitree integration sources.
- `simulation/semantic_teleoperation` for H1 simulation, Gazebo, MoveIt, and MuJoCo scripts.
- `robot_models/unitree_h1_2` for the Unitree H1-2 model source copied from the ROS workspace.
- `vendor/unitree_sdk2_python` for the Unitree SDK2 Python source snapshot.
- `tools/rh56` for RH56 hand utility scripts.

See `docs/workspace_inventory.md` for details about what was copied and what generated artifacts were intentionally left out.

## Requirements

Runtime requirements:

- Python 3.10 or newer recommended.
- Unitree SDK2 Python installed on the machine that can see the robot DDS network.
- Network access to the Unitree DDS domain used by the robot.
- A modern browser with WebGL support for the 3D viewer.

The server automatically prepends this SDK path if it exists:

```text
~/unitree_sdk2_python
```

If your SDK is installed elsewhere, make sure `unitree_sdk2py` is importable in the Python environment used to run `server.py`.

## DDS Topics

The server subscribes to:

| Topic | Message Type | Purpose |
| --- | --- | --- |
| `rt/lowstate` | `unitree_hg.msg.dds_.LowState_` | Main H1-2 body telemetry, motor state, IMU, robot mode, battery, and force fields. |
| `rt/inspire/state` | `unitree_go.msg.dds_.MotorStates_` | RH56BFX / Inspire hand state, if the hand service is running. |

If the hand topic is not available, the dashboard still works. The hand panel will show a disconnected state and a note telling you to start the Inspire hand service if the RH56BFX hands are connected over serial.

## Quick Start

From the robot PC or another machine that has access to the same Unitree DDS network:

```bash
cd ~/Workspace/robot_telemetry_web
python3 -u server.py --host 127.0.0.1 --port 8090
```

Open:

```text
http://127.0.0.1:8090
```

The server prints a local URL and a LAN URL:

```text
Unitree telemetry dashboard
Listening on http://127.0.0.1:8090
Try from another machine: http://<detected-lan-ip>:8090
Press Ctrl+C to stop.
```

## LAN Access

To view the dashboard from another computer on the same network, bind the server to all interfaces:

```bash
cd ~/Workspace/robot_telemetry_web
python3 -u server.py --host 0.0.0.0 --port 8090
```

Then open the LAN URL printed by the server, for example:

```text
http://192.168.1.42:8090
```

Use a trusted network. The server does not implement authentication.

## DDS Domain

By default the server initializes Unitree DDS with domain `0`.

```bash
python3 -u server.py --host 127.0.0.1 --port 8090 --domain 0
```

If your robot or network setup uses a different DDS domain, pass it explicitly:

```bash
python3 -u server.py --host 127.0.0.1 --port 8090 --domain 1
```

## HTTP API

The server exposes a small HTTP surface:

| Path | Type | Description |
| --- | --- | --- |
| `/` | HTML | Main dashboard page. |
| `/index.html` | HTML | Same dashboard page. |
| `/api/state` | JSON | Current normalized telemetry snapshot. |
| `/events` | Server-Sent Events | Continuous telemetry stream, sent about every 100 ms. |
| `/app.js` | JavaScript | Dashboard rendering logic. |
| `/viewer.js` | JavaScript module | Three.js URDF viewer logic. |
| `/styles.css` | CSS | Dashboard styles. |
| `/models/...` | Static assets | URDF, XML, STL, and image files for the H1-2 model. |
| `/vendor/...` | Static assets | Vendored Three.js modules. |

Example:

```bash
curl -sS http://127.0.0.1:8090/api/state
```

The JSON shape is normalized for the UI. It includes fields such as:

```json
{
  "connected": true,
  "timestamp": 1710000000.0,
  "samples": 1200,
  "sample_rate_hz": 499.8,
  "motor_count": 27,
  "motors": [],
  "imu": {},
  "robot": {},
  "battery": {},
  "hands": {}
}
```

Unavailable, non-finite, or firmware-specific fields are handled defensively so the UI can continue rendering when a robot exposes a different LowState variant.

## Dashboard Panels

### Header Status

The header shows:

- Connection state.
- Snapshot age in seconds.
- Estimated sample rate in Hz.
- Motor count and total samples when connected.
- The latest subscriber error when disconnected.

### Robot Panel

Shows normalized top-level LowState fields when available:

- `version`
- `mode_pr`
- `mode_machine`
- `tick`
- `crc`
- `wireless_remote`

### IMU Panel

Shows available IMU fields:

- `quaternion`
- `gyroscope`
- `accelerometer`
- `rpy`
- `temperature`

### Battery Panel

Shows BMS data when the firmware exposes `bms_state`:

- `version_h`
- `version_l`
- `bms_status`
- `soc`
- `current`
- `cycle`
- `temperature`

If `bms_state` is not exposed, the panel reports that the active firmware does not provide that field.

### RH56BFX Hands Panel

Shows the state of the `rt/inspire/state` subscriber:

- Whether the topic is connected.
- Number of received hand samples.
- Joint count.
- Per-joint hand positions.

Known hand joints are mapped as:

| Index | Name |
| --- | --- |
| 0 | `RightPinky` |
| 1 | `RightRing` |
| 2 | `RightMiddle` |
| 3 | `RightIndex` |
| 4 | `RightThumbBend` |
| 5 | `RightThumbRotation` |
| 6 | `LeftPinky` |
| 7 | `LeftRing` |
| 8 | `LeftMiddle` |
| 9 | `LeftIndex` |
| 10 | `LeftThumbBend` |
| 11 | `LeftThumbRotation` |

### Forces Panel

Shows:

- `foot_force`
- `foot_force_est`

These fields appear only when the LowState message exposes them.

### Motor Table

The motor table shows:

- Motor index.
- Human-readable joint name.
- Motor mode.
- Position `q`.
- Velocity `dq`.
- Estimated torque `tau_est`.
- Temperature.
- Voltage.

Use the filter box to search by joint index or joint name.

Known body motor indexes are mapped as:

| Index | Name |
| --- | --- |
| 0 | `LeftHipYaw` |
| 1 | `LeftHipPitch` |
| 2 | `LeftHipRoll` |
| 3 | `LeftKnee` |
| 4 | `LeftAnklePitch` |
| 5 | `LeftAnkleRoll` |
| 6 | `RightHipYaw` |
| 7 | `RightHipPitch` |
| 8 | `RightHipRoll` |
| 9 | `RightKnee` |
| 10 | `RightAnklePitch` |
| 11 | `RightAnkleRoll` |
| 12 | `WaistYaw` |
| 13 | `LeftShoulderPitch` |
| 14 | `LeftShoulderRoll` |
| 15 | `LeftShoulderYaw` |
| 16 | `LeftElbow` |
| 17 | `LeftWristRoll` |
| 18 | `LeftWristPitch` |
| 19 | `LeftWristYaw` |
| 20 | `RightShoulderPitch` |
| 21 | `RightShoulderRoll` |
| 22 | `RightShoulderYaw` |
| 23 | `RightElbow` |
| 24 | `RightWristRoll` |
| 25 | `RightWristPitch` |
| 26 | `RightWristYaw` |

Additional motor slots are labeled `ReservedMotorSlot<N>`.

### Live Robot Viewer

The viewer loads:

```text
static/models/h1_2_description/h1_2.urdf
```

It renders STL meshes through Three.js and applies telemetry values to matching URDF joints. Body motors are mapped from the LowState joint names to URDF joint names. Hand telemetry is mapped to the corresponding RH56BFX finger joints.

Viewer controls:

- Drag to orbit.
- Scroll to zoom.
- Use the view cube labels to jump to front, back, left, right, top, or bottom views.
- Use `Grid` to toggle the floor grid.
- Use `Rotate` to toggle auto-rotation.

The viewer status line reports:

- Loaded model.
- Live/waiting state.
- Body motor count.
- Hand joint count.
- Sample rate.
- Mesh load count.
- Failed mesh count.

## Running Without Robot Telemetry

The web server can still start without live robot data. If the Unitree SDK cannot be imported or the DDS subscriber cannot initialize, `/api/state` and the UI report a disconnected state with an error message.

This is useful for checking that static assets, the HTML layout, and the Three.js model are served correctly.

## Troubleshooting

### `Could not import Unitree SDK`

The Python process cannot import `unitree_sdk2py`.

Check that the SDK exists at:

```text
~/unitree_sdk2_python
```

Or activate the Python environment where the SDK is installed before running the server.

### `Could not initialize DDS subscriber`

The SDK imported successfully, but DDS could not subscribe to the configured topics.

Check:

- The robot is powered and reachable on the network.
- The correct network interface is being used for Unitree DDS traffic.
- The DDS domain passed with `--domain` matches the robot setup.
- No firewall rule is blocking the required DDS traffic.

### Dashboard Stays Disconnected

Open the raw state endpoint:

```bash
curl -sS http://127.0.0.1:8090/api/state
```

Look for the `error` field. The UI displays the same state, but the raw JSON is easier to inspect when debugging.

### Hand Panel Shows Disconnected

The body telemetry and hand telemetry are independent subscriptions. The body dashboard can be live while the hand panel is disconnected.

Check that:

- The RH56BFX hands are connected.
- The Inspire hand service is running.
- The `rt/inspire/state` topic is being published.

### 3D Model Does Not Render

Check:

- Browser WebGL support is enabled.
- `/vendor/three/three.module.js` returns HTTP 200.
- `/vendor/three/OrbitControls.js` returns HTTP 200.
- `/vendor/three/STLLoader.js` returns HTTP 200.
- `/models/h1_2_description/h1_2.urdf` returns HTTP 200.
- STL mesh requests under `/models/h1_2_description/meshes/` return HTTP 200.

Example:

```bash
curl -sSI http://127.0.0.1:8090/models/h1_2_description/h1_2.urdf
```

### Port Already In Use

Use a different port:

```bash
python3 -u server.py --host 127.0.0.1 --port 8091
```

## Development Notes

The app intentionally avoids a frontend build step:

- No npm install is required.
- Three.js modules are vendored under `static/vendor/three`.
- The UI is plain HTML, CSS, and browser JavaScript.
- The server uses Python standard library HTTP primitives.

This keeps deployment simple on robot PCs where installing extra frontend tooling is undesirable.

## Common Development Commands

Run the server locally:

```bash
python3 -u server.py --host 127.0.0.1 --port 8090
```

Check the current state endpoint:

```bash
curl -sS http://127.0.0.1:8090/api/state
```

Check static asset headers:

```bash
curl -sSI http://127.0.0.1:8090/vendor/three/three.module.js
curl -sSI http://127.0.0.1:8090/models/h1_2_description/h1_2.urdf
```

Check git status:

```bash
git status --short --branch
```

Publish changes:

```bash
git add README.md
git commit -m "Expand telemetry dashboard README"
git push
```

## Notes On Model Assets

The H1-2 description assets under `static/models/h1_2_description` are used directly by the browser. The dashboard serves URDF, XML, image, and STL files through the Python server.

The included model README credits Unitree Robotics for the H1-2 robot description package. Keep those assets and attribution intact when updating the model files.

## License

No explicit project license is currently included in this repository. Add one before distributing the project more broadly or accepting external contributions.
