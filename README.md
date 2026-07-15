# Unitree H1-2 Telemetry and XR Operator Dashboard

This repository contains a local web dashboard, telemetry server, guarded robot
command surface, and Vision Pro / XR service deployment for the Unitree H1-2
robot.

The project is designed to:

- Read Unitree DDS telemetry from the robot PC.
- Show H1-2 robot state, joints, IMU, battery, hands, camera, and command
  history in a browser dashboard.
- Expose guarded right-wrist and H1 locomotion commands with explicit operator
  intent and runtime limits.
- Run Vision Pro / XR teleoperation services on the robot PC as user-level
  systemd services.
- Let a development laptop test the UI and static assets without live robot
  telemetry.

This is not a read-only application. Some HTTP endpoints can move the robot.
Run it only on trusted networks and only with an operator who can physically
observe the robot and understands the risk.

## Quick Map

Local development checkout:

```text
/Users/vodafone/Workspace/humanoid-robot-gui
```

Robot runtime checkout:

```text
/home/unitree/robot_telemetry_web
```

Robot network values used by the current setup:

```text
Robot Wi-Fi host: 10.2.100.142
Robot SSH user:  unitree
Robot DDS host:  192.168.123.164
Robot DDS iface: eth0
```

Main ports:

| Port | Service | URL |
| --- | --- | --- |
| 8088 | Dashboard HTTP, API, and SSE | `http://10.2.100.142:8088` |
| 8088 | Welcome page (choose Wi-Fi or Ethernet entry) | `http://10.2.100.142:8088/welcome` or `http://192.168.123.164:8088/welcome` |
| 8012 | XR / Vuer HTTPS and WSS | `https://10.2.100.142:8012/?ws=wss://10.2.100.142:8012` |
| 60001 | TeleImager WebRTC camera | `https://10.2.100.142:60001` |

## Repository Layout

```text
.
├── README.md
├── server.py
├── run_servers.py
├── kill_servers.py
├── Makefile
├── static/
│   ├── index.html
│   ├── app.js
│   ├── viewer.js
│   ├── styles.css
│   ├── assets/
│   ├── vendor/three/
│   └── models/h1_2_description/
├── deployment/
│   ├── install_robot_services.sh
│   ├── systemd/
│   ├── patch_*.py
│   ├── patch_*.sh
│   ├── robot_autoupdate.sh
│   └── xr_home_watchdog.py
├── docs/
├── teleoperation/vision_pro_control/
├── execution/semantic_teleoperation/
├── simulation/semantic_teleoperation/
├── robot_models/unitree_h1_2/
├── vendor/unitree_sdk2_python/
├── tools/
└── tests/
```

Important files:

- `server.py`: Python HTTP server, DDS subscribers, camera bridge, dashboard
  APIs, and command endpoints.
- `static/index.html`: Dashboard document structure.
- `static/app.js`: UI state management, API calls, SSE connection, loco
  controls, wrist controls, and dashboard rendering.
- `static/viewer.js`: Three.js H1-2 URDF viewer.
- `static/models/h1_2_description`: H1-2 URDF, XML, and STL model assets served
  directly to the browser.
- `run_servers.py`: Helper that kills stale servers and starts the dashboard in
  foreground or systemd mode.
- `kill_servers.py`: Helper that stops dashboard/server processes launched from
  this workspace.
- `deployment/install_robot_services.sh`: Robot-side installer for user systemd
  services and XR checkout patches.
- `deployment/systemd/*.service`: Robot runtime service definitions.
- `tests/test_contracts.py`: Contract tests for joint ordering, command limits,
  and API behavior.

## Features

The dashboard shows:

- Live H1-2 body telemetry from `rt/lowstate`.
- State for the 27 known H1-2 body motors: position, velocity, estimated
  torque, temperature, voltage, and motor mode.
- IMU fields: quaternion, gyroscope, accelerometer, roll/pitch/yaw, and
  temperature when exposed by the firmware.
- Robot metadata fields such as `version`, `mode_pr`, `mode_machine`, `tick`,
  `crc`, and wireless remote state.
- Battery/BMS values when the active firmware exposes them.
- Foot force and estimated foot force arrays when present in LowState.
- RH56BFX / Inspire hand telemetry from `rt/inspire/state`.
- A live Three.js H1-2 URDF viewer.
- Viewer presets for front, back, left, right, top, and bottom.
- Grid and auto-rotate controls for the viewer.
- A raw JSON state panel.
- A TeleImager camera panel.
- Links to the Vision Pro / XR Vuer page.
- H1 locomotion controls.
- Right-wrist controls.
- A telemetry recorder for sequential full-body motor samples, including legs,
  waist, arms, and RH56BFX hand samples.
- A 6-DOF hand target panel in the Recorder pose editor: clicking a hand ball
  (without dragging) opens a hovering panel with X/Y/Z position controls — with
  a Ground/Relative frame toggle (ground = X/Y from the pelvis axis and Z from
  the fixed floor; relative = offsets from the hand's pre-edit position) — and
  wrist roll/pitch/yaw sliders bounded by the real URDF joint limits. All
  panel edits drive the same IK and self-collision checks as dragging, and
  only change the preview pose.
- Command result and command history panels.

Dashboard command surfaces:

- `Release` (red, formerly "Chill Motors"): requests damp behavior — the arms
  go limp.
- `Home` (green): engages an arm_sdk hold whose target is always the arms'
  measured position at the moment the button is pressed, so it stiffens the
  arms in place without commanding motion. The hold runs the same closed-loop
  PID + gravity feed-forward corrector as arm replay (disable with
  `HOME_HOLD_CLOSED_LOOP=0`). Release or any new arm command stops the hold.
  Falls back to the legacy XR teleop home command when DDS is unavailable.
- `Stand Up` (formerly "Straight"): recovery after Release — sends the
  LocoClient `StandUp()` lock-stand request, the software equivalent of the
  RC left+up combo. The legacy XR straight-arm preset remains available at
  `POST /api/robot/straight`.
- `Loco Control`: ready, stand, start, damp, zero torque, velocity, target
  position, and odometry requests.
- `Right Wrist`: target position, relative step, gains, duration, command rate,
  and oscillation controls.
- `Camera`: TeleImager WebRTC preview and XR page entry points.

## Safety Model

This repository contains code that can move the robot.

- The dashboard serves HTTP.
- `/events` opens a Server-Sent Events telemetry stream.
- Read endpoints include `/api/state`, `/api/camera`, `/api/loco/status`, and
  `/api/wrist/status`.
- Motion-related POST endpoints include `/api/wrist/command`,
  `/api/wrist/stop`, `/api/loco/command`, `/api/robot/chill`,
  `/api/robot/home`, and `/api/robot/straight`.
- Wrist commands require `armed=true` and `i_understand_risk=true`.
- Loco and chill commands also fail closed when the robot-side LocoClient is not
  available or when command validation fails.
- Low-level `/lowcmd` body control is the highest-risk path. Prefer high-level
  `LocoClient` control for initial locomotion work.

Anyone who can reach the dashboard host can see robot state and can send HTTP
requests to exposed endpoints. This repository does not implement
authentication. Use only on a trusted robot network.

## Requirements

For local UI development:

- Python 3.10 or newer is recommended.
- A modern browser with WebGL support.
- No frontend build system is required.
- No `npm install` is required.

For live robot operation:

- Unitree SDK2 Python.
- CycloneDDS / Unitree DDS network access.
- A robot PC that can reach the H1-2 DDS network.
- User-level systemd services under the `unitree` user.
- The micromamba `tv` environment if using the deployment service files:

```text
/home/unitree/.micromamba/envs/tv
```

The server may also use this SDK path when present:

```text
~/unitree_sdk2_python
```

If the SDK is installed elsewhere, make sure `unitree_sdk2py` is importable in
the Python environment used to run `server.py`.

## Git and Checkout Preparation

The local development checkout is currently expected here:

```bash
cd /Users/vodafone/Workspace/humanoid-robot-gui
git status --short --branch
```

To prepare a new development machine:

```bash
mkdir -p ~/Workspace
cd ~/Workspace
git clone <repo-url> humanoid-robot-gui
cd humanoid-robot-gui
```

The expected runtime path on the robot PC is:

```bash
ssh unitree@10.2.100.142
cd /home/unitree
git clone <repo-url> robot_telemetry_web
cd /home/unitree/robot_telemetry_web
```

To update an existing robot checkout:

```bash
cd /home/unitree/robot_telemetry_web
git fetch origin
git status --short --branch
git pull --ff-only
```

Do not commit robot credentials, passwords, private keys, tokens, or other
secrets. Keep the robot SSH password outside the repository.

## Local Development

To start the dashboard without live robot telemetry:

```bash
cd /Users/vodafone/Workspace/humanoid-robot-gui
python3 run_servers.py --mode foreground --host 0.0.0.0 --port 8088 --no-kill-first
```

Open:

```text
http://127.0.0.1:8088
```

You can also start the server directly:

```bash
python3 -u server.py --host 127.0.0.1 --port 8090
```

If the local Mac environment does not have Unitree SDK2 Python or CycloneDDS,
robot telemetry will not connect. That is expected for UI-only work. The
`/api/state` response may show a disconnected state like:

```text
connected: false
sample_rate_hz: 0
Could not import Unitree SDK: No module named 'cyclonedds'
```

In that mode, you can still verify that the UI loads, static assets are served,
and the 3D model renders. It does not prove live robot telemetry is working.

## Running on the Robot

SSH into the robot PC:

```bash
ssh unitree@10.2.100.142
cd /home/unitree/robot_telemetry_web
```

Install and start the robot service set:

```bash
deployment/install_robot_services.sh
```

The installer:

- Updates the `unitreerobotics/xr_teleoperate` checkout when possible.
- Copies user service files into `~/.config/systemd/user`.
- Applies repository patch scripts to the XR teleoperation checkout.
- Configures TeleImager, Inspire hands, XR teleop, and the watchdog.
- Runs `systemctl --user daemon-reload`.
- Enables and restarts the services.
- Prints full systemd status for the core services.

If the installer stops because an XR patch no longer matches the upstream file,
and the service files are already installed, restart the services manually:

```bash
systemctl --user daemon-reload
systemctl --user enable --now robot-telemetry-web.service teleimager.service inspire-hands.service xr-teleop.service xr-home-watchdog.service
systemctl --user restart robot-telemetry-web.service teleimager.service inspire-hands.service xr-teleop.service xr-home-watchdog.service
```

Check service status:

```bash
systemctl --user --no-pager --full status \
  robot-telemetry-web.service \
  teleimager.service \
  inspire-hands.service \
  xr-teleop.service \
  xr-home-watchdog.service
```

The dashboard service runs this command on the robot:

```bash
/home/unitree/.micromamba/envs/tv/bin/python -u \
  /home/unitree/robot_telemetry_web/server.py \
  --host 0.0.0.0 \
  --port 8088 \
  --robot-host 192.168.123.164 \
  --camera-source eth0 \
  --camera-backend teleimager
```

Open these from a laptop on the robot Wi-Fi network:

```text
Dashboard: http://10.2.100.142:8088
XR / Vuer: https://10.2.100.142:8012/?ws=wss://10.2.100.142:8012
Camera:    https://10.2.100.142:60001
```

The browser may require you to open the XR or camera HTTPS page once and accept
the self-signed certificate.

## Services

| Service | Purpose | Main port/path |
| --- | --- | --- |
| `robot-telemetry-web.service` | Dashboard, HTTP API, DDS telemetry, guarded wrist/loco endpoints | `http://10.2.100.142:8088` |
| `robot-telemetry-web-autoupdate.timer` | Periodically updates the robot checkout from `origin/main` | systemd timer |
| `teleimager.service` | Unitree TeleImager WebRTC camera server | `https://10.2.100.142:60001` |
| `inspire-hands.service` | Inspire DFX/RH56 hand bridge | DDS hand topics |
| `xr-teleop.service` | Vision Pro / XR Vuer teleoperation server | `https://10.2.100.142:8012` |
| `xr-home-watchdog.service` | Watchdog for lost XR home/pose packets | XR port `8012`, dashboard API `8088` |

Robot logs are generally written under:

```text
/home/unitree/logs
```

Dashboard log:

```text
/home/unitree/logs/robot_telemetry_web.log
```

Watchdog log:

```text
/home/unitree/logs/xr_home_watchdog.log
```

## HTTP API

Read endpoints:

| Path | Type | Description |
| --- | --- | --- |
| `/` | HTML | Main dashboard |
| `/index.html` | HTML | Main dashboard |
| `/api/state` | JSON | Normalized live robot snapshot |
| `/api/camera` | JSON | Camera worker and backend state |
| `/api/ros-graph` | JSON | ROS graph snapshot, when available |
| `/api/wrist/status` | JSON | Right-wrist control state |
| `/api/loco/status` | JSON | LocoClient / locomotion state |
| `/api/recording/status` | JSON | Current telemetry recorder state |
| `/api/recording/files` | JSON | Available JSONL recording files |
| `/api/recording/files/<name>.jsonl` | JSONL | A single recording file for dashboard replay |
| `/events` | SSE | State stream, sent roughly every 100 ms |
| `/camera.mjpg` | MJPEG | Camera bridge MJPEG stream |
| `/models/...` | Static | URDF, XML, and STL model assets |
| `/vendor/...` | Static | Vendored Three.js modules |
| `/assets/...` | Static | Dashboard visual assets |

POST endpoints:

| Path | Description |
| --- | --- |
| `/api/wrist/command` | Sends a right-wrist target or oscillation command |
| `/api/wrist/stop` | Stops the active wrist command |
| `/api/robot/chill` | Requests damp/chill behavior |
| `/api/robot/home` | Holds the arms at their current position (arm_sdk; XR fallback) |
| `/api/robot/straight` | Requests the straight posture preset |
| `/api/loco/command` | Sends H1 LocoClient commands |
| `/api/xr/mode` | Requests an XR mode change |
| `/api/recording/start` | Starts a JSONL telemetry recording |
| `/api/recording/stop` | Stops the active telemetry recording |
| `/api/recording/pose` | Captures the current full-body pose as a single target point |
| `/api/recording/replay/robot` | Locked robot playback request; validates preview state but refuses physical playback until a safety controller exists |
| `/mcp` | MCP endpoint (off by default; see the MCP Endpoint section) |

State check:

```bash
curl -sS http://10.2.100.142:8088/api/state
```

Loco status:

```bash
curl -sS http://10.2.100.142:8088/api/loco/status
```

Wrist status:

```bash
curl -sS http://10.2.100.142:8088/api/wrist/status
```

Recording status:

```bash
curl -sS http://10.2.100.142:8088/api/recording/status
```

Start recording:

```bash
curl -sS -X POST http://10.2.100.142:8088/api/recording/start \
  -H 'Content-Type: application/json' \
  -d '{"label":"h1_2_full_body_hands"}'
```

Stop recording:

```bash
curl -sS -X POST http://10.2.100.142:8088/api/recording/stop
```

## Smart Plug (Home Assistant)

The dashboard's Robot Status grid has a **Smart Plug** card that switches the
showcase Sonoff plug (`SomoffSwitch2408`) through the lab Home Assistant at
`http://10.2.200.100`. The server proxies the calls so the HA token never
reaches the browser:

- `GET /api/smartplug/status` — current plug state (`on`/`off`/`unavailable`).
- `POST /api/smartplug/toggle` — toggle the plug via
  `POST /api/services/switch/toggle` on Home Assistant.

Configuration (service environment):

| Variable | Default | Meaning |
| --- | --- | --- |
| `HA_TOKEN` | empty | Home Assistant long-lived access token (HA profile → Security → Long-lived access tokens). Without it the card shows "Not set up" and the toggle stays disabled. |
| `HA_BASE_URL` | `http://10.2.200.100` | Home Assistant base URL. |
| `HA_SWITCH_ENTITY` | `switch.somoffswitch2408` | Entity id of the plug. Verify with `curl -H "Authorization: Bearer <token>" http://10.2.200.100/api/states \| python3 -m json.tool \| grep -i somoff`. |
| `HA_TIMEOUT_SECONDS` | `6` | Timeout for HA requests. |

Set the token on the robot in the service unit (then
`systemctl --user daemon-reload && systemctl --user restart
robot-telemetry-web.service`):

```ini
# ~/.config/systemd/user/robot-telemetry-web.service.d/override.conf, or the
# [Service] section of deployment/systemd/robot-telemetry-web.service
Environment=HA_TOKEN=<long-lived access token>
```

## MCP Endpoint

`POST /mcp` exposes the chat assistant's tools to any MCP (Model Context
Protocol) client over stateless streamable HTTP — same tool specs, same
dispatch (`run_chat_tool`), and therefore exactly the same guards as the
dashboard chat: the `chill_motors` confirm gate, ros2 name validation, and the
`LLM_TOOL_CHILL_ENABLED` flag all apply unchanged. This is the stable,
client-agnostic interface for LLM access to the robot; future motion skills
should be added here (as guarded tools) rather than in any one client.

Configuration (service environment):

| Variable | Default | Meaning |
| --- | --- | --- |
| `MCP_ENABLED` | `0` | Serve `POST /mcp`. Off by default so an autoupdate deploy never silently opens the endpoint. |
| `MCP_TOKEN` | empty | If set, requests must send `Authorization: Bearer <token>`. |

Connect from Claude Code on the operator Mac:

```bash
claude mcp add --transport http robot http://10.2.100.142:8088/mcp \
  --header "Authorization: Bearer <token>"
```

Or drive the tools with the on-prem LLM (Ollama on the AI host) using the
bundled agent CLI — an interactive REPL, or one-shot with `-p`:

```bash
python3 tools/mcp_agent.py --token <token> \
  -p "What is the robot's locomotion status right now?"
```

`tools/mcp_agent.py` is a stdlib-only MCP client: it discovers the tools via
`tools/list`, offers them to the model as OpenAI-style function specs, and
executes requested calls via `tools/call`. Defaults target the robot dashboard
(`--mcp-url`) and the AI host's Ollama (`--llm-url`, `--model`); run with
`--help` for options.

Protocol notes: stateless JSON responses only (no SSE stream, no sessions, no
resumability); notifications are acknowledged with `202`; `GET /mcp` returns
`405`. Supported methods: `initialize`, `ping`, `tools/list`, `tools/call`.

## Telemetry Recording

The Recorder page supports two capture modes:

- `Sequence`: the existing process recording mode. It records every available
  `rt/lowstate` sample over time and writes a `.jsonl` file.
- `Pose Point`: captures the robot's current full-body pose as one target point
  and writes a `.pose.json` file. When replayed in the dashboard, the red model
  shows the target pose and the green simulated trajectory moves from the live
  or neutral start pose toward that target.

Recordings are written under:

```text
recordings/
```

The directory is intentionally ignored by Git because recordings can become
large quickly.

Sequence JSONL files contain one record per line. The current schema uses these
record types:

- `recording_start`: metadata, schema name, body joint names, and hand joint
  names.
- `telemetry_sample`: one sampled `rt/lowstate` frame plus the latest
  `rt/inspire/state` hand state.
- `command_event`: dashboard command markers such as loco, wrist, home, chill,
  and XR mode requests.
- `recording_stop`: final sample and event counts.

Each `telemetry_sample` includes:

- Wall-clock `timestamp`.
- Monotonic `monotonic_ns` timestamp for precise ordering.
- Body motor rows for every `rt/lowstate` motor slot exposed by the robot,
  including legs, waist, and arms. Each row includes `index`, `name`, `mode`,
  `q`, `dq`, `ddq`, `tau`, `tau_est`, `temperature`, `vol`, `sensor`, and
  `reserve` when those fields are exposed by the active Unitree message.
- IMU values.
- Robot mode fields.
- Battery, foot force, and estimated foot force when exposed.
- RH56BFX / Inspire hand joint rows from the latest hand state.

The recorder is intended first for offline replay, simulation export, debugging,
and imitation-learning dataset preparation. Replaying raw joint trajectories
back onto the physical robot is intentionally not implemented here because it
requires additional controller ownership checks, interpolation, velocity and
torque limits, emergency stop handling, and simulation validation.

Dry-run replay planning is implemented. `POST /api/recording/replay/robot` with
`dry_run=true` reports the planned command path (`arm_sdk` or `lowcmd`),
trajectory validity, moving joints, lower-body movement, velocity/delta limits,
and a per-joint `kp`/`kd` gain plan. If the recording contains RH56BFX /
Inspire finger movement, the same response includes a parallel `hand_plan` for
`rt/inspire/cmd`. It still does not publish motor commands.

## H1 Locomotion Strategy

For Vision Pro walking, prefer the high-level Unitree H1 `LocoClient` layer.
Do not start with low-level body joint trajectories or direct `/lowcmd`
publishing; those can fight the onboard locomotion controller.

Current and planned control options:

| Option | Input | Robot path | Notes |
| --- | --- | --- | --- |
| XR controller motion | Vision Pro controller / thumbstick | XR teleop `--motion` -> `LocoClient.Move` | Fastest practical path when controller input is available |
| Hand gesture walking | Pinch, hand pose, palm direction | Gesture mapper -> `/api/loco/command` | Requires a deadman gesture and watchdog |
| Head/gaze walking | Head yaw or gaze plus deadman | Head mapper -> `/api/loco/command` | Must filter normal head motion |
| Dashboard joystick | Browser buttons | Dashboard -> `/api/loco/command` | Best for early supervised tests |
| Voice commands | Speech recognizer | Speech -> loco bridge | Should not be the only stop channel |
| Split control | Physical remote plus XR hands | Remote for walking, XR for manipulation | Conservative demo mode |
| ROS2 teleop node | ROS input source | ROS node -> Unitree API topics | Useful for logging and replay |
| Waypoint/click-to-walk | Gaze or map target | Odom/target position API | Use after velocity walking is stable |
| Custom low-level gait | Learned/generated trajectories | MuJoCo/sim -> `/lowcmd` | Highest risk; simulate first |

Recommended first live test flow:

1. Confirm the dashboard and robot-side LocoClient are available.
2. Confirm `Damp` or `Stop Move` works.
3. Send a tiny velocity command, for example `vx=0.05`, `vy=0`, `yaw=0`, for
   less than one second.
4. Immediately send `StopMove()` / `stop_move`.
5. Do not increase velocity until watchdog, lost-tracking, and deadman behavior
   are verified.

## DDS and Robot Topics

The server starts with DDS domain `0` by default:

```bash
python3 -u server.py --host 127.0.0.1 --port 8090 --domain 0
```

Use a different domain if your robot setup requires it:

```bash
python3 -u server.py --host 127.0.0.1 --port 8090 --domain 1
```

Primary DDS topics subscribed by the dashboard:

| Topic | Message Type | Purpose |
| --- | --- | --- |
| `rt/lowstate` | `unitree_hg.msg.dds_.LowState_` | H1-2 body telemetry, motors, IMU, forces, battery |
| `rt/inspire/state` | `unitree_go.msg.dds_.MotorStates_` | RH56BFX / Inspire hand state |
| `rt/inspire/cmd` | `unitree_go.msg.dds_.MotorCmds_` | Planned RH56BFX / Inspire hand command path |

High-level locomotion uses Unitree API request/response topics. The dashboard
HTTP endpoint is `/api/loco/command`, while the robot-side DDS API path is
conceptually `rt/api/loco/request` and `rt/api/loco/response`.

## Dashboard Panels

### Header

- Connection state.
- Snapshot age.
- Sample rate.
- Motor count.
- Total sample count.
- Latest error message.

### Robot

- `version`
- `mode_pr`
- `mode_machine`
- `tick`
- `crc`
- `wireless_remote`

### IMU

- Quaternion.
- Gyroscope.
- Accelerometer.
- Roll / pitch / yaw.
- Temperature.

### Battery

When exposed by firmware:

- `version_h`
- `version_l`
- `bms_status`
- `soc`
- `current`
- `cycle`
- `temperature`

### Hands

When `rt/inspire/state` is available:

- Hand topic connected/disconnected state.
- Hand sample count.
- Joint count.
- RH56BFX finger joint positions.

Hand joint order:

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

### Forces

- `foot_force`
- `foot_force_est`

### Motor Table

The motor table shows:

- Index.
- Joint name.
- Mode.
- Position `q`.
- Velocity `dq`.
- Estimated torque `tau_est`.
- Temperature.
- Voltage.

Body joint order:

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

Additional slots are named `ReservedMotorSlot<N>`.

### 3D Viewer

The viewer loads:

```text
static/models/h1_2_description/h1_2.urdf
```

Features:

- STL mesh rendering.
- Body motor telemetry to URDF joint mapping.
- Hand telemetry to RH56BFX finger joint mapping.
- Orbit, zoom, and view cube controls.
- Grid toggle.
- Auto-rotate toggle.
- Mesh loaded/failed status.

### Camera and XR

The camera panel:

- Embeds the TeleImager WebRTC preview.
- Links to the direct camera page.
- Links to the XR / Vuer page.
- Notes that the browser may need certificate trust before embedding works.

XR page:

```text
https://10.2.100.142:8012/?ws=wss://10.2.100.142:8012
```

Camera page:

```text
https://10.2.100.142:60001
```

## Helper Commands

Run tests:

```bash
cd /Users/vodafone/Workspace/humanoid-robot-gui
python3 -m unittest discover -s tests -p 'test_*.py'
```

Run tests through the Makefile:

```bash
make test
```

Run the production gate:

```bash
make production-gate
```

Generate synthetic replay-compatible recordings without a robot:

```bash
python3 tools/generate_synthetic_recordings.py --frames 240 --prefix synthetic-h1-2
```

This writes one sequence `.jsonl` file and one pose-point `.pose.json` file into
`recordings/` so the Recorder replay UI can be tested without live DDS data.

Convert a recording into CSV tables:

```bash
python3 tools/recording_to_csv.py recordings/example.jsonl
```

The converter writes `samples.csv`, `body_motors.csv`, `hand_joints.csv`,
`imu.csv`, `forces.csv`, and `events.csv` into a sibling output directory.

Stop local dashboard servers:

```bash
python3 kill_servers.py
```

Stop robot-side dashboard processes:

```bash
cd /home/unitree/robot_telemetry_web
python3 kill_servers.py
```

Check static assets:

```bash
curl -sSI http://127.0.0.1:8088/vendor/three/three.module.js
curl -sSI http://127.0.0.1:8088/models/h1_2_description/h1_2.urdf
```

## Troubleshooting

### Dashboard stays disconnected or at 0 Hz

Read the raw state endpoint:

```bash
curl -sS http://10.2.100.142:8088/api/state
```

Check the `error` field. If you are running locally on a Mac, `cyclonedds` or
`unitree_sdk2py` import errors can be expected. Live robot telemetry should run
from the robot PC.

Being able to reach the robot Wi-Fi host does not guarantee DDS visibility.
H1-2 body DDS traffic may be on the robot PC's `eth0` control link. The robot
runtime command therefore uses `--camera-source eth0` and
`--robot-host 192.168.123.164`.

### Unitree SDK cannot be imported

Check:

```bash
python3 -c "import unitree_sdk2py; print('ok')"
```

Expected SDK paths include:

```text
/home/unitree/unitree_sdk2_python
vendor/unitree_sdk2_python
```

Make sure the correct Python environment is active.

### DDS subscriber cannot initialize

Check:

- Is the robot powered on?
- Is the robot PC on the correct network?
- Is the `eth0` link active?
- Is the DDS domain correct?
- Is a firewall blocking DDS traffic?
- Is `192.168.123.164` reachable from the robot PC?

### Hand panel is disconnected

Body telemetry and hand telemetry are independent. The body can be connected
while the hand panel is disconnected.

Check:

- Are the RH56BFX hands connected?
- Is `inspire-hands.service` active?
- Is `rt/inspire/state` being published?
- Do the serial device paths match the systemd service configuration?

### Camera does not open

Check:

- Is `teleimager.service` active?
- Does `https://10.2.100.142:60001` open directly?
- Is the browser blocking the certificate?
- Do `XR_TELEOP_CERT` and `XR_TELEOP_KEY` exist?
- Is `camera-backend` set as expected: `teleimager`, `ros2`, or `auto`?

### XR page does not open

Check:

- Is `xr-teleop.service` active?
- Is port `8012` listening?
- Are you using this URL format?

```text
https://10.2.100.142:8012/?ws=wss://10.2.100.142:8012
```

- Did the browser grant camera, hand tracking, and WebXR permissions?

### 3D model does not render

Check:

```bash
curl -sSI http://127.0.0.1:8088/vendor/three/three.module.js
curl -sSI http://127.0.0.1:8088/vendor/three/OrbitControls.js
curl -sSI http://127.0.0.1:8088/vendor/three/STLLoader.js
curl -sSI http://127.0.0.1:8088/models/h1_2_description/h1_2.urdf
```

Browser WebGL support must also be enabled.

### Port is already in use

Start on another port:

```bash
python3 -u server.py --host 127.0.0.1 --port 8091
```

Or clean up existing processes first:

```bash
python3 kill_servers.py
```

## Development Notes

The frontend intentionally has no build step:

- Plain HTML, CSS, and browser JavaScript.
- Vendored Three.js modules.
- No Node/npm requirement.
- Easier deployment on robot PCs where extra frontend tooling is undesirable.

After code changes, run at least:

```bash
python3 -m unittest discover -s tests -p 'test_*.py'
git status --short --branch
```

When changing README or operator documentation, verify that runtime paths,
endpoint names, and systemd service files still agree with the code.

## Model Asset Notes

The browser uses these H1-2 model assets directly:

```text
static/models/h1_2_description
```

The source robot model assets copied from the ROS workspace are kept here:

```text
robot_models/unitree_h1_2
```

`static/models/h1_2_description` is served directly by the dashboard. Preserve
Unitree Robotics attribution and the included model README when updating those
files.

## Recording Replay Digital Twin

The Recorder page includes a separate digital twin viewer for replaying saved
JSONL recordings. This replay viewer is intentionally independent from the live
robot telemetry stream and from all robot command paths.

Replay workflow:

1. Open the `Recorder` page.
2. Choose a JSONL or `.pose.json` file from the auto-refreshing file selector.
3. The selected file loads automatically.
4. Click `Simulate Trajectory`.
5. The green simulated trajectory first moves from the blue/reference pose to
   the red target frame. For sequence recordings, red waits at frame 0 until
   green reaches it, then both advance through the sequence together.
6. After the preview reaches the end, the `Move Robot` button unlocks. The
   current implementation still refuses physical playback at the server because
   raw joint replay needs a dedicated safety controller first.

During replay, the dashboard converts each `telemetry_sample` row into a viewer
snapshot. The replay page shows three H1-2 models in the same scene: a
translucent blue reference model, a red target/recorded model, and a green
simulated trajectory model. This makes both the target and the approach path
visible before any physical motion request is allowed.

No replay data is sent to the physical robot yet. Raw trajectory playback must
first add interpolation, joint/velocity/torque limits, controller ownership
checks, emergency stop supervision, and simulation validation.

## License

No explicit project license is currently included in this repository. Add one
before broader distribution, external contributions, or commercial use.
