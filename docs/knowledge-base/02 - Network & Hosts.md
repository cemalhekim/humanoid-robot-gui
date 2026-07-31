---
tags: [network, hosts, ports, deployment, systemd]
summary: Hosts, IP addresses, ports, service map, and the 60 s auto-update timer that pulls origin/main.
---

# 02 - Network & Hosts

## Hosts

| Host | Address | Role | Notes |
| --- | --- | --- | --- |
| Robot PC (Ethernet / DDS) | `192.168.123.164` | DDS control link (`eth0`), dashboard | **Reliable** path; H1-2 body DDS traffic is on this link |
| Robot PC (Wi-Fi) | `10.2.100.142` | Dashboard, XR, camera over Wi-Fi | **Flaky** — reachable ≠ DDS-visible |
| AI host ("AI-DEV") | `10.2.125.3` | On-prem LLM + STT/TTS + YOLO detection | Ollama, faster-whisper, TTS, detect |
| Home Assistant | `10.2.200.100` | Showcase Sonoff smart plug | Proxied via `/api/smartplug/*` |
| Jetson AGX Orin | `192.168.123.100` (eth, static) / `10.2.100.178` (Wi-Fi, DHCP) | Edge GPU box (JetPack 5.1.2) | On `Vaderfone@CSIG` Wi-Fi + robot subnet — see 29 - Jetson AGX Orin |

- Jetson SSH user: `jetson` (`ssh jetson` alias on the Mac; key auth via `id_ed25519_robot`).

## Lab Wi-Fi (Vaderfone@CSIG)

Our T+M (test & measurement) devices at Vodafone must be connected to the
**`Vaderfone@CSIG`** Wi-Fi SSID. The passphrase is a secret and lives only in
the local credentials store on the Mac (memory note `credentials.md`) — never
in this repo. The Jetson AGX Orin already carries a saved NM profile for this
SSID (see 29 - Jetson AGX Orin).

- Robot SSH user: `unitree` (`ssh unitree@10.2.100.142` or `@192.168.123.164`).
- Robot runtime checkout: `/home/unitree/robot_telemetry_web`.
- Local dev checkout: `/Users/vodafone/Workspace/humanoid-robot-gui`.

> [!warning] Wi-Fi vs Ethernet
> Being able to reach the robot Wi-Fi host (`10.2.100.142`) does **not**
> guarantee DDS visibility. H1-2 body DDS is on the robot PC's `eth0` control
> link, so the robot runtime uses `--robot-host 192.168.123.164` and
> `--camera-source eth0`. The Wi-Fi link is known to be flaky — this is a
> central design constraint for the tracking feature's
> staleness handling.

## Ports

| Port | Service | URL |
| --- | --- | --- |
| **8088** | Dashboard HTTP, API, SSE | `http://10.2.100.142:8088` / `http://192.168.123.164:8088` |
| 8088 | Welcome page (choose Wi-Fi/Ethernet) | `.../welcome` |
| **8012** | XR / Vuer HTTPS + WSS | `https://10.2.100.142:8012/?ws=wss://10.2.100.142:8012` |
| **60001** | TeleImager WebRTC camera | `https://10.2.100.142:60001` |
| **11434** | AI host: Ollama LLM | `http://10.2.125.3:11434` (`LLM_BASE_URL`) |
| **8188** | AI host: YOLOv8n detect | `http://10.2.125.3:8188/detect` (`TRACKING_DETECT_URL`) |
| 8001 | AI host: STT (faster-whisper) | `http://10.2.125.3:8001` (`LLM_STT_BASE_URL`, disabled by default) |
| 8002 | AI host: TTS | `http://10.2.125.3:8002` (`LLM_TTS_BASE_URL`, disabled by default) |
| 80 | Home Assistant | `http://10.2.200.100` (`HA_BASE_URL`) |

The robot dashboard runtime command:

```bash
/home/unitree/.micromamba/envs/tv/bin/python -u \
  /home/unitree/robot_telemetry_web/server.py \
  --host 0.0.0.0 --port 8088 \
  --robot-host 192.168.123.164 \
  --camera-source eth0 --camera-backend teleimager
```

DDS domain defaults to `0` (`--domain 0`).

## systemd service map

User-level systemd services under the `unitree` user (installed by
`deployment/install_robot_services.sh`):

| Service | Purpose | Port/path |
| --- | --- | --- |
| `robot-telemetry-web.service` | Dashboard, HTTP API, DDS telemetry, guarded endpoints | `:8088` |
| `robot-telemetry-web-autoupdate.timer` | Pulls `origin/main` periodically | systemd timer |
| `teleimager.service` | TeleImager WebRTC camera | `:60001` |
| `inspire-hands.service` | Inspire DFX/RH56 hand bridge | DDS hand topics |
| `xr-teleop.service` | Vision Pro / XR Vuer teleop | `:8012` |
| `xr-home-watchdog.service` | Watchdog for lost XR home/pose packets | `:8012` + API `:8088` |

`XR_MOTION_SERVICES = ("xr-home-watchdog.service", "xr-teleop.service")` are the
two units suspended before dashboard motion — see 03 - Safety Interlocks.

Logs: `/home/unitree/logs/` (dashboard: `robot_telemetry_web.log`, watchdog: `xr_home_watchdog.log`).

## Auto-update timer (auto-deploy)

The robot **pulls and redeploys `origin/main` every ~60 seconds**. This is how
code reaches the robot — see 08 - Development Workflow.

`deployment/systemd/robot-telemetry-web-autoupdate.timer`:

```ini
[Timer]
OnBootSec=45s
OnUnitActiveSec=60s
Unit=robot-telemetry-web-autoupdate.service
```

The oneshot service runs `deployment/robot_autoupdate.sh`, which:

1. `git fetch origin main` (exits quietly on failure).
2. If `HEAD != origin/main`: `git checkout -B main origin/main` + `git reset --hard origin/main` (marks changed).
3. Runs `deployment/update_xr_teleoperate.sh` (rc `10` also marks changed).
4. If changed, runs `deployment/install_robot_services.sh` to re-apply units/patches and restart services.

> [!warning] Auto-deploy implications
> Anything pushed to `main` reaches the robot within ~60 s and restarts
> services. This is why risky features ship **dark** behind default-off flags
> (e.g. `MCP_ENABLED=0`, and the planned `TRACKING_ENABLED=0`) so an autoupdate
> deploy never silently opens a new surface. See 05 - Chat & MCP Tools and
> 06 - Person Tracking (CV Feature).

