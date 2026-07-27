---
tags: [yolo, detection, microservice, ai-host, computer-vision, systemd]
summary: The YOLOv8n person-detection microservice on the AI host — HTTP contract, systemd unit, install steps, and GPU performance.
---

# 07 - Detection Service (YOLO)

A minimal HTTP microservice that runs **YOLOv8n** on the AI host GPU and returns
normalized person bounding boxes. It is the detection half of the
person-pointing feature. Canonical source
lives in the repo at `deployment/ai_host/detect_service.py` (Task 2 of the plan —
already landed).

- **Host**: `10.2.125.3` (AI host / "AI-DEV") — see 02 - Network & Hosts.
- **Port**: `8188` (`ThreadingHTTPServer` on `0.0.0.0`).
- **Model**: `YOLO("yolov8n.pt")` (ultralytics), person class only (`classes=[0]`), `conf=0.4`.
- **GPU**: NVIDIA A40, ~6.8–7 ms/frame steady state (first request ~2 s for model load).
- **Deps** (AI host only): `ultralytics`, `opencv-python-headless` (imported as `cv2`), `numpy`. No robot-side deps.

## HTTP contract

### `GET /health`

```json
{ "ok": true }
```

Anything else on GET → `404 {"error": "not found"}`.

### `POST /detect`

- **Body**: raw **JPEG** bytes (`Content-Type: image/jpeg`). Decoded via
  `cv2.imdecode(...)`; a bad image → `400 {"error": "bad image"}`.
- **Response** (`200`):

```json
{
  "persons": [
    {"id": 3, "x1": 0.31, "y1": 0.10, "x2": 0.55, "y2": 0.92,
     "cx": 0.43, "cy": 0.51, "conf": 0.94,
     "keypoints": {"nose": {"x": 0.43, "y": 0.14, "conf": 0.92},
                    "l_shoulder": {"x": 0.48, "y": 0.24, "conf": 0.9},
                    "l_elbow": {"x": 0.52, "y": 0.33, "conf": 0.85},
                    "l_wrist": {"x": 0.55, "y": 0.41, "conf": 0.8}},
     "head": {"x": 0.43, "y": 0.14}}
  ],
  "ms": 6.8,
  "w": 1280,
  "h": 720
}
```

Keypoints exposed (conf ≥ 0.3, normalized 0..1): `nose`, `l_eye`/`r_eye`,
`l_ear`/`r_ear`, `l_shoulder`/`r_shoulder`, `l_elbow`/`r_elbow`,
`l_wrist`/`r_wrist`, `l_hip`/`r_hip`. Elbows/wrists added + deployed
2026-07-27 for 27 - Mimic Mode (Live Motion). `head` is the nose, else
eye midpoint, else ear midpoint (each at `DETECT_HEAD_CONF`, default 0.15);
with `DETECT_REQUIRE_HEAD=1` (default) headless boxes are dropped at the
source — that is how the robot's own arm is filtered out.

> [!warning] Repo copy = deployed copy since 2026-07-27
> The AI-host file had diverged (head-gate + eyes existed only on AI-DEV).
> The deployed version was synced back into
> `deployment/ai_host/detect_service.py` — treat the repo copy as canonical
> again and deploy BY COPYING IT, don't hand-edit on the host.
> Backups on the host: `detect_service.py.bak-20260723`, `.bak-20260727`.

| Field | Meaning |
| --- | --- |
| `x1,y1,x2,y2` | Bbox corners, **normalized 0..1** (pixel / width or height) |
| `cx,cy` | Bbox center, normalized 0..1 |
| `conf` | Detection confidence (≥ 0.4) |
| `persons` | **Sorted by area descending** (largest first) |
| `ms` | Inference time in milliseconds |
| `w,h` | Source image pixel dimensions |

The tracking loop consumes `persons`; `associate()` in `tracking.py` picks the
target (largest, then nearest-to-previous). See
06 - Person Tracking (CV Feature).

> [!note] Implementation detail
> The service normalizes coordinates itself (`x1/w`, `y1/h`, …) and computes
> `cx,cy` as the box-center midpoint. It suppresses access logging
> (`log_message` is a no-op). It is single-file, stdlib HTTP + ultralytics.

## systemd unit

`deployment/ai_host/person-detect.service` (installed as a **user** service on
the AI host as user `vodafone`):

```ini
[Unit]
Description=YOLOv8n person-detection HTTP service (port 8188)
After=network-online.target

[Service]
Type=simple
WorkingDirectory=/home/vodafone/person-tracking
ExecStart=/home/vodafone/person-tracking/venv/bin/python /home/vodafone/person-tracking/detect_service.py
Restart=always
RestartSec=3

[Install]
WantedBy=default.target
```

## Install (from `deployment/ai_host/README.md`)

```bash
mkdir -p ~/person-tracking && cd ~/person-tracking
python3 -m venv venv
./venv/bin/pip install ultralytics opencv-python-headless
# copy detect_service.py here from this repo, then:
mkdir -p ~/.config/systemd/user
cp person-detect.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now person-detect.service
loginctl enable-linger vodafone   # survive logout
```

Smoke test:

```bash
curl -s http://localhost:8188/health                 # {"ok": true}
curl -s -X POST --data-binary @some.jpg http://localhost:8188/detect
```

The robot reaches it at `http://10.2.125.3:8188/detect` (`TRACKING_DETECT_URL`).

> [!info] Latency budget
> The robot-side loop uses a **0.5 s** timeout per detect call. Over Wi-Fi the
> expected round-trip with a real head-camera frame is well under 0.25 s
> (bring-up step 1). If the service stalls, the staleness fail-safe
> ramps the arm to neutral and aborts after 10 consecutive failures.

