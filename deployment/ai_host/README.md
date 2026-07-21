# AI host (10.2.125.3) person-detection service

YOLOv8n person detector used by the robot's person-tracking / arm-pointing
feature. The robot's `server.py` tracking loop POSTs head-camera JPEGs here and
maps the returned person boxes to right-shoulder joint targets. See
`docs/superpowers/specs/2026-07-21-person-pointing-design.md`.

## HTTP contract

- `POST /detect` — body is a raw JPEG. Response:
  `{"persons": [{"x1","y1","x2","y2","cx","cy","conf"} ...], "ms": float, "w": int, "h": int}`.
  Coordinates are normalized 0..1; persons are sorted by box area descending;
  YOLOv8n, person class only (`classes=[0]`), `conf >= 0.4`.
- `GET /health` -> `{"ok": true}`.

## One-time install (as user `vodafone` on the AI host)

    mkdir -p ~/person-tracking && cd ~/person-tracking
    python3 -m venv venv
    ./venv/bin/pip install ultralytics opencv-python-headless
    # copy detect_service.py here from this repo, then:
    mkdir -p ~/.config/systemd/user
    cp person-detect.service ~/.config/systemd/user/
    systemctl --user daemon-reload
    systemctl --user enable --now person-detect.service
    loginctl enable-linger vodafone

## Smoke test

    curl -s http://localhost:8188/health          # {"ok": true}
    curl -s -X POST --data-binary @some.jpg http://localhost:8188/detect

The robot reaches this service at `http://10.2.125.3:8188/detect`
(`TRACKING_DETECT_URL`). First request after start is slow (~2 s model load);
steady state is <10 ms inference on the A40 GPU (measured 6.8 ms/frame,
2026-07-21).

> [!note] Network reachability
> The AI host cannot reach the robot's internal ethernet network
> (192.168.123.x); the robot reaches the AI host outbound over the shared LAN.
> Wi-Fi links are known to stall — the tracking loop's staleness handling
> (arm ramps to neutral after 1.5 s without a detection) exists for this.
