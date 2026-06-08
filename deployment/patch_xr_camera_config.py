#!/usr/bin/env python3
"""Tune XR camera config for low-latency Vision Pro teleoperation."""

from __future__ import annotations

from pathlib import Path

import yaml


CONFIGS = [
    Path("/home/unitree/teleimager/cam_config_server.yaml"),
    Path("/home/unitree/teleimager/cam_config_client.yaml"),
    Path("/home/unitree/xr_teleoperate/teleop/teleimager/cam_config_client.yaml"),
]


def patch_config(path: Path) -> None:
    if not path.exists():
        return

    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    head = data.setdefault("head_camera", {})
    head["enable_zmq"] = False
    head["enable_webrtc"] = True
    head["zmq_port"] = int(head.get("zmq_port", 55555))
    head["webrtc_port"] = int(head.get("webrtc_port", 60001))
    head["webrtc_codec"] = head.get("webrtc_codec", "h264")
    head["type"] = head.get("type", "videohub")
    head["image_shape"] = [720, 1280]
    head["binocular"] = False
    head["fps"] = 15
    head["network_interface"] = head.get("network_interface", "eth0")

    for name in ("left_wrist_camera", "right_wrist_camera"):
        camera = data.setdefault(name, {})
        camera["enable_zmq"] = False
        camera["enable_webrtc"] = False

    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")


def main() -> int:
    for path in CONFIGS:
        patch_config(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
