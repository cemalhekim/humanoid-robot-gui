#!/usr/bin/env python3
"""Enforce the dashboard-required teleimager camera config before every start.

/home/unitree/teleimager is a git checkout that gets reset by update flows, so
hand-edits to cam_config_server.yaml do not survive (observed: head_camera
enable_zmq flipped back to false after a reboot, killing the dashboard camera
until someone re-enabled it by hand). Run as an ExecStartPre of
teleimager.service, this re-applies the required settings idempotently:

- head_camera.enable_zmq: true  — the dashboard's ImageClient consumes JPEG
  frames over ZMQ; WebRTC stays untouched for the XR headset.
- webrtc.bitrate / gop_length — without this block aiortc encodes the 1080p
  headset stream at its 1 Mbit/s default (0.5-3 Mbit/s adaptive), which is a
  blur in the Vision Pro (seen 2026-10-12: 1.1 Mbit/s on the Wi-Fi link).
  2/4/8 Mbit/s matches the upstream "shared Wi-Fi" guidance; gop 30 recovers
  faster from loss on the 2.4 GHz link. Existing values are left alone.
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

CONFIG = Path("/home/unitree/teleimager/cam_config_server.yaml")
WEBRTC_BITRATE = {"min": 2_000_000, "default": 4_000_000, "max": 8_000_000}
WEBRTC_GOP_LENGTH = 30


def main() -> int:
    try:
        data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    except OSError as exc:
        print(f"cannot read {CONFIG}: {exc}", file=sys.stderr)
        return 0  # never block the service start
    if not isinstance(data, dict) or "head_camera" not in data:
        print(f"{CONFIG} has no head_camera section; leaving untouched")
        return 0
    changes: list[str] = []
    if data["head_camera"].get("enable_zmq") is not True:
        data["head_camera"]["enable_zmq"] = True
        changes.append("head_camera.enable_zmq -> true")
    webrtc = data.get("webrtc")
    if not isinstance(webrtc, dict):
        webrtc = {}
        data["webrtc"] = webrtc
    bitrate = webrtc.get("bitrate")
    if not isinstance(bitrate, dict):
        bitrate = {}
        webrtc["bitrate"] = bitrate
    for key, value in WEBRTC_BITRATE.items():
        if key not in bitrate:
            bitrate[key] = value
            changes.append(f"webrtc.bitrate.{key} -> {value}")
    if "gop_length" not in webrtc:
        webrtc["gop_length"] = WEBRTC_GOP_LENGTH
        changes.append(f"webrtc.gop_length -> {WEBRTC_GOP_LENGTH}")
    if not changes:
        print("teleimager config already compliant")
        return 0
    CONFIG.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    print("patched " + ", ".join(changes))
    return 0


if __name__ == "__main__":
    sys.exit(main())
