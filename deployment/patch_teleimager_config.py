#!/usr/bin/env python3
"""Enforce the dashboard-required teleimager camera config before every start.

/home/unitree/teleimager is a git checkout that gets reset by update flows, so
hand-edits to cam_config_server.yaml do not survive (observed: head_camera
enable_zmq flipped back to false after a reboot, killing the dashboard camera
until someone re-enabled it by hand). Run as an ExecStartPre of
teleimager.service, this re-applies the required settings idempotently:

- head_camera.enable_zmq: true  — the dashboard's ImageClient consumes JPEG
  frames over ZMQ; WebRTC stays untouched for the XR headset.
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

CONFIG = Path("/home/unitree/teleimager/cam_config_server.yaml")


def main() -> int:
    try:
        data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    except OSError as exc:
        print(f"cannot read {CONFIG}: {exc}", file=sys.stderr)
        return 0  # never block the service start
    if not isinstance(data, dict) or "head_camera" not in data:
        print(f"{CONFIG} has no head_camera section; leaving untouched")
        return 0
    if data["head_camera"].get("enable_zmq") is True:
        print("head_camera.enable_zmq already true")
        return 0
    data["head_camera"]["enable_zmq"] = True
    CONFIG.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    print("patched head_camera.enable_zmq -> true")
    return 0


if __name__ == "__main__":
    sys.exit(main())
