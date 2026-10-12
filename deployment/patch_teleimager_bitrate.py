#!/usr/bin/env python3
"""Give the robot's TeleImager checkout the WebRTC bitrate hook it predates.

teleimager.service runs /home/unitree/teleimager (an older checkout, Dec 2025)
whose image_server.py never reads the `webrtc:` block of cam_config_server.yaml,
so aiortc's module defaults applied: 1 Mbit/s target, 0.5-3 Mbit/s adaptive.
At 1080p that is a blur in the Vision Pro (measured 1.1 Mbit/s on 2026-10-12).
This inserts the upstream `_apply_webrtc_config` (bitrate caps only) and calls
it from ImageServer.__init__. Idempotent; a no-op when the hook already exists.
"""

from __future__ import annotations

import sys
from pathlib import Path


IMAGE_SERVER = Path("/home/unitree/teleimager/src/teleimager/image_server.py")

IMPORT_ANCHOR = "from aiortc.codecs import h264\n"

HOOK = '''from aiortc.codecs import h264


# ---- robot_telemetry_web patch: WebRTC bitrate caps from cam_config_server.yaml ----
def _apply_webrtc_config(cam_config):
    """Apply webrtc.bitrate.{min,default,max} (bit/s) to aiortc's module-level caps."""
    cfg = (cam_config or {}).get("webrtc", {}) or {}
    bitrate = cfg.get("bitrate", {}) or {}
    from aiortc.codecs import vpx
    for key, attr in (("min", "MIN_BITRATE"), ("default", "DEFAULT_BITRATE"), ("max", "MAX_BITRATE")):
        if key in bitrate and int(bitrate[key]) >= 100_000:
            setattr(h264, attr, int(bitrate[key]))
            setattr(vpx, attr, int(bitrate[key]))
    logger_mp.info(
        f"[WebRTC] bitrate min/default/max={h264.MIN_BITRATE}/{h264.DEFAULT_BITRATE}/{h264.MAX_BITRATE}"
    )
# ---- end patch ----
'''

INIT_ANCHOR = (
    "class ImageServer:\n"
    "    def __init__(self, cam_config, realsense_enable=False, camera_finder_verbose=False, isaacsim_enable=False):\n"
)
INIT_CALL = INIT_ANCHOR + "        _apply_webrtc_config(cam_config)\n"


def patch(text: str) -> str:
    if "def _apply_webrtc_config(" not in text:
        if text.count(IMPORT_ANCHOR) != 1:
            raise SystemExit("image_server.py: aiortc h264 import anchor not found exactly once")
        text = text.replace(IMPORT_ANCHOR, HOOK, 1)
    if "        _apply_webrtc_config(cam_config)\n" not in text:
        if text.count(INIT_ANCHOR) != 1:
            raise SystemExit("image_server.py: ImageServer.__init__ anchor not found exactly once")
        text = text.replace(INIT_ANCHOR, INIT_CALL, 1)
    return text


def main(argv: list[str]) -> int:
    path = Path(argv[1]) if len(argv) > 1 else IMAGE_SERVER
    if not path.exists():
        return 0
    text = path.read_text(encoding="utf-8")
    patched = patch(text)
    if patched != text:
        path.write_text(patched, encoding="utf-8")
        print(f"patched {path}")
    else:
        print("bitrate hook already present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
