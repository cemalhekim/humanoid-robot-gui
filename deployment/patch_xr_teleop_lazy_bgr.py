#!/usr/bin/env python3
"""Stop the XR teleop from decoding head-camera JPEGs it never uses.

teleop_hand_and_arm.py creates its ImageClient with request_bgr=True, so the
ZMQ subscriber decodes every 1080p head frame at 30 fps inside the teleop
process -- even in WebRTC display mode, where the headset gets its video
straight from TeleImager and `xr_need_local_img` is False. That decode shares
the GIL with the 60 Hz IK loop, the Vuer websocket and DDS (teleop process at
~110-125 % CPU on 2026-10-12, laggy hand tracking). The client is now created
without BGR decoding and only asks for it when the teleop will actually read
frames (local display mode or --record); ImageClient enables it lazily on the
next get_head_frame(). Idempotent; anchored on the upstream lines.
"""

from __future__ import annotations

import sys
from pathlib import Path


TELEOP = Path("/home/unitree/xr_teleoperate/teleop/teleop_hand_and_arm.py")

OLD_CLIENT = "        img_client = ImageClient(host=args.img_server_ip, request_bgr=True)\n"
NEW_CLIENT = "        img_client = ImageClient(host=args.img_server_ip, request_bgr=False)  # XR_LAZY_BGR\n"

OLD_NEED = (
    "        xr_need_local_img = not (args.display_mode == 'pass-through' or camera_config['head_camera']['enable_webrtc'])\n"
)
NEW_NEED = OLD_NEED + (
    "        # XR_LAZY_BGR: decode ZMQ frames only when this process will read them.\n"
    "        img_client._request_bgr = bool(xr_need_local_img or getattr(args, 'record', False))\n"
)


def patch(text: str) -> str:
    if "XR_LAZY_BGR" in text:
        return text
    if text.count(OLD_CLIENT) != 1:
        raise SystemExit("teleop_hand_and_arm.py: ImageClient creation line not found exactly once")
    if text.count(OLD_NEED) != 1:
        raise SystemExit("teleop_hand_and_arm.py: xr_need_local_img line not found exactly once")
    return text.replace(OLD_CLIENT, NEW_CLIENT, 1).replace(OLD_NEED, NEW_NEED, 1)


def main(argv: list[str]) -> int:
    path = Path(argv[1]) if len(argv) > 1 else TELEOP
    if not path.exists():
        return 0
    text = path.read_text(encoding="utf-8")
    patched = patch(text)
    if patched != text:
        path.write_text(patched, encoding="utf-8")
        print(f"patched {path}")
    else:
        print("lazy BGR already applied")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
