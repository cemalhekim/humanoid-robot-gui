#!/usr/bin/env python3
"""Patch TeleVuer with a rawChildren head-locked visual HUD test."""

from __future__ import annotations

from pathlib import Path


TELEVUER = Path("/home/unitree/xr_teleoperate/teleop/televuer/src/televuer/televuer.py")

IMPORT_OLD = (
    "from vuer.schemas import ImageBackground, Hands, MotionControllers, "
    "WebRTCVideoPlane, WebRTCStereoVideoPlane\n"
)
IMPORT_NEW = (
    "from vuer.schemas import ImageBackground, Hands, MotionControllers, "
    "WebRTCVideoPlane, WebRTCStereoVideoPlane, HUDPlane\n"
)

HELPER = r'''
# robot_telemetry_web rawChildren HUD visual test
_RTW_RAW_HUD_IMAGE = None


def _rtw_raw_hud_enabled():
    return os.getenv("XR_RAW_HUD_VISUAL", "0") == "1"


def _rtw_raw_hud_image():
    global _RTW_RAW_HUD_IMAGE
    if _RTW_RAW_HUD_IMAGE is not None:
        return _RTW_RAW_HUD_IMAGE
    canvas = np.zeros((320, 960, 4), dtype=np.uint8)
    cv2.rectangle(canvas, (18, 18), (942, 302), (12, 18, 26, 220), -1, lineType=cv2.LINE_AA)
    cv2.rectangle(canvas, (18, 18), (942, 302), (255, 255, 255, 230), 4, lineType=cv2.LINE_AA)
    cv2.putText(canvas, "RAW HUD TEST", (52, 92), cv2.FONT_HERSHEY_SIMPLEX, 1.7, (255, 255, 255, 250), 5, cv2.LINE_AA)
    cv2.putText(canvas, "A", (142, 232), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (255, 255, 255, 250), 8, cv2.LINE_AA)
    cv2.putText(canvas, "B", (442, 232), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (255, 255, 255, 250), 8, cv2.LINE_AA)
    cv2.putText(canvas, "C", (742, 232), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (255, 255, 255, 250), 8, cv2.LINE_AA)
    cv2.rectangle(canvas, (82, 132), (252, 276), (62, 126, 226, 235), 3, lineType=cv2.LINE_AA)
    cv2.rectangle(canvas, (382, 132), (552, 276), (64, 180, 95, 235), 3, lineType=cv2.LINE_AA)
    cv2.rectangle(canvas, (682, 132), (852, 276), (226, 122, 62, 235), 3, lineType=cv2.LINE_AA)
    _RTW_RAW_HUD_IMAGE = canvas
    return _RTW_RAW_HUD_IMAGE


def _rtw_upsert_raw_hud(session):
    if not _rtw_raw_hud_enabled():
        return
    session.upsert(
        HUDPlane(
            _rtw_raw_hud_image(),
            aspect=3.0,
            height=0.9,
            distanceToCamera=2.2,
            position=[0.0, -0.55, 0.0],
            key="rtw-raw-hud-test",
            format="png",
            interpolate=True,
        ),
        to="rawChildren",
    )
'''


def main() -> int:
    if not TELEVUER.exists():
        return 0
    text = TELEVUER.read_text(encoding="utf-8")
    if IMPORT_OLD in text:
        text = text.replace(IMPORT_OLD, IMPORT_NEW, 1)
    if "# robot_telemetry_web rawChildren HUD visual test" not in text:
        marker = "from typing import Literal\n"
        if marker not in text:
            raise SystemExit("Could not find TeleVuer helper insertion point")
        text = text.replace(marker, marker + HELPER, 1)

    function_names = [
        "main_image_binocular_webrtc",
        "main_image_monocular_webrtc",
    ]
    for name in function_names:
        start = text.find(f"    async def {name}(self, session):")
        if start == -1:
            continue
        next_func = text.find("\n    async def ", start + 1)
        block = text[start:next_func]
        if "_rtw_upsert_raw_hud(session)" in block:
            continue
        while_marker = "        while True:\n"
        while_index = block.find(while_marker)
        if while_index == -1:
            continue
        block = block[:while_index] + "        _rtw_upsert_raw_hud(session)\n" + block[while_index:]
        text = text[:start] + block + text[next_func:]

    TELEVUER.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
