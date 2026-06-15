#!/usr/bin/env python3
"""Patch TeleVuer with a head-locked visual debug menu in WebRTC VR."""

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
# robot_telemetry_web head-locked debug menu visual
_RTW_HEADLOCKED_MENU_IMAGE = None


def _rtw_headlocked_menu_enabled():
    return os.getenv("XR_HEADLOCKED_MENU_VISUAL", "0") == "1"


def _rtw_headlocked_menu_image():
    global _RTW_HEADLOCKED_MENU_IMAGE
    if _RTW_HEADLOCKED_MENU_IMAGE is not None:
        return _RTW_HEADLOCKED_MENU_IMAGE
    canvas = np.zeros((384, 768, 4), dtype=np.uint8)
    cv2.rectangle(canvas, (12, 12), (756, 372), (8, 12, 18, 205), -1, lineType=cv2.LINE_AA)
    cv2.rectangle(canvas, (12, 12), (756, 372), (255, 255, 255, 210), 4, lineType=cv2.LINE_AA)
    cv2.putText(canvas, "VR MENU TEST", (54, 82), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (255, 255, 255, 245), 4, cv2.LINE_AA)

    items = [
        ((54, 126), (236, 318), (62, 126, 226, 235), "A"),
        ((292, 126), (474, 318), (64, 180, 95, 235), "B"),
        ((530, 126), (712, 318), (226, 122, 62, 235), "C"),
    ]
    for top_left, bottom_right, color, label in items:
        cv2.rectangle(canvas, top_left, bottom_right, color, -1, lineType=cv2.LINE_AA)
        cv2.rectangle(canvas, top_left, bottom_right, (255, 255, 255, 230), 3, lineType=cv2.LINE_AA)
        x = top_left[0] + 63
        y = top_left[1] + 121
        cv2.putText(canvas, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 2.7, (255, 255, 255, 245), 7, cv2.LINE_AA)

    _RTW_HEADLOCKED_MENU_IMAGE = canvas
    return _RTW_HEADLOCKED_MENU_IMAGE


def _rtw_headlocked_menu_plane(key="rtw-headlocked-menu-test"):
    return HUDPlane(
        _rtw_headlocked_menu_image(),
        aspect=2.0,
        height=1.35,
        distanceToCamera=3.2,
        position=[0.0, -0.75, 0.0],
        key=key,
        format="png",
        interpolate=True,
    )
'''

OLD_STEREO = """            session.upsert(
                WebRTCStereoVideoPlane(
                    src=self.webrtc_url,
                    iceServer=None,
                    iceServers=[], 
                    key="video-quad",
                    aspect=self.aspect_ratio,
                    height = 7,
                    layout="stereo-left-right"
                ),
                to="bgChildren",
            )
"""

NEW_STEREO = """            video_plane = WebRTCStereoVideoPlane(
                src=self.webrtc_url,
                iceServer=None,
                iceServers=[], 
                key="video-quad",
                aspect=self.aspect_ratio,
                height = 7,
                layout="stereo-left-right"
            )
            nodes = [video_plane, _rtw_headlocked_menu_plane()] if _rtw_headlocked_menu_enabled() else video_plane
            session.upsert(nodes, to="bgChildren")
"""

OLD_MONO = """            session.upsert(
                WebRTCVideoPlane(
                    src=self.webrtc_url,
                    iceServer=None,
                    iceServers=[],
                    key="video-quad",
                    aspect=self.aspect_ratio,
                    height = 7,
                ),
                to="bgChildren",
            )
"""

NEW_MONO = """            video_plane = WebRTCVideoPlane(
                src=self.webrtc_url,
                iceServer=None,
                iceServers=[],
                key="video-quad",
                aspect=self.aspect_ratio,
                height = 7,
            )
            nodes = [video_plane, _rtw_headlocked_menu_plane()] if _rtw_headlocked_menu_enabled() else video_plane
            session.upsert(nodes, to="bgChildren")
"""


def main() -> int:
    if not TELEVUER.exists():
        return 0
    text = TELEVUER.read_text(encoding="utf-8")
    if IMPORT_OLD in text:
        text = text.replace(IMPORT_OLD, IMPORT_NEW, 1)
    if "# robot_telemetry_web head-locked debug menu visual" not in text:
        marker = "from typing import Literal\n"
        if marker not in text:
            raise SystemExit("Could not find TeleVuer helper insertion point")
        text = text.replace(marker, marker + HELPER, 1)
    text = text.replace(OLD_STEREO, NEW_STEREO)
    text = text.replace(OLD_MONO, NEW_MONO)
    TELEVUER.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
