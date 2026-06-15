#!/usr/bin/env python3
"""Patch TeleVuer to draw a visual loco pad together with WebRTC video planes."""

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
# robot_telemetry_web WebRTC visual pad
_RTW_WEBRTC_PAD_IMAGE = None


def _rtw_webrtc_pad_enabled():
    return os.getenv("XR_WEBRTC_PAD_VISUAL", "0") == "1"


def _rtw_webrtc_pad_image():
    global _RTW_WEBRTC_PAD_IMAGE
    if _RTW_WEBRTC_PAD_IMAGE is not None:
        return _RTW_WEBRTC_PAD_IMAGE
    canvas = np.zeros((512, 512, 4), dtype=np.uint8)

    def circle(cx, cy, radius, color):
        cv2.circle(canvas, (cx, cy), radius, color, -1, lineType=cv2.LINE_AA)

    def arrow(points, color):
        cv2.fillConvexPoly(canvas, np.array(points, dtype=np.int32), color, lineType=cv2.LINE_AA)

    circle(256, 256, 236, (8, 12, 18, 145))
    orange = (62, 111, 226, 245)
    blue = (226, 122, 62, 245)
    red = (58, 58, 230, 245)
    white = (245, 245, 245, 245)

    arrow([(256, 62), (202, 132), (234, 132), (234, 190), (278, 190), (278, 132), (310, 132)], orange)
    arrow([(256, 450), (202, 380), (234, 380), (234, 322), (278, 322), (278, 380), (310, 380)], orange)
    arrow([(62, 256), (132, 202), (132, 234), (190, 234), (190, 278), (132, 278), (132, 310)], orange)
    arrow([(450, 256), (380, 202), (380, 234), (322, 234), (322, 278), (380, 278), (380, 310)], orange)

    cv2.ellipse(canvas, (150, 150), (78, 78), 145, 0, 245, blue, 18, lineType=cv2.LINE_AA)
    arrow([(98, 78), (164, 58), (146, 126)], blue)
    cv2.ellipse(canvas, (362, 150), (78, 78), 35, -65, 180, blue, 18, lineType=cv2.LINE_AA)
    arrow([(414, 78), (348, 58), (366, 126)], blue)

    circle(256, 256, 48, red)
    cv2.line(canvas, (236, 236), (276, 276), white, 9, lineType=cv2.LINE_AA)
    cv2.line(canvas, (276, 236), (236, 276), white, 9, lineType=cv2.LINE_AA)
    _RTW_WEBRTC_PAD_IMAGE = canvas
    return _RTW_WEBRTC_PAD_IMAGE


def _rtw_webrtc_pad_plane(key="rtw-webrtc-loco-pad"):
    return HUDPlane(
        _rtw_webrtc_pad_image(),
        aspect=1,
        height=1.0,
        distanceToCamera=8.5,
        position=[2.15, -1.35, 0.0],
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
            nodes = [video_plane, _rtw_webrtc_pad_plane()] if _rtw_webrtc_pad_enabled() else video_plane
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
            nodes = [video_plane, _rtw_webrtc_pad_plane()] if _rtw_webrtc_pad_enabled() else video_plane
            session.upsert(nodes, to="bgChildren")
"""


def main() -> int:
    if not TELEVUER.exists():
        return 0
    text = TELEVUER.read_text(encoding="utf-8")
    if IMPORT_OLD in text:
        text = text.replace(IMPORT_OLD, IMPORT_NEW, 1)
    if "# robot_telemetry_web WebRTC visual pad" not in text:
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
