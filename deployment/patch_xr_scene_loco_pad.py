#!/usr/bin/env python3
"""Patch XR teleop with a real scene HUD loco pad and hand-pinch commands."""

from __future__ import annotations

from pathlib import Path


TELEVUER = Path("/home/unitree/xr_teleoperate/teleop/televuer/src/televuer/televuer.py")
TELEOP = Path("/home/unitree/xr_teleoperate/teleop/teleop_hand_and_arm.py")


TELEVUER_IMPORT_OLD = (
    "from vuer.schemas import ImageBackground, Hands, MotionControllers, "
    "WebRTCVideoPlane, WebRTCStereoVideoPlane\n"
)
TELEVUER_IMPORT_NEW = (
    "from vuer.schemas import ImageBackground, Hands, MotionControllers, "
    "WebRTCVideoPlane, WebRTCStereoVideoPlane, HUDPlane\n"
)

TELEVUER_HELPER = r'''
# robot_telemetry_web scene loco pad
_RTW_XR_LOCO_PAD_IMAGE = None


def _rtw_loco_pad_image():
    global _RTW_XR_LOCO_PAD_IMAGE
    if _RTW_XR_LOCO_PAD_IMAGE is not None:
        return _RTW_XR_LOCO_PAD_IMAGE
    canvas = np.zeros((512, 512, 4), dtype=np.uint8)
    canvas[:, :, 3] = 0

    def circle(cx, cy, radius, color):
        cv2.circle(canvas, (cx, cy), radius, color, -1, lineType=cv2.LINE_AA)

    def arrow(points, color):
        cv2.fillConvexPoly(canvas, np.array(points, dtype=np.int32), color, lineType=cv2.LINE_AA)

    bg = (8, 12, 18, 150)
    circle(256, 256, 236, bg)
    orange = (70, 126, 255, 245)
    blue = (255, 140, 45, 245)
    red = (72, 72, 255, 245)

    # Forward, back, left, right.
    arrow([(256, 68), (210, 130), (238, 130), (238, 184), (274, 184), (274, 130), (302, 130)], orange)
    arrow([(256, 444), (210, 382), (238, 382), (238, 328), (274, 328), (274, 382), (302, 382)], orange)
    arrow([(68, 256), (130, 210), (130, 238), (184, 238), (184, 274), (130, 274), (130, 302)], orange)
    arrow([(444, 256), (382, 210), (382, 238), (328, 238), (328, 274), (382, 274), (382, 302)], orange)

    # Rotation arrows as curved arcs.
    cv2.ellipse(canvas, (150, 150), (74, 74), 145, 0, 245, blue, 18, lineType=cv2.LINE_AA)
    arrow([(98, 80), (158, 62), (143, 122)], blue)
    cv2.ellipse(canvas, (362, 150), (74, 74), 35, -65, 180, blue, 18, lineType=cv2.LINE_AA)
    arrow([(414, 80), (354, 62), (369, 122)], blue)

    circle(256, 256, 46, red)
    cv2.line(canvas, (238, 238), (274, 274), (255, 255, 255, 245), 9, lineType=cv2.LINE_AA)
    cv2.line(canvas, (274, 238), (238, 274), (255, 255, 255, 245), 9, lineType=cv2.LINE_AA)
    _RTW_XR_LOCO_PAD_IMAGE = canvas
    return _RTW_XR_LOCO_PAD_IMAGE


def _rtw_upsert_loco_pad(session):
    session.upsert(
        HUDPlane(
            _rtw_loco_pad_image(),
            aspect=1,
            height=0.48,
            distanceToCamera=0.92,
            key="rtw-xr-scene-loco-pad",
            format="png",
            interpolate=True,
        ),
        to="bgChildren",
    )
'''


TELEOP_HELPER = r'''
# robot_telemetry_web scene loco pad commands
import json as _rtw_json
import urllib.error as _rtw_urllib_error
import urllib.request as _rtw_urllib_request

_RTW_LOCO_PRESETS = {
    "forward": (0.5, 0.0, 0.0),
    "back": (-0.5, 0.0, 0.0),
    "left": (0.0, 0.5, 0.0),
    "right": (0.0, -0.5, 0.0),
    "turn-left": (0.0, 0.0, 0.5),
    "turn-right": (0.0, 0.0, -0.5),
}

_RTW_PAD_BUTTONS = [
    ("turn-right", -0.20, 0.20),
    ("forward", 0.0, 0.20),
    ("turn-left", 0.20, 0.20),
    ("left", -0.20, 0.0),
    ("stop", 0.0, 0.0),
    ("right", 0.20, 0.0),
    ("back", 0.0, -0.20),
]


def _rtw_loco_command(action: str, values=None) -> None:
    payload = {
        "action": action,
        "armed": True,
        "i_understand_risk": True,
        "duration": 1.0,
        "continuous_move": True,
    }
    if values is not None:
        payload.update({"vx": values[0], "vy": values[1], "vyaw": values[2]})
    data = _rtw_json.dumps(payload).encode("utf-8")
    req = _rtw_urllib_request.Request(
        "http://127.0.0.1:8088/api/loco/command",
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        _rtw_urllib_request.urlopen(req, timeout=0.4).read()
    except Exception:
        pass


def _rtw_scene_loco_pad_hit(tele_data):
    if os.getenv("XR_SCENE_LOCO_PAD", "0") != "1":
        return None
    candidates = []
    if getattr(tele_data, "left_hand_pinch", False) and tele_data.left_hand_pos is not None:
        candidates.append(tele_data.left_hand_pos[9])
    if getattr(tele_data, "right_hand_pinch", False) and tele_data.right_hand_pos is not None:
        candidates.append(tele_data.right_hand_pos[9])
    if not candidates:
        return None
    try:
        head_inv = np.linalg.inv(tele_data.head_pose)
    except Exception:
        return None
    for tip in candidates:
        if tip is None or not np.isfinite(tip).all():
            continue
        local = head_inv @ np.array([tip[0], tip[1], tip[2], 1.0], dtype=float)
        x, y, z = float(local[0]), float(local[1]), float(local[2])
        if not (-1.45 <= z <= -0.20):
            continue
        for name, bx, by in _RTW_PAD_BUTTONS:
            if abs(x - bx) <= 0.105 and abs(y - by) <= 0.105:
                return name
    return None


def _rtw_handle_scene_loco_pad(tele_data, state: dict) -> None:
    action = _rtw_scene_loco_pad_hit(tele_data)
    now = time.time()
    if action is None:
        if state.get("active"):
            _rtw_loco_command("stop_move")
        state["active"] = None
        state["last"] = 0.0
        return
    if action == "stop":
        if state.get("active") != "stop" or now - state.get("last", 0.0) > 0.25:
            _rtw_loco_command("stop_move")
        state["active"] = "stop"
        state["last"] = now
        return
    if action != state.get("active") or now - state.get("last", 0.0) > 0.30:
        _rtw_loco_command("move", _RTW_LOCO_PRESETS[action])
        state["active"] = action
        state["last"] = now
'''


def patch_televuer() -> None:
    if not TELEVUER.exists():
        return
    text = TELEVUER.read_text(encoding="utf-8")
    if TELEVUER_IMPORT_OLD in text:
        text = text.replace(TELEVUER_IMPORT_OLD, TELEVUER_IMPORT_NEW, 1)
    if "# robot_telemetry_web scene loco pad" not in text:
        marker = "from typing import Literal\n"
        if marker not in text:
            raise SystemExit("Could not find TeleVuer helper insertion point")
        text = text.replace(marker, marker + TELEVUER_HELPER, 1)

    function_names = [
        "main_image_binocular_zmq",
        "main_image_monocular_zmq",
        "main_image_binocular_webrtc",
        "main_image_monocular_webrtc",
        "main_image_binocular_zmq_ego",
        "main_image_monocular_zmq_ego",
        "main_image_binocular_webrtc_ego",
        "main_image_monocular_webrtc_ego",
        "main_pass_through",
    ]
    for name in function_names:
        start = text.find(f"    async def {name}(self, session):")
        if start == -1:
            continue
        next_func = text.find("\n    async def ", start + 1)
        if next_func == -1:
            next_func = text.find("\n    # ====================", start + 1)
        block = text[start:next_func]
        if "_rtw_upsert_loco_pad(session)" in block:
            continue
        while_marker = "        while True:\n"
        while_index = block.find(while_marker)
        if while_index == -1:
            continue
        block = block[:while_index] + "        _rtw_upsert_loco_pad(session)\n" + block[while_index:]
        text = text[:start] + block + text[next_func:]
    TELEVUER.write_text(text, encoding="utf-8")


def patch_teleop() -> None:
    if not TELEOP.exists():
        return
    text = TELEOP.read_text(encoding="utf-8")
    if "# robot_telemetry_web scene loco pad commands" not in text:
        marker = "from sshkeyboard import listen_keyboard, stop_listening\n"
        if marker not in text:
            raise SystemExit("Could not find teleop helper insertion point")
        text = text.replace(marker, marker + TELEOP_HELPER, 1)
    state_marker = "    try:\n        user_input_thread = threading.Thread(target=listen_keyboard, kwargs={\"on_press\": on_press, \"until\": \"esc\"})\n"
    if "xr_scene_loco_pad_state = {\"active\": None, \"last\": 0.0}" not in text:
        if state_marker not in text:
            raise SystemExit("Could not find teleop state insertion point")
        text = text.replace(
            state_marker,
            "    xr_scene_loco_pad_state = {\"active\": None, \"last\": 0.0}\n\n" + state_marker,
            1,
        )
    call_marker = """            # get current robot state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
"""
    if "_rtw_handle_scene_loco_pad(tele_data, xr_scene_loco_pad_state)" not in text:
        if call_marker not in text:
            raise SystemExit("Could not find teleop loop insertion point")
        text = text.replace(
            call_marker,
            """            if args.input_mode == "hand" and args.motion:
                _rtw_handle_scene_loco_pad(tele_data, xr_scene_loco_pad_state)

""" + call_marker,
            1,
        )
    TELEOP.write_text(text, encoding="utf-8")


def main() -> int:
    patch_televuer()
    patch_teleop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
