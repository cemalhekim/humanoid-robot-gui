#!/usr/bin/env python3
"""Patch TeleVuer with an optional root scene loco control pad."""

from __future__ import annotations

from pathlib import Path


TELEVUER = Path("/home/unitree/xr_teleoperate/teleop/televuer/src/televuer/televuer.py")

MARKER = "# robot_telemetry_web root children loco pad\n"
OLD_MARKER = "# robot_telemetry_web root children visual test\n"

HELPER = '''# robot_telemetry_web root children loco pad
def _rtw_root_children_visual_enabled():
    return os.getenv("XR_ROOT_CHILDREN_VISUAL", "0") == "1"


def _rtw_loco_payload_for_key(key):
    if not key:
        return None
    key = str(key).split("-mark-", 1)[0]
    presets = {
        "rtw-loco-turn-right": (0.0, 0.0, -0.5),
        "rtw-loco-forward": (0.5, 0.0, 0.0),
        "rtw-loco-turn-left": (0.0, 0.0, 0.5),
        "rtw-loco-left": (0.0, 0.5, 0.0),
        "rtw-loco-back": (-0.5, 0.0, 0.0),
        "rtw-loco-right": (0.0, -0.5, 0.0),
    }
    if key not in presets:
        return None
    vx, vy, vyaw = presets[key]
    return {
        "action": "velocity",
        "vx": vx,
        "vy": vy,
        "vyaw": vyaw,
        "duration": 0.35,
    }


_rtw_loco_hold_lock = threading.Lock()
_rtw_loco_hold_stop = None
_rtw_loco_last_pointer_event_at = 0.0
_rtw_loco_hover_key = None
_rtw_loco_select_active = False


def _rtw_post_loco(payload):
    import json
    import urllib.request

    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        "http://127.0.0.1:8088/api/loco/command",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=0.7) as response:
        response.read()


def _rtw_loco_hold_loop(key, stop_event):
    payload = _rtw_loco_payload_for_key(key)
    if payload is None:
        return
    while not stop_event.is_set():
        try:
            _rtw_post_loco(payload)
        except Exception as exc:
            print(f"[rtw loco pad] command failed for {key}: {exc}")
            break
        stop_event.wait(0.18)


def _rtw_stop_loco_hold():
    global _rtw_loco_hold_stop
    with _rtw_loco_hold_lock:
        stop_event = _rtw_loco_hold_stop
        _rtw_loco_hold_stop = None
    if stop_event is not None:
        stop_event.set()
    try:
        _rtw_post_loco({"action": "stop_move"})
    except Exception as exc:
        print(f"[rtw loco pad] stop failed: {exc}")


def _rtw_start_loco_hold(key):
    global _rtw_loco_hold_stop
    if _rtw_loco_payload_for_key(key) is None:
        return
    _rtw_stop_loco_hold()
    stop_event = threading.Event()
    with _rtw_loco_hold_lock:
        _rtw_loco_hold_stop = stop_event
    thread = threading.Thread(target=_rtw_loco_hold_loop, args=(key, stop_event), daemon=True)
    thread.start()


def _rtw_handle_loco_pad_pointer_down(key):
    global _rtw_loco_last_pointer_event_at
    if not _rtw_root_children_visual_enabled():
        return
    _rtw_loco_last_pointer_event_at = __import__("time").time()
    _rtw_start_loco_hold(key)


def _rtw_handle_loco_pad_pointer_up(key):
    global _rtw_loco_last_pointer_event_at
    if not _rtw_root_children_visual_enabled():
        return
    _rtw_loco_last_pointer_event_at = __import__("time").time()
    _rtw_stop_loco_hold()


def _rtw_handle_loco_pad_hover(key):
    global _rtw_loco_hover_key
    if not _rtw_root_children_visual_enabled():
        return
    if key and _rtw_loco_payload_for_key(key) is not None:
        _rtw_loco_hover_key = str(key).split("-mark-", 1)[0]
    else:
        _rtw_loco_hover_key = None
        if _rtw_loco_select_active:
            _rtw_stop_loco_hold()


def _rtw_loco_state_pressed(*states):
    bool_fields = (
        "trigger",
        "squeeze",
        "pinch",
        "select",
        "pressed",
        "down",
        "primaryButton",
        "aButton",
        "bButton",
    )
    analog_fields = (
        "triggerValue",
        "squeezeValue",
        "pinchValue",
        "selectValue",
        "pressValue",
    )
    for state in states:
        if not isinstance(state, dict):
            continue
        for field in bool_fields:
            if bool(state.get(field, False)):
                return True
        for field in analog_fields:
            try:
                if float(state.get(field, 0.0) or 0.0) > 0.15:
                    return True
            except (TypeError, ValueError):
                pass
        buttons = state.get("buttons")
        if isinstance(buttons, (list, tuple)):
            for button in buttons:
                if isinstance(button, dict):
                    if bool(button.get("pressed", False)):
                        return True
                    try:
                        if float(button.get("value", 0.0) or 0.0) > 0.15:
                            return True
                    except (TypeError, ValueError):
                        pass
                elif bool(button):
                    return True
    return False


def _rtw_handle_loco_controller_state(value):
    global _rtw_loco_select_active, _rtw_loco_last_pointer_event_at
    if not _rtw_root_children_visual_enabled():
        return
    left = value.get("leftState") or {}
    right = value.get("rightState") or {}
    pressed = _rtw_loco_state_pressed(left, right)
    if pressed and not _rtw_loco_select_active:
        _rtw_loco_select_active = True
        _rtw_loco_last_pointer_event_at = __import__("time").time()
        if _rtw_loco_hover_key:
            _rtw_start_loco_hold(_rtw_loco_hover_key)
    elif pressed and _rtw_loco_select_active and _rtw_loco_hover_key:
        with _rtw_loco_hold_lock:
            running = _rtw_loco_hold_stop is not None
        if not running:
            _rtw_start_loco_hold(_rtw_loco_hover_key)
    elif not pressed and _rtw_loco_select_active:
        _rtw_loco_select_active = False
        _rtw_loco_last_pointer_event_at = __import__("time").time()
        _rtw_stop_loco_hold()


def _rtw_send_loco_pulse(key):
    payload = _rtw_loco_payload_for_key(key)
    if payload is None:
        return
    try:
        _rtw_post_loco(payload)
    except Exception as exc:
        print(f"[rtw loco pad] command failed for {key}: {exc}")
        return
    try:
        __import__("time").sleep(0.45)
        _rtw_post_loco({"action": "stop_move"})
    except Exception as exc:
        print(f"[rtw loco pad] stop failed after {key}: {exc}")


def _rtw_handle_loco_pad_click(key):
    if not _rtw_root_children_visual_enabled():
        return
    if __import__("time").time() - _rtw_loco_last_pointer_event_at < 0.35:
        return
    thread = threading.Thread(target=_rtw_send_loco_pulse, args=(key,), daemon=True)
    thread.start()


def _rtw_loco_visual_key(key):
    return str(key).replace("rtw-loco-", "rtw-pad-", 1)


def _rtw_loco_button_visual(key, x, y, color):
    return Box(
        **{
            "key": f"{_rtw_loco_visual_key(key)}-bg",
            "position": [x, y, -1.35],
            "scale": [0.36, 0.24, 0.04],
            "color": color,
            "materialType": "basic",
            "material": {
                "color": color,
                "transparent": False,
                "depthTest": False,
                "toneMapped": False,
            },
            "onClick": False,
        }
    )


def _rtw_loco_hitbox(key, x, y):
    return Box(
        **{
            "key": key,
            "position": [x, y, -1.27],
            "scale": [0.42, 0.30, 0.03],
            "color": "#ffffff",
            "materialType": "basic",
            "material": {
                "color": "#ffffff",
                "transparent": True,
                "opacity": 0.01,
                "depthTest": False,
                "toneMapped": False,
            },
            "onClick": True,
        }
    )


def _rtw_loco_mark(key, x, y, sx, sy, color="#ffffff", rz=0.0):
    return Box(
        **{
            "key": key,
            "position": [x, y, -1.31],
            "scale": [sx, sy, 0.035],
            "rotation": [0, 0, rz],
            "color": color,
            "materialType": "basic",
            "material": {
                "color": color,
                "transparent": False,
                "depthTest": False,
                "toneMapped": False,
            },
            "onClick": False,
        }
    )


def _rtw_loco_arrow_icon(key, x, y, direction):
    color = "#f8fbff"
    if direction == "up":
        parts = [
            (0.0, -0.025, 0.055, 0.155, 0.0),
            (-0.052, 0.055, 0.115, 0.038, 0.72),
            (0.052, 0.055, 0.115, 0.038, -0.72),
        ]
    elif direction == "down":
        parts = [
            (0.0, 0.025, 0.055, 0.155, 0.0),
            (-0.052, -0.055, 0.115, 0.038, -0.72),
            (0.052, -0.055, 0.115, 0.038, 0.72),
        ]
    elif direction == "left":
        parts = [
            (0.028, 0.0, 0.17, 0.052, 0.0),
            (-0.06, 0.052, 0.115, 0.038, -0.72),
            (-0.06, -0.052, 0.115, 0.038, 0.72),
        ]
    else:
        parts = [
            (-0.028, 0.0, 0.17, 0.052, 0.0),
            (0.06, 0.052, 0.115, 0.038, 0.72),
            (0.06, -0.052, 0.115, 0.038, -0.72),
        ]
    return [
        _rtw_loco_mark(f"{_rtw_loco_visual_key(key)}-mark-{index}", x + dx, y + dy, sx, sy, color, rz)
        for index, (dx, dy, sx, sy, rz) in enumerate(parts)
    ]


def _rtw_loco_turn_icon(key, x, y, direction):
    color = "#f8fbff"
    sign = -1 if direction == "right" else 1
    parts = [
        (-0.052 * sign, -0.038, 0.115, 0.04, 0.0),
        (-0.086 * sign, 0.02, 0.105, 0.04, 0.62 * sign),
        (-0.044 * sign, 0.078, 0.105, 0.04, 1.18 * sign),
        (0.045 * sign, 0.09, 0.11, 0.038, -0.38 * sign),
        (0.075 * sign, 0.03, 0.11, 0.038, 0.88 * sign),
    ]
    return [
        _rtw_loco_mark(f"{_rtw_loco_visual_key(key)}-mark-{index}", x + dx, y + dy, sx, sy, color, rz)
        for index, (dx, dy, sx, sy, rz) in enumerate(parts)
    ]


def _rtw_loco_control(key, x, y, color, icon, direction):
    if icon == "turn":
        marks = _rtw_loco_turn_icon(key, x, y, direction)
    else:
        marks = _rtw_loco_arrow_icon(key, x, y, direction)
    return [
        _rtw_loco_button_visual(key, x, y, color),
        *marks,
        _rtw_loco_hitbox(key, x, y),
    ]


def _rtw_upsert_root_children_visual(session):
    if not _rtw_root_children_visual_enabled():
        return
    session.upsert(
        [
            *_rtw_loco_control("rtw-loco-turn-right", -0.5, 0.18, "#b8322a", "turn", "right"),
            *_rtw_loco_control("rtw-loco-forward", 0.0, 0.18, "#1f8f4d", "arrow", "up"),
            *_rtw_loco_control("rtw-loco-turn-left", 0.5, 0.18, "#b8322a", "turn", "left"),
            *_rtw_loco_control("rtw-loco-left", -0.5, -0.18, "#2457d6", "arrow", "left"),
            *_rtw_loco_control("rtw-loco-back", 0.0, -0.18, "#7a2fc2", "arrow", "down"),
            *_rtw_loco_control("rtw-loco-right", 0.5, -0.18, "#2457d6", "arrow", "right"),
        ],
        to="children",
    )


'''

IMPORT_OLD = "from vuer.schemas import ImageBackground, Hands, MotionControllers, WebRTCVideoPlane, WebRTCStereoVideoPlane\n"
IMPORT_NEW = "from vuer.schemas import ImageBackground, Hands, MotionControllers, WebRTCVideoPlane, WebRTCStereoVideoPlane, Box\n"

INSERT_POINTS = [
    """        while True:
            session.upsert(
                WebRTCStereoVideoPlane(
""",
    """        while True:
            session.upsert(
                WebRTCVideoPlane(
""",
]

HANDLER_INSERT = """        self.vuer.add_handler("CAMERA_MOVE")(self.on_cam_move)
"""
HANDLER_REPLACEMENT = """        self.vuer.add_handler("CAMERA_MOVE")(self.on_cam_move)
        self.vuer.add_handler("ON_CLICK")(self.on_rtw_root_loco_click)
        self.vuer.add_handler("RTW_LOCO_HOVER")(self.on_rtw_root_loco_hover)
        self.vuer.add_handler("RTW_LOCO_POINTER_DOWN")(self.on_rtw_root_loco_pointer_down)
        self.vuer.add_handler("RTW_LOCO_POINTER_UP")(self.on_rtw_root_loco_pointer_up)
"""

METHOD_INSERT = """    async def on_cam_move(self, event, session, fps=60):
"""
METHOD_BLOCK = """    async def on_rtw_root_loco_pointer_down(self, event, session):
        try:
            value = event.value or {}
            _rtw_handle_loco_pad_pointer_down(value.get("key"))
        except Exception as exc:
            print(f"[rtw loco pad] pointer down handler failed: {exc}")

    async def on_rtw_root_loco_pointer_up(self, event, session):
        try:
            value = event.value or {}
            _rtw_handle_loco_pad_pointer_up(value.get("key"))
        except Exception as exc:
            print(f"[rtw loco pad] pointer up handler failed: {exc}")

    async def on_rtw_root_loco_hover(self, event, session):
        try:
            value = event.value or {}
            _rtw_handle_loco_pad_hover(value.get("key"))
        except Exception as exc:
            print(f"[rtw loco pad] hover handler failed: {exc}")

    async def on_rtw_root_loco_click(self, event, session):
        try:
            value = event.value or {}
            _rtw_handle_loco_pad_click(value.get("key"))
        except Exception as exc:
            print(f"[rtw loco pad] click handler failed: {exc}")

"""

CONTROLLER_INSERT = """            extract_controllers(left_controller, "left")
            extract_controllers(right_controller, "right")
"""
CONTROLLER_REPLACEMENT = """            extract_controllers(left_controller, "left")
            extract_controllers(right_controller, "right")
            _rtw_handle_loco_controller_state(event.value)
"""

HAND_INSERT = """            extract_hands(left_hand, "left")
            extract_hands(right_hand, "right")
"""
HAND_REPLACEMENT = """            extract_hands(left_hand, "left")
            extract_hands(right_hand, "right")
            _rtw_handle_loco_controller_state(event.value)
"""


def main() -> int:
    if not TELEVUER.exists():
        return 0

    text = TELEVUER.read_text(encoding="utf-8")

    if IMPORT_NEW not in text:
        if IMPORT_OLD not in text:
            raise SystemExit("Could not find TeleVuer schema import line")
        text = text.replace(IMPORT_OLD, IMPORT_NEW, 1)

    for marker in (MARKER, OLD_MARKER):
        if marker in text:
            start = text.index(marker)
            end = text.index("\n\nclass TeleVuer:", start)
            text = text[:start] + HELPER.rstrip() + text[end:]
            break
    else:
        class_index = text.find("class TeleVuer:")
        if class_index < 0:
            raise SystemExit("Could not find TeleVuer class")
        text = text[:class_index] + HELPER + text[class_index:]

    if HANDLER_REPLACEMENT not in text:
        if HANDLER_INSERT not in text:
            raise SystemExit("Could not find TeleVuer camera handler registration")
        text = text.replace(HANDLER_INSERT, HANDLER_REPLACEMENT, 1)

    if METHOD_BLOCK not in text:
        if METHOD_INSERT not in text:
            raise SystemExit("Could not find TeleVuer on_cam_move method")
        text = text.replace(METHOD_INSERT, METHOD_BLOCK + METHOD_INSERT, 1)

    if CONTROLLER_REPLACEMENT not in text and CONTROLLER_INSERT in text:
        text = text.replace(CONTROLLER_INSERT, CONTROLLER_REPLACEMENT, 1)

    if HAND_REPLACEMENT not in text and HAND_INSERT in text:
        text = text.replace(HAND_INSERT, HAND_REPLACEMENT, 1)

    for point in INSERT_POINTS:
        replacement = "        _rtw_upsert_root_children_visual(session)\n\n" + point
        if replacement in text:
            continue
        if point not in text:
            raise SystemExit("Could not find WebRTC render loop insertion point")
        text = text.replace(point, replacement, 1)

    TELEVUER.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
