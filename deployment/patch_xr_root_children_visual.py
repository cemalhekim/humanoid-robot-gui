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
        "duration": 0.45,
    }


_rtw_loco_hold_lock = threading.Lock()
_rtw_loco_hold_stop = None
_rtw_loco_hold_key = None
_rtw_last_pointer_event_at = 0.0


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
        stop_event.wait(0.22)


def _rtw_stop_loco_hold():
    global _rtw_loco_hold_stop, _rtw_loco_hold_key
    with _rtw_loco_hold_lock:
        stop_event = _rtw_loco_hold_stop
        _rtw_loco_hold_stop = None
        _rtw_loco_hold_key = None
    if stop_event is not None:
        stop_event.set()
    try:
        _rtw_post_loco({"action": "stop_move"})
    except Exception as exc:
        print(f"[rtw loco pad] stop failed: {exc}")


def _rtw_start_loco_hold(key):
    global _rtw_loco_hold_stop, _rtw_loco_hold_key
    if not _rtw_root_children_visual_enabled():
        return
    payload = _rtw_loco_payload_for_key(key)
    if payload is None:
        return
    _rtw_stop_loco_hold()
    stop_event = threading.Event()
    with _rtw_loco_hold_lock:
        _rtw_loco_hold_stop = stop_event
        _rtw_loco_hold_key = str(key).split("-mark-", 1)[0]
    thread = threading.Thread(target=_rtw_loco_hold_loop, args=(key, stop_event), daemon=True)
    thread.start()


def _rtw_handle_loco_pad_pointer_down(key):
    global _rtw_last_pointer_event_at
    _rtw_last_pointer_event_at = __import__("time").time()
    _rtw_start_loco_hold(key)


def _rtw_handle_loco_pad_pointer_up(key):
    global _rtw_last_pointer_event_at
    _rtw_last_pointer_event_at = __import__("time").time()
    _rtw_stop_loco_hold()


def _rtw_handle_loco_pad_click(key):
    now = __import__("time").time()
    if now - _rtw_last_pointer_event_at < 0.35:
        return
    _rtw_start_loco_hold(key)


def _rtw_loco_button(key, x, y, color):
    return Box(
        key=key,
        position=[x, y, -1.35],
        scale=[0.36, 0.24, 0.04],
        color=color,
        onClick=True,
        onPointerDown=True,
        onPointerUp=True,
    )


def _rtw_loco_mark(key, x, y, sx, sy, color="#ffffff"):
    return Box(
        key=key,
        position=[x, y, -1.31],
        scale=[sx, sy, 0.035],
        color=color,
        onClick=True,
        onPointerDown=True,
        onPointerUp=True,
    )


def _rtw_upsert_root_children_visual(session):
    if not _rtw_root_children_visual_enabled():
        return
    session.upsert(
        [
            _rtw_loco_button("rtw-loco-turn-right", -0.5, 0.18, "#b8322a"),
            _rtw_loco_button("rtw-loco-forward", 0.0, 0.18, "#1f8f4d"),
            _rtw_loco_button("rtw-loco-turn-left", 0.5, 0.18, "#b8322a"),
            _rtw_loco_button("rtw-loco-left", -0.5, -0.18, "#2457d6"),
            _rtw_loco_button("rtw-loco-back", 0.0, -0.18, "#7a2fc2"),
            _rtw_loco_button("rtw-loco-right", 0.5, -0.18, "#2457d6"),
            _rtw_loco_mark("rtw-loco-turn-right-mark-a", -0.58, 0.21, 0.16, 0.04),
            _rtw_loco_mark("rtw-loco-turn-right-mark-b", -0.43, 0.13, 0.04, 0.16),
            _rtw_loco_mark("rtw-loco-turn-right-mark-c", -0.37, 0.08, 0.1, 0.04),
            _rtw_loco_mark("rtw-loco-forward-mark-a", 0.0, 0.18, 0.06, 0.18),
            _rtw_loco_mark("rtw-loco-forward-mark-b", -0.055, 0.265, 0.12, 0.04),
            _rtw_loco_mark("rtw-loco-forward-mark-c", 0.055, 0.265, 0.12, 0.04),
            _rtw_loco_mark("rtw-loco-turn-left-mark-a", 0.58, 0.21, 0.16, 0.04),
            _rtw_loco_mark("rtw-loco-turn-left-mark-b", 0.43, 0.13, 0.04, 0.16),
            _rtw_loco_mark("rtw-loco-turn-left-mark-c", 0.37, 0.08, 0.1, 0.04),
            _rtw_loco_mark("rtw-loco-left-mark-a", -0.5, -0.18, 0.18, 0.06),
            _rtw_loco_mark("rtw-loco-left-mark-b", -0.59, -0.12, 0.04, 0.12),
            _rtw_loco_mark("rtw-loco-left-mark-c", -0.59, -0.24, 0.04, 0.12),
            _rtw_loco_mark("rtw-loco-back-mark-a", 0.0, -0.18, 0.06, 0.18),
            _rtw_loco_mark("rtw-loco-back-mark-b", -0.055, -0.265, 0.12, 0.04),
            _rtw_loco_mark("rtw-loco-back-mark-c", 0.055, -0.265, 0.12, 0.04),
            _rtw_loco_mark("rtw-loco-right-mark-a", 0.5, -0.18, 0.18, 0.06),
            _rtw_loco_mark("rtw-loco-right-mark-b", 0.59, -0.12, 0.04, 0.12),
            _rtw_loco_mark("rtw-loco-right-mark-c", 0.59, -0.24, 0.04, 0.12),
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
        self.vuer.add_handler("ON_POINTER_DOWN")(self.on_rtw_root_loco_pointer_down)
        self.vuer.add_handler("ON_POINTER_UP")(self.on_rtw_root_loco_pointer_up)
"""

METHOD_INSERT = """    async def on_cam_move(self, event, session, fps=60):
"""
METHOD_BLOCK = """    async def on_rtw_root_loco_click(self, event, session):
        try:
            value = event.value or {}
            _rtw_handle_loco_pad_click(value.get("key"))
        except Exception as exc:
            print(f"[rtw loco pad] click handler failed: {exc}")

    async def on_rtw_root_loco_pointer_down(self, event, session):
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

    for line in (
        '        self.vuer.add_handler("ON_CLICK")(self.on_rtw_root_loco_click)\n',
        '        self.vuer.add_handler("ON_POINTER_DOWN")(self.on_rtw_root_loco_pointer_down)\n',
        '        self.vuer.add_handler("ON_POINTER_UP")(self.on_rtw_root_loco_pointer_up)\n',
    ):
        text = text.replace(line, "")

    method_start = text.find("    async def on_rtw_root_loco_click")
    if method_start >= 0:
        method_end = text.find("    async def on_cam_move", method_start)
        if method_end < 0:
            raise SystemExit("Could not find end of existing loco pad methods")
        text = text[:method_start] + text[method_end:]

    if HANDLER_REPLACEMENT not in text:
        if HANDLER_INSERT not in text:
            raise SystemExit("Could not find TeleVuer camera handler registration")
        text = text.replace(HANDLER_INSERT, HANDLER_REPLACEMENT, 1)

    if METHOD_BLOCK not in text:
        if METHOD_INSERT not in text:
            raise SystemExit("Could not find TeleVuer on_cam_move method")
        text = text.replace(METHOD_INSERT, METHOD_BLOCK + METHOD_INSERT, 1)

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
