#!/usr/bin/env python3
"""Patch TeleImager WebRTC page to auto-start the embedded preview."""

from __future__ import annotations

from pathlib import Path


IMAGE_SERVERS = [
    Path("/home/unitree/teleimager/src/teleimager/image_server.py"),
    Path("/home/unitree/xr_teleoperate/teleop/teleimager/src/teleimager/image_server.py"),
]

OLD = """function start() {
    var config = {
        sdpSemantics: 'unified-plan'
    };
"""

NEW = """function start() {
    if (pc) {
        return;
    }

    var config = {
        sdpSemantics: 'unified-plan'
    };
"""

OLD_STOP = """function stop() {
    document.getElementById('stop').style.display = 'none';
    document.getElementById('start').style.display = 'inline-block';
    if (pc) {
        pc.close();
        pc = null;
    }
}
"""

NEW_STOP = """function stop() {
    document.getElementById('stop').style.display = 'none';
    document.getElementById('start').style.display = 'inline-block';
    if (pc) {
        pc.close();
        pc = null;
    }
}

window.addEventListener('DOMContentLoaded', () => {
    start();
});
"""

OLD_VIDEO_STYLE = "video { width: 100%; max-width: 1280px; background: #000; margin-top: 10px; }"
NEW_VIDEO_STYLE = "video { width: 100%; max-width: 1920px; background: #000; margin-top: 10px; }"


def replace_once(text: str, old: str, new: str, label: str, required: bool = True) -> str:
    if new in text:
        return text
    if old not in text:
        if not required:
            return text
        raise SystemExit(f"Could not find TeleImager {label} block")
    return text.replace(old, new, 1)


def patch_start_guard(text: str) -> str:
    if "function start() {\n    if (pc) {" in text:
        return text
    return replace_once(text, OLD, NEW, "start", required=False)


def patch_auto_start(text: str) -> str:
    if "window.addEventListener('DOMContentLoaded'" in text or "window.addEventListener('load'" in text:
        return text
    return replace_once(text, OLD_STOP, NEW_STOP, "auto-start", required=False)


def main() -> int:
    for image_server in IMAGE_SERVERS:
        if not image_server.exists():
            continue
        text = image_server.read_text(encoding="utf-8")
        text = patch_start_guard(text)
        text = patch_auto_start(text)
        text = replace_once(text, OLD_VIDEO_STYLE, NEW_VIDEO_STYLE, "video style", required=False)
        image_server.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
