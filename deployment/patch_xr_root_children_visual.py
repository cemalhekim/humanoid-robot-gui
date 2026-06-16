#!/usr/bin/env python3
"""Patch TeleVuer with an optional head-locked root scene visual test."""

from __future__ import annotations

from pathlib import Path


TELEVUER = Path("/home/unitree/xr_teleoperate/teleop/televuer/src/televuer/televuer.py")

MARKER = "# robot_telemetry_web root children visual test\n"

HELPER = '''# robot_telemetry_web root children visual test
def _rtw_root_children_visual_enabled():
    return os.getenv("XR_ROOT_CHILDREN_VISUAL", "0") == "1"


def _rtw_matrix_from_head(head_pose, x, y, z):
    pose = np.array(head_pose, dtype=float).reshape(4, 4, order="F")
    if not np.isfinite(pose).all() or abs(np.linalg.det(pose[:3, :3])) < 1e-6:
        pose = np.eye(4)
    offset = np.eye(4)
    offset[:3, 3] = [x, y, z]
    return (pose @ offset).flatten(order="F").tolist()


def _rtw_upsert_root_children_visual(session, head_pose):
    if not _rtw_root_children_visual_enabled():
        return
    session.upsert(
        [
            Box(
                key="rtw-root-visual-center",
                matrix=_rtw_matrix_from_head(head_pose, 0.0, -0.18, -1.15),
                scale=[0.45, 0.18, 0.04],
                color="#ff1744",
            ),
            Box(
                key="rtw-root-visual-left",
                matrix=_rtw_matrix_from_head(head_pose, -0.55, -0.18, -1.15),
                scale=[0.18, 0.18, 0.04],
                color="#ffd400",
            ),
            Box(
                key="rtw-root-visual-right",
                matrix=_rtw_matrix_from_head(head_pose, 0.55, -0.18, -1.15),
                scale=[0.18, 0.18, 0.04],
                color="#00e5ff",
            ),
            Box(
                key="rtw-root-visual-above",
                matrix=_rtw_matrix_from_head(head_pose, 0.0, 0.18, -1.15),
                scale=[0.22, 0.22, 0.04],
                color="#39ff14",
            ),
            Box(
                key="rtw-root-visual-alt-forward",
                matrix=_rtw_matrix_from_head(head_pose, 0.0, -0.18, 1.15),
                scale=[0.28, 0.28, 0.04],
                color="#ffffff",
            ),
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

OLD_STATIC_CALL = "        _rtw_upsert_root_children_visual(session)\\n\\n"


def main() -> int:
    if not TELEVUER.exists():
        return 0

    text = TELEVUER.read_text(encoding="utf-8")

    if IMPORT_NEW not in text:
        if IMPORT_OLD not in text:
            raise SystemExit("Could not find TeleVuer schema import line")
        text = text.replace(IMPORT_OLD, IMPORT_NEW, 1)

    if MARKER not in text:
        class_index = text.find("class TeleVuer:")
        if class_index < 0:
            raise SystemExit("Could not find TeleVuer class")
        text = text[:class_index] + HELPER + text[class_index:]
    elif "def _rtw_upsert_root_children_visual(session):" in text:
        start = text.index(MARKER)
        end = text.index("\n\nclass TeleVuer:", start)
        text = text[:start] + HELPER.rstrip() + text[end:]

    text = text.replace(OLD_STATIC_CALL, "")

    for point in INSERT_POINTS:
        replacement = point.replace("            session.upsert(", "            _rtw_upsert_root_children_visual(session, self.head_pose)\n            session.upsert(", 1)
        if replacement in text:
            continue
        if point not in text:
            raise SystemExit("Could not find WebRTC render loop insertion point")
        text = text.replace(point, replacement, 1)

    TELEVUER.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
