#!/usr/bin/env python3
"""Patch Inspire DFX hand control to use direct per-finger XR curl mapping."""

from __future__ import annotations

from pathlib import Path


TARGETS = [
    Path("/home/unitree/xr_teleoperate/teleop/robot_control/robot_hand_inspire.py"),
]

IMPORT_OLD = """import numpy as np
from enum import IntEnum
"""

IMPORT_NEW = """import numpy as np
import os
from enum import IntEnum
"""

HELPER = r'''
def _angle_curl(hand_data: np.ndarray, mcp: int, pip: int, tip: int) -> float:
    """Return 0=open, 1=closed from one finger's XR landmarks."""
    v1 = hand_data[mcp] - hand_data[pip]
    v2 = hand_data[tip] - hand_data[pip]
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    cos_angle = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    angle = np.arccos(cos_angle)
    return float(np.clip((np.pi - angle) / (np.pi * 0.55), 0.0, 1.0))


def _direct_inspire_targets(hand_data: np.ndarray) -> np.ndarray:
    """
    Map XR hand landmarks directly to Inspire DFX command order.
    Output order is [pinky, ring, middle, index, thumb_bend, thumb_rotation].
    1.0=open, 0.0=closed.
    """
    if hand_data is None or hand_data.shape != (25, 3) or not np.isfinite(hand_data).all():
        return np.ones(Inspire_Num_Motors, dtype=float)
    if np.allclose(hand_data, 0.0):
        return np.ones(Inspire_Num_Motors, dtype=float)

    # WebXR/OpenXR hand joint order:
    # wrist=0, thumb=1..4, index=5..9, middle=10..14, ring=15..19, little=20..24.
    pinky = max(_angle_curl(hand_data, 20, 21, 24), _angle_curl(hand_data, 21, 22, 24))
    ring = max(_angle_curl(hand_data, 15, 16, 19), _angle_curl(hand_data, 16, 17, 19))
    middle = max(_angle_curl(hand_data, 10, 11, 14), _angle_curl(hand_data, 11, 12, 14))
    index = max(_angle_curl(hand_data, 5, 6, 9), _angle_curl(hand_data, 6, 7, 9))
    thumb_bend = max(_angle_curl(hand_data, 1, 2, 4), _angle_curl(hand_data, 2, 3, 4))

    palm_width = np.linalg.norm(hand_data[5] - hand_data[17])
    thumb_to_index = np.linalg.norm(hand_data[4] - hand_data[9])
    if palm_width < 1e-6:
        thumb_rotation = thumb_bend
    else:
        thumb_rotation = float(np.clip(1.0 - thumb_to_index / (palm_width * 1.1), 0.0, 1.0))

    closed = np.array([pinky, ring, middle, index, thumb_bend, thumb_rotation], dtype=float)
    return 1.0 - np.clip(closed, 0.0, 1.0)

'''

CLASS_MARKER = "\nclass Inspire_Controller_DFX:\n"

OLD_BLOCK = """                if xr_motion_data_ready:
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                    # In website https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand, you can find
                    #     In the official document, the angles are in the range [0, 1] ==> 0.0: fully closed  1.0: fully open
                    # The q_target now is in radians, ranges:
                    #     - idx 0~3: 0~1.7 (1.7 = closed)
                    #     - idx 4:   0~0.5
                    #     - idx 5:  -0.1~1.3
                    # We normalize them using (max - value) / range
                    def normalize(val, min_val, max_val):
                        return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

                    for idx in range(Inspire_Num_Motors):
                        if idx <= 3:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                        elif idx == 4:
                            left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                            right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                        elif idx == 5:
                            left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                            right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)
"""

NEW_BLOCK = """                if xr_motion_data_ready:
                    use_direct_curl = os.getenv("XR_INSPIRE_DIRECT_CURL", "1") != "0"
                    if use_direct_curl:
                        left_q_target = _direct_inspire_targets(left_hand_data)
                        right_q_target = _direct_inspire_targets(right_hand_data)
                    else:
                        ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                        ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                        left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                        right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                        # In website https://support.unitree.com/home/en/G1_developer/inspire_dfx_dexterous_hand, you can find
                        #     In the official document, the angles are in the range [0, 1] ==> 0.0: fully closed  1.0: fully open
                        # The q_target now is in radians, ranges:
                        #     - idx 0~3: 0~1.7 (1.7 = closed)
                        #     - idx 4:   0~0.5
                        #     - idx 5:  -0.1~1.3
                        # We normalize them using (max - value) / range
                        def normalize(val, min_val, max_val):
                            return np.clip((max_val - val) / (max_val - min_val), 0.0, 1.0)

                        for idx in range(Inspire_Num_Motors):
                            if idx <= 3:
                                left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 1.7)
                                right_q_target[idx] = normalize(right_q_target[idx], 0.0, 1.7)
                            elif idx == 4:
                                left_q_target[idx]  = normalize(left_q_target[idx], 0.0, 0.5)
                                right_q_target[idx] = normalize(right_q_target[idx], 0.0, 0.5)
                            elif idx == 5:
                                left_q_target[idx]  = normalize(left_q_target[idx], -0.1, 1.3)
                                right_q_target[idx] = normalize(right_q_target[idx], -0.1, 1.3)
"""


def patch_file(path: Path) -> None:
    if not path.exists():
        return

    text = path.read_text(encoding="utf-8")
    if "import os\nfrom enum import IntEnum" not in text:
        if IMPORT_OLD not in text:
            raise SystemExit(f"Could not find import block in {path}")
        text = text.replace(IMPORT_OLD, IMPORT_NEW, 1)

    if "def _direct_inspire_targets(" not in text:
        if CLASS_MARKER not in text:
            raise SystemExit(f"Could not find Inspire DFX class marker in {path}")
        text = text.replace(CLASS_MARKER, HELPER + CLASS_MARKER, 1)

    if "XR_INSPIRE_DIRECT_CURL" not in text:
        if OLD_BLOCK not in text:
            raise SystemExit(f"Could not find Inspire retargeting block in {path}")
        text = text.replace(OLD_BLOCK, NEW_BLOCK, 1)

    text = text.replace(
        "    pinky = _angle_curl(hand_data, 20, 21, 24)\n"
        "    ring = _angle_curl(hand_data, 15, 16, 19)\n"
        "    middle = _angle_curl(hand_data, 10, 11, 14)\n"
        "    index = _angle_curl(hand_data, 5, 6, 9)\n"
        "    thumb_bend = _angle_curl(hand_data, 1, 2, 4)\n",
        "    pinky = max(_angle_curl(hand_data, 20, 21, 24), _angle_curl(hand_data, 21, 22, 24))\n"
        "    ring = max(_angle_curl(hand_data, 15, 16, 19), _angle_curl(hand_data, 16, 17, 19))\n"
        "    middle = max(_angle_curl(hand_data, 10, 11, 14), _angle_curl(hand_data, 11, 12, 14))\n"
        "    index = max(_angle_curl(hand_data, 5, 6, 9), _angle_curl(hand_data, 6, 7, 9))\n"
        "    thumb_bend = max(_angle_curl(hand_data, 1, 2, 4), _angle_curl(hand_data, 2, 3, 4))\n",
    )

    path.write_text(text, encoding="utf-8")


def main() -> int:
    for path in TARGETS:
        patch_file(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
