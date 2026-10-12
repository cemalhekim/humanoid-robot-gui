#!/usr/bin/env python3
"""Keep the arms where they are until the headset actually tracks the hands.

Upstream teleop_hand_and_arm.py feeds the IK every cycle. Before the first
hand event, and whenever a hand is not tracked (tv_wrapper falls back to
CONST_LEFT/RIGHT_ARM_POSE), the IK target is the fixed "arms stretched forward"
pose: on 2026-10-12 the arms rose to ~90 deg as soon as the Vuer page connected
and snapped there again whenever tracking dropped. With XR_TELEOP_SEND_START=1
nobody presses [r] after the hands are visible, so this happened on every
start. The patch exposes per-arm validity on TeleData and, for an arm that has
no valid tracking (or before motion data is ready), commands the arm's current
joint angles with zero feed-forward torque: it stays where Home (or the
previous session) left it and only starts following once its hand is tracked.
Idempotent; anchored on the upstream lines.
"""

from __future__ import annotations

import sys
from pathlib import Path


TV_WRAPPER = Path("/home/unitree/xr_teleoperate/teleop/televuer/src/televuer/tv_wrapper.py")
TELEOP = Path("/home/unitree/xr_teleoperate/teleop/teleop_hand_and_arm.py")

MARK = "XR_HOLD_UNTIL_TRACKED"

OLD_FIELD = "    motion_data_ready: bool = False        # True after the first hand or controller motion data event is received\n"
NEW_FIELD = OLD_FIELD + (
    f"    left_arm_is_valid: bool = True         # {MARK}: False while the left wrist pose is a fallback constant\n"
    f"    right_arm_is_valid: bool = True        # {MARK}: False while the right wrist pose is a fallback constant\n"
)

OLD_CTOR = (
    "            return TeleData(\n"
    "                head_pose=Brobot_world_head,\n"
    "                head_pose_updated_at=self.tvuer.head_pose_updated_at,\n"
    "                left_wrist_pose=left_IPunitree_Brobot_waist_arm,\n"
    "                right_wrist_pose=right_IPunitree_Brobot_waist_arm,\n"
    "                left_hand_pos=left_IPunitree_Brobot_arm_hand_pos,\n"
)
NEW_CTOR = (
    "            return TeleData(\n"
    "                head_pose=Brobot_world_head,\n"
    "                head_pose_updated_at=self.tvuer.head_pose_updated_at,\n"
    "                left_wrist_pose=left_IPunitree_Brobot_waist_arm,\n"
    "                right_wrist_pose=right_IPunitree_Brobot_waist_arm,\n"
    f"                left_arm_is_valid=bool(left_arm_is_valid),   # {MARK}\n"
    f"                right_arm_is_valid=bool(right_arm_is_valid),  # {MARK}\n"
    "                left_hand_pos=left_IPunitree_Brobot_arm_hand_pos,\n"
)

OLD_SOLVE = (
    "            sol_q, sol_tauff  = arm_ik.solve_ik(tele_data.left_wrist_pose, tele_data.right_wrist_pose, "
    "current_lr_arm_q, current_lr_arm_dq)\n"
)
NEW_SOLVE = OLD_SOLVE + f'''            # {MARK}: an arm without valid hand tracking (or before the first motion
            # event) holds its current joint angles instead of chasing the fallback
            # constant pose. Left arm = joints 0-6, right arm = joints 7-13.
            _hold_left = not tele_data.motion_data_ready or not getattr(tele_data, "left_arm_is_valid", True)
            _hold_right = not tele_data.motion_data_ready or not getattr(tele_data, "right_arm_is_valid", True)
            if _hold_left or _hold_right:
                sol_q = np.array(sol_q, dtype=float).copy()
                sol_tauff = np.array(sol_tauff, dtype=float).copy()
                if _hold_left:
                    sol_q[0:7] = current_lr_arm_q[0:7]
                    sol_tauff[0:7] = 0.0
                if _hold_right:
                    sol_q[7:14] = current_lr_arm_q[7:14]
                    sol_tauff[7:14] = 0.0
'''


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if new in text:
        return text
    if text.count(old) != 1:
        raise SystemExit(f"{label}: anchor not found exactly once")
    return text.replace(old, new, 1)


def main(argv: list[str]) -> int:
    wrapper = Path(argv[1]) if len(argv) > 1 else TV_WRAPPER
    teleop = Path(argv[2]) if len(argv) > 2 else TELEOP
    changed = []
    if wrapper.exists():
        text = wrapper.read_text(encoding="utf-8")
        patched = replace_once(text, OLD_FIELD, NEW_FIELD, "tv_wrapper TeleData fields")
        patched = replace_once(patched, OLD_CTOR, NEW_CTOR, "tv_wrapper TeleData constructor")
        if patched != text:
            wrapper.write_text(patched, encoding="utf-8")
            changed.append(str(wrapper))
    if teleop.exists():
        text = teleop.read_text(encoding="utf-8")
        patched = replace_once(text, OLD_SOLVE, NEW_SOLVE, "teleop solve_ik")
        # teleop_hand_and_arm.py has no `import numpy as np` of its own.
        if "\nimport numpy as np\n" not in patched:
            patched = replace_once(patched, "\nimport os\n", f"\nimport os\nimport numpy as np  # {MARK}\n", "teleop numpy import")
        if patched != text:
            teleop.write_text(patched, encoding="utf-8")
            changed.append(str(teleop))
    print("patched " + ", ".join(changed) if changed else "hold-until-tracked already applied")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
