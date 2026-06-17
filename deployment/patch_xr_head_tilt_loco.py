#!/usr/bin/env python3
"""Patch XR teleop with optional head Y/P/R locomotion."""

from __future__ import annotations

import os
from pathlib import Path


ROOTS = [
    Path(os.getenv("XR_TELEOPERATE_DIR", "/home/unitree/xr_teleoperate")),
    Path(__file__).resolve().parents[1] / "teleoperation/vision_pro_control/external/xr_teleoperate",
]

HELPER_MARKER = "# robot_telemetry_web head tilt loco\n"

HELPER = r'''# robot_telemetry_web head tilt loco
def _rtw_env_bool(name, default=False):
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _rtw_env_float(name, default):
    value = os.getenv(name)
    if value is None:
        return float(default)
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        logger_mp.warning(f"[head tilt loco] invalid {name}={value!r}; using {default}")
        return float(default)
    if not math.isfinite(parsed):
        logger_mp.warning(f"[head tilt loco] non-finite {name}={value!r}; using {default}")
        return float(default)
    return parsed


def _rtw_clamp(value, low, high):
    return max(low, min(high, value))


class HeadTiltLocoController:
    def __init__(self):
        self.enabled = _rtw_env_bool("XR_HEAD_TILT_LOCO", False)
        self.neutral_rot = None
        self.active = False
        self.last_log_at = 0.0
        self.pitch_deadzone = math.radians(_rtw_env_float("XR_HEAD_TILT_PITCH_DEADZONE_DEG", 10.0))
        self.roll_deadzone = math.radians(_rtw_env_float("XR_HEAD_TILT_ROLL_DEADZONE_DEG", 10.0))
        self.yaw_deadzone = math.radians(_rtw_env_float("XR_HEAD_TILT_YAW_DEADZONE_DEG", 12.0))
        self.pitch_full = math.radians(_rtw_env_float("XR_HEAD_TILT_PITCH_FULL_DEG", 28.0))
        self.roll_full = math.radians(_rtw_env_float("XR_HEAD_TILT_ROLL_FULL_DEG", 28.0))
        self.yaw_full = math.radians(_rtw_env_float("XR_HEAD_TILT_YAW_FULL_DEG", 35.0))
        self.max_vx = _rtw_env_float("XR_HEAD_TILT_MAX_VX", 0.25)
        self.max_vy = _rtw_env_float("XR_HEAD_TILT_MAX_VY", 0.18)
        self.max_vyaw = _rtw_env_float("XR_HEAD_TILT_MAX_VYAW", 0.35)
        self.pitch_sign = _rtw_env_float("XR_HEAD_TILT_PITCH_SIGN", -1.0)
        self.roll_sign = _rtw_env_float("XR_HEAD_TILT_ROLL_SIGN", 1.0)
        self.yaw_sign = _rtw_env_float("XR_HEAD_TILT_YAW_SIGN", 1.0)

    @staticmethod
    def _relative_ypr(rotation):
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        pitch = math.asin(_rtw_clamp(-rotation[2, 0], -1.0, 1.0))
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        return yaw, pitch, roll

    @staticmethod
    def _axis_to_unit(angle, deadzone, full):
        magnitude = abs(angle)
        if magnitude <= deadzone:
            return 0.0
        span = max(full - deadzone, 1e-6)
        scaled = _rtw_clamp((magnitude - deadzone) / span, 0.0, 1.0)
        return math.copysign(scaled, angle)

    def update(self, tele_data, loco_wrapper):
        if not self.enabled:
            return
        try:
            head_rot = tele_data.head_pose[:3, :3]
            if self.neutral_rot is None:
                self.neutral_rot = head_rot.copy()
                logger_mp.info("[head tilt loco] calibrated neutral head pose")
                return
            rel_rot = self.neutral_rot.T @ head_rot
            yaw, pitch, roll = self._relative_ypr(rel_rot)
            pitch_unit = self._axis_to_unit(pitch, self.pitch_deadzone, self.pitch_full)
            roll_unit = self._axis_to_unit(roll, self.roll_deadzone, self.roll_full)
            yaw_unit = self._axis_to_unit(yaw, self.yaw_deadzone, self.yaw_full)
            vx = _rtw_clamp(self.pitch_sign * pitch_unit * self.max_vx, -self.max_vx, self.max_vx)
            vy = _rtw_clamp(self.roll_sign * roll_unit * self.max_vy, -self.max_vy, self.max_vy)
            vyaw = _rtw_clamp(self.yaw_sign * yaw_unit * self.max_vyaw, -self.max_vyaw, self.max_vyaw)
            if vx == 0.0 and vy == 0.0 and vyaw == 0.0:
                if self.active:
                    loco_wrapper.Move(0.0, 0.0, 0.0)
                    self.active = False
                    logger_mp.info("[head tilt loco] neutral tolerance reached; stop_move sent")
                return
            loco_wrapper.Move(vx, vy, vyaw)
            self.active = True
            now = time.time()
            if now - self.last_log_at > 1.0:
                logger_mp.info(
                    "[head tilt loco] ypr_deg=(%.1f, %.1f, %.1f) velocity=(%.2f, %.2f, %.2f)",
                    math.degrees(yaw), math.degrees(pitch), math.degrees(roll), vx, vy, vyaw,
                )
                self.last_log_at = now
        except Exception as exc:
            logger_mp.warning(f"[head tilt loco] update failed: {exc}")

    def stop(self, loco_wrapper):
        if not self.enabled or not self.active or loco_wrapper is None:
            return
        try:
            loco_wrapper.Move(0.0, 0.0, 0.0)
        except Exception as exc:
            logger_mp.warning(f"[head tilt loco] stop failed: {exc}")
        self.active = False

'''


def patch_file(path: Path) -> bool:
    if not path.exists():
        return False
    text = path.read_text(encoding="utf-8")
    original = text

    if "import math\n" not in text:
        text = text.replace("import time\n", "import time\nimport math\n", 1)

    if HELPER_MARKER not in text:
        insert = 'def on_press(key):\n'
        if insert not in text:
            raise SystemExit(f"Could not find on_press insertion point in {path}")
        text = text.replace(insert, HELPER + insert, 1)

    old_motion = '''        # motion mode (G1: Regular mode R1+X, not Running mode R2+A)
        if args.motion:
            if args.input_mode == "controller":
                loco_wrapper = LocoClientWrapper()
        else:
            motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Enter_Debug_Mode()
            logger_mp.info(f"Enter debug mode: {'Success' if status == 0 else 'Failed'}")
'''
    new_motion = '''        head_tilt_loco = HeadTiltLocoController()
        loco_wrapper = None

        # motion mode (G1: Regular mode R1+X, not Running mode R2+A)
        if args.motion:
            if args.input_mode == "controller" or head_tilt_loco.enabled:
                loco_wrapper = LocoClientWrapper()
            if head_tilt_loco.enabled:
                logger_mp.info("[head tilt loco] enabled; VR control pad should be disabled for option 2")
        else:
            if head_tilt_loco.enabled:
                logger_mp.warning("[head tilt loco] disabled because --motion is not enabled")
                head_tilt_loco.enabled = False
            motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Enter_Debug_Mode()
            logger_mp.info(f"Enter debug mode: {'Success' if status == 0 else 'Failed'}")
'''
    if old_motion in text:
        text = text.replace(old_motion, new_motion, 1)

    old_loop = '''            # high level control
            if args.input_mode == "controller" and args.motion:
                # quit teleoperate
'''
    new_loop = '''            # high level control
            if head_tilt_loco.enabled and args.motion:
                head_tilt_loco.update(tele_data, loco_wrapper)
            elif args.input_mode == "controller" and args.motion:
                # quit teleoperate
'''
    if old_loop in text:
        text = text.replace(old_loop, new_loop, 1)

    old_finally = '''    finally:
        try:
            arm_ctrl.ctrl_dual_arm_go_home()
'''
    new_finally = '''    finally:
        try:
            head_tilt_loco.stop(loco_wrapper)
        except Exception as e:
            logger_mp.error(f"Failed to stop head tilt loco: {e}")

        try:
            arm_ctrl.ctrl_dual_arm_go_home()
'''
    if old_finally in text:
        text = text.replace(old_finally, new_finally, 1)

    if text != original:
        path.write_text(text, encoding="utf-8")
        return True
    return False


def main() -> int:
    changed = []
    seen = set()
    for root in ROOTS:
        path = root / "teleop/teleop_hand_and_arm.py"
        if path in seen:
            continue
        seen.add(path)
        if patch_file(path):
            changed.append(str(path))
    if changed:
        print("Patched head tilt loco in:")
        for path in changed:
            print(f"  {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
