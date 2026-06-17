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


def _rtw_wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class HeadTiltLocoController:
    def __init__(self):
        self.enabled = _rtw_env_bool("XR_HEAD_TILT_LOCO", False)
        self.neutral_rot = None
        self.active = False
        self.last_log_at = 0.0
        self.pitch_deadzone = math.radians(_rtw_env_float("XR_HEAD_TILT_PITCH_DEADZONE_DEG", 18.0))
        self.roll_deadzone = math.radians(_rtw_env_float("XR_HEAD_TILT_ROLL_DEADZONE_DEG", 12.0))
        self.yaw_deadzone = math.radians(_rtw_env_float("XR_HEAD_TILT_YAW_DEADZONE_DEG", 20.0))
        self.pitch_full = math.radians(_rtw_env_float("XR_HEAD_TILT_PITCH_FULL_DEG", 42.0))
        self.roll_full = math.radians(_rtw_env_float("XR_HEAD_TILT_ROLL_FULL_DEG", 30.0))
        self.yaw_full = math.radians(_rtw_env_float("XR_HEAD_TILT_YAW_FULL_DEG", 50.0))
        self.max_vx = _rtw_env_float("XR_HEAD_TILT_MAX_VX", 0.14)
        self.max_vy = _rtw_env_float("XR_HEAD_TILT_MAX_VY", 0.16)
        self.max_vyaw = _rtw_env_float("XR_HEAD_TILT_MAX_VYAW", 0.20)
        self.pitch_sign = _rtw_env_float("XR_HEAD_TILT_PITCH_SIGN", -1.0)
        self.roll_sign = _rtw_env_float("XR_HEAD_TILT_ROLL_SIGN", -1.0)
        self.yaw_sign = _rtw_env_float("XR_HEAD_TILT_YAW_SIGN", 1.0)
        self.last_command_at = 0.0
        self.command_period = _rtw_env_float("XR_HEAD_TILT_COMMAND_PERIOD", 0.18)
        self.require_client = _rtw_env_bool("XR_HEAD_TILT_REQUIRE_CLIENT", True)
        self.xr_port = int(_rtw_env_float("XR_TELEOP_VUER_PORT", 8012))
        self.max_pose_age = _rtw_env_float("XR_HEAD_TILT_MAX_POSE_AGE", 1.0)
        self.calibration_delay = _rtw_env_float("XR_HEAD_TILT_CALIBRATION_DELAY", 1.5)
        self.calibration_started_at = None
        self.client_was_connected = False
        self.last_no_client_stop_at = 0.0

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

    @staticmethod
    def _post_loco(payload):
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

    @staticmethod
    def _request_loco(payload):
        import json
        import urllib.request

        body = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            "http://127.0.0.1:8088/api/loco/command",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=0.8) as response:
            return json.loads(response.read().decode("utf-8", errors="replace") or "{}")

    def _has_xr_client(self):
        port_hex = f"{self.xr_port:04X}"
        for proc_path in ("/proc/net/tcp", "/proc/net/tcp6"):
            try:
                with open(proc_path, "r", encoding="utf-8") as handle:
                    rows = handle.readlines()[1:]
            except OSError:
                continue
            for row in rows:
                fields = row.split()
                if len(fields) < 4:
                    continue
                local = fields[1]
                state = fields[3]
                if state == "01" and local.rsplit(":", 1)[-1].upper() == port_hex:
                    return True
        return False

    def _handle_no_client(self):
        now = time.time()
        self.neutral_rot = None
        self.calibration_started_at = None
        should_stop = self.active or self.client_was_connected
        if should_stop:
            try:
                self._post_loco({"action": "stop_move"})
            except Exception as exc:
                logger_mp.warning(f"[head tilt loco] no-client stop failed: {exc}")
            self.last_no_client_stop_at = now
        if self.client_was_connected and now - self.last_log_at > 1.0:
            logger_mp.info("[head tilt loco] VR client disconnected; stop_move sent")
            self.last_log_at = now
        self.active = False
        self.client_was_connected = False

    def _handle_stale_pose(self, age):
        self.neutral_rot = None
        self.calibration_started_at = None
        if self.active:
            try:
                self._post_loco({"action": "stop_move"})
            except Exception as exc:
                logger_mp.warning(f"[head tilt loco] stale-pose stop failed: {exc}")
            self.active = False
            now = time.time()
            if now - self.last_log_at > 1.0:
                logger_mp.info("[head tilt loco] head pose stale for %.2fs; stop_move sent", age)
                self.last_log_at = now

    def update(self, tele_data, loco_wrapper=None):
        if not self.enabled:
            return
        try:
            if self.require_client:
                has_client = self._has_xr_client()
                if not has_client:
                    self._handle_no_client()
                    return
                if not self.client_was_connected:
                    self.neutral_rot = None
                    self.calibration_started_at = None
                    logger_mp.info("[head tilt loco] VR client connected; recalibrating neutral pose")
                self.client_was_connected = True

            head_pose_updated_at = float(getattr(tele_data, "head_pose_updated_at", 0.0) or 0.0)
            pose_age = time.time() - head_pose_updated_at if head_pose_updated_at > 0.0 else float("inf")
            if pose_age > self.max_pose_age:
                self._handle_stale_pose(pose_age)
                return

            head_rot = tele_data.head_pose[:3, :3]
            if self.calibration_started_at is None:
                self.calibration_started_at = time.time()
                self.neutral_rot = head_rot.copy()
                logger_mp.info("[head tilt loco] neutral settle started")
                return
            if time.time() - self.calibration_started_at < self.calibration_delay:
                self.neutral_rot = head_rot.copy()
                return
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
                    self._post_loco({"action": "stop_move"})
                    self.active = False
                    logger_mp.info("[head tilt loco] neutral tolerance reached; stop_move sent")
                return
            now = time.time()
            if now - self.last_command_at < self.command_period:
                return
            self._post_loco({
                "action": "velocity",
                "vx": vx,
                "vy": vy,
                "vyaw": vyaw,
                "duration": max(self.command_period * 2.0, 0.35),
            })
            self.last_command_at = now
            self.active = True
            if now - self.last_log_at > 1.0:
                logger_mp.info(
                    "[head tilt loco] ypr_deg=(%.1f, %.1f, %.1f) velocity=(%.2f, %.2f, %.2f)",
                    math.degrees(yaw), math.degrees(pitch), math.degrees(roll), vx, vy, vyaw,
                )
                self.last_log_at = now
        except Exception as exc:
            logger_mp.warning(f"[head tilt loco] update failed: {exc}")

    def stop(self, loco_wrapper=None):
        if not self.enabled or not self.active:
            return
        try:
            self._post_loco({"action": "stop_move"})
        except Exception as exc:
            logger_mp.warning(f"[head tilt loco] stop failed: {exc}")
        self.active = False


class PositionMatchLocoController(HeadTiltLocoController):
    def __init__(self):
        super().__init__()
        self.enabled = _rtw_env_bool("XR_POSITION_MATCH_LOCO", False)
        self.neutral_pos = None
        self.neutral_rot = None
        self.estimated_x = 0.0
        self.estimated_y = 0.0
        self.estimated_yaw = 0.0
        self.last_integrated_at = None
        self.last_cmd_x = 0.0
        self.last_cmd_y = 0.0
        self.last_cmd_yaw = 0.0
        self.last_target_log_at = 0.0
        self.scale_x = _rtw_env_float("XR_POSITION_MATCH_SCALE_X", 1.0)
        self.scale_y = _rtw_env_float("XR_POSITION_MATCH_SCALE_Y", 1.0)
        self.scale_yaw = _rtw_env_float("XR_POSITION_MATCH_SCALE_YAW", 1.0)
        self.deadband_x = _rtw_env_float("XR_POSITION_MATCH_DEADBAND_X", 0.04)
        self.deadband_y = _rtw_env_float("XR_POSITION_MATCH_DEADBAND_Y", 0.04)
        self.deadband_yaw = math.radians(_rtw_env_float("XR_POSITION_MATCH_DEADBAND_YAW_DEG", 6.0))
        self.max_target_x = _rtw_env_float("XR_POSITION_MATCH_MAX_TARGET_X", 1.0)
        self.max_target_y = _rtw_env_float("XR_POSITION_MATCH_MAX_TARGET_Y", 1.0)
        self.max_target_yaw = math.radians(_rtw_env_float("XR_POSITION_MATCH_MAX_TARGET_YAW_DEG", 60.0))
        self.kx = _rtw_env_float("XR_POSITION_MATCH_KX", 0.8)
        self.ky = _rtw_env_float("XR_POSITION_MATCH_KY", 0.8)
        self.kyaw = _rtw_env_float("XR_POSITION_MATCH_KYAW", 0.8)
        self.max_vx = _rtw_env_float("XR_POSITION_MATCH_MAX_VX", 0.16)
        self.max_vy = _rtw_env_float("XR_POSITION_MATCH_MAX_VY", 0.16)
        self.max_vyaw = _rtw_env_float("XR_POSITION_MATCH_MAX_VYAW", 0.25)

    def _reset_reference(self):
        self.neutral_pos = None
        self.neutral_rot = None
        self.estimated_x = 0.0
        self.estimated_y = 0.0
        self.estimated_yaw = 0.0
        self.last_integrated_at = None
        self.last_cmd_x = 0.0
        self.last_cmd_y = 0.0
        self.last_cmd_yaw = 0.0
        self.calibration_started_at = None

    def _handle_no_client(self):
        super()._handle_no_client()
        self._reset_reference()

    def _handle_stale_pose(self, age):
        super()._handle_stale_pose(age)
        self._reset_reference()

    def _stop_active(self, message):
        if self.active:
            try:
                self._post_loco({"action": "stop_move"})
            except Exception as exc:
                logger_mp.warning(f"[position match loco] stop failed: {exc}")
            self.active = False
            now = time.time()
            self.last_integrated_at = now
            self.last_cmd_x = 0.0
            self.last_cmd_y = 0.0
            self.last_cmd_yaw = 0.0
            if now - self.last_log_at > 1.0:
                logger_mp.info(message)
                self.last_log_at = now

    def _integrate_estimate(self, now):
        if self.last_integrated_at is None:
            self.last_integrated_at = now
            return
        dt = _rtw_clamp(now - self.last_integrated_at, 0.0, 0.5)
        self.estimated_x += self.last_cmd_x * dt
        self.estimated_y += self.last_cmd_y * dt
        self.estimated_yaw = _rtw_wrap_angle(self.estimated_yaw + self.last_cmd_yaw * dt)
        self.last_integrated_at = now

    def update(self, tele_data, loco_wrapper=None):
        if not self.enabled:
            return
        try:
            now = time.time()
            self._integrate_estimate(now)

            if self.require_client:
                has_client = self._has_xr_client()
                if not has_client:
                    self._handle_no_client()
                    return
                if not self.client_was_connected:
                    self._reset_reference()
                    logger_mp.info("[position match loco] VR client connected; calibrating reference")
                self.client_was_connected = True

            head_pose_updated_at = float(getattr(tele_data, "head_pose_updated_at", 0.0) or 0.0)
            pose_age = time.time() - head_pose_updated_at if head_pose_updated_at > 0.0 else float("inf")
            if pose_age > self.max_pose_age:
                self._handle_stale_pose(pose_age)
                return

            head_rot = tele_data.head_pose[:3, :3]
            head_pos = tele_data.head_pose[:3, 3].copy()

            if self.calibration_started_at is None:
                self.calibration_started_at = now
                self.neutral_pos = head_pos
                self.neutral_rot = head_rot.copy()
                self.estimated_x = 0.0
                self.estimated_y = 0.0
                self.estimated_yaw = 0.0
                self.last_integrated_at = now
                logger_mp.info("[position match loco] neutral settle started; using commanded-distance estimate")
                return
            if now - self.calibration_started_at < self.calibration_delay:
                self.neutral_pos = head_pos
                self.neutral_rot = head_rot.copy()
                self.estimated_x = 0.0
                self.estimated_y = 0.0
                self.estimated_yaw = 0.0
                self.last_integrated_at = now
                return
            if self.neutral_pos is None or self.neutral_rot is None:
                self.neutral_pos = head_pos
                self.neutral_rot = head_rot.copy()
                self.estimated_x = 0.0
                self.estimated_y = 0.0
                self.estimated_yaw = 0.0
                self.last_integrated_at = now
                logger_mp.info("[position match loco] calibrated neutral pose and command estimate")
                return

            local_delta = self.neutral_rot.T @ (head_pos - self.neutral_pos)
            rel_rot = self.neutral_rot.T @ head_rot
            yaw, _pitch, _roll = self._relative_ypr(rel_rot)

            target_x = _rtw_clamp(float(local_delta[0]) * self.scale_x, -self.max_target_x, self.max_target_x)
            target_y = _rtw_clamp(float(local_delta[1]) * self.scale_y, -self.max_target_y, self.max_target_y)
            target_yaw = _rtw_clamp(yaw * self.scale_yaw, -self.max_target_yaw, self.max_target_yaw)

            err_x = target_x - self.estimated_x
            err_y = target_y - self.estimated_y
            err_yaw = _rtw_wrap_angle(target_yaw - self.estimated_yaw)

            cmd_x = 0.0 if abs(err_x) < self.deadband_x else _rtw_clamp(self.kx * err_x, -self.max_vx, self.max_vx)
            cmd_y = 0.0 if abs(err_y) < self.deadband_y else _rtw_clamp(self.ky * err_y, -self.max_vy, self.max_vy)
            cmd_yaw = 0.0 if abs(err_yaw) < self.deadband_yaw else _rtw_clamp(self.kyaw * err_yaw, -self.max_vyaw, self.max_vyaw)

            if cmd_x == 0.0 and cmd_y == 0.0 and cmd_yaw == 0.0:
                self._stop_active("[position match loco] target reached; stop_move sent")
                return

            if now - self.last_command_at < self.command_period:
                return
            self._post_loco({
                "action": "velocity",
                "vx": cmd_x,
                "vy": cmd_y,
                "vyaw": cmd_yaw,
                "duration": max(self.command_period * 2.0, 0.35),
            })
            self.last_command_at = now
            self.last_cmd_x = cmd_x
            self.last_cmd_y = cmd_y
            self.last_cmd_yaw = cmd_yaw
            self.active = True
            if now - self.last_target_log_at > 1.0:
                logger_mp.info(
                    "[position match loco] target=(%.2f, %.2f, %.1fdeg) estimated=(%.2f, %.2f, %.1fdeg) velocity=(%.2f, %.2f, %.2f)",
                    target_x, target_y, math.degrees(target_yaw),
                    self.estimated_x, self.estimated_y, math.degrees(self.estimated_yaw),
                    cmd_x, cmd_y, cmd_yaw,
                )
                self.last_target_log_at = now
        except Exception as exc:
            logger_mp.warning(f"[position match loco] update failed: {exc}")

'''


def patch_file(path: Path) -> bool:
    if not path.exists():
        return False
    text = path.read_text(encoding="utf-8")
    original = text

    if "import math\n" not in text:
        text = text.replace("import time\n", "import time\nimport math\n", 1)

    insert = 'def on_press(key):\n'
    if insert not in text:
        raise SystemExit(f"Could not find on_press insertion point in {path}")
    if HELPER_MARKER in text:
        start = text.index(HELPER_MARKER)
        end = text.index(insert, start)
        text = text[:start] + HELPER + text[end:]
    else:
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
        position_match_loco = PositionMatchLocoController()
        loco_wrapper = None

        # motion mode (G1: Regular mode R1+X, not Running mode R2+A)
        if args.motion:
            if args.input_mode == "controller":
                loco_wrapper = LocoClientWrapper()
            if head_tilt_loco.enabled:
                logger_mp.info("[head tilt loco] enabled; commands route through dashboard H1 loco API")
            if position_match_loco.enabled:
                logger_mp.info("[position match loco] enabled; Vision Pro displacement routes through dashboard H1 loco API")
        else:
            if head_tilt_loco.enabled:
                logger_mp.warning("[head tilt loco] disabled because --motion is not enabled")
                head_tilt_loco.enabled = False
            if position_match_loco.enabled:
                logger_mp.warning("[position match loco] disabled because --motion is not enabled")
                position_match_loco.enabled = False
            motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Enter_Debug_Mode()
            logger_mp.info(f"Enter debug mode: {'Success' if status == 0 else 'Failed'}")
'''
    if old_motion in text:
        text = text.replace(old_motion, new_motion, 1)
    if "position_match_loco = PositionMatchLocoController()" not in text:
        text = text.replace(
            "        head_tilt_loco = HeadTiltLocoController()\n"
            "        loco_wrapper = None\n",
            "        head_tilt_loco = HeadTiltLocoController()\n"
            "        position_match_loco = PositionMatchLocoController()\n"
            "        loco_wrapper = None\n",
            1,
        )
    if "[position match loco] enabled; Vision Pro displacement routes through dashboard H1 loco API" not in text:
        text = text.replace(
            '            if head_tilt_loco.enabled:\n'
            '                logger_mp.info("[head tilt loco] enabled; commands route through dashboard H1 loco API")\n',
            '            if head_tilt_loco.enabled:\n'
            '                logger_mp.info("[head tilt loco] enabled; commands route through dashboard H1 loco API")\n'
            '            if position_match_loco.enabled:\n'
            '                logger_mp.info("[position match loco] enabled; Vision Pro displacement routes through dashboard H1 loco API")\n',
            1,
        )
    if "[position match loco] disabled because --motion is not enabled" not in text:
        text = text.replace(
            '            if head_tilt_loco.enabled:\n'
            '                logger_mp.warning("[head tilt loco] disabled because --motion is not enabled")\n'
            '                head_tilt_loco.enabled = False\n',
            '            if head_tilt_loco.enabled:\n'
            '                logger_mp.warning("[head tilt loco] disabled because --motion is not enabled")\n'
            '                head_tilt_loco.enabled = False\n'
            '            if position_match_loco.enabled:\n'
            '                logger_mp.warning("[position match loco] disabled because --motion is not enabled")\n'
            '                position_match_loco.enabled = False\n',
            1,
        )
    text = text.replace(
        '            if args.input_mode == "controller" or head_tilt_loco.enabled:\n'
        '                loco_wrapper = LocoClientWrapper()\n'
        '            if head_tilt_loco.enabled:\n'
        '                logger_mp.info("[head tilt loco] enabled; VR control pad should be disabled for option 2")\n',
        '            if args.input_mode == "controller":\n'
        '                loco_wrapper = LocoClientWrapper()\n'
        '            if head_tilt_loco.enabled:\n'
        '                logger_mp.info("[head tilt loco] enabled; commands route through dashboard H1 loco API")\n',
        1,
    )

    old_loop = '''            # high level control
            if args.input_mode == "controller" and args.motion:
                # quit teleoperate
'''
    new_loop = '''            # high level control
            if head_tilt_loco.enabled and args.motion:
                head_tilt_loco.update(tele_data, loco_wrapper)
            elif position_match_loco.enabled and args.motion:
                position_match_loco.update(tele_data, loco_wrapper)
            elif args.input_mode == "controller" and args.motion:
                # quit teleoperate
'''
    if old_loop in text:
        text = text.replace(old_loop, new_loop, 1)
    if "position_match_loco.update(tele_data, loco_wrapper)" not in text:
        text = text.replace(
            "            if head_tilt_loco.enabled and args.motion:\n"
            "                head_tilt_loco.update(tele_data, loco_wrapper)\n"
            "            elif args.input_mode == \"controller\" and args.motion:\n",
            "            if head_tilt_loco.enabled and args.motion:\n"
            "                head_tilt_loco.update(tele_data, loco_wrapper)\n"
            "            elif position_match_loco.enabled and args.motion:\n"
            "                position_match_loco.update(tele_data, loco_wrapper)\n"
            "            elif args.input_mode == \"controller\" and args.motion:\n",
            1,
        )

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
            position_match_loco.stop(loco_wrapper)
        except Exception as e:
            logger_mp.error(f"Failed to stop position match loco: {e}")

        try:
            arm_ctrl.ctrl_dual_arm_go_home()
'''
    if old_finally in text:
        text = text.replace(old_finally, new_finally, 1)
    if "Failed to stop position match loco" not in text:
        text = text.replace(
            '        try:\n'
            '            head_tilt_loco.stop(loco_wrapper)\n'
            '        except Exception as e:\n'
            '            logger_mp.error(f"Failed to stop head tilt loco: {e}")\n\n'
            '        try:\n'
            '            arm_ctrl.ctrl_dual_arm_go_home()\n',
            '        try:\n'
            '            head_tilt_loco.stop(loco_wrapper)\n'
            '        except Exception as e:\n'
            '            logger_mp.error(f"Failed to stop head tilt loco: {e}")\n\n'
            '        try:\n'
            '            position_match_loco.stop(loco_wrapper)\n'
            '        except Exception as e:\n'
            '            logger_mp.error(f"Failed to stop position match loco: {e}")\n\n'
            '        try:\n'
            '            arm_ctrl.ctrl_dual_arm_go_home()\n',
            1,
        )

    if text != original:
        path.write_text(text, encoding="utf-8")
        return True
    return False


def patch_televuer(path: Path) -> bool:
    if not path.exists():
        return False
    text = path.read_text(encoding="utf-8")
    original = text

    if "import time\n" not in text:
        text = text.replace("import os\n", "import os\nimport time\n", 1)

    if "self.head_pose_updated_at_shared = Value('d', 0.0, lock=True)" not in text:
        text = text.replace(
            "        self.head_pose_shared = Array('d', 16, lock=True)\n",
            "        self.head_pose_shared = Array('d', 16, lock=True)\n"
            "        self.head_pose_updated_at_shared = Value('d', 0.0, lock=True)\n",
            1,
        )

    old_cam = '''    async def on_cam_move(self, event, session, fps=60):
        try:
            with self.head_pose_shared.get_lock():
                self.head_pose_shared[:] = event.value["camera"]["matrix"]
        except:
            pass
'''
    new_cam = '''    async def on_cam_move(self, event, session, fps=60):
        try:
            with self.head_pose_shared.get_lock():
                self.head_pose_shared[:] = event.value["camera"]["matrix"]
            with self.head_pose_updated_at_shared.get_lock():
                self.head_pose_updated_at_shared.value = time.time()
        except:
            pass
'''
    if old_cam in text:
        text = text.replace(old_cam, new_cam, 1)

    if "def head_pose_updated_at(self):" not in text:
        marker = '''    @property
    def left_arm_pose(self):
'''
        addition = '''    @property
    def head_pose_updated_at(self):
        """float, wall-clock timestamp of the latest Vuer CAMERA_MOVE event."""
        with self.head_pose_updated_at_shared.get_lock():
            return self.head_pose_updated_at_shared.value

'''
        text = text.replace(marker, addition + marker, 1)

    if text != original:
        path.write_text(text, encoding="utf-8")
        return True
    return False


def patch_tv_wrapper(path: Path) -> bool:
    if not path.exists():
        return False
    text = path.read_text(encoding="utf-8")
    original = text

    text = text.replace(
        "    head_pose: np.ndarray                  # (4,4) SE(3) pose of head matrix\n"
        "    head_pose_updated_at: float = 0.0      # wall-clock timestamp of latest CAMERA_MOVE event\n"
        "    left_wrist_pose: np.ndarray",
        "    head_pose: np.ndarray                  # (4,4) SE(3) pose of head matrix\n"
        "    left_wrist_pose: np.ndarray",
        1,
    )
    if "head_pose_updated_at: float = 0.0" not in text:
        text = text.replace(
            "    right_wrist_pose: np.ndarray           # (4,4) SE(3) pose of right wrist of arm\n",
            "    right_wrist_pose: np.ndarray           # (4,4) SE(3) pose of right wrist of arm\n"
            "    head_pose_updated_at: float = 0.0      # wall-clock timestamp of latest CAMERA_MOVE event\n",
            1,
        )

    if "head_pose_updated_at=self.tvuer.head_pose_updated_at" not in text:
        text = text.replace(
            "                head_pose=Brobot_world_head,\n",
            "                head_pose=Brobot_world_head,\n"
            "                head_pose_updated_at=self.tvuer.head_pose_updated_at,\n",
        )

    if text != original:
        path.write_text(text, encoding="utf-8")
        return True
    return False


def main() -> int:
    changed = []
    seen = set()
    for root in ROOTS:
        for path, patcher in (
            (root / "teleop/teleop_hand_and_arm.py", patch_file),
            (root / "teleop/televuer/src/televuer/televuer.py", patch_televuer),
            (root / "teleop/televuer/src/televuer/tv_wrapper.py", patch_tv_wrapper),
        ):
            if path in seen:
                continue
            seen.add(path)
            if patcher(path):
                changed.append(str(path))
    if changed:
        print("Patched head tilt loco in:")
        for path in changed:
            print(f"  {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
