import time
import math
import argparse
from multiprocessing import Value, Array, Lock
import threading
import logging_mp
logging_mp.basicConfig(level=logging_mp.INFO)
logger_mp = logging_mp.getLogger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize # dds 
from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController, H2_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK, H2_ArmIK
from teleimager.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from teleop.utils.ipc import IPC_Server
from teleop.utils.motion_switcher import MotionSwitcher, LocoClientWrapper
from sshkeyboard import listen_keyboard, stop_listening

# for simulation
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
def publish_reset_category(category: int, publisher): # Scene Reset signal
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

# state transition
START          = False  # Enable to start robot following VR user motion
STOP           = False  # Enable to begin system exit procedure
READY          = False  # Ready to (1) enter START state, (2) enter RECORD_RUNNING state
RECORD_RUNNING = False  # True if [Recording]
RECORD_TOGGLE  = False  # Toggle recording state
#  -------        ---------                -----------                -----------            ---------
#   state          [Ready]      ==>        [Recording]     ==>         [AutoSave]     -->     [Ready]
#  -------        ---------      |         -----------      |         -----------      |     ---------
#   START           True         |manual      True          |manual      True          |        True
#   READY           True         |set         False         |set         False         |auto    True
#   RECORD_RUNNING  False        |to          True          |to          False         |        False
#                                ∨                          ∨                          ∨
#   RECORD_TOGGLE   False       True          False        True          False                  False
#  -------        ---------                -----------                 -----------            ---------
#  ==> manual: when READY is True, set RECORD_TOGGLE=True to transition.
#  --> auto  : Auto-transition after saving data.

# robot_telemetry_web head tilt loco
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

def on_press(key):
    global STOP, START, RECORD_TOGGLE
    if key == 'r':
        START = True
    elif key == 'q':
        START = False
        STOP = True
    elif key == 's' and START == True:
        RECORD_TOGGLE = True
    else:
        logger_mp.warning(f"[on_press] {key} was pressed, but no action is defined for this key.")

def get_state() -> dict:
    """Return current heartbeat state"""
    global START, STOP, RECORD_RUNNING, READY
    return {
        "START": START,
        "STOP": STOP,
        "READY": READY,
        "RECORD_RUNNING": RECORD_RUNNING,
    }

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # basic control parameters
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'control and record \'s frequency')
    parser.add_argument('--input-mode', type=str, choices=['hand', 'controller'], default='hand', help='Select XR device input tracking source')
    parser.add_argument('--display-mode', type=str, choices=['immersive', 'ego', 'pass-through'], default='immersive', help='Select XR device display mode')
    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1', 'H2'], default='G1_29', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex3', 'inspire_ftp', 'inspire_dfx', 'brainco'], help='Select end effector controller')
    parser.add_argument('--img-server-ip', type=str, default='192.168.123.164', help='IP address of image server, used by teleimager and televuer')
    parser.add_argument('--network-interface', type=str, default=None, help='Network interface for dds communication, e.g., eth0, wlan0. If None, use default interface.')
    # mode flags
    parser.add_argument('--motion', action = 'store_true', help = 'Enable motion control mode')
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action = 'store_true', help = 'Enable isaac simulation mode')
    parser.add_argument('--ipc', action = 'store_true', help = 'Enable IPC server to handle input; otherwise enable sshkeyboard')
    parser.add_argument('--affinity', action = 'store_true', help = 'Enable high priority and set CPU affinity mode')
    # record mode and task info
    parser.add_argument('--record', action = 'store_true', help = 'Enable data recording mode')
    parser.add_argument('--task-dir', type = str, default = './utils/data/', help = 'path to save data')
    parser.add_argument('--task-name', type = str, default = 'pick cube', help = 'task file name for recording')
    parser.add_argument('--task-goal', type = str, default = 'pick up cube.', help = 'task goal for recording at json file')
    parser.add_argument('--task-desc', type = str, default = 'task description', help = 'task description for recording at json file')
    parser.add_argument('--task-steps', type = str, default = 'step1: do this; step2: do that;', help = 'task steps for recording at json file')

    args = parser.parse_args()
    logger_mp.debug(f"args: {args}")

    try:
        # setup dds communication domains id
        if args.sim:
            ChannelFactoryInitialize(1, networkInterface=args.network_interface)
        else:
            ChannelFactoryInitialize(0, networkInterface=args.network_interface)

        # ipc communication mode. client usage: see utils/ipc.py
        if args.ipc:
            ipc_server = IPC_Server(on_press=on_press,get_state=get_state)
            ipc_server.start()
        # sshkeyboard communication mode
        else:
            listen_keyboard_thread = threading.Thread(target=listen_keyboard, 
                                                      kwargs={"on_press": on_press, "until": None, "sequential": False,}, 
                                                      daemon=True)
            listen_keyboard_thread.start()

        # image client
        img_client = ImageClient(host=args.img_server_ip, request_bgr=True)
        camera_config = img_client.get_cam_config()
        logger_mp.debug(f"Camera config: {camera_config}")
        xr_need_local_img = not (args.display_mode == 'pass-through' or camera_config['head_camera']['enable_webrtc'])

        # televuer_wrapper: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
        tv_wrapper = TeleVuerWrapper(use_hand_tracking=args.input_mode == "hand", 
                                     binocular=camera_config['head_camera']['binocular'],
                                     img_shape=camera_config['head_camera']['image_shape'],
                                     # maybe should decrease fps for better performance?
                                     # https://github.com/unitreerobotics/xr_teleoperate/issues/172
                                     # display_fps=camera_config['head_camera']['fps'] ? args.frequency? 30.0?
                                     display_mode=args.display_mode,
                                     zmq=camera_config['head_camera']['enable_zmq'],
                                     webrtc=camera_config['head_camera']['enable_webrtc'],
                                     webrtc_url=f"https://{args.img_server_ip}:{camera_config['head_camera']['webrtc_port']}/offer",
                                     )
        
        head_tilt_loco = HeadTiltLocoController()
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

        # arm
        if args.arm == "G1_29":
            arm_ik = G1_29_ArmIK()
            arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "G1_23":
            arm_ik = G1_23_ArmIK()
            arm_ctrl = G1_23_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1_2":
            arm_ik = H1_2_ArmIK()
            arm_ctrl = H1_2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1":
            arm_ik = H1_ArmIK()
            arm_ctrl = H1_ArmController(simulation_mode=args.sim)
        elif args.arm == "H2":
            arm_ik = H2_ArmIK()
            arm_ctrl = H2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)

        # end-effector
        if args.ee == "dex3":
            from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
            dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
            hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                          dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "dex1":
            from teleop.robot_control.robot_hand_unitree import Dex1_1_Gripper_Controller
            left_gripper_value = Value('d', 0.0, lock=True)        # [input]
            right_gripper_value = Value('d', 0.0, lock=True)       # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
            dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
            gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, 
                                                     dual_gripper_state_array, dual_gripper_action_array, simulation_mode=args.sim)
        elif args.ee == "inspire_dfx":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_DFX
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_DFX(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "inspire_ftp":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_FTP
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_FTP(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "brainco":
            from teleop.robot_control.robot_hand_brainco import Brainco_Controller
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, 
                                           dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        else:
            pass
        
        # affinity mode (if you dont know what it is, then you probably don't need it)
        if args.affinity:
            import psutil
            p = psutil.Process(os.getpid())
            p.cpu_affinity([0,1,2,3]) # Set CPU affinity to cores 0-3
            try:
                p.nice(-20)           # Set highest priority
                logger_mp.info("Set high priority successfully.")
            except psutil.AccessDenied:
                logger_mp.warning("Failed to set high priority. Please run as root.")
                
            for child in p.children(recursive=True):
                try:
                    logger_mp.info(f"Child process {child.pid} name: {child.name()}")
                    child.cpu_affinity([5,6])
                    child.nice(-20)
                except psutil.AccessDenied:
                    pass

        # simulation mode
        if args.sim:
            reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
            reset_pose_publisher.Init()
            from teleop.utils.sim_state_topic import start_sim_state_subscribe
            sim_state_subscriber = start_sim_state_subscribe()

        # record + headless / non-headless mode
        if args.record:
            recorder = EpisodeWriter(task_dir = os.path.join(args.task_dir, args.task_name),
                                     task_goal = args.task_goal,
                                     task_desc = args.task_desc,
                                     task_steps = args.task_steps,
                                     frequency = args.frequency, 
                                     rerun_log = not args.headless)

        logger_mp.info("----------------------------------------------------------------")
        logger_mp.info("🟢  Press [r] to start syncing the robot with your movements.")
        if args.record:
            logger_mp.info("🟡  Press [s] to START or SAVE recording (toggle cycle).")
        else:
            logger_mp.info("🔵  Recording is DISABLED (run with --record to enable).")
        logger_mp.info("🔴  Press [q] to stop and exit the program.")
        logger_mp.info("⚠️  IMPORTANT: Please keep your distance and stay safe.")
        READY = True                  # now ready to (1) enter START state
        while not START and not STOP: # wait for start or stop signal.
            time.sleep(0.033)
            if camera_config['head_camera']['enable_zmq'] and xr_need_local_img:
                head_img = img_client.get_head_frame()
                if head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)

        logger_mp.info("---------------------🚀start Tracking🚀-------------------------")
        arm_ctrl.speed_gradual_max()

        head_img = None
        left_wrist_img = None
        right_wrist_img = None

        # main loop. robot start to follow VR user's motion
        while not STOP:
            start_time = time.time()
            # get image
            if camera_config['head_camera']['enable_zmq']:
                if args.record or xr_need_local_img:
                    head_img = img_client.get_head_frame()
                if xr_need_local_img and head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img.bgr)
            if camera_config['left_wrist_camera']['enable_zmq']:
                if args.record:
                    left_wrist_img = img_client.get_left_wrist_frame()
            if camera_config['right_wrist_camera']['enable_zmq']:
                if args.record:
                    right_wrist_img = img_client.get_right_wrist_frame()

            # record mode
            if args.record and RECORD_TOGGLE:
                RECORD_TOGGLE = False
                if not RECORD_RUNNING:
                    if recorder.create_episode():
                        RECORD_RUNNING = True
                    else:
                        logger_mp.error("Failed to create episode. Recording not started.")
                else:
                    RECORD_RUNNING = False
                    recorder.save_episode()
                    if args.sim:
                        publish_reset_category(1, reset_pose_publisher)

            # get xr's tele data
            tele_data = tv_wrapper.get_tele_data()
            if (args.ee == "dex3" or args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco") and args.input_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
            elif args.ee == "dex1" and args.input_mode == "controller":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_ctrl_triggerValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_ctrl_triggerValue
            elif args.ee == "dex1" and args.input_mode == "hand":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_hand_pinchValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_hand_pinchValue
            else:
                pass
            
            # high level control
            if head_tilt_loco.enabled and args.motion:
                head_tilt_loco.update(tele_data, loco_wrapper)
            elif position_match_loco.enabled and args.motion:
                position_match_loco.update(tele_data, loco_wrapper)
            elif args.input_mode == "controller" and args.motion:
                # quit teleoperate
                if tele_data.right_ctrl_aButton:
                    START = False
                    STOP = True
                # command robot to enter damping mode. soft emergency stop function
                if tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
                    loco_wrapper.Damp()
                # https://github.com/unitreerobotics/xr_teleoperate/issues/135, control, limit velocity to within 0.3
                loco_wrapper.Move(-tele_data.left_ctrl_thumbstickValue[1] * 0.3,
                                  -tele_data.left_ctrl_thumbstickValue[0] * 0.3,
                                  -tele_data.right_ctrl_thumbstickValue[0]* 0.3)

            # get current robot state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            # solve ik using motor data and wrist pose, then use ik results to control arms.
            time_ik_start = time.time()
            sol_q, sol_tauff  = arm_ik.solve_ik(tele_data.left_wrist_pose, tele_data.right_wrist_pose, current_lr_arm_q, current_lr_arm_dq)
            time_ik_end = time.time()
            logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

            # record data
            if args.record:
                READY = recorder.is_ready() # now ready to (2) enter RECORD_RUNNING state
                # dex hand or gripper
                if args.ee == "dex3" and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:7]
                        right_ee_state = dual_hand_state_array[-7:]
                        left_hand_action = dual_hand_action_array[:7]
                        right_hand_action = dual_hand_action_array[-7:]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "hand":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "controller":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = arm_ctrl.get_current_motor_q().tolist()
                        current_body_action = [-tele_data.left_ctrl_thumbstickValue[1]  * 0.3,
                                               -tele_data.left_ctrl_thumbstickValue[0]  * 0.3,
                                               -tele_data.right_ctrl_thumbstickValue[0] * 0.3]
                elif (args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco") and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = []
                        current_body_action = []
                else:
                    left_ee_state = []
                    right_ee_state = []
                    left_hand_action = []
                    right_hand_action = []
                    current_body_state = []
                    current_body_action = []

                # arm state and action
                left_arm_state  = current_lr_arm_q[:7]
                right_arm_state = current_lr_arm_q[-7:]
                left_arm_action = sol_q[:7]
                right_arm_action = sol_q[-7:]
                if RECORD_RUNNING:
                    colors = {}
                    depths = {}
                    if camera_config['head_camera']['binocular']:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img.bgr[:, :camera_config['head_camera']['image_shape'][1]//2]
                            colors[f"color_{1}"] = head_img.bgr[:, camera_config['head_camera']['image_shape'][1]//2:]
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if left_wrist_img is not None:
                                colors[f"color_{2}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if right_wrist_img is not None:
                                colors[f"color_{3}"] = right_wrist_img.bgr
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    else:
                        if head_img is not None:
                            colors[f"color_{0}"] = head_img.bgr
                        else:
                            logger_mp.warning("Head image is None!")
                        if camera_config['left_wrist_camera']['enable_zmq']:
                            if left_wrist_img is not None:
                                colors[f"color_{1}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config['right_wrist_camera']['enable_zmq']:
                            if right_wrist_img is not None:
                                colors[f"color_{2}"] = right_wrist_img.bgr
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    states = {
                        "left_arm": {                                                                    
                            "qpos":   left_arm_state.tolist(),    # numpy.array -> list
                            "qvel":   [],                          
                            "torque": [],                        
                        }, 
                        "right_arm": {                                                                    
                            "qpos":   right_arm_state.tolist(),       
                            "qvel":   [],                          
                            "torque": [],                         
                        },                        
                        "left_ee": {                                                                    
                            "qpos":   left_ee_state,           
                            "qvel":   [],                           
                            "torque": [],                          
                        }, 
                        "right_ee": {                                                                    
                            "qpos":   right_ee_state,       
                            "qvel":   [],                           
                            "torque": [],  
                        }, 
                        "body": {
                            "qpos": current_body_state,
                        }, 
                    }
                    actions = {
                        "left_arm": {                                   
                            "qpos":   left_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],      
                        }, 
                        "right_arm": {                                   
                            "qpos":   right_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],       
                        },                         
                        "left_ee": {                                   
                            "qpos":   left_hand_action,       
                            "qvel":   [],       
                            "torque": [],       
                        }, 
                        "right_ee": {                                   
                            "qpos":   right_hand_action,       
                            "qvel":   [],       
                            "torque": [], 
                        }, 
                        "body": {
                            "qpos": current_body_action,
                        }, 
                    }
                    if args.sim:
                        sim_state = sim_state_subscriber.read_data()            
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions, sim_state=sim_state)
                    else:
                        recorder.add_item(colors=colors, depths=depths, states=states, actions=actions)

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("⛔ KeyboardInterrupt, exiting program...")
    except Exception:
        import traceback
        logger_mp.error(traceback.format_exc())
    finally:
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
        except Exception as e:
            logger_mp.error(f"Failed to ctrl_dual_arm_go_home: {e}")
        
        try:
            if args.ipc:
                ipc_server.stop()
            else:
                stop_listening()
                listen_keyboard_thread.join()
        except Exception as e:
            logger_mp.error(f"Failed to stop keyboard listener or ipc server: {e}")
        
        try:
            if img_client is not None:
                img_client.close()
        except Exception as e:
            logger_mp.error(f"Failed to close image client: {e}")

        try:
            tv_wrapper.close()
        except Exception as e:
            logger_mp.error(f"Failed to close televuer wrapper: {e}")

        try:
            if not args.motion:
                pass
                # status, result = motion_switcher.Exit_Debug_Mode()
                # logger_mp.info(f"Exit debug mode: {'Success' if status == 3104 else 'Failed'}")
        except Exception as e:
            logger_mp.error(f"Failed to exit debug mode: {e}")

        try:
            if args.sim:
                sim_state_subscriber.stop_subscribe()
        except Exception as e:
            logger_mp.error(f"Failed to stop sim state subscriber: {e}")
        
        try:
            if args.record:
                recorder.close()
        except Exception as e:
            logger_mp.error(f"Failed to close recorder: {e}")
        logger_mp.info("✅ Finally, exiting program.")
        exit(0)
