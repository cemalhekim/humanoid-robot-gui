#!/usr/bin/env python3
"""Unitree H1-2 telemetry web dashboard.

Runs on the robot PC. Subscribes to rt/lowstate continuously and serves a
dependency-free web UI with JSON and Server-Sent Events endpoints.
"""

from __future__ import annotations

import argparse
import json
import math
import mimetypes
import os
import queue
import socket
import struct
import subprocess
import sys
import threading
import time
import contextlib
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit
from typing import Any

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None

try:
    import numpy as np  # type: ignore
except Exception:  # pragma: no cover
    np = None


APP_DIR = Path(__file__).resolve().parent
STATIC_DIR = APP_DIR / "static"
CAMERA_JPEG_PATH = Path("/tmp/robot_telemetry_front_camera.jpg")
UNITREE_ROS2_INSTALL = (
    Path.home()
    / "Workspace/semantic-teleoperation/execution/external/unitree_ros2/cyclonedds_ws/install"
)
UNITREE_GO_PYTHON = UNITREE_ROS2_INSTALL / "unitree_go/local/lib/python3.10/dist-packages"
if UNITREE_GO_PYTHON.exists():
    sys.path.insert(0, str(UNITREE_GO_PYTHON))

SDK_PATHS = [
    Path.home() / "Workspace/semantic-teleoperation/execution/external/unitree_sdk2_python",
    Path.home() / "unitree_sdk2_python",
]
for sdk_path in reversed(SDK_PATHS):
    if sdk_path.exists():
        sys.path.insert(0, str(sdk_path))

JOINT_NAMES = {
    0: "LeftHipYaw",
    1: "LeftHipPitch",
    2: "LeftHipRoll",
    3: "LeftKnee",
    4: "LeftAnklePitch",
    5: "LeftAnkleRoll",
    6: "RightHipYaw",
    7: "RightHipPitch",
    8: "RightHipRoll",
    9: "RightKnee",
    10: "RightAnklePitch",
    11: "RightAnkleRoll",
    12: "WaistYaw",
    13: "LeftShoulderPitch",
    14: "LeftShoulderRoll",
    15: "LeftShoulderYaw",
    16: "LeftElbow",
    17: "LeftWristRoll",
    18: "LeftWristPitch",
    19: "LeftWristYaw",
    20: "RightShoulderPitch",
    21: "RightShoulderRoll",
    22: "RightShoulderYaw",
    23: "RightElbow",
    24: "RightWristRoll",
    25: "RightWristPitch",
    26: "RightWristYaw",
}

JOINT_GROUPS = {
    "left_leg": list(range(0, 6)),
    "right_leg": list(range(6, 12)),
    "waist": [12],
    "left_arm": list(range(13, 20)),
    "right_arm": list(range(20, 27)),
    "reserved": list(range(27, 35)),
}

RIGHT_WRIST_YAW = 26
ARM_SDK_WEIGHT_SLOT = 27
ARM_SDK_JOINTS = [13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 12]
ARM_SDK_KP = [120, 120, 80, 50, 50, 50, 50, 120, 120, 80, 50, 50, 50, 50, 200]
ARM_SDK_KD = [2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0, 2.0]
WRIST_LIMITS = (-1.2, 1.2)

HAND_JOINT_NAMES = {
    0: "RightPinky",
    1: "RightRing",
    2: "RightMiddle",
    3: "RightIndex",
    4: "RightThumbBend",
    5: "RightThumbRotation",
    6: "LeftPinky",
    7: "LeftRing",
    8: "LeftMiddle",
    9: "LeftIndex",
    10: "LeftThumbBend",
    11: "LeftThumbRotation",
}


def public_host() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def default_interface() -> str | None:
    try:
        for line in Path("/proc/net/route").read_text().splitlines()[1:]:
            fields = line.split()
            if len(fields) >= 2 and fields[1] == "00000000":
                return fields[0]
    except OSError:
        return None
    return None


def network_type(interface: str | None) -> str:
    if not interface:
        return "Network"
    lowered = interface.lower()
    if lowered.startswith(("eth", "en")):
        return "Ethernet"
    if lowered.startswith(("wl", "wifi")):
        return "Wi-Fi"
    if lowered.startswith(("ww", "wwan", "cell", "usb")):
        return "Cellular"
    if lowered.startswith(("tun", "tap", "wg")):
        return "VPN"
    return interface


def route_interface(destination: str) -> str | None:
    try:
        output = subprocess.check_output(
            ["ip", "route", "get", destination],
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=1,
        )
    except Exception:
        return None

    parts = output.split()
    if "dev" in parts:
        index = parts.index("dev")
        if index + 1 < len(parts):
            return parts[index + 1]
    return None


def interface_status(interface: str | None, host: str | None = None) -> dict[str, Any]:
    return {
        "type": network_type(interface),
        "interface": interface or "unknown",
        "host": host or public_host(),
        "quality": "Connected" if interface else "Disconnected",
    }


def network_status(robot_host: str) -> dict[str, Any]:
    return {
        "host": interface_status(default_interface()),
        "robot": {
            **interface_status(route_interface(robot_host), robot_host),
            "target": robot_host,
        },
    }


def finite_number(value: Any) -> Any:
    if isinstance(value, float):
        if value != value or value in (float("inf"), float("-inf")):
            return None
        return round(value, 6)
    return value


def listify(value: Any) -> list[Any]:
    try:
        return [finite_number(item) for item in value]
    except TypeError:
        return []


def fields_from(obj: Any, names: list[str]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for name in names:
        if hasattr(obj, name):
            value = getattr(obj, name)
            if isinstance(value, (str, int, bool)) or value is None:
                out[name] = value
            elif isinstance(value, float):
                out[name] = finite_number(value)
            elif isinstance(value, (list, tuple)):
                out[name] = listify(value)
            else:
                try:
                    out[name] = listify(value)
                except Exception:
                    out[name] = str(value)
    return out


def motor_to_dict(index: int, motor: Any) -> dict[str, Any]:
    data = fields_from(
        motor,
        [
            "mode",
            "q",
            "dq",
            "ddq",
            "tau_est",
            "temperature",
            "vol",
            "sensor",
            "reserve",
        ],
    )
    data["index"] = index
    data["name"] = JOINT_NAMES.get(index, f"ReservedMotorSlot{index}")
    return data


def hand_motor_to_dict(index: int, motor: Any) -> dict[str, Any]:
    data = fields_from(
        motor,
        [
            "mode",
            "q",
            "dq",
            "ddq",
            "tau",
            "tau_est",
            "temperature",
            "vol",
            "sensor",
            "reserve",
        ],
    )
    data["index"] = index
    data["name"] = HAND_JOINT_NAMES.get(index, f"HandMotor{index}")
    return data


def handstate_to_dict(msg: Any | None, samples: int, timestamp: float | None) -> dict[str, Any]:
    if msg is None:
        return {
            "connected": False,
            "topic": "rt/inspire/state",
            "samples": samples,
            "timestamp": timestamp,
            "joints": [],
            "note": "No hand state received. Start inspire_h1 service if the RH56BFX hands are connected over serial.",
        }

    states = getattr(msg, "states", [])
    return {
        "connected": True,
        "topic": "rt/inspire/state",
        "samples": samples,
        "timestamp": timestamp,
        "joint_count": len(states),
        "joints": [hand_motor_to_dict(i, motor) for i, motor in enumerate(states)],
    }


def numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        if math.isfinite(float(value)):
            return float(value)
    return None


def motor_temperature(motor: dict[str, Any]) -> float | None:
    values = [numeric(value) for value in motor.get("temperature", [])]
    values = [value for value in values if value is not None and value > 0]
    return max(values) if values else None


def is_reserved_motor(motor: dict[str, Any]) -> bool:
    return str(motor.get("name", "")).startswith("ReservedMotorSlot")


def compact_motor(motor: dict[str, Any], value: float | None = None) -> dict[str, Any] | None:
    if not motor:
        return None
    data: dict[str, Any] = {
        "index": motor.get("index"),
        "name": motor.get("name"),
    }
    if value is not None:
        data["value"] = round(value, 6)
    return data


def summarize_motor_groups(motors: list[dict[str, Any]]) -> dict[str, Any]:
    by_index = {motor.get("index"): motor for motor in motors}
    groups: dict[str, Any] = {}
    for group, indexes in JOINT_GROUPS.items():
        group_motors = [by_index[index] for index in indexes if index in by_index]
        temps = [motor_temperature(motor) for motor in group_motors]
        temps = [temp for temp in temps if temp is not None]
        groups[group] = {
            "count": len(group_motors),
            "moving": sum(1 for motor in group_motors if abs(numeric(motor.get("dq")) or 0.0) > 0.05),
            "max_temperature": round(max(temps), 1) if temps else None,
        }
    return groups


def summarize_motors(motors: list[dict[str, Any]]) -> dict[str, Any]:
    real_motors = [motor for motor in motors if not is_reserved_motor(motor)]
    reserved_motors = [motor for motor in motors if is_reserved_motor(motor)]
    mode_counts: dict[str, int] = {}
    for motor in real_motors:
        key = str(motor.get("mode", "unknown"))
        mode_counts[key] = mode_counts.get(key, 0) + 1

    hottest = max(real_motors, key=lambda motor: motor_temperature(motor) or -1.0, default={})
    max_tau = max(real_motors, key=lambda motor: abs(numeric(motor.get("tau_est")) or 0.0), default={})
    max_velocity = max(real_motors, key=lambda motor: abs(numeric(motor.get("dq")) or 0.0), default={})

    return {
        "real_count": len(real_motors),
        "reserved_count": len(reserved_motors),
        "mode_counts": mode_counts,
        "moving_count": sum(
            1 for motor in real_motors if abs(numeric(motor.get("dq")) or 0.0) > 0.05
        ),
        "hottest": compact_motor(hottest, motor_temperature(hottest)) if hottest else None,
        "max_abs_tau": compact_motor(max_tau, abs(numeric(max_tau.get("tau_est")) or 0.0))
        if max_tau
        else None,
        "max_abs_velocity": compact_motor(max_velocity, abs(numeric(max_velocity.get("dq")) or 0.0))
        if max_velocity
        else None,
        "groups": summarize_motor_groups(motors),
    }


def summarize_imu(imu: dict[str, Any]) -> dict[str, Any]:
    rpy = imu.get("rpy") or []
    roll, pitch, yaw = (list(rpy) + [None, None, None])[:3]
    return {
        "roll_deg": round(math.degrees(roll), 2) if numeric(roll) is not None else None,
        "pitch_deg": round(math.degrees(pitch), 2) if numeric(pitch) is not None else None,
        "yaw_deg": round(math.degrees(yaw), 2) if numeric(yaw) is not None else None,
        "temperature": imu.get("temperature"),
    }


def health_flags(snapshot: dict[str, Any], motor_summary: dict[str, Any]) -> list[dict[str, str]]:
    flags: list[dict[str, str]] = []
    if not snapshot.get("connected"):
        flags.append({"level": "critical", "message": "No LowState telemetry is being received."})

    mode_counts = motor_summary.get("mode_counts", {})
    real_count = motor_summary.get("real_count", 0)
    if real_count and mode_counts == {"0": real_count}:
        flags.append({"level": "info", "message": "All real motors report mode 0, so the robot is passive/idle."})

    hottest = motor_summary.get("hottest") or {}
    hottest_value = numeric(hottest.get("value"))
    if hottest_value is not None and hottest_value >= 70:
        flags.append(
            {
                "level": "warning",
                "message": f"Hottest motor is {hottest.get('name')} at {round(hottest_value, 1)} C.",
            }
        )

    imu_temp = numeric((snapshot.get("imu") or {}).get("temperature"))
    if imu_temp is not None and imu_temp >= 75:
        flags.append({"level": "warning", "message": f"IMU temperature is {round(imu_temp, 1)} C."})

    if not (snapshot.get("hands") or {}).get("connected"):
        flags.append({"level": "info", "message": "Hand telemetry is offline on rt/inspire/state."})

    if (snapshot.get("battery") or {}).get("state"):
        flags.append({"level": "info", "message": "Battery details are not exposed by this LowState firmware."})

    return flags


def analyze_snapshot(snapshot: dict[str, Any]) -> dict[str, Any]:
    motor_summary = summarize_motors(snapshot.get("motors", []))
    imu_summary = summarize_imu(snapshot.get("imu", {}))
    flags = health_flags(snapshot, motor_summary)
    return {
        "motors": motor_summary,
        "imu": imu_summary,
        "health": {
            "state": "warning" if any(flag["level"] == "warning" for flag in flags) else "ok",
            "flags": flags,
        },
    }


def lowstate_to_dict(
    msg: Any, samples: int, rate_hz: float, hands: dict[str, Any] | None = None
) -> dict[str, Any]:
    motors = [motor_to_dict(i, motor) for i, motor in enumerate(msg.motor_state)]
    imu = fields_from(
        msg.imu_state,
        ["quaternion", "gyroscope", "accelerometer", "rpy", "temperature"],
    )

    data = {
        "connected": True,
        "timestamp": time.time(),
        "samples": samples,
        "sample_rate_hz": round(rate_hz, 2),
        "motor_count": len(motors),
        "motors": motors,
        "imu": imu,
        "robot": fields_from(
            msg,
            [
                "version",
                "mode_pr",
                "mode_machine",
                "tick",
                "crc",
                "wireless_remote",
            ],
        ),
        "hands": hands or handstate_to_dict(None, 0, None),
    }

    if hasattr(msg, "bms_state"):
        data["battery"] = fields_from(
            msg.bms_state,
            ["version_h", "version_l", "bms_status", "soc", "current", "cycle", "temperature"],
        )
    else:
        data["battery"] = {
            "state": "not exposed by this LowState firmware",
            "checked_fields": ["bms_state", "battery_state", "power_v", "power_a"],
        }

    if hasattr(msg, "foot_force"):
        data["foot_force"] = listify(msg.foot_force)
    if hasattr(msg, "foot_force_est"):
        data["foot_force_est"] = listify(msg.foot_force_est)

    data["analysis"] = analyze_snapshot(data)
    return data


def h264_payload_from_raw_ros(data: bytes, target_resolution: int = 360) -> bytes | None:
    offset = 4
    if len(data) < offset + 8:
        return None
    offset += 8
    payloads: dict[int, bytes] = {}
    for resolution in (720, 360, 180):
        if offset + 4 > len(data):
            break
        payload_size = struct.unpack_from("<I", data, offset)[0]
        offset += 4
        if payload_size > len(data) - offset:
            break
        payload = data[offset : offset + payload_size]
        offset += payload_size
        while offset % 4:
            offset += 1
        if not payload:
            continue
        payloads[resolution] = payload
    return payloads.get(target_resolution) or payloads.get(720) or payloads.get(360) or payloads.get(180)


def clean_h264_payload(payload: bytes) -> bytes | None:
    for marker in (b"\x00\x00\x00\x01", b"\x00\x00\x01"):
        index = payload.find(marker)
        if index >= 0:
            return payload[index:]
    return payload or None


def h264_payload_from_video_msg(msg: Any, target_resolution: int = 360) -> bytes | None:
    fields = {
        720: bytes(getattr(msg, "video720p", b"")),
        360: bytes(getattr(msg, "video360p", b"")),
        180: bytes(getattr(msg, "video180p", b"")),
    }
    payload = fields.get(target_resolution) or fields[720] or fields[360] or fields[180]
    return clean_h264_payload(payload) if payload else None


def configure_ros2_camera_environment(interface: str) -> None:
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    if interface and "CYCLONEDDS_URI" not in os.environ:
        os.environ["CYCLONEDDS_URI"] = (
            "<CycloneDDS><Domain><General><Interfaces>"
            f'<NetworkInterface name="{interface}" priority="default" multicast="default" />'
            "</Interfaces></General></Domain></CycloneDDS>"
        )


def run_ros2_command(args: list[str], timeout: float = 2.5) -> tuple[bool, str]:
    try:
        result = subprocess.run(
            ["ros2", *args],
            capture_output=True,
            check=False,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    output = result.stdout.strip() or result.stderr.strip()
    return result.returncode == 0, output


def parse_topic_list(output: str) -> dict[str, list[str]]:
    topics: dict[str, list[str]] = {}
    for line in output.splitlines():
        line = line.strip()
        if not line or " [" not in line or not line.endswith("]"):
            continue
        name, raw_types = line.rsplit(" [", 1)
        topics[name] = [item.strip() for item in raw_types[:-1].split(",") if item.strip()]
    return topics


def parse_node_info(name: str, output: str) -> dict[str, Any]:
    sections = {
        "Subscribers:": "subscribers",
        "Publishers:": "publishers",
        "Service Servers:": "service_servers",
        "Service Clients:": "service_clients",
        "Action Servers:": "action_servers",
        "Action Clients:": "action_clients",
    }
    node = {key: [] for key in sections.values()}
    node["name"] = name
    current: str | None = None
    for raw_line in output.splitlines():
        stripped = raw_line.strip()
        if stripped in sections:
            current = sections[stripped]
            continue
        if current is None or not raw_line.startswith("    ") or ":" not in stripped:
            continue
        topic, msg_type = stripped.split(":", 1)
        node[current].append({"name": topic.strip(), "type": msg_type.strip()})
    return node


def collect_ros_graph(interface: str) -> dict[str, Any]:
    configure_ros2_camera_environment(interface)
    timestamp = time.time()
    ok_nodes, node_output = run_ros2_command(["node", "list"])
    ok_topics, topic_output = run_ros2_command(["topic", "list", "-t"])
    if not ok_nodes:
        return {"timestamp": timestamp, "nodes": [], "topics": {}, "subscriptions": [], "error": node_output}

    node_names = [line.strip() for line in node_output.splitlines() if line.strip()]
    topic_types = parse_topic_list(topic_output if ok_topics else "")
    nodes = []
    subscriptions = []
    publishers = []
    for node_name in node_names[:40]:
        ok_info, info_output = run_ros2_command(["node", "info", node_name], timeout=2.0)
        if not ok_info:
            nodes.append({"name": node_name, "subscribers": [], "publishers": [], "error": info_output})
            continue
        node = parse_node_info(node_name, info_output)
        nodes.append(node)
        for sub in node["subscribers"]:
            subscriptions.append({"node": node_name, "topic": sub["name"], "type": sub["type"]})
        for pub in node["publishers"]:
            publishers.append({"node": node_name, "topic": pub["name"], "type": pub["type"]})

    return {
        "timestamp": timestamp,
        "interface": interface or "default",
        "nodes": nodes,
        "topics": topic_types,
        "subscriptions": subscriptions,
        "publishers": publishers,
        "error": None if ok_topics else topic_output,
    }


def camera_decoder_worker(store: "TelemetryStore", fifo_path: str) -> None:
    if cv2 is None:
        store.set_camera_error("OpenCV is not available for H264 decode.")
        return
    os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "quiet")
    try:
        cv2.setLogLevel(0)
    except AttributeError:
        pass
    try:
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, 2)
        os.close(devnull)
    except OSError:
        pass
    cap = cv2.VideoCapture(fifo_path, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        store.set_camera_error("OpenCV could not open H264 camera stream.")
        return
    while store.running:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.02)
            continue
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        if ok:
            store.set_camera_frame(encoded.tobytes())
    cap.release()


def camera_fifo_writer(store: "TelemetryStore", fifo_path: str, payloads: "queue.Queue[bytes]") -> None:
    while store.running:
        try:
            with open(fifo_path, "wb", buffering=0) as stream:
                while store.running:
                    stream.write(payloads.get())
        except BrokenPipeError:
            time.sleep(0.1)
        except OSError as exc:
            store.set_camera_error(f"H264 pipe writer failed: {exc}")
            time.sleep(0.5)


def decode_h264_file(path: str) -> bytes | None:
    if cv2 is None:
        return None
    cap = cv2.VideoCapture(path, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        return None
    latest = None
    try:
        for _ in range(120):
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            latest = frame
    finally:
        cap.release()
    if latest is None:
        return None
    ok, encoded = cv2.imencode(".jpg", latest, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    return encoded.tobytes() if ok else None


def camera_buffer_decoder_worker(store: "TelemetryStore", payloads: "queue.Queue[bytes]") -> None:
    if cv2 is None:
        store.set_camera_error("OpenCV is not available for H264 decode.")
        return
    os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "quiet")
    buffer = bytearray()
    max_bytes = 4_000_000
    decode_path = f"/tmp/robot_telemetry_front_camera_{os.getpid()}.h264"
    next_decode = 0.0
    last_frame_at = 0.0
    while store.running:
        try:
            payload = payloads.get(timeout=0.2)
        except queue.Empty:
            continue
        buffer.extend(payload)
        if len(buffer) > max_bytes:
            del buffer[: len(buffer) - max_bytes]
            first_start = buffer.find(b"\x00\x00\x00\x01")
            if first_start > 0:
                del buffer[:first_start]
        now = time.time()
        if now < next_decode or len(buffer) < 32_000:
            continue
        next_decode = now + 0.25
        try:
            Path(decode_path).write_bytes(buffer)
            frame = decode_h264_file(decode_path)
        except Exception as exc:
            store.set_camera_error(f"H264 buffer decode failed: {exc}")
            continue
        if frame is not None:
            store.set_camera_frame(frame)
            last_frame_at = now
        elif last_frame_at == 0.0 or now - last_frame_at > 3.0:
            store.set_camera_error("Waiting for H264 keyframe from front video stream.")


def camera_bridge_main(camera_source: str, resolution: int, output_path: Path) -> None:
    configure_ros2_camera_environment(camera_source)
    try:
        import rclpy
        from rclpy.node import Node
        from unitree_go.msg import Go2FrontVideoData
    except Exception as exc:
        print(f"Could not import ROS2 camera dependencies: {exc}", file=sys.stderr)
        return

    payloads: queue.Queue[bytes] = queue.Queue(maxsize=240)

    def decoder() -> None:
        buffer = bytearray()
        max_bytes = 4_000_000
        decode_path = Path(f"/tmp/robot_telemetry_front_camera_bridge_{os.getpid()}.h264")
        next_decode = 0.0
        while True:
            payload = payloads.get()
            buffer.extend(payload)
            if len(buffer) > max_bytes:
                del buffer[: len(buffer) - max_bytes]
                first_start = buffer.find(b"\x00\x00\x00\x01")
                if first_start > 0:
                    del buffer[:first_start]
            now = time.time()
            if now < next_decode or len(buffer) < 8_000:
                continue
            next_decode = now + 0.2
            try:
                decode_path.write_bytes(buffer)
                frame = decode_h264_file(str(decode_path))
                if frame is None:
                    continue
                tmp_path = output_path.with_suffix(".jpg.tmp")
                tmp_path.write_bytes(frame)
                os.replace(tmp_path, output_path)
            except Exception as exc:
                print(f"Camera bridge decode failed: {exc}", file=sys.stderr)

    threading.Thread(target=decoder, daemon=True).start()

    rclpy.init(args=None)

    class FrontVideoNode(Node):
        def __init__(self) -> None:
            super().__init__("robot_telemetry_front_video_bridge")
            self.create_subscription(Go2FrontVideoData, "/frontvideostream", self.on_frame, 10)

        def on_frame(self, msg: Any) -> None:
            payload = h264_payload_from_video_msg(msg, resolution)
            if not payload:
                return
            try:
                payloads.put_nowait(payload)
            except queue.Full:
                with contextlib.suppress(queue.Empty):
                    payloads.get_nowait()
                payloads.put_nowait(payload)

    node = FrontVideoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def camera_file_watcher(store: "TelemetryStore", image_path: Path) -> None:
    last_mtime = 0.0
    while store.running:
        try:
            stat = image_path.stat()
            if stat.st_mtime != last_mtime and stat.st_size > 0:
                frame = image_path.read_bytes()
                if frame.startswith(b"\xff\xd8"):
                    last_mtime = stat.st_mtime
                    store.set_camera_frame(frame)
            elif store.camera_frame is None:
                store.set_camera_error("Waiting for camera bridge frame.")
        except FileNotFoundError:
            if store.camera_frame is None:
                store.set_camera_error("Waiting for camera bridge frame.")
        except OSError as exc:
            store.set_camera_error(f"Camera frame watcher failed: {exc}")
        time.sleep(0.1)


def start_camera_bridge(store: "TelemetryStore") -> None:
    with contextlib.suppress(FileNotFoundError):
        CAMERA_JPEG_PATH.unlink()
    cmd = [
        sys.executable,
        str(Path(__file__).resolve()),
        "--camera-bridge",
        "--camera-source",
        store.camera_source,
        "--camera-resolution",
        str(store.camera_resolution),
        "--camera-output",
        str(CAMERA_JPEG_PATH),
    ]
    store.camera_process = subprocess.Popen(
        cmd,
        cwd=str(APP_DIR),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    threading.Thread(target=camera_file_watcher, args=(store, CAMERA_JPEG_PATH), daemon=True).start()


def ros_camera_worker(store: "TelemetryStore") -> None:
    configure_ros2_camera_environment(store.camera_source)
    try:
        import rclpy
        from rclpy.node import Node
        from unitree_go.msg import Go2FrontVideoData
    except Exception as exc:
        store.set_camera_error(f"Could not import ROS2 camera dependencies: {exc}")
        return

    payloads: queue.Queue[bytes] = queue.Queue(maxsize=240)
    threading.Thread(target=camera_buffer_decoder_worker, args=(store, payloads), daemon=True).start()

    rclpy.init(args=None)

    class FrontVideoNode(Node):
        def __init__(self) -> None:
            super().__init__("robot_telemetry_front_video")
            self.create_subscription(Go2FrontVideoData, "/frontvideostream", self.on_frame, 10)

        def on_frame(self, msg: Any) -> None:
            if isinstance(msg, bytes):
                payload = h264_payload_from_raw_ros(bytes(msg), store.camera_resolution)
                payload = clean_h264_payload(payload) if payload else None
            else:
                payload = h264_payload_from_video_msg(msg, store.camera_resolution)
            if not payload:
                store.set_camera_error("Front video packet did not contain H264 payload.")
                return
            try:
                payloads.put_nowait(payload)
            except queue.Full:
                try:
                    payloads.get_nowait()
                except queue.Empty:
                    pass
                payloads.put_nowait(payload)

    node = FrontVideoNode()
    store.set_camera_error(None)
    try:
        while store.running:
            rclpy.spin_once(node, timeout_sec=0.2)
    except Exception as exc:
        store.set_camera_error(f"ROS2 front video subscriber failed: {exc}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


class TelemetryStore:
    def __init__(self, domain: int, robot_host: str) -> None:
        self.domain = domain
        self.robot_host = robot_host
        self.camera_source = os.environ.get("CAMERA_SOURCE", "")
        self.lock = threading.Lock()
        self.camera_lock = threading.Lock()
        self.camera_condition = threading.Condition(self.camera_lock)
        self.camera_frame: bytes | None = None
        self.camera_timestamp: float | None = None
        self.camera_error: str | None = None
        self.camera_process: subprocess.Popen[bytes] | None = None
        self.camera_topic = "/frontvideostream"
        self.camera_resolution = int(os.environ.get("CAMERA_RESOLUTION", "360"))
        self.latest: dict[str, Any] = {
            "connected": False,
            "network": network_status(self.robot_host),
            "timestamp": None,
            "samples": 0,
            "sample_rate_hz": 0,
            "motor_count": 0,
            "motors": [],
            "imu": {},
            "robot": {},
            "hands": handstate_to_dict(None, 0, None),
            "error": "Subscriber has not started yet.",
        }
        self.running = False
        self.thread: threading.Thread | None = None
        self.sample_times: deque[float] = deque(maxlen=300)
        self.samples = 0
        self.ros_graph_cache: dict[str, Any] | None = None
        self.ros_graph_timestamp = 0.0
        self.lowstate_msg: Any | None = None
        self.command_lock = threading.Lock()
        self.wrist_publisher: Any | None = None
        self.lowcmd_publisher: Any | None = None
        self.motion_switcher: Any | None = None
        self.loco_client: Any | None = None
        self.lowcmd_factory: Any | None = None
        self.lowcmd_type: Any | None = None
        self.crc: Any | None = None
        self.wrist_cancel: threading.Event | None = None
        self.wrist_thread: threading.Thread | None = None
        self.wrist_status: dict[str, Any] = {
            "available": False,
            "active": False,
            "message": "DDS command publisher has not started yet.",
            "last_command": None,
            "updated_at": None,
        }

    def start(self) -> None:
        self.running = True
        self.thread = threading.Thread(target=self._run, name="unitree-lowstate", daemon=True)
        self.thread.start()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {**self.latest, "network": network_status(self.robot_host)}

    def camera_snapshot(self) -> dict[str, Any]:
        with self.camera_lock:
            return {
                "source": self.camera_topic,
                "interface": self.camera_source or "default",
                "resolution": self.camera_resolution,
                "available": self.camera_frame is not None,
                "timestamp": self.camera_timestamp,
                "error": self.camera_error,
            }

    def ros_graph_snapshot(self) -> dict[str, Any]:
        now = time.time()
        if self.ros_graph_cache is not None and now - self.ros_graph_timestamp < 3.0:
            return self.ros_graph_cache
        graph = collect_ros_graph(self.camera_source)
        self.ros_graph_cache = graph
        self.ros_graph_timestamp = now
        return graph

    def wrist_snapshot(self) -> dict[str, Any]:
        snapshot = self.snapshot()
        motors = snapshot.get("motors") or []
        wrist = next((motor for motor in motors if motor.get("index") == RIGHT_WRIST_YAW), None)
        with self.command_lock:
            status = dict(self.wrist_status)
        return {
            **status,
            "joint": {
                "index": RIGHT_WRIST_YAW,
                "name": JOINT_NAMES[RIGHT_WRIST_YAW],
                "limits": {"min": WRIST_LIMITS[0], "max": WRIST_LIMITS[1]},
                "telemetry": wrist,
            },
        }

    def _set_wrist_status(self, **updates: Any) -> None:
        with self.command_lock:
            self.wrist_status = {**self.wrist_status, **updates, "updated_at": time.time()}

    def _build_arm_sdk_cmd(self, msg: Any, target_q: float, kp: float, kd: float, weight: float = 1.0) -> Any:
        if self.lowcmd_factory is None or self.crc is None:
            raise RuntimeError("LowCmd factory is not initialized")
        cmd = self.lowcmd_factory()
        cmd.mode_pr = int(getattr(msg, "mode_pr", 0) or 0)
        cmd.mode_machine = int(getattr(msg, "mode_machine", 0) or 0)
        for i in range(35):
            motor = cmd.motor_cmd[i]
            motor.mode = 0
            motor.q = 0.0
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = 0.0
            motor.kd = 0.0
            motor.reserve = 0

        cmd.motor_cmd[ARM_SDK_WEIGHT_SLOT].q = float(weight)
        if weight > 0:
            for joint, hold_kp, hold_kd in zip(ARM_SDK_JOINTS, ARM_SDK_KP, ARM_SDK_KD):
                motor = cmd.motor_cmd[joint]
                motor.mode = 1
                motor.q = float(msg.motor_state[joint].q)
                motor.dq = 0.0
                motor.tau = 0.0
                motor.kp = hold_kp
                motor.kd = hold_kd

            wrist = cmd.motor_cmd[RIGHT_WRIST_YAW]
            wrist.mode = 1
            wrist.q = target_q
            wrist.dq = 0.0
            wrist.tau = 0.0
            wrist.kp = kp
            wrist.kd = kd

        cmd.crc = self.crc.Crc(cmd)
        return cmd

    def _build_lowcmd_wrist_cmd(self, msg: Any, hold_q: list[float], wrist_q: float, kp: float, kd: float) -> Any:
        if self.lowcmd_factory is None or self.crc is None:
            raise RuntimeError("LowCmd factory is not initialized")
        cmd = self.lowcmd_factory()
        cmd.mode_pr = 0
        cmd.mode_machine = int(getattr(msg, "mode_machine", 0) or 0)
        for i in range(27):
            motor = cmd.motor_cmd[i]
            motor.mode = 1
            motor.q = wrist_q if i == RIGHT_WRIST_YAW else hold_q[i]
            motor.dq = 0.0
            motor.tau = 0.0
            if i == RIGHT_WRIST_YAW:
                motor.kp = kp
                motor.kd = kd
            elif i < 13:
                motor.kp = 70.0
                motor.kd = 1.0
            else:
                motor.kp = 25.0
                motor.kd = 0.8
            motor.reserve = 0
        cmd.crc = self.crc.Crc(cmd)
        return cmd

    @staticmethod
    def _auto_wrist_gains(mode: str, start_q: float, target_q: float, delta: float, period: float) -> tuple[float, float]:
        if mode == "oscillate":
            amplitude = abs(delta)
            max_target_speed = (2.0 * math.pi * amplitude) / max(0.4, period)
            kp = 6.0 + 80.0 * amplitude + 3.0 * max_target_speed
            kd = 0.4 + 0.08 * math.sqrt(kp) + 0.8 * max_target_speed
        else:
            error = abs(delta) if mode == "relative" else abs(target_q - start_q)
            x = max(0.0, min(1.0, error / 0.2))
            kp = 4.0 + (18.0 - 4.0) * x
            kd = 0.28 * 2.0 * math.sqrt(kp)
        return max(4.0, min(22.0, kp)), max(0.35, min(2.0, kd))

    def stop_wrist(self) -> dict[str, Any]:
        with self.command_lock:
            cancel = self.wrist_cancel
            publisher = self.wrist_publisher
            msg = self.lowstate_msg
        if cancel is not None:
            cancel.set()
        if publisher is not None and msg is not None:
            try:
                current_q = float(msg.motor_state[RIGHT_WRIST_YAW].q)
                release = self._build_arm_sdk_cmd(msg, current_q, 0.0, 0.0, weight=0.0)
                for _ in range(10):
                    publisher.Write(release)
                    time.sleep(0.01)
            except Exception as exc:
                self._set_wrist_status(active=False, message=f"Stop publish failed: {exc}")
                return self.wrist_snapshot()
        self._set_wrist_status(active=False, message="Right wrist command stopped.")
        return self.wrist_snapshot()

    def chill_motors(self) -> tuple[int, dict[str, Any]]:
        with self.command_lock:
            cancel = self.wrist_cancel
            motion_switcher = self.motion_switcher
            loco_client = self.loco_client
        if cancel is not None:
            cancel.set()

        wrist_status = self.stop_wrist()
        select_code = None
        stop_code = None
        damp_code = None
        try:
            if motion_switcher is not None:
                select_code, _ = motion_switcher.SelectMode("ai")
                time.sleep(0.15)
            if loco_client is None:
                return 503, {"ok": False, "error": "H1 loco client is not available.", "wrist": wrist_status}
            stop_code = loco_client.SetVelocity(0.0, 0.0, 0.0, 0.2)
            damp_code = loco_client.SetFsmId(1)
        except Exception as exc:
            return 500, {
                "ok": False,
                "error": f"Could not request damp mode: {exc}",
                "select_mode_code": select_code,
                "stop_move_code": stop_code,
                "damp_code": damp_code,
                "wrist": wrist_status,
            }

        ok = damp_code == 0
        message = "Damp mode requested. Motors should stop actively pushing." if ok else f"Damp request returned code {damp_code}."
        self._set_wrist_status(active=False, message=message)
        return (200 if ok else 502), {
            "ok": ok,
            "message": message,
            "select_mode_code": select_code,
            "stop_move_code": stop_code,
            "damp_code": damp_code,
            "wrist": wrist_status,
        }

    def command_wrist(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        if not payload.get("armed") or not payload.get("i_understand_risk"):
            return 400, {"ok": False, "error": "Command requires armed=true and i_understand_risk=true."}

        def number(name: str, default: float, low: float, high: float) -> float:
            try:
                value = float(payload.get(name, default))
            except (TypeError, ValueError):
                raise ValueError(f"{name} must be a number")
            if value < low or value > high:
                raise ValueError(f"{name} must be between {low} and {high}")
            return value

        try:
            mode = str(payload.get("mode", "absolute"))
            control_path = str(payload.get("control_path", "arm_sdk"))
            target = number("target_q", 0.0, WRIST_LIMITS[0], WRIST_LIMITS[1])
            delta = number("delta_q", 0.0, -0.25, 0.25)
            kp = number("kp", 4.0, 0.0, 30.0)
            kd = number("kd", 0.35, 0.0, 5.0)
            duration = number("duration", 0.35, 0.05, 12.0)
            rate = number("rate", 80.0, 20.0, 200.0)
            period = number("period", 2.0, 0.4, 8.0)
        except ValueError as exc:
            return 400, {"ok": False, "error": str(exc)}

        with self.command_lock:
            publisher = self.wrist_publisher
            lowcmd_publisher = self.lowcmd_publisher
            motion_switcher = self.motion_switcher
            msg = self.lowstate_msg
            previous_cancel = self.wrist_cancel
        if self.lowcmd_factory is None or self.crc is None:
            return 503, {"ok": False, "error": "DDS command factory is not available."}
        if control_path == "lowcmd" and lowcmd_publisher is None:
            return 503, {"ok": False, "error": "DDS lowcmd publisher is not available."}
        if control_path != "lowcmd" and publisher is None:
            return 503, {"ok": False, "error": "DDS arm_sdk publisher is not available."}
        if msg is None:
            return 503, {"ok": False, "error": "No rt/lowstate sample is available yet."}
        if previous_cancel is not None:
            previous_cancel.set()

        start_q = float(msg.motor_state[RIGHT_WRIST_YAW].q)
        if mode == "relative":
            target_q = start_q + delta
        else:
            target_q = target
        target_q = max(WRIST_LIMITS[0], min(WRIST_LIMITS[1], target_q))
        if payload.get("auto_gains"):
            kp, kd = self._auto_wrist_gains(mode, start_q, target_q, delta, period)

        cancel = threading.Event()
        command = {
            "mode": mode,
            "start_q": round(start_q, 6),
            "target_q": round(target_q, 6),
            "kp": kp,
            "kd": kd,
            "duration": duration,
            "rate": rate,
            "period": period,
            "control_path": control_path,
        }

        def run_command() -> None:
            self._set_wrist_status(
                available=True,
                active=True,
                message=f"Publishing right wrist {control_path} command.",
                last_command=command,
            )
            publish_period = 1.0 / rate
            writes = 0
            try:
                hold_q = [float(msg.motor_state[i].q) for i in range(27)]
                center_q = hold_q[RIGHT_WRIST_YAW]
                if control_path == "lowcmd" and motion_switcher is not None:
                    code, result = motion_switcher.CheckMode()
                    if code == 0 and result and result.get("name"):
                        motion_switcher.ReleaseMode()
                        time.sleep(0.25)
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline and not cancel.is_set():
                    with self.command_lock:
                        latest_msg = self.lowstate_msg
                        latest_publisher = self.wrist_publisher
                        latest_lowcmd_publisher = self.lowcmd_publisher
                    if latest_msg is None:
                        break
                    if control_path == "lowcmd":
                        if latest_lowcmd_publisher is None:
                            break
                        elapsed = duration - max(0.0, deadline - time.monotonic())
                        wrist_q = (
                            center_q + delta * math.sin((2.0 * math.pi * elapsed) / command["period"])
                            if mode == "oscillate"
                            else target_q
                        )
                        latest_lowcmd_publisher.Write(self._build_lowcmd_wrist_cmd(latest_msg, hold_q, wrist_q, kp, kd))
                    else:
                        if latest_publisher is None:
                            break
                        latest_publisher.Write(self._build_arm_sdk_cmd(latest_msg, target_q, kp, kd, weight=1.0))
                    writes += 1
                    time.sleep(publish_period)
                if control_path == "lowcmd" and motion_switcher is not None:
                    with contextlib.suppress(Exception):
                        motion_switcher.SelectMode("ai")
                message = "Right wrist command cancelled." if cancel.is_set() else f"Right wrist command complete ({writes} writes)."
                self._set_wrist_status(active=False, message=message, last_command={**command, "writes": writes})
            except Exception as exc:
                self._set_wrist_status(active=False, message=f"Right wrist command failed: {exc}", last_command=command)

        thread = threading.Thread(target=run_command, name="right-wrist-command", daemon=True)
        with self.command_lock:
            self.wrist_cancel = cancel
            self.wrist_thread = thread
        thread.start()
        return 202, {"ok": True, "status": self.wrist_snapshot()}

    def set_camera_frame(self, frame: bytes) -> None:
        with self.camera_lock:
            self.camera_frame = frame
            self.camera_timestamp = time.time()
            self.camera_error = None
            self.camera_condition.notify_all()

    def set_camera_error(self, error: str | None) -> None:
        with self.camera_lock:
            self.camera_error = error

    def get_camera_frame(self) -> bytes | None:
        with self.camera_lock:
            return self.camera_frame

    def wait_for_camera_frame(self, last_timestamp: float | None, timeout: float = 1.0) -> tuple[bytes | None, float | None]:
        with self.camera_condition:
            if self.camera_frame is not None and self.camera_timestamp != last_timestamp:
                return self.camera_frame, self.camera_timestamp
            self.camera_condition.wait(timeout)
            return self.camera_frame, self.camera_timestamp

    def _set_error(self, error: str) -> None:
        with self.lock:
            self.latest = {
                **self.latest,
                "connected": False,
                "timestamp": time.time(),
                "error": error,
            }

    def _run(self) -> None:
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
            from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_
            from unitree_sdk2py.utils.crc import CRC
        except Exception as exc:
            self._set_error(f"Could not import Unitree SDK: {exc}")
            return

        hand_msg = None
        hand_samples = 0
        hand_timestamp = None

        def on_hand(msg: Any) -> None:
            nonlocal hand_msg, hand_samples, hand_timestamp
            hand_msg = msg
            hand_samples += 1
            hand_timestamp = time.time()
            hands = handstate_to_dict(hand_msg, hand_samples, hand_timestamp)
            with self.lock:
                self.latest["hands"] = hands

        def on_lowstate(msg: Any) -> None:
            try:
                hands = handstate_to_dict(hand_msg, hand_samples, hand_timestamp)
                now = time.time()
                self.samples += 1
                self.sample_times.append(now)
                if len(self.sample_times) > 1:
                    elapsed = self.sample_times[-1] - self.sample_times[0]
                    rate = (len(self.sample_times) - 1) / elapsed if elapsed > 0 else 0
                else:
                    rate = 0

                snapshot = lowstate_to_dict(msg, self.samples, rate, hands)
                with self.lock:
                    self.latest = snapshot
                with self.command_lock:
                    self.lowstate_msg = msg
            except Exception as exc:
                self._set_error(f"LowState callback failed: {exc}")

        try:
            ChannelFactoryInitialize(self.domain, self.camera_source or None)
            wrist_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
            wrist_pub.Init()
            lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
            lowcmd_pub.Init()
            motion_switcher = MotionSwitcherClient()
            motion_switcher.SetTimeout(5.0)
            motion_switcher.Init()
            loco_client = LocoClient()
            loco_client.SetTimeout(5.0)
            loco_client.Init()
            with self.command_lock:
                self.wrist_publisher = wrist_pub
                self.lowcmd_publisher = lowcmd_pub
                self.motion_switcher = motion_switcher
                self.loco_client = loco_client
                self.lowcmd_factory = unitree_hg_msg_dds__LowCmd_
                self.lowcmd_type = LowCmd_
                self.crc = CRC()
                self.wrist_status = {
                    **self.wrist_status,
                    "available": True,
                    "message": "DDS arm_sdk publisher is ready.",
                    "updated_at": time.time(),
                }

            sub = ChannelSubscriber("rt/lowstate", LowState_)
            sub.Init(on_lowstate, 10)

            hand_sub = ChannelSubscriber("rt/inspire/state", MotorStates_)
            hand_sub.Init(on_hand, 10)
        except Exception as exc:
            self._set_error(f"Could not initialize DDS subscriber: {exc}")
            return

        while self.running:
            try:
                hands = handstate_to_dict(hand_msg, hand_samples, hand_timestamp)
                with self.lock:
                    self.latest["hands"] = hands
                time.sleep(0.25)
            except Exception as exc:
                self._set_error(f"Subscriber loop failed: {exc}")
                time.sleep(0.25)


class TelemetryHandler(BaseHTTPRequestHandler):
    store: TelemetryStore

    def log_message(self, fmt: str, *args: Any) -> None:
        sys.stdout.write("[%s] %s\n" % (self.log_date_time_string(), fmt % args))

    def do_GET(self) -> None:
        request_path = urlsplit(self.path).path
        if request_path in ("/", "/index.html"):
            self._send_file(STATIC_DIR / "index.html", "text/html; charset=utf-8")
        elif request_path == "/app.js":
            self._send_file(STATIC_DIR / "app.js", "application/javascript; charset=utf-8")
        elif request_path == "/viewer.js":
            self._send_file(STATIC_DIR / "viewer.js", "application/javascript; charset=utf-8")
        elif request_path == "/styles.css":
            self._send_file(STATIC_DIR / "styles.css", "text/css; charset=utf-8")
        elif request_path == "/api/state":
            self._send_json(self.store.snapshot())
        elif request_path == "/api/camera":
            self._send_json(self.store.camera_snapshot())
        elif request_path == "/api/ros-graph":
            self._send_json(self.store.ros_graph_snapshot())
        elif request_path == "/api/wrist/status":
            self._send_json(self.store.wrist_snapshot())
        elif request_path == "/camera.mjpg":
            self._send_camera_stream()
        elif request_path == "/events":
            self._send_events()
        elif request_path.startswith("/models/") or request_path.startswith("/vendor/") or request_path.startswith("/assets/"):
            self._send_static_asset(request_path)
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def do_POST(self) -> None:
        request_path = urlsplit(self.path).path
        if request_path not in ("/api/wrist/command", "/api/wrist/stop", "/api/robot/chill"):
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")
            return

        payload: dict[str, Any] = {}
        if request_path == "/api/wrist/command":
            try:
                length = int(self.headers.get("Content-Length", "0"))
                body = self.rfile.read(min(length, 4096))
                payload = json.loads(body.decode("utf-8")) if body else {}
            except Exception as exc:
                self._send_json_status({"ok": False, "error": f"Invalid JSON body: {exc}"}, HTTPStatus.BAD_REQUEST)
                return

        if request_path == "/api/wrist/stop":
            self._send_json(self.store.stop_wrist())
            return

        if request_path == "/api/robot/chill":
            status, response = self.store.chill_motors()
            self._send_json_status(response, HTTPStatus(status))
            return

        status, response = self.store.command_wrist(payload)
        self._send_json_status(response, HTTPStatus(status))

    def _send_file(self, path: Path, content_type: str) -> None:
        try:
            body = path.read_bytes()
        except OSError:
            self.send_error(HTTPStatus.NOT_FOUND, "Missing asset")
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_static_asset(self, request_path: str) -> None:
        relative = Path(unquote(request_path.lstrip("/")))
        path = (STATIC_DIR / relative).resolve()
        try:
            path.relative_to(STATIC_DIR.resolve())
        except ValueError:
            self.send_error(HTTPStatus.FORBIDDEN, "Forbidden")
            return

        content_type = mimetypes.guess_type(path.name)[0]
        if content_type is None:
            if path.suffix.lower() == ".stl":
                content_type = "model/stl"
            elif path.suffix.lower() in (".urdf", ".xml"):
                content_type = "application/xml"
            elif path.suffix.lower() == ".js":
                content_type = "application/javascript"
            else:
                content_type = "application/octet-stream"

        self._send_file(path, content_type)

    def _send_json(self, data: dict[str, Any]) -> None:
        self._send_json_status(data, HTTPStatus.OK)

    def _send_json_status(self, data: dict[str, Any], status: HTTPStatus) -> None:
        body = json.dumps(data, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()

        while True:
            payload = json.dumps(self.store.snapshot(), separators=(",", ":"))
            try:
                self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                break
            time.sleep(0.1)

    def _send_camera_stream(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Pragma", "no-cache")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        last_timestamp: float | None = None
        while True:
            frame, timestamp = self.store.wait_for_camera_frame(last_timestamp, timeout=1.0)
            if frame is None:
                continue
            if timestamp == last_timestamp:
                continue
            last_timestamp = timestamp
            payload = (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                + f"Content-Length: {len(frame)}\r\n\r\n".encode("utf-8")
                + frame
                + b"\r\n"
            )
            try:
                self.wfile.write(payload)
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                break


def main() -> None:
    parser = argparse.ArgumentParser(description="Unitree telemetry web dashboard")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--robot-host", default="192.168.123.164")
    parser.add_argument("--camera-source", default=os.environ.get("CAMERA_SOURCE", ""))
    parser.add_argument("--camera-resolution", type=int, default=int(os.environ.get("CAMERA_RESOLUTION", "360")))
    parser.add_argument("--camera-output", default=str(CAMERA_JPEG_PATH))
    parser.add_argument("--camera-bridge", action="store_true")
    parser.add_argument("--disable-camera", action="store_true")
    args = parser.parse_args()

    if args.camera_bridge:
        camera_bridge_main(args.camera_source, args.camera_resolution, Path(args.camera_output))
        return

    store = TelemetryStore(domain=args.domain, robot_host=args.robot_host)
    store.camera_source = args.camera_source or route_interface(args.robot_host) or default_interface() or ""
    store.camera_resolution = args.camera_resolution
    store.start()
    if args.disable_camera:
        store.set_camera_error("Camera worker disabled for this server run.")
    else:
        start_camera_bridge(store)

    TelemetryHandler.store = store
    server = ThreadingHTTPServer((args.host, args.port), TelemetryHandler)

    print("Unitree telemetry dashboard")
    print(f"Listening on http://{args.host}:{args.port}")
    print(f"Try from another machine: http://{public_host()}:{args.port}")
    print("Press Ctrl+C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        store.running = False
        if store.camera_process is not None:
            store.camera_process.terminate()
        server.server_close()


if __name__ == "__main__":
    main()
