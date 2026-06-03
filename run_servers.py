#!/usr/bin/env python3
"""Start the robot dashboard from a clean server state."""

from __future__ import annotations

import argparse
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path


APP_DIR = Path(__file__).resolve().parent
SERVICE_NAME = "robot-telemetry-web.service"
DEFAULT_MICROMAMBA = Path("/home/unitree/.local/micromamba")
DEFAULT_MAMBA_ROOT = Path("/home/unitree/.micromamba")
DEFAULT_LOG_DIR = Path("/home/unitree/logs")
DEFAULT_SYSTEMD_DIR = Path.home() / ".config/systemd/user"


def systemctl_available() -> bool:
    return shutil.which("systemctl") is not None and os.environ.get("XDG_RUNTIME_DIR") is not None


def default_mode() -> str:
    return "systemd" if Path.home().name == "unitree" and systemctl_available() else "foreground"


def build_server_command(args: argparse.Namespace) -> list[str]:
    server = str(APP_DIR / "server.py")
    server_args = [
        server,
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--domain",
        str(args.domain),
        "--robot-host",
        args.robot_host,
        "--camera-source",
        args.camera_source,
    ]
    if server_supports("--camera-backend"):
        server_args.extend(["--camera-backend", args.camera_backend])
    if args.camera_resolution is not None:
        server_args.extend(["--camera-resolution", str(args.camera_resolution)])

    micromamba = Path(args.micromamba).expanduser()
    if args.use_micromamba and micromamba.exists():
        return [str(micromamba), "run", "-n", args.env, "python", "-u", *server_args]
    return [sys.executable, "-u", *server_args]


def server_supports(option: str) -> bool:
    try:
        return option in (APP_DIR / "server.py").read_text(encoding="utf-8")
    except OSError:
        return False


def run(command: list[str], dry_run: bool) -> subprocess.CompletedProcess[str]:
    print("$", shlex.join(command))
    if dry_run:
        return subprocess.CompletedProcess(command, 0, "", "")
    return subprocess.run(command, text=True, check=False)


def kill_existing(args: argparse.Namespace) -> int:
    if args.no_kill_first:
        return 0
    command = [sys.executable, str(APP_DIR / "kill_servers.py")]
    if args.dry_run:
        command.append("--dry-run")
    if args.include_all_python:
        command.append("--include-all-python")
    result = run(command, args.dry_run)
    return result.returncode


def write_systemd_service(args: argparse.Namespace, command: list[str]) -> Path:
    systemd_dir = Path(args.systemd_dir).expanduser()
    service_path = systemd_dir / SERVICE_NAME
    log_dir = Path(args.log_dir).expanduser()
    if args.dry_run:
        print(f"Would create {service_path}")
        return service_path

    systemd_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    lines = [
        "[Unit]",
        "Description=Robot Telemetry Web Dashboard",
        "After=default.target",
        "",
        "[Service]",
        "Type=simple",
        f"WorkingDirectory={APP_DIR}",
    ]
    if args.use_micromamba and Path(args.micromamba).expanduser().exists():
        lines.append(f"Environment=MAMBA_ROOT_PREFIX={Path(args.mamba_root).expanduser()}")
    lines.extend(
        [
            f"ExecStart={shlex.join(command)}",
            "Restart=always",
            "RestartSec=3",
            f"StandardOutput=append:{log_dir / 'robot_telemetry_web.log'}",
            f"StandardError=append:{log_dir / 'robot_telemetry_web.log'}",
            "",
            "[Install]",
            "WantedBy=default.target",
            "",
        ]
    )
    service_path.write_text("\n".join(lines), encoding="utf-8")
    return service_path


def start_systemd(args: argparse.Namespace, command: list[str]) -> int:
    if not systemctl_available() and not args.dry_run:
        print("systemctl --user is not available; use --foreground instead.", file=sys.stderr)
        return 2
    service_path = write_systemd_service(args, command)
    print(f"Using service file: {service_path}")
    for command_line in (
        ["systemctl", "--user", "daemon-reload"],
        ["systemctl", "--user", "enable", "--now", SERVICE_NAME],
        ["systemctl", "--user", "status", SERVICE_NAME, "--no-pager"],
    ):
        result = run(command_line, args.dry_run)
        if result.returncode != 0:
            return result.returncode
    return 0


def start_foreground(args: argparse.Namespace, command: list[str]) -> int:
    print("Starting dashboard in the foreground. Press Ctrl+C to stop.")
    if args.dry_run:
        print("$", shlex.join(command))
        return 0
    os.execvp(command[0], command)
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Kill old dashboard servers and start a fresh one.")
    parser.add_argument(
        "--mode",
        choices=("auto", "systemd", "foreground"),
        default="auto",
        help="auto uses systemd for the unitree user and foreground elsewhere.",
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8088)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--robot-host", default="192.168.123.164")
    parser.add_argument("--camera-source", default="eth0")
    parser.add_argument("--camera-backend", choices=("auto", "teleimager", "ros2"), default="teleimager")
    parser.add_argument("--camera-resolution", type=int, default=360)
    parser.add_argument("--env", default="tv", help="Micromamba environment name.")
    parser.add_argument("--micromamba", default=str(DEFAULT_MICROMAMBA))
    parser.add_argument("--mamba-root", default=str(DEFAULT_MAMBA_ROOT))
    parser.add_argument("--systemd-dir", default=str(DEFAULT_SYSTEMD_DIR))
    parser.add_argument("--log-dir", default=str(DEFAULT_LOG_DIR))
    parser.add_argument("--no-micromamba", dest="use_micromamba", action="store_false")
    parser.set_defaults(use_micromamba=True)
    parser.add_argument("--no-kill-first", action="store_true")
    parser.add_argument("--include-all-python", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    mode = default_mode() if args.mode == "auto" else args.mode
    kill_code = kill_existing(args)
    if kill_code != 0:
        return kill_code

    command = build_server_command(args)
    print("Server command:", shlex.join(command))
    if mode == "systemd":
        return start_systemd(args, command)
    return start_foreground(args, command)


if __name__ == "__main__":
    raise SystemExit(main())
