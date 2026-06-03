#!/usr/bin/env python3
"""Copy and run a local Python motion script on the robot over SSH."""

from __future__ import annotations

import argparse
import shlex
import subprocess
from pathlib import Path


REPO_DIR = Path(__file__).resolve().parents[2]
DEFAULT_SDK = str(REPO_DIR / "execution/semantic_teleoperation/external/unitree_sdk2_python")


def run(cmd: list[str]) -> None:
    print("+", " ".join(shlex.quote(part) for part in cmd))
    subprocess.run(cmd, check=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("script", help="Local .py file to copy and run on the robot.")
    parser.add_argument("script_args", nargs=argparse.REMAINDER)
    parser.add_argument("--robot", default="ch@192.168.123.164")
    parser.add_argument("--remote-dir", default="~/robot_motion_tools")
    parser.add_argument("--python", default="python3")
    parser.add_argument("--sdk-path", default=DEFAULT_SDK)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    script = Path(args.script).expanduser().resolve()
    if not script.exists():
        raise SystemExit(f"script not found: {script}")

    remote_dir = args.remote_dir
    remote_script = f"{remote_dir}/{script.name}"
    run(["ssh", args.robot, "mkdir", "-p", remote_dir])
    run(["scp", str(script), f"{args.robot}:{remote_script}"])

    quoted_args = " ".join(shlex.quote(part) for part in args.script_args)
    remote_command = (
        f"cd {shlex.quote(remote_dir)} && "
        f"export PYTHONPATH={shlex.quote(args.sdk_path)}:${{PYTHONPATH:-}} && "
        f"{shlex.quote(args.python)} {shlex.quote(script.name)} {quoted_args}"
    )
    run(["ssh", "-tt", args.robot, remote_command])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
