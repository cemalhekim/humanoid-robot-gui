#!/usr/bin/env python3
"""Stop dashboard-related Python servers started from this workspace."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


SERVICE_NAMES = (
    "robot-telemetry-web.service",
    "robot-telemetry-web-main.service",
)
PROCESS_MARKERS = (
    "robot_telemetry_web/server.py",
    "/server.py --host",
    "python3 -m http.server",
    "python -m http.server",
)
SCRIPT_NAMES = {
    "kill_servers.py",
    "run_servers.py",
}


def run_quiet(command: list[str], dry_run: bool) -> None:
    print("$", " ".join(command))
    if dry_run:
        return
    subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)


def stop_systemd_units(dry_run: bool) -> None:
    if not shutil_available("systemctl"):
        return
    for service in SERVICE_NAMES:
        run_quiet(["systemctl", "--user", "stop", service], dry_run)
        run_quiet(["systemctl", "--user", "kill", "-s", "KILL", service], dry_run)
    run_quiet(["systemctl", "--user", "reset-failed", *SERVICE_NAMES], dry_run)


def shutil_available(binary: str) -> bool:
    for directory in os.environ.get("PATH", "").split(os.pathsep):
        if directory and (Path(directory) / binary).exists():
            return True
    return False


def proc_cmdline(pid: int) -> str | None:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return None
    return raw.replace(b"\x00", b" ").decode("utf-8", errors="replace").strip()


def matching_pids(include_all_python: bool) -> dict[int, str]:
    current_pid = os.getpid()
    parent_pid = os.getppid()
    matches: dict[int, str] = {}
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        pid = int(proc.name)
        if pid in (current_pid, parent_pid):
            continue
        cmdline = proc_cmdline(pid)
        if not cmdline:
            continue
        if any(name in cmdline for name in SCRIPT_NAMES):
            continue
        if include_all_python and "python" in cmdline and "server" in cmdline:
            matches[pid] = cmdline
            continue
        if any(marker in cmdline for marker in PROCESS_MARKERS):
            matches[pid] = cmdline
    return matches


def signal_pids(pids: dict[int, str], sig: signal.Signals, dry_run: bool) -> None:
    for pid, cmdline in sorted(pids.items()):
        print(f"{sig.name} {pid}: {cmdline}")
        if dry_run:
            continue
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
        except PermissionError as exc:
            print(f"Permission denied for pid {pid}: {exc}", file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(description="Stop dashboard Python server processes.")
    parser.add_argument("--dry-run", action="store_true", help="Print what would be stopped without killing it.")
    parser.add_argument(
        "--no-systemd",
        action="store_true",
        help="Do not stop robot-telemetry-web user systemd units.",
    )
    parser.add_argument(
        "--include-all-python",
        action="store_true",
        help="Also kill Python commands that contain both 'python' and 'server'. Use only on the robot dashboard host.",
    )
    parser.add_argument("--grace", type=float, default=2.0, help="Seconds to wait between TERM and KILL.")
    args = parser.parse_args()

    if not args.no_systemd:
        stop_systemd_units(args.dry_run)

    pids = matching_pids(args.include_all_python)
    if not pids:
        print("No matching dashboard Python server processes found.")
        return 0

    signal_pids(pids, signal.SIGTERM, args.dry_run)
    if args.dry_run:
        return 0
    time.sleep(max(0.0, args.grace))

    remaining = matching_pids(args.include_all_python)
    if remaining:
        signal_pids(remaining, signal.SIGKILL, args.dry_run)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
