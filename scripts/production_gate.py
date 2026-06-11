#!/usr/bin/env python3
"""Run the offline production gate for safe incremental refactoring.

The default gate is intentionally offline: it must not require robot access and
must not publish DDS commands. Use --live only for an explicit robot reachability
and service check after the offline gate passes.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OWNED_PYTHON_PREFIXES = (
    "deployment/",
    "tools/",
)
EXCLUDED_PREFIXES = (
    "execution/",
    "simulation/",
    "teleoperation/vision_pro_control/external/",
    "vendor/",
)
STATIC_JS = (
    "static/app.js",
    "static/viewer.js",
)


def run(command: list[str], *, cwd: Path = ROOT) -> int:
    print("$", " ".join(command), flush=True)
    completed = subprocess.run(command, cwd=cwd, text=True, check=False)
    return completed.returncode


def git_files() -> list[str]:
    result = subprocess.run(
        ["git", "ls-files"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        print(result.stderr.strip(), file=sys.stderr)
        raise SystemExit(result.returncode)
    return [line for line in result.stdout.splitlines() if line]


def owned_python_files(files: list[str]) -> list[str]:
    selected: list[str] = []
    for path in files:
        if not path.endswith(".py"):
            continue
        if path.startswith(EXCLUDED_PREFIXES):
            continue
        if path in {"server.py", "run_servers.py", "kill_servers.py"} or path.startswith(OWNED_PYTHON_PREFIXES):
            selected.append(path)
    return selected


def shell_files(files: list[str]) -> list[str]:
    return [path for path in files if path.endswith(".sh") and path.startswith("deployment/")]


def python_compile(files: list[str]) -> int:
    if not files:
        return 0
    return run([sys.executable, "-m", "py_compile", *files])


def unit_tests() -> int:
    return run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_*.py"])


def shell_syntax(files: list[str]) -> int:
    for path in files:
        code = run(["bash", "-n", path])
        if code != 0:
            return code
    return 0


def javascript_syntax() -> int:
    node = shutil.which("node")
    if node is None:
        print("node is required for JavaScript syntax checks.", file=sys.stderr)
        return 127
    for path in STATIC_JS:
        code = run([node, "--check", path])
        if code != 0:
            return code
    return 0


def live_robot_check(robot_host: str) -> int:
    ssh = shutil.which("ssh")
    if ssh is None:
        print("ssh is required for --live checks.", file=sys.stderr)
        return 127
    remote = (
        "systemctl --user is-active "
        "teleimager.service inspire-hands.service robot-telemetry-web.service xr-teleop.service; "
        "ss -ltnp | egrep ':(60001|8012|8088)' || true"
    )
    return run([ssh, "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", f"unitree@{robot_host}", remote])


def main() -> int:
    parser = argparse.ArgumentParser(description="Run production safety checks before/after a refactor slice.")
    parser.add_argument("--live", action="store_true", help="Also check robot-side service status over SSH.")
    parser.add_argument("--robot-host", default="10.2.100.142", help="Robot PC SSH host used by --live.")
    args = parser.parse_args()

    files = git_files()
    steps = [
        ("python syntax", lambda: python_compile(owned_python_files(files))),
        ("unit contracts", unit_tests),
        ("shell syntax", lambda: shell_syntax(shell_files(files))),
        ("javascript syntax", javascript_syntax),
    ]
    if args.live:
        steps.append(("live robot services", lambda: live_robot_check(args.robot_host)))

    for name, step in steps:
        print(f"\n== {name} ==", flush=True)
        code = step()
        if code != 0:
            print(f"\nFAILED: {name}", file=sys.stderr)
            return code

    print("\nProduction gate passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
