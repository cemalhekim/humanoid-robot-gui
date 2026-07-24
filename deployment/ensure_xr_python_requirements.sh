#!/usr/bin/env bash
set -euo pipefail

XR_DIR="${XR_TELEOPERATE_DIR:-/home/unitree/xr_teleoperate}"
MAMBA="${MAMBA_BIN:-/home/unitree/.local/micromamba}"
ENV_NAME="${XR_TELEOP_ENV:-tv}"
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-/home/unitree/.micromamba}"

if [[ ! -f "$XR_DIR/requirements.txt" ]]; then
  exit 0
fi

# A crash-truncated micromamba proc file (empty or NUL-filled JSON) makes every
# later `micromamba run` die with a libmamba JSON parse error, which blocked
# install_robot_services.sh — and with it the dashboard service restart — on
# 2026-07-23. Drop unparseable proc files before touching micromamba. Purge BOTH
# candidate locations (the observed one under ~/.cache/mamba, and the root-prefix
# one) so the cleanup can't miss depending on the micromamba build.
MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-/home/unitree/.micromamba}" python3 - <<'PY'
import json
import glob
import os

dirs = [
    os.path.expanduser("~/.cache/mamba/proc"),
    os.path.join(os.environ.get("MAMBA_ROOT_PREFIX", ""), "proc"),
]
for directory in dirs:
    if not directory:
        continue
    for path in glob.glob(os.path.join(directory, "*.json")):
        try:
            with open(path, encoding="utf-8") as handle:
                json.load(handle)
        except Exception:
            try:
                os.remove(path)
            except OSError:
                pass
PY

if "$MAMBA" run -n "$ENV_NAME" python - <<'PY'
import importlib.util
import sys

modules = ("rerun", "meshcat", "sshkeyboard")
missing = [name for name in modules if importlib.util.find_spec(name) is None]
if missing:
    print("Missing XR Python modules:", ", ".join(missing))
    sys.exit(1)
PY
then
  exit 0
fi

"$MAMBA" run -n "$ENV_NAME" python -m pip install -r "$XR_DIR/requirements.txt"
