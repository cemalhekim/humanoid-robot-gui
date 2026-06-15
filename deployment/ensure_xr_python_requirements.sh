#!/usr/bin/env bash
set -euo pipefail

XR_DIR="${XR_TELEOPERATE_DIR:-/home/unitree/xr_teleoperate}"
MAMBA="${MAMBA_BIN:-/home/unitree/.local/micromamba}"
ENV_NAME="${XR_TELEOP_ENV:-tv}"
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-/home/unitree/.micromamba}"

if [[ ! -f "$XR_DIR/requirements.txt" ]]; then
  exit 0
fi

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

if ! "$MAMBA" run -n "$ENV_NAME" python -m pip install -r "$XR_DIR/requirements.txt"; then
  echo "Warning: could not verify or install XR Python requirements; continuing with existing environment." >&2
fi
