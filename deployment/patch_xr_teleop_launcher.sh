#!/usr/bin/env bash
set -euo pipefail

LAUNCHER="${1:-/home/unitree/xr_teleoperate/deploy/scripts/start_xr_teleop.sh}"

if [[ ! -f "$LAUNCHER" ]]; then
  echo "XR teleop launcher not found: $LAUNCHER" >&2
  exit 0
fi

if ! grep -q 'XR_TELEOP_SEND_START' "$LAUNCHER"; then
  sed -i '/XR_NETWORK_INTERFACE=/a export XR_TELEOP_SEND_START="${XR_TELEOP_SEND_START:-0}"' "$LAUNCHER"
fi

sed -i 's/XR_NETWORK_INTERFACE:-wlx74da387f0099/XR_NETWORK_INTERFACE:-eth0/' "$LAUNCHER"

if ! grep -q 'if \[\[ "${XR_TELEOP_SEND_START}" == "1" \]\]; then' "$LAUNCHER"; then
  python3 - "$LAUNCHER" <<'PATCH_XR_TELEOP_PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text()
old = '''MAMBA_ROOT_PREFIX=/home/unitree/.micromamba /home/unitree/.local/micromamba run -n tv python - <<'PY'
import time
from teleop.utils.ipc import IPC_Client

client = IPC_Client(hb_fps=10.0)
for _ in range(60):
    if client.is_online():
        break
    time.sleep(0.1)
print(client.send_data("CMD_START"))
client.stop()
PY
'''
new = '''if [[ "${XR_TELEOP_SEND_START}" == "1" ]]; then
  MAMBA_ROOT_PREFIX=/home/unitree/.micromamba /home/unitree/.local/micromamba run -n tv python - <<'PY'
import time
from teleop.utils.ipc import IPC_Client

client = IPC_Client(hb_fps=10.0)
for _ in range(60):
    if client.is_online():
        break
    time.sleep(0.1)
print(client.send_data("CMD_START"))
client.stop()
PY
fi
'''
if old not in text:
    raise SystemExit("Could not find IPC start block in XR launcher")
path.write_text(text.replace(old, new))
PATCH_XR_TELEOP_PY
fi
