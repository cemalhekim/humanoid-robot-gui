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
if ! grep -q 'XR_TELEOP_FREQUENCY' "$LAUNCHER"; then
  sed -i '/XR_TELEOP_SEND_START=/a export XR_TELEOP_FREQUENCY="${XR_TELEOP_FREQUENCY:-60}"' "$LAUNCHER"
fi
if ! grep -q 'XR_TELEOP_EE' "$LAUNCHER"; then
  sed -i '/XR_TELEOP_VUER_PORT=/a export XR_TELEOP_EE="${XR_TELEOP_EE:-}"' "$LAUNCHER"
fi

sed -i 's/XR_NETWORK_INTERFACE:-wlx74da387f0099/XR_NETWORK_INTERFACE:-eth0/' "$LAUNCHER"
sed -i 's/XR_TELEOP_VUER_PORT=8013/XR_TELEOP_VUER_PORT=8012/' "$LAUNCHER"
sed -i 's/--frequency 30/--frequency "$XR_TELEOP_FREQUENCY"/' "$LAUNCHER"

python3 - "$LAUNCHER" <<'PATCH_XR_TELEOP_PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
lines = path.read_text().splitlines()
if not any('TELEOP_ARGS+=(--ee "${XR_TELEOP_EE}")' in line for line in lines):
    try:
        args_start = next(i for i, line in enumerate(lines) if line.strip() == "TELEOP_ARGS=(")
        args_end = next(i for i in range(args_start + 1, len(lines)) if lines[i].strip() == ")")
    except StopIteration:
        raise SystemExit("Could not find TELEOP_ARGS block in XR launcher")
    ee_block = [
        "",
        'if [[ -n "${XR_TELEOP_EE:-}" ]]; then',
        '  TELEOP_ARGS+=(--ee "${XR_TELEOP_EE}")',
        "fi",
    ]
    lines = lines[: args_end + 1] + ee_block + lines[args_end + 1 :]
server_only_line = 'if [[ "${XR_TELEOP_SEND_START}" != "1" ]]; then'
if server_only_line in lines:
    start = lines.index(server_only_line)
else:
    try:
        start = lines.index('"${TELEOP_ARGS[@]}" >"$LOG_FILE" 2>&1 &')
    except ValueError:
        if 'exec "${TELEOP_ARGS[@]}" >"$LOG_FILE" 2>&1' in lines:
            raise SystemExit(0)
        raise SystemExit("Could not find teleop start block in XR launcher")

replacement = [
    server_only_line,
    '  exec "${TELEOP_ARGS[@]}" >"$LOG_FILE" 2>&1',
    'fi',
    '',
    '"${TELEOP_ARGS[@]}" >"$LOG_FILE" 2>&1 &',
    'TELEOP_PID=$!',
    '',
    "MAMBA_ROOT_PREFIX=/home/unitree/.micromamba /home/unitree/.local/micromamba run -n tv python - <<'PY'",
    'import time',
    'from teleop.utils.ipc import IPC_Client',
    '',
    'client = IPC_Client(hb_fps=10.0)',
    'for _ in range(60):',
    '    if client.is_online():',
    '        break',
    '    time.sleep(0.1)',
    'print(client.send_data("CMD_START"))',
    'client.stop()',
    'PY',
    '',
    'wait "$TELEOP_PID"',
]
path.write_text("\n".join(lines[:start] + replacement) + "\n")
PATCH_XR_TELEOP_PY
