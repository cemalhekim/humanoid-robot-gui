#!/usr/bin/env bash
# Run the dashboard server.py against the H1-2 MuJoCo twin instead of the robot.
# Same code path as production: DDS telemetry in, rt/arm_sdk + rt/lowcmd out.
# Only the transport differs: loopback interface, DDS domain 1, no camera, no
# systemd XR-suspend (there is no XR service to stop on a sim host).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$here/../.." && pwd)"
python_bin="${HANS_SIM_PYTHON:-python3}"
port="${HANS_SIM_PORT:-8088}"
export PYTHONPATH="$repo_root/execution/semantic_teleoperation/external/unitree_sdk2_python${PYTHONPATH:+:$PYTHONPATH}"
export RTW_SKIP_XR_SUSPEND=1
cd "$repo_root"
exec "$python_bin" -u server.py \
  --host 0.0.0.0 --port "$port" \
  --domain "${HANS_SIM_DOMAIN:-1}" --robot-host 127.0.0.1 --camera-source lo --disable-camera "$@"
