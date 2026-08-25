#!/usr/bin/env bash
# Start the H1-2 MuJoCo twin (headless) on loopback DDS domain 1.
# Pass extra flags through, e.g. ./run_h1_2_twin.sh --viewer
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$here/../.." && pwd)"
python_bin="${HANS_SIM_PYTHON:-python3}"
export PYTHONPATH="$repo_root/execution/semantic_teleoperation/external/unitree_sdk2_python${PYTHONPATH:+:$PYTHONPATH}"
exec "$python_bin" "$here/h1_2_mujoco_sim.py" "$@"
