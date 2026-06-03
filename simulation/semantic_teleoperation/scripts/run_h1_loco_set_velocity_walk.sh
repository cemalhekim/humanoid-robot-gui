#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -d "$repo_root/external/unitree_mujoco" || ! -d "$repo_root/external/unitree_sdk2_python" ]]; then
  "$repo_root/scripts/fetch_unitree_mujoco.sh"
fi

touch "$repo_root/external/unitree_sdk2_python/unitree_sdk2py/b2/__init__.py"
export PYTHONPATH="$repo_root/external/unitree_sdk2_python${PYTHONPATH:+:$PYTHONPATH}"

cd "$repo_root"
python3 "$repo_root/scripts/h1_loco_set_velocity_walk.py" "$@"
