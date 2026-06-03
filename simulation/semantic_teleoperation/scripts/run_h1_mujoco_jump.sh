#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -d "$repo_root/external/unitree_mujoco" || ! -d "$repo_root/external/unitree_sdk2_python" ]]; then
  "$repo_root/scripts/fetch_unitree_mujoco.sh"
fi

"$repo_root/scripts/configure_unitree_mujoco_h1.py"

touch "$repo_root/external/unitree_sdk2_python/unitree_sdk2py/b2/__init__.py"
export PYTHONPATH="$repo_root/external/unitree_sdk2_python${PYTHONPATH:+:$PYTHONPATH}"

cleanup() {
  if [[ -n "${sim_pid:-}" ]] && kill -0 "$sim_pid" 2>/dev/null; then
    kill "$sim_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

cd "$repo_root/external/unitree_mujoco/simulate_python"
python3 ./unitree_mujoco.py &
sim_pid=$!

cd "$repo_root"
python3 "$repo_root/scripts/h1_mujoco_jump.py" "$@"

echo
echo "Jump demo finished. Close the MuJoCo viewer or press Ctrl-C to stop."
wait "$sim_pid"
