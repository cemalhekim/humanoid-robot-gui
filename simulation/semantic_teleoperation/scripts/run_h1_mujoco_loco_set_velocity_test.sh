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
  for pid in "${client_pid:-}" "${shim_pid:-}" "${sim_pid:-}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

cd "$repo_root/external/unitree_mujoco/simulate_python"
python3 ./unitree_mujoco.py &
sim_pid=$!

cd "$repo_root"
python3 "$repo_root/scripts/h1_mujoco_loco_shim.py" &
shim_pid=$!

sleep 3
python3 "$repo_root/scripts/h1_loco_set_velocity_walk.py" lo \
  --domain-id 1 \
  --vx "${VX:-0.35}" \
  --vy "${VY:-0.0}" \
  --vyaw "${VYAW:-0.0}" \
  --duration "${DURATION:-1.5}" \
  --timeout "${TIMEOUT:-30.0}" \
  --standup-delay "${STANDUP_DELAY:-0.5}" \
  "$@" &
client_pid=$!

wait "$client_pid"

echo
echo "SetVelocity sim test finished. Close the MuJoCo viewer or press Ctrl-C to stop."
wait "$sim_pid"
