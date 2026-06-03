#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="${REPO_DIR}/external/unitree_sim_isaaclab"

DEVICE="${DEVICE:-cpu}"
TASK="${TASK:-Isaac-PickPlace-Cylinder-H12-27dof-Inspire-Joint}"
ROBOT_TYPE="${ROBOT_TYPE:-h1_2}"
DEX_FLAG="${DEX_FLAG:---enable_inspire_dds}"

cd "$SIM_DIR"

exec python sim_main.py \
  --device "$DEVICE" \
  --enable_cameras \
  --task "$TASK" \
  "$DEX_FLAG" \
  --robot_type "$ROBOT_TYPE"

