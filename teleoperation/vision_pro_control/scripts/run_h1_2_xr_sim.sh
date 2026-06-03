#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
XR_DIR="${REPO_DIR}/external/xr_teleoperate/teleop"

INPUT_MODE="${INPUT_MODE:-hand}"
DISPLAY_MODE="${DISPLAY_MODE:-immersive}"
ARM="${ARM:-H1_2}"
EE="${EE:-inspire_dfx}"
FREQUENCY="${FREQUENCY:-30.0}"

cd "$XR_DIR"

exec python teleop_hand_and_arm.py \
  --frequency "$FREQUENCY" \
  --input-mode "$INPUT_MODE" \
  --display-mode "$DISPLAY_MODE" \
  --arm "$ARM" \
  --ee "$EE" \
  --sim

