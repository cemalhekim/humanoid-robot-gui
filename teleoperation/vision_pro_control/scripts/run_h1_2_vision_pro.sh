#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
XR_DIR="${REPO_DIR}/external/xr_teleoperate/teleop"

INPUT_MODE="${INPUT_MODE:-hand}"
DISPLAY_MODE="${DISPLAY_MODE:-immersive}"
ARM="${ARM:-H1_2}"
IMG_SERVER_IP="${IMG_SERVER_IP:-192.168.123.164}"
NETWORK_INTERFACE="${NETWORK_INTERFACE:-eth0}"
FREQUENCY="${FREQUENCY:-30.0}"
MOTION_FLAG="--motion"

for arg in "$@"; do
  case "$arg" in
    --no-motion)
      MOTION_FLAG=""
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      exit 2
      ;;
  esac
done

cd "$XR_DIR"

exec python teleop_hand_and_arm.py \
  --frequency "$FREQUENCY" \
  --input-mode "$INPUT_MODE" \
  --display-mode "$DISPLAY_MODE" \
  --arm "$ARM" \
  --img-server-ip "$IMG_SERVER_IP" \
  --network-interface "$NETWORK_INTERFACE" \
  $MOTION_FLAG
