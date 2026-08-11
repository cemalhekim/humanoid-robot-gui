#!/usr/bin/env bash
# Supervisor for the Inspire RH56 hand bridge (inspire_h1: serial -> DDS).
#
# This lives in THIS repo rather than in ~/xr_teleoperate/deploy/scripts/,
# because that checkout is `git reset --hard`-ed by update_xr_teleoperate.sh
# and every fix below would be wiped on the next XR update.
#
# Two failure modes seen on the robot that the original script did not cover:
#
#  1. inspire_h1 does NOT exit when its serial port disappears. Unplugging an
#     arm takes its USB converter with it; the process then spins on the dead
#     fd (~8% CPU) and keeps publishing the last hand values on
#     rt/inspire/state, so the dashboard shows connected hands that are not
#     attached to the robot. The original loop only checked the device before
#     launching, so it never noticed. We watch both ports while it runs.
#  2. On a cold boot the DDS interface is not up yet and inspire_h1 dies with
#     "eth0: does not match an available interface" -> "Failed to create domain
#     explicitly", then crash-loops. The other units gate on eth0 with an
#     ExecStartPre; do the same here.
set -uo pipefail

LOG_DIR=/home/unitree/logs
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/inspire_hands.log"

HAND_BIN="${HAND_BIN:-/home/unitree/unitree_sdk2_python/example/h1_2/DFX_inspire_service/build/inspire_h1}"
HAND_DEVICE="${XR_INSPIRE_DEVICE:-/dev/serial/by-id/usb-unitree.CN_USB_Quad_Serial_BC49B3ABCD-if00-port0}"
HAND_NETWORK="${XR_INSPIRE_NETWORK:-wlx74da387f0099}"
LEFT_HAND_DEVICE="${XR_INSPIRE_LEFT_DEVICE:-}"
NETWORK_WAIT_SECONDS="${XR_INSPIRE_NETWORK_WAIT:-60}"

log() {
  echo "$*" | tee -a "$LOG_FILE"
}

if [[ ! -x "$HAND_BIN" ]]; then
  log "inspire_h1 binary not found at $HAND_BIN"
  exit 1
fi

# Names the ports that should be there but are not, so the wait message says
# which arm is missing instead of just "waiting".
missing_devices() {
  local missing=()
  [[ -e "$HAND_DEVICE" ]] || missing+=("right:$HAND_DEVICE")
  if [[ -n "$LEFT_HAND_DEVICE" && ! -e "$LEFT_HAND_DEVICE" ]]; then
    missing+=("left:$LEFT_HAND_DEVICE")
  fi
  echo "${missing[*]-}"
}

wait_for_network() {
  for _ in $(seq 1 "$NETWORK_WAIT_SECONDS"); do
    if ip -br addr show "$HAND_NETWORK" 2>/dev/null | grep -q "UP"; then
      return 0
    fi
    sleep 1
  done
  log "network interface ${HAND_NETWORK} not up after ${NETWORK_WAIT_SECONDS}s; starting anyway"
}

run_bridge() {
  local args=(-s "$HAND_DEVICE")
  if [[ -n "$LEFT_HAND_DEVICE" ]]; then
    args+=(--left-serial "$LEFT_HAND_DEVICE")
  fi
  args+=(--network "$HAND_NETWORK")

  log "Launching inspire_h1 ${args[*]}"
  "$HAND_BIN" "${args[@]}" >>"$LOG_FILE" 2>&1 &
  local child=$!

  while kill -0 "$child" 2>/dev/null; do
    local gone
    gone="$(missing_devices)"
    if [[ -n "$gone" ]]; then
      log "serial device disappeared (${gone}); stopping inspire_h1 so it stops publishing stale hand state"
      kill "$child" 2>/dev/null
      for _ in $(seq 1 5); do
        kill -0 "$child" 2>/dev/null || break
        sleep 1
      done
      kill -9 "$child" 2>/dev/null
      break
    fi
    sleep 1
  done
  wait "$child" 2>/dev/null
  log "inspire_h1 exited; retrying in 2s"
}

log "Starting Inspire hand bridge for ${HAND_DEVICE}${LEFT_HAND_DEVICE:+ (left: ${LEFT_HAND_DEVICE})}"
wait_for_network

last_missing=""
while true; do
  missing="$(missing_devices)"
  if [[ -z "$missing" ]]; then
    last_missing=""
    run_bridge
    sleep 2
  else
    # Only log when the missing set changes: with an arm off the bench this
    # loop would otherwise write a line every 2s forever.
    if [[ "$missing" != "$last_missing" ]]; then
      log "Waiting for hand serial device(s): ${missing}"
      last_missing="$missing"
    fi
    sleep 2
  fi
done
