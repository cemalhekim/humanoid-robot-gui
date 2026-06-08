#!/usr/bin/env bash
set -euo pipefail

cd /home/unitree/robot_telemetry_web

if ! git fetch origin main; then
  exit 0
fi

old="$(git rev-parse HEAD)"
new="$(git rev-parse origin/main)"

changed=0

if [ "$old" != "$new" ]; then
  git checkout -B main origin/main
  git reset --hard origin/main
  changed=1
fi

xr_update_rc=0
deployment/update_xr_teleoperate.sh || xr_update_rc=$?
if [ "$xr_update_rc" = "10" ]; then
  changed=1
elif [ "$xr_update_rc" != "0" ]; then
  exit 0
fi

if [ "$changed" = "1" ]; then
  deployment/install_robot_services.sh
fi
