#!/usr/bin/env bash
set -euo pipefail

cd /home/unitree/robot_telemetry_web

# Last commit install_robot_services.sh actually ran for. Restart decisions
# compare against THIS marker instead of the transient pre-fetch HEAD: if a
# run dies after git already updated the checkout (e.g. the XR update failed
# once), the next tick still sees marker != HEAD and finishes the install —
# previously that state skipped the service restart forever, leaving the
# dashboard serving new static files on an old server process.
marker=/home/unitree/.robot_telemetry_web_installed_commit

if ! git fetch origin main; then
  exit 0
fi

if [ "$(git rev-parse HEAD)" != "$(git rev-parse origin/main)" ]; then
  git checkout -B main origin/main
  git reset --hard origin/main
fi

xr_update_rc=0
deployment/update_xr_teleoperate.sh || xr_update_rc=$?
if [ "$xr_update_rc" != "0" ] && [ "$xr_update_rc" != "10" ]; then
  exit 0
fi

installed="$(cat "$marker" 2>/dev/null || echo none)"
if [ "$installed" != "$(git rev-parse HEAD)" ] || [ "$xr_update_rc" = "10" ]; then
  deployment/install_robot_services.sh
  git rev-parse HEAD > "$marker"
fi
