#!/usr/bin/env bash
set -euo pipefail

# One update at a time: a timer tick overlapping a still-running install
# re-restarted services mid-deploy (and mid arm-motion) on 2026-07-23.
exec 9>/tmp/robot_autoupdate.lock
if ! flock -n 9; then
  exit 0
fi

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
head="$(git rev-parse HEAD)"
needs_install=1
if [ "$installed" = "$head" ]; then
  needs_install=0
elif [ "$installed" != "none" ] && git rev-parse --verify --quiet "$installed" >/dev/null; then
  # The robot itself commits feedback data (data/pose_feedback.csv). A delta
  # that touches ONLY data/ is not new code: adopt it without reinstalling,
  # otherwise every operator thumbs-up/down would restart all services and
  # kill live telemetry streams and in-flight chats.
  if [ -z "$(git diff --name-only "$installed" "$head" -- ':(exclude)data')" ]; then
    needs_install=0
    echo "$head" > "$marker"
  fi
fi
if [ "$xr_update_rc" = "10" ]; then
  needs_install=1
fi
if [ "$needs_install" = "1" ]; then
  deployment/install_robot_services.sh
  echo "$head" > "$marker"
fi
