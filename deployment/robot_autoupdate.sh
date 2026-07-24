#!/usr/bin/env bash
set -euo pipefail

# One update at a time: a timer tick overlapping a still-running install
# re-restarted services mid-deploy (and mid arm-motion) on 2026-07-23.
# Degrade gracefully if util-linux flock is absent (don't become a silent
# permanent no-op).
if command -v flock >/dev/null 2>&1; then
  exec 9>/tmp/robot_autoupdate.lock
  if ! flock -n 9; then
    exit 0
  fi
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

installed="$(cat "$marker" 2>/dev/null || true)"
[ -n "$installed" ] || installed=none   # empty (truncated write) counts as none
head="$(git rev-parse HEAD)"
needs_install=1
if [ "$installed" = "$head" ]; then
  needs_install=0
elif [ "$installed" != "none" ] && git rev-parse --verify --quiet "$installed" >/dev/null; then
  # The robot itself commits feedback data (data/pose_feedback.csv). A delta
  # that touches ONLY data/ is not new code: adopt it without reinstalling,
  # otherwise every operator thumbs-up/down would restart all services and
  # kill live telemetry streams and in-flight chats. Detected with plain git +
  # a pure-shell scan (no `:(exclude)` magic pathspec, no grep-flavor exit
  # quirks) and FAIL-CLOSED: if the diff can't be computed, deploy.
  if changed="$(git diff --name-only "$installed" "$head" 2>/dev/null)"; then
    data_only=1
    while IFS= read -r changed_file; do
      case "$changed_file" in
        data/*|"") ;;              # feedback data (or blank) — not code
        *) data_only=0 ;;          # anything else is a real code change
      esac
    done <<EOF
$changed
EOF
    if [ -n "$changed" ] && [ "$data_only" = 1 ]; then
      needs_install=0
      echo "$head" > "$marker"
    fi
  fi
fi
if [ "$xr_update_rc" = "10" ]; then
  needs_install=1
fi
if [ "$needs_install" = "1" ]; then
  # Reuse the XR result we already computed so install doesn't re-run the XR
  # update (which would report "no change" and skip a needed xr-teleop restart).
  XR_UPDATE_RC="$xr_update_rc" deployment/install_robot_services.sh
  echo "$head" > "$marker"
fi
