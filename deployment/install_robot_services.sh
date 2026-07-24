#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SYSTEMD_USER_DIR="${HOME}/.config/systemd/user"

mkdir -p "$SYSTEMD_USER_DIR" /home/unitree/logs

# Copy every unit file up front (cheap, no side effects).
cp "$REPO_DIR/deployment/systemd/robot-telemetry-web.service" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/robot-telemetry-web-autoupdate.service" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/robot-telemetry-web-autoupdate.timer" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/teleimager.service" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/inspire-hands.service" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/xr-teleop.service" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/xr-home-watchdog.service" "$SYSTEMD_USER_DIR/"

if command -v loginctl >/dev/null 2>&1; then
  loginctl enable-linger "${USER}" 2>/dev/null || sudo -n loginctl enable-linger "${USER}" 2>/dev/null || true
fi

# ---------------------------------------------------------------------------
# Deliver the DASHBOARD first. It is the service this script exists to ship,
# and it must not be held hostage by the best-effort XR patch/requirement
# steps below: a wedged micromamba there used to abort the whole install under
# `set -e` BEFORE the dashboard ever restarted, so the fix never deployed.
# ---------------------------------------------------------------------------

# SAFETY: never restart the dashboard while it is actively commanding the robot
# (arm replay or person tracking) — killing the publisher mid-motion drops the
# arms. Wait bounded, then proceed so a wedged flag cannot block updates forever.
for attempt in $(seq 1 60); do
  motion="$(curl -s -m 2 http://localhost:8088/api/motion/active 2>/dev/null | grep -o '"active": *true' || true)"
  if [ -z "$motion" ]; then
    break
  fi
  echo "motion active; delaying dashboard restart (${attempt}/60)..."
  sleep 5
done

systemctl --user daemon-reload
systemctl --user enable --now robot-telemetry-web.service
systemctl --user enable --now robot-telemetry-web-autoupdate.timer
systemctl --user restart robot-telemetry-web.service
# Additive health note (does not block the marker): surfaces a crash-on-startup
# deploy in the log without risking a false-positive reinstall loop.
sleep 2
systemctl --user is-active --quiet robot-telemetry-web.service \
  || echo "warn: robot-telemetry-web not active shortly after restart" >&2

# ---------------------------------------------------------------------------
# Best-effort auxiliary work: XR repo update, source patches, python deps, and
# the non-dashboard services. NONE of this may abort the script now that the
# dashboard is already delivered.
# ---------------------------------------------------------------------------

# Reuse the XR update result from robot_autoupdate.sh when present, so the XR
# repo is fetched at most once per cycle (a second run would report "no change"
# and wrongly skip the xr-teleop restart below).
if [ -n "${XR_UPDATE_RC:-}" ]; then
  xr_update_rc="$XR_UPDATE_RC"
else
  xr_update_rc=0
  "$REPO_DIR/deployment/update_xr_teleoperate.sh" || xr_update_rc=$?
fi

"$REPO_DIR/deployment/patch_xr_teleop_launcher.sh" || echo "warn: patch_xr_teleop_launcher failed" >&2
for patch in \
  patch_xr_camera_config.py patch_xr_image_server.py patch_xr_dex_retargeting.py \
  patch_xr_inspire_direct_curl.py patch_xr_hand_input_swap.py patch_xr_head_tilt_loco.py \
  patch_xr_root_children_visual.py patch_vuer_loco_pointer_events.py; do
  python3 "$REPO_DIR/deployment/$patch" || echo "warn: $patch failed" >&2
done
"$REPO_DIR/deployment/ensure_xr_python_requirements.sh" || echo "warn: ensure_xr_python_requirements failed" >&2

for unit in teleimager.service inspire-hands.service xr-teleop.service xr-home-watchdog.service; do
  systemctl --user enable --now "$unit" || echo "warn: enable $unit failed" >&2
done

# teleimager/inspire drive no arm motion — safe to restart every cycle.
systemctl --user restart teleimager.service || echo "warn: restart teleimager failed" >&2
systemctl --user restart inspire-hands.service || echo "warn: restart inspire-hands failed" >&2
# xr-teleop/xr-home-watchdog DO drive the arms. Only restart them when the XR
# stack actually changed (rc 10); restarting mid XR-teleoperation drops the arms.
if [ "$xr_update_rc" = "10" ]; then
  systemctl --user restart xr-teleop.service || echo "warn: restart xr-teleop failed" >&2
  systemctl --user restart xr-home-watchdog.service || echo "warn: restart xr-home-watchdog failed" >&2
fi

systemctl --user --no-pager --full status robot-telemetry-web.service teleimager.service inspire-hands.service xr-teleop.service xr-home-watchdog.service || true
exit 0
