#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SYSTEMD_USER_DIR="${HOME}/.config/systemd/user"

mkdir -p "$SYSTEMD_USER_DIR" /home/unitree/logs

xr_update_rc=0
"$REPO_DIR/deployment/update_xr_teleoperate.sh" || xr_update_rc=$?
if [[ "$xr_update_rc" != "0" && "$xr_update_rc" != "10" ]]; then
  exit "$xr_update_rc"
fi

cp "$REPO_DIR/deployment/systemd/robot-telemetry-web.service" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/robot-telemetry-web-autoupdate.service" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/robot-telemetry-web-autoupdate.timer" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/teleimager.service" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/inspire-hands.service" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/xr-teleop.service" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/xr-home-watchdog.service" "$SYSTEMD_USER_DIR/"

"$REPO_DIR/deployment/patch_xr_teleop_launcher.sh"
python3 "$REPO_DIR/deployment/patch_xr_camera_config.py"
python3 "$REPO_DIR/deployment/patch_xr_image_server.py"
python3 "$REPO_DIR/deployment/patch_xr_dex_retargeting.py"
python3 "$REPO_DIR/deployment/patch_xr_inspire_direct_curl.py"
python3 "$REPO_DIR/deployment/patch_xr_hand_input_swap.py"
python3 "$REPO_DIR/deployment/patch_xr_head_tilt_loco.py"
python3 "$REPO_DIR/deployment/patch_xr_root_children_visual.py"
python3 "$REPO_DIR/deployment/patch_vuer_loco_pointer_events.py"
"$REPO_DIR/deployment/ensure_xr_python_requirements.sh"

if command -v loginctl >/dev/null 2>&1; then
  loginctl enable-linger "${USER}" 2>/dev/null || sudo -n loginctl enable-linger "${USER}" 2>/dev/null || true
fi

# SAFETY: never restart the dashboard while it is actively commanding the
# robot (arm replay or person tracking) — killing the publisher mid-motion
# drops the arms (operator-reported incident 2026-07-23). Wait bounded, then
# proceed so a wedged flag cannot block updates forever.
for attempt in $(seq 1 60); do
  motion="$(curl -s -m 2 http://localhost:8088/api/motion/active 2>/dev/null | grep -o '"active": *true' || true)"
  if [ -z "$motion" ]; then
    break
  fi
  echo "motion active; delaying dashboard restart (${attempt}/60)..."
  sleep 5
done

systemctl --user daemon-reload
# The dashboard is the one service this script must deliver — keep it strict.
systemctl --user enable --now robot-telemetry-web.service
systemctl --user enable --now robot-telemetry-web-autoupdate.timer
systemctl --user restart robot-telemetry-web.service
# Auxiliary services are best-effort: a canceled/hanging xr-teleop restart used
# to abort the whole install (set -e) BEFORE the installed-commit marker was
# written, so every later tick reinstalled and re-restarted the dashboard.
for unit in teleimager.service inspire-hands.service xr-teleop.service xr-home-watchdog.service; do
  systemctl --user enable --now "$unit" || echo "warn: enable $unit failed"
  systemctl --user restart "$unit" || echo "warn: restart $unit failed"
done

systemctl --user --no-pager --full status robot-telemetry-web.service teleimager.service inspire-hands.service xr-teleop.service xr-home-watchdog.service || true
