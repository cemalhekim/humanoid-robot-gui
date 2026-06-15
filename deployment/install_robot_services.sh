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
python3 "$REPO_DIR/deployment/patch_xr_vr_loco_pad.py"
python3 "$REPO_DIR/deployment/patch_xr_scene_loco_pad.py"
"$REPO_DIR/deployment/ensure_xr_python_requirements.sh"

if command -v loginctl >/dev/null 2>&1; then
  loginctl enable-linger "${USER}" 2>/dev/null || sudo -n loginctl enable-linger "${USER}" 2>/dev/null || true
fi

systemctl --user daemon-reload
systemctl --user enable --now robot-telemetry-web.service
systemctl --user enable --now robot-telemetry-web-autoupdate.timer
systemctl --user enable --now teleimager.service
systemctl --user enable --now inspire-hands.service
systemctl --user enable --now xr-teleop.service
systemctl --user enable --now xr-home-watchdog.service
systemctl --user restart robot-telemetry-web.service
systemctl --user restart teleimager.service
systemctl --user restart inspire-hands.service
systemctl --user restart xr-teleop.service
systemctl --user restart xr-home-watchdog.service

systemctl --user --no-pager --full status robot-telemetry-web.service teleimager.service inspire-hands.service xr-teleop.service xr-home-watchdog.service
