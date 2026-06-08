#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SYSTEMD_USER_DIR="${HOME}/.config/systemd/user"

mkdir -p "$SYSTEMD_USER_DIR" /home/unitree/logs

cp "$REPO_DIR/deployment/systemd/robot-telemetry-web.service" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/robot-telemetry-web-autoupdate.service" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/robot-telemetry-web-autoupdate.timer" "$SYSTEMD_USER_DIR/"
cp "$REPO_DIR/deployment/systemd/xr-teleop.service" "$SYSTEMD_USER_DIR/"

"$REPO_DIR/deployment/patch_xr_teleop_launcher.sh"
python3 "$REPO_DIR/deployment/patch_xr_camera_config.py"
python3 "$REPO_DIR/deployment/patch_xr_image_client.py"
python3 "$REPO_DIR/deployment/patch_xr_televuer.py"

if command -v loginctl >/dev/null 2>&1; then
  loginctl enable-linger "${USER}" 2>/dev/null || sudo -n loginctl enable-linger "${USER}" 2>/dev/null || true
fi

systemctl --user daemon-reload
systemctl --user enable --now robot-telemetry-web.service
systemctl --user enable --now robot-telemetry-web-autoupdate.timer
systemctl --user enable --now xr-teleop.service
systemctl --user restart robot-telemetry-web.service
systemctl --user restart xr-teleop.service

systemctl --user --no-pager --full status robot-telemetry-web.service xr-teleop.service
