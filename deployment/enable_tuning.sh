#!/usr/bin/env bash
# Enable a tuning file for the dashboard service on the robot (systemd user unit).
#   deployment/enable_tuning.sh deployment/tuning/round2-75-b1.json   # enable
#   deployment/enable_tuning.sh off                                    # back to shipped constants
# The service restarts; nothing else changes. Only use with a spotter at the robot.
set -euo pipefail
dropin_dir="$HOME/.config/systemd/user/robot-telemetry-web.service.d"
dropin="$dropin_dir/20-tuning.conf"
if [[ "${1:-}" == "off" ]]; then
  rm -f "$dropin"
  echo "tuning disabled (shipped constants)"
else
  file="$(readlink -f "${1:?tuning json}")"
  python3 -c "import json,sys; json.load(open(sys.argv[1]))" "$file"
  mkdir -p "$dropin_dir"
  printf '[Service]\nEnvironment=RTW_TUNING_JSON=%s\n' "$file" > "$dropin"
  echo "tuning enabled: $file"
fi
systemctl --user daemon-reload
systemctl --user restart robot-telemetry-web.service
sleep 3
systemctl --user --no-pager status robot-telemetry-web.service | grep -E "Active:|tuning" || true
journalctl --user -u robot-telemetry-web -n 5 --no-pager -o cat | grep -i "tuning" || echo "(no tuning line in the log = shipped constants)"
