# Robot Dashboard LaunchAgents

This directory stores the LaunchAgent definitions used for the local robot
dashboard services.

The active macOS LaunchAgent files remain in:

```text
~/Library/LaunchAgents/
```

They were copied here for versioned backup and deployment reference so the
running local services are not disrupted by repository organization changes.

## arm-mmini offline mirror (since 2026-09-11)

The public read-only mirror moved from the Intel operator Mac to the M1 mini
(`arm-mmini`). Same chain, different runtimes:

- dashboard (offline, `--disable-camera`), read-only proxy and the cloudflared
  quick tunnel run as Docker services `hans-dashboard` / `hans-readonly-proxy` /
  `hans-cloudflared` — compose file:
  `~/Workspace/cemos/integrations/robotics-server/docker-compose.yml`.
- `com.cemalhekim.hans-tunnel-url.plist` (this directory) is the only
  LaunchAgent: every 5 min it runs `tools/update_remote_entrance.py` with
  `HANS_TUNNEL_LOG_DIR=~/Library/Logs/hans-tunnel` (cloudflared's `--logfile`
  dir, bind-mounted out of the container) and pushes a changed URL to
  `static/remote-entrance.json`.

The `com.vodafone.*` plists remain the retired Intel-Mac versions.
