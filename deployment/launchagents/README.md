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

The offline dashboard copy runs on the M1 mini (`arm-mmini`) as the Docker
service `hans-dashboard` (offline, `--disable-camera`) — compose file:
`~/Workspace/cemos/integrations/robotics-server/docker-compose.yml`. It is
**local-only**: loopback `127.0.0.1:8088` plus tailnet HTTPS via
`tailscale serve`. The public cloudflared quick tunnel, the read-only proxy
and the tunnel-url LaunchAgent that briefly replicated the Intel-Mac chain
here were removed the same day on operator request;
`static/remote-entrance.json` publishes `null` accordingly.

The `com.vodafone.*` plists remain the retired Intel-Mac versions.
