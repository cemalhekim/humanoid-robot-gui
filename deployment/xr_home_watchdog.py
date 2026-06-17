#!/usr/bin/env python3
"""Send robot home if an active XR teleop session disconnects."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
import urllib.error
import urllib.request
from typing import Any


def established_xr_clients(port: int) -> list[str]:
    try:
        result = subprocess.run(
            ["ss", "-Htan"],
            capture_output=True,
            check=False,
            text=True,
            timeout=1.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    clients: list[str] = []
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) < 5 or fields[0] != "ESTAB":
            continue
        local = fields[3]
        peer = fields[4]
        if local.endswith(f":{port}"):
            clients.append(peer.rsplit(":", 1)[0])
    return sorted(set(clients))


class IpcStatus:
    def __init__(self) -> None:
        try:
            from teleop.utils.ipc import IPC_Client
        except Exception as exc:
            self.client = None
            self.error = f"IPC client unavailable: {exc}"
            return
        self.client = IPC_Client(hb_fps=10.0)
        self.error = None

    def snapshot(self) -> dict[str, Any]:
        if self.client is None:
            return {"online": False, "state": {}, "error": self.error}
        return {
            "online": bool(self.client.is_online()),
            "state": self.client.latest_state(),
            "error": None,
        }

    def close(self) -> None:
        if self.client is not None:
            self.client.stop()


def post_home(url: str) -> tuple[bool, str]:
    request = urllib.request.Request(url, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=8.0) as response:
            body = response.read().decode("utf-8", errors="replace")
            return 200 <= response.status < 300, body
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        return False, body or str(exc)
    except Exception as exc:
        return False, str(exc)


def post_json(url: str, payload: dict[str, Any]) -> tuple[bool, str]:
    request = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=2.0) as response:
            body = response.read().decode("utf-8", errors="replace")
            return 200 <= response.status < 300, body
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        return False, body or str(exc)
    except Exception as exc:
        return False, str(exc)


def main() -> int:
    parser = argparse.ArgumentParser(description="Watch XR teleop connection and request home on disconnect.")
    parser.add_argument("--xr-port", type=int, default=8012)
    parser.add_argument("--home-url", default="http://127.0.0.1:8088/api/robot/home")
    parser.add_argument("--loco-url", default="http://127.0.0.1:8088/api/loco/command")
    parser.add_argument("--poll-seconds", type=float, default=0.5)
    parser.add_argument("--lost-seconds", type=float, default=5.0)
    args = parser.parse_args()

    ipc = IpcStatus()
    armed = False
    lost_since: float | None = None
    last_loco_stop_at = 0.0
    last_log = 0.0
    print(
        f"XR home watchdog listening on port {args.xr_port}; "
        f"lost threshold {args.lost_seconds:.1f}s.",
        flush=True,
    )
    try:
        while True:
            now = time.monotonic()
            clients = established_xr_clients(args.xr_port)
            ipc_state = ipc.snapshot()
            tracking = bool((ipc_state.get("state") or {}).get("START"))
            active = bool(clients) and bool(ipc_state.get("online")) and tracking

            if active:
                if not armed:
                    print(f"XR active; watchdog armed. clients={clients}", flush=True)
                armed = True
                lost_since = None
            elif armed:
                if lost_since is None:
                    lost_since = now
                    print("XR active session lost; starting home countdown.", flush=True)
                if now - last_loco_stop_at >= 0.5:
                    ok, body = post_json(args.loco_url, {"action": "stop_move"})
                    print(
                        "Loco stop requested after XR disconnect: "
                        + json.dumps({"ok": ok, "response": body[:300]}),
                        flush=True,
                    )
                    last_loco_stop_at = now
                if now - lost_since >= args.lost_seconds:
                    ok, body = post_home(args.home_url)
                    print(
                        "Home requested after XR disconnect: "
                        + json.dumps({"ok": ok, "response": body[:500]}),
                        flush=True,
                    )
                    armed = False
                    lost_since = None

            if now - last_log >= 10.0:
                print(
                    json.dumps(
                        {
                            "armed": armed,
                            "active": active,
                            "clients": clients,
                            "ipc_online": ipc_state.get("online"),
                            "tracking": tracking,
                            "lost_for": None if lost_since is None else round(now - lost_since, 2),
                        },
                        separators=(",", ":"),
                    ),
                    flush=True,
                )
                last_log = now

            time.sleep(max(0.1, args.poll_seconds))
    finally:
        ipc.close()


if __name__ == "__main__":
    raise SystemExit(main())
