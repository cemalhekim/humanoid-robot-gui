#!/usr/bin/env python3
"""Publish the current cloudflared quick-tunnel URLs for the welcome page.

Runs periodically on the operator Mac (LaunchAgent
com.vodafone.robot-dashboard-tunnel-url). Two quick tunnels run on the Mac:

- "url" -> the read-only mirror of the Mac's dashboard copy
           (com.vodafone.robot-dashboard-cloudflared, welcome "Offline" card —
           works even with the robot switched off).

Each quick tunnel gets a fresh https://<random>.trycloudflare.com hostname
whenever cloudflared restarts, so this script:

1. extracts the newest tunnel URL of each from the cloudflared logs,
2. verifies the tunnel edge actually answers,
3. writes them to static/remote-entrance.json, and
4. commits + pushes when (and only when) a URL changed, so the copy served
   from GitHub (raw + Pages) and the robot stays current.

The welcome page reads the JSON same-origin first, then falls back to the raw
GitHub copy — which is reachable even when both the robot and the Mac are off.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
TARGET = REPO / "static" / "remote-entrance.json"
LOG_DIR = Path("/tmp/robot-dashboard-remote")
# SECURITY: only the READ-ONLY mirror is ever published. A second "live relay"
# tunnel straight to the robot's dashboard was removed on operator request —
# it exposed unauthenticated robot control to the public internet.
TUNNELS: dict[str, dict] = {
    "url": {
        "logs": (LOG_DIR / "cloudflared.err.log", LOG_DIR / "cloudflared.log"),
        "check_path": "/welcome",
    },
}
URL_PATTERN = re.compile(r"https://[a-z0-9-]+\.trycloudflare\.com")
NOTE = (
    "Auto-updated by tools/update_remote_entrance.py on the operator Mac; "
    "quick-tunnel hostnames change when cloudflared restarts. url = read-only "
    "mirror (Offline card). No live-control tunnel is published, by design."
)


def newest_tunnel_url(logs: tuple[Path, ...]) -> str | None:
    best: tuple[float, str] | None = None
    for log in logs:
        try:
            text = log.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        matches = URL_PATTERN.findall(text)
        if matches:
            mtime = log.stat().st_mtime
            if best is None or mtime > best[0]:
                best = (mtime, matches[-1])
    return best[1] if best else None


def tunnel_answers(url: str, check_path: str) -> bool:
    try:
        request = urllib.request.Request(url + check_path, method="GET")
        with urllib.request.urlopen(request, timeout=10):
            return True
    except urllib.error.HTTPError:
        # Any HTTP status — even the 502 cloudflared serves while the robot is
        # switched off — means the tunnel edge itself is up and routable.
        return True
    except Exception:
        return False


def current_published() -> dict:
    try:
        data = json.loads(TARGET.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def main() -> int:
    published = current_published()
    changed: list[str] = []
    for key, spec in TUNNELS.items():
        url = newest_tunnel_url(spec["logs"])
        if not url:
            print(f"{key}: no tunnel URL found in cloudflared logs")
            continue
        if url == published.get(key):
            print(f"{key}: tunnel URL unchanged ({url})")
            continue
        if not tunnel_answers(url, spec["check_path"]):
            print(f"{key}: tunnel {url} does not answer yet; will retry next run")
            continue
        published[key] = url
        changed.append(f"{key} -> {url}")

    if not changed:
        return 0

    TARGET.write_text(
        json.dumps(
            {
                "url": published.get("url"),
                "updated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                "note": NOTE,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    def run(*args: str) -> subprocess.CompletedProcess:
        return subprocess.run(args, cwd=REPO, capture_output=True, text=True, check=False)

    run("git", "add", str(TARGET))
    commit = run(
        "git", "commit", "-m",
        "Publish new remote tunnel URL for the welcome page\n\n"
        + "\n".join(changed)
        + "\n\nCo-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>",
    )
    if commit.returncode != 0:
        print(f"nothing committed: {commit.stdout} {commit.stderr}")
        return 0
    push = run("git", "push")
    print(f"published {', '.join(changed)}; push rc={push.returncode} {push.stderr.strip()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
