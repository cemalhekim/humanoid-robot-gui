#!/usr/bin/env python3
"""Publish the current cloudflared quick-tunnel URL for the welcome page.

Runs periodically on the operator Mac (LaunchAgent
com.vodafone.robot-dashboard-tunnel-url). The quick tunnel gets a fresh
https://<random>.trycloudflare.com hostname whenever cloudflared restarts, so
this script:

1. extracts the newest tunnel URL from the cloudflared logs,
2. verifies the tunnel actually answers,
3. writes it to static/remote-entrance.json, and
4. commits + pushes when (and only when) the URL changed, so the copy served
   from GitHub (raw + Pages) and the robot stays current.

The welcome page reads the JSON same-origin first, then falls back to the raw
GitHub copy — which is reachable even when both the robot and the Mac are off.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
TARGET = REPO / "static" / "remote-entrance.json"
LOG_CANDIDATES = (
    Path("/tmp/robot-dashboard-remote/cloudflared.err.log"),
    Path("/tmp/robot-dashboard-remote/cloudflared.log"),
)
URL_PATTERN = re.compile(r"https://[a-z0-9-]+\.trycloudflare\.com")


def newest_tunnel_url() -> str | None:
    best: tuple[float, str] | None = None
    for log in LOG_CANDIDATES:
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


def tunnel_answers(url: str) -> bool:
    try:
        request = urllib.request.Request(url + "/welcome", method="GET")
        with urllib.request.urlopen(request, timeout=10) as response:
            return response.status < 500
    except Exception:
        return False


def current_published_url() -> str | None:
    try:
        return json.loads(TARGET.read_text(encoding="utf-8")).get("url")
    except Exception:
        return None


def main() -> int:
    url = newest_tunnel_url()
    if not url:
        print("no tunnel URL found in cloudflared logs; nothing to do")
        return 0
    if url == current_published_url():
        print(f"tunnel URL unchanged ({url})")
        return 0
    if not tunnel_answers(url):
        print(f"tunnel {url} does not answer yet; will retry next run")
        return 0

    TARGET.write_text(
        json.dumps(
            {
                "url": url,
                "updated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                "note": (
                    "Auto-updated by tools/update_remote_entrance.py on the operator Mac; "
                    "the quick-tunnel hostname changes when cloudflared restarts."
                ),
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
        f"Publish new remote tunnel URL for the welcome page\n\n{url}\n\n"
        "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>",
    )
    if commit.returncode != 0:
        print(f"nothing committed: {commit.stdout} {commit.stderr}")
        return 0
    push = run("git", "push")
    print(f"published {url}; push rc={push.returncode} {push.stderr.strip()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
