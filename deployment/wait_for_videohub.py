#!/usr/bin/env python3
"""Block until the robot's built-in head camera (videohub) answers.

Used as an ExecStartPre for teleimager.service. The image server initializes
its VideoHubCamera exactly ONCE at startup; on a cold boot the MCU's video
service comes up later than this PC's user services, so that single attempt
failed and teleimager ran degraded until someone restarted it by hand (the
"camera does not work after reboot" symptom). Waiting here until
VideoClient.GetImageSample actually returns a frame makes the camera come up
by itself on every boot. Exits 0 after the timeout as well — starting anyway
is better than blocking the unit forever, and Restart=always retries.
"""

from __future__ import annotations

import os
import sys
import time

TIMEOUT_SECONDS = float(os.environ.get("VIDEOHUB_WAIT_TIMEOUT", "180"))
INTERFACE = os.environ.get("VIDEOHUB_INTERFACE", "eth0")


def main() -> int:
    deadline = time.monotonic() + TIMEOUT_SECONDS
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.video.video_client import VideoClient

            ChannelFactoryInitialize(0, INTERFACE)
            client = VideoClient()
            client.SetTimeout(3.0)
            client.Init()
            code, data = client.GetImageSample()
            if code == 0 and data:
                print(f"videohub ready after {attempt} attempt(s): {len(data)} bytes")
                return 0
            print(f"attempt {attempt}: GetImageSample code={code}, retrying")
        except Exception as exc:  # noqa: BLE001 - DDS init can fail many ways at boot
            print(f"attempt {attempt}: {exc}")
        time.sleep(5.0)
    print(f"videohub not ready after {TIMEOUT_SECONDS:.0f}s; starting anyway", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
