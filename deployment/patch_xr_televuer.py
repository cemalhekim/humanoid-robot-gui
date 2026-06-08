#!/usr/bin/env python3
"""Patch TeleVuer ZMQ render conversion for reliable Vision Pro rendering."""

from __future__ import annotations

from pathlib import Path


TELEVUER = Path("/home/unitree/xr_teleoperate/teleop/televuer/src/televuer/televuer.py")

OLD = """            latest_frame = self.latest_frame
            latest_frame = cv2.cvtColor(latest_frame, cv2.COLOR_BGR2RGB)
            self.img2display[:] = latest_frame
"""

NEW = """            latest_frame = np.ascontiguousarray(self.latest_frame[:, :, ::-1])
            self.img2display[:] = latest_frame
"""


def main() -> int:
    if not TELEVUER.exists():
        return 0
    text = TELEVUER.read_text(encoding="utf-8")
    if NEW in text:
        return 0
    if OLD not in text:
        raise SystemExit("Could not find TeleVuer OpenCV render conversion block")
    TELEVUER.write_text(text.replace(OLD, NEW), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
