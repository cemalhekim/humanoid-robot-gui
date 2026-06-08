#!/usr/bin/env python3
"""Patch TeleVuer ZMQ render conversion for reliable Vision Pro rendering."""

from __future__ import annotations

from pathlib import Path


TELEVUER = Path("/home/unitree/xr_teleoperate/teleop/televuer/src/televuer/televuer.py")

OLD_BLOCKS = [
    """            latest_frame = self.latest_frame
            latest_frame = cv2.cvtColor(latest_frame, cv2.COLOR_BGR2RGB)
            self.img2display[:] = latest_frame
""",
    """            latest_frame = np.ascontiguousarray(self.latest_frame[:, :, ::-1])
            self.img2display[:] = latest_frame
""",
]

NEW = """            latest_frame = self.latest_frame
            target_height, target_width = self.img_shape[:2]
            if latest_frame.shape[0] != target_height or latest_frame.shape[1] != target_width:
                if latest_frame.shape[0] % target_height == 0 and latest_frame.shape[1] % target_width == 0:
                    row_step = latest_frame.shape[0] // target_height
                    col_step = latest_frame.shape[1] // target_width
                    latest_frame = latest_frame[::row_step, ::col_step][:target_height, :target_width]
                else:
                    row_idx = np.linspace(0, latest_frame.shape[0] - 1, target_height).astype(np.intp)
                    col_idx = np.linspace(0, latest_frame.shape[1] - 1, target_width).astype(np.intp)
                    latest_frame = latest_frame[row_idx][:, col_idx]
            latest_frame = np.ascontiguousarray(latest_frame[:, :, ::-1])
            self.img2display[:] = latest_frame
"""


def main() -> int:
    if not TELEVUER.exists():
        return 0
    text = TELEVUER.read_text(encoding="utf-8")
    if NEW in text:
        return 0
    for old in OLD_BLOCKS:
        if old in text:
            TELEVUER.write_text(text.replace(old, NEW), encoding="utf-8")
            return 0
    raise SystemExit("Could not find TeleVuer render conversion block")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
