#!/usr/bin/env python3
"""Patch dex-retargeting for torch dtype inference on Inspire hand control."""

from __future__ import annotations

from pathlib import Path


OPTIMIZERS = [
    Path(
        "/home/unitree/xr_teleoperate/teleop/robot_control/"
        "dex-retargeting/src/dex_retargeting/optimizer.py"
    ),
]

OLD = """        weight = torch.as_tensor(
            np.asarray(
                np.concatenate([weight, np.ones(self.num_fingers, dtype=np.float32) * len_proj + self.num_fingers]),
                dtype=np.float32,
            )
        )
"""

NEW = """        weight = torch.as_tensor(
            np.asarray(
                np.concatenate([weight, np.ones(self.num_fingers, dtype=np.float32) * len_proj + self.num_fingers]),
                dtype=np.float32,
            ),
            dtype=torch.float32,
        )
"""


def patch_optimizer(path: Path) -> None:
    if not path.exists():
        return

    text = path.read_text(encoding="utf-8")
    if "dtype=torch.float32,\n        )\n\n        # Compute reference distance vector" in text:
        return
    if OLD not in text:
        raise SystemExit(f"Could not find dex-retargeting weight tensor block in {path}")
    path.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")


def main() -> int:
    for path in OPTIMIZERS:
        patch_optimizer(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
