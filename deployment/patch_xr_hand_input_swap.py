#!/usr/bin/env python3
"""Patch XR teleop to allow swapping left/right hand tracking inputs."""

from __future__ import annotations

from pathlib import Path


TARGETS = [
    Path("/home/unitree/xr_teleoperate/teleop/teleop_hand_and_arm.py"),
]

OLD = """            if args.ee in ("dex3", "inspire_ftp", "inspire_dfx", "brainco")  and args.input_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
"""

NEW = """            if args.ee in ("dex3", "inspire_ftp", "inspire_dfx", "brainco")  and args.input_mode == "hand":
                swap_hand_inputs = os.getenv("XR_SWAP_HAND_INPUTS", "0") == "1"
                left_hand_source = tele_data.right_hand_pos if swap_hand_inputs else tele_data.left_hand_pos
                right_hand_source = tele_data.left_hand_pos if swap_hand_inputs else tele_data.right_hand_pos
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = left_hand_source.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = right_hand_source.flatten()
"""


def patch_file(path: Path) -> None:
    if not path.exists():
        return

    text = path.read_text(encoding="utf-8")
    if 'swap_hand_inputs = os.getenv("XR_SWAP_HAND_INPUTS", "0") == "1"' in text:
        return
    if OLD not in text:
        raise SystemExit(f"Could not find hand input assignment block in {path}")
    path.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")


def main() -> int:
    for path in TARGETS:
        patch_file(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
