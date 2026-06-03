#!/usr/bin/env python3
"""Small direct serial smoke test for Inspire RH56 hands.

This uses correlllab/rh56_controller's RH56Hand class without ROS. It reads both
hand IDs, opens them, makes a small close/open move, then leaves them open.
"""

import argparse
import sys
import time
from pathlib import Path


def clamp(value: int) -> int:
    return max(0, min(1000, int(value)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", default="/home/unitree/rh56_controller")
    parser.add_argument("--port", default="/dev/serial/by-id/usb-unitree.CN_USB_Quad_Serial_BC49B3ABCD-if00-port0")
    parser.add_argument("--hands", default="1,2", help="Comma separated hand IDs.")
    parser.add_argument("--open", type=int, default=1000)
    parser.add_argument("--close", type=int, default=750, help="Conservative partial close target.")
    parser.add_argument("--speed", type=int, default=200)
    parser.add_argument("--force", type=int, default=500)
    parser.add_argument("--hold", type=float, default=0.7)
    args = parser.parse_args()

    repo = Path(args.repo)
    sys.path.insert(0, str(repo))
    from rh56_controller.rh56_hand import RH56Hand

    open_pos = [clamp(args.open)] * 6
    close_pos = [clamp(args.close)] * 6
    speed = [clamp(args.speed)] * 6
    force = [clamp(args.force)] * 6
    hand_ids = [int(x.strip()) for x in args.hands.split(",") if x.strip()]

    hands = []
    try:
        for hand_id in hand_ids:
            hand = RH56Hand(port=args.port, hand_id=hand_id)
            hands.append((hand_id, hand))
            print(f"hand {hand_id}: connected on {args.port}", flush=True)
            print(f"hand {hand_id}: angles={hand.angle_read()} forces={hand.force_act()}", flush=True)
            hand.clear_errors()
            hand.speed_set(speed)
            hand.force_set(force)

        for label, target in (("open", open_pos), ("partial_close", close_pos), ("open", open_pos)):
            print(f"command: {label} {target}", flush=True)
            for _, hand in hands:
                resp = hand.angle_set(target)
                print(f"  response={'ok' if resp is not None else 'none'}", flush=True)
            time.sleep(args.hold)
            for hand_id, hand in hands:
                print(f"  hand {hand_id}: angles={hand.angle_read()} forces={hand.force_act()}", flush=True)

        return 0
    finally:
        for _, hand in hands:
            try:
                hand.ser.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
