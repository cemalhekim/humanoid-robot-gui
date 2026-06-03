#!/usr/bin/env python3
"""H1 MuJoCo demo: perform a simple scripted jump from home pose."""

import argparse
import sys

from h1_mujoco_motion import H1MotionController, HOME_POSE


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain-id", type=int, default=1)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--rate", type=float, default=200.0)
    parser.add_argument(
        "--height",
        type=float,
        default=0.12,
        help="Approximate torso lift magnitude for the jump.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.5,
        help="Total duration of the jump sequence, seconds.",
    )
    parser.add_argument(
        "--final-hold-seconds",
        type=float,
        default=1.0,
        help="Seconds to hold the final home pose after the jump.",
    )
    parser.add_argument("--wait-timeout", type=float, default=12.0)
    return parser.parse_args()


def main():
    args = parse_args()
    controller = H1MotionController(args.domain_id, args.interface, args.rate)
    if not controller.wait_for_simulator(args.wait_timeout):
        print(
            "No MuJoCo lowstate received. Start the H1 simulator first:\n"
            "  ./scripts/run_unitree_mujoco_h1.sh",
            file=sys.stderr,
        )
        return 1

    controller.jump(args.height, args.duration)
    if args.final_hold_seconds > 0.0:
        controller.hold(
            HOME_POSE,
            args.final_hold_seconds,
            "Final home pose hold after jump.",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
