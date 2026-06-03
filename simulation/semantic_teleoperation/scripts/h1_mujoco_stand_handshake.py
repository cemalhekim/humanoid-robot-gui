#!/usr/bin/env python3
"""H1 MuJoCo demo: hold a stable standing pose, then perform a handshake motion.

This talks to Unitree MuJoCo through Unitree SDK2 Python DDS topics. It is meant
for simulation only and publishes low-level position commands on rt/lowcmd.
"""

import argparse
import sys

from h1_mujoco_motion import H1DemoController


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain-id", type=int, default=1)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--rate", type=float, default=200.0)
    parser.add_argument("--stand-seconds", type=float, default=5.0)
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument(
        "--final-hold-seconds",
        type=float,
        default=0.0,
        help="Seconds to hold the final home pose. Use 0 or less to hold forever.",
    )
    parser.add_argument("--wait-timeout", type=float, default=12.0)
    return parser.parse_args()


def main():
    args = parse_args()
    controller = H1DemoController(args.domain_id, args.interface, args.rate)
    if not controller.wait_for_simulator(args.wait_timeout):
        print(
            "No MuJoCo lowstate received. Start the H1 simulator first:\n"
            "  ./scripts/run_unitree_mujoco_h1.sh",
            file=sys.stderr,
        )
        return 1

    controller.run_handshake(args.stand_seconds, args.cycles, args.final_hold_seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
