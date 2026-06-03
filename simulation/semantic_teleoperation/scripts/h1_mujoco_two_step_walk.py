#!/usr/bin/env python3
"""H1 MuJoCo demo: take two conservative scripted steps, then hold home.

This is a simulation-only low-level position demo. It is not a dynamic walking
controller; it relies on the MuJoCo H1 elastic band and slow quasi-static moves.
"""

import argparse
import sys

from h1_mujoco_motion import H1MotionController, HOME_POSE, NUM_MOTORS


class H1TwoStepWalkController(H1MotionController):
    def __init__(self, domain_id, interface, rate_hz, speed_scale=1.0):
        super().__init__(domain_id, interface, rate_hz)
        self.speed_scale = speed_scale

    def run(self, final_hold_seconds):
        self.walk_steps(2, initial_side="right", speed_scale=self.speed_scale)
        duration = None if final_hold_seconds <= 0.0 else final_hold_seconds
        self.hold(HOME_POSE, duration, "Two-step walk complete, holding home pose.")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain-id", type=int, default=1)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--rate", type=float, default=200.0)
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=1.35,
        help="Multiplier for scripted step speed; values above 1 move faster.",
    )
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
    if NUM_MOTORS != 20:
        print("This demo expects the 20-motor H1 MuJoCo model.", file=sys.stderr)
        return 2

    controller = H1TwoStepWalkController(
        args.domain_id,
        args.interface,
        args.rate,
        speed_scale=args.speed_scale,
    )
    if not controller.wait_for_simulator(args.wait_timeout):
        print(
            "No MuJoCo lowstate received. Start the H1 simulator first:\n"
            "  ./scripts/run_unitree_mujoco_h1.sh",
            file=sys.stderr,
        )
        return 1

    controller.run(args.final_hold_seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
