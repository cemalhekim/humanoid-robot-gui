#!/usr/bin/env python3
"""Send an H1 high-level loco SetVelocity command.

This uses Unitree's H1 `loco` RPC service. It is intended for a real H1, or a
runtime that implements that service. The current Unitree MuJoCo bridge in this
repo does not implement the `loco` RPC server, so this script will not make that
simulator walk by itself.
"""

import argparse
import sys
import time

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient
except ImportError as exc:
    print(
        "Missing Unitree SDK2 Python dependency. Run through:\n"
        "  ./scripts/run_h1_loco_set_velocity_walk.sh\n\n"
        f"Import error: {exc}",
        file=sys.stderr,
    )
    raise SystemExit(2)


def check_call(name, code, allow_timeout=False):
    if code == 0:
        print(f"{name}: ok")
        return True

    if code is None:
        print(f"{name}: ok (empty response status)")
        return True

    if allow_timeout:
        print(f"{name}: returned code {code}; continuing.")
        return False

    print(f"{name}: failed with code {code}", file=sys.stderr)
    return False


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "interface",
        nargs="?",
        default=None,
        help="Network interface for the H1 DDS connection, for example enp3s0.",
    )
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--vx", type=float, default=0.9, help="Forward velocity, m/s.")
    parser.add_argument("--vy", type=float, default=0.0, help="Lateral velocity, m/s.")
    parser.add_argument("--vyaw", type=float, default=0.0, help="Yaw velocity, rad/s.")
    parser.add_argument("--duration", type=float, default=1.5, help="Move duration, seconds.")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--standup-delay", type=float, default=3.0)
    parser.add_argument("--skip-standup", action="store_true")
    parser.add_argument("--skip-start", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.interface:
        ChannelFactoryInitialize(args.domain_id, args.interface)
    else:
        ChannelFactoryInitialize(args.domain_id)

    client = LocoClient()
    client.SetTimeout(args.timeout)
    client.Init()

    print(
        "Using H1 loco SetVelocity. Make sure the robot has clear space, "
        "is supported by its normal safety stack, and can be stopped."
    )

    if not args.skip_standup:
        if not check_call("StandUp", client.StandUp()):
            return 1
        time.sleep(args.standup_delay)

    if not args.skip_start:
        check_call("Start", client.Start(), allow_timeout=True)
        time.sleep(0.5)

    print(
        f"SetVelocity vx={args.vx:.3f}, vy={args.vy:.3f}, "
        f"vyaw={args.vyaw:.3f}, duration={args.duration:.3f}s"
    )
    if not check_call(
        "SetVelocity",
        client.SetVelocity(args.vx, args.vy, args.vyaw, args.duration),
    ):
        return 1

    time.sleep(max(0.0, args.duration))
    check_call("StopMove", client.SetVelocity(0.0, 0.0, 0.0, 1.0), allow_timeout=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
