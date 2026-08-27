#!/usr/bin/env python3
"""Read or set the H1-2 loco FSM id directly through the Unitree LocoClient.

The dashboard's "Stand Up" sends SetFsmId(2) (H1 semantics). On the H1-2 the loco
service is of the G1/"hg" family where 2 = squat and 4 = stand up / locked stand,
so FSM 2 is ignored from Damp (observed 2026-08-27). This tool lets you query the
FSM and set an explicit id while someone supports the robot.

    python3 loco_fsm_probe.py eth0                 # read fsm id + mode
    python3 loco_fsm_probe.py eth0 --set 4 --i-understand-risk   # e.g. stand up

Run ON the robot PC (unitree@192.168.123.164, micromamba env tv) or from a machine
on the 192.168.123.x link with unitree_sdk2py on PYTHONPATH.
"""
import argparse
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("interface", help="network interface on the robot link, e.g. eth0")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--set", type=int, default=None, help="FSM id to set (0 zero torque, 1 damp, 2 squat?, 3 sit?, 4 stand up/locked stand?, 200/500 start?)")
    ap.add_argument("--i-understand-risk", action="store_true")
    ap.add_argument("--wait", type=float, default=3.0, help="seconds to wait before re-reading the FSM after --set")
    args = ap.parse_args()

    ChannelFactoryInitialize(args.domain, args.interface)
    client = LocoClient()
    client.SetTimeout(5.0)
    client.Init()

    def read(label: str) -> None:
        code, fsm = client.GetFsmId()
        mcode, mode = client.GetFsmMode()
        print(f"{label}: fsm_id={fsm} (code {code}) fsm_mode={mode} (code {mcode})", flush=True)

    read("before")
    if args.set is None:
        return 0
    if not args.i_understand_risk:
        print("refusing to change the FSM without --i-understand-risk (the robot will move)", file=sys.stderr)
        return 2
    code = client.SetFsmId(args.set)
    print(f"SetFsmId({args.set}) -> code {code}", flush=True)
    time.sleep(args.wait)
    read("after")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
