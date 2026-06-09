#!/usr/bin/env python3
"""DDS smoke test for only the Inspire DFX left pinky motor.

This publishes to rt/inspire/cmd through the DFX Inspire bridge. The robot's
observed DFX command order is left hand ids 0..5, right hand ids 6..11.
Left pinky is id 0.
"""

from __future__ import annotations

import argparse
import subprocess
import time
from contextlib import suppress

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_


TOPIC_CMD = "rt/inspire/cmd"
TOPIC_STATE = "rt/inspire/state"
LEFT_PINKY_ID = 0
MOTOR_COUNT = 12


def clamp_q(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def systemctl_user(*args: str) -> None:
    subprocess.run(["systemctl", "--user", *args], check=False)


def make_cmd(left_pinky_q: float, open_q: float) -> MotorCmds_:
    msg = MotorCmds_()
    msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(MOTOR_COUNT)]
    for cmd in msg.cmds:
        cmd.q = open_q
    msg.cmds[LEFT_PINKY_ID].q = left_pinky_q
    return msg


def read_left_pinky_state(subscriber: ChannelSubscriber, timeout: float = 0.25) -> float | None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        state = subscriber.Read()
        if state is not None and len(state.states) > LEFT_PINKY_ID:
            return float(state.states[LEFT_PINKY_ID].q)
        time.sleep(0.01)
    return None


def publish_pose(
    publisher: ChannelPublisher,
    subscriber: ChannelSubscriber,
    label: str,
    left_pinky_q: float,
    open_q: float,
    hold: float,
    rate: float,
) -> None:
    msg = make_cmd(left_pinky_q=left_pinky_q, open_q=open_q)
    interval = 1.0 / rate
    deadline = time.time() + hold
    print(f"{label}: command left_pinky_id={LEFT_PINKY_ID} q={left_pinky_q:.3f}", flush=True)
    next_state_print = 0.0
    while time.time() < deadline:
        publisher.Write(msg)
        now = time.time()
        if now >= next_state_print:
            q_state = read_left_pinky_state(subscriber, timeout=0.02)
            if q_state is not None:
                print(f"  state left_pinky q={q_state:.3f}", flush=True)
            next_state_print = now + 0.5
        time.sleep(interval)


def main() -> int:
    parser = argparse.ArgumentParser(description="Move only the Inspire DFX left pinky via DDS.")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--open", type=clamp_q, default=1.0, help="Open q for all motors.")
    parser.add_argument("--close", type=clamp_q, default=0.0, help="Close q for left pinky.")
    parser.add_argument("--half", type=clamp_q, default=0.45, help="Intermediate q for left pinky.")
    parser.add_argument("--hold", type=float, default=1.2)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument(
        "--no-manage-xr",
        action="store_true",
        help="Do not stop/restart xr-teleop.service. Use only if XR is already stopped.",
    )
    parser.add_argument(
        "--keep-xr-stopped",
        action="store_true",
        help="When managing XR, leave xr-teleop.service stopped after the test.",
    )
    args = parser.parse_args()

    if args.rate <= 0:
        raise SystemExit("--rate must be positive")
    if args.hold <= 0:
        raise SystemExit("--hold must be positive")

    managed_xr = not args.no_manage_xr
    if managed_xr:
        print("Stopping xr-teleop.service so it does not overwrite the finger test.", flush=True)
        systemctl_user("stop", "xr-teleop.service")
        time.sleep(1.0)
        systemctl_user("start", "inspire-hands.service")

    try:
        ChannelFactoryInitialize(0, networkInterface=args.network_interface)
        publisher = ChannelPublisher(TOPIC_CMD, MotorCmds_)
        publisher.Init()
        subscriber = ChannelSubscriber(TOPIC_STATE, MotorStates_)
        subscriber.Init()

        print("DDS ready. Other 11 motors stay open at q={:.3f}.".format(args.open), flush=True)
        publish_pose(publisher, subscriber, "open_all", args.open, args.open, args.hold, args.rate)
        for cycle in range(args.cycles):
            print(f"cycle {cycle + 1}/{args.cycles}", flush=True)
            publish_pose(publisher, subscriber, "left_pinky_close", args.close, args.open, args.hold, args.rate)
            publish_pose(publisher, subscriber, "left_pinky_half", args.half, args.open, args.hold, args.rate)
            publish_pose(publisher, subscriber, "left_pinky_open", args.open, args.open, args.hold, args.rate)
        publish_pose(publisher, subscriber, "final_open", args.open, args.open, args.hold, args.rate)
        return 0
    finally:
        if managed_xr and not args.keep_xr_stopped:
            print("Restarting xr-teleop.service.", flush=True)
            systemctl_user("restart", "xr-teleop.service")
        with suppress(Exception):
            time.sleep(0.2)


if __name__ == "__main__":
    raise SystemExit(main())
