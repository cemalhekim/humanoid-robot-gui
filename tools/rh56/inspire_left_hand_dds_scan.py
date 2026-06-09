#!/usr/bin/env python3
"""DDS scan for each observed left-hand Inspire DFX motor.

This robot's observed DFX command order is left hand ids 0..5, right hand
ids 6..11. Each step keeps the other 11 motors open and moves one left-hand
motor through close, half, open.
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
MOTOR_COUNT = 12
LEFT_HAND_MOTORS = [
    (0, "left_pinky"),
    (1, "left_ring"),
    (2, "left_middle"),
    (3, "left_index"),
    (4, "left_thumb_bend"),
    (5, "left_thumb_rotation"),
]


def clamp_q(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def systemctl_user(*args: str) -> None:
    subprocess.run(["systemctl", "--user", *args], check=False)


def make_cmd(target_id: int, target_q: float, open_q: float) -> MotorCmds_:
    msg = MotorCmds_()
    msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(MOTOR_COUNT)]
    for cmd in msg.cmds:
        cmd.q = open_q
    msg.cmds[target_id].q = target_q
    return msg


def read_state(subscriber: ChannelSubscriber, motor_id: int, timeout: float = 0.25) -> float | None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        state = subscriber.Read()
        if state is not None and len(state.states) > motor_id:
            return float(state.states[motor_id].q)
        time.sleep(0.01)
    return None


def publish_pose(
    publisher: ChannelPublisher,
    subscriber: ChannelSubscriber,
    label: str,
    motor_id: int,
    target_q: float,
    open_q: float,
    hold: float,
    rate: float,
) -> None:
    msg = make_cmd(target_id=motor_id, target_q=target_q, open_q=open_q)
    interval = 1.0 / rate
    deadline = time.time() + hold
    print(f"{label}: command id={motor_id} q={target_q:.3f}", flush=True)
    next_state_print = 0.0
    while time.time() < deadline:
        publisher.Write(msg)
        now = time.time()
        if now >= next_state_print:
            q_state = read_state(subscriber, motor_id, timeout=0.02)
            if q_state is not None:
                print(f"  state id={motor_id} q={q_state:.3f}", flush=True)
            next_state_print = now + 0.5
        time.sleep(interval)


def main() -> int:
    parser = argparse.ArgumentParser(description="Move each observed left Inspire DFX finger one at a time.")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--open", type=clamp_q, default=1.0, help="Open q for all non-target motors.")
    parser.add_argument("--close", type=clamp_q, default=0.0, help="Close q for the target motor.")
    parser.add_argument("--half", type=clamp_q, default=0.45, help="Intermediate q for the target motor.")
    parser.add_argument("--hold", type=float, default=1.4)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument(
        "--ids",
        default="0,1,2,3,4,5",
        help="Comma-separated motor ids to test. Default tests observed left hand ids 0..5.",
    )
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

    names_by_id = dict(LEFT_HAND_MOTORS)
    try:
        motor_ids = [int(part.strip()) for part in args.ids.split(",") if part.strip()]
    except ValueError as exc:
        raise SystemExit("--ids must contain comma-separated integers") from exc
    for motor_id in motor_ids:
        if not 0 <= motor_id < MOTOR_COUNT:
            raise SystemExit(f"motor id out of range: {motor_id}")

    managed_xr = not args.no_manage_xr
    if managed_xr:
        print("Stopping xr-teleop.service so it does not overwrite the finger scan.", flush=True)
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
        for motor_id in motor_ids:
            name = names_by_id.get(motor_id, f"motor_{motor_id}")
            print(f"\n=== Testing {name} id={motor_id} ===", flush=True)
            publish_pose(publisher, subscriber, f"{name}_open_all", motor_id, args.open, args.open, args.hold, args.rate)
            publish_pose(publisher, subscriber, f"{name}_close", motor_id, args.close, args.open, args.hold, args.rate)
            publish_pose(publisher, subscriber, f"{name}_half", motor_id, args.half, args.open, args.hold, args.rate)
            publish_pose(publisher, subscriber, f"{name}_open", motor_id, args.open, args.open, args.hold, args.rate)
        print("\nFinal open command.", flush=True)
        for motor_id in motor_ids:
            publish_pose(publisher, subscriber, "final_open", motor_id, args.open, args.open, 0.3, args.rate)
        return 0
    finally:
        if managed_xr and not args.keep_xr_stopped:
            print("Restarting xr-teleop.service.", flush=True)
            systemctl_user("restart", "xr-teleop.service")
        with suppress(Exception):
            time.sleep(0.2)


if __name__ == "__main__":
    raise SystemExit(main())
