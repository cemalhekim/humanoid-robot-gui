#!/usr/bin/env python3
"""Direct DDS test for Inspire DFX hand motors.

Publishes MotorCmds_ to rt/inspire/cmd and reads MotorStates_ from
rt/inspire/state. By default it stops xr-teleop.service during the test so XR
does not overwrite the commanded finger positions.

Observed command order on this robot:
  0..5   left pinky, ring, middle, index, thumb bend, thumb rotation
  6..11  right pinky, ring, middle, index, thumb bend, thumb rotation

The Inspire DFX q convention used here is 1.0=open and 0.0=closed.
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
MOTOR_NAMES = {
    0: "left_pinky",
    1: "left_ring",
    2: "left_middle",
    3: "left_index",
    4: "left_thumb_bend",
    5: "left_thumb_rotation",
    6: "right_pinky",
    7: "right_ring",
    8: "right_middle",
    9: "right_index",
    10: "right_thumb_bend",
    11: "right_thumb_rotation",
}


def clamp_q(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def systemctl_user(*args: str) -> None:
    subprocess.run(["systemctl", "--user", *args], check=False)


def parse_ids(raw: str) -> list[int]:
    ids = [int(part.strip()) for part in raw.split(",") if part.strip()]
    for motor_id in ids:
        if motor_id < 0 or motor_id >= MOTOR_COUNT:
            raise argparse.ArgumentTypeError(f"motor id out of range: {motor_id}")
    return ids


def make_cmd(targets: dict[int, float], open_q: float) -> MotorCmds_:
    msg = MotorCmds_()
    msg.cmds = [unitree_go_msg_dds__MotorCmd_() for _ in range(MOTOR_COUNT)]
    for motor_id, cmd in enumerate(msg.cmds):
        cmd.q = clamp_q(targets.get(motor_id, open_q))
    return msg


def read_state(subscriber: ChannelSubscriber, timeout: float = 0.2) -> list[float] | None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        state = subscriber.Read()
        if state is not None and len(state.states) >= MOTOR_COUNT:
            return [float(state.states[i].q) for i in range(MOTOR_COUNT)]
        time.sleep(0.01)
    return None


def print_state(values: list[float] | None, ids: list[int]) -> None:
    if values is None:
        print("  state: no sample", flush=True)
        return
    parts = [f"{MOTOR_NAMES[i]}({i})={values[i]:.3f}" for i in ids]
    print("  state: " + " ".join(parts), flush=True)


def publish_pose(
    publisher: ChannelPublisher,
    subscriber: ChannelSubscriber,
    label: str,
    targets: dict[int, float],
    observe_ids: list[int],
    open_q: float,
    hold: float,
    rate: float,
) -> None:
    msg = make_cmd(targets=targets, open_q=open_q)
    interval = 1.0 / rate
    deadline = time.time() + hold
    target_text = ", ".join(f"{MOTOR_NAMES[i]}({i})={q:.3f}" for i, q in sorted(targets.items()))
    print(f"\n{label}: {target_text or f'all open q={open_q:.3f}'}", flush=True)
    next_state_print = 0.0
    while time.time() < deadline:
        publisher.Write(msg)
        now = time.time()
        if now >= next_state_print:
            print_state(read_state(subscriber, timeout=0.02), observe_ids)
            next_state_print = now + 0.5
        time.sleep(interval)


def run_scan(
    publisher: ChannelPublisher,
    subscriber: ChannelSubscriber,
    ids: list[int],
    open_q: float,
    close_q: float,
    half_q: float,
    hold: float,
    rate: float,
) -> None:
    publish_pose(publisher, subscriber, "open_all", {}, ids, open_q, hold, rate)
    for motor_id in ids:
        name = MOTOR_NAMES[motor_id]
        publish_pose(publisher, subscriber, f"{name}_close", {motor_id: close_q}, ids, open_q, hold, rate)
        publish_pose(publisher, subscriber, f"{name}_half", {motor_id: half_q}, ids, open_q, hold, rate)
        publish_pose(publisher, subscriber, f"{name}_open", {motor_id: open_q}, ids, open_q, hold, rate)
    publish_pose(publisher, subscriber, "final_open_all", {}, ids, open_q, hold, rate)


def run_all(
    publisher: ChannelPublisher,
    subscriber: ChannelSubscriber,
    ids: list[int],
    open_q: float,
    close_q: float,
    half_q: float,
    hold: float,
    rate: float,
    cycles: int,
) -> None:
    targets_close = {motor_id: close_q for motor_id in ids}
    targets_half = {motor_id: half_q for motor_id in ids}
    publish_pose(publisher, subscriber, "open_all", {}, ids, open_q, hold, rate)
    for cycle in range(cycles):
        print(f"\n=== cycle {cycle + 1}/{cycles} ===", flush=True)
        publish_pose(publisher, subscriber, "selected_close", targets_close, ids, open_q, hold, rate)
        publish_pose(publisher, subscriber, "selected_half", targets_half, ids, open_q, hold, rate)
        publish_pose(publisher, subscriber, "selected_open", {}, ids, open_q, hold, rate)
    publish_pose(publisher, subscriber, "final_open_all", {}, ids, open_q, hold, rate)


def main() -> int:
    parser = argparse.ArgumentParser(description="Test Inspire DFX hand motors through DDS.")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--ids", default="0,1,2,3,4,5,6,7,8,9,10,11", help="Comma-separated motor ids.")
    parser.add_argument("--mode", choices=("all", "scan"), default="scan")
    parser.add_argument("--open", type=clamp_q, default=1.0)
    parser.add_argument("--close", type=clamp_q, default=0.0)
    parser.add_argument("--half", type=clamp_q, default=0.45)
    parser.add_argument("--hold", type=float, default=1.2)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--no-manage-xr", action="store_true", help="Do not stop/restart xr-teleop.service.")
    parser.add_argument("--keep-xr-stopped", action="store_true", help="Leave xr-teleop.service stopped.")
    args = parser.parse_args()

    if args.rate <= 0:
        raise SystemExit("--rate must be positive")
    if args.hold <= 0:
        raise SystemExit("--hold must be positive")
    if args.cycles <= 0:
        raise SystemExit("--cycles must be positive")

    ids = parse_ids(args.ids)
    managed_xr = not args.no_manage_xr
    if managed_xr:
        print("Stopping xr-teleop.service so it cannot overwrite hand commands.", flush=True)
        systemctl_user("stop", "xr-teleop.service")
        time.sleep(1.0)
        systemctl_user("start", "inspire-hands.service")

    try:
        ChannelFactoryInitialize(0, networkInterface=args.network_interface)
        publisher = ChannelPublisher(TOPIC_CMD, MotorCmds_)
        publisher.Init()
        subscriber = ChannelSubscriber(TOPIC_STATE, MotorStates_)
        subscriber.Init()

        print(f"DDS ready on {args.network_interface}. Testing ids={ids}.", flush=True)
        if args.mode == "all":
            run_all(publisher, subscriber, ids, args.open, args.close, args.half, args.hold, args.rate, args.cycles)
        else:
            run_scan(publisher, subscriber, ids, args.open, args.close, args.half, args.hold, args.rate)
        return 0
    finally:
        if managed_xr and not args.keep_xr_stopped:
            print("\nRestarting xr-teleop.service.", flush=True)
            systemctl_user("restart", "xr-teleop.service")
        with suppress(Exception):
            time.sleep(0.2)


if __name__ == "__main__":
    raise SystemExit(main())
