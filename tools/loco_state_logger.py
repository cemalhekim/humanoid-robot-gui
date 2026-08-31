#!/usr/bin/env python3
"""Log the H1-2 loco controller's own state while it runs. Read-only.

The telemetry recorder captures IMU and joints, but not what the locomotion
controller thinks it is doing. This polls the LocoClient getters and writes them
to CSV so a gait run can be read against the IMU trace on the same clock:

  * fsm_id / fsm_mode -- catches the controller entering or dropping out of
    motion mode on its own, which a joint trace alone will not show
  * phase             -- the gait cycle. A phase that advances at zero commanded
    velocity means the controller is stepping by design (ContinuousGait), not
    reacting to a tilt error. This is the measurement that separates "designed
    to march in place" from "correcting a bad attitude estimate".

Every call here is a getter. This tool sets nothing and cannot move the robot.

    tools/loco_state_logger.py enp31s0
    tools/loco_state_logger.py enp31s0 --duration 120 --csv /tmp/loco.csv

Run from a machine on the 192.168.123.x link with unitree_sdk2py on PYTHONPATH,
or on the robot PC itself.
"""

from __future__ import annotations

import argparse
import csv
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient

REPO_ROOT = Path(__file__).resolve().parent.parent

# Only getters. Nothing on this list changes robot state.
GETTERS = ("GetFsmId", "GetFsmMode", "GetBalanceMode", "GetPhase")

CSV_COLUMNS = [
    "wall_time", "iso_time", "elapsed_s",
    "fsm_id", "fsm_mode", "balance_mode",
    "phase_0", "phase_1", "phase_advanced",
    "errors",
]

# FSM ids verified on this H1-2 (server.py:461). The SDK helpers assume H1 ids,
# which this machine ignores.
FSM_NAMES = {
    0: "zero torque",
    1: "damp",
    3: "stand up / ready",
    201: "motion mode (gait)",
}


def fsm_label(value: Any) -> str:
    try:
        return f"{value} ({FSM_NAMES.get(int(value), 'bilinmiyor')})"
    except (TypeError, ValueError):
        return str(value)


def default_csv_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return REPO_ROOT / "recordings" / "imu" / f"loco-{stamp}.csv"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("interface", help="network interface on the robot link, e.g. enp31s0")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--rate", type=float, default=5.0, help="poll rate in Hz (default 5)")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (default: until Ctrl-C)")
    parser.add_argument("--csv", type=Path, default=None, help="CSV path (default recordings/imu/loco-<stamp>.csv)")
    parser.add_argument("--timeout", type=float, default=5.0, help="LocoClient RPC timeout (default 5)")
    args = parser.parse_args(argv)

    ChannelFactoryInitialize(args.domain, args.interface)
    client = LocoClient()
    client.SetTimeout(args.timeout)
    client.Init()

    csv_path = args.csv or default_csv_path()
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    handle = csv_path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
    writer.writeheader()

    stopping = False

    def on_signal(_signum: int, _frame: Any) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    period = 1.0 / args.rate if args.rate > 0 else 0.0
    start = time.monotonic()
    prev_fsm: Any = None
    prev_phase: Any = None
    phase_moves = 0
    n = 0

    print(f"log -> {csv_path}   (salt-okunur, Ctrl-C ile bitir)", flush=True)
    try:
        while not stopping:
            loop_start = time.monotonic()
            elapsed = loop_start - start
            if args.duration and elapsed >= args.duration:
                break

            values: dict[str, Any] = {}
            errors = 0
            for name in GETTERS:
                try:
                    code, val = getattr(client, name)()
                    values[name] = val if code == 0 else None
                    if code != 0:
                        errors += 1
                except Exception:
                    values[name] = None
                    errors += 1

            phase = values.get("GetPhase") or []
            phase_0 = phase[0] if len(phase) > 0 else None
            phase_1 = phase[1] if len(phase) > 1 else None
            advanced = int(prev_phase is not None and phase != prev_phase)
            phase_moves += advanced
            n += 1

            writer.writerow({
                "wall_time": round(time.time(), 6),
                "iso_time": datetime.now().isoformat(timespec="milliseconds"),
                "elapsed_s": round(elapsed, 3),
                "fsm_id": values.get("GetFsmId"),
                "fsm_mode": values.get("GetFsmMode"),
                "balance_mode": values.get("GetBalanceMode"),
                "phase_0": phase_0,
                "phase_1": phase_1,
                "phase_advanced": advanced,
                "errors": errors,
            })
            if n % 10 == 0:
                handle.flush()

            fsm = values.get("GetFsmId")
            if fsm != prev_fsm:
                print(f"[{elapsed:7.1f}s] FSM DEGISTI: {fsm_label(prev_fsm)} -> {fsm_label(fsm)}", flush=True)
                prev_fsm = fsm
            prev_phase = phase

            print(
                f"[{elapsed:7.1f}s] fsm={fsm} mode={values.get('GetFsmMode')}"
                f" balance={values.get('GetBalanceMode')}"
                f" phase=[{phase_0}, {phase_1}]"
                f" {'ILERLIYOR' if advanced else 'sabit'}"
                + (f"  hata={errors}" if errors else ""),
                flush=True,
            )

            remaining = period - (time.monotonic() - loop_start)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        handle.close()

    print()
    print(f"{n} ornek, fazin degistigi ornek sayisi: {phase_moves}")
    if n:
        if phase_moves == 0:
            print("Faz hic ilerlemedi -> kontrolcu adim dongusu dondurmuyordu.")
        else:
            print(f"Faz orneklerin %{100 * phase_moves / n:.0f}'inde ilerledi -> aktif adim dongusu vardi.")
    print(f"CSV: {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
