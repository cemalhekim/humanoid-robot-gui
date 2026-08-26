#!/usr/bin/env python3
"""Hold endurance and speed-envelope tests on the twin, one twin+server pair per
tuning file (same Worker as optimize_params.py).

--mode hold      : for each pose, Move there, then hold for --hold-seconds; report
                   shake per 10 s window (must not grow), peak-to-peak in the last
                   window, drift, final error. "Static" = p2p < --static-p2p and the
                   last window's shake <= the first's * 1.2.
--mode envelope  : run the 18-pose sweep at --response (e.g. 10) for each tuning file,
                   which may raise TRAJECTORY_MAX_VELOCITY_RAD_S; report converge time,
                   overshoot, reversals, torque fraction -- the price of speed.

    endurance.py --mode hold --tunings baseline.json b1.json --hold-seconds 120
    endurance.py --mode envelope --tunings b1-env2.json b1-env4.json --response 10
Results: <out>/<stamp>-<mode>.json + .md
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import optimize_params as O  # noqa: E402
import sweep_arm_workspace as sweep  # noqa: E402

HOLD_POSES = ["forward_horizontal", "side_horizontal_T", "curl_forearm_up", "overhead_bent", "straight_up", "wrist_extremes"]


def hold_test(dash: sweep.Dashboard, args) -> list[dict]:
    rows = []
    move_args = argparse.Namespace(response=args.response, rate=20.0, hold_seconds=args.hold_seconds, timeout=args.hold_seconds + 60.0,
                                   settle_band=0.02, samples=False)
    for name in args.poses:
        targets = sweep.pose_targets(sweep.RIGHT_POSES[name], True)
        m = sweep.run_move(dash, targets, move_args, print)
        win = m.get("shake_windows_10s") or []
        static = bool(win) and (win[-1] <= win[0] * 1.2 + 1e-5) and m.get("hold_shake", 1.0) * 2.83 < args.static_p2p
        rows.append({"pose": name, "t_converge": m.get("t_converge"), "final_err": m.get("final_err"), "hold_drift": m.get("hold_drift"),
                     "hold_shake": m.get("hold_shake"), "shake_windows_10s": win, "escalation": m.get("escalation"), "static": static,
                     "error": m.get("error")})
        print(f"  {name:20s} conv={m.get('t_converge')} err={m.get('final_err')} drift={m.get('hold_drift')} shake={m.get('hold_shake')} windows={win} static={static}", flush=True)
    return rows


def envelope_test(dash: sweep.Dashboard, args) -> list[dict]:
    rows = []
    move_args = argparse.Namespace(response=args.response, rate=20.0, hold_seconds=2.0, timeout=45.0, settle_band=0.02, samples=False)
    for name in args.poses:
        targets = sweep.pose_targets(sweep.RIGHT_POSES[name], True)
        m = sweep.run_move(dash, targets, move_args, print)
        rows.append({"pose": name, **{k: m.get(k) for k in ("t_converge", "t_settle", "final_err", "overshoot", "reversals", "vel_ripple", "hold_shake", "tau_frac", "escalation", "fault", "error")}})
        print(f"  {name:20s} conv={m.get('t_converge')} settle={m.get('t_settle')} err={m.get('final_err')} over={m.get('overshoot')} rev={m.get('reversals')} tau={m.get('tau_frac')}", flush=True)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=("hold", "envelope"), required=True)
    ap.add_argument("--tunings", nargs="+", required=True, help="tuning JSON files; the literal 'baseline' = shipped constants")
    ap.add_argument("--poses", nargs="*", default=None)
    ap.add_argument("--response", type=float, default=2.5)
    ap.add_argument("--hold-seconds", type=float, default=120.0)
    ap.add_argument("--static-p2p", type=float, default=0.001)
    ap.add_argument("--python", default=sys.executable)
    ap.add_argument("--out", type=Path, default=HERE / "reports" / "endurance")
    ap.add_argument("--label", default="")
    args = ap.parse_args()
    args.poses = args.poses or (HOLD_POSES if args.mode == "hold" else list(sweep.RIGHT_POSES))
    args.out.mkdir(parents=True, exist_ok=True)
    log_dir = args.out / "logs"
    log_dir.mkdir(exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    report = {"mode": args.mode, "response": args.response, "hold_seconds": args.hold_seconds, "runs": []}
    for k, tuning in enumerate(args.tunings):
        tuning_path = log_dir / f"tuning-{k}.json"
        tuning_path.write_text("{}" if tuning == "baseline" else Path(tuning).read_text(), encoding="utf-8")
        print(f"== {tuning}", flush=True)
        worker = O.Worker(k, tuning_path, args.python, log_dir)
        try:
            if not worker.wait_ready():
                report["runs"].append({"tuning": tuning, "error": "worker did not come up"})
                continue
            dash = sweep.Dashboard(worker.url)
            rows = hold_test(dash, args) if args.mode == "hold" else envelope_test(dash, args)
            report["runs"].append({"tuning": tuning, "rows": rows})
        finally:
            worker.stop()
    base = args.out / f"{stamp}-{args.mode}{('-' + args.label) if args.label else ''}"
    base.with_suffix(".json").write_text(json.dumps(report, indent=1), encoding="utf-8")
    lines = [f"# {args.mode} test {stamp} {args.label}", "", f"response {args.response}" + (f" · hold {args.hold_seconds} s" if args.mode == "hold" else ""), ""]
    for run in report["runs"]:
        lines.append(f"## {run['tuning']}")
        if run.get("error"):
            lines.append(f"ERROR {run['error']}")
            continue
        if args.mode == "hold":
            lines += ["", "| pose | converge s | final err | drift | shake | shake per 10 s | static |", "|---|---|---|---|---|---|---|"]
            for r in run["rows"]:
                lines.append(f"| {r['pose']} | {r['t_converge']} | {r['final_err']} | {r['hold_drift']} | {r['hold_shake']} | {r['shake_windows_10s']} | {'yes' if r['static'] else 'NO'} |")
        else:
            lines += ["", "| pose | converge s | settle s | final err | overshoot | reversals | tau sh/el/wr | esc |", "|---|---|---|---|---|---|---|---|"]
            for r in run["rows"]:
                tf = r.get("tau_frac") or {}
                lines.append(f"| {r['pose']} | {r['t_converge']} | {r['t_settle']} | {r['final_err']} | {r['overshoot']} | {r['reversals']} | {tf.get('shoulder')}/{tf.get('elbow')}/{tf.get('wrist')} | {r['escalation']} |")
        lines.append("")
    base.with_suffix(".md").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))
    print(f"report: {base.with_suffix('.md')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
