#!/usr/bin/env python3
"""Evaluate many arm-replay parameter sets against the MuJoCo twin on a fixed
bank of random motions, in parallel, and rank them.

One trial = one parameter set. For each trial a worker starts its own twin
(DDS domain 10+worker) and its own server.py with RTW_TUNING_JSON pointing at
the trial's overrides, then drives every motion of the bank through the real
Move / sequence path (sweep_arm_workspace.run_move / run_sequence) and scores
it. Trials are independent, so N workers give N-fold speed-up; results are
appended to results.jsonl as they finish and the run can be resumed.

Per-motion cost (lower is better; weighted for precision, no shake, no oscillation):
    3*final_err/0.005 + 2*hold_shake/0.002 + 2*(vel_ripple/0.3 + reversals/3)
    + t_converge/15 + overshoot/0.06 + hold_drift/0.01
    + 3 if escalation > 1  + 10 if ceiling/fault/never-holding  + 2 if torque >= 95 %
Trial score = mean cost over the bank (``mean``) and mean of the worst 10 %
(``cvar10``); ranking uses cvar10 so a set that is great on average but bad in
one direction loses ("omnidirectional").

    optimize_params.py --motions motions.json --trials 50 --motions-per-trial 100 --workers 5
    optimize_params.py --report results.jsonl          # rank what has finished so far
    optimize_params.py --strategy cmaes --seed-from round1/results.jsonl --generations 8 --popsize 12
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import signal
import socket
import subprocess
import sys
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
import sweep_arm_workspace as sweep  # noqa: E402

# Search space: multiplicative ranges around the shipped value (log-uniform),
# plus a few absolute ranges. Keys are server.py constants (see RTW_TUNING_JSON).
BASELINE = {
    "ARM_REPLAY_PID_GAINS": {"shoulder": [0.28, 0.035, 0.018], "elbow": [0.24, 0.03, 0.014], "wrist": [0.18, 0.02, 0.012], "waist": [0.12, 0.01, 0.01]},
    "ARM_REPLAY_INNER_KP_SCALE": 0.35, "ARM_REPLAY_INNER_KD_SCALE": 1.2,
    "ARM_REPLAY_HOLD_KP_SCALE": 0.55, "ARM_REPLAY_HOLD_KD_SCALE": 1.2,
    "ARM_REPLAY_GRAVITY_HOLD_SCALE": 0.95, "ARM_REPLAY_GRAVITY_MOVE_SCALE": 0.5,
    "ARM_REPLAY_GRAVITY_LEARN_GAIN": 22.0, "ARM_REPLAY_GRAVITY_LEARN_LIMIT": 4.0,
    "ARM_REPLAY_GRAVITY_TAU_FILTER_SECONDS": 0.4,
    "ARM_REPLAY_MAX_PID_CORRECTION_RAD": 0.12, "ARM_REPLAY_INTEGRAL_LIMIT": 0.35,
    "ARM_REPLAY_APPROACH_PEAK_VEL_RAD_S": 0.6,
    "ARM_REPLAY_STALL_SECONDS": 2.5, "ARM_REPLAY_SETTLE_SECONDS": 0.6,
}
# (min multiplier, max multiplier) for the log-uniform scalar search
SCALAR_RANGES = {
    "pid_kp": (0.4, 3.0), "pid_ki": (0.2, 3.0), "pid_kd": (0.3, 3.0),
    "ARM_REPLAY_INNER_KP_SCALE": (0.5, 2.5), "ARM_REPLAY_INNER_KD_SCALE": (0.6, 1.8),
    "ARM_REPLAY_HOLD_KP_SCALE": (0.5, 1.8), "ARM_REPLAY_HOLD_KD_SCALE": (0.6, 1.8),
    "ARM_REPLAY_GRAVITY_HOLD_SCALE": (0.6, 1.05), "ARM_REPLAY_GRAVITY_MOVE_SCALE": (0.4, 1.8),
    "ARM_REPLAY_GRAVITY_LEARN_GAIN": (0.2, 4.0), "ARM_REPLAY_GRAVITY_LEARN_LIMIT": (0.5, 2.5),
    "ARM_REPLAY_GRAVITY_TAU_FILTER_SECONDS": (0.25, 3.0),
    "ARM_REPLAY_MAX_PID_CORRECTION_RAD": (0.4, 2.5), "ARM_REPLAY_INTEGRAL_LIMIT": (0.4, 2.5),
    "ARM_REPLAY_APPROACH_PEAK_VEL_RAD_S": (0.7, 2.5),
    "ARM_REPLAY_STALL_SECONDS": (0.8, 4.0), "ARM_REPLAY_SETTLE_SECONDS": (0.5, 2.0),
}


KEYS = list(SCALAR_RANGES)


def params_from_mult(mult: dict) -> dict:
    """Multipliers (one per SCALAR_RANGES key) -> server constants. PID gains are scaled
    per term (kp/ki/kd) across all joint groups, everything else per key."""
    params: dict = {}
    gains = {}
    for group, (kp, ki, kd) in BASELINE["ARM_REPLAY_PID_GAINS"].items():
        gains[group] = [round(kp * mult["pid_kp"], 5), round(ki * mult["pid_ki"], 5), round(kd * mult["pid_kd"], 5)]
    params["ARM_REPLAY_PID_GAINS"] = gains
    for key, base in BASELINE.items():
        if key == "ARM_REPLAY_PID_GAINS":
            continue
        params[key] = round(base * mult[key], 5)
    return params


def mult_from_params(params: dict) -> dict:
    sh = params["ARM_REPLAY_PID_GAINS"]["shoulder"]
    base = BASELINE["ARM_REPLAY_PID_GAINS"]["shoulder"]
    mult = {"pid_kp": sh[0] / base[0], "pid_ki": sh[1] / base[1], "pid_kd": sh[2] / base[2]}
    for key, value in BASELINE.items():
        if key != "ARM_REPLAY_PID_GAINS":
            mult[key] = params[key] / value
    return mult


def sample_params(rng: random.Random) -> dict:
    """One random parameter set (log-uniform inside SCALAR_RANGES)."""
    return params_from_mult({k: math.exp(rng.uniform(math.log(lo), math.log(hi))) for k, (lo, hi) in SCALAR_RANGES.items()})


def fitness(score: dict) -> float:
    """CMA-ES objective: average cost plus the worst-10 % cost, so both the typical
    move and the worst direction improve."""
    return score["mean"] + score["cvar10"]


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Worker:
    """A twin + a server.py with a given tuning file, on their own DDS domain."""

    def __init__(self, index: int, tuning_path: Path, python: str, log_dir: Path) -> None:
        self.domain = 10 + index
        self.port = free_port()
        self.url = f"http://127.0.0.1:{self.port}"
        env = dict(os.environ, RTW_SKIP_XR_SUSPEND="1", PYTHONUNBUFFERED="1",
                   PYTHONPATH=str(REPO / "execution/semantic_teleoperation/external/unitree_sdk2_python"))
        self.twin = subprocess.Popen(
            [python, str(HERE / "h1_2_mujoco_sim.py"), "--domain", str(self.domain), "--status-every", "0"],
            env=env, stdout=open(log_dir / f"twin-{index}.log", "a"), stderr=subprocess.STDOUT, start_new_session=True)
        env["RTW_TUNING_JSON"] = str(tuning_path)
        self.server = subprocess.Popen(
            [python, "-u", str(REPO / "server.py"), "--host", "127.0.0.1", "--port", str(self.port), "--domain", str(self.domain),
             "--robot-host", "127.0.0.1", "--camera-source", "lo", "--disable-camera"],
            cwd=str(REPO), env=env, stdout=open(log_dir / f"server-{index}.log", "a"), stderr=subprocess.STDOUT, start_new_session=True)

    def wait_ready(self, timeout: float = 30.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                state = json.load(urllib.request.urlopen(self.url + "/api/state", timeout=2))
                if state.get("connected") and state.get("sample_rate_hz", 0) > 100:
                    return True
            except Exception:
                pass
            if self.server.poll() is not None or self.twin.poll() is not None:
                return False
            time.sleep(0.5)
        return False

    def stop(self) -> None:
        for proc in (self.server, self.twin):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
        for proc in (self.server, self.twin):
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)


# Weights follow the stated goals: tighter final tolerance, no shake after
# reaching, no oscillation on the way (especially descending with gravity).
def motion_cost(m: dict) -> float:
    if m.get("error"):
        return 20.0
    cost = 0.0
    cost += 3.0 * m["final_err"] / 0.005          # precision: 0.005 rad = 0.3 deg costs 3
    cost += 2.0 * m.get("hold_shake", 0.0) / 0.002  # shake after reaching
    cost += 2.0 * (m.get("vel_ripple", 0.0) / 0.3 + m.get("reversals", 0) / 3.0)  # oscillation while moving
    cost += (m["t_converge"] if m["t_converge"] is not None else 15.0) / 15.0
    cost += m["overshoot"] / 0.06
    cost += m["hold_drift"] / 0.01
    if m.get("escalation") and float(m["escalation"]) > 1.0:
        cost += 3.0
    if m["t_converge"] is None or m.get("ceiling") or m.get("fault"):
        cost += 10.0
    if any(v >= 0.95 for v in m.get("tau_frac", {}).values()):
        cost += 2.0
    return round(cost, 4)


def sequence_cost(r: dict) -> float:
    if r.get("error"):
        return 20.0
    cost = r["rms_after_lag"] / 0.03 + r["lag_s"] / 1.0 + r["final_err"] / 0.02
    if r["t_hold"] is None:
        cost += 10.0
    return round(cost, 4)


def run_trial(trial: dict, motions: list[dict], args, log_dir: Path, worker_index: int) -> dict:
    tuning_path = log_dir / f"trial-{trial['id']:04d}.json"
    tuning_path.write_text(json.dumps(trial["params"]), encoding="utf-8")
    worker = Worker(worker_index, tuning_path, args.python, log_dir)
    try:
        if not worker.wait_ready():
            return {**trial, "error": "worker did not come up", "score": None}
        dash = sweep.Dashboard(worker.url)
        move_args = argparse.Namespace(response=args.response, rate=20.0, hold_seconds=args.hold_seconds, timeout=args.timeout,
                                       settle_band=0.02, samples=args.store_samples)
        results = []
        t0 = time.time()
        for m in motions:
            start = {int(k): v for k, v in m["start"].items()}
            goal = {int(k): v for k, v in m["goal"].items()}
            pre = sweep.run_move(dash, start, move_args, print)  # get to the start pose (not scored)
            if pre.get("error"):
                results.append({"id": m["id"], "cost": 20.0, "error": "start: " + pre["error"]})
                continue
            if m["kind"] == "trajectory":
                r = sweep.run_sequence(dash, start, goal, m["duration_s"], move_args, print)
                results.append({"id": m["id"], "kind": "trajectory", "direction": m["direction"], "cost": sequence_cost(r),
                                **{k: r.get(k) for k in ("lag_s", "rms_after_lag", "final_err", "t_hold", "error")}})
            else:
                r = sweep.run_move(dash, goal, move_args, print)
                results.append({"id": m["id"], "kind": "pose", "direction": m["direction"], "cost": motion_cost(r),
                                **{k: r.get(k) for k in ("t_converge", "final_err", "overshoot", "hold_drift", "hold_shake", "vel_ripple", "reversals", "ripple_joint", "escalation", "ceiling", "fault", "error")},
                                **({"samples": r.get("samples")} if args.store_samples else {})})
        costs = sorted(r["cost"] for r in results)
        worst = costs[-max(1, len(costs) // 10):]
        by_dir: dict[str, list] = {}
        for r in results:
            by_dir.setdefault(r.get("direction", "?"), []).append(r["cost"])
        return {**trial, "score": {"mean": round(sum(costs) / len(costs), 4), "cvar10": round(sum(worst) / len(worst), 4), "max": costs[-1],
                                   "by_direction": {d: round(sum(v) / len(v), 4) for d, v in by_dir.items()},
                                   "escalations": sum(1 for r in results if r.get("escalation") and float(r["escalation"]) > 1.0),
                                   "oscillating": sum(1 for r in results if (r.get("reversals") or 0) > 3),
                                   "failures": sum(1 for r in results if r.get("error") or (r.get("kind") == "pose" and r.get("t_converge") is None) or (r.get("kind") == "trajectory" and r.get("t_hold") is None))},
                "seconds": round(time.time() - t0, 1), "results": results}
    finally:
        worker.stop()


def run_cmaes(args, motions: list[dict], log_dir: Path, results_path: Path) -> None:
    """Covariance-matrix-adaptation evolution strategy over log-multipliers.
    Seeded from the best finished trials of --seed-from (mean of the top-k in
    log space), otherwise from the shipped constants. State is pickled after every
    generation, so a killed run continues from the next generation."""
    import cma  # pip install cma

    lo = [math.log(SCALAR_RANGES[k][0]) for k in KEYS]
    hi = [math.log(SCALAR_RANGES[k][1]) for k in KEYS]
    state_path = args.out / "cmaes.pkl"
    done_rows = [json.loads(l) for l in results_path.read_text().splitlines() if l.strip()] if results_path.exists() else []
    next_id = max((r["id"] for r in done_rows), default=-1) + 1
    if state_path.exists():
        es = cma.CMAEvolutionStrategy.pickle_loads(state_path.read_bytes())
        print(f"resuming CMA-ES at generation {es.countiter}", flush=True)
    else:
        x0 = [0.0] * len(KEYS)
        if args.seed_from:
            rows = [json.loads(l) for l in args.seed_from.read_text().splitlines() if l.strip()]
            rows = sorted((r for r in rows if r.get("score")), key=lambda r: fitness(r["score"]))[: args.seed_top]
            if rows:
                logs = [[math.log(mult_from_params(r["params"])[k]) for k in KEYS] for r in rows]
                x0 = [sum(col) / len(col) for col in zip(*logs)]
                print(f"seeded from top {len(rows)} of {args.seed_from}: " + ", ".join(f"{k}={math.exp(v):.2f}" for k, v in zip(KEYS, x0)), flush=True)
        es = cma.CMAEvolutionStrategy(x0, args.sigma0, {"bounds": [lo, hi], "popsize": args.popsize, "seed": args.seed, "verbose": -9})
    with ThreadPoolExecutor(max_workers=args.workers) as pool, open(results_path, "a", encoding="utf-8") as out:
        while es.countiter < args.generations:
            X = es.ask()
            trials = []
            for x in X:
                mult = {k: math.exp(v) for k, v in zip(KEYS, x)}
                trials.append({"id": next_id, "gen": es.countiter, "params": params_from_mult(mult), "x": [float(v) for v in x]})
                next_id += 1
            slots = list(range(args.workers))
            futures = {}
            queue = list(trials)
            finished = {}
            while queue or futures:
                while queue and slots:
                    trial = queue.pop(0)
                    slot = slots.pop(0)
                    futures[pool.submit(run_trial, trial, motions, args, log_dir, slot)] = slot
                fut = next(as_completed(list(futures)))
                slots.append(futures.pop(fut))
                res = fut.result()
                out.write(json.dumps(res) + "\n")
                out.flush()
                finished[res["id"]] = res
                sc = res.get("score")
                print(f"gen {res['gen']} trial {res['id']:>4}: " + (f"fitness {fitness(sc):.3f} (mean {sc['mean']:.3f} cvar10 {sc['cvar10']:.3f}) esc {sc['escalations']} osc {sc.get('oscillating', 0)} fail {sc['failures']} ({res['seconds']} s)"
                                                                if sc else f"ERROR {res.get('error')}"), flush=True)
            values = [fitness(finished[t["id"]]["score"]) if finished[t["id"]].get("score") else 1e3 for t in trials]
            es.tell(X, values)
            state_path.write_bytes(es.pickle_dumps())
            best = min(values)
            print(f"== generation {es.countiter} done: best {best:.3f}, mean {sum(values) / len(values):.3f}, sigma {es.sigma:.3f}", flush=True)
    print("CMA-ES finished; best-ever x:", {k: round(math.exp(v), 3) for k, v in zip(KEYS, es.result.xbest)})


def rescore(results: list[dict]) -> dict:
    """Recompute a trial score from its per-motion results (used after excluding motions)."""
    costs = sorted(r["cost"] for r in results)
    worst = costs[-max(1, len(costs) // 10):]
    by_dir: dict[str, list] = {}
    for r in results:
        by_dir.setdefault(r.get("direction", "?"), []).append(r["cost"])
    return {"mean": round(sum(costs) / len(costs), 4), "cvar10": round(sum(worst) / len(worst), 4), "max": costs[-1],
            "by_direction": {d: round(sum(v) / len(v), 4) for d, v in by_dir.items()},
            "escalations": sum(1 for r in results if r.get("escalation") and float(r["escalation"]) > 1.0),
            "oscillating": sum(1 for r in results if (r.get("reversals") or 0) > 3),
            "failures": sum(1 for r in results if r.get("error") or (r.get("kind") == "pose" and r.get("t_converge") is None) or (r.get("kind") == "trajectory" and r.get("t_hold") is None))}


def report(path: Path, top: int = 15, exclude: set[int] | None = None) -> None:
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    rows = [r for r in rows if r.get("score")]
    if exclude:
        for r in rows:
            kept = [x for x in r["results"] if x["id"] not in exclude]
            if kept:
                r["score"] = rescore(kept)
        print(f"(excluding {len(exclude)} motions, e.g. colliding joint-space paths)")
    if not rows:
        print("no finished trials")
        return
    rows.sort(key=lambda r: fitness(r["score"]))
    base = next((r for r in rows if r.get("baseline")), None)
    print(f"{len(rows)} trials · ranked by fitness = mean + cvar10 (mean of the worst 10 %)\n")
    print(f"{'rank':>4} {'id':>4} {'gen':>3} {'fitness':>7} {'cvar10':>7} {'mean':>7} {'max':>6} {'esc':>4} {'osc':>4} {'fail':>4}  against  with  parallel  note")
    for k, r in enumerate(rows[:top], 1):
        s = r["score"]
        bd = s["by_direction"]
        note = "BASELINE" if r.get("baseline") else ""
        print(f"{k:>4} {r['id']:>4} {str(r.get('gen', '-')):>3} {fitness(s):>7.3f} {s['cvar10']:>7.3f} {s['mean']:>7.3f} {s['max']:>6.2f} {s['escalations']:>4} {s.get('oscillating', 0):>4} {s['failures']:>4}  "
              f"{bd.get('against_gravity', 0):>7.3f} {bd.get('with_gravity', 0):>5.3f} {bd.get('parallel', 0):>9.3f}  {note}")
    if base:
        rank = rows.index(base) + 1
        print(f"\nbaseline (shipped constants) ranks {rank}/{len(rows)}: fitness {fitness(base['score']):.3f} (cvar10 {base['score']['cvar10']}, mean {base['score']['mean']})")
    best = rows[0]
    print("\nbest parameters:\n" + json.dumps(best["params"], indent=1))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--motions", type=Path, default=HERE / "motions.json")
    ap.add_argument("--trials", type=int, default=50)
    ap.add_argument("--motions-per-trial", type=int, default=100)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--python", default=sys.executable)
    ap.add_argument("--response", type=float, default=2.5, help="replay_response dial (UI default 2.5 = 25% of the 0-10 slider)")
    ap.add_argument("--hold-seconds", type=float, default=2.0)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--out", type=Path, default=HERE / "optimize")
    ap.add_argument("--include-baseline", action="store_true", default=True)
    ap.add_argument("--params-file", type=Path, help="evaluate only these parameter sets (JSON list) instead of sampling")
    ap.add_argument("--store-samples", action="store_true", help="keep the 20 Hz joint traces of every move in results.jsonl (~2 MB/trial) so runs can be re-scored offline")
    ap.add_argument("--report", type=Path, help="print the ranking of an existing results.jsonl and exit")
    ap.add_argument("--exclude", type=Path, help="JSON list of motion ids to leave out when reporting (e.g. motions-colliding-path.json)")
    ap.add_argument("--strategy", choices=("random", "cmaes"), default="random")
    ap.add_argument("--generations", type=int, default=8)
    ap.add_argument("--popsize", type=int, default=12)
    ap.add_argument("--sigma0", type=float, default=0.35, help="initial CMA-ES step in log-multiplier space")
    ap.add_argument("--seed-from", type=Path, help="results.jsonl of a previous round to seed CMA-ES from")
    ap.add_argument("--seed-top", type=int, default=5)
    args = ap.parse_args()
    if args.report:
        report(args.report, exclude=set(json.loads(args.exclude.read_text())) if args.exclude else None)
        return 0

    args.out.mkdir(parents=True, exist_ok=True)
    results_path = args.out / "results.jsonl"
    done = set()
    if results_path.exists():
        for line in results_path.read_text().splitlines():
            if line.strip():
                done.add(json.loads(line)["id"])
    bank = json.loads(args.motions.read_text())
    rng = random.Random(args.seed)
    motions = bank["motions"][:]
    rng.shuffle(motions)
    motions = motions[: args.motions_per_trial]
    log_dir = args.out / "logs"
    log_dir.mkdir(exist_ok=True)
    if args.strategy == "cmaes":
        run_cmaes(args, motions, log_dir, results_path)
        report(results_path)
        return 0
    if args.params_file:
        sets = json.loads(args.params_file.read_text())
        trials = [{"id": i, "params": p} for i, p in enumerate(sets)]
    else:
        trials = []
        if args.include_baseline:
            trials.append({"id": 0, "params": BASELINE, "baseline": True})
        while len(trials) < args.trials:
            trials.append({"id": len(trials), "params": sample_params(rng)})
    pending = [t for t in trials if t["id"] not in done]
    est = len(motions) * 9.0 * len(pending) / max(1, args.workers) / 60.0
    print(f"{len(pending)} trials pending ({len(done)} done) × {len(motions)} motions on {args.workers} workers — "
          f"~{est:.0f} min at ~9 s/motion", flush=True)
    with ThreadPoolExecutor(max_workers=args.workers) as pool, open(results_path, "a", encoding="utf-8") as out:
        slots = list(range(args.workers))
        futures = {}
        queue = list(pending)
        while queue or futures:
            while queue and slots:
                trial = queue.pop(0)
                slot = slots.pop(0)
                futures[pool.submit(run_trial, trial, motions, args, log_dir, slot)] = slot
            fut = next(as_completed(list(futures)))
            slots.append(futures.pop(fut))
            res = fut.result()
            out.write(json.dumps(res) + "\n")
            out.flush()
            s = res.get("score")
            print(f"trial {res['id']:>4}: " + (f"cvar10 {s['cvar10']:.3f} mean {s['mean']:.3f} esc {s['escalations']} osc {s.get('oscillating', 0)} fail {s['failures']} ({res['seconds']} s)"
                                            if s else f"ERROR {res.get('error')}"), flush=True)
    report(results_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
