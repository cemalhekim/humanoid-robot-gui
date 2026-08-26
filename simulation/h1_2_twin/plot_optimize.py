#!/usr/bin/env python3
"""Plots for optimize_params.py runs: random-search sensitivity, CMA-ES
convergence, parameter drift per generation, and a top-k vs baseline comparison.

    plot_optimize.py --round1 r1/results.jsonl --round2 r2/results.jsonl --out reports/optimize
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import optimize_params as O  # noqa: E402

DIRS = ["against_gravity", "with_gravity", "parallel"]
DCOL = {"against_gravity": "#d62728", "with_gravity": "#1f77b4", "parallel": "#2ca02c"}


def load(path: Path) -> list[dict]:
    rows = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
    return [r for r in rows if r.get("score")]


def short(k: str) -> str:
    return k.replace("ARM_REPLAY_", "").replace("_SCALE", "").replace("_SECONDS", "_s").replace("GRAVITY_", "G_").lower()


def plot_random(rows: list[dict], out: Path) -> Path:
    rows = sorted(rows, key=lambda r: O.fitness(r["score"]))
    fig = plt.figure(figsize=(16, 13))
    gs = fig.add_gridspec(5, 6, hspace=0.6, wspace=0.35)
    ax = fig.add_subplot(gs[0, :])
    vals = [O.fitness(r["score"]) for r in rows]
    cols = ["#ff7f0e" if r.get("baseline") else ("#2ca02c" if v < vals[0] * 1.5 else "#7f7f7f") for r, v in zip(rows, vals)]
    ax.bar(range(len(rows)), vals, color=cols)
    base = next((i for i, r in enumerate(rows) if r.get("baseline")), None)
    if base is not None:
        ax.annotate(f"baseline (rank {base + 1})", (base, vals[base]), xytext=(base + 2, vals[base] * 1.6), arrowprops=dict(arrowstyle="->"), fontsize=9)
    ax.set_yscale("log")
    ax.set_ylabel("fitness = mean + cvar10 (log)")
    ax.set_xlabel("random parameter sets, sorted")
    ax.set_title("Round 1 — random search: fitness of every set (lower is better)")
    ax.grid(axis="y", alpha=0.3)
    # sensitivity: fitness vs each multiplier
    for n, key in enumerate(O.KEYS):
        a = fig.add_subplot(gs[1 + n // 6, n % 6])
        xs = [O.mult_from_params(r["params"])[key] for r in rows]
        ys = vals
        a.scatter(xs, ys, s=12, c=["#ff7f0e" if r.get("baseline") else "#1f77b4" for r in rows], alpha=0.8)
        a.set_xscale("log")
        a.set_yscale("log")
        a.axvline(1.0, color="k", lw=0.6, ls="--")
        a.set_title(short(key), fontsize=8)
        a.tick_params(labelsize=6)
        a.grid(alpha=0.25)
    fig.text(0.5, 0.005, "lower panels: fitness vs multiplier of each parameter (×1 = shipped value, dashed). A downward trend = this parameter matters in that direction.", ha="center", fontsize=9)
    path = out / "round1-random-search.png"
    fig.savefig(path, dpi=110, bbox_inches="tight")
    return path


def plot_cmaes(rows: list[dict], out: Path) -> Path:
    gens = sorted({r["gen"] for r in rows if "gen" in r})
    fig, axes = plt.subplots(2, 1, figsize=(14, 11), gridspec_kw={"height_ratios": [1, 1.6]})
    ax = axes[0]
    for g in gens:
        vals = [O.fitness(r["score"]) for r in rows if r.get("gen") == g]
        ax.scatter([g] * len(vals), vals, s=14, color="#1f77b4", alpha=0.6)
    best = [min(O.fitness(r["score"]) for r in rows if r.get("gen") == g) for g in gens]
    mean = [sum(O.fitness(r["score"]) for r in rows if r.get("gen") == g) / max(1, len([r for r in rows if r.get("gen") == g])) for g in gens]
    ax.plot(gens, best, "o-", color="#2ca02c", label="best of generation")
    ax.plot(gens, mean, "s--", color="#7f7f7f", label="mean of generation")
    ax.set_yscale("log")
    ax.set_xlabel("generation")
    ax.set_ylabel("fitness (log)")
    ax.set_title("Round 2 — CMA-ES: population fitness per generation")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)
    # parameter drift: population mean multiplier per generation
    ax = axes[1]
    for key in O.KEYS:
        series = []
        for g in gens:
            ms = [math.log(O.mult_from_params(r["params"])[key]) for r in rows if r.get("gen") == g]
            series.append(math.exp(sum(ms) / len(ms)))
        ax.plot(gens, series, marker=".", lw=1.2, label=short(key))
    ax.axhline(1.0, color="k", lw=0.8, ls="--")
    ax.set_yscale("log")
    ax.set_xlabel("generation")
    ax.set_ylabel("population-mean multiplier vs shipped (log)")
    ax.set_title("Where the search is moving: mean multiplier of each parameter per generation (×1 = shipped)")
    ax.legend(fontsize=7, ncol=4, loc="upper left")
    ax.grid(alpha=0.3)
    path = out / "round2-cmaes.png"
    fig.savefig(path, dpi=110, bbox_inches="tight")
    return path


def plot_top(rows_by_round: dict[str, list[dict]], out: Path, k: int = 3) -> Path:
    picks = []
    for name, rows in rows_by_round.items():
        base = next((r for r in rows if r.get("baseline")), None)
        if base and not any(p[1].get("baseline") for p in picks):
            picks.append((f"baseline ({name})", base))
        for r in sorted(rows, key=lambda r: O.fitness(r["score"]))[:k]:
            picks.append((f"{name} #{r['id']}", r))
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5))
    x = range(len(picks))
    labels = [p[0] for p in picks]
    ax = axes[0]
    w = 0.27
    for j, d in enumerate(DIRS):
        ax.bar([i + (j - 1) * w for i in x], [p[1]["score"]["by_direction"].get(d, 0) for p in picks], w, color=DCOL[d], label=d)
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
    ax.set_ylabel("mean cost per direction")
    ax.set_title("Cost by gravity direction (lower is better)")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.3)
    ax = axes[1]
    ax.bar([i - 0.2 for i in x], [p[1]["score"].get("oscillating", 0) for p in picks], 0.4, color="#d62728", label="oscillating moves")
    ax.bar([i + 0.2 for i in x], [p[1]["score"]["escalations"] + p[1]["score"]["failures"] for p in picks], 0.4, color="#7f7f7f", label="escalations + failures")
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
    ax.set_title("Bad moves out of the trial's motion set")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.3)
    ax = axes[2]
    keys = ["pid_kp", "pid_ki", "pid_kd", "ARM_REPLAY_INNER_KP_SCALE", "ARM_REPLAY_HOLD_KP_SCALE", "ARM_REPLAY_GRAVITY_MOVE_SCALE",
            "ARM_REPLAY_GRAVITY_LEARN_GAIN", "ARM_REPLAY_GRAVITY_TAU_FILTER_SECONDS", "ARM_REPLAY_STALL_SECONDS", "ARM_REPLAY_APPROACH_PEAK_VEL_RAD_S"]
    for name, r in picks:
        if r.get("baseline"):
            continue
        m = O.mult_from_params(r["params"])
        ax.plot([m[key] for key in keys], marker="o", lw=1, label=name)
    ax.axhline(1.0, color="k", lw=0.8, ls="--", label="shipped")
    ax.set_yscale("log")
    ax.set_xticks(range(len(keys)))
    ax.set_xticklabels([short(k) for k in keys], rotation=35, ha="right", fontsize=8)
    ax.set_ylabel("multiplier vs shipped (log)")
    ax.set_title("What the best sets changed")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
    fig.suptitle("Top sets vs shipped constants — note: round 1 and round 2 were scored on different 60-motion subsets; the finalists run puts them on one set", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    path = out / "top-sets-vs-baseline.png"
    fig.savefig(path, dpi=110)
    return path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--round1", type=Path)
    ap.add_argument("--round2", type=Path)
    ap.add_argument("--out", type=Path, default=HERE / "reports" / "optimize")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    paths = []
    rows_by_round = {}
    if args.round1:
        r1 = load(args.round1)
        rows_by_round["round1"] = r1
        paths.append(plot_random(r1, args.out))
    if args.round2:
        r2 = load(args.round2)
        rows_by_round["round2"] = r2
        paths.append(plot_cmaes(r2, args.out))
    if rows_by_round:
        paths.append(plot_top(rows_by_round, args.out))
    for p in paths:
        print(p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
