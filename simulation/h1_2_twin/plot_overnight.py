#!/usr/bin/env python3
"""Overnight comparison plots: all candidate sets on the common 300-motion set, and
the 120 s hold-endurance shake decay per tuning."""
from __future__ import annotations
import json, sys
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import optimize_params as O
D = HERE / "reports" / "optimize"; E = HERE / "reports" / "endurance"
excl = set(json.loads((HERE / "motions-colliding-path.json").read_text()))
def load(p): return [json.loads(l) for l in Path(p).read_text().splitlines() if l.strip()]
def stats(r):
    res = [x for x in r["results"] if x["id"] not in excl]; s = O.rescore(res)
    p = [x for x in res if x.get("kind") == "pose"]
    avg = lambda k: sum(x[k] for x in p if x.get(k) is not None) / max(1, len([x for x in p if x.get(k) is not None]))
    return dict(fit=O.fitness(s), mean=s["mean"], osc=s["oscillating"], fail=s["failures"], err=avg("final_err"), shake=avg("hold_shake"), rev=avg("reversals"), tconv=avg("t_converge"), by=s["by_direction"])
rows = []
fin = load(D / "finalists-scores.jsonl")
r2 = sorted([r for r in load(D / "round2-results.jsonl") if r.get("score")], key=lambda r: O.fitness(r["score"]))[:5]
r1 = sorted([r for r in load(D / "round1-results.jsonl") if r.get("score")], key=lambda r: O.fitness(r["score"]))[:3]
names = {0: "A shipped (baseline)"}
for i, r in enumerate(r2): names[1 + i] = f"A r2 #{r['id']}"
for i, r in enumerate(r1): names[6 + i] = f"A r1 #{r['id']}"
for r in fin: rows.append((names.get(r["id"], f"finalist {r['id']}"), stats(r)))
for r, n in zip(load(D / "b1-on300-scores.jsonl"), ["B1 (#75 + model gravity)", "B1 learn-low"]): rows.append((n, stats(r)))
for r in load(D / "b1gated-on300-scores.jsonl"): rows.append(("B1-gated (+ learn gate/leak)", stats(r)))
extra = D / "2b-on300-scores.jsonl"
if extra.exists():
    for r in load(extra): rows.append(("2b #70 on B1-gated (CMA-ES)", stats(r)))
rows.sort(key=lambda x: x[1]["fit"])
fig, axes = plt.subplots(2, 2, figsize=(16, 10))
labels = [n for n, _ in rows]; x = range(len(rows))
col = ["#2ca02c" if n.startswith("B1") or n.startswith("2b") else ("#ff7f0e" if "shipped" in n else "#1f77b4") for n in labels]
for ax, key, title, lim in ((axes[0, 0], "fit", "fitness = mean + worst-10 % (lower is better)", None), (axes[0, 1], "err", "mean final error [rad]", 0.005), (axes[1, 0], "shake", "mean hold shake std [rad]", 0.002), (axes[1, 1], "tconv", "mean converge time [s]", None)):
    ax.bar(x, [s[key] for _, s in rows], color=col)
    if lim: ax.axhline(lim, color="k", ls="--", lw=0.8)
    ax.set_xticks(list(x)); ax.set_xticklabels(labels, rotation=40, ha="right", fontsize=8); ax.set_title(title, fontsize=10); ax.grid(axis="y", alpha=0.3)
for i, (n, s) in enumerate(rows):
    axes[0, 0].text(i, s["fit"], f"osc {s['osc']}\nfail {s['fail']}", ha="center", va="bottom", fontsize=7)
fig.suptitle("All candidates on the same 300 random motions (18 colliding-path motions excluded) — green = model-gravity structure, orange = shipped", fontsize=11)
fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(D / "overnight-300-comparison.png", dpi=110); print(D / "overnight-300-comparison.png")
# endurance decay
hold = sorted(E.glob("*-hold-*.json"))
if hold:
    rep = json.loads(hold[-1].read_text())
    fig, ax = plt.subplots(figsize=(11, 5))
    for run in rep["runs"]:
        if run.get("error"): continue
        name = Path(run["tuning"]).stem.replace("t-", "") if run["tuning"] != "baseline" else "baseline"
        for r in run["rows"]:
            w = r["shake_windows_10s"]
            ax.plot([10 * (k + 1) for k in range(len(w))], w, marker=".", lw=1, alpha=0.8, label=name if r is run["rows"][0] else None,
                    color={"baseline": "#ff7f0e", "round2-75": "#1f77b4", "b1": "#2ca02c", "b1gated": "#17becf"}.get(name, "k"))
    ax.set_yscale("symlog", linthresh=1e-4); ax.set_xlabel("seconds into the hold (10 s windows)"); ax.set_ylabel("shake std per window [rad]")
    ax.set_title("120 s hold endurance, 6 loaded poses — does the hold become static? (baseline/#75: constant limit cycle; B1: decays to 0)")
    ax.legend(); ax.grid(alpha=0.3); fig.tight_layout(); fig.savefig(E / "hold-endurance-decay.png", dpi=110); print(E / "hold-endurance-decay.png")
