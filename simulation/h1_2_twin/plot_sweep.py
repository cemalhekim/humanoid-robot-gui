#!/usr/bin/env python3
"""Plot a sweep_arm_workspace.py JSON report: per-move joint error vs time,
tracking sequences, and a summary. Writes PNGs next to the report."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

JOINT_NAMES = {
    13: "L ShPitch", 14: "L ShRoll", 15: "L ShYaw", 16: "L Elbow", 17: "L WrRoll", 18: "L WrPitch", 19: "L WrYaw",
    20: "R ShPitch", 21: "R ShRoll", 22: "R ShYaw", 23: "R Elbow", 24: "R WrRoll", 25: "R WrPitch", 26: "R WrYaw",
}
SHORT = {"SP": "ShoulderPitch", "SR": "ShoulderRoll", "SY": "ShoulderYaw", "E": "Elbow",
         "WR": "WristRoll", "WP": "WristPitch", "WY": "WristYaw"}
MIRROR_NEGATE = {"ShoulderRoll", "ShoulderYaw", "WristRoll", "WristYaw"}
NAME_TO_INDEX = {"Left" + v: k for k, v in {13: "ShoulderPitch", 14: "ShoulderRoll", 15: "ShoulderYaw", 16: "Elbow", 17: "WristRoll", 18: "WristPitch", 19: "WristYaw"}.items()}
NAME_TO_INDEX.update({"Right" + v: k for k, v in {20: "ShoulderPitch", 21: "ShoulderRoll", 22: "ShoulderYaw", 23: "Elbow", 24: "WristRoll", 25: "WristPitch", 26: "WristYaw"}.items()})
COLOR = {"against_gravity": "#d62728", "with_gravity": "#1f77b4", "parallel": "#2ca02c"}


def targets_for(step: dict) -> dict[int, float]:
    joints = [int(k) for k in step["samples"][0][1].keys()]
    if step.get("arms") == "right":
        joints = [j for j in joints if j >= 20]  # left arm was only held at its live pose
    t = {j: 0.0 for j in joints}
    for short, value in step["spec"].items():
        name = SHORT[short]
        t[NAME_TO_INDEX["Right" + name]] = value
        li = NAME_TO_INDEX["Left" + name]
        if li in t:
            t[li] = -value if name in MIRROR_NEGATE else value
    return t


def plot_moves(report: dict, out: Path) -> Path:
    steps = [s for s in report["steps"] if s.get("samples")]
    cols = 3
    rows = math.ceil(len(steps) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(6.2 * cols, 3.1 * rows), sharex=False)
    for ax, step in zip(axes.flat, steps):
        t = targets_for(step)
        times = [s[0] for s in step["samples"]]
        for j in sorted(t):
            err = [s[1][str(j)] - t[j] for s in step["samples"]]
            if max(abs(e) for e in err) < 0.02:
                continue  # joints that never moved clutter the plot
            ax.plot(times, err, lw=1.1, label=JOINT_NAMES[j])
        ax.axhspan(-0.01, 0.01, color="0.85", zorder=0)
        if step.get("t_converge") is not None:
            ax.axvline(step["t_converge"], color="k", ls="--", lw=0.9)
            ax.text(step["t_converge"], ax.get_ylim()[1] * 0.92 if ax.get_ylim()[1] > 0 else 0.05, f" hold {step['t_converge']}s", fontsize=7, va="top")
        fails = step.get("fails") or []
        ax.set_title(f"{step['pose']}  [{step['direction']}, dz {step['hand_dz_m']:+.2f} m]" + ("  FAIL" if fails else ""),
                     fontsize=9, color=COLOR.get(step["direction"], "k"))
        ax.set_ylabel("q − target [rad]", fontsize=8)
        ax.set_xlabel("s", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=6, ncol=2, loc="upper right")
    for ax in list(axes.flat)[len(steps):]:
        ax.axis("off")
    fig.suptitle(f"Closed-loop Moves on the H1-2 twin — joint error vs time (grey band = ±0.01 rad tolerance)\n{out.stem}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    path = out.with_name(out.stem + "-moves.png")
    fig.savefig(path, dpi=110)
    return path


def plot_summary(report: dict, out: Path) -> Path:
    steps = [s for s in report["steps"] if s.get("t_converge") is not None or s.get("final_err") is not None]
    names = [s["pose"] for s in steps]
    colors = [COLOR.get(s["direction"], "k") for s in steps]
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    metrics = [("t_converge", "converge time [s]", None), ("final_err", "final error [rad]", 0.02),
               ("overshoot", "overshoot [rad]", 0.06), ("hold_drift", "hold drift [rad]", 0.01)]
    x = range(len(steps))
    for ax, (key, label, limit) in zip(axes, metrics):
        vals = [s.get(key) or 0.0 for s in steps]
        ax.bar(x, vals, color=colors)
        if limit:
            ax.axhline(limit, color="k", ls="--", lw=0.8, label=f"threshold {limit}")
            ax.legend(fontsize=7, loc="upper right")
        ax.set_ylabel(label, fontsize=9)
        ax.grid(axis="y", alpha=0.3)
        for i, s in enumerate(steps):
            if s.get("escalation") and float(s["escalation"]) > 1.0 and key == "t_converge":
                ax.text(i, vals[i], f"esc ×{s['escalation']}", ha="center", va="bottom", fontsize=7, color="red")
    axes[-1].set_xticks(list(x))
    axes[-1].set_xticklabels(names, rotation=40, ha="right", fontsize=8)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in COLOR.values()]
    axes[0].legend(handles, list(COLOR.keys()), fontsize=8, loc="upper left")
    fig.suptitle(f"Workspace sweep summary — {out.stem}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    path = out.with_name(out.stem + "-summary.png")
    fig.savefig(path, dpi=110)
    return path


def plot_sequences(report: dict, out: Path) -> Path | None:
    seqs = [s for s in report.get("sequences", []) if not s.get("error")]
    if not seqs:
        return None
    fig, axes = plt.subplots(1, len(seqs), figsize=(6 * len(seqs), 4))
    axes = list(axes) if len(seqs) > 1 else [axes]
    for ax, s in zip(axes, seqs):
        ax.set_title(f"{s['name']}  playback ×{s['playback_speed']}\nlag {s['lag_s']} s, RMS after lag {s['rms_after_lag']} rad", fontsize=9)
        ax.text(0.02, 0.95, f"{s['joint']}: travel {s['travel_rad']} rad\neffective peak vel {s['effective_peak_vel']} rad/s\n(no samples stored for sequences)",
                transform=ax.transAxes, va="top", fontsize=8)
        ax.axis("off")
    fig.tight_layout()
    path = out.with_name(out.stem + "-sequences.png")
    fig.savefig(path, dpi=110)
    return path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("report", type=Path, help="…-sweep-<label>.json")
    args = ap.parse_args()
    report = json.loads(args.report.read_text())
    out = args.report.with_suffix("")
    paths = [plot_moves(report, out), plot_summary(report, out)]
    seq = plot_sequences(report, out)
    if seq:
        paths.append(seq)
    for p in paths:
        print(p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
