#!/usr/bin/env python3
"""Generate a fixed, seeded bank of random arm motions for controller evaluation.

Each motion is a start pose and a goal pose for BOTH arms, drawn uniformly
inside JOINT_LIMITS (with a margin), rejected when MuJoCo reports a self-
contact at either pose, and tagged with the goal's gravity direction (hand dz)
and size (max joint travel). A share of the motions are multi-frame
trajectories (smoothstep start->goal over a duration chosen so the native
peak velocity stays under the server's 2.0 rad/s gate).

The bank is deterministic for a seed, so every parameter set in
optimize_params.py is judged on the same motions.

    random_motions.py --count 1000 --seed 7 --out motions-1000.json
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

import mujoco

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))
import h1_2_mujoco_sim as twin  # noqa: E402
import kinematics  # noqa: E402
from sweep_arm_workspace import ARM_JOINTS, JOINT_LIMITS, JOINT_NAMES, classify  # noqa: E402


class PoseChecker:
    def __init__(self) -> None:
        self.model = twin.load_model(twin.DEFAULT_SCENE, True)
        self.data = mujoco.MjData(self.model)
        self.qadr = [int(self.model.jnt_qposadr[self.model.actuator_trnid[i, 0]]) for i in range(self.model.nu)]
        self.kin = kinematics.ArmKinematics()

    def contacts(self, targets: dict[int, float]) -> int:
        self.data.qpos[:] = 0.0
        for i, q in targets.items():
            self.data.qpos[self.qadr[i]] = q
        mujoco.mj_forward(self.model, self.data)
        return int(self.data.ncon)

    def hand_z(self, targets: dict[int, float]) -> float:
        lm = self.kin.landmarks({JOINT_NAMES[i]: q for i, q in targets.items()}, round_digits=None)
        return (lm["left"]["hand"]["z"] + lm["right"]["hand"]["z"]) / 2.0


def random_pose(rng: random.Random, margin: float, checker: PoseChecker, tries: int = 200) -> dict[int, float]:
    for _ in range(tries):
        pose = {}
        for j in ARM_JOINTS:
            lo, hi = JOINT_LIMITS[j]
            span = hi - lo
            pose[j] = rng.uniform(lo + margin * span, hi - margin * span)
        if checker.contacts(pose) == 0:
            return pose
    raise RuntimeError("could not sample a contact-free pose")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--count", type=int, default=1000)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--margin", type=float, default=0.05, help="fraction of each joint range kept clear of the limits")
    ap.add_argument("--trajectory-share", type=float, default=0.3)
    ap.add_argument("--max-native-vel", type=float, default=1.6, help="rad/s peak of generated trajectories (gate is 2.0)")
    ap.add_argument("--out", type=Path, default=HERE / "motions.json")
    args = ap.parse_args()

    rng = random.Random(args.seed)
    checker = PoseChecker()
    motions = []
    rejected = 0
    while len(motions) < args.count:
        try:
            start = random_pose(rng, args.margin, checker)
            goal = random_pose(rng, args.margin, checker)
        except RuntimeError:
            rejected += 1
            continue
        travel = max(abs(goal[j] - start[j]) for j in ARM_JOINTS)
        dz = checker.hand_z(goal) - checker.hand_z(start)
        motion = {
            "id": len(motions),
            "start": {str(j): round(q, 4) for j, q in start.items()},
            "goal": {str(j): round(q, 4) for j, q in goal.items()},
            "travel_rad": round(travel, 3),
            "hand_dz_m": round(dz, 3),
            "direction": classify(dz),
            "kind": "pose",
        }
        if rng.random() < args.trajectory_share:
            # smoothstep peak velocity = 1.5 * travel / duration
            min_duration = 1.5 * travel / args.max_native_vel
            motion["kind"] = "trajectory"
            motion["duration_s"] = round(rng.uniform(max(1.0, min_duration), max(2.0, min_duration * 3.0)), 2)
        motions.append(motion)
    dirs = {}
    for m in motions:
        dirs[m["direction"]] = dirs.get(m["direction"], 0) + 1
    bank = {"seed": args.seed, "count": len(motions), "margin": args.margin, "directions": dirs,
            "trajectories": sum(1 for m in motions if m["kind"] == "trajectory"), "motions": motions}
    args.out.write_text(json.dumps(bank, indent=0), encoding="utf-8")
    print(f"{len(motions)} motions -> {args.out} ({dirs}, {bank['trajectories']} trajectories, {rejected} rejected pairs)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
