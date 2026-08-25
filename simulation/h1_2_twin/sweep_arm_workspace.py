#!/usr/bin/env python3
"""Drive the High Level Controller through the whole arm workspace on the twin.

Runs a chain of closed-loop Moves (``POST /api/recording/replay/robot`` with
``execute_arm_sdk`` + ``closed_loop``, exactly what the Recorder "Move" button
sends) through poses that cover every direction the arm can be loaded in:
hanging, forward-horizontal (max shoulder gravity moment), straight up,
sideways T-pose, behind the body, elbow-loaded, overhead-bent, wrist extremes,
both arms at once. Every transition is classified by the hand's vertical
displacement -- against gravity / with gravity / parallel -- and measured:

    t_converge   seconds until the server reports "holding final pose"
    t_settle     seconds until every commanded joint is within --settle-band
                 and stays there (our own criterion, independent of the server)
    final_err    max |q - target| over commanded joints at the end of the hold
    overshoot    max travel past the target, in the direction of motion
    hold_drift   max |q(end of hold) - q(start of hold)|  (sag / limit cycle)
    tau_peak     max |tau_est| per gain group vs the MJCF torque limit
    escalation / ceiling / fault  from the server's replay status

With ``--sequences`` it also replays generated multi-frame trajectories at
several speeds and reports tracking lag + residual RMS after lag alignment.

Output: a Markdown table + JSON (with the raw 20 Hz samples) under
``simulation/h1_2_twin/reports/``. Exit status 1 when any step misses a
threshold, so it doubles as a regression gate for the controller.

Usage (twin + dashboard already running):
    simulation/h1_2_twin/sweep_arm_workspace.py --url http://127.0.0.1:8088 --sequences
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO))

import kinematics  # noqa: E402
import tracking  # noqa: E402

# Server contract we depend on (kept literal so this tool is readable on its own).
JOINT_NAMES = {
    12: "WaistYaw",
    13: "LeftShoulderPitch", 14: "LeftShoulderRoll", 15: "LeftShoulderYaw", 16: "LeftElbow",
    17: "LeftWristRoll", 18: "LeftWristPitch", 19: "LeftWristYaw",
    20: "RightShoulderPitch", 21: "RightShoulderRoll", 22: "RightShoulderYaw", 23: "RightElbow",
    24: "RightWristRoll", 25: "RightWristPitch", 26: "RightWristYaw",
}
INDEX_BY_NAME = {name: index for index, name in JOINT_NAMES.items()}
JOINT_LIMITS = {
    13: (-3.14, 1.57), 14: (-0.38, 3.4), 15: (-2.66, 3.01), 16: (-0.95, 3.18),
    17: (-3.01, 2.75), 18: (-0.4625, 0.4625), 19: (-1.27, 1.27),
    20: (-3.14, 1.57), 21: (-3.4, 0.38), 22: (-3.01, 2.66), 23: (-0.95, 3.18),
    24: (-2.75, 3.01), 25: (-0.4625, 0.4625), 26: (-1.27, 1.27),
}
ARM_JOINTS = list(range(13, 27))
# MJCF ctrlrange of the twin (Nm) by joint index -- the saturation reference.
TAU_LIMIT = {13: 40, 14: 40, 15: 18, 16: 18, 17: 19, 18: 19, 19: 19,
             20: 40, 21: 40, 22: 18, 23: 18, 24: 19, 25: 19, 26: 19}
GROUP = {13: "shoulder", 14: "shoulder", 15: "shoulder", 16: "elbow", 17: "wrist", 18: "wrist", 19: "wrist",
         20: "shoulder", 21: "shoulder", 22: "shoulder", 23: "elbow", 24: "wrist", 25: "wrist", 26: "wrist"}
# Joints whose sign flips between the right and the mirrored left arm.
MIRROR_NEGATE = {"ShoulderRoll", "ShoulderYaw", "WristRoll", "WristYaw"}

# Right-arm poses (radians). SP shoulder pitch (0 = hanging, negative = forward/up),
# SR shoulder roll (negative = out to the side for the right arm), SY shoulder yaw,
# E elbow (0 = straight, positive = flexed), WR/WP/WY wrist roll/pitch/yaw.
RIGHT_POSES: dict[str, dict[str, float]] = {
    # A little abduction (SR -0.3) keeps the two arms clear of each other and of the
    # head/pelvis in the forward, overhead and elbow-loaded poses; it does not change
    # the gravity loading those poses are there to test.
    "hang":                     {},
    "forward_45":               {"SP": -0.8, "SR": -0.3},
    "forward_horizontal":       {"SP": -1.57, "SR": -0.3},
    "straight_up":              {"SP": -3.0, "SR": -0.45},
    "forward_horizontal_2":     {"SP": -1.57, "SR": -0.3},
    "side_horizontal_T":        {"SR": -1.5},
    "side_45":                  {"SR": -0.8},
    "hang_2":                   {},
    "elbow_forearm_forward":    {"SP": -0.3, "SR": -0.35, "E": 1.5},
    "curl_forearm_up":          {"SP": -1.57, "SR": -0.35, "E": 1.57},
    "forward_elbow_45":         {"SP": -1.57, "SR": -0.3, "E": 0.8},
    "overhead_bent":            {"SP": -2.5, "SR": -0.5, "E": 1.0},
    "backward":                 {"SP": 1.0},
    "low_side_back":            {"SP": 0.8, "SR": -0.8},
    "yaw_in_forward":           {"SP": -1.2, "SR": -0.3, "SY": -1.2, "E": 0.6},
    # SR +0.3 makes the shoulder-roll link hit the torso in MuJoCo (contact from +0.2 on); the
    # editor allows +0.38 but +0.15 is the physical adduction limit at this pitch/elbow.
    "across_body":              {"SP": -1.2, "SR": 0.15, "E": 1.2, "_right_only": 1},
    "wrist_extremes":           {"SP": -1.0, "SR": -0.45, "E": 1.0, "WR": 1.5, "WP": 0.4, "WY": 1.0},
    "hang_final":               {},
}
SHORT = {"SP": "ShoulderPitch", "SR": "ShoulderRoll", "SY": "ShoulderYaw", "E": "Elbow",
         "WR": "WristRoll", "WP": "WristPitch", "WY": "WristYaw"}


def pose_targets(spec: dict[str, float], both_arms: bool) -> dict[int, float]:
    """Expand a right-arm spec to joint-index targets (mirrored to the left arm)."""
    both_arms = both_arms and not spec.get("_right_only")
    targets = {index: 0.0 for index in ARM_JOINTS} if both_arms else {index: 0.0 for index in range(20, 27)}
    for short, value in spec.items():
        if short.startswith("_"):
            continue
        joint = SHORT[short]
        targets[INDEX_BY_NAME["Right" + joint]] = value
        if both_arms:
            targets[INDEX_BY_NAME["Left" + joint]] = -value if joint in MIRROR_NEGATE else value
    for index, value in targets.items():
        lo, hi = JOINT_LIMITS[index]
        if not (lo <= value <= hi):
            raise ValueError(f"{JOINT_NAMES[index]}={value} outside {JOINT_LIMITS[index]}")
    return targets


class Dashboard:
    def __init__(self, url: str) -> None:
        self.url = url.rstrip("/")

    def get(self, path: str):
        return json.load(urllib.request.urlopen(self.url + path, timeout=5))

    def post(self, path: str, body: dict) -> tuple[int, dict]:
        req = urllib.request.Request(
            self.url + path, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"}, method="POST"
        )
        try:
            resp = urllib.request.urlopen(req, timeout=15)
            return resp.status, json.load(resp)
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read() or b"{}")

    def motors(self) -> list[dict]:
        return self.get("/api/state")["motors"]

    def joint_q(self) -> tuple[float, dict[int, float], dict[int, float]]:
        motors = self.motors()
        return time.time(), {m["index"]: m["q"] for m in motors[:27]}, {m["index"]: m.get("tau_est", 0.0) for m in motors[:27]}

    def replay_status(self) -> dict:
        w = self.get("/api/wrist/status")
        lc = w.get("last_command") or {}
        cl = lc.get("closed_loop") or {}
        return {
            "active": w.get("active"),
            "message": w.get("message"),
            "holding": lc.get("holding_final_pose"),
            "converged": cl.get("converged"),
            "ceiling": cl.get("ceiling_reached"),
            "fault": cl.get("fault_reason"),
            "escalation": cl.get("escalation"),
            "writes": lc.get("writes"),
        }


def hand_height(kin: kinematics.ArmKinematics, targets: dict[int, float]) -> float:
    angles = {JOINT_NAMES[i]: q for i, q in targets.items()}
    lm = kin.landmarks(angles, round_digits=None)
    sides = [s for s in ("left", "right") if any(i in targets for i in (range(13, 20) if s == "left" else range(20, 27)))]
    return sum(lm[s]["hand"]["z"] for s in sides) / len(sides)


def classify(dz: float, band: float = 0.08) -> str:
    if dz > band:
        return "against_gravity"
    if dz < -band:
        return "with_gravity"
    return "parallel"


def run_move(dash: Dashboard, targets: dict[int, float], args, log) -> dict:
    """Execute one closed-loop Move and sample it. Returns metrics + samples."""
    motors = dash.motors()
    snapshot = [{"index": m["index"], "name": m["name"], "q": targets.get(m["index"], m["q"])} for m in motors[:27]]
    t0, q0, _ = dash.joint_q()
    code, res = dash.post("/api/recording/replay/robot", {
        "snapshot": {"motors": snapshot}, "command_scope": "arms",
        "execute_arm_sdk": True, "closed_loop": True, "hold_after_convergence": True,
        "replay_response": args.response, "armed": True, "i_understand_risk": True,
    })
    if code != 202:
        return {"error": f"{code}: {res.get('error') or res.get('message')}", "plan": res.get("plan")}
    t_start = time.time()
    samples: list[tuple[float, dict[int, float], dict[int, float]]] = []
    status = dash.replay_status()
    t_hold_reported = None
    t_settle = None
    settle_since = None
    deadline = t_start + args.timeout
    hold_until = None
    next_status = 0.0
    period = 1.0 / args.rate
    while True:
        now = time.time()
        t, q, tau = dash.joint_q()
        samples.append((t - t_start, {i: q[i] for i in targets}, {i: tau[i] for i in targets}))
        err = max(abs(q[i] - v) for i, v in targets.items())
        if err <= args.settle_band:
            settle_since = settle_since or (t - t_start)
            if t_settle is None and (t - t_start) - settle_since >= 0.5:
                t_settle = settle_since
        else:
            settle_since = None
        if now >= next_status:
            status = dash.replay_status()
            next_status = now + 0.25
            if t_hold_reported is None and status["holding"]:
                t_hold_reported = now - t_start
                hold_until = now + args.hold_seconds
            if not status["active"] and (status["fault"] or status["ceiling"]):
                break
        if hold_until is not None and now >= hold_until:
            break
        if now >= deadline:
            break
        time.sleep(max(0.0, period - (time.time() - now)))
    # ---- metrics
    final = samples[-1]
    final_err_by = {i: final[1][i] - v for i, v in targets.items()}
    final_err = max(abs(e) for e in final_err_by.values())
    overshoot = 0.0
    for i, v in targets.items():
        direction = math.copysign(1.0, v - q0[i]) if abs(v - q0[i]) > 1e-3 else 0.0
        if direction:
            overshoot = max(overshoot, max(direction * (s[1][i] - v) for s in samples))
    drift = 0.0
    post_hold_settle = 0.0
    if t_hold_reported is not None:
        early = [s for s in samples if t_hold_reported <= s[0] < t_hold_reported + 1.5]
        steady = [s for s in samples if s[0] >= t_hold_reported + 1.5]
        if len(early) > 2:
            post_hold_settle = max(abs(early[-1][1][i] - early[0][1][i]) for i in targets)
        if len(steady) > 2:
            drift = max(abs(steady[-1][1][i] - steady[0][1][i]) for i in targets)
    tau_peak = {}
    for i in targets:
        peak = max(abs(s[2][i]) for s in samples)
        g = GROUP[i]
        tau_peak[g] = max(tau_peak.get(g, 0.0), peak / TAU_LIMIT[i])
    # Motion smoothness: velocity residual after a 0.5 s moving average and the number
    # of velocity reversals while moving (a clean smoothstep has none). Hold shake: the
    # std of q in the steady part of the hold (after the first 1.5 s).
    moving = [s for s in samples if t_hold_reported is None or s[0] < t_hold_reported]
    vel_ripple, reversals, ripple_joint = 0.0, 0, None
    for i in targets:
        r, n = _velocity_ripple([s[1][i] for s in moving], args.rate)
        if r > vel_ripple:
            vel_ripple, ripple_joint = r, i
        reversals = max(reversals, n)
    hold_shake = 0.0
    if t_hold_reported is not None:
        steady = [s for s in samples if s[0] >= t_hold_reported + 1.5]
        if len(steady) > 3:
            hold_shake = max(_pstdev([s[1][i] for s in steady]) for i in targets)
    return {
        "t_converge": None if t_hold_reported is None else round(t_hold_reported, 2),
        "t_settle": None if t_settle is None else round(t_settle, 2),
        "final_err": round(final_err, 4),
        "worst_joint": JOINT_NAMES[max(final_err_by, key=lambda k: abs(final_err_by[k]))],
        "overshoot": round(overshoot, 4),
        "hold_drift": round(drift, 4),
        "post_hold_settle": round(post_hold_settle, 4),
        "tau_frac": {g: round(v, 2) for g, v in tau_peak.items()},
        "vel_ripple": round(vel_ripple, 4),
        "reversals": reversals,
        "ripple_joint": JOINT_NAMES.get(ripple_joint, None),
        "hold_shake": round(hold_shake, 5),
        "escalation": status["escalation"],
        "ceiling": bool(status["ceiling"]),
        "fault": status["fault"],
        "writes": status["writes"],
        "samples": [(round(s[0], 3), {str(i): round(v, 4) for i, v in s[1].items()}) for s in samples] if args.samples else None,
    }


def _pstdev(values: list[float]) -> float:
    n = len(values)
    if n < 2:
        return 0.0
    mean = sum(values) / n
    return math.sqrt(sum((v - mean) ** 2 for v in values) / n)


def _velocity_ripple(series: list[float], rate_hz: float, window: int = 10) -> tuple[float, int]:
    """(std of the velocity residual after a centred moving average, velocity reversals)."""
    if len(series) < window + 2:
        return 0.0, 0
    v = [(series[i + 1] - series[i]) * rate_hz for i in range(len(series) - 1)]
    residual = []
    for i in range(len(v)):
        lo, hi = max(0, i - window // 2), min(len(v), i + window // 2 + 1)
        residual.append(v[i] - sum(v[lo:hi]) / (hi - lo))
    reversals = sum(1 for i in range(1, len(v)) if v[i] * v[i - 1] < 0 and abs(v[i]) > 0.03)
    return _pstdev(residual), reversals


def verdict(m: dict, args) -> list[str]:
    fails = []
    if m.get("error"):
        return [m["error"]]
    if m["t_converge"] is None:
        fails.append("never reported holding")
    elif m["t_converge"] > args.max_converge:
        fails.append(f"converge {m['t_converge']}s > {args.max_converge}s")
    if m["final_err"] > args.max_final_err:
        fails.append(f"final_err {m['final_err']} > {args.max_final_err}")
    if m["overshoot"] > args.max_overshoot:
        fails.append(f"overshoot {m['overshoot']} > {args.max_overshoot}")
    if m["hold_drift"] > args.max_drift:
        fails.append(f"drift {m['hold_drift']} > {args.max_drift}")
    if m.get("reversals", 0) > args.max_reversals:
        fails.append(f"oscillation: {m['reversals']} reversals on {m.get('ripple_joint')}")
    if m.get("hold_shake", 0.0) > args.max_shake:
        fails.append(f"hold shake {m['hold_shake']} > {args.max_shake}")
    if m["ceiling"]:
        fails.append("ceiling reached")
    if m["fault"]:
        fails.append(f"fault {m['fault']}")
    if m["escalation"] and float(m["escalation"]) > 1.0:
        fails.append(f"escalation {m['escalation']}")
    if any(v >= 0.95 for v in m["tau_frac"].values()):
        fails.append("torque saturation")
    return fails


def smoothstep(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)


def run_sequence(dash: Dashboard, start: dict[int, float], end: dict[int, float], duration: float, args, log) -> dict:
    """Replay a generated trajectory start->end and measure tracking lag / residual."""
    # 30 Hz frames with only the commanded joints: the server densifies sparse frames
    # itself (TRAJECTORY_DENSE_MAX_DT = 1/30) and caps request bodies at 1 MB.
    dt = 1.0 / 30.0
    n = max(2, int(round(duration / dt)))
    frames = []
    for k in range(n + 1):
        a = smoothstep(k / n)
        frames.append({"timestamp": round(k * dt, 5), "motors": [
            {"index": j, "q": round(start[j] + a * (end[j] - start[j]), 5)} for j in sorted(end)
        ]})
    code, plan = dash.post("/api/recording/replay/robot", {"points": frames, "command_scope": "arms", "dry_run": True})
    p = plan.get("plan", {})
    if code != 200 or not p.get("valid_for_execution"):
        return {"error": f"plan invalid ({code}): {plan.get('error') or p.get('violations') or p.get('reason')}", "plan": p}
    t0 = time.time()
    code, res = dash.post("/api/recording/replay/robot", {
        "points": frames, "command_scope": "arms", "execute_arm_sdk": True, "closed_loop": True,
        "hold_after_convergence": True, "replay_response": args.response, "armed": True, "i_understand_risk": True,
    })
    if code != 202:
        return {"error": f"{code}: {res.get('error')}"}
    speed = float((res.get("closed_loop") or {}).get("tuning", {}).get("playback_speed", 1.0) or 1.0)
    approach_s = float(res.get("approach_frame_count", 0) or 0) * dt / speed
    t_start = time.time()
    samples = []
    hold_at = None
    next_status = 0.0
    while time.time() - t_start < duration + args.timeout:
        now = time.time()
        t, q, _ = dash.joint_q()
        samples.append((t - t_start, {i: q[i] for i in end}))
        if now >= next_status:
            st = dash.replay_status()
            next_status = now + 0.25
            if st["holding"] and hold_at is None:
                hold_at = now - t_start
            if hold_at is not None and now - t_start >= hold_at + 1.0:
                break
        time.sleep(max(0.0, 1.0 / args.rate - (time.time() - now)))
    # Lag alignment on the joint with the largest travel: find the shift that minimises RMS.
    joint = max(end, key=lambda i: abs(end[i] - start[i]))
    travel = abs(end[joint] - start[joint])

    def commanded(t: float) -> float:
        # The server prepends a velocity-bounded approach (start pose -> frame 0, here a
        # no-op) and plays the frames at tuning.playback_speed x native timing.
        return start[joint] + smoothstep((t - approach_s) * speed / duration) * (end[joint] - start[joint])

    best = None
    for shift in [x * 0.05 for x in range(-40, int(15 / 0.05))]:
        rms = math.sqrt(sum((s[1][joint] - commanded(s[0] - shift)) ** 2 for s in samples) / len(samples))
        if best is None or rms < best[1]:
            best = (shift, rms)
    raw_rms = math.sqrt(sum((s[1][joint] - commanded(s[0])) ** 2 for s in samples) / len(samples))
    return {
        "duration": duration, "joint": JOINT_NAMES[joint], "travel_rad": round(travel, 3),
        "playback_speed": round(speed, 3), "approach_s": round(approach_s, 2),
        "peak_cmd_vel": round(1.5 * travel / duration, 3), "effective_peak_vel": round(1.5 * travel / duration * speed, 3),
        "plan_max_vel": p.get("max_velocity_rad_s"),
        "lag_s": round(best[0], 2), "rms_after_lag": round(best[1], 4), "rms_raw": round(raw_rms, 4),
        "t_hold": None if hold_at is None else round(hold_at, 2),
        "final_err": round(max(abs(samples[-1][1][i] - v) for i, v in end.items()), 4),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="http://127.0.0.1:8088")
    ap.add_argument("--response", type=float, default=2.5, help="replay_response dial (UI default 2.5)")
    ap.add_argument("--right-only", action="store_true", help="move only the right arm")
    ap.add_argument("--only", nargs="*", help="pose names to run (chain order kept)")
    ap.add_argument("--sequences", action="store_true", help="also run trajectory-tracking replays")
    ap.add_argument("--rate", type=float, default=20.0, help="sampling rate of /api/state")
    ap.add_argument("--hold-seconds", type=float, default=4.0)
    ap.add_argument("--timeout", type=float, default=45.0)
    ap.add_argument("--settle-band", type=float, default=0.02)
    ap.add_argument("--max-converge", type=float, default=15.0)
    ap.add_argument("--max-final-err", type=float, default=0.02)
    ap.add_argument("--max-overshoot", type=float, default=0.06)
    ap.add_argument("--max-drift", type=float, default=0.01)
    ap.add_argument("--max-reversals", type=int, default=3, help="velocity reversals while moving (with-gravity oscillation)")
    ap.add_argument("--max-shake", type=float, default=0.004, help="std of q during the steady hold [rad]")
    ap.add_argument("--no-samples", dest="samples", action="store_false")
    ap.add_argument("--out", type=Path, default=HERE / "reports")
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    dash = Dashboard(args.url)
    state = dash.get("/api/state")
    if not state.get("connected"):
        print(f"dashboard at {args.url} is not connected to a robot/twin", file=sys.stderr)
        return 2
    kin = kinematics.ArmKinematics()
    both = not args.right_only
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    args.out.mkdir(parents=True, exist_ok=True)
    log = lambda msg: print(msg, flush=True)  # noqa: E731

    names = [n for n in RIGHT_POSES if not args.only or n in args.only]
    _, current, _ = dash.joint_q()
    prev_targets = {i: current[i] for i in (ARM_JOINTS if both else range(20, 27))}
    prev_z = hand_height(kin, prev_targets)
    steps = []
    for name in names:
        targets = pose_targets(RIGHT_POSES[name], both)
        if both and RIGHT_POSES[name].get("_right_only"):
            _, live, _ = dash.joint_q()
            targets.update({i: live[i] for i in range(13, 20)})
        collision = tracking.mimic_pose_collides({**{i: 0.0 for i in ARM_JOINTS}, **targets})
        z = hand_height(kin, targets)
        direction = classify(z - prev_z)
        step = {"pose": name, "spec": {k: v for k, v in RIGHT_POSES[name].items() if not k.startswith("_")},
                "arms": "right" if RIGHT_POSES[name].get("_right_only") or not both else "both",
                "direction": direction, "hand_dz_m": round(z - prev_z, 3)}
        if collision:
            step.update({"skipped": f"self-collision: {collision}"})
            log(f"[{name}] SKIPPED {collision}")
            steps.append(step)
            continue
        log(f"[{name}] {direction} (hand dz {z - prev_z:+.2f} m) ...")
        m = run_move(dash, targets, args, log)
        fails = verdict(m, args)
        step.update(m)
        step["fails"] = fails
        log(f"    converge={m.get('t_converge')}s settle={m.get('t_settle')}s final_err={m.get('final_err')} "
            f"overshoot={m.get('overshoot')} drift={m.get('hold_drift')} shake={m.get('hold_shake')} ripple={m.get('vel_ripple')}/{m.get('reversals')} tau={m.get('tau_frac')} "
            f"esc={m.get('escalation')} {'FAIL ' + '; '.join(fails) if fails else 'ok'}")
        steps.append(step)
        prev_targets, prev_z = targets, z

    sequences = []
    if args.sequences:
        hang = pose_targets({}, both)
        goal = pose_targets({"SP": -1.57, "E": 0.8}, both)
        for duration in (8.0, 4.0, 2.0):
            log(f"[sequence {duration}s] hang -> forward_elbow_45 ...")
            run_move(dash, hang, args, log)  # start from a known pose
            r = run_sequence(dash, hang, goal, duration, args, log)
            r["name"] = f"hang->forward_elbow_45 {duration}s"
            log(f"    {r}")
            sequences.append(r)
        run_move(dash, hang, args, log)

    # ---- report
    by_dir: dict[str, list] = {}
    for s in steps:
        if "fails" in s:
            by_dir.setdefault(s["direction"], []).append(s)
    lines = [f"# Arm workspace sweep {stamp} {args.label}", "",
             f"url `{args.url}` · response {args.response} · {'both arms' if both else 'right arm'} · "
             f"thresholds converge≤{args.max_converge}s err≤{args.max_final_err} overshoot≤{args.max_overshoot} drift≤{args.max_drift}", "",
             "| # | pose | arms | direction | dz m | converge s | settle s | final err rad | worst joint | overshoot | post-hold settle | drift | shake | ripple / reversals | tau sh/el/wr | esc | verdict |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for k, s in enumerate(steps, 1):
        if "skipped" in s:
            lines.append(f"| {k} | {s['pose']} | {s.get('arms')} | {s['direction']} | {s['hand_dz_m']} | — | — | — | — | — | — | — | — | — | — | — | skipped: {s['skipped']} |")
            continue
        if s.get("error"):
            lines.append(f"| {k} | {s['pose']} | {s.get('arms')} | {s['direction']} | {s['hand_dz_m']} | — | — | — | — | — | — | — | — | — | — | — | ERROR {s['error']} |")
            continue
        tf = s["tau_frac"]
        lines.append(f"| {k} | {s['pose']} | {s.get('arms')} | {s['direction']} | {s['hand_dz_m']:+.2f} | {s['t_converge']} | {s['t_settle']} | {s['final_err']} | {s['worst_joint']} | "
                     f"{s['overshoot']} | {s['post_hold_settle']} | {s['hold_drift']} | {s.get('hold_shake')} | {s.get('vel_ripple')} / {s.get('reversals')} ({s.get('ripple_joint')}) | {tf.get('shoulder', 0)}/{tf.get('elbow', 0)}/{tf.get('wrist', 0)} | {s['escalation']} | "
                     f"{'FAIL: ' + '; '.join(s['fails']) if s['fails'] else 'ok'} |")
    lines += ["", "## By direction", "", "| direction | moves | fails | mean converge s | max final err | max overshoot | max drift |", "|---|---|---|---|---|---|---|"]
    for d, group in by_dir.items():
        ok = [g for g in group if not g.get("error")]
        conv = [g["t_converge"] for g in ok if g["t_converge"] is not None]
        lines.append(f"| {d} | {len(group)} | {sum(1 for g in group if g['fails'])} | {round(sum(conv) / len(conv), 2) if conv else '—'} | "
                     f"{max((g['final_err'] for g in ok), default='—')} | {max((g['overshoot'] for g in ok), default='—')} | {max((g['hold_drift'] for g in ok), default='—')} |")
    if sequences:
        lines += ["", "## Trajectory tracking", "", "| sequence | travel rad | native peak vel | playback x | effective peak vel | lag s | rms after lag | rms raw | hold at s | final err |", "|---|---|---|---|---|---|---|---|---|---|"]
        for r in sequences:
            if r.get("error"):
                lines.append(f"| {r['name']} | ERROR {r['error']} |")
            else:
                lines.append(f"| {r['name']} | {r['travel_rad']} | {r['peak_cmd_vel']} | {r['playback_speed']} | {r['effective_peak_vel']} | {r['lag_s']} | {r['rms_after_lag']} | {r['rms_raw']} | {r['t_hold']} | {r['final_err']} |")
    total_fail = sum(1 for s in steps if s.get("fails"))
    lines += ["", f"**{len([s for s in steps if 'fails' in s])} moves, {total_fail} failed, {sum(1 for s in steps if 'skipped' in s)} skipped.**"]
    md = "\n".join(lines) + "\n"
    base = args.out / f"{stamp}-sweep{('-' + args.label) if args.label else ''}"
    base.with_suffix(".md").write_text(md, encoding="utf-8")
    base.with_suffix(".json").write_text(json.dumps({"args": vars(args) | {"out": str(args.out)}, "steps": steps, "sequences": sequences}, indent=1), encoding="utf-8")
    print("\n" + md)
    print(f"report: {base.with_suffix('.md')}")
    return 1 if total_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
