# Tuning files for the arm-replay controller

Loaded through `RTW_TUNING_JSON` (see `_apply_tuning_overrides` in `server.py`); unset = shipped constants.
On the robot: `deployment/enable_tuning.sh deployment/tuning/<file>.json` writes a systemd drop-in and restarts the
dashboard service; `deployment/enable_tuning.sh off` returns to the shipped constants. Only with a spotter at the robot.

| File | What | Twin evidence (2026-08-26) |
|---|---|---|
| `round2-75.json` | CMA-ES round-2 winner #75, control law unchanged (variant A) | 300 motions: 68 oscillating -> 0, mean cost 5.5 -> 3.3 |
| `round2-75-b1.json` | #75 + URDF gravity-model feed-forward (B1), hands included for the robot | 60 motions: final error 0.0034 -> 0.0019 rad, shake /2.5, no reversals |
| `round2-75-b1-gated.json` | B1 + learn only while the whole arm is still + leak 0.5/s | **recommended**: 300 motions fitness 6.98, 0 osc/fail, error 0.0019 rad, static 120 s holds; most robust under 30 ms latency + 1 Nm friction (0 fails, error 0.0075) |
| `round2b-70-b1-gated.json` | CMA-ES re-search on the B1-gated structure | 300 motions fitness 6.75, error 0.0013 rad on the ideal twin, but degrades more under latency+friction (error 0.0135) — overfit to the ideal twin, prefer the gated file on hardware |

All numbers are from the MuJoCo twin; the robot has friction, backlash and Inspire hands the twin does not.
Start with single poses at low response, compare against `off` with `simulation/h1_2_twin/sweep_arm_workspace.py --url <robot>`.
