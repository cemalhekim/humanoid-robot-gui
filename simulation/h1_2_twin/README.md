# H1-2 MuJoCo twin

A physics simulation of the Unitree H1-2 that speaks the robot's own DDS
dialect, so the unmodified dashboard (`server.py`) drives it exactly as it
drives the hardware. It is not a viewer of recorded motion: commands go in,
MuJoCo integrates gravity, inertia, joint limits and the motor PD, and the
resulting state comes back on `rt/lowstate` — the closed loop is real.

| Wire | Twin | Real robot |
|---|---|---|
| `rt/lowstate` (`unitree_hg` `LowState_`) | published at 500 Hz: `q`, `dq`, `ddq`, `tau_est`, IMU | ~500 Hz |
| `rt/arm_sdk` (`unitree_hg` `LowCmd_`) | consumed; slots 12–26, slot 27 = weight | arm bridge |
| `rt/lowcmd` (`unitree_hg` `LowCmd_`) | consumed; any slot with `mode != 0` | low-level path |
| motor law | `tau_ff + kp(q_des−q) + kd(dq_des−dq)`, clipped to MJCF `ctrlrange` | motor driver |
| onboard controller | PD hold on legs/waist; arms latch where the last external command left them | Unitree motion mode |
| `LocoClient` RPC | **not implemented** — `/api/loco/*` returns 503 | proprietary |

Model: `execution/semantic_teleoperation/external/unitree_mujoco/unitree_robots/h1_2/scene.xml`
(27 actuators, same joint order as the real `rt/lowstate` table; the twin
verifies the order at start-up and refuses to run on a mismatch). The pelvis
is fixed by default (`--free-base` to drop it) because there is no balance
controller. The upstream MJCF has a pelvis/torso collision overlap at q0; the
loader excludes pairs that are already in contact at q0 so the waist is not
torqued by a phantom contact.

## Run

```bash
# terminal 1 — twin (headless; add --viewer on a machine with a display)
simulation/h1_2_twin/run_h1_2_twin.sh

# terminal 2 — dashboard on the same DDS domain, no camera, no XR suspend
simulation/h1_2_twin/run_server_against_twin.sh          # http://localhost:8088
```

Python needs `mujoco`, `cyclonedds==0.10.2` and the vendored
`unitree_sdk2_python` importable (the run scripts add it to `PYTHONPATH`).
`HANS_SIM_PYTHON` selects the interpreter, `HANS_SIM_PORT` the dashboard port.
On AI-DEV both run as user services from `deployment/sim/`.

## Smoke test (what was verified on 2026-08-25)

`/api/state` → `connected: true`, `sample_rate_hz ≈ 500`, 35 motor slots.
An inline six-joint pose posted to `/api/recording/replay/robot` with
`execute_arm_sdk: true, closed_loop: true` converged in 3.5 s with every joint
inside 0.003 rad; the 120 Hz hold loop measured ≈110 Hz on the twin side.

## Twin-vs-real caveats

- `tau_est` is the clean MuJoCo actuator force. The real motors add friction
  and noise, so hold-phase gains that look perfect here will be optimistic.
- The motor model has torque limits (`ctrlrange`) but no velocity/current
  limits, no backlash, no bus latency beyond DDS on loopback.
- `mode_machine`/CRC are not checked. Do not use the twin to validate framing.
- No walking. Anything through `LocoClient` needs the real robot.

## Flags

`--domain 1 --interface lo --physics-dt 0.002 --state-hz 500 --cmd-timeout 0.5
--hold-kp 300 --hold-kd 6 --arm-hold-kp 120 --arm-hold-kd 2 --realtime 1.0
--viewer --status-every 2 --duration 0 --free-base`
