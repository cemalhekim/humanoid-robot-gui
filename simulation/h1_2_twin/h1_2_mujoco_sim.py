#!/usr/bin/env python3
"""Unitree H1-2 MuJoCo twin that speaks the real robot's DDS dialect.

Publishes ``rt/lowstate`` (``unitree_hg`` ``LowState_``) and consumes
``rt/lowcmd`` and ``rt/arm_sdk`` (``unitree_hg`` ``LowCmd_``), so ``server.py``
talks to it exactly as it talks to the robot -- same joint order, same 35-slot
messages, same arm_sdk weight slot 27. Headless by default; the dashboard's
blue URDF twin is the visualiser. ``--viewer`` opens a local MuJoCo window.

Motor model: per joint ``tau = tau_ff + kp*(q_des - q) + kd*(dq_des - dq)``
(the same law the upstream ``unitree_sdk2py_bridge`` uses), clipped to the
MJCF ``ctrlrange``. Command priority per joint, evaluated every physics step:

1. ``rt/lowcmd`` motor with ``mode != 0`` (fresh)      -> full authority
2. ``rt/arm_sdk`` motor 13..26 with ``mode != 0``       -> blended by slot-27 weight
3. otherwise the "onboard controller" stand-in         -> PD hold on the latched pose

The stand-in mirrors what the real H1-2 does in its normal motion mode: legs
and waist are held, and arms hold wherever they were when the last external
command went stale. There is no balance controller; use ``--fix-base`` (the
default) for arm work.
"""
from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from pathlib import Path

import mujoco

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
except ImportError as exc:  # pragma: no cover
    print(f"unitree_sdk2py is required (pip install -e external/unitree_sdk2_python): {exc}", file=sys.stderr)
    raise SystemExit(2)

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
DEFAULT_SCENE = (
    REPO / "execution/semantic_teleoperation/external/unitree_mujoco/unitree_robots/h1_2/scene.xml"
)

# Real rt/lowstate motor order (docs/robot_control_paths.md) expressed as MJCF joint names.
EXPECTED_JOINTS = [
    "left_hip_yaw_joint", "left_hip_pitch_joint", "left_hip_roll_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_yaw_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "torso_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
NUM_MOTORS = len(EXPECTED_JOINTS)  # 27
# rt/arm_sdk drives arms only (13-26). WaistYaw (12) is NOT an arm_sdk joint on the H1-2:
# it belongs to the onboard controller like the legs and moves only via rt/lowcmd.
ARM_SDK_JOINTS = list(range(13, 27))
ARM_SDK_WEIGHT_SLOT = 27
NUM_SLOTS = 35
BODY_JOINTS = list(range(0, 13))  # legs + waist: held by the onboard-controller stand-in
ARM_JOINTS = list(range(13, 27))  # latch where the last external command left them


def load_model(scene: Path, fix_base: bool) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(str(scene))
    if fix_base:
        # Delete the free joint so the pelvis is welded to the world at its model pose.
        removed = 0
        for body in spec.bodies:
            for joint in list(body.joints):
                if joint.type == mujoco.mjtJoint.mjJNT_FREE:
                    spec.delete(joint)
                    removed += 1
        if removed != 1:
            print(f"warning: expected one free joint, removed {removed}", file=sys.stderr)
    model = spec.compile()
    # Upstream's H1-2 MJCF ships collision geoms that already overlap at q0 (pelvis vs
    # torso_link); that phantom contact torques the waist. Exclude exactly those pairs.
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    pairs = set()
    for c in data.contact[: data.ncon]:
        b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[c.geom1])
        b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[c.geom2])
        if b1 and b2 and b1 != b2:
            pairs.add(tuple(sorted((b1, b2))))
    if pairs:
        for b1, b2 in sorted(pairs):
            spec.add_exclude(bodyname1=b1, bodyname2=b2)
        model = spec.compile()
        print(f"excluded initial-overlap contact pairs: {sorted(pairs)}", file=sys.stderr)
    return model


class H12Twin:
    def __init__(self, model: mujoco.MjModel, args: argparse.Namespace) -> None:
        self.model = model
        self.data = mujoco.MjData(model)
        self.args = args
        self.lock = threading.Lock()
        if model.nu != NUM_MOTORS:
            raise SystemExit(f"scene has {model.nu} actuators, expected {NUM_MOTORS}")
        self.qadr = []
        self.dadr = []
        for i in range(model.nu):
            jid = model.actuator_trnid[i, 0]
            jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if jname != EXPECTED_JOINTS[i]:
                raise SystemExit(f"actuator {i} drives {jname}, expected {EXPECTED_JOINTS[i]}")
            self.qadr.append(int(model.jnt_qposadr[jid]))
            self.dadr.append(int(model.jnt_dofadr[jid]))
        self.imu_quat = self._sensor_adr(mujoco.mjtSensor.mjSENS_FRAMEQUAT)
        self.imu_gyro = self._sensor_adr(mujoco.mjtSensor.mjSENS_GYRO)
        self.imu_acc = self._sensor_adr(mujoco.mjtSensor.mjSENS_ACCELEROMETER)
        self.pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")

        mujoco.mj_forward(model, self.data)
        self.hold_q = [float(self.data.qpos[a]) for a in self.qadr]
        self.lowcmd: LowCmd_ | None = None
        self.lowcmd_at = 0.0
        self.lowcmd_count = 0
        self.armcmd: LowCmd_ | None = None
        self.armcmd_at = 0.0
        self.armcmd_count = 0
        self.arm_external_prev = [False] * NUM_MOTORS
        self.latency = max(0.0, float(getattr(args, "cmd_latency_ms", 0.0) or 0.0)) / 1000.0
        self.lowcmd_queue: list[tuple[float, LowCmd_]] = []
        self.armcmd_queue: list[tuple[float, LowCmd_]] = []
        self.state_msg = unitree_hg_msg_dds__LowState_()
        self.tick = 0

        self.state_pub = ChannelPublisher("rt/lowstate", LowState_)
        self.state_pub.Init()
        self.lowcmd_sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self.lowcmd_sub.Init(self._on_lowcmd, 10)
        self.armcmd_sub = ChannelSubscriber("rt/arm_sdk", LowCmd_)
        self.armcmd_sub.Init(self._on_armcmd, 10)

    def _sensor_adr(self, kind: int) -> int | None:
        for i in range(self.model.nsensor):
            if self.model.sensor_type[i] == kind:
                return int(self.model.sensor_adr[i])
        return None

    def _on_lowcmd(self, msg: LowCmd_) -> None:
        with self.lock:
            self.lowcmd_count += 1
            if self.latency > 0.0:
                self.lowcmd_queue.append((time.monotonic() + self.latency, msg))
            else:
                self.lowcmd = msg
                self.lowcmd_at = time.monotonic()

    def _on_armcmd(self, msg: LowCmd_) -> None:
        with self.lock:
            self.armcmd_count += 1
            if self.latency > 0.0:
                self.armcmd_queue.append((time.monotonic() + self.latency, msg))
            else:
                self.armcmd = msg
                self.armcmd_at = time.monotonic()

    def _drain_queues(self, now: float) -> None:
        """Promote delayed commands whose delivery time has come (called under the lock)."""
        while self.lowcmd_queue and self.lowcmd_queue[0][0] <= now:
            _, self.lowcmd = self.lowcmd_queue.pop(0)
            self.lowcmd_at = now
        while self.armcmd_queue and self.armcmd_queue[0][0] <= now:
            _, self.armcmd = self.armcmd_queue.pop(0)
            self.armcmd_at = now

    @staticmethod
    def _pd(cmd, q: float, dq: float) -> float:
        return float(cmd.tau) + float(cmd.kp) * (float(cmd.q) - q) + float(cmd.kd) * (float(cmd.dq) - dq)

    def control(self) -> None:
        now = time.monotonic()
        stale = self.args.cmd_timeout
        with self.lock:
            if self.latency > 0.0:
                self._drain_queues(now)
            lowcmd = self.lowcmd if (now - self.lowcmd_at) < stale else None
            armcmd = self.armcmd if (now - self.armcmd_at) < stale else None
        weight = 0.0
        if armcmd is not None:
            weight = max(0.0, min(1.0, float(armcmd.motor_cmd[ARM_SDK_WEIGHT_SLOT].q)))
        hold_kp, hold_kd = self.args.hold_kp, self.args.hold_kd
        arm_hold_kp, arm_hold_kd = self.args.arm_hold_kp, self.args.arm_hold_kd
        for i in range(NUM_MOTORS):
            q = float(self.data.qpos[self.qadr[i]])
            dq = float(self.data.qvel[self.dadr[i]])
            external = False
            tau = 0.0
            if lowcmd is not None and int(lowcmd.motor_cmd[i].mode) != 0:
                tau = self._pd(lowcmd.motor_cmd[i], q, dq)
                external = True
            elif armcmd is not None and i in ARM_SDK_JOINTS and weight > 0.0 and int(armcmd.motor_cmd[i].mode) != 0:
                kp, kd = (arm_hold_kp, arm_hold_kd) if i in ARM_JOINTS else (hold_kp, hold_kd)
                hold = kp * (self.hold_q[i] - q) - kd * dq
                tau = weight * self._pd(armcmd.motor_cmd[i], q, dq) + (1.0 - weight) * hold
                external = True
            else:
                kp, kd = (arm_hold_kp, arm_hold_kd) if i in ARM_JOINTS else (hold_kp, hold_kd)
                tau = kp * (self.hold_q[i] - q) - kd * dq
            # Latch the hold pose for upper-body joints the moment external authority ends,
            # like the onboard controller holding the arms where arm_sdk left them.
            if self.arm_external_prev[i] and not external and i in ARM_JOINTS:
                self.hold_q[i] = q
            self.arm_external_prev[i] = external
            self.data.ctrl[i] = tau

    def publish_state(self) -> None:
        msg = self.state_msg
        for i in range(NUM_MOTORS):
            m = msg.motor_state[i]
            m.mode = 1
            m.q = float(self.data.qpos[self.qadr[i]])
            m.dq = float(self.data.qvel[self.dadr[i]])
            m.ddq = float(self.data.qacc[self.dadr[i]])
            m.tau_est = float(self.data.actuator_force[i])
            m.temperature[0] = 35
            m.temperature[1] = 35
        imu = msg.imu_state
        sd = self.data.sensordata
        if self.imu_quat is not None:
            imu.quaternion[:] = [float(v) for v in sd[self.imu_quat:self.imu_quat + 4]]
        else:
            imu.quaternion[:] = [float(v) for v in self.data.xquat[self.pelvis_id]]
        if self.imu_gyro is not None:
            imu.gyroscope[:] = [float(v) for v in sd[self.imu_gyro:self.imu_gyro + 3]]
        if self.imu_acc is not None:
            imu.accelerometer[:] = [float(v) for v in sd[self.imu_acc:self.imu_acc + 3]]
        w, x, y, z = imu.quaternion
        imu.rpy[0] = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        imu.rpy[1] = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        imu.rpy[2] = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        msg.mode_machine = self.args.mode_machine
        msg.mode_pr = 0
        self.tick = (self.tick + 1) & 0xFFFFFFFF
        msg.tick = self.tick
        self.state_pub.Write(msg)

    def status_line(self, wall_elapsed: float, prev: dict) -> str:
        with self.lock:
            lc, ac = self.lowcmd_count, self.armcmd_count
            arm_age = time.monotonic() - self.armcmd_at if self.armcmd else None
        dt = wall_elapsed - prev.get("t", 0.0)
        lhz = (lc - prev.get("lc", 0)) / dt if dt > 0 else 0.0
        ahz = (ac - prev.get("ac", 0)) / dt if dt > 0 else 0.0
        prev.update(t=wall_elapsed, lc=lc, ac=ac)
        rtf = self.data.time / wall_elapsed if wall_elapsed > 0 else 0.0
        err = ""
        if self.armcmd is not None and arm_age is not None and arm_age < self.args.cmd_timeout:
            worst = max(
                (abs(float(self.armcmd.motor_cmd[i].q) - float(self.data.qpos[self.qadr[i]]))
                 for i in range(13, 27) if int(self.armcmd.motor_cmd[i].mode) != 0),
                default=0.0,
            )
            err = f" arm_sdk max|q_err|={worst:.4f} rad w={float(self.armcmd.motor_cmd[ARM_SDK_WEIGHT_SLOT].q):.2f}"
        return f"sim t={self.data.time:8.2f}s rtf={rtf:.2f} lowcmd={lhz:5.1f}Hz arm_sdk={ahz:5.1f}Hz{err}"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    p.add_argument("--domain", type=int, default=1, help="DDS domain (server.py --domain must match)")
    p.add_argument("--interface", default="lo", help="network interface for CycloneDDS")
    p.add_argument("--physics-dt", type=float, default=0.002, help="MuJoCo timestep (s)")
    p.add_argument("--state-hz", type=float, default=500.0, help="rt/lowstate publish rate")
    p.add_argument("--cmd-timeout", type=float, default=0.5, help="seconds before a command stream is stale")
    p.add_argument("--fix-base", dest="fix_base", action="store_true", default=True)
    p.add_argument("--free-base", dest="fix_base", action="store_false", help="keep the floating base (falls without a balance controller)")
    p.add_argument("--hold-kp", type=float, default=300.0, help="leg/waist hold stiffness (onboard-controller stand-in)")
    p.add_argument("--hold-kd", type=float, default=6.0)
    p.add_argument("--arm-hold-kp", type=float, default=120.0, help="arm hold stiffness when no arm_sdk/lowcmd authority")
    p.add_argument("--arm-hold-kd", type=float, default=2.0)
    p.add_argument("--mode-machine", type=int, default=4)
    p.add_argument("--realtime", type=float, default=1.0, help="wall-clock speed factor (0 = as fast as possible)")
    p.add_argument("--viewer", action="store_true", help="open the MuJoCo passive viewer (needs a display)")
    p.add_argument("--status-every", type=float, default=2.0, help="seconds between status lines (0 = quiet)")
    p.add_argument("--duration", type=float, default=0.0, help="stop after N seconds of sim time (0 = run forever)")
    # Realism knobs (literature: 20-60 ms DDS+motor delay on Unitree stacks; Coulomb
    # friction of a 40 Nm QDD gearbox is 0.5-2 Nm, the MJCF default is 0.2).
    p.add_argument("--cmd-latency-ms", type=float, default=0.0, help="delay applied to every incoming rt/lowcmd and rt/arm_sdk command")
    p.add_argument("--frictionloss", type=float, default=None, help="override joint frictionloss (Nm) on the 14 arm joints")
    p.add_argument("--armature", type=float, default=None, help="override joint armature on the 14 arm joints")
    p.add_argument("--integrator", choices=("euler", "implicitfast"), default="euler")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if not args.scene.exists():
        print(f"scene not found: {args.scene}", file=sys.stderr)
        return 2
    model = load_model(args.scene, args.fix_base)
    model.opt.timestep = args.physics_dt
    if args.integrator == "implicitfast":
        model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    if args.frictionloss is not None or args.armature is not None:
        for i in range(13, 27):
            dof = int(model.jnt_dofadr[model.actuator_trnid[i, 0]])
            if args.frictionloss is not None:
                model.dof_frictionloss[dof] = args.frictionloss
            if args.armature is not None:
                model.dof_armature[dof] = args.armature
    ChannelFactoryInitialize(args.domain, args.interface)
    twin = H12Twin(model, args)
    print(
        f"H1-2 twin: {model.nu} motors, base {'fixed' if args.fix_base else 'free'}, "
        f"physics {1/args.physics_dt:.0f} Hz, lowstate {args.state_hz:.0f} Hz, DDS domain {args.domain} on {args.interface}, "
        f"latency {args.cmd_latency_ms:.0f} ms, integrator {args.integrator}"
        + (f", frictionloss {args.frictionloss}" if args.frictionloss is not None else "")
        + (f", armature {args.armature}" if args.armature is not None else ""),
        flush=True,
    )
    viewer = None
    if args.viewer:
        from mujoco import viewer as mjviewer

        viewer = mjviewer.launch_passive(model, twin.data)

    steps_per_state = max(1, int(round(1.0 / (args.state_hz * args.physics_dt))))
    step = 0
    t0 = time.perf_counter()
    next_status = args.status_every
    status_prev: dict = {}
    last_sync = 0.0
    try:
        while True:
            if viewer is not None and not viewer.is_running():
                break
            twin.control()
            mujoco.mj_step(model, twin.data)
            step += 1
            if step % steps_per_state == 0:
                twin.publish_state()
            wall = time.perf_counter() - t0
            if viewer is not None and wall - last_sync > 0.02:
                viewer.sync()
                last_sync = wall
            if args.status_every > 0 and wall >= next_status:
                print(twin.status_line(wall, status_prev), flush=True)
                next_status += args.status_every
            if args.duration and twin.data.time >= args.duration:
                break
            if args.realtime > 0:
                target = twin.data.time / args.realtime
                sleep = target - (time.perf_counter() - t0)
                if sleep > 0:
                    time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
