#!/usr/bin/env python3
"""Minimal H1 `loco` RPC shim for the Unitree MuJoCo Python simulator.

The upstream simulator does not implement Unitree's high-level H1 locomotion
service. This shim implements just enough of that service to test SDK clients
that call StandUp/Start/SetVelocity against MuJoCo. Positive vx commands are
translated into a conservative low-level two-step sequence.
"""

import argparse
import json
import sys
import threading
import time

try:
    from unitree_sdk2py.core.channel import (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
    )
    from unitree_sdk2py.h1.loco.h1_loco_api import (
        LOCO_API_VERSION,
        LOCO_SERVICE_NAME,
        ROBOT_API_ID_LOCO_GET_FSM_ID,
        ROBOT_API_ID_LOCO_GET_FSM_MODE,
        ROBOT_API_ID_LOCO_GET_STAND_HEIGHT,
        ROBOT_API_ID_LOCO_SET_FSM_ID,
        ROBOT_API_ID_LOCO_SET_STAND_HEIGHT,
        ROBOT_API_ID_LOCO_SET_VELOCITY,
    )
    from unitree_sdk2py.idl.unitree_api.msg.dds_ import (
        RequestIdentity_,
        Request_,
        ResponseHeader_,
        ResponseStatus_,
        Response_,
    )
    from unitree_sdk2py.rpc.internal import (
        RPC_API_ID_INTERNAL_API_VERSION,
        RPC_ERR_SERVER_API_NOT_IMPL,
        RPC_ERR_SERVER_INTERNAL,
    )
except ImportError as exc:
    print(
        "Missing Unitree SDK2 Python dependency. Run through one of the repo "
        "wrapper scripts so PYTHONPATH includes external/unitree_sdk2_python.\n\n"
        f"Import error: {exc}",
        file=sys.stderr,
    )
    raise SystemExit(2)

from h1_mujoco_two_step_walk import H1TwoStepWalkController
from h1_mujoco_stand_handshake import HOME_POSE


def speed_scale_from_vx(vx):
    return min(1.8, max(0.8, abs(vx) / 0.2))


class H1MuJoCoLocoShim:
    def __init__(self, controller, final_hold_seconds):
        self.controller = controller
        self.final_hold_seconds = final_hold_seconds
        self.fsm_id = 0
        self.stand_height = 0.0
        self.motion_lock = threading.Lock()
        self.handlers = {}
        self.response_pub = ChannelPublisher(
            f"rt/api/{LOCO_SERVICE_NAME}/response", Response_
        )
        self.request_sub = ChannelSubscriber(
            f"rt/api/{LOCO_SERVICE_NAME}/request", Request_
        )

    def Init(self):
        self.handlers = {
            ROBOT_API_ID_LOCO_GET_FSM_ID: self.get_fsm_id,
            ROBOT_API_ID_LOCO_GET_FSM_MODE: self.get_fsm_mode,
            ROBOT_API_ID_LOCO_GET_STAND_HEIGHT: self.get_stand_height,
            ROBOT_API_ID_LOCO_SET_FSM_ID: self.set_fsm_id,
            ROBOT_API_ID_LOCO_SET_STAND_HEIGHT: self.set_stand_height,
            ROBOT_API_ID_LOCO_SET_VELOCITY: self.set_velocity,
        }
        self.response_pub.Init()
        self.request_sub.Init(self.handle_request, 10)

    def Start(self, _enable_prio_queue=False):
        print(
            f"H1 MuJoCo loco shim ready on rt/api/{LOCO_SERVICE_NAME}. "
            "Waiting for SDK loco RPC calls."
        )

    def handle_request(self, request):
        api_id = request.header.identity.api_id
        code = 0
        data = ""

        if api_id == RPC_API_ID_INTERNAL_API_VERSION:
            data = LOCO_API_VERSION
        elif api_id in self.handlers:
            try:
                code, data = self.handlers[api_id](request.parameter)
            except Exception as exc:
                print(f"loco shim: handler error for api {api_id}: {exc}")
                code = RPC_ERR_SERVER_INTERNAL
                data = ""
        else:
            code = RPC_ERR_SERVER_API_NOT_IMPL

        if request.header.policy.noreply:
            return

        identity = RequestIdentity_(request.header.identity.id, api_id)
        status = ResponseStatus_(int(code))
        response = Response_(ResponseHeader_(identity, status), data, [])
        self.response_pub.Write(response, 1.0)

    def get_fsm_id(self, _parameter):
        return 0, json.dumps({"data": self.fsm_id})

    def get_fsm_mode(self, _parameter):
        return 0, json.dumps({"data": "mujoco_loco_shim"})

    def get_stand_height(self, _parameter):
        return 0, json.dumps({"data": self.stand_height})

    def set_fsm_id(self, parameter):
        payload = json.loads(parameter or "{}")
        self.fsm_id = int(payload.get("data", 0))
        print(f"loco shim: SetFsmId({self.fsm_id})")

        if self.fsm_id in {2, 204}:
            with self.motion_lock:
                self.controller.move_to(HOME_POSE, 1.5, "loco shim: moving to home.")
                self.controller.hold(HOME_POSE, 0.4, "loco shim: home hold.")
        elif self.fsm_id == 0:
            with self.motion_lock:
                self.controller.hold(HOME_POSE, 0.4, "loco shim: zero velocity hold.")
        return 0, ""

    def set_stand_height(self, parameter):
        payload = json.loads(parameter or "{}")
        self.stand_height = float(payload.get("data", 0.0))
        print(f"loco shim: SetStandHeight({self.stand_height})")
        return 0, ""

    def set_velocity(self, parameter):
        payload = json.loads(parameter or "{}")
        vx, vy, vyaw = payload.get("velocity", [0.0, 0.0, 0.0])
        duration = float(payload.get("duration", 1.0))
        print(
            "loco shim: SetVelocity("
            f"vx={vx:.3f}, vy={vy:.3f}, vyaw={vyaw:.3f}, duration={duration:.3f})"
        )

        with self.motion_lock:
            if abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(vyaw) < 1e-3:
                self.controller.move_to(HOME_POSE, 1.0, "loco shim: stopping at home.")
                return 0, ""

            if abs(vy) > 1e-3 or abs(vyaw) > 1e-3 or vx < -1e-3:
                print("loco shim: only positive vx is implemented for this test shim.")
                return 0, ""

            self.controller.speed_scale = speed_scale_from_vx(vx)
            print(f"loco shim: step speed scale {self.controller.speed_scale:.2f}")
            self.controller.move_to(HOME_POSE, 1.0, "loco shim: preparing to walk.")
            self.controller.step_once("right")
            self.controller.step_once("left")
            self.controller.move_to(HOME_POSE, 1.0, "loco shim: returning home.")

            if self.final_hold_seconds > 0.0:
                self.controller.hold(
                    HOME_POSE,
                    self.final_hold_seconds,
                    "loco shim: final home hold.",
                )
        return 0, ""


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain-id", type=int, default=1)
    parser.add_argument("--interface", default="lo")
    parser.add_argument("--rate", type=float, default=200.0)
    parser.add_argument("--wait-timeout", type=float, default=12.0)
    parser.add_argument("--final-hold-seconds", type=float, default=1.0)
    return parser.parse_args()


def main():
    args = parse_args()
    ChannelFactoryInitialize(args.domain_id, args.interface)

    controller = H1TwoStepWalkController(args.domain_id, args.interface, args.rate)
    if not controller.wait_for_simulator(args.wait_timeout):
        print(
            "No MuJoCo lowstate received. Start the H1 simulator first.",
            file=sys.stderr,
        )
        return 1

    server = H1MuJoCoLocoShim(controller, args.final_hold_seconds)
    server.Init()
    server.Start(False)
    print("H1 MuJoCo loco shim ready. Waiting for SDK loco RPC calls.")

    while True:
        time.sleep(10)


if __name__ == "__main__":
    raise SystemExit(main())
