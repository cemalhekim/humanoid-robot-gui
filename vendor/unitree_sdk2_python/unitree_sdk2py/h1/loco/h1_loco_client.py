import json

from ...rpc.client import Client
from .h1_loco_api import *

"""
" class SportClient
"""
class LocoClient(Client):
    def __init__(self):
        super().__init__(LOCO_SERVICE_NAME, False)


    def Init(self):
        # set api version
        self._SetApiVerson(LOCO_API_VERSION)

        # regist api
        self._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_ID, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_FSM_MODE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_BALANCE_MODE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_SWING_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_STAND_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_PHASE, 0) # deprecated

        self._RegistApi(ROBOT_API_ID_LOCO_SET_FSM_ID, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_BALANCE_MODE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_SWING_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_STAND_HEIGHT, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_VELOCITY, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_PHASE, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_ARM_TASK, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_ENABLE_ODOM, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_DISABLE_ODOM, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_GET_ODOM, 0)
        self._RegistApi(ROBOT_API_ID_LOCO_SET_TARGET_POSITION, 0)

    # 8101
    def SetFsmId(self, fsm_id: int):
        p = {}
        p["data"] = fsm_id
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_FSM_ID, parameter)
        return code

    def SetBalanceMode(self, balance_mode: int):
        p = {}
        p["data"] = balance_mode
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_BALANCE_MODE, parameter)
        return code

    def SetSwingHeight(self, swing_height: float):
        p = {}
        p["data"] = swing_height
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_SWING_HEIGHT, parameter)
        return code

    # 8104
    def SetStandHeight(self, stand_height: float):
        p = {}
        p["data"] = stand_height
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_STAND_HEIGHT, parameter)
        return code

    # 8105
    def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0):
        p = {}
        velocity = [vx,vy,omega]
        p["velocity"] = velocity
        p["duration"] = duration
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_VELOCITY, parameter)
        return code

    def SetPhase(self, phase: list[float]):
        p = {}
        p["data"] = phase
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_PHASE, parameter)
        return code

    def SetTargetPos(self, x: float, y: float, yaw: float, relative: bool = True):
        p = {}
        p["x"] = x
        p["y"] = y
        p["yaw"] = yaw
        p["relative"] = relative
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_TARGET_POSITION, parameter)
        return code

    def SetTaskId(self, task_id: int):
        p = {}
        p["data"] = task_id
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_SET_ARM_TASK, parameter)
        return code

    def EnableOdom(self):
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_ENABLE_ODOM, parameter)
        return code

    def DisableOdom(self):
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_DISABLE_ODOM, parameter)
        return code

    def GetOdom(self):
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_GET_ODOM, parameter)
        if code != 0:
            return code, None
        js = json.loads(data)
        return code, {"x": js.get("x"), "y": js.get("y"), "yaw": js.get("z", js.get("yaw"))}

    def Damp(self):
        self.SetFsmId(1)
    
    def Start(self):
        self.SetFsmId(204)

    def StandUp(self):
        self.SetFsmId(2)

    def ZeroTorque(self):
        self.SetFsmId(0)

    def StopMove(self):
        self.SetVelocity(0., 0., 0.)

    def HighStand(self):
        UINT32_MAX = (1 << 32) - 1
        return self.SetStandHeight(UINT32_MAX)

    def LowStand(self):
        UINT32_MIN = 0
        return self.SetStandHeight(UINT32_MIN)

    def Move(self, vx: float, vy: float, vyaw: float, continous_move: bool = False):
        duration = 864000.0 if continous_move else 1
        return self.SetVelocity(vx, vy, vyaw, duration)

    def BalanceStand(self):
        return self.SetBalanceMode(0)

    def ContinuousGait(self, flag: bool):
        return self.SetBalanceMode(1 if flag else 0)

    def SetNextFoot(self, foot: bool):
        return self.SetPhase([0.0, 1.0] if foot else [1.0, 0.0])

    def WaveHand(self):
        return self.SetTaskId(0)

    def ShakeHand(self, stage: int = -1):
        if not hasattr(self, "first_shake_hand_stage_"):
            self.first_shake_hand_stage_ = True
        if stage == 0:
            self.first_shake_hand_stage_ = False
            return self.SetTaskId(1)
        if stage == 1:
            self.first_shake_hand_stage_ = True
            return self.SetTaskId(2)
        self.first_shake_hand_stage_ = not self.first_shake_hand_stage_
        return self.SetTaskId(2 if self.first_shake_hand_stage_ else 1)

    def GetFsmId(self):
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, parameter)
        if code != 0:
            return code, None
        js = json.loads(data)
        return code, js.get("data")

    def GetFsmMode(self):
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_GET_FSM_MODE, parameter)
        if code != 0:
            return code, None
        js = json.loads(data)
        return code, js.get("data")

    def GetBalanceMode(self):
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_GET_BALANCE_MODE, parameter)
        if code != 0:
            return code, None
        js = json.loads(data)
        return code, js.get("data")

    def GetSwingHeight(self):
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_GET_SWING_HEIGHT, parameter)
        if code != 0:
            return code, None
        js = json.loads(data)
        return code, js.get("data")

    def GetStandHeight(self):
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_GET_STAND_HEIGHT, parameter)
        if code != 0:
            return code, None
        js = json.loads(data)
        return code, js.get("data")

    def GetPhase(self):
        p = {}
        parameter = json.dumps(p)
        code, data = self._Call(ROBOT_API_ID_LOCO_GET_PHASE, parameter)
        if code != 0:
            return code, None
        js = json.loads(data)
        return code, js.get("data")
