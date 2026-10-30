"""
Workstation-side DDS client for Sharpa hands.

Publishes joint position commands to the DDS bridge running on Thor.
Drop-in replacement for direct Sharpa SDK calls — no SDK needed on the workstation.

Topics:
  Publish   rt/sharpa/left/cmd    HandCmd_  — 22 x motor_cmd[i].q (radians)
  Publish   rt/sharpa/right/cmd   HandCmd_  — 22 x motor_cmd[i].q (radians)
  Subscribe rt/sharpa/left/state  HandState_ — 22 x motor_state[i].q (degrees)
  Subscribe rt/sharpa/right/state HandState_ — 22 x motor_state[i].q (degrees)
"""

import math
import threading
import time

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, MotorCmd_

NUM_JOINTS = 22
# Sharpa ControlMode ids carried in motor_cmd[0].mode (0 = leave the mode unchanged).
MODE_UNCHANGED = 0
MODE_ZERO_FORCE = 10
MODE_POSITION = 13
TOPIC_CMD = {"left": "rt/sharpa/left/cmd", "right": "rt/sharpa/right/cmd"}
TOPIC_STATE = {"left": "rt/sharpa/left/state", "right": "rt/sharpa/right/state"}


class SharpaHandDDSClient:
    """
    Publishes HandCmd_ to the bridge and subscribes to HandState_.
    Mimics the set_joint_position / get_joint_position_degree interface
    of the real SharpaWave object so callers need minimal changes.
    """

    def __init__(self, side: str):
        assert side in ("left", "right"), f"side must be 'left' or 'right', got {side!r}"
        self.side = side
        self._state_lock = threading.Lock()
        self._state_deg: list = [0.0] * NUM_JOINTS
        self._state_mode = MODE_UNCHANGED  # control mode the bridge reports (0 = old bridge / no state yet)

        self._pub = ChannelPublisher(TOPIC_CMD[side], HandCmd_)
        self._pub.Init()

        self._sub = ChannelSubscriber(TOPIC_STATE[side], HandState_)
        self._sub.Init(self._on_state, 1)

    def _on_state(self, msg):
        if msg is None or not msg.motor_state:
            return
        with self._state_lock:
            self._state_deg = [msg.motor_state[i].q for i in range(min(NUM_JOINTS, len(msg.motor_state)))]
            self._state_mode = int(msg.motor_state[0].mode)

    def _make_cmd(self, positions_rad: list, mode: int = MODE_UNCHANGED) -> HandCmd_:
        # MotorCmd_ requires (mode, q, dq, tau, kp, kd, reserve); HandCmd_ requires (motor_cmd, reserve).
        motor_cmd = [
            MotorCmd_(
                mode=mode if i == 0 else 0,  # the bridge reads the mode from joint 0
                q=float(positions_rad[i]) if i < len(positions_rad) else 0.0,
                dq=0.0,
                tau=0.0,
                kp=0.0,
                kd=0.0,
                reserve=0,
            )
            for i in range(NUM_JOINTS)
        ]
        return HandCmd_(motor_cmd=motor_cmd, reserve=[0, 0, 0, 0])

    def set_joint_position(self, positions_rad: list, interpolation: bool = False, mode: int = MODE_UNCHANGED):
        """Send target joint positions (radians) to the bridge; mode optionally switches the control mode."""
        self._pub.Write(self._make_cmd(positions_rad, mode))

    def get_control_mode(self) -> int:
        """Control mode reported by the bridge: 13 POSITION, 10 ZERO_FORCE, 0 unknown (old bridge)."""
        with self._state_lock:
            return self._state_mode

    def set_control_mode(self, mode: int):
        """Switch the hand to MODE_POSITION or MODE_ZERO_FORCE, holding the measured pose (no jump)."""
        self.set_joint_position(self.get_joint_position_rad(), mode=mode)

    def get_joint_position_degree(self):
        """Return (error_code=0, list of 22 joint angles in degrees)."""
        with self._state_lock:
            return 0, list(self._state_deg)

    def get_joint_position_rad(self):
        """Return list of 22 joint angles in radians."""
        with self._state_lock:
            return [math.radians(d) for d in self._state_deg]

    def go_neutral(self):
        """Send all joints to zero."""
        self.set_joint_position([0.0] * NUM_JOINTS)


def connect_hand_dds(side: str) -> SharpaHandDDSClient:
    """Create a DDS client for one hand. ChannelFactoryInitialize must be called first."""
    client = SharpaHandDDSClient(side)
    # send neutral command so the bridge applies it immediately
    client.go_neutral()
    return client


def disconnect_hand_dds(client: SharpaHandDDSClient):
    """Send neutral command before releasing."""
    if client is None:
        return
    client.go_neutral()
    time.sleep(0.5)
