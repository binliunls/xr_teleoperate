"""
SharpaWave dexterous hand state subscriber for xr_teleoperate.

Read-only — subscribes to the Sharpa hand joint-state topics on DDS and exposes
the angles via shared arrays for recording. Does NOT send commands; commands
are sent by the retargeting daemon (e.g. retargeting_manus_demo_dds.py) running
in parallel on the same DDS domain.

The state topics can be published by either:
  - the C++ bridge on Thor (scripts/sharpa_dds_bridge), or
  - the retargeting demo with -sdk -dds running directly against the SDK.

Topics:
  rt/sharpa/left/state   — 22 joints, motor_state[i].q in degrees
  rt/sharpa/right/state  — 22 joints, motor_state[i].q in degrees

22 DOF / hand joint order:
    thumb:  CMC_FE, CMC_AA, MCP_FE, MCP_AA, IP        (5)
    index:  MCP_FE, MCP_AA, PIP, DIP                  (4)
    middle: MCP_FE, MCP_AA, PIP, DIP                  (4)
    ring:   MCP_FE, MCP_AA, PIP, DIP                  (4)
    pinky:  CMC, MCP_FE, MCP_AA, PIP, DIP             (5)

Implementation note: runs in a daemon *thread* (not a subprocess) so that it
reuses the DDS ChannelFactory already initialised by the main process.
Using a subprocess with fork() would inherit the already-initialised DDS state
and a second ChannelFactoryInitialize() call in the child silently fails,
leaving all subscribers returning None and the shared arrays stuck at zero.
"""

import math
import time
import threading
from enum import IntEnum

import numpy as np
import logging_mp

logger_mp = logging_mp.getLogger(__name__)

SHARPA_DOF = 22
SHARPA_TACTILE_CHANNELS = 5
SHARPA_F6 = 6
SHARPA_TACTILE_PER_HAND = SHARPA_TACTILE_CHANNELS * SHARPA_F6  # 30


class SharpaFingerIndex(IntEnum):
    THUMB = 0
    INDEX = 1
    MIDDLE = 2
    RING = 3
    PINKY = 4


class SharpaWave_Controller:
    """
    Reads Sharpa hand joint states from DDS and exposes them via shared arrays.

    Requires a publisher of rt/sharpa/{left,right}/state on the same DDS domain
    (the C++ bridge on Thor, or the retargeting demo in -sdk -dds mode).
    Does not use the Sharpa SDK — no port 50001 conflict.

    Args:
        dual_hand_data_lock:    multiprocessing Lock
        dual_hand_state_array:  mp.Array of length 2*22 — joint angles in radians
        dual_hand_action_array: mp.Array of length 2*22 — same as state (hardware feedback)
        fps:        polling frequency
        dds_domain: CycloneDDS domain ID — informational only; the main process
                    must have called ChannelFactoryInitialize with this domain
                    before constructing this controller.
    """

    def __init__(
        self,
        dual_hand_data_lock=None,
        dual_hand_state_array=None,
        dual_hand_action_array=None,
        fps=60.0,
        dds_domain=0,
    ):
        logger_mp.info("Initialize SharpaWave_Controller (DDS thread mode)...")
        self.fps = fps

        ctrl_thread = threading.Thread(
            target=self._control_thread,
            args=(dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array),
            daemon=True,
        )
        ctrl_thread.start()
        logger_mp.info("Initialize SharpaWave_Controller OK!")

    def _control_thread(self, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array):
        # DDS factory is already initialised by the main process — just create subscribers.
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

        sub_left = ChannelSubscriber("rt/sharpa/left/state", HandState_)
        sub_left.Init()
        sub_right = ChannelSubscriber("rt/sharpa/right/state", HandState_)
        sub_right.Init()
        logger_mp.info("[SharpaWave] subscribed to rt/sharpa/{left,right}/state")

        left_angles = np.zeros(SHARPA_DOF, dtype=np.float64)
        right_angles = np.zeros(SHARPA_DOF, dtype=np.float64)
        period = 1.0 / self.fps

        try:
            while True:
                t0 = time.time()

                msg = sub_left.Read()
                if msg is not None and msg.motor_state:
                    n = min(len(msg.motor_state), SHARPA_DOF)
                    for i in range(n):
                        left_angles[i] = math.radians(msg.motor_state[i].q)

                msg = sub_right.Read()
                if msg is not None and msg.motor_state:
                    n = min(len(msg.motor_state), SHARPA_DOF)
                    for i in range(n):
                        right_angles[i] = math.radians(msg.motor_state[i].q)

                if dual_hand_state_array is not None and dual_hand_action_array is not None:
                    combined = np.concatenate([left_angles, right_angles])
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = combined
                        dual_hand_action_array[:] = combined

                dt = time.time() - t0
                time.sleep(max(0.0, period - dt))
        finally:
            logger_mp.info("[SharpaWave] control thread closed.")
