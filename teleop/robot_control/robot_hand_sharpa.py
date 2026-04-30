"""
SharpaWave dexterous hand controller for xr_teleoperate.

Differs from the other hand controllers in this folder: the SharpaWave hand is
NOT driven over Unitree DDS topics — it has its own Python SDK (`sharpa`).
Each hand is connected by `HandSide` (LEFT / RIGHT) and commanded via
`hand.set_joint_position_rad(angles_22)`.

22 DOF / hand. Joint order (matches URDF + `unitree_sharpa.yml`):
    thumb:  CMC_FE, CMC_AA, MCP_FE, MCP_AA, IP        (5)
    index:  MCP_FE, MCP_AA, PIP, DIP                  (4)
    middle: MCP_FE, MCP_AA, PIP, DIP                  (4)
    ring:   MCP_FE, MCP_AA, PIP, DIP                  (4)
    pinky:  CMC, MCP_FE, MCP_AA, PIP, DIP             (5)

Tactile: 5 channels per hand (one per finger), each producing a 6-axis F6
[Fx, Fy, Fz, Mx, My, Mz]. Flattened to 30 floats per hand.

Input modes:
    "hand"  — XR 25×3 hand-skeleton keypoints, run through dex-retargeting.
    "manus" — pre-retargeted 22 joint angles (radians) supplied directly by
              the Manus glove client. Skip retargeting.

Sim mode: skip the SDK entirely. Useful for testing the recording / vis
pipeline without hardware.
"""

import math
import os
import sys
import time
import threading
from enum import IntEnum
from multiprocessing import Process

import numpy as np

from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
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
    Controls a pair of SharpaWave dexterous hands.

    Args:
        left_hand_array, right_hand_array: mp.Array, the per-hand input.
            - input_mode == "hand":  shape (75,) float — XR 25×3 keypoints, flattened.
            - input_mode == "manus": shape (22,) float — joint angles in radians.
        dual_hand_state_array:  mp.Array of length 2*22  — current measured joint angles.
        dual_hand_action_array: mp.Array of length 2*22  — last commanded joint angles.
        dual_hand_tactile_array: mp.Array of length 2*30 — per-frame F6 readings
            (left first, then right). None to skip tactile streaming.
        input_mode: "hand" (XR retargeting) or "manus" (direct joint angles).
        simulation_mode: skip the SDK and echo commands back as state.
        speed_coeff, current_coeff: SDK motor scaling, 0..1.
    """

    def __init__(
        self,
        left_hand_array,
        right_hand_array,
        dual_hand_data_lock=None,
        dual_hand_state_array=None,
        dual_hand_action_array=None,
        dual_hand_tactile_array=None,
        input_mode="hand",
        mount_variant="flange",
        fps=60.0,
        simulation_mode=False,
        speed_coeff=0.5,
        current_coeff=0.5,
        Unit_Test=False,
        source="sdk",
        dds_domain=0,
    ):
        logger_mp.info("Initialize SharpaWave_Controller...")
        self.fps = fps
        self.input_mode = input_mode
        self.simulation_mode = simulation_mode
        self.speed_coeff = speed_coeff
        self.current_coeff = current_coeff
        self.source = source
        self.dds_domain = dds_domain

        if source not in ("sdk", "dds"):
            raise ValueError(f"Unknown source: {source!r}")

        if input_mode not in ("hand", "manus"):
            raise ValueError(f"Unknown input_mode: {input_mode!r}")

        if mount_variant not in ("flange", "wrist"):
            raise ValueError(f"mount_variant must be 'flange' or 'wrist', got {mount_variant!r}")
        self.mount_variant = mount_variant

        if input_mode == "hand":
            self.hand_retargeting = HandRetargeting(
                HandType.UNITREE_SHARPA_Unit_Test if Unit_Test else HandType.UNITREE_SHARPA,
                variant=mount_variant,
            )
        else:
            self.hand_retargeting = None

        if source == "dds":
            ctrl_proc = Process(
                target=self._control_process_dds,
                args=(dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, dds_domain),
            )
        else:
            ctrl_proc = Process(
                target=self._control_process,
                args=(
                    left_hand_array,
                    right_hand_array,
                    dual_hand_data_lock,
                    dual_hand_state_array,
                    dual_hand_action_array,
                    dual_hand_tactile_array,
                ),
            )
        ctrl_proc.daemon = True
        ctrl_proc.start()
        logger_mp.info("Initialize SharpaWave_Controller OK!")

    # ------------------------------------------------------------------
    # SDK glue (executed inside the control subprocess)
    # ------------------------------------------------------------------
    def _open_hands(self):
        """Discover and connect both hands via the sharpa SDK.

        Returns (manager, left_hand, right_hand). In sim mode returns
        (None, None, None).
        """
        if self.simulation_mode:
            logger_mp.warning("[SharpaWave] simulation_mode=True — running without hardware.")
            return None, None, None

        from sharpa import (  # local import so non-SDK installs can still import this module
            SharpaWaveManager,
            HandSide,
            ControlMode,
            ControlSource,
        )

        manager = SharpaWaveManager.get_instance()

        # Wait for heartbeat discovery
        for _ in range(40):
            if manager.get_all_device_sn():
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("SharpaWave: no devices discovered (heartbeat timeout).")

        left_hand = manager.connect(HandSide.LEFT)
        right_hand = manager.connect(HandSide.RIGHT)

        for h, label in ((left_hand, "left"), (right_hand, "right")):
            h.start()
            t0 = time.time()
            while not (h.is_hand_ready() and h.is_tactile_ready()):
                if time.time() - t0 > 10.0:
                    raise RuntimeError(f"SharpaWave: {label} hand not ready after 10s.")
                time.sleep(0.05)
            h.set_control_source(ControlSource.SDK)
            h.set_control_mode(ControlMode.POSITION)
            h.set_enable_state(True)
            time.sleep(0.1)  # ~70 ms for enable to take effect
            h.set_speed_coeff(self.speed_coeff)
            h.set_current_coeff(self.current_coeff)
            h.enable_collision_protection(True)
            try:
                h.calib_tactile(num_frames=20, max_retry=5)
            except Exception as e:
                logger_mp.warning(f"[SharpaWave] {label} tactile tare failed: {e}")
            logger_mp.info(f"[SharpaWave] {label} hand ready.")

        return manager, left_hand, right_hand

    @staticmethod
    def _read_state(hand):
        s = hand.get_states()
        return np.asarray(s.angles, dtype=np.float64)

    @staticmethod
    def _command(hand, angles_22):
        hand.set_joint_position_rad(list(angles_22), False)

    # ------------------------------------------------------------------
    # Per-hand input → 22 angles
    # ------------------------------------------------------------------
    def _retarget_one(self, side, hand_data_25x3):
        """Run dex-retargeting on one hand's XR keypoints."""
        if side == "left":
            indices = self.hand_retargeting.left_indices
            ref = hand_data_25x3[indices[1, :]] - hand_data_25x3[indices[0, :]]
            q = self.hand_retargeting.left_retargeting.retarget(ref)
            return q[self.hand_retargeting.left_dex_retargeting_to_hardware]
        else:
            indices = self.hand_retargeting.right_indices
            ref = hand_data_25x3[indices[1, :]] - hand_data_25x3[indices[0, :]]
            q = self.hand_retargeting.right_retargeting.retarget(ref)
            return q[self.hand_retargeting.right_dex_retargeting_to_hardware]

    # ------------------------------------------------------------------
    # DDS-based state reader (subprocess) — no SDK, no port conflicts
    # ------------------------------------------------------------------
    def _control_process_dds(self, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, dds_domain):
        _sdk_path = os.path.expanduser("~/unitree_sdk2_python")
        if os.path.isdir(_sdk_path) and _sdk_path not in sys.path:
            sys.path.insert(0, _sdk_path)

        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

        ChannelFactoryInitialize(dds_domain)

        sub_left = ChannelSubscriber("rt/sharpa/left/state", HandState_)
        sub_left.Init()
        sub_right = ChannelSubscriber("rt/sharpa/right/state", HandState_)
        sub_right.Init()
        logger_mp.info("[SharpaWave/DDS] subscribed to rt/sharpa/{left,right}/state domain=%d", dds_domain)

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
            logger_mp.info("[SharpaWave/DDS] control loop closed.")

    # ------------------------------------------------------------------
    # Main control loop (subprocess)
    # ------------------------------------------------------------------
    def _control_process(
        self,
        left_hand_array,
        right_hand_array,
        dual_hand_data_lock,
        dual_hand_state_array,
        dual_hand_action_array,
        dual_hand_tactile_array,
    ):
        try:
            manager, left_hand, right_hand = self._open_hands()
        except Exception as e:
            logger_mp.error(f"[SharpaWave] open_hands failed: {e}")
            return

        # Tactile cache populated by SDK callbacks (real mode) or zeros (sim).
        tactile_lock = threading.Lock()
        tactile_cache = {
            "left": np.zeros(SHARPA_TACTILE_PER_HAND, dtype=np.float64),
            "right": np.zeros(SHARPA_TACTILE_PER_HAND, dtype=np.float64),
        }

        def _make_tactile_cb(side):
            def _cb(frame):
                try:
                    ch = frame["channel"] if isinstance(frame, dict) else frame.channel
                    if isinstance(frame, dict):
                        f6 = np.asarray(frame["content"]["F6"], dtype=np.float64)
                    else:
                        f6 = np.asarray(
                            frame.f6_data if hasattr(frame, "f6_data") else frame.content["F6"], dtype=np.float64
                        )
                    if f6.size != SHARPA_F6:
                        return
                    finger = ch % SHARPA_TACTILE_CHANNELS  # right=0..4, left=5..9 → 0..4
                    with tactile_lock:
                        tactile_cache[side][finger * SHARPA_F6 : (finger + 1) * SHARPA_F6] = f6
                except Exception as e:
                    logger_mp.warning(f"[SharpaWave] tactile cb {side} error: {e}")

            return _cb

        if not self.simulation_mode:
            try:
                left_hand.set_tactile_callback(_make_tactile_cb("left"))
                right_hand.set_tactile_callback(_make_tactile_cb("right"))
            except Exception as e:
                logger_mp.warning(f"[SharpaWave] failed to set tactile callbacks: {e}")

        left_q = np.zeros(SHARPA_DOF, dtype=np.float64)
        right_q = np.zeros(SHARPA_DOF, dtype=np.float64)
        period = 1.0 / self.fps

        try:
            while True:
                t0 = time.time()

                # --- read input ---
                if self.input_mode == "hand":
                    with left_hand_array.get_lock():
                        left_in = np.array(left_hand_array[:]).reshape(25, 3).copy()
                    with right_hand_array.get_lock():
                        right_in = np.array(right_hand_array[:]).reshape(25, 3).copy()
                    # Skip retargeting until the XR side has populated valid data.
                    valid = (not np.all(right_in == 0.0)) and (
                        not np.allclose(left_in[4], np.array([-1.13, 0.3, 0.15]))
                    )
                    if valid:
                        try:
                            left_q = np.asarray(self._retarget_one("left", left_in), dtype=np.float64)
                            right_q = np.asarray(self._retarget_one("right", right_in), dtype=np.float64)
                        except Exception as e:
                            logger_mp.warning(f"[SharpaWave] retarget error: {e}")
                else:  # manus
                    with left_hand_array.get_lock():
                        left_q = np.array(left_hand_array[:SHARPA_DOF], dtype=np.float64)
                    with right_hand_array.get_lock():
                        right_q = np.array(right_hand_array[:SHARPA_DOF], dtype=np.float64)

                # --- send command ---
                if self.simulation_mode:
                    state_left, state_right = left_q.copy(), right_q.copy()
                else:
                    try:
                        self._command(left_hand, left_q)
                        self._command(right_hand, right_q)
                        state_left = self._read_state(left_hand)
                        state_right = self._read_state(right_hand)
                    except Exception as e:
                        logger_mp.warning(f"[SharpaWave] command/read error: {e}")
                        state_left, state_right = left_q.copy(), right_q.copy()

                # --- publish to shared arrays ---
                if dual_hand_state_array is not None and dual_hand_action_array is not None:
                    state = np.concatenate([state_left, state_right])
                    action = np.concatenate([left_q, right_q])
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = state
                        dual_hand_action_array[:] = action

                if dual_hand_tactile_array is not None:
                    with tactile_lock:
                        tac = np.concatenate([tactile_cache["left"], tactile_cache["right"]])
                    with dual_hand_data_lock:
                        dual_hand_tactile_array[:] = tac

                dt = time.time() - t0
                time.sleep(max(0.0, period - dt))
        finally:
            logger_mp.info("[SharpaWave] control loop closed.")
            try:
                if left_hand is not None:
                    left_hand.set_enable_state(False)
                    left_hand.stop()
                if right_hand is not None:
                    right_hand.set_enable_state(False)
                    right_hand.stop()
                if manager is not None:
                    manager.disconnect_all()
            except Exception:
                pass
