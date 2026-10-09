"""
SharpaWave dexterous hand state subscriber for xr_teleoperate.

Read-only — subscribes to the Sharpa hand joint-state and command topics on DDS
and exposes them for recording. It does NOT send commands; commands are sent by
the retargeting daemon (for example avatar_hand_dds_bridge.py) running in
parallel on the same DDS domain.

The state topics can be published by either:
  - the C++ bridge on Thor (scripts/sharpa_dds_bridge), or
  - the retargeting demo with -sdk -dds running directly against the SDK.

Topics:
  rt/sharpa/left/state   — 22 joints, motor_state[i].q in degrees
  rt/sharpa/right/state  — 22 joints, motor_state[i].q in degrees
  rt/sharpa/left/cmd     — 22 desired joints, motor_cmd[i].q in radians
  rt/sharpa/right/cmd    — 22 desired joints, motor_cmd[i].q in radians

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

from teleop.utils import sharpa_tactile_wire as wire

logger_mp = logging_mp.getLogger(__name__)

SHARPA_DOF = 22
SHARPA_TACTILE_CHANNELS = 5
SHARPA_F6 = 6
SHARPA_TACTILE_PER_HAND = SHARPA_TACTILE_CHANNELS * SHARPA_F6  # 30


def _sharpa_desired_q(msg):
    """Extract one complete desired joint vector without padding/invention."""
    motor_cmd = getattr(msg, "motor_cmd", None)
    if motor_cmd is None or len(motor_cmd) < SHARPA_DOF:
        return None
    return np.array(
        [float(motor_cmd[i].q) for i in range(SHARPA_DOF)],
        dtype=np.float64,
    )


class SharpaFingerIndex(IntEnum):
    THUMB = 0
    INDEX = 1
    MIDDLE = 2
    RING = 3
    PINKY = 4


class SharpaWave_Controller:
    """
    Reads Sharpa hand state and desired-command topics from DDS for recording.

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
        self._snapshot_lock = threading.Lock()
        self._left_angles = np.zeros(SHARPA_DOF, dtype=np.float64)
        self._right_angles = np.zeros(SHARPA_DOF, dtype=np.float64)
        self._state_timestamps = {"left": {}, "right": {}}
        self._desired_q = {"left": None, "right": None}
        self._desired_timestamps = {"left": {}, "right": {}}

        ctrl_thread = threading.Thread(
            target=self._control_thread,
            args=(dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array),
            daemon=True,
        )
        ctrl_thread.start()
        logger_mp.info("Initialize SharpaWave_Controller OK!")

    def get_state_snapshot(self):
        """Return states, desired commands and receive times atomically."""
        with self._snapshot_lock:
            return {
                "left": self._left_angles.tolist(),
                "right": self._right_angles.tolist(),
                "timestamps": {
                    "left": dict(self._state_timestamps["left"]),
                    "right": dict(self._state_timestamps["right"]),
                },
                "desired_q": {
                    "left": None
                    if self._desired_q["left"] is None
                    else self._desired_q["left"].tolist(),
                    "right": None
                    if self._desired_q["right"] is None
                    else self._desired_q["right"].tolist(),
                },
                "desired_timestamps": {
                    "left": dict(self._desired_timestamps["left"]),
                    "right": dict(self._desired_timestamps["right"]),
                },
            }

    def _control_thread(self, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array):
        # DDS factory is already initialised by the main process — just create subscribers.
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_

        sub_left = ChannelSubscriber("rt/sharpa/left/state", HandState_)
        sub_left.Init()
        sub_right = ChannelSubscriber("rt/sharpa/right/state", HandState_)
        sub_right.Init()
        cmd_sub_left = ChannelSubscriber("rt/sharpa/left/cmd", HandCmd_)
        cmd_sub_left.Init()
        cmd_sub_right = ChannelSubscriber("rt/sharpa/right/cmd", HandCmd_)
        cmd_sub_right.Init()
        logger_mp.info("[SharpaWave] subscribed to rt/sharpa/{left,right}/{state,cmd}")

        left_angles = np.zeros(SHARPA_DOF, dtype=np.float64)
        right_angles = np.zeros(SHARPA_DOF, dtype=np.float64)
        state_timestamps = {"left": {}, "right": {}}
        desired_q = {"left": None, "right": None}
        desired_timestamps = {"left": {}, "right": {}}
        period = 1.0 / self.fps

        try:
            while True:
                t0 = time.time()

                msg = sub_left.Read()
                if msg is not None and msg.motor_state:
                    state_timestamps["left"] = {
                        "workstation_receive_monotonic_ns": time.monotonic_ns(),
                        "workstation_receive_realtime_ns": time.time_ns(),
                    }
                    n = min(len(msg.motor_state), SHARPA_DOF)
                    for i in range(n):
                        left_angles[i] = math.radians(msg.motor_state[i].q)

                msg = sub_right.Read()
                if msg is not None and msg.motor_state:
                    state_timestamps["right"] = {
                        "workstation_receive_monotonic_ns": time.monotonic_ns(),
                        "workstation_receive_realtime_ns": time.time_ns(),
                    }
                    n = min(len(msg.motor_state), SHARPA_DOF)
                    for i in range(n):
                        right_angles[i] = math.radians(msg.motor_state[i].q)

                msg = cmd_sub_left.Read()
                left_desired_q = _sharpa_desired_q(msg)
                if left_desired_q is not None:
                    desired_timestamps["left"] = {
                        "workstation_receive_monotonic_ns": time.monotonic_ns(),
                        "workstation_receive_realtime_ns": time.time_ns(),
                    }
                    desired_q["left"] = left_desired_q

                msg = cmd_sub_right.Read()
                right_desired_q = _sharpa_desired_q(msg)
                if right_desired_q is not None:
                    desired_timestamps["right"] = {
                        "workstation_receive_monotonic_ns": time.monotonic_ns(),
                        "workstation_receive_realtime_ns": time.time_ns(),
                    }
                    desired_q["right"] = right_desired_q

                with self._snapshot_lock:
                    self._left_angles = left_angles.copy()
                    self._right_angles = right_angles.copy()
                    self._state_timestamps = {
                        "left": dict(state_timestamps["left"]),
                        "right": dict(state_timestamps["right"]),
                    }
                    self._desired_q = {
                        "left": None if desired_q["left"] is None else desired_q["left"].copy(),
                        "right": None if desired_q["right"] is None else desired_q["right"].copy(),
                    }
                    self._desired_timestamps = {
                        "left": dict(desired_timestamps["left"]),
                        "right": dict(desired_timestamps["right"]),
                    }

                if dual_hand_state_array is not None and dual_hand_action_array is not None:
                    combined = np.concatenate([left_angles, right_angles])
                    with dual_hand_data_lock:
                        dual_hand_state_array[:] = combined
                        dual_hand_action_array[:] = combined

                dt = time.time() - t0
                time.sleep(max(0.0, period - dt))
        finally:
            logger_mp.info("[SharpaWave] control thread closed.")


class SharpaTactile_Subscriber:
    """
    ZMQ SUB client for the Sharpa tactile stream published by sharpa_dds_bridge
    on Thor. Maintains a per-channel latest-frame cache and exposes snapshot()
    for the recorder.

    The bridge sends one message per fingertip (10 channels total) at ~30 Hz.
    Wire format: see teleop/utils/sharpa_tactile_wire.py.

    Args:
        host: bridge hostname or IP (e.g. "thor.local" or "192.168.123.x")
        port: bridge tactile PUB port (must match the bridge's --tactile-port)
    """

    def __init__(self, host, port):
        import zmq

        self._host = host
        self._port = port
        self._zmq = zmq

        # channel -> (wire.TactileMessage, receive_monotonic_ns, receive_realtime_ns)
        self._cache = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()

        logger_mp.info(
            f"Initialize SharpaTactile_Subscriber (ZMQ SUB tcp://{host}:{port})..."
        )
        self._thread = threading.Thread(target=self._sub_loop, daemon=True)
        self._thread.start()
        logger_mp.info("Initialize SharpaTactile_Subscriber OK!")

    def _sub_loop(self):
        zmq = self._zmq
        ctx = zmq.Context.instance()
        socket = ctx.socket(zmq.SUB)
        # HWM sized for one full hardware tick (10 fingers) × a few ticks of slack.
        socket.setsockopt(zmq.RCVHWM, 30)
        socket.setsockopt(zmq.LINGER, 0)
        socket.connect(f"tcp://{self._host}:{self._port}")
        socket.setsockopt_string(zmq.SUBSCRIBE, "")

        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)

        try:
            while not self._stop.is_set():
                events = dict(poller.poll(timeout=100))
                if socket not in events:
                    continue
                # Drain everything the SUB has buffered; bridge bursts 10 msgs/tick.
                while True:
                    try:
                        payload = socket.recv(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    receive_monotonic_ns = time.monotonic_ns()
                    receive_realtime_ns = time.time_ns()
                    try:
                        msg = wire.unpack(payload)
                    except Exception as e:
                        logger_mp.warning(f"[SharpaTactile] decode failed: {e}")
                        continue
                    # Defensive: a malformed channel would crash snapshot() later,
                    # not here. Drop it now and log so the recorder loop stays alive.
                    if not (0 <= msg.channel < wire.NUM_CHANNELS):
                        logger_mp.warning(
                            f"[SharpaTactile] drop msg with bad channel={msg.channel}"
                        )
                        continue
                    self._store_message(
                        msg,
                        receive_monotonic_ns=receive_monotonic_ns,
                        receive_realtime_ns=receive_realtime_ns,
                    )
        finally:
            socket.close()
            logger_mp.info("[SharpaTactile] subscriber thread closed.")

    def snapshot(self):
        """Return latest tactile data for all available fingers.

        Shape:
            {
              "left_ee":  {finger_name: {"deform": ndarray(240,240) uint8,
                                          "f6": [float]*6,
                                          "contact_point": [float]*3N,
                                          "ts": float,
                                          "frame_id": int,
                                          "joint_qpos": [float]*22 (radians, optional),
                                          "workstation_receive_monotonic_ns": int,
                                          "workstation_receive_realtime_ns": int}, ...},
              "right_ee": {...}
            }

        Fingers with no data yet are omitted. Returned ndarrays alias the wire
        payload (read-only); convert to a writable copy only if the caller
        needs to mutate. F6 and CONTACT_POINT are returned as plain Python lists
        so the result is directly JSON-serializable.
        """
        with self._lock:
            cached = list(self._cache.values())

        out = {"left_ee": {}, "right_ee": {}}
        for msg, receive_monotonic_ns, receive_realtime_ns in cached:
            hand, finger = msg.hand_finger
            entry = {
                "deform": msg.deform,
                "f6": [float(x) for x in msg.f6],
                "contact_point": [float(x) for x in msg.contact_point],
                "ts": msg.ts,
                "frame_id": msg.frame_id,
                "workstation_receive_monotonic_ns": receive_monotonic_ns,
                "workstation_receive_realtime_ns": receive_realtime_ns,
            }
            if msg.joints is not None:
                entry["joint_qpos"] = [math.radians(float(x)) for x in msg.joints]
            out[hand][finger] = entry
        return out

    def _store_message(self, msg, *, receive_monotonic_ns, receive_realtime_ns):
        """Atomically pair a tactile frame with its workstation receive clocks."""
        with self._lock:
            self._cache[msg.channel] = (
                msg,
                int(receive_monotonic_ns),
                int(receive_realtime_ns),
            )

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
