"""
Sharpa hand DDS bridge — run on Thor.

Connects to both Sharpa hands via the Sharpa SDK (192.168.124.x network)
and bridges them to DDS so the workstation can command and read state
over the 192.168.123.x network without needing the SDK installed.

DDS topics:
  Subscribe  rt/sharpa/left/cmd    HandCmd_  — 22 x motor_cmd[i].q (radians)
  Subscribe  rt/sharpa/right/cmd   HandCmd_  — 22 x motor_cmd[i].q (radians)
  Publish    rt/sharpa/left/state  HandState_ — 22 x motor_state[i].q (degrees)
  Publish    rt/sharpa/right/state HandState_ — 22 x motor_state[i].q (degrees)

Usage:
    python scripts/sharpa_dds_bridge.py [--speed 0.5] [--state-hz 50]
"""

import argparse
import os
import sys
import threading
import time

SHARPA_SDK_PATH = os.environ.get("SHARPA_SDK_PATH", "/usr/lib/sharpa-wave-sdk/python")
if SHARPA_SDK_PATH not in sys.path:
    sys.path.insert(0, SHARPA_SDK_PATH)

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

TOPIC_LEFT_CMD = "rt/sharpa/left/cmd"
TOPIC_RIGHT_CMD = "rt/sharpa/right/cmd"
TOPIC_LEFT_STATE = "rt/sharpa/left/state"
TOPIC_RIGHT_STATE = "rt/sharpa/right/state"
NUM_JOINTS = 22


# ── DDS helpers ───────────────────────────────────────────────────────────────


def _make_hand_state_msg(angles_deg: list):
    """Build a HandState_ message from a list of 22 joint angles in degrees."""
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_, MotorState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import PressSensorState_, IMUState_

    motor_states = []
    for q in angles_deg:
        ms = MotorState_()
        ms.q = float(q)
        motor_states.append(ms)
    msg = HandState_()
    msg.motor_state = motor_states
    msg.press_sensor_state = [PressSensorState_() for _ in range(NUM_JOINTS)]
    msg.imu_state = IMUState_()
    return msg


# ── Sharpa hand connection ────────────────────────────────────────────────────


def _connect_hand(side: str, speed: float, discovery_timeout: float = 300.0):
    from sharpa import SharpaWaveManager, ControlMode, ControlSource, HandSide

    manager = SharpaWaveManager.get_instance()
    print(f"[{side}] waiting for device discovery (up to {discovery_timeout:.0f}s)...")
    deadline = time.perf_counter() + discovery_timeout
    while time.perf_counter() < deadline:
        if manager.get_all_device_sn():
            break
        time.sleep(1.0)
    devices = manager.get_all_device_sn()
    if not devices:
        raise RuntimeError(f"[{side}] no devices found after {discovery_timeout:.0f}s")
    print(f"[{side}] devices found: {devices}")

    hand_side = HandSide.LEFT if side == "left" else HandSide.RIGHT
    hand = manager.connect(hand_side)
    if hand is None:
        raise RuntimeError(f"[{side}] manager.connect returned None")

    for name, call in [
        ("set_control_mode", lambda: hand.set_control_mode(ControlMode.POSITION)),
        ("set_speed_coeff", lambda: hand.set_speed_coeff(speed)),
        ("set_current_coeff", lambda: hand.set_current_coeff(0.6)),
        ("set_control_source", lambda: hand.set_control_source(ControlSource.SDK)),
    ]:
        err = call()
        if err.code != 0:
            raise RuntimeError(f"[{side}] {name} failed: {err.message}")

    hand.start()
    hand.set_joint_position([0.0] * NUM_JOINTS, True)
    print(f"[{side}] hand ready.")
    return hand


def _disconnect_hand(hand, side: str):
    if hand is None:
        return
    try:
        hand.set_joint_position([0.0] * NUM_JOINTS, True)
        time.sleep(1.0)
        hand.stop()
    except Exception as e:
        print(f"[{side}] stop error: {e}")
    try:
        from sharpa import SharpaWaveManager

        SharpaWaveManager.get_instance().disconnect_all()
    except Exception:
        pass
    print(f"[{side}] disconnected.")


# ── Bridge ────────────────────────────────────────────────────────────────────


class SharpaHandBridge:
    def __init__(self, side: str, hand, state_hz: float):
        self.side = side
        self.hand = hand
        self._cmd_lock = threading.Lock()
        self._cmd_positions = [0.0] * NUM_JOINTS  # radians
        self._running = False
        self._state_dt = 1.0 / state_hz

    def _on_cmd(self, msg):
        if msg is None or not msg.motor_cmd:
            return
        positions = [msg.motor_cmd[i].q for i in range(min(NUM_JOINTS, len(msg.motor_cmd)))]
        with self._cmd_lock:
            self._cmd_positions = positions

    def _cmd_thread(self):
        """Apply latest command to hand at 50 Hz."""
        while self._running:
            with self._cmd_lock:
                positions = list(self._cmd_positions)
            err = self.hand.set_joint_position(positions, False)
            if err.code != 0:
                print(f"[{self.side}] set_joint_position error: {err.message}")
            time.sleep(0.02)

    def _state_thread(self, publisher):
        """Read hand state and publish to DDS at state_hz."""
        while self._running:
            err, angles_deg = self.hand.get_joint_position_degree()
            if err.code == 0:
                msg = _make_hand_state_msg(angles_deg[:NUM_JOINTS])
                publisher.Write(msg)
            time.sleep(self._state_dt)

    def start(self, cmd_topic: str, state_topic: str):
        from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_

        pub = ChannelPublisher(state_topic, HandState_)
        pub.Init()

        sub = ChannelSubscriber(cmd_topic, HandCmd_)
        sub.Init(self._on_cmd, 1)

        self._running = True
        threading.Thread(target=self._cmd_thread, daemon=True).start()
        threading.Thread(target=self._state_thread, args=(pub,), daemon=True).start()
        print(f"[{self.side}] bridge running — cmd: {cmd_topic}  state: {state_topic}")

    def stop(self):
        self._running = False


# ── main ──────────────────────────────────────────────────────────────────────


def run(args):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize(0)

    print("Connecting hands (this may take a while on first boot)...")
    left_hand = _connect_hand("left", args.speed)
    right_hand = _connect_hand("right", args.speed)

    left_bridge = SharpaHandBridge("left", left_hand, args.state_hz)
    right_bridge = SharpaHandBridge("right", right_hand, args.state_hz)

    left_bridge.start(TOPIC_LEFT_CMD, TOPIC_LEFT_STATE)
    right_bridge.start(TOPIC_RIGHT_CMD, TOPIC_RIGHT_STATE)

    print("\nBridge running. Ctrl+C to stop.\n")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        left_bridge.stop()
        right_bridge.stop()
        _disconnect_hand(left_hand, "left")
        _disconnect_hand(right_hand, "right")


def parse_args():
    ap = argparse.ArgumentParser(description="Sharpa hand DDS bridge — run on Thor.")
    ap.add_argument("--speed", type=float, default=0.5, help="Hand speed coefficient (default: 0.5)")
    ap.add_argument("--state-hz", type=float, default=50.0, help="State publish rate Hz (default: 50)")
    return ap.parse_args()


if __name__ == "__main__":
    run(parse_args())
