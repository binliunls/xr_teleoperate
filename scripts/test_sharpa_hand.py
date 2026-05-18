"""
Test script for a single Sharpa hand (left or right).

Runs a sequence of gestures on the chosen hand:
  1. Neutral (all zeros)
  2. Close fist (flex all fingers)
  3. Open hand (back to neutral)
  4. Finger-by-finger curl (index → middle → ring → pinky → thumb)
  5. Return to neutral and disconnect

Can run in two modes:
  --mode sdk   Direct SDK connection (run on Thor where hands are reachable).
  --mode dds   Via the DDS bridge (run on workstation; requires sharpa_dds_bridge
               running on Thor first, with the matching --side).

Joint order (22 DOF, radians):
  [0]  Thumb  CMC FE      range  0 ~ 50 deg
  [1]  Thumb  CMC AA      range  0 ~ 10 deg
  [2]  Thumb  MCP FE      range  0 ~ 30 deg
  [3]  Thumb  MCP AA      range  0 ~ 10 deg
  [4]  Thumb  DIP FE      range  0 ~ 40 deg
  [5]  Index  MCP FE      range  0 ~ 20 deg
  [6]  Index  MCP AA      range -20 ~ 20 deg
  [7]  Index  PIP FE      range  0 ~ 20 deg
  [8]  Index  DIP FE      range  0 ~ 20 deg
  [9]  Middle MCP FE      range  0 ~ 20 deg
  [10] Middle MCP AA      range -20 ~ 20 deg
  [11] Middle PIP FE      range  0 ~ 20 deg
  [12] Middle DIP FE      range  0 ~ 20 deg
  [13] Ring   MCP FE      range  0 ~ 20 deg
  [14] Ring   MCP AA      range -20 ~ 20 deg
  [15] Ring   PIP FE      range  0 ~ 20 deg
  [16] Ring   DIP FE      range  0 ~ 20 deg
  [17] Pinky  CMC FE      range  0 ~ 10 deg
  [18] Pinky  MCP FE      range  0 ~ 20 deg
  [19] Pinky  MCP AA      range -20 ~ 20 deg
  [20] Pinky  PIP FE      range  0 ~ 20 deg
  [21] Pinky  DIP FE      range  0 ~ 20 deg

Usage:
    # On Thor (direct SDK):
    python scripts/test_sharpa_left_hand.py --mode sdk --side left
    python scripts/test_sharpa_left_hand.py --mode sdk --side right

    # On workstation (via DDS bridge — bridge must run with matching --side):
    python scripts/test_sharpa_left_hand.py --mode dds --side right
"""

import argparse
import math
import os
import sys
import time

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)


# ── gesture definitions (degrees, converted to radians on send) ────────────────

NEUTRAL = [0.0] * 22

FIST = [
    # thumb
    40.0,
    5.0,
    25.0,
    5.0,
    30.0,
    # index
    18.0,
    0.0,
    18.0,
    18.0,
    # middle
    18.0,
    0.0,
    18.0,
    18.0,
    # ring
    18.0,
    0.0,
    18.0,
    18.0,
    # pinky
    8.0,
    18.0,
    0.0,
    18.0,
    18.0,
]


# Finger flex-only positions: only the FE joints of one finger, rest neutral
def _finger_pose(finger: str) -> list:
    pose = [0.0] * 22
    if finger == "index":
        pose[5], pose[7], pose[8] = 18.0, 18.0, 18.0
    elif finger == "middle":
        pose[9], pose[11], pose[12] = 18.0, 18.0, 18.0
    elif finger == "ring":
        pose[13], pose[15], pose[16] = 18.0, 18.0, 18.0
    elif finger == "pinky":
        pose[17], pose[18], pose[20], pose[21] = 8.0, 18.0, 18.0, 18.0
    elif finger == "thumb":
        pose[0], pose[2], pose[4] = 40.0, 25.0, 30.0
    return pose


def deg2rad(deg_list: list) -> list:
    return [math.radians(d) for d in deg_list]


# ── hand setup / teardown ─────────────────────────────────────────────────────


def connect_hand_sdk(side: str, speed: float, discovery_timeout: float = 300.0):
    SHARPA_SDK_PATH = os.environ.get("SHARPA_SDK_PATH", "/usr/lib/sharpa-wave-sdk/python")
    if SHARPA_SDK_PATH not in sys.path:
        sys.path.insert(0, SHARPA_SDK_PATH)
    from sharpa import SharpaWaveManager, ControlMode, ControlSource, HandSide

    manager = SharpaWaveManager.get_instance()
    print(f"Waiting for device discovery (up to {discovery_timeout:.0f}s)...")
    deadline = time.perf_counter() + discovery_timeout
    while time.perf_counter() < deadline:
        devices = manager.get_all_device_sn()
        if devices:
            print(f"  Found {len(devices)} device(s): {devices}")
            break
        time.sleep(1.0)
    else:
        raise RuntimeError(f"No devices found after {discovery_timeout:.0f}s")

    hand_side = HandSide.LEFT if side == "left" else HandSide.RIGHT
    hand = manager.connect(hand_side)
    if hand is None:
        raise RuntimeError(f"manager.connect(HandSide.{side.upper()}) returned None")

    for name, call in [
        ("set_control_mode", lambda: hand.set_control_mode(ControlMode.POSITION)),
        ("set_speed_coeff", lambda: hand.set_speed_coeff(speed)),
        ("set_current_coeff", lambda: hand.set_current_coeff(0.6)),
        ("set_control_source", lambda: hand.set_control_source(ControlSource.SDK)),
    ]:
        err = call()
        if err.code != 0:
            raise RuntimeError(f"{name} failed: {err.message}")

    hand.start()
    print(f"{side.capitalize()} hand connected (SDK).")
    return hand


def disconnect_hand_sdk(hand, side: str) -> None:
    print(f"Returning {side} hand to neutral and disconnecting...")
    hand.set_joint_position(deg2rad(NEUTRAL), True)
    time.sleep(1.5)
    hand.stop()
    from sharpa import SharpaWaveManager

    SharpaWaveManager.get_instance().disconnect_all()
    print(f"{side.capitalize()} hand disconnected.")


def connect_hand_dds(side: str):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize(0)
    from teleop.robot_control.robot_hand_sharpa_dds import SharpaHandDDSClient

    client = SharpaHandDDSClient(side)
    print(f"{side.capitalize()} hand DDS client ready (bridge must be running on Thor).")
    return client


def disconnect_hand_dds(client, side: str) -> None:
    client.go_neutral()
    time.sleep(0.5)
    print(f"{side.capitalize()} hand DDS client disconnected.")


# ── gesture helpers ───────────────────────────────────────────────────────────


def move(hand, deg_list: list, label: str, pause: float) -> None:
    print(f"  → {label}")
    err = hand.set_joint_position(deg2rad(deg_list), True)
    # DDS client returns None; SDK returns an Error object with .code
    if err is not None and getattr(err, "code", 0) != 0:
        print(f"    WARNING: set_joint_position failed: {err.message}")
    time.sleep(pause)


def read_state(hand) -> None:
    err, angles = hand.get_joint_position_degree()
    code = getattr(err, "code", err)  # SDK: Error obj; DDS: int
    if code != 0:
        msg = getattr(err, "message", str(err))
        print(f"  get_joint_position_degree failed: {msg}")
        return
    print("  current angles (deg):")
    names = [
        "Thumb CMC FE",
        "Thumb CMC AA",
        "Thumb MCP FE",
        "Thumb MCP AA",
        "Thumb DIP FE",
        "Index MCP FE",
        "Index MCP AA",
        "Index PIP FE",
        "Index DIP FE",
        "Middle MCP FE",
        "Middle MCP AA",
        "Middle PIP FE",
        "Middle DIP FE",
        "Ring MCP FE",
        "Ring MCP AA",
        "Ring PIP FE",
        "Ring DIP FE",
        "Pinky CMC FE",
        "Pinky MCP FE",
        "Pinky MCP AA",
        "Pinky PIP FE",
        "Pinky DIP FE",
    ]
    for i, (name, angle) in enumerate(zip(names, angles)):
        print(f"    [{i:2d}] {name:<25} {angle:6.2f}°")


# ── main ──────────────────────────────────────────────────────────────────────


def run(args):
    side = args.side
    if args.mode == "dds":
        hand = connect_hand_dds(side)

        def disconnect_fn(h):
            disconnect_hand_dds(h, side)
    else:
        hand = connect_hand_sdk(side, args.speed)

        def disconnect_fn(h):
            disconnect_hand_sdk(h, side)

    try:
        print("\n--- Step 1: neutral ---")
        move(hand, NEUTRAL, "neutral", args.pause)
        read_state(hand)

        print("\n--- Step 2: fist ---")
        move(hand, FIST, "fist", args.pause)
        read_state(hand)

        print("\n--- Step 3: open hand ---")
        move(hand, NEUTRAL, "neutral", args.pause)

        print("\n--- Step 4: finger-by-finger curl ---")
        for finger in ["index", "middle", "ring", "pinky", "thumb"]:
            move(hand, _finger_pose(finger), finger, args.pause)
            move(hand, NEUTRAL, "neutral", args.pause * 0.5)

        print("\n--- Step 5: full fist → open ---")
        move(hand, FIST, "fist", args.pause)
        move(hand, NEUTRAL, "neutral", args.pause)

        print("\nAll gestures complete.")
    finally:
        disconnect_fn(hand)


def parse_args():
    ap = argparse.ArgumentParser(description="Test a single Sharpa hand with a gesture sequence.")
    ap.add_argument(
        "--mode",
        choices=["sdk", "dds"],
        default="sdk",
        help="sdk: direct SDK (run on Thor) | dds: via bridge (run on workstation)",
    )
    ap.add_argument("--side", choices=["left", "right"], default="left", help="Which hand to test (default: left)")
    ap.add_argument("--speed", type=float, default=0.3, help="Speed coefficient (sdk mode only, default: 0.3)")
    ap.add_argument("--pause", type=float, default=2.0, help="Seconds to hold each pose (default: 2.0)")
    return ap.parse_args()


if __name__ == "__main__":
    run(parse_args())
