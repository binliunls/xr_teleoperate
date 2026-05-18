"""
Replay a LeRobot episode on the H2 robot.

Arm actions are sent via DDS (H2_ArmController).
Hand actions are sent via Sharpa SDK (SharpaWaveManager).
Both can be toggled independently for debugging.

Usage (tv conda env):
    # replay both arm and hands
    python teleop/replay_episode.py --dataset-dir /home/nvidia/lerobot/pick_apple

    # arm only (no hand hardware needed)
    python teleop/replay_episode.py --dataset-dir /home/nvidia/lerobot/pick_apple --no-hands

    # hands only
    python teleop/replay_episode.py --dataset-dir /home/nvidia/lerobot/pick_apple --no-arm

    # specific episode, 0.5× speed
    python teleop/replay_episode.py --dataset-dir /home/nvidia/lerobot/pick_apple \
        --episode 0 --speed 0.5
"""

import argparse
import os
import sys
import time

# ensure the xr_teleoperate root is on sys.path so `teleop.*` imports work
# regardless of which directory the script is run from
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import numpy as np
import torch

# ── dataset indices ───────────────────────────────────────────────────────────
ARM_SLICE = slice(0, 14)  # left(0:7) + right(7:14) arm joints
LEFT_EE_SLICE = slice(14, 36)  # left  Sharpa hand 22 DOF
RIGHT_EE_SLICE = slice(36, 58)  # right Sharpa hand 22 DOF

SHARPA_SDK_PATH = os.environ.get("SHARPA_SDK_PATH", "/usr/lib/sharpa-wave-sdk/python")
UNITREE_SDK_PATH = os.path.expanduser("~/unitree_sdk2_python")


# ── Sharpa hand helpers ───────────────────────────────────────────────────────


def _add_sharpa_path():
    if SHARPA_SDK_PATH not in sys.path:
        sys.path.insert(0, SHARPA_SDK_PATH)


def connect_hand(side: str, speed_coeff: float = 0.5, current_coeff: float = 0.6):
    """Connect to one Sharpa hand by side and return the hand object, or None on failure."""
    _add_sharpa_path()
    from sharpa import SharpaWaveManager, ControlMode, ControlSource, HandSide

    hand_side = HandSide.LEFT if side == "left" else HandSide.RIGHT
    manager = SharpaWaveManager.get_instance()
    time.sleep(1.0)

    print(f"[{side}] connecting to {hand_side.name} hand...")
    hand = None
    try:
        hand = manager.connect(hand_side)
        print(f"[{side}] connected.")
    except Exception as e:
        print(f"[{side}] WARNING: could not connect — {e}. Skipping.")
        return None

    if hand is None:
        print(f"[{side}] WARNING: manager.connect returned None — skipping.")
        return None

    err = hand.set_control_mode(ControlMode.POSITION)
    if err.code != 0:
        print(f"[{side}] set_control_mode failed: {err.message}")
        return None

    err = hand.set_speed_coeff(speed_coeff)
    if err.code != 0:
        print(f"[{side}] set_speed_coeff failed: {err.message}")

    err = hand.set_current_coeff(current_coeff)
    if err.code != 0:
        print(f"[{side}] set_current_coeff failed: {err.message}")

    err = hand.set_control_source(ControlSource.SDK)
    if err.code != 0:
        print(f"[{side}] set_control_source failed: {err.message}")
        return None

    hand.start()
    # reset to neutral
    hand.set_joint_position([0.0] * 22, True)
    print(f"[{side}] Sharpa hand ready.")
    return hand


def disconnect_hand(hand, side: str):
    if hand is None:
        return
    _add_sharpa_path()
    from sharpa import SharpaWaveManager

    try:
        hand.set_joint_position([0.0] * 22, True)
        time.sleep(0.5)
        hand.stop()
    except Exception as e:
        print(f"[{side}] stop error: {e}")
    try:
        SharpaWaveManager.get_instance().disconnect_all()
    except Exception:
        pass
    print(f"[{side}] disconnected.")


# ── dataset loading ───────────────────────────────────────────────────────────


def load_episode(dataset_dir: str, episode_idx: int):
    """Return action tensor of shape (T, 58) for the requested episode.

    dataset_dir can be either:
      - the repo root containing meta/info.json  (repo_id inferred from info.json)
      - parent/repo_id  (repo_id = basename, root = parent)
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset_dir = os.path.expanduser(dataset_dir)
    # root must be the directory that directly contains meta/info.json
    if os.path.exists(os.path.join(dataset_dir, "meta", "info.json")):
        root = dataset_dir
        repo_id = os.path.basename(dataset_dir.rstrip("/"))
    else:
        # assume dataset_dir is parent/repo_id
        root = os.path.dirname(dataset_dir)
        repo_id = os.path.basename(dataset_dir.rstrip("/"))

    dataset = LeRobotDataset(repo_id=repo_id, root=root)
    ep_indices = dataset.hf_dataset["episode_index"]
    indices = [i for i, ep in enumerate(ep_indices) if ep == episode_idx]
    if not indices:
        raise ValueError(f"Episode {episode_idx} not found in dataset at {dataset_dir}.")

    actions = []
    for i in indices:
        sample = dataset[i]
        actions.append(sample["action"])

    actions = torch.stack(actions)  # (T, 58)
    print(f"Loaded episode {episode_idx}: {len(actions)} frames, " f"action shape={tuple(actions.shape)}")
    return actions


# ── main replay loop ──────────────────────────────────────────────────────────


def _countdown(seconds: int, label: str):
    for i in range(seconds, 0, -1):
        print(f"  {label} {i}s", end="\r", flush=True)
        time.sleep(1.0)
    print(f"  {label} done.          ")


def _go_home(arm_ctrl, replay_hands, left_hand, right_hand, hold_s: int):
    """Move arm to home (q=0) and hands to neutral, then hold for hold_s seconds."""
    print(f"\nGoing to home position (hold {hold_s}s)...")
    if arm_ctrl is not None:
        arm_ctrl.ctrl_dual_arm_go_home()
    if replay_hands:
        if left_hand is not None:
            left_hand.set_joint_position([0.0] * 22, True)
        if right_hand is not None:
            right_hand.set_joint_position([0.0] * 22, True)
    _countdown(hold_s, "holding home")


def _run_episode(arm_ctrl, replay_hands, left_hand, right_hand, actions, dt):
    """Play through all frames. Returns True if completed, False if interrupted."""
    T = len(actions)
    for frame_idx in range(T):
        t0 = time.time()
        action = actions[frame_idx].numpy()

        if arm_ctrl is not None:
            arm_ctrl.ctrl_dual_arm(
                action[ARM_SLICE].astype(np.float64),
                np.zeros(14, dtype=np.float64),
            )

        if replay_hands:
            if left_hand is not None:
                err = left_hand.set_joint_position(action[LEFT_EE_SLICE].tolist(), False)
                if err.code != 0:
                    print(f"[frame {frame_idx}] left hand error: {err.message}")
            if right_hand is not None:
                err = right_hand.set_joint_position(action[RIGHT_EE_SLICE].tolist(), False)
                if err.code != 0:
                    print(f"[frame {frame_idx}] right hand error: {err.message}")

        elapsed = time.time() - t0
        time.sleep(max(0.0, dt - elapsed))

        if (frame_idx + 1) % 30 == 0:
            print(f"  frame {frame_idx + 1}/{T}  loop_dt={1000*(elapsed + max(0.0, dt-elapsed)):.1f}ms")

    return True


def replay(
    dataset_dir: str,
    episode_idx: int,
    num_repeats: int,
    replay_arm: bool,
    replay_hands: bool,
    speed: float,
    hand_speed_coeff: float,
    hand_current_coeff: float,
):
    actions = load_episode(dataset_dir, episode_idx)
    fps = 30.0
    dt = (1.0 / fps) / speed

    # ── arm setup ────────────────────────────────────────────────────────────
    arm_ctrl = None
    if replay_arm:
        if UNITREE_SDK_PATH not in sys.path:
            sys.path.insert(0, UNITREE_SDK_PATH)
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        ChannelFactoryInitialize(0)
        from teleop.robot_control.robot_arm import H2_ArmController

        print("Initialising H2_ArmController...")
        arm_ctrl = H2_ArmController()
        print("H2_ArmController ready.")

    # ── hand setup ───────────────────────────────────────────────────────────
    left_hand = None
    right_hand = None
    if replay_hands:
        print("Connecting left hand...")
        left_hand = connect_hand("left", hand_speed_coeff, hand_current_coeff)
        print("Connecting right hand...")
        right_hand = connect_hand("right", hand_speed_coeff, hand_current_coeff)
        print("Waiting for hands to reach neutral position...")
        time.sleep(2.0)

    # ── synchronized start: go home and hold 20s ─────────────────────────────
    print(f"\nAll hardware ready.  arm={replay_arm}  hands={replay_hands}")
    print(f"Will replay {num_repeats}x at {fps * speed:.1f} Hz  (speed={speed}x)\n")
    _go_home(arm_ctrl, replay_hands, left_hand, right_hand, hold_s=20)

    # ── repeat loop ──────────────────────────────────────────────────────────
    interrupted = False
    try:
        for rep in range(num_repeats):
            print(f"\n=== Replay {rep + 1}/{num_repeats} ===")
            _run_episode(arm_ctrl, replay_hands, left_hand, right_hand, actions, dt)

            if rep < num_repeats - 1:
                # between repeats: go home and hold 2s
                _go_home(arm_ctrl, replay_hands, left_hand, right_hand, hold_s=10)

    except KeyboardInterrupt:
        interrupted = True
        print("\nReplay interrupted by user.")

    # ── final hold: last action for 20s ──────────────────────────────────────
    if not interrupted:
        print("\nAll repeats done. Holding last position for 20s...")
        last_arm_q = actions[-1].numpy()[ARM_SLICE].astype(np.float64)
        for i in range(20, 0, -1):
            print(f"  {i}s", end="\r", flush=True)
            if arm_ctrl is not None:
                arm_ctrl.ctrl_dual_arm(last_arm_q, np.zeros(14, dtype=np.float64))
            time.sleep(1.0)
        print("  Done.          ")

    if replay_hands:
        disconnect_hand(left_hand, "left")
        disconnect_hand(right_hand, "right")
    print("Replay finished.")


# ── CLI ───────────────────────────────────────────────────────────────────────


def parse_args():
    ap = argparse.ArgumentParser(description="Replay a LeRobot episode on H2 + Sharpa hands.")
    ap.add_argument(
        "--dataset-dir",
        required=True,
        help="Root directory of the LeRobot dataset, e.g. /home/nvidia/lerobot/pick_apple",
    )
    ap.add_argument("--episode", type=int, default=0, help="Episode index to replay (default: 0)")
    ap.add_argument("--num-repeats", type=int, default=1, help="Number of times to replay the episode (default: 1)")
    ap.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier (default: 1.0)")

    arm_group = ap.add_mutually_exclusive_group()
    arm_group.add_argument("--arm", dest="replay_arm", action="store_true", default=True)
    arm_group.add_argument("--no-arm", dest="replay_arm", action="store_false")

    hand_group = ap.add_mutually_exclusive_group()
    hand_group.add_argument("--hands", dest="replay_hands", action="store_true", default=True)
    hand_group.add_argument("--no-hands", dest="replay_hands", action="store_false")

    ap.add_argument("--hand-speed-coeff", type=float, default=0.5, help="Sharpa speed coefficient 0–1 (default: 0.5)")
    ap.add_argument(
        "--hand-current-coeff", type=float, default=0.6, help="Sharpa current coefficient 0–1 (default: 0.6)"
    )
    return ap.parse_args()


if __name__ == "__main__":
    args = parse_args()
    replay(
        dataset_dir=args.dataset_dir,
        episode_idx=args.episode,
        num_repeats=args.num_repeats,
        replay_arm=args.replay_arm,
        replay_hands=args.replay_hands,
        speed=args.speed,
        hand_speed_coeff=args.hand_speed_coeff,
        hand_current_coeff=args.hand_current_coeff,
    )
