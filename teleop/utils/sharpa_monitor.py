"""
Standalone Sharpa hand-state visualizer.

Connects to one or both Sharpa hands via the `sharpa` SDK and streams joint
angles + tactile (5x F6 per hand) into Rerun. No teleop, no recording, no
arm controller — just observe.

Usage:
    python -m teleop.utils.sharpa_monitor                  # both hands, real
    python -m teleop.utils.sharpa_monitor --side right     # right only
    python -m teleop.utils.sharpa_monitor --sim            # no hardware (zeros)
    python -m teleop.utils.sharpa_monitor --rate 60        # log at 60 Hz
"""

import argparse
import time
import numpy as np
import rerun as rr


SHARPA_DOF = 22
F6_AXES = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]

# Joint names in URDF/SDK order (must match hand_retargeting.py)
LEFT_JOINT_NAMES = [
    "thumb_CMC_FE", "thumb_CMC_AA", "thumb_MCP_FE", "thumb_MCP_AA", "thumb_IP",
    "index_MCP_FE", "index_MCP_AA", "index_PIP", "index_DIP",
    "middle_MCP_FE", "middle_MCP_AA", "middle_PIP", "middle_DIP",
    "ring_MCP_FE", "ring_MCP_AA", "ring_PIP", "ring_DIP",
    "pinky_CMC", "pinky_MCP_FE", "pinky_MCP_AA", "pinky_PIP", "pinky_DIP",
]


def _open_hand(side):
    """Connect a single Sharpa hand. Returns (manager, hand) or (None, None) on failure."""
    from sharpa import SharpaWaveManager, HandSide

    manager = SharpaWaveManager.get_instance()
    for _ in range(40):
        if manager.get_all_device_sn():
            break
        time.sleep(0.5)
    else:
        raise RuntimeError("No Sharpa devices discovered (heartbeat timeout).")

    hand_side = HandSide.LEFT if side == "left" else HandSide.RIGHT
    hand = manager.connect(hand_side)
    hand.start()
    t0 = time.time()
    while not (hand.is_hand_ready() and hand.is_tactile_ready()):
        if time.time() - t0 > 10.0:
            raise RuntimeError(f"{side} hand not ready after 10s.")
        time.sleep(0.05)
    # Read-only: do NOT change control source / enable / mode.
    print(f"[sharpa_monitor] {side} hand ready (read-only).")
    return manager, hand


def _setup_blueprint(sides):
    import rerun.blueprint as rrb
    views = []
    for side in sides:
        views.append(rrb.TimeSeriesView(
            origin=f"{side}/joints",
            plot_legend=rrb.PlotLegend(visible=True),
        ))
        views.append(rrb.TimeSeriesView(
            origin=f"{side}/tactile",
            plot_legend=rrb.PlotLegend(visible=True),
        ))
    grid = rrb.Grid(contents=views, grid_columns=2)
    rr.send_blueprint(grid)


def _log_state(side, angles, tactile_cache):
    for i, name in enumerate(LEFT_JOINT_NAMES):
        if i < len(angles):
            rr.log(f"{side}/joints/{name}", rr.Scalar(float(angles[i])))
    for ch in range(5):
        f6 = tactile_cache[ch]
        fmag = float(np.linalg.norm(f6[:3]))
        rr.log(f"{side}/tactile/{FINGER_NAMES[ch]}/Fmag", rr.Scalar(fmag))
        for i, ax in enumerate(F6_AXES):
            rr.log(f"{side}/tactile/{FINGER_NAMES[ch]}/{ax}", rr.Scalar(float(f6[i])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right", "both"], default="both")
    ap.add_argument("--rate", type=float, default=30.0, help="Hz")
    ap.add_argument("--sim", action="store_true", help="No hardware; emit zeros.")
    args = ap.parse_args()

    sides = ["left", "right"] if args.side == "both" else [args.side]

    rr.init("sharpa_monitor", spawn=True)
    _setup_blueprint(sides)

    # tactile_cache[side][channel] = 6-vec; filled by SDK callbacks
    tactile_cache = {s: [np.zeros(6) for _ in range(5)] for s in sides}

    def _make_cb(side):
        def _cb(frame):
            try:
                ch = frame["channel"] if isinstance(frame, dict) else frame.channel
                if isinstance(frame, dict):
                    f6 = np.asarray(frame["content"]["F6"], dtype=np.float64)
                else:
                    f6 = np.asarray(getattr(frame, "f6_data", None) or frame.content["F6"],
                                    dtype=np.float64)
                if f6.size != 6:
                    return
                tactile_cache[side][ch % 5] = f6
            except Exception as e:
                print(f"[sharpa_monitor] tactile cb {side} error: {e}")
        return _cb

    manager = None
    hands = {}
    if not args.sim:
        for side in sides:
            manager, h = _open_hand(side)
            hands[side] = h
            try:
                h.set_tactile_callback(_make_cb(side))
            except Exception as e:
                print(f"[sharpa_monitor] failed to set {side} tactile callback: {e}")
    else:
        print("[sharpa_monitor] sim mode — emitting zeros.")

    period = 1.0 / args.rate
    seq = 0
    try:
        while True:
            t0 = time.time()
            rr.set_time_sequence("idx", seq); seq += 1
            for side in sides:
                if args.sim:
                    angles = np.zeros(SHARPA_DOF)
                else:
                    angles = np.asarray(hands[side].get_states().angles, dtype=np.float64)
                _log_state(side, angles, tactile_cache[side])
            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        print("\n[sharpa_monitor] stopping.")
    finally:
        if manager is not None:
            try:
                for h in hands.values():
                    h.stop()
                manager.disconnect_all()
            except Exception:
                pass


if __name__ == "__main__":
    main()
