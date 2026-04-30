"""
Sharpa joint-state reader with live matplotlib curves — DDS edition.

Reads hand joint states published by retargeting_manus_demo_multiprocess.py
via CycloneDDS (unitree_sdk2py).  Does NOT use the Sharpa SDK; no port conflicts.

Topics (published at -dds_hz by the retargeting script):
  rt/sharpa/left/state   — left  hand, 22 joints, motor_state[i].q in DEGREES
  rt/sharpa/right/state  — right hand, 22 joints, motor_state[i].q in DEGREES

Layout: rows = sides (1 or 2), cols = 5 fingers.  Each subplot overlays
that finger's joints as colored lines.  A rolling window of `window`
samples is shown on the X axis.

Usage:
    python sharpa_read_joints.py                          # both hands, 50 Hz, forever
    python sharpa_read_joints.py --side right             # right only
    python sharpa_read_joints.py --rate 30 --duration 60  # 30 Hz for 60 s
    python sharpa_read_joints.py --units deg -v           # degrees, verbose
    python sharpa_read_joints.py --no-viz                 # disable plot
    python sharpa_read_joints.py --window 600             # show last 600 samples
    python sharpa_read_joints.py -domain 1 -interface eth0
"""

import argparse
import logging
import math
import sys
import os
import time
from collections import deque

_UNITREE_SDK_PATH = os.path.expanduser("~/unitree_sdk2_python")
if os.path.isdir(_UNITREE_SDK_PATH) and _UNITREE_SDK_PATH not in sys.path:
    sys.path.insert(0, _UNITREE_SDK_PATH)

JOINT_NAMES = [
    "thumb_CMC_FE",
    "thumb_CMC_AA",
    "thumb_MCP_FE",
    "thumb_MCP_AA",
    "thumb_IP",
    "index_MCP_FE",
    "index_MCP_AA",
    "index_PIP",
    "index_DIP",
    "middle_MCP_FE",
    "middle_MCP_AA",
    "middle_PIP",
    "middle_DIP",
    "ring_MCP_FE",
    "ring_MCP_AA",
    "ring_PIP",
    "ring_DIP",
    "pinky_CMC",
    "pinky_MCP_FE",
    "pinky_MCP_AA",
    "pinky_PIP",
    "pinky_DIP",
]
FINGER_GROUPS = ["thumb", "index", "middle", "ring", "pinky"]
SHARPA_DOF = 22

TOPIC_LEFT = "rt/sharpa/left/state"
TOPIC_RIGHT = "rt/sharpa/right/state"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d | %(levelname)-7s | %(name)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("sharpa_read_joints")


def fmt_angles(angles_rad, units):
    if units == "deg":
        vals = [math.degrees(a) for a in angles_rad]
        suffix = "°"
    else:
        vals = list(angles_rad)
        suffix = "rad"
    return ", ".join(f"{n}={v:+7.3f}{suffix}" for n, v in zip(JOINT_NAMES, vals))


class LivePlot:
    """Matplotlib live joint-angle plotter.

    Layout: rows = sides (1 or 2), cols = 5 fingers. Each subplot overlays
    that finger's joints as colored lines. A rolling window of `window`
    samples is shown on the X axis (sample index).
    """

    def __init__(self, sides, units, window=300):
        import matplotlib

        matplotlib.use("TkAgg")  # interactive backend
        import matplotlib.pyplot as plt

        self.plt = plt
        self.sides = sides
        self.units = units
        self.window = window

        self.x = deque(maxlen=window)
        self.data = {(s, n): deque(maxlen=window) for s in sides for n in JOINT_NAMES}

        plt.ion()
        n_rows = len(sides)
        n_cols = len(FINGER_GROUPS)
        self.fig, axes = plt.subplots(
            n_rows,
            n_cols,
            figsize=(3.2 * n_cols, 2.5 * n_rows),
            sharex=True,
            squeeze=False,
        )
        self.fig.suptitle("Sharpa joint angles (live — DDS)")
        self.fig.canvas.manager.set_window_title("sharpa_read_joints")

        self.lines = {}
        self.axes = {}
        for r, side in enumerate(sides):
            for c, finger in enumerate(FINGER_GROUPS):
                ax = axes[r][c]
                self.axes[(side, finger)] = ax
                ax.set_title(f"{side} {finger}", fontsize=9)
                ax.grid(True, alpha=0.3)
                ax.set_ylabel(f"angle ({units})", fontsize=8)
                ax.tick_params(labelsize=7)
                for n in JOINT_NAMES:
                    if n.startswith(finger + "_") or n == finger:
                        (ln,) = ax.plot([], [], label=n.replace(finger + "_", ""), linewidth=1.2)
                        self.lines[(side, n)] = ln
                ax.legend(fontsize=6, loc="upper left", ncol=1)
        for c in range(n_cols):
            axes[-1][c].set_xlabel("sample idx", fontsize=8)

        self.fig.tight_layout(rect=[0, 0, 1, 0.96])
        self.fig.canvas.draw()
        self.plt.pause(0.001)

    def push(self, idx, side, angles_rad):
        """Push one sample. angles_rad: 22 values in radians."""
        if not self.x or self.x[-1] != idx:
            self.x.append(idx)
        for name, a in zip(JOINT_NAMES, angles_rad):
            v = math.degrees(a) if self.units == "deg" else a
            self.data[(side, name)].append(float(v))

    def redraw(self):
        xs = list(self.x)
        for (side, name), ln in self.lines.items():
            ys = list(self.data[(side, name)])
            n = min(len(xs), len(ys))
            ln.set_data(xs[-n:], ys[-n:])
        for ax in self.axes.values():
            ax.relim()
            ax.autoscale_view()
        try:
            self.fig.canvas.draw_idle()
            self.plt.pause(0.001)
        except Exception:
            pass

    def is_open(self):
        return self.plt.fignum_exists(self.fig.number)

    def close(self):
        try:
            self.plt.close(self.fig)
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right", "both"], default="both")
    ap.add_argument("--rate", type=float, default=50.0, help="poll Hz (default: 50)")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds to run; 0 = forever (default)")
    ap.add_argument("--units", choices=["rad", "deg"], default="rad")
    ap.add_argument("-v", "--verbose", action="store_true", help="Print full per-joint angle vectors each iteration.")
    ap.add_argument("--no-viz", action="store_true", help="Disable matplotlib visualization.")
    ap.add_argument(
        "--window", type=int, default=300, help="Rolling sample-window length for live plot (default: 300)."
    )
    ap.add_argument("-domain", type=int, default=0, help="DDS domain ID (default: 0)")
    ap.add_argument("-interface", type=str, default=None, help="Network interface, e.g. eth0")
    args = ap.parse_args()

    if args.verbose:
        log.setLevel(logging.DEBUG)

    sides = ["left", "right"] if args.side == "both" else [args.side]
    log.info(
        "config: sides=%s  rate=%.1fHz  duration=%s  units=%s  viz=%s  verbose=%s",
        sides,
        args.rate,
        "∞" if args.duration == 0 else f"{args.duration:.1f}s",
        args.units,
        not args.no_viz,
        args.verbose,
    )

    # ── DDS setup ────────────────────────────────────────────────────────────
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

    if args.interface:
        ChannelFactoryInitialize(args.domain, networkInterface=args.interface)
    else:
        ChannelFactoryInitialize(args.domain)

    subscribers = {}
    if "left" in sides:
        sub = ChannelSubscriber(TOPIC_LEFT, HandState_)
        sub.Init()
        subscribers["left"] = sub
    if "right" in sides:
        sub = ChannelSubscriber(TOPIC_RIGHT, HandState_)
        sub.Init()
        subscribers["right"] = sub

    log.info("Subscribed — waiting for retargeting script to publish on domain %d.", args.domain)

    # ── matplotlib ───────────────────────────────────────────────────────────
    plot = None
    if not args.no_viz:
        try:
            plot = LivePlot(sides, args.units, window=args.window)
            log.info(
                "matplotlib live plot ready: %d×%d grid (rows=sides, cols=fingers).", len(sides), len(FINGER_GROUPS)
            )
        except Exception as e:
            log.warning("matplotlib setup failed (%s) — continuing without viz.", e)

    # ── main loop ────────────────────────────────────────────────────────────
    log.info("=" * 60)
    log.info("Streaming joint states (%d DOF / hand). Ctrl+C to stop.", SHARPA_DOF)
    log.info("=" * 60)

    period = 1.0 / args.rate
    deadline = None if args.duration == 0 else time.time() + args.duration
    iter_idx = 0

    try:
        while True:
            t_loop = time.time()
            iter_idx += 1

            for side, sub in subscribers.items():
                msg = sub.Read()
                if msg is None or not msg.motor_state:
                    continue

                n_joints = len(msg.motor_state)
                if n_joints != SHARPA_DOF:
                    log.warning("[%s] expected %d joints, got %d", side, SHARPA_DOF, n_joints)

                # DDS stores degrees; convert to radians for uniform handling
                angles_rad = [math.radians(m.q) for m in msg.motor_state[:SHARPA_DOF]]

                log.info("[%s] iter=%d  n=%d", side, iter_idx, n_joints)
                log.debug("[%s]   angles: %s", side, fmt_angles(angles_rad, args.units))

                if plot is not None:
                    plot.push(iter_idx, side, angles_rad)

            if plot is not None:
                if not plot.is_open():
                    log.info("plot window closed — stopping.")
                    break
                plot.redraw()

            if deadline is not None and time.time() >= deadline:
                log.info("duration elapsed — stopping.")
                break

            time.sleep(max(0.0, period - (time.time() - t_loop)))

    except KeyboardInterrupt:
        log.info("interrupted by user.")
    finally:
        if plot is not None:
            plot.close()


if __name__ == "__main__":
    main()
