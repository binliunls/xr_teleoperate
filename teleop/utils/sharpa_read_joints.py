"""
Minimal Sharpa joint-state reader with live matplotlib curves.

Connects to one or both Sharpa hands via the `sharpa` SDK, prints concise
status to stdout, and plots every joint angle as a real-time curve using
matplotlib. Read-only — does not change control source, enable state, or
commanded position.

Verbose mode (-v / --verbose) additionally dumps full per-joint angle,
velocity, and torque vectors each iteration.

Usage:
    python -m teleop.utils.sharpa_read_joints                    # both hands, 5 Hz, 10s + viz
    python -m teleop.utils.sharpa_read_joints --side right       # right only
    python -m teleop.utils.sharpa_read_joints --rate 30 --duration 0   # forever, 30 Hz
    python -m teleop.utils.sharpa_read_joints --units deg -v     # degrees, verbose
    python -m teleop.utils.sharpa_read_joints --no-viz           # disable plot
    python -m teleop.utils.sharpa_read_joints --window 600       # show last 600 samples
"""

import argparse
import logging
import math
import time
from collections import deque

JOINT_NAMES = [
    "thumb_CMC_FE", "thumb_CMC_AA", "thumb_MCP_FE", "thumb_MCP_AA", "thumb_IP",
    "index_MCP_FE", "index_MCP_AA", "index_PIP", "index_DIP",
    "middle_MCP_FE", "middle_MCP_AA", "middle_PIP", "middle_DIP",
    "ring_MCP_FE", "ring_MCP_AA", "ring_PIP", "ring_DIP",
    "pinky_CMC", "pinky_MCP_FE", "pinky_MCP_AA", "pinky_PIP", "pinky_DIP",
]
FINGER_GROUPS = ["thumb", "index", "middle", "ring", "pinky"]
SHARPA_DOF = 22

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d | %(levelname)-7s | %(name)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("sharpa_read_joints")


def discover(manager, timeout_s=20.0):
    """Wait for at least one device to show up via heartbeat."""
    log.info("Discovering Sharpa devices (heartbeat, up to %.1fs)...", timeout_s)
    t0 = time.time()
    last_count = -1
    while time.time() - t0 < timeout_s:
        sns = manager.get_all_device_sn()
        if len(sns) != last_count:
            log.info("  discovered %d device(s): %s", len(sns), sns)
            last_count = len(sns)
        if sns:
            return sns
        time.sleep(0.5)
    raise RuntimeError(f"No Sharpa devices discovered within {timeout_s:.1f}s.")


def open_hand(manager, side):
    from sharpa import HandSide

    hand_side = HandSide.LEFT if side == "left" else HandSide.RIGHT
    log.info("[%s] connecting via HandSide.%s ...", side, hand_side.name)
    hand = manager.connect(hand_side)

    info = hand.get_device_info()
    log.info("[%s]   sn=%s  fw=%s  ip=%s  side=%s",
             side, info.sn, info.firmware_version, info.ip, info.hand_side)
    log.info("[%s]   temp=%.1f°C  battery=%d%%  err_code=%d",
             side, info.status.temperature, info.status.battery, info.status.error_code)

    log.info("[%s] starting data streams...", side)
    hand.start()
    t0 = time.time()
    while not hand.is_hand_ready():
        if time.time() - t0 > 10.0:
            raise RuntimeError(f"[{side}] hand not ready after 10s.")
        time.sleep(0.05)
    log.info("[%s] hand_ready=%s tactile_ready=%s",
             side, hand.is_hand_ready(), hand.is_tactile_ready())
    return hand


def fmt_angles(angles, units):
    if units == "deg":
        vals = [math.degrees(a) for a in angles]
        suffix = "°"
    else:
        vals = list(angles)
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

        # Build per-(side, joint_name) deque of values + a shared sample index deque.
        self.x = deque(maxlen=window)
        self.data = {(s, n): deque(maxlen=window) for s in sides for n in JOINT_NAMES}

        plt.ion()
        n_rows = len(sides)
        n_cols = len(FINGER_GROUPS)
        self.fig, axes = plt.subplots(
            n_rows, n_cols, figsize=(3.2 * n_cols, 2.5 * n_rows),
            sharex=True, squeeze=False,
        )
        self.fig.suptitle("Sharpa joint angles (live)")
        self.fig.canvas.manager.set_window_title("sharpa_read_joints")

        # lines[(side, joint_name)] -> Line2D
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
                # one line per joint that belongs to this finger
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

    def push(self, idx, side, angles):
        if not self.x or self.x[-1] != idx:
            self.x.append(idx)
        for name, a in zip(JOINT_NAMES, angles):
            v = math.degrees(a) if self.units == "deg" else a
            self.data[(side, name)].append(float(v))

    def redraw(self):
        xs = list(self.x)
        for (side, name), ln in self.lines.items():
            ys = list(self.data[(side, name)])
            # x and y must be the same length; trim if a side missed an iteration
            n = min(len(xs), len(ys))
            ln.set_data(xs[-n:], ys[-n:])
        # Rescale each axis
        for ax in self.axes.values():
            ax.relim()
            ax.autoscale_view()
        try:
            self.fig.canvas.draw_idle()
            self.plt.pause(0.001)
        except Exception:
            # window closed by user; let main loop continue
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
    ap.add_argument("--rate", type=float, default=5.0, help="poll Hz")
    ap.add_argument("--duration", type=float, default=10.0,
                    help="seconds to run; 0 = forever")
    ap.add_argument("--units", choices=["rad", "deg"], default="rad")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="Print full per-joint angle/velocity/torque vectors each iteration.")
    ap.add_argument("--no-viz", action="store_true",
                    help="Disable matplotlib visualization.")
    ap.add_argument("--window", type=int, default=300,
                    help="Rolling sample-window length for live plot.")
    args = ap.parse_args()

    if args.verbose:
        log.setLevel(logging.DEBUG)

    sides = ["left", "right"] if args.side == "both" else [args.side]
    log.info("config: sides=%s rate=%.1fHz duration=%s units=%s viz=%s verbose=%s",
             sides, args.rate, "∞" if args.duration == 0 else f"{args.duration:.1f}s",
             args.units, not args.no_viz, args.verbose)

    plot = None
    if not args.no_viz:
        try:
            plot = LivePlot(sides, args.units, window=args.window)
            log.info("matplotlib live plot ready: %d×%d grid (rows=sides, cols=fingers).",
                     len(sides), len(FINGER_GROUPS))
        except Exception as e:
            log.warning("matplotlib setup failed (%s) — continuing without viz.", e)
            plot = None

    from sharpa import SharpaWaveManager
    manager = SharpaWaveManager.get_instance()
    discover(manager)

    hands = {s: open_hand(manager, s) for s in sides}

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

            for side, h in hands.items():
                t0 = time.time()
                state = h.get_states()
                dt_ms = (time.time() - t0) * 1000.0

                angles = list(state.angles)
                if len(angles) != SHARPA_DOF:
                    log.warning("[%s] expected %d angles, got %d",
                                side, SHARPA_DOF, len(angles))

                log.info("[%s] iter=%d seq=%d ts=%s n=%d read=%.2fms",
                         side, iter_idx, state.sequence,
                         getattr(state, "timestamp", "?"),
                         len(angles), dt_ms)

                # Verbose mode: full per-joint dump
                log.debug("[%s]   angles : %s", side, fmt_angles(angles, args.units))
                if state.velocities:
                    log.debug("[%s]   vel    : %s",
                              side, [f"{v:+.3f}" for v in state.velocities[:SHARPA_DOF]])
                if state.torques:
                    log.debug("[%s]   torque : %s",
                              side, [f"{t:+.3f}" for t in state.torques[:SHARPA_DOF]])

                if plot is not None:
                    plot.push(iter_idx, side, angles[:SHARPA_DOF])

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
        for side, h in hands.items():
            try:
                h.stop()
                log.info("[%s] stopped.", side)
            except Exception as e:
                log.warning("[%s] stop failed: %s", side, e)
        try:
            manager.disconnect_all()
            log.info("disconnected all.")
        except Exception as e:
            log.warning("disconnect_all failed: %s", e)


if __name__ == "__main__":
    main()
