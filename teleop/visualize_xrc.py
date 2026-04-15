#!/usr/bin/env python3
"""
visualize_xrc.py

Standalone real-time visualization of data streamed from the Pico headset
via the XRoboToolkit SDK.

Runs WITHOUT connecting to a robot — purely a diagnostic / inspection tool.

Two display modes (selectable with --mode):
    terminal   : compact live text dashboard (no extra dependencies)
    3d         : interactive 3-D plot of tracker + headset poses using matplotlib

Usage:
    # Text dashboard (always works)
    python visualize_xrc.py

    # 3-D plot
    python visualize_xrc.py --mode 3d

    # Skip ADB (Pico on Wi-Fi)
    python visualize_xrc.py --no-adb

    # Force serial assignment
    python visualize_xrc.py --left-serial ABC123 --right-serial XYZ789
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
for _p in [_THIS_DIR, _REPO_ROOT]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from xrc_wrapper import XrcWrapper, TrackerPose

# ---------------------------------------------------------------------------
# Terminal dashboard
# ---------------------------------------------------------------------------

def _euler_str(rot33: np.ndarray) -> str:
    """Return 'roll pitch yaw' in degrees from a 3×3 rotation matrix."""
    try:
        rpy = R.from_matrix(rot33).as_euler("xyz", degrees=True)
        return f"R={rpy[0]:6.1f}° P={rpy[1]:6.1f}° Y={rpy[2]:6.1f}°"
    except Exception:
        return "R=?      P=?      Y=?     "


def _vel_str(vel: np.ndarray) -> str:
    mag = float(np.linalg.norm(vel))
    return f"|v|={mag:.3f} m/s  [{vel[0]:6.3f} {vel[1]:6.3f} {vel[2]:6.3f}]"


def _print_dashboard(
    xrc: XrcWrapper,
    arm_poses: dict,
    headset_T: np.ndarray | None,
    rate: float,
    frame: int,
) -> None:
    """Render a full-screen terminal dashboard."""

    lines: list[str] = [
        "\033[H\033[J",    # ANSI: move to top-left + clear screen
        "┌──────────────────────────────────────────────────────────┐",
        "│         XRoboToolkit SDK  —  Live Data Visualizer        │",
        "└──────────────────────────────────────────────────────────┘",
        f"  Update rate : {rate:5.1f} Hz    Frame: {frame}",
        f"  Trackers detected : {xrc.num_trackers}",
        f"  Serial (L) : {xrc.left_serial  or 'not assigned'}",
        f"  Serial (R) : {xrc.right_serial or 'not assigned'}",
        "",
    ]

    # ── Wrist trackers ──────────────────────────────────────────────
    lines.append("  ┌─ WRIST TRACKERS (robot frame: Z-up, X-forward) ─────────┐")
    for side in ("left", "right"):
        pose = arm_poses[side]
        if pose["valid"]:
            p = pose["position"]
            lines.append(f"  │  {side.upper():5s}  VALID")
            lines.append(f"  │    pos  : [{p[0]:7.3f}  {p[1]:7.3f}  {p[2]:7.3f}] m")
            lines.append(f"  │    rot  : {_euler_str(pose['rotation'])}")
            lines.append(f"  │    vel  : {_vel_str(pose['velocity'])}")
        else:
            lines.append(f"  │  {side.upper():5s}  ── NOT FOUND ──")
    lines.append("  └──────────────────────────────────────────────────────────┘")

    # ── All trackers (raw) ──────────────────────────────────────────
    all_t = xrc.get_all_trackers()
    lines.append("")
    lines.append(f"  ┌─ ALL TRACKERS ({len(all_t)} total) ─────────────────────────────┐")
    if all_t:
        for t in all_t:
            role = ""
            if t.serial == xrc.left_serial:
                role = " [LEFT]"
            elif t.serial == xrc.right_serial:
                role = " [RIGHT]"
            short = t.serial[:36] if len(t.serial) > 36 else t.serial
            p = t.position
            lines.append(f"  │  {short}{role}")
            lines.append(f"  │    pos: [{p[0]:7.3f}  {p[1]:7.3f}  {p[2]:7.3f}]")
    else:
        lines.append("  │  (none detected — check ADB / Pico connection)")
    lines.append("  └──────────────────────────────────────────────────────────┘")

    # ── Headset ─────────────────────────────────────────────────────
    lines.append("")
    lines.append("  ┌─ HEADSET ────────────────────────────────────────────────┐")
    if headset_T is not None:
        hp = headset_T[:3, 3]
        lines.append(f"  │  pos : [{hp[0]:7.3f}  {hp[1]:7.3f}  {hp[2]:7.3f}] m")
        lines.append(f"  │  rot : {_euler_str(headset_T[:3, :3])}")
    else:
        lines.append("  │  (not available)")
    lines.append("  └──────────────────────────────────────────────────────────┘")

    lines.append("")
    lines.append("  Press Ctrl+C to exit")

    print("\n".join(lines), flush=True)


def run_terminal(args: argparse.Namespace) -> None:
    xrc = XrcWrapper(
        use_adb=not args.no_adb,
        left_serial=args.left_serial,
        right_serial=args.right_serial,
    )
    xrc.start()
    print("XRoboToolkit SDK started — waiting for data…")

    frame   = 0
    t_last  = time.perf_counter()
    rate    = 0.0
    period  = 1.0 / args.rate

    try:
        while True:
            t0 = time.perf_counter()

            xrc.update()
            arm_poses  = xrc.get_arm_poses()
            headset_T  = xrc.get_headset_pose()

            # rolling rate estimate
            dt   = t0 - t_last
            rate = 0.9 * rate + 0.1 * (1.0 / dt) if dt > 0 else rate
            t_last = t0
            frame += 1

            _print_dashboard(xrc, arm_poses, headset_T, rate, frame)

            sleep = period - (time.perf_counter() - t0)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n\nExiting…")
    finally:
        xrc.stop()


# ---------------------------------------------------------------------------
# 3-D matplotlib visualization
# ---------------------------------------------------------------------------

def _draw_frame(ax, T: np.ndarray, length: float = 0.05, label: str = "") -> None:
    """Draw a coordinate frame (3 coloured axes) at pose T."""
    origin = T[:3, 3]
    for i, (col, lbl) in enumerate(zip(("r", "g", "b"), ("x", "y", "z"))):
        direction = T[:3, i] * length
        ax.quiver(
            *origin, *direction,
            color=col, arrow_length_ratio=0.3, linewidth=2,
        )
    if label:
        ax.text(*origin, label, fontsize=7)


def run_3d(args: argparse.Namespace) -> None:
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D   # noqa: F401
    except ImportError:
        print("matplotlib is required for 3-D mode: pip install matplotlib")
        sys.exit(1)

    xrc = XrcWrapper(
        use_adb=not args.no_adb,
        left_serial=args.left_serial,
        right_serial=args.right_serial,
    )
    xrc.start()

    plt.ion()
    fig = plt.figure(figsize=(10, 8))
    ax  = fig.add_subplot(111, projection="3d")

    period  = 1.0 / args.rate
    history: dict[str, list[np.ndarray]] = {"left": [], "right": [], "head": []}
    MAX_TRAIL = 80   # number of past positions to show as trail

    try:
        while plt.fignum_exists(fig.number):
            t0 = time.perf_counter()

            xrc.update()
            arm_poses = xrc.get_arm_poses()
            headset_T = xrc.get_headset_pose()

            ax.cla()
            ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
            ax.set_title("XRoboToolkit SDK — Live Tracker Visualization")
            span = 0.5
            ax.set_xlim(-span, span); ax.set_ylim(-span, span); ax.set_zlim(0, span * 2)

            # Draw ground plane grid (light)
            gx = np.linspace(-span, span, 5)
            for v in gx:
                ax.plot([v, v], [-span, span], [0, 0], color="lightgray", linewidth=0.5)
                ax.plot([-span, span], [v, v], [0, 0], color="lightgray", linewidth=0.5)

            # Wrist trackers
            SIDE_COLOR = {"left": "#1f77b4", "right": "#d62728"}
            for side in ("left", "right"):
                pose = arm_poses[side]
                col  = SIDE_COLOR[side]
                if pose["valid"]:
                    p   = pose["position"]
                    rot = pose["rotation"]
                    T   = np.eye(4); T[:3, :3] = rot; T[:3, 3] = p

                    history[side].append(p.copy())
                    if len(history[side]) > MAX_TRAIL:
                        history[side].pop(0)

                    # Trail
                    if len(history[side]) > 1:
                        trail = np.array(history[side])
                        alphas = np.linspace(0.05, 0.6, len(trail))
                        for i in range(1, len(trail)):
                            ax.plot(
                                trail[i-1:i+1, 0], trail[i-1:i+1, 1], trail[i-1:i+1, 2],
                                color=col, alpha=float(alphas[i]), linewidth=1.5,
                            )

                    # Pose frame
                    _draw_frame(ax, T, length=0.06, label=f"{side}\n[{p[0]:.2f},{p[1]:.2f},{p[2]:.2f}]")

                    # Sphere marker
                    ax.scatter(*p, color=col, s=80, zorder=5)

            # Headset
            if headset_T is not None:
                hp = headset_T[:3, 3]
                history["head"].append(hp.copy())
                if len(history["head"]) > MAX_TRAIL:
                    history["head"].pop(0)
                _draw_frame(ax, headset_T, length=0.04, label="head")
                ax.scatter(*hp, color="purple", s=60, marker="^", zorder=5)

            # Legend / info
            lv = arm_poses["left"]["valid"]
            rv = arm_poses["right"]["valid"]
            info = (
                f"L={'OK' if lv else '??'}  R={'OK' if rv else '??'}  "
                f"trackers={xrc.num_trackers}"
            )
            ax.text2D(0.02, 0.97, info, transform=ax.transAxes, fontsize=9,
                      verticalalignment="top", color="black")

            plt.draw()
            plt.pause(0.001)

            sleep = period - (time.perf_counter() - t0)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        print("\n\nExiting…")
    finally:
        plt.close("all")
        xrc.stop()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> None:
    p = argparse.ArgumentParser(
        description="Real-time visualization of Pico Motion Tracker data via XRoboToolkit SDK",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--mode", choices=["terminal", "3d"], default="terminal",
        help="Display mode: 'terminal' (text dashboard) or '3d' (matplotlib 3-D plot)",
    )
    p.add_argument("--rate",         type=float, default=20.0,  help="Visualization update rate (Hz)")
    p.add_argument("--no-adb",       action="store_true",        help="Skip ADB tunnel (use Wi-Fi)")
    p.add_argument("--left-serial",  type=str,   default=None,  help="Force serial for left tracker")
    p.add_argument("--right-serial", type=str,   default=None,  help="Force serial for right tracker")
    args = p.parse_args()

    if args.mode == "3d":
        run_3d(args)
    else:
        run_terminal(args)


if __name__ == "__main__":
    main()
