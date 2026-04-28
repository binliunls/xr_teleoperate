"""
Minimal Sharpa joint-state reader.

Connects to one or both Sharpa hands via the `sharpa` SDK and prints joint
angles to stdout with verbose logging. Read-only — does not change control
source, enable state, or commanded position.

Usage:
    python -m teleop.utils.sharpa_read_joints                    # both hands, 5 Hz, ~10s
    python -m teleop.utils.sharpa_read_joints --side right       # right only
    python -m teleop.utils.sharpa_read_joints --rate 30 --duration 0   # forever, 30 Hz
    python -m teleop.utils.sharpa_read_joints --units deg        # print in degrees
"""

import argparse
import logging
import math
import time

JOINT_NAMES = [
    "thumb_CMC_FE", "thumb_CMC_AA", "thumb_MCP_FE", "thumb_MCP_AA", "thumb_IP",
    "index_MCP_FE", "index_MCP_AA", "index_PIP", "index_DIP",
    "middle_MCP_FE", "middle_MCP_AA", "middle_PIP", "middle_DIP",
    "ring_MCP_FE", "ring_MCP_AA", "ring_PIP", "ring_DIP",
    "pinky_CMC", "pinky_MCP_FE", "pinky_MCP_AA", "pinky_PIP", "pinky_DIP",
]
SHARPA_DOF = 22

logging.basicConfig(
    level=logging.DEBUG,
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--side", choices=["left", "right", "both"], default="both")
    ap.add_argument("--rate", type=float, default=5.0, help="poll Hz")
    ap.add_argument("--duration", type=float, default=10.0,
                    help="seconds to run; 0 = forever")
    ap.add_argument("--units", choices=["rad", "deg"], default="rad")
    args = ap.parse_args()

    sides = ["left", "right"] if args.side == "both" else [args.side]
    log.info("config: sides=%s rate=%.1fHz duration=%s units=%s",
             sides, args.rate, "∞" if args.duration == 0 else f"{args.duration:.1f}s",
             args.units)

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
                log.debug("[%s]   angles : %s", side, fmt_angles(angles, args.units))
                if state.velocities:
                    log.debug("[%s]   vel    : %s",
                              side, [f"{v:+.3f}" for v in state.velocities[:SHARPA_DOF]])
                if state.torques:
                    log.debug("[%s]   torque : %s",
                              side, [f"{t:+.3f}" for t in state.torques[:SHARPA_DOF]])

            if deadline is not None and time.time() >= deadline:
                log.info("duration elapsed — stopping.")
                break
            time.sleep(max(0.0, period - (time.time() - t_loop)))
    except KeyboardInterrupt:
        log.info("interrupted by user.")
    finally:
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
