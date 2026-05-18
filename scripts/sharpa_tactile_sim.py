"""
Workstation-side stub publisher for the Sharpa tactile stream.

Emits synthetic ZMQ messages for all 10 fingertip channels at 30 Hz using the
wire format in teleop/utils/sharpa_tactile_wire.py — same as the real Thor
bridge will eventually produce. Lets the rest of the recording pipeline be
developed and tested without hardware.

Usage:
    python scripts/sharpa_tactile_sim.py                 # zeros + sinusoidal F6
    python scripts/sharpa_tactile_sim.py --bump          # moving Gaussian in DEFORM
    python scripts/sharpa_tactile_sim.py --port 7779 --rate 30
"""

import argparse
import os
import sys
import time

import numpy as np
import zmq

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from teleop.utils import sharpa_tactile_wire as wire


def _zero_deform():
    return np.zeros((wire.DEFORM_H, wire.DEFORM_W), dtype=np.uint8)


def _bump_deform(t, channel):
    """A moving 2-D Gaussian bump in uint8, distinct per channel."""
    h, w = wire.DEFORM_H, wire.DEFORM_W
    cx = w * (0.5 + 0.3 * np.sin(2 * np.pi * 0.3 * t + channel))
    cy = h * (0.5 + 0.3 * np.cos(2 * np.pi * 0.3 * t + channel))
    yy, xx = np.mgrid[0:h, 0:w]
    sigma = 25.0
    g = np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * sigma ** 2))
    return (200.0 * g).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0",
                    help="bind address (default 0.0.0.0)")
    ap.add_argument("--port", type=int, default=7779,
                    help="ZMQ PUB port (default 7779)")
    ap.add_argument("--rate", type=float, default=30.0,
                    help="emit rate Hz, all 10 channels per tick (default 30)")
    ap.add_argument("--bump", action="store_true",
                    help="emit a moving Gaussian bump instead of zeros")
    args = ap.parse_args()

    ctx = zmq.Context.instance()
    socket = ctx.socket(zmq.PUB)
    # Send-buffer must hold one tick's burst (10 fingers) plus slack.
    socket.setsockopt(zmq.SNDHWM, 100)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(f"tcp://{args.host}:{args.port}")
    print(f"[sharpa_tactile_sim] PUB tcp://{args.host}:{args.port} @ {args.rate} Hz "
          f"({'bump' if args.bump else 'zeros'})")

    period = 1.0 / args.rate
    frame_id = 0
    t0 = time.time()
    try:
        while True:
            tick_start = time.time()
            ts = tick_start - t0
            for channel in range(wire.NUM_CHANNELS):
                if args.bump:
                    deform = _bump_deform(ts, channel)
                else:
                    deform = _zero_deform()
                # Sinusoidal F6 distinct per channel so the recorder JSON varies.
                f6 = np.array(
                    [np.sin(2 * np.pi * 0.5 * ts + channel + axis * 0.3)
                     for axis in range(6)],
                    dtype=np.float32,
                )
                contact_point = np.empty(0, dtype=np.float32)
                payload = wire.pack(channel, frame_id, ts,
                                    deform, f6, contact_point)
                socket.send(payload)
            frame_id += 1
            sleep_for = period - (time.time() - tick_start)
            if sleep_for > 0:
                time.sleep(sleep_for)
    except KeyboardInterrupt:
        print("\n[sharpa_tactile_sim] stopping.")
    finally:
        socket.close()
        ctx.term()


if __name__ == "__main__":
    main()
