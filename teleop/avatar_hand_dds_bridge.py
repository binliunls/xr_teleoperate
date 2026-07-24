#!/usr/bin/env python3
"""Sharpa Avatar glove -> Sharpa Wave hand DDS bridge (single process).

Replaces the two Manus-era processes:
  * SharpaManusClient.out            (glove I/O)              and
  * retargeting_manus_demo_dds.py    (retarget + haptics)

with one process, using the Avatar SDK's built-in retargeting.

Forward path (glove -> hand):
  Avatar glove --(usb_serial/ethernet)--> avatar_sdk
    glove.fetch_data(ROBOT) -> 22 retargeted Wave joints (radians, robot_joint_names order)
    -> SharpaHandDDSClient.set_joint_position() -> DDS rt/sharpa/{side}/cmd
    -> (unchanged) Thor sharpa_dds_bridge -> real Wave hands.

Feedback path (hand tactile -> glove haptics):
  Thor sharpa_dds_bridge tactile ZMQ PUB (tcp://<thor>:7779, F6 per channel)
    -> parse F6 (Fx,Fy,Fz) -> glove.set_task({"key":"set_3d_force", ...})
    -> Avatar glove fingertip LRA feedback.

The Thor side and teleop_hand_and_arm.py are unchanged: teleop keeps reading
rt/sharpa/{side}/state for recording, and keeps recording tactile off :7779.

Run in the `tv` conda env (py3.10; has avatar_sdk 1.6.2 + unitree_sdk2py + zmq).
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import struct
import sys
import time
from pathlib import Path

# --- make both stacks importable -------------------------------------------------
SDK_PREFIX = Path(os.environ.get("AVATAR_SDK_PREFIX", "/opt/avatar-sdk"))
sys.path.insert(0, str(SDK_PREFIX / "lib/python"))
# teleop package root (this file lives in xr_teleoperate/teleop/)
sys.path.insert(0, str(Path(__file__).resolve().parent))

from avatar_sdk import (  # noqa: E402
    AvatarSDK,
    DeviceDataCategery,   # NOTE: vendor spelling (sic)
    DeviceSide,
    DeviceType,
    ErrorCode,
)
from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: E402
from robot_control.robot_hand_sharpa_dds import SharpaHandDDSClient, NUM_JOINTS  # noqa: E402

logger = logging.getLogger("avatar_bridge")

# ---- tactile channel layout (identical to the working Thor haptics bridge & wave.py) ----
# SDK channels 0-4 = RIGHT hand, 5-9 = LEFT; within a hand the order is
# pinky, ring, middle, index, thumb.  set_3d_force wants thumb..pinky (idx 0=thumb),
# so map each channel to its thumb-first finger index:
CHANNEL_TO_FINGER_IDX = {0: 4, 1: 3, 2: 2, 3: 1, 4: 0, 5: 4, 6: 3, 7: 2, 8: 1, 9: 0}
CHANNEL_TO_SIDE = {**{i: "right" for i in range(5)}, **{i: "left" for i in range(5, 10)}}

_TRANSPORT_LINK = {"wireless": "usb_serial", "wired": "ethernet_udp"}

# 28-byte tactile wire header: <channel:u32, frame_id:u32, ts:f64, deform_len:u32, f6_len:u32, cp_len:u32>
_HDR_FMT = "<IIdIII"
_HDR_SIZE = struct.calcsize(_HDR_FMT)


def _parse_f6(payload: bytes):
    """Return (channel, [fx,fy,fz]) from a tactile wire frame, skipping the deform blob."""
    if len(payload) < _HDR_SIZE:
        return None
    channel, _fid, _ts, dlen, flen, _clen = struct.unpack_from(_HDR_FMT, payload, 0)
    off = _HDR_SIZE + dlen
    if flen < 12 or len(payload) < off + 12:
        return None
    fx, fy, fz = struct.unpack_from("<3f", payload, off)
    return int(channel), (fx, fy, fz)


def _sides(side_arg: str):
    if side_arg == "both":
        return [("left", DeviceSide.LEFT), ("right", DeviceSide.RIGHT)]
    if side_arg == "left":
        return [("left", DeviceSide.LEFT)]
    return [("right", DeviceSide.RIGHT)]


def _load_config(cfg_path: Path, transport: str) -> str:
    if cfg_path.is_file():
        cfg_path = cfg_path.resolve()
        os.environ.setdefault("AVATAR_SDK_CONFIG_PATH", str(cfg_path))
        os.environ.setdefault("AVATAR_SDK_CONFIG_DIR", str(cfg_path.parent))
        os.environ.setdefault("AVATAR_SDK_HAND_FK_DATA_DIR", str(cfg_path.parent / "hand_fk"))
        cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
    else:
        cfg = {}
    cfg["transport_link"] = _TRANSPORT_LINK[transport]
    cfg.pop("transport_link_options", None)
    cfg["enable_wave"] = False  # we route joints via DDS, NOT the SDK's local Wave bridge
    return json.dumps(cfg, ensure_ascii=False)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--side", choices=("left", "right", "both"), default="both")
    p.add_argument("--transport", choices=("wireless", "wired"), default="wireless")
    p.add_argument("--config", default=str(SDK_PREFIX / "share/sdk_config.json"))
    p.add_argument("--rate-hz", type=float, default=120.0, help="Per-hand ROBOT fetch/publish rate")
    p.add_argument("--dds-domain", type=int, default=0, help="Must match Thor sharpa_dds_bridge")
    p.add_argument("--dds-interface", default=None, help="DDS network interface (default: auto)")
    p.add_argument("--tactile-host", default="192.168.123.163")
    p.add_argument("--tactile-port", type=int, default=7779)
    p.add_argument("--no-haptics", action="store_true", help="Disable tactile->glove feedback")
    p.add_argument("--force-scale", type=float, default=1.0, help="Scale F6 before set_3d_force")
    p.add_argument("--dry-run", action="store_true",
                   help="Do NOT publish hand cmd; just print ROBOT joints (safe bring-up)")
    p.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = p.parse_args()

    logging.basicConfig(level=getattr(logging, args.log_level),
                        format="[%(levelname)s] %(name)s: %(message)s")

    # ---- DDS: must be initialized once before any SharpaHandDDSClient ----
    if not args.dry_run:
        if args.dds_interface:
            ChannelFactoryInitialize(args.dds_domain, networkInterface=args.dds_interface)
        else:
            ChannelFactoryInitialize(args.dds_domain)
        logger.info("DDS domain=%d interface=%s", args.dds_domain, args.dds_interface or "auto")

    # ---- Avatar SDK ----
    config_text = _load_config(Path(args.config), args.transport)
    logger.info("transport=%s config=%s", args.transport, args.config)
    sdk = AvatarSDK.get_instance()
    if sdk.initialize(config_text) != ErrorCode.SUCCESS:
        logger.error("AvatarSDK.initialize failed")
        return 1

    running = {"v": True}

    def _stop(_sig=None, _frm=None):
        running["v"] = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    gloves = {}          # side_label -> AvatarDevice
    hands = {}           # side_label -> SharpaHandDDSClient
    tac_sub = None
    last_force = {"left": None, "right": None}

    try:
        # discover gloves
        logger.info("Waiting for glove discovery...")
        time.sleep(2.0)
        for info in sdk.list_device_info():
            logger.info("device sn=%s side=%s online=%s", info.sn, info.device_side, info.online)
        for label, ds in _sides(args.side):
            g = sdk.get_device(DeviceType.GLOVE, ds)
            if g is None:
                logger.error("No %s glove found", label.upper())
                if args.side != "both":
                    return 1
                continue
            g.init('{"enable_retarget": true}')
            g.start()
            gloves[label] = g
            logger.info("Connected %s glove", label.upper())
        if not gloves:
            logger.error("No gloves connected")
            return 1

        # DDS hand clients (SharpaHandDDSClient.__init__ builds pub/sub; not go_neutral)
        if not args.dry_run:
            for label in gloves:
                hands[label] = SharpaHandDDSClient(label)
                hands[label].go_neutral()
            logger.info("DDS hand clients ready: %s", list(hands))

        # tactile SUB
        if not args.no_haptics:
            import zmq
            ctx = zmq.Context.instance()
            tac_sub = ctx.socket(zmq.SUB)
            tac_sub.setsockopt(zmq.SUBSCRIBE, b"")
            tac_sub.setsockopt(zmq.RCVHWM, 20)
            tac_sub.connect(f"tcp://{args.tactile_host}:{args.tactile_port}")
            logger.info("Tactile SUB tcp://%s:%d", args.tactile_host, args.tactile_port)

        period = 1.0 / max(1.0, args.rate_hz)
        logger.info("Bridging at %.0f Hz%s (Ctrl+C to stop)",
                    args.rate_hz, " [DRY-RUN: no cmd published]" if args.dry_run else "")

        while running["v"]:
            t0 = time.monotonic()

            # ---- forward: ROBOT joints -> DDS cmd ----
            for label, g in gloves.items():
                ec, frame = g.fetch_data(DeviceDataCategery.ROBOT)
                if ec != ErrorCode.SUCCESS or frame is None:
                    continue
                q = [float(v) for v in list(frame.robot.joint.position)[:NUM_JOINTS]]
                if len(q) != NUM_JOINTS:
                    continue
                if args.dry_run:
                    logger.info("[%s] ROBOT(rad) %s", label,
                                " ".join(f"{v:+.2f}" for v in q))
                else:
                    hands[label].set_joint_position(q)

            # ---- feedback: tactile F6 -> set_3d_force ----
            if tac_sub is not None:
                fx = {"left": [0.0] * 5, "right": [0.0] * 5}
                fy = {"left": [0.0] * 5, "right": [0.0] * 5}
                fz = {"left": [0.0] * 5, "right": [0.0] * 5}
                touched = set()
                # drain everything currently queued, keep latest per channel
                while True:
                    try:
                        payload = tac_sub.recv(flags=1)  # zmq.NOBLOCK
                    except Exception:
                        break
                    parsed = _parse_f6(payload)
                    if parsed is None:
                        continue
                    ch, (vx, vy, vz) = parsed
                    if ch not in CHANNEL_TO_FINGER_IDX:
                        continue
                    side = CHANNEL_TO_SIDE[ch]
                    if side not in gloves:
                        continue
                    idx = CHANNEL_TO_FINGER_IDX[ch]
                    s = args.force_scale
                    fx[side][idx], fy[side][idx], fz[side][idx] = vx * s, vy * s, vz * s
                    touched.add(side)
                for side in touched:
                    key = (tuple(fx[side]), tuple(fy[side]), tuple(fz[side]))
                    if last_force[side] == key:
                        continue
                    payload = {
                        "key": "set_3d_force",
                        "device_side": side.upper(),
                        "force_x": fx[side],
                        "force_y": fy[side],
                        "force_z": fz[side],
                    }
                    ec, _ = gloves[side].set_task(json.dumps(payload, ensure_ascii=False))
                    if ec == ErrorCode.SUCCESS:
                        last_force[side] = key

            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)

        logger.info("Stopping...")
        return 0
    finally:
        # zero the glove haptics
        for label, g in gloves.items():
            try:
                for _ in range(5):
                    g.set_task(json.dumps({
                        "key": "set_3d_force", "device_side": label.upper(),
                        "force_x": [0.0] * 5, "force_y": [0.0] * 5, "force_z": [0.0] * 5,
                    }))
                    time.sleep(0.05)
            except Exception:
                pass
            try:
                g.stop()
            except Exception:
                pass
        for label, h in hands.items():
            try:
                h.go_neutral()
            except Exception:
                pass
        if tac_sub is not None:
            tac_sub.close(0)
        sdk.destroy()


if __name__ == "__main__":
    raise SystemExit(main())
