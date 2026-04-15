#!/usr/bin/env python3
"""
teleop_hand_and_arm_xrc.py

Wrist teleoperation using Pico Motion Trackers via the XRoboToolkit SDK.

This is a drop-in alternative to:
  - teleop_hand_and_arm.py   (uses televuer/WebXR browser stack)
  - alvr_tracker_teleop.py   (uses ALVR + OpenVR/SteamVR)

Key differences from those scripts:
  - No browser, no ALVR, no SteamVR required
  - Pico connects via USB → ADB reverse tunnel → XRoboToolkit SDK
  - Wrist pose comes from Pico Motion Trackers (strapped to wrists, hands free)
  - Hand/finger tracking is NOT implemented here (future work)

Usage:
    python teleop_hand_and_arm_xrc.py --arm G1_29
    python teleop_hand_and_arm_xrc.py --arm H2 --sim
    python teleop_hand_and_arm_xrc.py --arm G1_29 --left-serial ABC123 --right-serial XYZ789
    python teleop_hand_and_arm_xrc.py --arm G1_29 --viz   # enable terminal visualization

Controls (keyboard):
    [c]  Calibrate  — stand in robot home pose and press to zero reference
    [r]  Start      — begin robot tracking after calibration
    [q]  Quit

Calibration:
    1. Position your arms to match the robot's home/init pose
    2. Hold still and press [c]
    3. Press [r] to start — robot follows your wrist motion
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import numpy as np
import pinocchio as pin
from sshkeyboard import listen_keyboard, stop_listening

# ---------------------------------------------------------------------------
# Path setup — allow running from teleop/ or repo root
# ---------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
for _p in [_THIS_DIR, _REPO_ROOT]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from xrc_wrapper import XrcWrapper

from robot_control.robot_arm import (
    G1_29_ArmController,
    G1_23_ArmController,
    H1_2_ArmController,
    H1_ArmController,
    H2_ArmController,
)
from robot_control.robot_arm_ik import (
    G1_29_ArmIK,
    G1_23_ArmIK,
    H1_2_ArmIK,
    H1_ArmIK,
    H2_ArmIK,
)

try:
    import logging_mp
    logging_mp.basicConfig(level=logging_mp.INFO)
    logger = logging_mp.getLogger(__name__)
except ImportError:
    import logging
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Global keyboard state
# ---------------------------------------------------------------------------
_START     = False
_STOP      = False
_CALIBRATE = False


def _on_press(key: str) -> None:
    global _START, _STOP, _CALIBRATE
    if key == "c":
        _CALIBRATE = True
        logger.info("Calibration requested…")
    elif key == "r":
        _START = True
        logger.info("Start tracking requested…")
    elif key == "q":
        _START = False
        _STOP  = True
        logger.info("Quit requested…")


# ---------------------------------------------------------------------------
# Main teleoperation class
# ---------------------------------------------------------------------------
class XrcTeleop:
    """Wrist teleoperation via XRoboToolkit SDK + Pico Motion Trackers."""

    ARM_MAP = {
        "G1_29": (G1_29_ArmController, G1_29_ArmIK),
        "G1_23": (G1_23_ArmController, G1_23_ArmIK),
        "H1_2":  (H1_2_ArmController,  H1_2_ArmIK),
        "H1":    (H1_ArmController,     H1_ArmIK),
        "H2":    (H2_ArmController,     H2_ArmIK),
    }

    def __init__(self, args: argparse.Namespace) -> None:
        self.args      = args
        self.frequency = args.frequency
        self.viz       = args.viz

        # ---- DDS / robot init ----
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        if args.sim:
            ChannelFactoryInitialize(1)
            logger.info("DDS initialized (sim domain 1)")
        elif args.network_interface:
            ChannelFactoryInitialize(0, args.network_interface)
            logger.info(f"DDS initialized (interface: {args.network_interface})")
        else:
            ChannelFactoryInitialize(0)
            logger.info("DDS initialized (real robot domain 0)")

        ctrl_cls, ik_cls = self.ARM_MAP[args.arm]
        self.arm_ik   = ik_cls()
        self.arm_ctrl = ctrl_cls(
            kp_low=args.kp,
            kp_wrist=args.kp_wrist,
            kd_low=args.kd,
            kd_wrist=args.kd_wrist,
        )
        logger.info(f"Robot {args.arm} initialized")

        # ---- XRoboToolkit SDK wrapper ----
        self.xrc = XrcWrapper(
            use_adb=not args.no_adb,
            left_serial=args.left_serial,
            right_serial=args.right_serial,
            position_scale=args.position_scale,
        )

        # ---- Calibration state ----
        self._calibrated       = False
        self._ref_left_pos:  np.ndarray | None = None
        self._ref_right_pos: np.ndarray | None = None
        self._ref_left_rot:  np.ndarray | None = None
        self._ref_right_rot: np.ndarray | None = None
        self._init_left_wrist:  np.ndarray | None = None  # (4,4) from FK
        self._init_right_wrist: np.ndarray | None = None

        # ---- Stats ----
        self._loop_times: list[float] = []
        self._ik_times:   list[float] = []

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def _calibrate(self, arm_poses: dict) -> bool:
        global _CALIBRATE
        _CALIBRATE = False

        if not arm_poses["left"]["valid"] or not arm_poses["right"]["valid"]:
            missing = [s for s in ("left", "right") if not arm_poses[s]["valid"]]
            logger.warning(f"Cannot calibrate — tracker(s) missing: {missing}")
            return False

        # Store reference tracker positions in robot frame
        self._ref_left_pos  = arm_poses["left"]["position"].copy()
        self._ref_right_pos = arm_poses["right"]["position"].copy()
        self._ref_left_rot  = arm_poses["left"]["rotation"].copy()
        self._ref_right_rot = arm_poses["right"]["rotation"].copy()

        # Compute FK to get actual EE poses as the starting target
        q = self.arm_ctrl.get_current_dual_arm_q()
        pin.forwardKinematics(
            self.arm_ik.reduced_robot.model,
            self.arm_ik.reduced_robot.data,
            q,
        )
        pin.updateFramePlacements(
            self.arm_ik.reduced_robot.model,
            self.arm_ik.reduced_robot.data,
        )
        self._init_left_wrist  = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.L_hand_id].homogeneous.copy()
        self._init_right_wrist = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.R_hand_id].homogeneous.copy()

        self._calibrated = True
        logger.info("=" * 55)
        logger.info("CALIBRATION COMPLETE")
        logger.info("  Reference pose captured. Press [r] to start.")
        logger.info(f"  L EE: {self._init_left_wrist[:3, 3].round(4)}")
        logger.info(f"  R EE: {self._init_right_wrist[:3, 3].round(4)}")
        logger.info("=" * 55)
        return True

    # ------------------------------------------------------------------
    # Wrist pose computation (mirrors ALVRTeleop.compute_wrist_poses)
    # ------------------------------------------------------------------

    def _compute_wrist_poses(
        self, arm_poses: dict
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        if not self._calibrated:
            return None, None
        if not arm_poses["left"]["valid"] or not arm_poses["right"]["valid"]:
            return None, None

        scale = self.args.position_scale

        for side, init_pose, ref_pos, ref_rot in (
            ("left",  self._init_left_wrist,  self._ref_left_pos,  self._ref_left_rot),
            ("right", self._init_right_wrist, self._ref_right_pos, self._ref_right_rot),
        ):
            _ = side  # used below in loop body

        results = {}
        for side, init_pose, ref_pos, ref_rot in (
            ("left",  self._init_left_wrist,  self._ref_left_pos,  self._ref_left_rot),
            ("right", self._init_right_wrist, self._ref_right_pos, self._ref_right_rot),
        ):
            cur_pos = arm_poses[side]["position"]
            cur_rot = arm_poses[side]["rotation"]

            # Position delta in robot frame (frame conversion already done in XrcWrapper)
            delta = (cur_pos - ref_pos) * scale

            # Relative rotation from reference
            rel_rot = cur_rot @ ref_rot.T

            target_pos = init_pose[:3, 3] + delta
            target_rot = rel_rot @ init_pose[:3, :3]

            T = np.eye(4)
            T[:3, :3] = target_rot
            T[:3,  3] = target_pos
            results[side] = T

        return results["left"], results["right"]

    # ------------------------------------------------------------------
    # Visualization (terminal)
    # ------------------------------------------------------------------

    def _print_status(
        self,
        arm_poses:   dict,
        left_wrist:  np.ndarray | None,
        right_wrist: np.ndarray | None,
        loop_hz:     float,
        ik_ms:       float,
    ) -> None:
        """Print a compact status block to the terminal (overwrites in place)."""
        lv = arm_poses["left"]["valid"]
        rv = arm_poses["right"]["valid"]
        lp = arm_poses["left"]["position"]
        rp = arm_poses["right"]["position"]

        lines = [
            "\033[H\033[J",   # clear screen
            "╔══════════════════════════════════════════════════════╗",
            "║          XRoboToolkit Wrist Tracker Teleop           ║",
            "╚══════════════════════════════════════════════════════╝",
            "",
            f"  Loop rate : {loop_hz:5.1f} Hz      IK solve: {ik_ms:5.1f} ms",
            f"  Trackers  : L={'OK  ' if lv else 'MISS'}  R={'OK  ' if rv else 'MISS'}",
            f"  Serials   : L={self.xrc.left_serial or '?'}",
            f"              R={self.xrc.right_serial or '?'}",
            "",
            "  ── Raw tracker poses (robot frame) ──",
            f"  L pos : [{lp[0]:7.3f}  {lp[1]:7.3f}  {lp[2]:7.3f}]" if lv else "  L pos : ---",
            f"  R pos : [{rp[0]:7.3f}  {rp[1]:7.3f}  {rp[2]:7.3f}]" if rv else "  R pos : ---",
        ]

        if left_wrist is not None:
            ltp = left_wrist[:3, 3]
            rtp = right_wrist[:3, 3]   # type: ignore[index]
            lines += [
                "",
                "  ── IK target wrist poses ──",
                f"  L wrist: [{ltp[0]:7.3f}  {ltp[1]:7.3f}  {ltp[2]:7.3f}]",
                f"  R wrist: [{rtp[0]:7.3f}  {rtp[1]:7.3f}  {rtp[2]:7.3f}]",
            ]

        state = "RUNNING" if _START else ("CALIBRATED — press [r]" if self._calibrated else "WAITING — press [c]")
        lines += [
            "",
            f"  State : {state}",
            "  Keys  : [c]=calibrate  [r]=start  [q]=quit",
        ]

        print("\n".join(lines), flush=True)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        global _START, _STOP, _CALIBRATE

        # Start XRoboToolkit SDK
        self.xrc.start()

        # Keyboard listener thread
        kb_thread = threading.Thread(
            target=listen_keyboard,
            kwargs={"on_press": _on_press, "until": None, "sequential": False},
            daemon=True,
        )
        kb_thread.start()

        logger.info("─" * 55)
        logger.info("XRoboToolkit Wrist Tracker Teleoperation")
        logger.info("─" * 55)
        logger.info("  [c]  Calibrate (match robot home pose first!)")
        logger.info("  [r]  Start tracking")
        logger.info("  [q]  Quit")
        logger.info("─" * 55)

        period = 1.0 / self.frequency

        try:
            while not _STOP:
                t0 = time.perf_counter()

                # 1. Poll SDK
                self.xrc.update()
                arm_poses = self.xrc.get_arm_poses()

                left_wrist  = None
                right_wrist = None
                ik_ms       = 0.0

                # 2. Calibrate on request
                if _CALIBRATE:
                    self._calibrate(arm_poses)

                # 3. Control robot when started + calibrated
                if _START and self._calibrated:
                    if arm_poses["left"]["valid"] and arm_poses["right"]["valid"]:
                        left_wrist, right_wrist = self._compute_wrist_poses(arm_poses)

                        if left_wrist is not None:
                            t_ik = time.perf_counter()
                            q   = self.arm_ctrl.get_current_dual_arm_q()
                            dq  = self.arm_ctrl.get_current_dual_arm_dq()
                            sol_q, sol_tauff = self.arm_ik.solve_ik(
                                left_wrist, right_wrist, q, dq
                            )
                            ik_ms = (time.perf_counter() - t_ik) * 1000.0
                            self.arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)
                    else:
                        missing = [s for s in ("left", "right") if not arm_poses[s]["valid"]]
                        logger.warning(f"Tracker(s) lost: {missing} — holding position")

                elif _START and not self._calibrated:
                    logger.warning("Must calibrate first — press [c]!")
                    _START = False

                # 4. Visualization
                elapsed = time.perf_counter() - t0
                loop_hz = 1.0 / elapsed if elapsed > 0 else 0.0
                if self.viz:
                    self._print_status(arm_poses, left_wrist, right_wrist, loop_hz, ik_ms)
                else:
                    lv = arm_poses["left"]["valid"]
                    rv = arm_poses["right"]["valid"]
                    if not _START:
                        state = "calibrated" if self._calibrated else "waiting"
                        print(
                            f"\r  L={'OK' if lv else '--'}  R={'OK' if rv else '--'}"
                            f"  [{state}]  [c]=cal [r]=start [q]=quit    ",
                            end="",
                            flush=True,
                        )

                # 5. Sleep remainder
                sleep = period - (time.perf_counter() - t0)
                if sleep > 0:
                    time.sleep(sleep)

        except KeyboardInterrupt:
            logger.info("Interrupted by Ctrl+C")
        finally:
            stop_listening()
            self.xrc.stop()
            self.arm_ctrl.ctrl_dual_arm_go_home()
            logger.info("Shutdown complete.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Wrist teleoperation via XRoboToolkit SDK + Pico Motion Trackers",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--arm", choices=list(XrcTeleop.ARM_MAP), default="G1_29",
        help="Robot arm type",
    )
    p.add_argument("--frequency",       type=float, default=30.0,  help="Control loop frequency (Hz)")
    p.add_argument("--position-scale",  type=float, default=0.8,   help="Scale factor for tracker position deltas")
    p.add_argument("--network-interface", type=str, default=None,  help="Network interface for DDS (e.g. eth0)")
    p.add_argument("--sim",             action="store_true",        help="Sim mode (DDS domain 1)")
    p.add_argument("--no-adb",          action="store_true",        help="Skip ADB tunnel (use Wi-Fi connection)")
    p.add_argument("--viz",             action="store_true",        help="Enable rich terminal visualization")
    p.add_argument("--left-serial",     type=str, default=None,    help="Force serial number for left wrist tracker")
    p.add_argument("--right-serial",    type=str, default=None,    help="Force serial number for right wrist tracker")
    p.add_argument("--kp",              type=float, default=None,  help="Kp gain for arm motors")
    p.add_argument("--kp-wrist",        type=float, default=None,  help="Kp gain for wrist motors")
    p.add_argument("--kd",              type=float, default=None,  help="Kd damping for arm motors")
    p.add_argument("--kd-wrist",        type=float, default=None,  help="Kd damping for wrist motors")
    return p


def main() -> None:
    args = _build_parser().parse_args()
    teleop = XrcTeleop(args)
    teleop.run()


if __name__ == "__main__":
    main()
