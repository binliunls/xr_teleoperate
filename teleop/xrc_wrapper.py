"""
XrcWrapper — XRoboToolkit SDK wrapper for xr_teleoperate.

Replaces TeleVuerWrapper/ALVRTrackerBridge as the data source for wrist
teleoperation.  Uses the native XrClient SDK (ADB tunnel, no browser/ALVR)
to stream Pico Motion Tracker poses directly.

Data flow:
    Pico Motion Trackers
        └── xrobotoolkit_sdk (xrt)  ←──── ADB reverse tunnel (port 63901)
                └── XrClient.get_motion_tracker_data()
                        └── XrcWrapper.get_arm_poses()
                                └── teleop_hand_and_arm_xrc.py
                                        └── solve_ik() → ctrl_dual_arm()

Coordinate frames:
    XRoboToolkit SDK  : Y-up, matches OpenXR/headset convention
    Robot frame        : Z-up, X-forward  (same as groot PicoStreamer)

    Conversion applied: R_XR_TO_ROBOT (Y↔Z swap with -Y)

Serial → side assignment:
    The SDK returns trackers by serial number (unknown at design time).
    On first detection we assign left/right by the order they appear OR
    by a user-provided serial map (--left-serial / --right-serial).
    After calibration the assignment is locked.

Usage:
    wrapper = XrcWrapper(use_adb=True)
    wrapper.start()
    arm_poses = wrapper.get_arm_poses()   # same interface as ALVRTrackerBridge
    wrapper.stop()
"""

from __future__ import annotations

import shutil
import subprocess
import threading
import time
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation as R

# ---------------------------------------------------------------------------
# Optional import: xrobotoolkit_sdk may not be installed in every env.
# We import lazily so that the module can be loaded for inspection without
# the SDK being present (useful for unit-tests / CI).
# ---------------------------------------------------------------------------
try:
    import xrobotoolkit_sdk as xrt
    _HAS_XRT = True
except ImportError:
    xrt = None  # type: ignore
    _HAS_XRT = False

# ---------------------------------------------------------------------------
# Coordinate frame: XRoboToolkit (Y-up OpenXR) → Robot (Z-up, X-forward)
#
#   XR  X  →  Robot  X  (no change)
#   XR  Y  →  Robot  Z  (Y-up becomes Z-up)
#   XR  Z  →  Robot -Y  (OpenXR -Z forward becomes robot +Y … left)
#
# This matches what groot/PicoStreamer calls R_HEADSET_TO_WORLD.
# ---------------------------------------------------------------------------
R_XR_TO_ROBOT = np.array([
    [1,  0,  0],
    [0,  0,  1],
    [0, -1,  0],
], dtype=float)


# ---------------------------------------------------------------------------
# ADB tunnel management (ported from groot PicoStreamer.PicoAdb, simplified)
# ---------------------------------------------------------------------------
TRACKING_PORT = 63901
_KEEPALIVE_INTERVAL = 2.0  # seconds


class _PicoAdb:
    """Open ADB reverse tunnel for the XRoboToolkit tracking port."""

    def __init__(self) -> None:
        adb = shutil.which("adb")
        if not adb:
            raise RuntimeError(
                "adb not found. Install with: sudo apt install -y android-tools-adb"
            )
        result = subprocess.run([adb, "devices"], capture_output=True, text=True, check=True)
        lines = [l for l in result.stdout.strip().splitlines()[1:] if l.strip()]
        if not lines:
            raise RuntimeError("No ADB devices found. Connect Pico via USB.")
        if "unauthorized" in lines[0]:
            raise RuntimeError(
                "ADB device unauthorized. Accept the prompt on the Pico headset."
            )
        self._adb = [adb, "-s", lines[0].split()[0]]
        self._stop = threading.Event()
        self._setup_tunnel()
        self._thread = threading.Thread(target=self._keepalive, daemon=True)
        self._thread.start()
        print(f"[XrcWrapper] ADB tunnel established on port {TRACKING_PORT}")

    def _setup_tunnel(self) -> None:
        subprocess.run(
            self._adb + ["reverse", f"tcp:{TRACKING_PORT}", f"tcp:{TRACKING_PORT}"],
            capture_output=True,
        )

    def _keepalive(self) -> None:
        while not self._stop.wait(timeout=_KEEPALIVE_INTERVAL):
            self._setup_tunnel()

    def close(self) -> None:
        self._stop.set()
        subprocess.run(
            self._adb + ["reverse", "--remove", f"tcp:{TRACKING_PORT}"],
            capture_output=True,
        )


# ---------------------------------------------------------------------------
# Tracker data container
# ---------------------------------------------------------------------------
@dataclass
class TrackerPose:
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rotation: np.ndarray = field(default_factory=lambda: np.eye(3))   # 3×3
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    serial: str = ""
    valid: bool = False


# ---------------------------------------------------------------------------
# XrcWrapper — public API
# ---------------------------------------------------------------------------
class XrcWrapper:
    """
    Drop-in replacement for ALVRTrackerBridge.

    Public API (matches ALVRTrackerBridge):
        start()              — init SDK + ADB
        stop()               — shutdown
        update()             — poll SDK (call every loop iteration)
        get_arm_poses()      — returns {"left": {...}, "right": {...}}

    Extended API:
        get_all_trackers()   — returns list[TrackerPose] (for viz)
        get_headset_pose()   — returns (4,4) SE(3) or None
        calibrated           — bool
        left_serial / right_serial — assigned serial numbers
    """

    def __init__(
        self,
        use_adb: bool = True,
        left_serial: str | None = None,
        right_serial: str | None = None,
        position_scale: float = 1.0,
    ) -> None:
        """
        Args:
            use_adb:        Open ADB reverse tunnel (needed when Pico is USB-connected).
            left_serial:    Force serial number for left wrist tracker.
            right_serial:   Force serial number for right wrist tracker.
            position_scale: Scale factor applied to position deltas.
        """
        if not _HAS_XRT:
            raise RuntimeError(
                "xrobotoolkit_sdk not installed. "
                "Install it from the XRoboToolkit repository."
            )

        self._use_adb = use_adb
        self._adb: _PicoAdb | None = None
        self._position_scale = position_scale

        # Serial → side assignment
        self._left_serial: str | None = left_serial
        self._right_serial: str | None = right_serial
        self._serials_locked = left_serial is not None and right_serial is not None

        # Latest raw tracker data keyed by serial
        self._trackers: dict[str, TrackerPose] = {}
        self._lock = threading.Lock()

        self._started = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Initialize ADB tunnel and XRoboToolkit SDK."""
        if self._started:
            return
        if self._use_adb:
            self._adb = _PicoAdb()
            time.sleep(0.5)   # let the tunnel stabilize
        xrt.init()
        print("[XrcWrapper] XRoboToolkit SDK initialized.")
        self._started = True

    def stop(self) -> None:
        """Shutdown SDK and ADB tunnel."""
        if not self._started:
            return
        xrt.close()
        if self._adb:
            self._adb.close()
        self._started = False
        print("[XrcWrapper] Stopped.")

    # ------------------------------------------------------------------
    # Data polling
    # ------------------------------------------------------------------

    def update(self) -> None:
        """
        Poll the SDK and refresh internal tracker state.
        Call this once per control-loop iteration.
        """
        num = xrt.num_motion_data_available()
        if num == 0:
            return

        raw_poses   = xrt.get_motion_tracker_pose()          # list of [x,y,z,qx,qy,qz,qw]
        raw_vels    = xrt.get_motion_tracker_velocity()       # list of [vx,vy,vz]
        serials     = xrt.get_motion_tracker_serial_numbers() # list of str

        new_trackers: dict[str, TrackerPose] = {}
        for i in range(num):
            serial = serials[i]
            vec7   = np.array(raw_poses[i], dtype=float)
            pos_xr = vec7[:3]
            quat   = vec7[3:]   # [qx,qy,qz,qw] scalar-last

            rot_xr = R.from_quat(quat).as_matrix()

            # Convert to robot frame
            pos_robot = R_XR_TO_ROBOT @ pos_xr
            rot_robot = R_XR_TO_ROBOT @ rot_xr @ R_XR_TO_ROBOT.T

            vel_xr    = np.array(raw_vels[i][:3], dtype=float)
            vel_robot = R_XR_TO_ROBOT @ vel_xr

            new_trackers[serial] = TrackerPose(
                position=pos_robot,
                rotation=rot_robot,
                velocity=vel_robot,
                serial=serial,
                valid=True,
            )

        with self._lock:
            self._trackers = new_trackers
            self._auto_assign_serials()

    def _auto_assign_serials(self) -> None:
        """
        Assign left/right serial numbers using the same strategy as
        ALVRTrackerBridge.get_wrist_trackers():

          1. Name-based (preferred): check if "Left" or "Right" appears in the
             serial string.  Pico Motion Tracker serials from both ALVR and the
             XRoboToolkit SDK contain the side in their name, e.g.
                 VRLINKQ_Hand_Left_XXXX  /  VRLINKQ_Hand_Right_XXXX
             This is reliable regardless of enumeration order.

          2. Sorted-order fallback: if neither serial contains a side token
             (e.g., a third-party tracker with an opaque serial), fall back to
             alphabetical order — first serial → left, second → right.
             This is the old behaviour and is noted as fragile in the docstring.

        Once both sides are assigned the mapping is locked until stop()/start().
        """
        if self._serials_locked:
            return
        known = list(self._trackers.keys())
        if not known:
            return

        # Pass 1 — name-based assignment (mirrors ALVR bridge lines 217-220)
        for serial in known:
            s_lower = serial.lower()
            if self._left_serial is None and ("left" in s_lower or "_l_" in s_lower):
                self._left_serial = serial
                print(f"[XrcWrapper] Name-assigned  LEFT  → serial: {serial}")
            elif self._right_serial is None and ("right" in s_lower or "_r_" in s_lower):
                self._right_serial = serial
                print(f"[XrcWrapper] Name-assigned  RIGHT → serial: {serial}")

        # Pass 2 — sorted-order fallback for opaque serials
        if self._left_serial is None or self._right_serial is None:
            unassigned = sorted(
                s for s in known
                if s not in (self._left_serial, self._right_serial)
            )
            if self._left_serial is None and len(unassigned) >= 1:
                self._left_serial = unassigned[0]
                print(f"[XrcWrapper] Order-assigned LEFT  → serial: {unassigned[0]}  (fallback)")
            if self._right_serial is None and len(unassigned) >= 2:
                self._right_serial = unassigned[1]
                print(f"[XrcWrapper] Order-assigned RIGHT → serial: {unassigned[1]}  (fallback)")

        if self._left_serial and self._right_serial:
            self._serials_locked = True

    # ------------------------------------------------------------------
    # Public getters
    # ------------------------------------------------------------------

    def get_all_trackers(self) -> list[TrackerPose]:
        """Return all currently tracked poses (for visualization)."""
        with self._lock:
            return list(self._trackers.values())

    def get_headset_pose(self) -> np.ndarray | None:
        """
        Return headset pose as (4,4) SE(3) in robot frame, or None if unavailable.
        Useful for yaw-compensated relative motion.
        """
        raw = np.array(xrt.get_headset_pose(), dtype=float)
        if np.allclose(raw, 0):
            return None
        pos_xr = raw[:3]
        quat   = raw[3:]
        rot_xr = R.from_quat(quat).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R_XR_TO_ROBOT @ rot_xr @ R_XR_TO_ROBOT.T
        T[:3,  3] = R_XR_TO_ROBOT @ pos_xr
        return T

    def get_arm_poses(self) -> dict[str, dict]:
        """
        Return wrist tracker poses in the same format as ALVRTrackerBridge.get_arm_poses():

            {
              "left":  {"position": (3,), "rotation": (3,3), "velocity": (3,), "valid": bool},
              "right": {"position": (3,), "rotation": (3,3), "velocity": (3,), "valid": bool},
            }

        position and rotation are already in robot frame (Z-up, X-forward).
        """
        empty = {"position": np.zeros(3), "rotation": np.eye(3), "velocity": np.zeros(3), "valid": False}
        result = {"left": dict(empty), "right": dict(empty)}

        with self._lock:
            if self._left_serial and self._left_serial in self._trackers:
                t = self._trackers[self._left_serial]
                result["left"] = {
                    "position": t.position.copy(),
                    "rotation": t.rotation.copy(),
                    "velocity": t.velocity.copy(),
                    "valid": t.valid,
                }
            if self._right_serial and self._right_serial in self._trackers:
                t = self._trackers[self._right_serial]
                result["right"] = {
                    "position": t.position.copy(),
                    "rotation": t.rotation.copy(),
                    "velocity": t.velocity.copy(),
                    "valid": t.valid,
                }

        return result

    @property
    def left_serial(self) -> str | None:
        return self._left_serial

    @property
    def right_serial(self) -> str | None:
        return self._right_serial

    @property
    def num_trackers(self) -> int:
        with self._lock:
            return len(self._trackers)
