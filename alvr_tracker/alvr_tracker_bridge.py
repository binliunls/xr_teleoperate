#!/usr/bin/env python3
"""
ALVR Motion Tracker Bridge

Reads PICO Motion Tracker data from ALVR via OpenVR/SteamVR API.

Requirements:
    pip install openvr numpy

Usage:
    python alvr_tracker_bridge.py --test     # Test mode with visualization
    python alvr_tracker_bridge.py            # Run as data source
"""

import argparse
import time
import sys
from dataclasses import dataclass, field
from typing import Optional, Dict, List
import numpy as np

try:
    import openvr

    HAS_OPENVR = True
except ImportError:
    HAS_OPENVR = False
    print("Error: openvr not installed. Install with: pip install openvr")
    print("Note: openvr requires SteamVR or ALVR to be running")


@dataclass
class TrackerPose:
    """Single tracker pose"""

    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    valid: bool = False
    device_index: int = -1
    serial: str = ""


class ALVRTrackerBridge:
    """
    Bridge to read Motion Tracker data from ALVR via OpenVR.

    ALVR exposes PICO Motion Trackers as generic trackers in SteamVR/OpenVR.
    This class reads those tracker poses and provides them in a usable format.
    """

    def __init__(self):
        self.vr_system = None
        self.is_initialized = False
        self.trackers: Dict[str, TrackerPose] = {}

        # Expected tracker roles (assigned in SteamVR)
        self.tracker_roles = {
            "waist": None,
            "left_foot": None,
            "right_foot": None,
            "left_wrist": None,  # If using wrist trackers
            "right_wrist": None,
        }

    def initialize(self) -> bool:
        """Initialize OpenVR connection"""
        if not HAS_OPENVR:
            return False

        try:
            # Initialize as background app (doesn't need SteamVR to display anything)
            self.vr_system = openvr.init(openvr.VRApplication_Background)
            self.is_initialized = True
            print("[ALVRTrackerBridge] OpenVR initialized successfully")
            self._enumerate_devices()
            return True
        except openvr.OpenVRError as e:
            print(f"[ALVRTrackerBridge] Failed to initialize OpenVR: {e}")
            print("Make sure ALVR streamer is running!")
            return False

    def shutdown(self):
        """Shutdown OpenVR connection"""
        if self.is_initialized:
            openvr.shutdown()
            self.is_initialized = False
            print("[ALVRTrackerBridge] OpenVR shutdown")

    def _enumerate_devices(self):
        """Find all connected trackers"""
        print("[ALVRTrackerBridge] Enumerating devices...")

        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = self.vr_system.getTrackedDeviceClass(i)

            if device_class == openvr.TrackedDeviceClass_Invalid:
                continue

            class_name = self._get_device_class_name(device_class)
            serial = self._get_string_property(i, openvr.Prop_SerialNumber_String)
            model = self._get_string_property(i, openvr.Prop_ModelNumber_String)

            print(f"  Device {i}: {class_name} - {model} (SN: {serial})")

            # Track generic trackers (full body Motion Trackers appear as this)
            if device_class == openvr.TrackedDeviceClass_GenericTracker:
                role = self._get_tracker_role(i)
                self.trackers[serial] = TrackerPose(device_index=i, serial=serial)
                print(f"    -> Registered as tracker (role: {role})")

            # Also track VRLink Hand Trackers (wrist Motion Trackers)
            # These appear as Controllers with "Hand" in the model name
            elif device_class == openvr.TrackedDeviceClass_Controller:
                if "Hand" in model or "VRLINKQ_Hand" in serial:
                    role = "left_wrist" if "Left" in model else "right_wrist"
                    self.trackers[serial] = TrackerPose(device_index=i, serial=serial)
                    print(f"    -> Registered as wrist tracker (role: {role})")

    def _get_device_class_name(self, device_class: int) -> str:
        """Get human-readable device class name"""
        classes = {
            openvr.TrackedDeviceClass_HMD: "HMD",
            openvr.TrackedDeviceClass_Controller: "Controller",
            openvr.TrackedDeviceClass_GenericTracker: "Tracker",
            openvr.TrackedDeviceClass_TrackingReference: "Base Station",
            openvr.TrackedDeviceClass_DisplayRedirect: "Display",
        }
        return classes.get(device_class, f"Unknown({device_class})")

    def _get_string_property(self, device_index: int, prop: int) -> str:
        """Get string property from device"""
        try:
            return self.vr_system.getStringTrackedDeviceProperty(device_index, prop)
        except Exception:
            return "Unknown"

    def _get_tracker_role(self, device_index: int) -> str:
        """Get tracker role (waist, left_foot, etc.)"""
        try:
            role = self.vr_system.getStringTrackedDeviceProperty(device_index, openvr.Prop_ControllerType_String)
            return role if role else "unassigned"
        except Exception:
            return "unassigned"

    def _matrix_to_pose(self, matrix) -> tuple:
        """Convert OpenVR 3x4 matrix to position and rotation"""
        # Extract position (last column)
        position = np.array([matrix[0][3], matrix[1][3], matrix[2][3]])

        # Extract rotation matrix (first 3 columns)
        rotation = np.array(
            [
                [matrix[0][0], matrix[0][1], matrix[0][2]],
                [matrix[1][0], matrix[1][1], matrix[1][2]],
                [matrix[2][0], matrix[2][1], matrix[2][2]],
            ]
        )

        return position, rotation

    def update(self) -> Dict[str, TrackerPose]:
        """Update all tracker poses"""
        if not self.is_initialized:
            return {}

        # Get all device poses
        poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0,  # Predicted seconds from now
            openvr.k_unMaxTrackedDeviceCount,
        )

        # Update each tracker
        for serial, tracker in self.trackers.items():
            if tracker.device_index < 0:
                continue

            pose = poses[tracker.device_index]
            tracker.valid = pose.bPoseIsValid

            if pose.bPoseIsValid:
                position, rotation = self._matrix_to_pose(pose.mDeviceToAbsoluteTracking)
                tracker.position = position
                tracker.rotation = rotation

                # Velocity
                tracker.velocity = np.array([pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2]])
                tracker.angular_velocity = np.array(
                    [pose.vAngularVelocity.v[0], pose.vAngularVelocity.v[1], pose.vAngularVelocity.v[2]]
                )

        return self.trackers

    def get_tracker_by_role(self, role: str) -> Optional[TrackerPose]:
        """Get tracker by its assigned role"""
        # TODO: Implement role-based lookup
        # For now, return first tracker
        for tracker in self.trackers.values():
            if tracker.valid:
                return tracker
        return None

    def get_all_valid_trackers(self) -> List[TrackerPose]:
        """Get all trackers with valid poses"""
        return [t for t in self.trackers.values() if t.valid]

    def get_wrist_trackers(self) -> Dict[str, Optional[TrackerPose]]:
        """Get left and right wrist trackers for robot arm control"""
        result = {"left": None, "right": None}

        for serial, tracker in self.trackers.items():
            if not tracker.valid:
                continue
            # Check for VRLink Hand Trackers
            if "Left" in serial or "left" in serial.lower():
                result["left"] = tracker
            elif "Right" in serial or "right" in serial.lower():
                result["right"] = tracker

        return result

    def get_arm_poses(self) -> Dict[str, dict]:
        """
        Get wrist tracker poses formatted for robot arm control.
        Returns dict with 'left' and 'right' keys containing position and rotation.
        """
        wrists = self.get_wrist_trackers()
        result = {}

        for side in ["left", "right"]:
            tracker = wrists[side]
            if tracker and tracker.valid:
                result[side] = {
                    "position": tracker.position.copy(),
                    "rotation": tracker.rotation.copy(),
                    "velocity": tracker.velocity.copy(),
                    "valid": True,
                }
            else:
                result[side] = {"position": np.zeros(3), "rotation": np.eye(3), "velocity": np.zeros(3), "valid": False}

        return result


def test_mode():
    """Run in test mode - print tracker data"""
    bridge = ALVRTrackerBridge()

    if not bridge.initialize():
        print("\nFailed to initialize. Make sure:")
        print("1. ALVR streamer is running on this PC")
        print("2. PICO is connected via ALVR")
        print("3. Motion Trackers are enabled in ALVR settings")
        sys.exit(1)

    print("\n" + "=" * 60)
    print("Tracker Test Mode")
    print("Press Ctrl+C to exit")
    print("=" * 60 + "\n")

    try:
        while True:
            trackers = bridge.update()
            arm_poses = bridge.get_arm_poses()

            # Clear screen and print header
            print("\033[H\033[J", end="")  # Clear screen
            print("ALVR Motion Tracker Bridge - Test Mode")
            print("=" * 60)

            # Show wrist trackers (for robot arm control)
            print("\n--- WRIST TRACKERS (for robot arms) ---")
            for side in ["left", "right"]:
                pose = arm_poses[side]
                status = "VALID" if pose["valid"] else "NOT FOUND"
                pos = pose["position"]
                print(f"  {side.upper()} wrist: {status}")
                if pose["valid"]:
                    print(f"    Position: [{pos[0]:7.3f}, {pos[1]:7.3f}, {pos[2]:7.3f}]")
                    vel = np.linalg.norm(pose["velocity"])
                    print(f"    Velocity: {vel:.3f} m/s")

            # Show all registered trackers
            print(f"\n--- ALL TRACKERS ({len(trackers)} registered) ---")
            for serial, tracker in trackers.items():
                status = "VALID" if tracker.valid else "INVALID"
                pos = tracker.position
                # Truncate serial for display
                display_name = serial[:35] if len(serial) > 35 else serial
                print(f"  {display_name}")
                print(f"    Status: {status}, Pos: [{pos[0]:6.2f}, {pos[1]:6.2f}, {pos[2]:6.2f}]")

            if not trackers:
                print("\nNo trackers detected!")
                print("Check that Motion Trackers are paired with PICO")

            time.sleep(0.05)  # 20 Hz update

    except KeyboardInterrupt:
        print("\n\nExiting...")
    finally:
        bridge.shutdown()


def main():
    parser = argparse.ArgumentParser(description="ALVR Motion Tracker Bridge")
    parser.add_argument("--test", action="store_true", help="Run in test mode (print tracker data)")
    args = parser.parse_args()

    if args.test:
        test_mode()
    else:
        # Run as importable module
        bridge = ALVRTrackerBridge()
        if bridge.initialize():
            print("Bridge initialized. Import this module to use.")
            bridge.shutdown()


if __name__ == "__main__":
    main()
