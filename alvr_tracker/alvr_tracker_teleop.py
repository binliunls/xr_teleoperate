#!/usr/bin/env python3
"""
ALVR Motion Tracker Teleoperation

Control the robot using PICO Motion Trackers streamed via ALVR.

Usage:
    python alvr_tracker_teleop.py --arm=H2
    python alvr_tracker_teleop.py --arm=H2 --visualize
"""

import argparse
import time
import sys
import os

# Add parent directory to path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

import numpy as np
import logging_mp

logging_mp.basicConfig(level=logging_mp.INFO)
logger = logging_mp.getLogger(__name__)

from alvr_tracker_bridge import ALVRTrackerBridge
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from teleop.robot_control.robot_arm import (
    G1_29_ArmController,
    G1_23_ArmController,
    H1_2_ArmController,
    H1_ArmController,
    H2_ArmController,
)
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK, H2_ArmIK
from sshkeyboard import listen_keyboard, stop_listening

# State variables
START = False
STOP = False
CALIBRATE = False


def on_press(key):
    global START, STOP, CALIBRATE
    if key == "r":
        START = True
        logger.info("Starting tracking...")
    elif key == "q":
        START = False
        STOP = True
        logger.info("Stopping...")
    elif key == "c":
        CALIBRATE = True
        logger.info("Calibrating...")


class ALVRTeleop:
    """Teleoperation using ALVR Motion Trackers"""

    def __init__(self, args):
        self.args = args
        self.frequency = args.frequency

        # Initialize DDS for robot communication
        if args.network_interface:
            ChannelFactoryInitialize(0, args.network_interface)
        else:
            ChannelFactoryInitialize(0)
        logger.info("DDS initialized")

        # Initialize robot arm controller and IK
        arm_controllers = {
            "G1_29": (G1_29_ArmController, G1_29_ArmIK),
            "G1_23": (G1_23_ArmController, G1_23_ArmIK),
            "H1_2": (H1_2_ArmController, H1_2_ArmIK),
            "H1": (H1_ArmController, H1_ArmIK),
            "H2": (H2_ArmController, H2_ArmIK),
        }

        controller_cls, ik_cls = arm_controllers[args.arm]
        self.arm_ik = ik_cls()
        self.robot = controller_cls()
        logger.info(f"Robot {args.arm} initialized")

        # Initialize ALVR tracker bridge
        self.tracker_bridge = ALVRTrackerBridge()

        # Calibration data
        self.ref_left_pos = None
        self.ref_right_pos = None
        self.ref_left_rot = None
        self.ref_right_rot = None

        # Robot initial pose
        self.robot_init_left_pos = None
        self.robot_init_right_pos = None
        self.robot_init_left_rot = None
        self.robot_init_right_rot = None

        # Scaling
        self.position_scale = args.position_scale

        # Coordinate transform (OpenVR to Robot)
        # OpenVR: Y-up, -Z forward
        # Robot: Z-up, X forward (adjust as needed)
        self.R_robot_openvr = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])

    def initialize(self) -> bool:
        """Initialize the tracker bridge"""
        if not self.tracker_bridge.initialize():
            logger.error("Failed to initialize ALVR tracker bridge")
            return False
        logger.info("ALVR tracker bridge initialized")
        return True

    def shutdown(self):
        """Cleanup"""
        self.tracker_bridge.shutdown()
        self.robot.ctrl_dual_arm_go_home()
        logger.info("Shutdown complete")

    def calibrate(self):
        """Capture current tracker positions as reference"""
        global CALIBRATE
        CALIBRATE = False

        trackers = self.tracker_bridge.get_all_valid_trackers()

        if len(trackers) < 2:
            logger.warning("Need at least 2 trackers for calibration")
            return False

        # Assume first two trackers are left and right wrists
        # TODO: Better tracker assignment based on position
        left_tracker = trackers[0]
        right_tracker = trackers[1] if len(trackers) > 1 else trackers[0]

        self.ref_left_pos = left_tracker.position.copy()
        self.ref_right_pos = right_tracker.position.copy()
        self.ref_left_rot = left_tracker.rotation.copy()
        self.ref_right_rot = right_tracker.rotation.copy()

        # Get robot initial pose
        self.robot_init_left_pos, self.robot_init_left_rot = self.arm_ik.get_init_ee_pose("left")
        self.robot_init_right_pos, self.robot_init_right_rot = self.arm_ik.get_init_ee_pose("right")

        logger.info("Calibration complete!")
        logger.info(f"  Left ref: {self.ref_left_pos}")
        logger.info(f"  Right ref: {self.ref_right_pos}")
        return True

    def transform_to_robot_frame(self, position: np.ndarray, rotation: np.ndarray) -> tuple:
        """Transform from OpenVR to robot coordinate frame"""
        robot_pos = self.R_robot_openvr @ position
        robot_rot = self.R_robot_openvr @ rotation @ self.R_robot_openvr.T
        return robot_pos, robot_rot

    def compute_robot_target(self, trackers: list) -> tuple:
        """Compute robot target positions from tracker data"""
        if self.ref_left_pos is None:
            return None, None, None, None

        if len(trackers) < 2:
            return None, None, None, None

        left_tracker = trackers[0]
        right_tracker = trackers[1]

        # Compute relative motion from reference
        left_delta = left_tracker.position - self.ref_left_pos
        right_delta = right_tracker.position - self.ref_right_pos

        # Transform to robot frame
        left_delta_robot, _ = self.transform_to_robot_frame(left_delta, np.eye(3))
        right_delta_robot, _ = self.transform_to_robot_frame(right_delta, np.eye(3))

        # Scale
        left_delta_robot *= self.position_scale
        right_delta_robot *= self.position_scale

        # Add to robot initial position
        left_target_pos = self.robot_init_left_pos + left_delta_robot
        right_target_pos = self.robot_init_right_pos + right_delta_robot

        # Compute rotation (relative rotation from reference)
        left_rel_rot = left_tracker.rotation @ self.ref_left_rot.T
        right_rel_rot = right_tracker.rotation @ self.ref_right_rot.T

        # Transform rotations
        _, left_target_rot = self.transform_to_robot_frame(np.zeros(3), left_rel_rot)
        _, right_target_rot = self.transform_to_robot_frame(np.zeros(3), right_rel_rot)

        # Apply to robot initial rotation
        left_target_rot = left_target_rot @ self.robot_init_left_rot
        right_target_rot = right_target_rot @ self.robot_init_right_rot

        return left_target_pos, left_target_rot, right_target_pos, right_target_rot

    def run(self):
        """Main control loop"""
        global START, STOP, CALIBRATE

        if not self.initialize():
            return

        # Start keyboard listener
        listen_keyboard(on_press=on_press, sequential=True, delay_second_char=0.05)

        logger.info("-" * 60)
        logger.info("ALVR Motion Tracker Teleoperation")
        logger.info("-" * 60)
        logger.info("Controls:")
        logger.info("  [r] - Start tracking")
        logger.info("  [c] - Calibrate (capture reference pose)")
        logger.info("  [q] - Quit")
        logger.info("-" * 60)
        logger.info("Waiting for trackers...")

        period = 1.0 / self.frequency
        calibrated = False

        try:
            while not STOP:
                loop_start = time.time()

                # Update tracker data
                self.tracker_bridge.update()
                trackers = self.tracker_bridge.get_all_valid_trackers()

                # Handle calibration request
                if CALIBRATE:
                    if self.calibrate():
                        calibrated = True

                # Control robot if started and calibrated
                if START and calibrated:
                    if len(trackers) >= 2:
                        # Compute targets
                        left_pos, left_rot, right_pos, right_rot = self.compute_robot_target(trackers)

                        if left_pos is not None:
                            # Solve IK
                            left_q = self.arm_ik.solve_ik(left_pos, left_rot, "left")
                            right_q = self.arm_ik.solve_ik(right_pos, right_rot, "right")

                            # Send to robot
                            if left_q is not None and right_q is not None:
                                self.robot.ctrl_dual_arm(left_q, right_q)
                    else:
                        # Print status occasionally
                        if int(time.time() * 2) % 2 == 0:
                            logger.warning(f"Only {len(trackers)} tracker(s) detected")

                elif START and not calibrated:
                    logger.info("Press [c] to calibrate first!")
                    START = False

                # Status display
                if len(trackers) > 0 and not START:
                    print(f"\rTrackers: {len(trackers)} | Press [c] to calibrate, [r] to start", end="")

                # Maintain loop frequency
                elapsed = time.time() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)

        except KeyboardInterrupt:
            logger.info("Interrupted")
        finally:
            stop_listening()
            self.shutdown()
            logger.info("Exited")


def main():
    parser = argparse.ArgumentParser(description="ALVR Motion Tracker Teleoperation")
    parser.add_argument(
        "--arm", type=str, choices=["G1_29", "G1_23", "H1_2", "H1", "H2"], default="H2", help="Robot arm type"
    )
    parser.add_argument("--frequency", type=float, default=30.0, help="Control frequency in Hz")
    parser.add_argument("--network-interface", type=str, default=None, help="Network interface for DDS")
    parser.add_argument("--position-scale", type=float, default=1.0, help="Scale factor for position movements")
    parser.add_argument("--visualize", action="store_true", help="Enable Rerun visualization")

    args = parser.parse_args()

    teleop = ALVRTeleop(args)
    teleop.run()


if __name__ == "__main__":
    main()
