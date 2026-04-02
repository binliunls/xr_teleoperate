#!/usr/bin/env python3
"""
ALVR Wrist Tracker Teleoperation

Control the robot arms using PICO wrist Motion Trackers streamed via ALVR.
The wrist trackers appear as "VRLink Hand Tracker" devices in SteamVR/OpenVR.

Usage:
    python alvr_tracker_teleop.py --arm=H2
"""

import argparse
import time
import sys
import os
import threading

# Add parent directory to path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

import numpy as np
import pinocchio as pin
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
from teleop.robot_control.robot_arm_ik import (
    G1_29_ArmIK,
    G1_23_ArmIK,
    H1_2_ArmIK,
    H1_ArmIK,
    H2_ArmIK,
)
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
    """Teleoperation using ALVR wrist Motion Trackers (VRLink Hand Trackers)"""

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
        self.arm_ctrl = controller_cls()
        logger.info(f"Robot {args.arm} initialized")

        # Initialize ALVR tracker bridge
        self.tracker_bridge = ALVRTrackerBridge()

        # Calibration data (reference poses when calibration is triggered)
        self.ref_left_pos = None
        self.ref_right_pos = None
        self.ref_left_rot = None
        self.ref_right_rot = None

        # Initial wrist poses (captured at calibration for relative motion)
        self.init_left_wrist_pose = None
        self.init_right_wrist_pose = None

        # Scaling
        self.position_scale = args.position_scale

        # Coordinate transform (OpenVR to Robot frame)
        # OpenVR: Y-up, -Z forward
        # Robot: Z-up, X forward (adjust based on your robot setup)
        self.R_openvr_to_robot = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])

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
        self.arm_ctrl.ctrl_dual_arm_go_home()
        logger.info("Shutdown complete")

    def transform_to_robot_frame(self, position: np.ndarray, rotation: np.ndarray) -> tuple:
        """Transform from OpenVR to robot coordinate frame"""
        robot_pos = self.R_openvr_to_robot @ position
        robot_rot = self.R_openvr_to_robot @ rotation @ self.R_openvr_to_robot.T
        return robot_pos, robot_rot

    def calibrate(self, arm_poses: dict) -> bool:
        """Capture current tracker positions as reference"""
        global CALIBRATE
        CALIBRATE = False

        if not arm_poses["left"]["valid"] or not arm_poses["right"]["valid"]:
            logger.warning("Need both wrist trackers for calibration")
            if not arm_poses["left"]["valid"]:
                logger.warning("  Left wrist tracker not detected")
            if not arm_poses["right"]["valid"]:
                logger.warning("  Right wrist tracker not detected")
            return False

        # Store reference positions in OpenVR frame
        self.ref_left_pos = arm_poses["left"]["position"].copy()
        self.ref_right_pos = arm_poses["right"]["position"].copy()
        self.ref_left_rot = arm_poses["left"]["rotation"].copy()
        self.ref_right_rot = arm_poses["right"]["rotation"].copy()

        # Get current robot state to compute actual end-effector poses
        current_lr_arm_q = self.arm_ctrl.get_current_dual_arm_q()

        # Compute FK
        pin.forwardKinematics(self.arm_ik.reduced_robot.model, self.arm_ik.reduced_robot.data, current_lr_arm_q)
        pin.updateFramePlacements(self.arm_ik.reduced_robot.model, self.arm_ik.reduced_robot.data)

        left_ee_se3 = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.L_hand_id]
        right_ee_se3 = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.R_hand_id]

        self.init_left_wrist_pose = left_ee_se3.homogeneous.copy()
        self.init_right_wrist_pose = right_ee_se3.homogeneous.copy()

        logger.info("=" * 60)
        logger.info("CALIBRATION COMPLETE!")
        logger.info("  Your current arm position is now the reference.")
        logger.info("  Press [r] to start - robot will follow your movements.")
        logger.info("=" * 60)
        return True

    def compute_wrist_poses(self, arm_poses: dict) -> tuple:
        """Compute wrist poses for IK from tracker data"""
        if self.ref_left_pos is None or self.init_left_wrist_pose is None:
            return None, None

        if not arm_poses["left"]["valid"] or not arm_poses["right"]["valid"]:
            return None, None

        # Get current tracker positions
        left_pos = arm_poses["left"]["position"]
        left_rot = arm_poses["left"]["rotation"]
        right_pos = arm_poses["right"]["position"]
        right_rot = arm_poses["right"]["rotation"]

        # Compute relative motion from reference (in OpenVR frame)
        left_delta = (left_pos - self.ref_left_pos) * self.position_scale
        right_delta = (right_pos - self.ref_right_pos) * self.position_scale

        # Transform delta to robot frame
        left_delta_robot, _ = self.transform_to_robot_frame(left_delta, np.eye(3))
        right_delta_robot, _ = self.transform_to_robot_frame(right_delta, np.eye(3))

        # Compute relative rotation from reference
        left_rel_rot = left_rot @ self.ref_left_rot.T
        right_rel_rot = right_rot @ self.ref_right_rot.T

        # Transform rotations to robot frame
        _, left_rel_rot_robot = self.transform_to_robot_frame(np.zeros(3), left_rel_rot)
        _, right_rel_rot_robot = self.transform_to_robot_frame(np.zeros(3), right_rel_rot)

        # Apply relative motion to initial poses
        left_target_pos = self.init_left_wrist_pose[:3, 3] + left_delta_robot
        right_target_pos = self.init_right_wrist_pose[:3, 3] + right_delta_robot

        # Apply relative rotation to initial robot poses
        left_target_rot = left_rel_rot_robot @ self.init_left_wrist_pose[:3, :3]
        right_target_rot = right_rel_rot_robot @ self.init_right_wrist_pose[:3, :3]

        # Create 4x4 homogeneous matrices for IK
        left_wrist_pose = np.eye(4)
        left_wrist_pose[:3, :3] = left_target_rot
        left_wrist_pose[:3, 3] = left_target_pos

        right_wrist_pose = np.eye(4)
        right_wrist_pose[:3, :3] = right_target_rot
        right_wrist_pose[:3, 3] = right_target_pos

        return left_wrist_pose, right_wrist_pose

    def run(self):
        """Main control loop"""
        global START, STOP, CALIBRATE

        if not self.initialize():
            return

        # Start keyboard listener in a thread
        listen_keyboard_thread = threading.Thread(
            target=listen_keyboard,
            kwargs={
                "on_press": on_press,
                "until": None,
                "sequential": False,
            },
            daemon=True,
        )
        listen_keyboard_thread.start()

        logger.info("-" * 60)
        logger.info("ALVR Wrist Tracker Teleoperation")
        logger.info("-" * 60)
        logger.info("Controls:")
        logger.info("  [c] - Calibrate (MUST do first!)")
        logger.info("  [r] - Start tracking (after calibration)")
        logger.info("  [q] - Quit")
        logger.info("-" * 60)
        logger.info("CALIBRATION INSTRUCTIONS:")
        logger.info("  1. Position your arms to match robot's home pose")
        logger.info("  2. Hold still and press [c] to calibrate")
        logger.info("  3. Then press [r] to start - robot follows your motion")
        logger.info("-" * 60)
        logger.info("Waiting for wrist trackers (VRLink Hand Trackers)...")

        period = 1.0 / self.frequency
        calibrated = False

        try:
            while not STOP:
                loop_start = time.time()

                # Update tracker data
                self.tracker_bridge.update()
                arm_poses = self.tracker_bridge.get_arm_poses()

                # Check tracker status
                left_valid = arm_poses["left"]["valid"]
                right_valid = arm_poses["right"]["valid"]
                both_valid = left_valid and right_valid

                # Handle calibration request
                if CALIBRATE:
                    if self.calibrate(arm_poses):
                        calibrated = True

                # Control robot if started and calibrated
                if START and calibrated:
                    if both_valid:
                        # Compute wrist poses
                        left_wrist_pose, right_wrist_pose = self.compute_wrist_poses(arm_poses)

                        if left_wrist_pose is not None:
                            # Get current robot state
                            current_lr_arm_q = self.arm_ctrl.get_current_dual_arm_q()
                            current_lr_arm_dq = self.arm_ctrl.get_current_dual_arm_dq()

                            # Solve IK (same interface as teleop_hand_and_arm.py)
                            sol_q, sol_tauff = self.arm_ik.solve_ik(
                                left_wrist_pose,
                                right_wrist_pose,
                                current_lr_arm_q,
                                current_lr_arm_dq,
                            )

                            # Send to robot
                            self.arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)
                    else:
                        # Print status occasionally
                        if int(time.time() * 2) % 2 == 0:
                            status = []
                            if not left_valid:
                                status.append("LEFT missing")
                            if not right_valid:
                                status.append("RIGHT missing")
                            logger.warning(f"Wrist tracker issue: {', '.join(status)}")

                elif START and not calibrated:
                    logger.info("Press [c] to calibrate first!")
                    START = False

                # Status display
                if not START:
                    left_status = "OK" if left_valid else "MISSING"
                    right_status = "OK" if right_valid else "MISSING"
                    print(
                        f"\rWrist trackers - L:{left_status} R:{right_status} | [c]=calibrate [r]=start",
                        end="",
                    )

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
    parser = argparse.ArgumentParser(description="ALVR Wrist Tracker Teleoperation")
    parser.add_argument(
        "--arm",
        type=str,
        choices=["G1_29", "G1_23", "H1_2", "H1", "H2"],
        default="H2",
        help="Robot arm type",
    )
    parser.add_argument("--frequency", type=float, default=30.0, help="Control frequency in Hz")
    parser.add_argument("--network-interface", type=str, default=None, help="Network interface for DDS")
    parser.add_argument("--position-scale", type=float, default=1.0, help="Scale factor for position movements")

    args = parser.parse_args()

    teleop = ALVRTeleop(args)
    teleop.run()


if __name__ == "__main__":
    main()
