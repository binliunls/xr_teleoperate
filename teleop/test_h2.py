import os
import sys
import time
import argparse
import numpy as np
import pinocchio as pin
import logging_mp

logging_mp.basicConfig(level=logging_mp.INFO)

# Setup path and working directory to ensure relative URDF paths work correctly
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
os.chdir(current_dir)

from teleop.robot_control.robot_arm_ik import H2_ArmIK


def main():
    parser = argparse.ArgumentParser(description="H2 Robot IK and Control Test")
    parser.add_argument(
        "--ik-only",
        action="store_true",
        help="Only run IK visually in Meshcat (no DDS/Real Robot)",
    )
    parser.add_argument("--sim", action="store_true", help="Use DDS channel 1 for simulation")
    args = parser.parse_args()

    # We enforce Unit_Test=False so it looks for '../h2_description/H2.urdf' which is correct
    # since we changed cwd to 'teleop/'
    is_unit_test = False

    if args.ik_only:
        print("=" * 60)
        print("🚀 Running in IK-Only Mode (Meshcat Visualization).")
        print("No DDS connection required. Open the URL printed below in your browser!")
        print("=" * 60)
        arm_ik = H2_ArmIK(Unit_Test=is_unit_test, Visualization=True)
        arm_ctrl = None
    else:
        print("=" * 60)
        print("🤖 Running in Full Control Mode (DDS Connection Required).")
        print("=" * 60)
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from teleop.robot_control.robot_arm import H2_ArmController
        except ImportError:
            print("Error: unitree_sdk2py not found in your environment.")
            sys.exit(1)

        ChannelFactoryInitialize(1 if args.sim else 0)
        arm_ik = H2_ArmIK(Unit_Test=is_unit_test, Visualization=False)

        print("\nWaiting for DDS lowstate from robot/simulator...")
        print("If it hangs here, check your robot connection or DDS network settings.")
        arm_ctrl = H2_ArmController(simulation_mode=args.sim)
        print("Controller initialized successfully.")

    time.sleep(1)

    # initial positon for testing
    L_tf_target = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, +0.25, 0.1]))
    R_tf_target = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, -0.25, 0.1]))

    rotation_speed = 0.005
    noise_amplitude_translation = 0.001 if args.ik_only else 0.0
    noise_amplitude_rotation = 0.01 if args.ik_only else 0.0

    if args.ik_only:
        print("\nInitialization complete. Open the Meshcat URL in your browser to see the visualization.")
    else:
        print("\nReady. Ensure robot is clear.")

    user_input = input("Please enter 's' to start moving the arms (Ctrl+C to stop): \n")
    if user_input.lower() == "s":
        step = 0
        if arm_ctrl:
            arm_ctrl.speed_gradual_max()

        try:
            while True:
                # Calculate small oscillatory motions
                rotation_noise_L = pin.Quaternion(
                    np.cos(np.random.normal(0, noise_amplitude_rotation) / 2),
                    0,
                    np.random.normal(0, noise_amplitude_rotation / 2),
                    0,
                ).normalized()
                rotation_noise_R = pin.Quaternion(
                    np.cos(np.random.normal(0, noise_amplitude_rotation) / 2),
                    0,
                    0,
                    np.random.normal(0, noise_amplitude_rotation / 2),
                ).normalized()

                if step <= 120:
                    angle = rotation_speed * step
                    L_quat = rotation_noise_L * pin.Quaternion(np.cos(angle / 2), 0, np.sin(angle / 2), 0)
                    R_quat = rotation_noise_R * pin.Quaternion(np.cos(angle / 2), 0, 0, np.sin(angle / 2))

                    L_tf_target.translation += np.array([0.001, 0.001, 0.001]) + np.random.normal(
                        0, noise_amplitude_translation, 3
                    )
                    R_tf_target.translation += np.array([0.001, -0.001, 0.001]) + np.random.normal(
                        0, noise_amplitude_translation, 3
                    )
                else:
                    angle = rotation_speed * (240 - step)
                    L_quat = rotation_noise_L * pin.Quaternion(np.cos(angle / 2), 0, np.sin(angle / 2), 0)
                    R_quat = rotation_noise_R * pin.Quaternion(np.cos(angle / 2), 0, 0, np.sin(angle / 2))

                    L_tf_target.translation -= np.array([0.001, 0.001, 0.001]) + np.random.normal(
                        0, noise_amplitude_translation, 3
                    )
                    R_tf_target.translation -= np.array([0.001, -0.001, 0.001]) + np.random.normal(
                        0, noise_amplitude_translation, 3
                    )

                L_tf_target.rotation = L_quat.toRotationMatrix()
                R_tf_target.rotation = R_quat.toRotationMatrix()

                # Get current states if controller is active
                if arm_ctrl:
                    current_lr_arm_q = arm_ctrl.get_current_dual_arm_q()
                    current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()
                else:
                    current_lr_arm_q = None
                    current_lr_arm_dq = None

                # Solve IK
                sol_q, sol_tauff = arm_ik.solve_ik(
                    L_tf_target.homogeneous,
                    R_tf_target.homogeneous,
                    current_lr_arm_q,
                    current_lr_arm_dq,
                )

                # Command Robot
                if arm_ctrl:
                    arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

                step += 1
                if step > 240:
                    step = 0
                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            if arm_ctrl:
                print("Sending arms to home position...")
                arm_ctrl.ctrl_dual_arm_go_home()


if __name__ == "__main__":
    main()
