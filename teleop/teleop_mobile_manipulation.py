"""
Mobile Manipulation Teleoperation Script

Enables simultaneous locomotion and manipulation control in Isaac Sim using VR controllers.

Quest Controller Button Mapping (OpenXR standard):
    Left Controller:
        - X button (lower) → left_aButton
        - Y button (upper) → left_bButton
    Right Controller:
        - A button (lower) → right_aButton  
        - B button (upper) → right_bButton

Control Mapping:
    - Left Joystick: Forward/Backward (Y), Strafe Left/Right (X)
    - Right Joystick: Turn Left/Right (X) [Height control disabled]
    - Y Button (Left): Start teleoperation
    - X Button (Left): Toggle recording (start/stop)
    - B Button (Right): Stop base movement (zero velocity, maintain height)
    - A Button (Right): Emergency stop & exit
    - Triggers: Gripper control (Dex1) / Hand control (Dex3 thumb)
"""

import numpy as np
import time
import argparse
import cv2
from multiprocessing import shared_memory, Value, Array, Lock
import threading
import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)

import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController, G1_23_ArmController, H1_2_ArmController, H1_ArmController
from teleop.robot_control.robot_arm_ik import G1_29_ArmIK, G1_23_ArmIK, H1_2_ArmIK, H1_ArmIK
from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller, Dex1_1_Gripper_Controller
from teleop.robot_control.robot_hand_inspire import Inspire_Controller
from teleop.robot_control.robot_hand_brainco import Brainco_Controller
from teleop.image_server.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter

# for simulation locomotion control
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

def publish_reset_category(category: int, publisher): # Scene Reset signal
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")

def publish_locomotion_command(x_vel, y_vel, yaw_vel, height, publisher):
    """Publish locomotion command to Isaac Sim"""
    commands_list = [float(x_vel), float(y_vel), float(yaw_vel), float(height)]
    commands_str = str(commands_list)
    msg = String_(data=commands_str)
    publisher.Write(msg)

# state transition
START              = False  # Enable to start robot following VR user motion  
STOP               = False  # Enable to begin system exit procedure
STOP_BASE_MOVEMENT = False  # True when B button pressed - stops base movement
RECORD_TOGGLE      = False  # [Ready] ⇄ [Recording] ⟶ [AutoSave] ⟶ [Ready]         (⇄ manual) (⟶ auto)
RECORD_RUNNING     = False  # True if [Recording]
RECORD_READY       = True   # True if [Ready], False if [Recording] / [AutoSave]

# Button state tracking for edge detection
prev_left_aButton = False

# Dex3 thumb control constants (for joystick button control)
DEX3_LEFT_LIMITS = {
    "thumb0": (-1.04719755, 1.04719755),
    "thumb1": (-0.72431163, 1.04719755),
    "thumb2": (0.0, 1.74532925),
}
# Left thumb1 target range
THUMB1_MIN_RAD = -30.0 * np.pi / 180.0          # closed
THUMB1_MAX_RAD = 60.0 * np.pi / 180.0  # open
# Right thumb1 target range
R_THUMB1_MIN_RAD = -60.0 * np.pi / 180.0  # open
R_THUMB1_MAX_RAD = 30.0 * np.pi / 180.0        # closed

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency', type = float, default = 30.0, help = 'save data\'s frequency')

    # basic control parameters
    parser.add_argument('--arm', type=str, choices=['G1_29', 'G1_23', 'H1_2', 'H1'], default='G1_29', help='Select arm controller')
    parser.add_argument('--ee', type=str, choices=['dex1', 'dex3', 'inspire1', 'brainco'], help='Select end effector controller')
    # mode flags
    parser.add_argument('--headless', action='store_true', help='Enable headless mode (no display)')
    parser.add_argument('--sim', action='store_true', default=True, help='Enable isaac simulation mode (default for mobile manipulation)')
    parser.add_argument('--record', action='store_true', help='Enable data recording')
    parser.add_argument('--task-dir', type=str, default='./utils/data/', help='path to save data')
    parser.add_argument('--task-name', type=str, default='mobile_manipulation', help='task name for recording')
    parser.add_argument('--task-desc', type=str, default='Mobile manipulation task', help='task goal for recording')

    args = parser.parse_args()
    logger_mp.info(f"args: {args}")
    
    # Force controller mode for mobile manipulation
    args.xr_mode = "controller"
    logger_mp.info("Mobile manipulation mode: using controller tracking")

    try:
        # image client: img_config should be the same as the configuration in image_server.py
        if args.sim:
            img_config = {
                'fps': 30,
                'head_camera_type': 'opencv',
                'head_camera_image_shape': [480, 640],  # Head camera resolution
                'head_camera_id_numbers': [0],
                'wrist_camera_type': 'opencv',
                'wrist_camera_image_shape': [480, 640],  # Wrist camera resolution
                'wrist_camera_id_numbers': [2, 4],
            }
        else:
            img_config = {
                'fps': 30,
                'head_camera_type': 'opencv',
                'head_camera_image_shape': [480, 1280],  # Head camera resolution
                'head_camera_id_numbers': [0],
                'wrist_camera_type': 'opencv',
                'wrist_camera_image_shape': [480, 640],  # Wrist camera resolution
                'wrist_camera_id_numbers': [2, 4],
            }

        ASPECT_RATIO_THRESHOLD = 2.0 # If the aspect ratio exceeds this value, it is considered binocular
        if len(img_config['head_camera_id_numbers']) > 1 or (img_config['head_camera_image_shape'][1] / img_config['head_camera_image_shape'][0] > ASPECT_RATIO_THRESHOLD):
            BINOCULAR = True
        else:
            BINOCULAR = False
        if 'wrist_camera_type' in img_config:
            WRIST = True
        else:
            WRIST = False
        
        if BINOCULAR and not (img_config['head_camera_image_shape'][1] / img_config['head_camera_image_shape'][0] > ASPECT_RATIO_THRESHOLD):
            tv_img_shape = (img_config['head_camera_image_shape'][0], img_config['head_camera_image_shape'][1] * 2, 3)
        else:
            tv_img_shape = (img_config['head_camera_image_shape'][0], img_config['head_camera_image_shape'][1], 3)

        tv_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(tv_img_shape) * np.uint8().itemsize)
        tv_img_array = np.ndarray(tv_img_shape, dtype = np.uint8, buffer = tv_img_shm.buf)

        if WRIST and args.sim:
            wrist_img_shape = (img_config['wrist_camera_image_shape'][0], img_config['wrist_camera_image_shape'][1] * 2, 3)
            wrist_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(wrist_img_shape) * np.uint8().itemsize)
            wrist_img_array = np.ndarray(wrist_img_shape, dtype = np.uint8, buffer = wrist_img_shm.buf)
            img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name, 
                                    wrist_img_shape = wrist_img_shape, wrist_img_shm_name = wrist_img_shm.name, server_address="127.0.0.1")
        elif WRIST and not args.sim:
            wrist_img_shape = (img_config['wrist_camera_image_shape'][0], img_config['wrist_camera_image_shape'][1] * 2, 3)
            wrist_img_shm = shared_memory.SharedMemory(create = True, size = np.prod(wrist_img_shape) * np.uint8().itemsize)
            wrist_img_array = np.ndarray(wrist_img_shape, dtype = np.uint8, buffer = wrist_img_shm.buf)
            img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name, 
                                    wrist_img_shape = wrist_img_shape, wrist_img_shm_name = wrist_img_shm.name)
        else:
            img_client = ImageClient(tv_img_shape = tv_img_shape, tv_img_shm_name = tv_img_shm.name)

        image_receive_thread = threading.Thread(target = img_client.receive_process, daemon = True)
        image_receive_thread.daemon = True
        image_receive_thread.start()

        # television: obtain controller pose data from the XR device and transmit the robot's head camera image to the XR device.
        # Force controller mode (use_hand_tracking=False)
        tv_wrapper = TeleVuerWrapper(binocular=BINOCULAR, use_hand_tracking=False, img_shape=tv_img_shape, img_shm_name=tv_img_shm.name, 
                                    return_state_data=True, return_hand_rot_data=False)

        # arm
        if args.arm == "G1_29":
            arm_ik = G1_29_ArmIK()
            arm_ctrl = G1_29_ArmController(motion_mode=False, simulation_mode=args.sim)
        elif args.arm == "G1_23":
            arm_ik = G1_23_ArmIK()
            arm_ctrl = G1_23_ArmController(motion_mode=False, simulation_mode=args.sim)
        elif args.arm == "H1_2":
            arm_ik = H1_2_ArmIK()
            arm_ctrl = H1_2_ArmController(motion_mode=False, simulation_mode=args.sim)
        elif args.arm == "H1":
            arm_ik = H1_ArmIK()
            arm_ctrl = H1_ArmController(simulation_mode=args.sim)

        # end-effector
        if args.ee == "dex3":
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 14, lock = False)   # [output] current left, right hand state(14) data.
            dual_hand_action_array = Array('d', 14, lock = False)  # [output] current left, right hand action(14) data.
            # Command arrays for trigger control (7 joints: thumb0, thumb1, thumb2, middle0, middle1, index0, index1)
            left_dex3_cmd_q_array = Array('d', 7, lock=True)
            right_dex3_cmd_q_array = Array('d', 7, lock=True)
            # Initialize with default positions
            with left_dex3_cmd_q_array.get_lock():
                left_init = np.array([
                    0.0 * np.pi / 180.0,  # thumb0
                    -30.0 * np.pi / 180.0,                    # thumb1 (controlled by trigger)
                    50.0 * np.pi / 180.0,                    # thumb2
                    -90.0 * np.pi / 180.0,  # middle0
                    -0.0 * np.pi / 180.0,  # middle1
                    -90.0 * np.pi / 180.0,  # index0
                    -0.0 * np.pi / 180.0,  # index1
                ], dtype=float)
                left_dex3_cmd_q_array[:] = left_init
            with right_dex3_cmd_q_array.get_lock():
                right_init = np.array([
                    0.0 * np.pi / 180.0,  # thumb0
                    30.0 * np.pi / 180.0,                    # thumb1 (controlled by trigger)
                    -50.0 * np.pi / 180.0,                    # thumb2
                    90.0 * np.pi / 180.0,   # middle0 (placeholder)
                    0.0 * np.pi / 180.0,   # middle1 (placeholder)
                    90.0 * np.pi / 180.0,   # index0
                    0.0 * np.pi / 180.0,   # index1
                ], dtype=float)
                right_dex3_cmd_q_array[:] = right_init
            hand_ctrl = Dex3_1_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim,
                                          left_cmd_q_in=left_dex3_cmd_q_array, right_cmd_q_in=right_dex3_cmd_q_array)
        elif args.ee == "dex1":
            left_gripper_value = Value('d', 0.0, lock=True)        # [input]
            right_gripper_value = Value('d', 0.0, lock=True)       # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array('d', 2, lock=False)   # current left, right gripper state(2) data.
            dual_gripper_action_array = Array('d', 2, lock=False)  # current left, right gripper action(2) data.
            gripper_ctrl = Dex1_1_Gripper_Controller(left_gripper_value, right_gripper_value, dual_gripper_data_lock, dual_gripper_state_array, dual_gripper_action_array, simulation_mode=args.sim)
        elif args.ee == "inspire1":
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        elif args.ee == "brainco":
            left_hand_pos_array = Array('d', 75, lock = True)      # [input]
            right_hand_pos_array = Array('d', 75, lock = True)     # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array('d', 12, lock = False)   # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array('d', 12, lock = False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller(left_hand_pos_array, right_hand_pos_array, dual_hand_data_lock, dual_hand_state_array, dual_hand_action_array, simulation_mode=args.sim)
        else:
            pass

        # simulation mode - initialize locomotion control
        if args.sim:
            logger_mp.info("Initializing locomotion control for mobile manipulation...")
            ChannelFactoryInitialize(1)
            reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
            reset_pose_publisher.Init()
            locomotion_publisher = ChannelPublisher("rt/run_command/cmd", String_)
            locomotion_publisher.Init()
            
            # Initialize sim state subscriber for recording
            from teleop.utils.sim_state_topic import start_sim_state_subscribe
            sim_state_subscriber = start_sim_state_subscribe()
            
            # Locomotion parameters
            default_height = 0.8
            locomotion_ranges = {
                'x_vel': (-0.6, 0.6),     # forward/backward velocity
                'y_vel': (-0.5, 0.5),     # lateral velocity
                'yaw_vel': (-1.57, 1.57), # yaw velocity
                'height': (-0.5, 0.0)     # height offset from default
            }
            logger_mp.info("Locomotion control initialized")
        
        # record + headless mode
        if args.record and args.headless:
            recorder = EpisodeWriter(task_dir = args.task_dir + args.task_name, task_goal = args.task_desc, frequency = args.frequency, rerun_log = False)
        elif args.record and not args.headless:
            recorder = EpisodeWriter(task_dir = args.task_dir + args.task_name, task_goal = args.task_desc, frequency = args.frequency, rerun_log = True)

        logger_mp.info("=" * 60)
        logger_mp.info("🎮 MOBILE MANIPULATION CONTROLLER MAPPING:")
        logger_mp.info("  Left Joystick:")
        logger_mp.info("    ↕️  Y-axis: Forward/Backward")
        logger_mp.info("    ↔️  X-axis: Strafe Left/Right")
        logger_mp.info("  Right Joystick:")
        logger_mp.info("    ↔️  X-axis: Turn Left/Right")
        logger_mp.info("    ⚠️  Y-axis: Height control DISABLED")
        logger_mp.info("  Buttons (Quest Controller):")
        logger_mp.info("    🟢 Y Button (Left):  Start Running")
        logger_mp.info("    🔵 X Button (Left):  Toggle Recording (Start/Stop)")
        logger_mp.info("    ⏸️  B Button (Right): Stop Base Movement (0 velocity, maintain height)")
        logger_mp.info("    🔴 A Button (Right): Emergency Stop & Exit")
        if args.ee == "dex1":
            logger_mp.info("  🎯 Triggers: Gripper control (Dex1)")
        elif args.ee == "dex3":
            logger_mp.info("  🎯 Triggers: Hand thumb control (Dex3)")
        logger_mp.info("=" * 60)
        logger_mp.info("Please press Y button (left controller) to start")
        
        while not START and not STOP:
            tele_data = tv_wrapper.get_motion_state_data()
            # Y button on left controller to start (Y = upper button = bButton)
            if tele_data.tele_state.left_bButton:  # Quest: Y button = left_bButton
                START = True
                logger_mp.info("🟢 Starting teleoperation...")
            time.sleep(0.01)
            
        logger_mp.info("start program.")
        arm_ctrl.speed_gradual_max()
        
        while not STOP:
            start_time = time.time()

            if not args.headless:
                tv_resized_image = cv2.resize(tv_img_array, (tv_img_shape[1], tv_img_shape[0]))
                cv2.imshow("Mobile Manipulation View", tv_resized_image)
                
                # Display wrist camera if available
                if WRIST:
                    wrist_resized_image = cv2.resize(wrist_img_array, (wrist_img_shape[1], wrist_img_shape[0]))
                    cv2.imshow("Wrist Camera View", wrist_resized_image)
                
                # opencv GUI communication
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    START = False
                    STOP = True
                    if args.sim:
                        publish_reset_category(2, reset_pose_publisher)
                elif key == ord('s'):
                    RECORD_TOGGLE = True
                elif key == ord('a'):
                    if args.sim:
                        publish_reset_category(2, reset_pose_publisher)

            # get input data
            tele_data = tv_wrapper.get_motion_state_data()
            
            # ========== BUTTON CONTROLS ==========
            # A button on right controller: Emergency stop & exit
            if tele_data.tele_state.right_aButton:
                START = False
                STOP = True
                logger_mp.info("🔴 Emergency stop! Exiting...")
            
            # X button on left controller: Toggle recording (X = lower button = aButton)
            # Use edge detection - only trigger on button press, not while held
            if tele_data.tele_state.left_aButton and not prev_left_aButton:
                RECORD_TOGGLE = True
                if RECORD_RUNNING:
                    logger_mp.info("🔵 X button: Stop recording pressed")
                else:
                    logger_mp.info("🔵 X button: Start recording pressed")
            prev_left_aButton = tele_data.tele_state.left_aButton
            
            # B button on right controller: Stop base movement (B = upper button = bButton)
            STOP_BASE_MOVEMENT = tele_data.tele_state.right_bButton

            if args.record and RECORD_TOGGLE:
                RECORD_TOGGLE = False
                if not RECORD_RUNNING:
                    if recorder.create_episode():
                        RECORD_RUNNING = True
                        logger_mp.info("📹 Recording started")
                    else:
                        logger_mp.error("Failed to create episode. Recording not started.")
                else:
                    RECORD_RUNNING = False
                    recorder.save_episode()
                    logger_mp.info("💾 Recording saved & scene reset")
                    if args.sim:
                        publish_reset_category(2, reset_pose_publisher)  # Full scene reset
            
            # ========== GRIPPER/HAND CONTROL ==========
            if args.ee == "dex1":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_trigger_value
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_trigger_value
            elif args.ee == "dex3":
                # Dex3 controller mode - Trigger continuous control
                # Read current command arrays
                with left_dex3_cmd_q_array.get_lock():
                    left_cmd = np.array(left_dex3_cmd_q_array[:])
                with right_dex3_cmd_q_array.get_lock():
                    right_cmd = np.array(right_dex3_cmd_q_array[:])

                # Get trigger values
                # Note: televuer returns trigger_value in range [10.0, 0.0] (inverted)
                # We need to normalize to [0.0, 1.0]
                left_trigger_raw = getattr(tele_data, 'left_trigger_value', 10.0)
                right_trigger_raw = getattr(tele_data, 'right_trigger_value', 10.0)
                
                # Normalize: 10.0 -> 0.0 (not pressed), 0.0 -> 1.0 (fully pressed)
                left_trigger = np.clip((10.0 - left_trigger_raw) / 10.0, 0.0, 1.0)
                right_trigger = np.clip((10.0 - right_trigger_raw) / 10.0, 0.0, 1.0)
                
                # Map trigger values to thumb1 range
                # Left trigger: 0.0 -> THUMB1_MIN_RAD (closed), 1.0 -> THUMB1_MAX_RAD (open)
                left_cmd[1] = THUMB1_MIN_RAD + left_trigger * (THUMB1_MAX_RAD - THUMB1_MIN_RAD)
                
                # Right trigger: 0.0 -> R_THUMB1_MAX_RAD (closed), 1.0 -> R_THUMB1_MIN_RAD (open)
                # (note: right hand has inverted range)
                right_cmd[1] = R_THUMB1_MAX_RAD + right_trigger * (R_THUMB1_MIN_RAD - R_THUMB1_MAX_RAD)

                # Final safety clipping for all joints
                left_cmd[0] = np.clip(left_cmd[0], *DEX3_LEFT_LIMITS["thumb0"])
                left_cmd[1] = np.clip(left_cmd[1], *DEX3_LEFT_LIMITS["thumb1"])
                left_cmd[2] = np.clip(left_cmd[2], *DEX3_LEFT_LIMITS["thumb2"])

                with left_dex3_cmd_q_array.get_lock():
                    left_dex3_cmd_q_array[:] = left_cmd
                with right_dex3_cmd_q_array.get_lock():
                    right_dex3_cmd_q_array[:] = right_cmd
            
            # ========== LOCOMOTION CONTROL ==========
            if args.sim:
                # Get joystick values
                left_joy = tele_data.tele_state.left_thumbstick_value   # [x, y]
                right_joy = tele_data.tele_state.right_thumbstick_value # [x, y]
                
                if not STOP_BASE_MOVEMENT:
                    # Normal operation - joystick control
                    # Left joystick: forward/backward (y) and strafe (x)
                    x_vel = -left_joy[1] * locomotion_ranges['x_vel'][1]  # forward/backward
                    y_vel = -left_joy[0] * locomotion_ranges['y_vel'][1]  # left/right strafe
                    
                    # Right joystick: turn (x) only, height disabled
                    yaw_vel = -right_joy[0] * locomotion_ranges['yaw_vel'][1]  # rotation
                    # height_offset = -right_joy[1] * abs(locomotion_ranges['height'][0])  # height adjustment (DISABLED)
                    
                    # Clamp values to ranges
                    x_vel = np.clip(x_vel, locomotion_ranges['x_vel'][0], locomotion_ranges['x_vel'][1])
                    y_vel = np.clip(y_vel, locomotion_ranges['y_vel'][0], locomotion_ranges['y_vel'][1])
                    yaw_vel = np.clip(yaw_vel, locomotion_ranges['yaw_vel'][0], locomotion_ranges['yaw_vel'][1])
                    # height_offset = np.clip(height_offset, locomotion_ranges['height'][0], locomotion_ranges['height'][1])  # (DISABLED)
                    
                    current_height = default_height  # Fixed height (height control disabled)
                else:
                    # B button pressed - stop base movement, maintain current height
                    x_vel = 0.0
                    y_vel = 0.0
                    yaw_vel = 0.0
                    # Keep current_height from last frame (don't update)
                
                # Publish locomotion command
                publish_locomotion_command(x_vel, y_vel, yaw_vel, current_height, locomotion_publisher)
                
                # Debug output (throttled)
                if int(time.time() * 2) % 5 == 0:  # Every 2.5 seconds
                    logger_mp.debug(f"Locomotion: x={x_vel:.2f}, y={y_vel:.2f}, yaw={yaw_vel:.2f}, h={current_height:.2f}")

            # ========== ARM CONTROL ==========
            # get current robot state data.
            current_lr_arm_q  = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            # solve ik using motor data and wrist pose, then use ik results to control arms.
            time_ik_start = time.time()
            sol_q, sol_tauff  = arm_ik.solve_ik(tele_data.left_arm_pose, tele_data.right_arm_pose, current_lr_arm_q, current_lr_arm_dq)
            time_ik_end = time.time()
            logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)

            # ========== RECORD DATA ==========
            if args.record:
                RECORD_READY = recorder.is_ready()
                # gripper
                if args.ee == "dex1":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        # Include locomotion commands in body state/action
                        current_body_state = [x_vel, y_vel, yaw_vel, current_height]
                        current_body_action = [x_vel, y_vel, yaw_vel, current_height]
                # dex3 hand
                elif args.ee == "dex3":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:7]
                        right_ee_state = dual_hand_state_array[-7:]
                        left_hand_action = dual_hand_action_array[:7]
                        right_hand_action = dual_hand_action_array[-7:]
                        current_body_state = [x_vel, y_vel, yaw_vel, current_height]
                        current_body_action = [x_vel, y_vel, yaw_vel, current_height]
                # inspire/brainco
                elif args.ee == "inspire1" or args.ee == "brainco":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = [x_vel, y_vel, yaw_vel, current_height]
                        current_body_action = [x_vel, y_vel, yaw_vel, current_height]
                else:
                    left_ee_state = []
                    right_ee_state = []
                    left_hand_action = []
                    right_hand_action = []
                    current_body_state = [x_vel, y_vel, yaw_vel, current_height]
                    current_body_action = [x_vel, y_vel, yaw_vel, current_height]
                    
                # head image
                current_tv_image = tv_img_array.copy()
                # wrist image
                if WRIST:
                    current_wrist_image = wrist_img_array.copy()
                # arm state and action
                left_arm_state  = current_lr_arm_q[:7]
                right_arm_state = current_lr_arm_q[-7:]
                left_arm_action = sol_q[:7]
                right_arm_action = sol_q[-7:]
                
                if RECORD_RUNNING:
                    colors = {}
                    depths = {}
                    if BINOCULAR:
                        colors[f"color_{0}"] = current_tv_image[:, :tv_img_shape[1]//2]
                        colors[f"color_{1}"] = current_tv_image[:, tv_img_shape[1]//2:]
                        if WRIST:
                            colors[f"color_{2}"] = current_wrist_image[:, :wrist_img_shape[1]//2]
                            colors[f"color_{3}"] = current_wrist_image[:, wrist_img_shape[1]//2:]
                    else:
                        colors[f"color_{0}"] = current_tv_image
                        if WRIST:
                            colors[f"color_{1}"] = current_wrist_image[:, :wrist_img_shape[1]//2]
                            colors[f"color_{2}"] = current_wrist_image[:, wrist_img_shape[1]//2:]
                    
                    states = {
                        "left_arm": {                                                                    
                            "qpos":   left_arm_state.tolist(),
                            "qvel":   [],                          
                            "torque": [],                        
                        }, 
                        "right_arm": {                                                                    
                            "qpos":   right_arm_state.tolist(),       
                            "qvel":   [],                          
                            "torque": [],                         
                        },                        
                        "left_ee": {                                                                    
                            "qpos":   left_ee_state,           
                            "qvel":   [],                           
                            "torque": [],                          
                        }, 
                        "right_ee": {                                                                    
                            "qpos":   right_ee_state,       
                            "qvel":   [],                           
                            "torque": [],  
                        }, 
                        "body": {
                            "qpos": current_body_state,  # [x_vel, y_vel, yaw_vel, height]
                        }, 
                    }
                    actions = {
                        "left_arm": {                                   
                            "qpos":   left_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],      
                        }, 
                        "right_arm": {                                   
                            "qpos":   right_arm_action.tolist(),       
                            "qvel":   [],       
                            "torque": [],       
                        },                         
                        "left_ee": {                                   
                            "qpos":   left_hand_action,       
                            "qvel":   [],       
                            "torque": [],       
                        }, 
                        "right_ee": {                                   
                            "qpos":   right_hand_action,       
                            "qvel":   [],       
                            "torque": [], 
                        }, 
                        "body": {
                            "qpos": current_body_action,  # [x_vel, y_vel, yaw_vel, height]
                        }, 
                    }
                    
                    # Read sim_state for replay compatibility
                    sim_state = sim_state_subscriber.read_data() if args.sim else None
                    recorder.add_item(colors=colors, depths=depths, states=states, actions=actions, sim_state=sim_state)

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("KeyboardInterrupt, exiting program...")
    finally:
        # Stop locomotion before exiting
        if args.sim:
            publish_locomotion_command(0.0, 0.0, 0.0, default_height, locomotion_publisher)
            logger_mp.info("Locomotion stopped")
            # Stop sim state subscriber
            if 'sim_state_subscriber' in locals():
                sim_state_subscriber.stop_subscribe()
                logger_mp.info("Sim state subscriber stopped")
        
        arm_ctrl.ctrl_dual_arm_go_home()

        tv_img_shm.close()
        tv_img_shm.unlink()
        if WRIST:
            wrist_img_shm.close()
            wrist_img_shm.unlink()

        if args.record:
            recorder.close()
        logger_mp.info("Finally, exiting program.")
        exit(0)

