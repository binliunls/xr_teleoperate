import time
import argparse
from multiprocessing import Value, Array, Lock
import threading
import logging_mp

logging_mp.basicConfig(level=logging_mp.INFO)
logger_mp = logging_mp.getLogger(__name__)

import os
import sys
import numpy as np
import pinocchio as pin

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # dds
from televuer import TeleVuerWrapper
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
from teleimager.image_client import ImageClient
from teleop.utils.episode_writer import EpisodeWriter
from teleop.utils.ipc import IPC_Server
from teleop.utils.motion_switcher import MotionSwitcher, LocoClientWrapper
from teleop.utils.discover_h2_dds_peer import (
    DEFAULT_THOR_SSH_TARGET,
    H2PeerDiscoveryError,
    discover_h2_body_peers_via_ssh,
)
from teleop.utils.unitree_dds_network import (
    H2_BODY_DDS_ADDRESSES,
    THOR_DDS_ADDRESS,
    WORKSTATION_DDS_ADDRESS,
    WORKSTATION_ROBOT_DDS_ADDRESS,
    build_address_config,
)
from teleop.utils.sharpa_native_capture import (
    DEFAULT_CONTROL_ADDRESS as SHARPA_CAPTURE_DEFAULT_ADDRESS,
    PROTOCOL as SHARPA_CAPTURE_PROTOCOL,
    CaptureControlError,
    SharpaNativeCaptureClient,
    resolve_tactile_host,
    write_capture_metadata,
)
from sshkeyboard import listen_keyboard, stop_listening

# for simulation
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_


def publish_reset_category(category: int, publisher):  # Scene Reset signal
    msg = String_(data=str(category))
    publisher.Write(msg)
    logger_mp.info(f"published reset category: {category}")


# state transition
START = False  # Enable to start robot following VR user motion
STOP = False  # Enable to begin system exit procedure
READY = False  # Ready to (1) enter START state, (2) enter RECORD_RUNNING state
RECORD_RUNNING = False  # True if [Recording]
RECORD_TOGGLE = False  # Toggle recording state
CALIBRATE = False
CALIBRATED = False
REF_LEFT_WRIST_POSE = None
REF_RIGHT_WRIST_POSE = None
INIT_LEFT_TARGET_POSE = None
INIT_RIGHT_TARGET_POSE = None
#  -------        ---------                -----------                -----------            ---------
#   state          [Ready]      ==>        [Recording]     ==>         [AutoSave]     -->     [Ready]
#  -------        ---------      |         -----------      |         -----------      |     ---------
#   START           True         |manual      True          |manual      True          |        True
#   READY           True         |set         False         |set         False         |auto    True
#   RECORD_RUNNING  False        |to          True          |to          False         |        False
#                                ∨                          ∨                          ∨
#   RECORD_TOGGLE   False       True          False        True          False                  False
#  -------        ---------                -----------                 -----------            ---------
#  ==> manual: when READY is True, set RECORD_TOGGLE=True to transition.
#  --> auto  : Auto-transition after saving data.


def on_press(key):
    global STOP, START, RECORD_TOGGLE, CALIBRATE
    if key == "r":
        START = True
    elif key == "c":
        CALIBRATE = True
    elif key == "q":
        START = False
        STOP = True
    elif key == "s" and START:
        RECORD_TOGGLE = True
    else:
        logger_mp.warning(f"[on_press] {key} was pressed, but no action is defined for this key.")


def _compute_relative_target_pose(current_pose, ref_pose, init_pose):
    current_pose = np.asarray(current_pose)
    ref_pose = np.asarray(ref_pose)
    init_pose = np.asarray(init_pose)

    delta_pos = current_pose[:3, 3] - ref_pose[:3, 3]
    rel_rot = current_pose[:3, :3] @ ref_pose[:3, :3].T

    target = np.eye(4, dtype=current_pose.dtype)
    target[:3, 3] = init_pose[:3, 3] + delta_pos
    target[:3, :3] = rel_rot @ init_pose[:3, :3]
    return target


def _get_ik_translation_scale(arm_ik) -> float:
    human_arm_length = getattr(arm_ik, "human_arm_length", None)
    robot_arm_length = getattr(arm_ik, "robot_arm_length", None)
    if human_arm_length is not None and robot_arm_length is not None:
        human_arm_length = float(human_arm_length)
        robot_arm_length = float(robot_arm_length)
        if human_arm_length > 0:
            return robot_arm_length / human_arm_length

    if isinstance(arm_ik, (H1_2_ArmIK, H1_ArmIK)):
        return 0.75 / 0.60

    return 1.0


def _try_calibrate_from_teleop(tele_data, arm_ctrl, arm_ik) -> bool:
    global CALIBRATED, REF_LEFT_WRIST_POSE, REF_RIGHT_WRIST_POSE, INIT_LEFT_TARGET_POSE, INIT_RIGHT_TARGET_POSE

    left_wrist_pose = getattr(tele_data, "left_wrist_pose", None)
    right_wrist_pose = getattr(tele_data, "right_wrist_pose", None)
    if left_wrist_pose is None or right_wrist_pose is None:
        return False

    left_wrist_pose = np.asarray(left_wrist_pose)
    right_wrist_pose = np.asarray(right_wrist_pose)
    if left_wrist_pose.shape != (4, 4) or right_wrist_pose.shape != (4, 4):
        return False

    if not hasattr(arm_ik, "reduced_robot") or not hasattr(arm_ik, "L_hand_id") or not hasattr(arm_ik, "R_hand_id"):
        return False

    current_lr_arm_q = arm_ctrl.get_current_dual_arm_q()
    pin.forwardKinematics(arm_ik.reduced_robot.model, arm_ik.reduced_robot.data, current_lr_arm_q)
    pin.updateFramePlacements(arm_ik.reduced_robot.model, arm_ik.reduced_robot.data)

    left_ee_se3 = arm_ik.reduced_robot.data.oMf[arm_ik.L_hand_id]
    right_ee_se3 = arm_ik.reduced_robot.data.oMf[arm_ik.R_hand_id]

    translation_scale = _get_ik_translation_scale(arm_ik)
    if translation_scale <= 0:
        return False

    REF_LEFT_WRIST_POSE = left_wrist_pose.copy()
    REF_RIGHT_WRIST_POSE = right_wrist_pose.copy()
    INIT_LEFT_TARGET_POSE = left_ee_se3.homogeneous.copy()
    INIT_RIGHT_TARGET_POSE = right_ee_se3.homogeneous.copy()
    INIT_LEFT_TARGET_POSE[:3, 3] /= translation_scale
    INIT_RIGHT_TARGET_POSE[:3, 3] /= translation_scale
    CALIBRATED = True
    return True


def get_state() -> dict:
    """Return current heartbeat state"""
    global START, STOP, RECORD_RUNNING, READY
    return {
        "START": START,
        "STOP": STOP,
        "READY": READY,
        "RECORD_RUNNING": RECORD_RUNNING,
    }


def _frame_camera_timestamps(frame, fallback_name: str) -> dict:
    """Extract real source/receive timestamps without inventing unavailable ones."""
    if frame is None:
        return {}
    camera_timestamps = getattr(frame, "camera_timestamps", None)
    if isinstance(camera_timestamps, dict) and camera_timestamps:
        return {
            str(name): {key: value for key, value in values.items() if value is not None}
            for name, values in camera_timestamps.items()
            if isinstance(values, dict) and any(value is not None for value in values.values())
        }

    receive_monotonic_ns = getattr(frame, "workstation_receive_monotonic_ns", None)
    receive_realtime_ns = getattr(frame, "workstation_receive_realtime_ns", None)
    if receive_monotonic_ns is None and receive_realtime_ns is None:
        return {}
    timestamps = {}
    if receive_monotonic_ns is not None:
        timestamps["workstation_receive_monotonic_ns"] = int(receive_monotonic_ns)
    if receive_realtime_ns is not None:
        timestamps["workstation_receive_realtime_ns"] = int(receive_realtime_ns)
    return {fallback_name: timestamps}


def _episode_capture_metadata(metadata: dict, episode_dir: str, task_name: str) -> dict:
    result = dict(metadata)
    result["workstation_episode"] = {
        "task_name": task_name,
        "episode_dir": os.path.basename(episode_dir),
    }
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # basic control parameters
    parser.add_argument("--frequency", type=float, default=30.0, help="control and record 's frequency")
    parser.add_argument(
        "--input-mode",
        type=str,
        choices=["hand", "controller"],
        default="controller",
        help="Select XR device input tracking source",
    )
    parser.add_argument(
        "--display-mode",
        type=str,
        choices=["immersive", "ego", "pass-through"],
        default="immersive",
        help="Select XR device display mode",
    )
    parser.add_argument(
        "--arm",
        type=str,
        choices=["G1_29", "G1_23", "H1_2", "H1", "H2"],
        default="H2",
        help="Select arm controller",
    )
    parser.add_argument(
        "--arm-side",
        type=str,
        choices=["left", "right", "both"],
        default="both",
        help="Which arm(s) to teleoperate. The disabled arm is pinned at its "
        "calibration initial pose so it stays mechanically static (useful for "
        "single-arm data collection). Default: both arms active.",
    )
    parser.add_argument(
        "--ee",
        type=str,
        default="sharpa",
        choices=["dex1", "dex3", "inspire_ftp", "inspire_dfx", "brainco", "sharpa"],
        help="Select end effector controller",
    )
    parser.add_argument(
        "--sharpa-dds-domain",
        type=int,
        default=0,
        help="DDS domain ID for Sharpa hand state — must match the publisher "
        "(bridge on Thor or retargeting daemon). Default: 0.",
    )
    parser.add_argument(
        "--sharpa-tactile-host",
        type=str,
        default=THOR_DDS_ADDRESS,
        help="Sharpa 30 Hz compatibility tactile host "
        f"(default: {THOR_DDS_ADDRESS}).",
    )
    parser.add_argument(
        "--sharpa-tactile-port",
        type=int,
        default=7779,
        help="Sharpa tactile ZMQ port — must match the bridge's --tactile-port.",
    )
    parser.add_argument(
        "--sharpa-native-capture",
        nargs="?",
        const=SHARPA_CAPTURE_DEFAULT_ADDRESS,
        default=None,
        metavar="CONTROL_ADDRESS",
        help="Enable Thor-local native-rate Sharpa capture. With no address, uses "
        f"{SHARPA_CAPTURE_DEFAULT_ADDRESS}. Only low-bandwidth START/SAMPLE/STOP "
        "markers cross the workstation link; 180 Hz tactile stays on Thor.",
    )
    parser.add_argument(
        "--sharpa-native-capture-timeout",
        type=float,
        default=0.35,
        metavar="SECONDS",
        help="Maximum START/SAMPLE control reply time (default: 0.35 s).",
    )
    parser.add_argument(
        "--sharpa-native-capture-stop-timeout",
        type=float,
        default=30.0,
        metavar="SECONDS",
        help="Thor STOP/finalize timeout on the background worker (default: 30 s).",
    )
    parser.add_argument(
        "--img-server-ip",
        type=str,
        default="192.168.124.162",
        help="IP address of image server, used by teleimager and televuer",
    )
    parser.add_argument(
        "--camera-source",
        type=str,
        choices=["zmq", "ros"],
        default="ros",
        help="Camera source: 'zmq' (teleimager image server) or 'ros' "
        "(rclpy subscriber on /head/{left,right}/image_raw and /wrist/{left,right}/image_raw). "
        "Default: zmq.",
    )
    parser.add_argument(
        "--head-mode",
        type=str,
        choices=["binocular", "mono"],
        default="mono",
        help="Head camera mode (ros source only). 'binocular': subscribe to both "
        "/head/left and /head/right and stitch side-by-side. 'mono' (default): use "
        "only /head/left; recording emits color_0=head_left, color_1=wrist_left, "
        "color_2=wrist_right, matching Thor's run_all_3cam.sh publisher.",
    )
    parser.add_argument(
        "--network-interface",
        type=str,
        default=None,
        help="Explicit Unitree DDS interface-name override. By default, exact "
        "--network-address/--network-peer IPs are used.",
    )
    parser.add_argument(
        "--network-address",
        type=str,
        default=WORKSTATION_ROBOT_DDS_ADDRESS,
        help="Exact workstation Unitree robot DDS address "
        f"(default: {WORKSTATION_ROBOT_DDS_ADDRESS}).",
    )
    parser.add_argument(
        "--network-peer",
        type=str,
        default=THOR_DDS_ADDRESS,
        help=f"Thor Sharpa DDS discovery peer (default: {THOR_DDS_ADDRESS}).",
    )
    parser.add_argument(
        "--body-dds-peer",
        action="append",
        default=None,
        help="H2 body DDS discovery peer. Repeat for multiple body endpoints; "
        "accepts IP or IP:PORT. When omitted for a physical H2, the current "
        "basic_service and humanoid ports are discovered through Thor. Explicit "
        "values bypass auto-discovery.",
    )
    parser.add_argument(
        "--body-dds-discovery-ssh",
        type=str,
        default=DEFAULT_THOR_SSH_TARGET,
        metavar="[USER@]HOST",
        help="Thor SSH target used only for automatic H2 DDS port discovery "
        f"(default: {DEFAULT_THOR_SSH_TARGET}).",
    )
    parser.add_argument(
        "--body-dds-discovery-timeout",
        type=float,
        default=35.0,
        metavar="SECONDS",
        help="Maximum time to wait for both H2 SPDP announcements (default: 35).",
    )
    parser.add_argument(
        "--camera-network-address",
        type=str,
        default=WORKSTATION_DDS_ADDRESS,
        help=f"Exact workstation ROS camera DDS address (default: {WORKSTATION_DDS_ADDRESS}).",
    )
    parser.add_argument(
        "--camera-network-peer",
        type=str,
        default=THOR_DDS_ADDRESS,
        help=f"Thor ROS camera DDS peer (default: {THOR_DDS_ADDRESS}).",
    )
    parser.add_argument(
        "--human-arm-length",
        type=float,
        default=0.60,
        metavar="M",
        help="Human arm length used for wrist pose scaling in IK (meters).",
    )
    # mode flags
    parser.add_argument("--motion", action="store_true", help="Enable motion control mode")
    parser.add_argument(
        "--head-pitch-home",
        type=float,
        default=0.6,
        metavar="RAD",
        help="H2 head pitch at home/rest position in radians (default: 0.3, looking slightly down). "
        "Range: -0.523 (up) to 0.837 (down).",
    )
    parser.add_argument("--headless", action="store_true", help="Enable headless mode (no display)")
    parser.add_argument("--sim", action="store_true", help="Enable isaac simulation mode")
    parser.add_argument(
        "--ipc",
        action="store_true",
        help="Enable IPC server to handle input; otherwise enable sshkeyboard",
    )
    parser.add_argument(
        "--affinity",
        action="store_true",
        help="Enable high priority and set CPU affinity mode",
    )
    # record mode and task info
    parser.add_argument("--record", action="store_true", help="Enable data recording mode")
    # H2 only: include the 3 waist joints (yaw, roll, pitch) in body state/action.
    # On by default — pass --no-record-waist to suppress. Waist is read-only on the
    # current hardware; the recorded "action" mirrors state (same pattern as sharpa ee).
    _waist_group = parser.add_mutually_exclusive_group()
    _waist_group.add_argument(
        "--record-waist",
        dest="record_waist",
        action="store_true",
        default=True,
        help="H2 only: record waist (yaw/roll/pitch) under body.qpos for state and action. Default: on.",
    )
    _waist_group.add_argument(
        "--no-record-waist",
        dest="record_waist",
        action="store_false",
        help="Disable waist recording for H2.",
    )
    parser.add_argument("--task-dir", type=str, default="./utils/data/", help="path to save data")
    parser.add_argument(
        "--task-name",
        type=str,
        default="assemble_trocar",
        help="task file name for recording",
    )
    parser.add_argument(
        "--task-goal",
        type=str,
        default="assemble_trocar.",
        help="task goal for recording at json file",
    )
    parser.add_argument(
        "--task-desc",
        type=str,
        default="assemble_the_trocar_and_place_it_on_the_table.",
        help="task description for recording at json file",
    )
    parser.add_argument(
        "--task-steps",
        type=str,
        default="step1: pick left part, step2: pick right part, step3: assemble the trocar, step4: place the trocar on the table.",
        help="task steps for recording at json file",
    )

    args = parser.parse_args()
    if args.body_dds_peer is None:
        args.body_dds_peer = []
        if args.arm == "H2" and not args.sim and not args.network_interface:
            if args.body_dds_discovery_timeout <= 0:
                parser.error("--body-dds-discovery-timeout must be positive")
            logger_mp.info(
                "[DDS] discovering current H2 basic_service and humanoid ports "
                f"through {args.body_dds_discovery_ssh}..."
            )
            try:
                args.body_dds_peer = discover_h2_body_peers_via_ssh(
                    ssh_target=args.body_dds_discovery_ssh,
                    source_ip=H2_BODY_DDS_ADDRESSES[0],
                    timeout_s=args.body_dds_discovery_timeout,
                )
            except H2PeerDiscoveryError as exc:
                parser.error(
                    "automatic H2 DDS peer discovery failed; robot control was not "
                    f"started: {exc}. Restore Thor/H2 connectivity or pass explicit "
                    "--body-dds-peer IP:PORT overrides for both participants"
                )
            logger_mp.info(
                "[DDS] discovered H2 peers: " + ", ".join(args.body_dds_peer)
            )
    if args.sharpa_native_capture is not None and (not args.record or args.ee != "sharpa"):
        parser.error("--sharpa-native-capture requires --record --ee sharpa")
    if args.sharpa_native_capture_timeout <= 0:
        parser.error("--sharpa-native-capture-timeout must be positive")
    if args.sharpa_native_capture_stop_timeout <= 0:
        parser.error("--sharpa-native-capture-stop-timeout must be positive")
    try:
        args.sharpa_tactile_host = resolve_tactile_host(
            args.sharpa_tactile_host,
            args.sharpa_native_capture,
        )
    except ValueError as exc:
        parser.error(str(exc))
    logger_mp.info(f"args: {args}")

    native_capture_client = None
    active_native_capture_id = None
    active_native_episode_dir = None
    pending_native_episode_dir = None
    pending_native_metadata = None

    try:
        # setup dds communication domains id
        if args.sim:
            ChannelFactoryInitialize(1, networkInterface=args.network_interface)
        elif args.network_interface:
            ChannelFactoryInitialize(0, networkInterface=args.network_interface)
        else:
            unitree_dds_peers = [args.network_peer, *args.body_dds_peer]
            ChannelFactoryInitialize(
                0,
                networkConfig=build_address_config(
                    args.network_address,
                    unitree_dds_peers,
                    allow_multicast="spdp",
                ),
            )
            logger_mp.info(
                "[DDS] exact address mapping: "
                f"workstation={args.network_address} peers={unitree_dds_peers} "
                "multicast=spdp domain=0"
            )

        # ipc communication mode. client usage: see utils/ipc.py
        if args.ipc:
            ipc_server = IPC_Server(on_press=on_press, get_state=get_state)
            ipc_server.start()
        # sshkeyboard communication mode
        else:
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

        # image client — pick ZMQ (teleimager) or ROS 2 source
        if args.camera_source == "ros":
            from teleop.utils.ros_image_client import ROSImageClient

            img_client = ROSImageClient(
                head_mode=args.head_mode,
                cyclonedds_uri=build_address_config(
                    args.camera_network_address,
                    args.camera_network_peer,
                    max_message_size_bytes=1400,
                ),
            )
            logger_mp.info(f"[camera] using ROS 2 image source (head_mode={args.head_mode})")
        else:
            img_client = ImageClient(host=args.img_server_ip, request_bgr=True)
            logger_mp.info(f"[camera] using ZMQ image source ({args.img_server_ip})")
        camera_config = img_client.get_cam_config()
        logger_mp.debug(f"Camera config: {camera_config}")
        # H2 uses pass-through display so never needs to push images to the XR headset,
        # but still captures images for recording when the server has zmq enabled.
        xr_need_local_img = args.arm != "H2" and not (
            args.display_mode == "pass-through" or camera_config["head_camera"]["enable_webrtc"]
        )

        # televuer_wrapper: obtain hand pose data from the XR device and transmit the robot's head camera image to the XR device.
        tv_wrapper = TeleVuerWrapper(
            use_hand_tracking=args.input_mode == "hand",
            binocular=camera_config["head_camera"]["binocular"],
            img_shape=camera_config["head_camera"]["image_shape"],
            # maybe should decrease fps for better performance?
            # https://github.com/unitreerobotics/xr_teleoperate/issues/172
            # display_fps=camera_config['head_camera']['fps'] ? args.frequency? 30.0?
            display_mode="pass-through" if args.arm == "H2" else args.display_mode,
            zmq=camera_config["head_camera"]["enable_zmq"],
            webrtc=camera_config["head_camera"]["enable_webrtc"],
            webrtc_url=f"https://{args.img_server_ip}:{camera_config['head_camera']['webrtc_port']}/offer",
        )

        # motion mode (G1: Regular mode R1+X, not Running mode R2+A)
        if args.motion:
            if args.input_mode == "controller":
                loco_wrapper = LocoClientWrapper()
        else:
            motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Enter_Debug_Mode()
            logger_mp.info(f"Enter debug mode: {'Success' if status == 0 else 'Failed'}")

        # arm
        if args.arm == "G1_29":
            arm_ik = G1_29_ArmIK()
            arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "G1_23":
            arm_ik = G1_23_ArmIK()
            arm_ctrl = G1_23_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1_2":
            arm_ik = H1_2_ArmIK()
            arm_ctrl = H1_2_ArmController(motion_mode=args.motion, simulation_mode=args.sim)
        elif args.arm == "H1":
            arm_ik = H1_ArmIK()
            arm_ctrl = H1_ArmController(simulation_mode=args.sim)
        elif args.arm == "H2":
            arm_ik = H2_ArmIK()
            arm_ctrl = H2_ArmController(
                motion_mode=args.motion, simulation_mode=args.sim, head_pitch_home=args.head_pitch_home
            )

        # end-effector
        if args.ee == "dex3":
            from teleop.robot_control.robot_hand_unitree import Dex3_1_Controller

            left_hand_pos_array = Array("d", 75, lock=True)  # [input]
            right_hand_pos_array = Array("d", 75, lock=True)  # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array("d", 14, lock=False)  # [output] current left, right hand state(14) data.
            dual_hand_action_array = Array("d", 14, lock=False)  # [output] current left, right hand action(14) data.
            hand_ctrl = Dex3_1_Controller(
                left_hand_pos_array,
                right_hand_pos_array,
                dual_hand_data_lock,
                dual_hand_state_array,
                dual_hand_action_array,
                simulation_mode=args.sim,
            )
        elif args.ee == "dex1":
            from teleop.robot_control.robot_hand_unitree import (
                Dex1_1_Gripper_Controller,
            )

            left_gripper_value = Value("d", 0.0, lock=True)  # [input]
            right_gripper_value = Value("d", 0.0, lock=True)  # [input]
            dual_gripper_data_lock = Lock()
            dual_gripper_state_array = Array("d", 2, lock=False)  # current left, right gripper state(2) data.
            dual_gripper_action_array = Array("d", 2, lock=False)  # current left, right gripper action(2) data.
            gripper_ctrl = Dex1_1_Gripper_Controller(
                left_gripper_value,
                right_gripper_value,
                dual_gripper_data_lock,
                dual_gripper_state_array,
                dual_gripper_action_array,
                simulation_mode=args.sim,
            )
        elif args.ee == "inspire_dfx":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_DFX

            left_hand_pos_array = Array("d", 75, lock=True)  # [input]
            right_hand_pos_array = Array("d", 75, lock=True)  # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array("d", 12, lock=False)  # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array("d", 12, lock=False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_DFX(
                left_hand_pos_array,
                right_hand_pos_array,
                dual_hand_data_lock,
                dual_hand_state_array,
                dual_hand_action_array,
                simulation_mode=args.sim,
            )
        elif args.ee == "inspire_ftp":
            from teleop.robot_control.robot_hand_inspire import Inspire_Controller_FTP

            left_hand_pos_array = Array("d", 75, lock=True)  # [input]
            right_hand_pos_array = Array("d", 75, lock=True)  # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array("d", 12, lock=False)  # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array("d", 12, lock=False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Inspire_Controller_FTP(
                left_hand_pos_array,
                right_hand_pos_array,
                dual_hand_data_lock,
                dual_hand_state_array,
                dual_hand_action_array,
                simulation_mode=args.sim,
            )
        elif args.ee == "brainco":
            from teleop.robot_control.robot_hand_brainco import Brainco_Controller

            left_hand_pos_array = Array("d", 75, lock=True)  # [input]
            right_hand_pos_array = Array("d", 75, lock=True)  # [input]
            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array("d", 12, lock=False)  # [output] current left, right hand state(12) data.
            dual_hand_action_array = Array("d", 12, lock=False)  # [output] current left, right hand action(12) data.
            hand_ctrl = Brainco_Controller(
                left_hand_pos_array,
                right_hand_pos_array,
                dual_hand_data_lock,
                dual_hand_state_array,
                dual_hand_action_array,
                simulation_mode=args.sim,
            )
        elif args.ee == "sharpa":
            from teleop.robot_control.robot_hand_sharpa import (
                SharpaWave_Controller,
                SharpaTactile_Subscriber,
                SHARPA_DOF,
            )
            from teleop.utils.sharpa_tactile_wire import FINGER_NAMES as SHARPA_FINGER_NAMES

            dual_hand_data_lock = Lock()
            dual_hand_state_array = Array("d", 2 * SHARPA_DOF, lock=False)
            dual_hand_action_array = Array("d", 2 * SHARPA_DOF, lock=False)
            hand_ctrl = SharpaWave_Controller(
                dual_hand_data_lock=dual_hand_data_lock,
                dual_hand_state_array=dual_hand_state_array,
                dual_hand_action_array=dual_hand_action_array,
                fps=args.frequency,
                dds_domain=args.sharpa_dds_domain,
            )
        else:
            pass

        # affinity mode (if you dont know what it is, then you probably don't need it)
        if args.affinity:
            import psutil

            p = psutil.Process(os.getpid())
            p.cpu_affinity([0, 1, 2, 3])  # Set CPU affinity to cores 0-3
            try:
                p.nice(-20)  # Set highest priority
                logger_mp.info("Set high priority successfully.")
            except psutil.AccessDenied:
                logger_mp.warning("Failed to set high priority. Please run as root.")

            for child in p.children(recursive=True):
                try:
                    logger_mp.info(f"Child process {child.pid} name: {child.name()}")
                    child.cpu_affinity([5, 6])
                    child.nice(-20)
                except psutil.AccessDenied:
                    pass

        # simulation mode
        if args.sim:
            reset_pose_publisher = ChannelPublisher("rt/reset_pose/cmd", String_)
            reset_pose_publisher.Init()
            from teleop.utils.sim_state_topic import start_sim_state_subscribe

            sim_state_subscriber = start_sim_state_subscribe()

        # record + headless / non-headless mode
        if args.record:
            recorder = EpisodeWriter(
                task_dir=os.path.join(args.task_dir, args.task_name),
                task_goal=args.task_goal,
                task_desc=args.task_desc,
                task_steps=args.task_steps,
                frequency=args.frequency,
                rerun_log=not args.headless,
            )
            recorder.info["clock_domains"] = {
                "workstation_monotonic_ns": "Workstation monotonic clock; comparable only within this boot.",
                "workstation_realtime_ns": "Workstation realtime clock; may be adjusted by time synchronization.",
                "ros_header_stamp_ns": "Thor ROS publisher stamp; not assumed to be hardware exposure time or workstation-aligned.",
                "unitree_tick": "Opaque H2 LowState counter; no timestamp unit or epoch is assumed.",
                "sharpa_tactile_ts": "Sharpa SDK source value stored as tactile.ts; not assumed workstation-aligned.",
                "sharpa_hand_desired_command": "Latest rt/sharpa/{side}/cmd observed by this recorder; not proof of Thor application.",
            }

            # End-effector joint / tactile metadata
            if args.ee == "sharpa":
                _sharpa_left_joint_names = [
                    "left_thumb_CMC_FE",
                    "left_thumb_CMC_AA",
                    "left_thumb_MCP_FE",
                    "left_thumb_MCP_AA",
                    "left_thumb_IP",
                    "left_index_MCP_FE",
                    "left_index_MCP_AA",
                    "left_index_PIP",
                    "left_index_DIP",
                    "left_middle_MCP_FE",
                    "left_middle_MCP_AA",
                    "left_middle_PIP",
                    "left_middle_DIP",
                    "left_ring_MCP_FE",
                    "left_ring_MCP_AA",
                    "left_ring_PIP",
                    "left_ring_DIP",
                    "left_pinky_CMC",
                    "left_pinky_MCP_FE",
                    "left_pinky_MCP_AA",
                    "left_pinky_PIP",
                    "left_pinky_DIP",
                ]
                _sharpa_right_joint_names = [n.replace("left_", "right_") for n in _sharpa_left_joint_names]
                _body_joint_names = None
                if args.arm == "H2" and args.record_waist:
                    _body_joint_names = list(H2_ArmController.H2_WAIST_JOINT_NAMES)
                recorder.set_ee_metadata(
                    left_joint_names=_sharpa_left_joint_names,
                    right_joint_names=_sharpa_right_joint_names,
                    left_tactile_names=list(SHARPA_FINGER_NAMES),
                    right_tactile_names=list(SHARPA_FINGER_NAMES),
                    body_joint_names=_body_joint_names,
                )
                # Keep the 30 Hz compatibility stream in every mode so inline
                # tactile PNG/F6/contact data and existing consumers stay
                # unchanged. Native capture is an additional Thor-local stream.
                tactile_sub = SharpaTactile_Subscriber(
                    host=args.sharpa_tactile_host,
                    port=args.sharpa_tactile_port,
                )
                if args.sharpa_native_capture is not None:
                    native_capture_client = SharpaNativeCaptureClient(
                        address=args.sharpa_native_capture,
                        request_timeout_s=args.sharpa_native_capture_timeout,
                        stop_timeout_s=args.sharpa_native_capture_stop_timeout,
                    )
                    logger_mp.info(
                        "[SharpaCapture] native Thor capture control enabled at "
                        f"{args.sharpa_native_capture}"
                    )
            else:
                tactile_sub = None
        else:
            tactile_sub = None

        logger_mp.info("----------------------------------------------------------------")
        logger_mp.info("�  Press [c] on keyboard to calibrate (recommended before starting).")
        logger_mp.info("�  Press [r] on keyboard or [B button] on right controller to start syncing.")
        if args.record:
            logger_mp.info(
                "🟡  Press [s] on keyboard or [Y button] on right controller to START or SAVE recording (toggle cycle)."
            )
        else:
            logger_mp.info("🔵  Recording is DISABLED (run with --record to enable).")
        logger_mp.info("🔴  Press [q] on keyboard or [A button] on right controller to stop and exit.")
        logger_mp.info("⚠️  IMPORTANT: Please keep your distance and stay safe.")
        READY = True  # now ready to (1) enter START state
        while not STOP:
            time.sleep(0.033)

            tele_data = None
            if args.input_mode == "controller":
                tele_data = tv_wrapper.get_tele_data()
                if tele_data.right_ctrl_bButton:
                    START = True
                elif tele_data.right_ctrl_aButton:
                    START = False
                    STOP = True

            if STOP:
                break

            if CALIBRATE:
                if tele_data is None:
                    tele_data = tv_wrapper.get_tele_data()
                if _try_calibrate_from_teleop(tele_data, arm_ctrl, arm_ik):
                    logger_mp.info("Calibration complete.")
                else:
                    logger_mp.warning("Calibration failed (missing wrist pose or robot model state).")
                CALIBRATE = False

            if START and not CALIBRATED:
                if tele_data is None:
                    tele_data = tv_wrapper.get_tele_data()
                if _try_calibrate_from_teleop(tele_data, arm_ctrl, arm_ik):
                    logger_mp.info("Calibration complete. Starting tracking.")
                else:
                    logger_mp.warning("Calibration failed. Tracking not started.")
                    START = False
                    continue

            if START and CALIBRATED:
                break

            if args.arm != "H2" and camera_config["head_camera"]["enable_zmq"] and xr_need_local_img:
                head_img = img_client.get_head_frame()
                tv_wrapper.render_to_xr(head_img)

        logger_mp.info("---------------------🚀start Tracking🚀-------------------------")
        arm_ctrl.speed_gradual_max()

        head_img = None
        left_wrist_img = None
        right_wrist_img = None

        # Buffer for t+1 action labeling: action[t] = state[t+1]
        _sharpa_ee_prev = None  # (left_list, right_list, receive_timestamps) from previous step
        prev_y_button = False  # for edge detection on Y button

        # main loop. robot start to follow VR user's motion
        while not STOP:
            start_time = time.time()

            # STOP/finalize runs on the capture worker.  Consume its result and
            # write the small local clock/manifest reference only after the ACK;
            # until then a new native episode is deliberately not ready.
            if native_capture_client is not None:
                finalized = native_capture_client.poll_finalized()
                if finalized is not None:
                    pending_native_metadata = finalized
                if pending_native_metadata is not None:
                    if pending_native_episode_dir is None:
                        logger_mp.warning(
                            "[SharpaCapture] cleanup finalized without a local episode: "
                            f"capture_id={pending_native_metadata.get('capture_id')}"
                        )
                        pending_native_metadata = None
                    else:
                        try:
                            metadata_path = write_capture_metadata(
                                pending_native_episode_dir,
                                _episode_capture_metadata(
                                    pending_native_metadata,
                                    pending_native_episode_dir,
                                    args.task_name,
                                ),
                            )
                            logger_mp.info(
                                "[SharpaCapture] finalized local metadata: "
                                f"{metadata_path} status={pending_native_metadata.get('status')}"
                            )
                            pending_native_metadata = None
                            pending_native_episode_dir = None
                        except Exception as e:
                            logger_mp.error(f"[SharpaCapture] metadata write failed; will retry: {e}")

            # get image
            if camera_config["head_camera"]["enable_zmq"]:
                if args.record or xr_need_local_img:
                    head_img = img_client.get_head_frame()
                if xr_need_local_img and head_img is not None and head_img.bgr is not None:
                    tv_wrapper.render_to_xr(head_img)
            if camera_config["left_wrist_camera"]["enable_zmq"]:
                if args.record:
                    left_wrist_img = img_client.get_left_wrist_frame()
            if camera_config["right_wrist_camera"]["enable_zmq"]:
                if args.record:
                    right_wrist_img = img_client.get_right_wrist_frame()

            # record mode
            if args.record and RECORD_TOGGLE:
                RECORD_TOGGLE = False
                if not RECORD_RUNNING:
                    local_ready = recorder.is_ready()
                    native_ready = (
                        native_capture_client is None
                        or (
                            native_capture_client.is_ready()
                            and pending_native_metadata is None
                            and pending_native_episode_dir is None
                        )
                    )
                    if not local_ready or not native_ready:
                        logger_mp.warning(
                            "Recording is still finalizing; START ignored "
                            f"(local_ready={local_ready}, native_ready={native_ready})."
                        )
                    else:
                        capture_started = native_capture_client is None
                        new_capture_id = None
                        if native_capture_client is not None:
                            new_capture_id = native_capture_client.new_capture_id()
                            try:
                                native_capture_client.start_capture(
                                    new_capture_id,
                                    start_timeout_s=args.sharpa_native_capture_timeout,
                                )
                                capture_started = True
                            except CaptureControlError as e:
                                pending_native_episode_dir = None
                                logger_mp.error(
                                    "[SharpaCapture] START not acknowledged; local recording "
                                    f"was not started: {e}"
                                )

                        if capture_started and recorder.create_episode():
                            if native_capture_client is None:
                                RECORD_RUNNING = True
                            else:
                                active_native_capture_id = new_capture_id
                                active_native_episode_dir = recorder.episode_dir
                                try:
                                    initial_metadata = native_capture_client.session_metadata()
                                    if initial_metadata is None:
                                        raise RuntimeError("missing native capture session metadata")
                                    write_capture_metadata(
                                        active_native_episode_dir,
                                        _episode_capture_metadata(
                                            initial_metadata,
                                            active_native_episode_dir,
                                            args.task_name,
                                        ),
                                    )
                                    RECORD_RUNNING = True
                                except Exception as e:
                                    logger_mp.error(
                                        "[SharpaCapture] local episode association failed; "
                                        f"recording aborted: {e}"
                                    )
                                    native_capture_client.stop_capture_async(new_capture_id)
                                    pending_native_episode_dir = active_native_episode_dir
                                    active_native_capture_id = None
                                    active_native_episode_dir = None
                                    recorder.save_episode()
                        elif capture_started:
                            logger_mp.error("Failed to create episode. Recording not started.")
                            if native_capture_client is not None and new_capture_id is not None:
                                native_capture_client.stop_capture_async(new_capture_id)
                                pending_native_episode_dir = None
                else:
                    RECORD_RUNNING = False
                    if native_capture_client is not None and active_native_capture_id is not None:
                        stop_monotonic_ns = time.monotonic_ns()
                        stop_realtime_ns = time.time_ns()
                        native_capture_client.stop_capture_async(
                            active_native_capture_id,
                            workstation_monotonic_ns=stop_monotonic_ns,
                            workstation_realtime_ns=stop_realtime_ns,
                        )
                        pending_native_episode_dir = active_native_episode_dir
                        active_native_capture_id = None
                        active_native_episode_dir = None
                    recorder.save_episode()
                    if args.sim:
                        publish_reset_category(1, reset_pose_publisher)

            # get xr's tele data
            tele_data = tv_wrapper.get_tele_data()
            if (
                args.ee == "dex3" or args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco"
            ) and args.input_mode == "hand":
                with left_hand_pos_array.get_lock():
                    left_hand_pos_array[:] = tele_data.left_hand_pos.flatten()
                with right_hand_pos_array.get_lock():
                    right_hand_pos_array[:] = tele_data.right_hand_pos.flatten()
            elif args.ee == "dex1" and args.input_mode == "controller":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_ctrl_triggerValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_ctrl_triggerValue
            elif args.ee == "dex1" and args.input_mode == "hand":
                with left_gripper_value.get_lock():
                    left_gripper_value.value = tele_data.left_hand_pinchValue
                with right_gripper_value.get_lock():
                    right_gripper_value.value = tele_data.right_hand_pinchValue
            else:
                pass

            # high level control
            if args.input_mode == "controller":
                # quit teleoperate
                if tele_data.right_ctrl_aButton:
                    START = False
                    STOP = True

                # toggle recording on rising edge of Y button (left controller B = Y)
                y_button = tele_data.left_ctrl_bButton
                if args.record and y_button and not prev_y_button:
                    RECORD_TOGGLE = True
                prev_y_button = y_button

                if args.motion:
                    # command robot to enter damping mode. soft emergency stop function
                    if tele_data.left_ctrl_thumbstick and tele_data.right_ctrl_thumbstick:
                        loco_wrapper.Damp()
                    # https://github.com/unitreerobotics/xr_teleoperate/issues/135, control, limit velocity to within 0.3
                    loco_wrapper.Move(
                        -tele_data.left_ctrl_thumbstickValue[1] * 0.3,
                        -tele_data.left_ctrl_thumbstickValue[0] * 0.3,
                        -tele_data.right_ctrl_thumbstickValue[0] * 0.3,
                    )

            if CALIBRATE:
                if _try_calibrate_from_teleop(tele_data, arm_ctrl, arm_ik):
                    logger_mp.info("Calibration complete.")
                else:
                    logger_mp.warning("Calibration failed (missing wrist pose or robot model state).")
                CALIBRATE = False

            if not CALIBRATED:
                continue

            # Get one coherent H2 LowState copy for arms, waist and timing.
            # Other robot types retain their existing state accessors.
            h2_state_timestamps = {}
            h2_waist_q = None
            if args.arm == "H2":
                h2_state_snapshot = arm_ctrl.get_recording_state_snapshot()
                current_lr_arm_q = h2_state_snapshot["dual_arm_q"]
                current_lr_arm_dq = h2_state_snapshot["dual_arm_dq"]
                h2_waist_q = h2_state_snapshot["waist_q"]
                h2_state_timestamps = h2_state_snapshot["timestamps"]
            else:
                current_lr_arm_q = arm_ctrl.get_current_dual_arm_q()
                current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            # --arm-side semantics: the disabled arm keeps targeting its calibrated
            # init pose every tick, so the IK solver resolves it to a constant joint
            # configuration. Mechanically the arm holds still; the recording captures
            # constant values for the disabled side instead of operator drift.
            if args.arm_side in ("left", "both"):
                left_target_pose = _compute_relative_target_pose(
                    tele_data.left_wrist_pose,
                    REF_LEFT_WRIST_POSE,
                    INIT_LEFT_TARGET_POSE,
                )
            else:
                left_target_pose = INIT_LEFT_TARGET_POSE
            if args.arm_side in ("right", "both"):
                right_target_pose = _compute_relative_target_pose(
                    tele_data.right_wrist_pose,
                    REF_RIGHT_WRIST_POSE,
                    INIT_RIGHT_TARGET_POSE,
                )
            else:
                right_target_pose = INIT_RIGHT_TARGET_POSE

            # solve ik using motor data and wrist pose, then use ik results to control arms.
            time_ik_start = time.time()
            sol_q, sol_tauff = arm_ik.solve_ik(
                left_target_pose,
                right_target_pose,
                current_lr_arm_q,
                current_lr_arm_dq,
            )
            time_ik_end = time.time()
            logger_mp.debug(f"ik:\t{round(time_ik_end - time_ik_start, 6)}")
            arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)
            h2_command_timestamps = (
                arm_ctrl.get_command_timing_snapshot() if args.arm == "H2" else {}
            )

            # record data
            if args.record:
                READY = recorder.is_ready() and (
                    native_capture_client is None
                    or (
                        native_capture_client.is_ready()
                        and pending_native_metadata is None
                        and pending_native_episode_dir is None
                    )
                )  # ready only after both local save and Thor finalization
                # dex hand or gripper
                sharpa_hand_state_timestamps = {}
                sharpa_hand_action_proxy_timestamps = {}
                sharpa_hand_desired_timestamps = {}
                sharpa_left_desired_q = None
                sharpa_right_desired_q = None
                if args.ee == "dex3" and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:7]
                        right_ee_state = dual_hand_state_array[-7:]
                        left_hand_action = dual_hand_action_array[:7]
                        right_hand_action = dual_hand_action_array[-7:]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "hand":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "dex1" and args.input_mode == "controller":
                    with dual_gripper_data_lock:
                        left_ee_state = [dual_gripper_state_array[0]]
                        right_ee_state = [dual_gripper_state_array[1]]
                        left_hand_action = [dual_gripper_action_array[0]]
                        right_hand_action = [dual_gripper_action_array[1]]
                        current_body_state = arm_ctrl.get_current_motor_q().tolist()
                        current_body_action = [
                            -tele_data.left_ctrl_thumbstickValue[1] * 0.3,
                            -tele_data.left_ctrl_thumbstickValue[0] * 0.3,
                            -tele_data.right_ctrl_thumbstickValue[0] * 0.3,
                        ]
                elif (
                    args.ee == "inspire_dfx" or args.ee == "inspire_ftp" or args.ee == "brainco"
                ) and args.input_mode == "hand":
                    with dual_hand_data_lock:
                        left_ee_state = dual_hand_state_array[:6]
                        right_ee_state = dual_hand_state_array[-6:]
                        left_hand_action = dual_hand_action_array[:6]
                        right_hand_action = dual_hand_action_array[-6:]
                        current_body_state = []
                        current_body_action = []
                elif args.ee == "sharpa":
                    hand_snapshot = hand_ctrl.get_state_snapshot()
                    curr_left_ee = hand_snapshot["left"]
                    curr_right_ee = hand_snapshot["right"]
                    current_hand_timestamps = hand_snapshot["timestamps"]
                    sharpa_left_desired_q = hand_snapshot["desired_q"]["left"]
                    sharpa_right_desired_q = hand_snapshot["desired_q"]["right"]
                    sharpa_hand_desired_timestamps = {
                        side: timestamps
                        for side, timestamps in hand_snapshot["desired_timestamps"].items()
                        if timestamps
                    }
                    if all(v == 0.0 for v in curr_left_ee) and all(v == 0.0 for v in curr_right_ee):
                        logger_mp.warning(
                            "[SharpaWave] hand state is ALL ZEROS — check that the bridge "
                            "(or retargeting -sdk -dds) is publishing rt/sharpa/{left,right}/state "
                            "on the same DDS domain"
                        )
                    # action[t] = state[t+1]: recorded action is the next step's observed state
                    left_ee_state = _sharpa_ee_prev[0] if _sharpa_ee_prev is not None else curr_left_ee
                    right_ee_state = _sharpa_ee_prev[1] if _sharpa_ee_prev is not None else curr_right_ee
                    sharpa_hand_state_timestamps = (
                        _sharpa_ee_prev[2]
                        if _sharpa_ee_prev is not None
                        else current_hand_timestamps
                    )
                    left_hand_action = curr_left_ee
                    right_hand_action = curr_right_ee
                    # This is feedback used as an action proxy, not the glove's
                    # actual desired/applied command. Preserve its source time
                    # under an explicitly named field.
                    sharpa_hand_action_proxy_timestamps = current_hand_timestamps
                    _sharpa_ee_prev = (
                        curr_left_ee,
                        curr_right_ee,
                        current_hand_timestamps,
                    )
                    current_body_state = []
                    current_body_action = []
                else:
                    left_ee_state = []
                    right_ee_state = []
                    left_hand_action = []
                    right_hand_action = []
                    current_body_state = []
                    current_body_action = []

                # H2 waist (yaw, roll, pitch) — recorded under body.qpos for state only.
                # Hardware is read-only for waist (no SDK control), so action stays empty.
                if args.arm == "H2" and args.record_waist:
                    current_body_state = h2_waist_q.tolist()

                # arm state and action
                left_arm_state = current_lr_arm_q[:7]
                right_arm_state = current_lr_arm_q[-7:]
                left_arm_action = sol_q[:7]
                right_arm_action = sol_q[-7:]
                if RECORD_RUNNING:
                    sample_monotonic_ns = time.monotonic_ns()
                    sample_realtime_ns = time.time_ns()
                    colors = {}
                    depths = {}
                    if camera_config["head_camera"]["binocular"]:
                        if camera_config["head_camera"]["enable_zmq"]:
                            if head_img is not None and head_img.bgr is not None:
                                colors[f"color_{0}"] = head_img.bgr[
                                    :,
                                    : camera_config["head_camera"]["image_shape"][1] // 2,
                                ]
                                colors[f"color_{1}"] = head_img.bgr[
                                    :,
                                    camera_config["head_camera"]["image_shape"][1] // 2 :,
                                ]
                            else:
                                logger_mp.warning("Head image is None!")
                        if camera_config["left_wrist_camera"]["enable_zmq"]:
                            if left_wrist_img is not None and left_wrist_img.bgr is not None:
                                colors[f"color_{2}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config["right_wrist_camera"]["enable_zmq"]:
                            if right_wrist_img is not None and right_wrist_img.bgr is not None:
                                colors[f"color_{3}"] = right_wrist_img.bgr
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    else:
                        if camera_config["head_camera"]["enable_zmq"]:
                            if head_img is not None and head_img.bgr is not None:
                                colors[f"color_{0}"] = head_img.bgr
                            else:
                                logger_mp.warning("Head image is None!")
                        if camera_config["left_wrist_camera"]["enable_zmq"]:
                            if left_wrist_img is not None and left_wrist_img.bgr is not None:
                                colors[f"color_{1}"] = left_wrist_img.bgr
                            else:
                                logger_mp.warning("Left wrist image is None!")
                        if camera_config["right_wrist_camera"]["enable_zmq"]:
                            if right_wrist_img is not None and right_wrist_img.bgr is not None:
                                colors[f"color_{2}"] = right_wrist_img.bgr
                            else:
                                logger_mp.warning("Right wrist image is None!")
                    states = {
                        "left_arm": {
                            "qpos": left_arm_state.tolist(),  # numpy.array -> list
                            "qvel": [],
                            "torque": [],
                        },
                        "right_arm": {
                            "qpos": right_arm_state.tolist(),
                            "qvel": [],
                            "torque": [],
                        },
                        "left_ee": {
                            "qpos": left_ee_state,
                            "qvel": [],
                            "torque": [],
                        },
                        "right_ee": {
                            "qpos": right_ee_state,
                            "qvel": [],
                            "torque": [],
                        },
                        "body": {
                            "qpos": current_body_state,
                        },
                    }
                    actions = {
                        "left_arm": {
                            "qpos": left_arm_action.tolist(),
                            "qvel": [],
                            "torque": [],
                        },
                        "right_arm": {
                            "qpos": right_arm_action.tolist(),
                            "qvel": [],
                            "torque": [],
                        },
                        "left_ee": {
                            "qpos": left_hand_action,
                            "qvel": [],
                            "torque": [],
                        },
                        "right_ee": {
                            "qpos": right_hand_action,
                            "qvel": [],
                            "torque": [],
                        },
                        "body": {
                            "qpos": current_body_action,
                        },
                    }
                    # Keep legacy actions.*_ee.qpos as the feedback proxy for
                    # compatibility. This additive vector is the actual desired
                    # command observed on rt/sharpa/{side}/cmd.
                    if sharpa_left_desired_q is not None:
                        actions["left_ee"]["desired_qpos"] = sharpa_left_desired_q
                    if sharpa_right_desired_q is not None:
                        actions["right_ee"]["desired_qpos"] = sharpa_right_desired_q
                    camera_timestamps = {}
                    camera_timestamps.update(_frame_camera_timestamps(head_img, "head"))
                    camera_timestamps.update(
                        _frame_camera_timestamps(left_wrist_img, "wrist_left")
                    )
                    camera_timestamps.update(
                        _frame_camera_timestamps(right_wrist_img, "wrist_right")
                    )
                    item_timestamps = {
                        "workstation_monotonic_ns": sample_monotonic_ns,
                        "workstation_realtime_ns": sample_realtime_ns,
                        "cameras": camera_timestamps,
                    }
                    if h2_state_timestamps:
                        item_timestamps["h2_lowstate"] = h2_state_timestamps
                    if h2_command_timestamps:
                        item_timestamps["h2_arm_command"] = h2_command_timestamps
                    if sharpa_hand_state_timestamps:
                        item_timestamps["sharpa_hand_state"] = sharpa_hand_state_timestamps
                    if sharpa_hand_action_proxy_timestamps:
                        item_timestamps["sharpa_hand_action_proxy"] = (
                            sharpa_hand_action_proxy_timestamps
                        )
                    if sharpa_hand_desired_timestamps:
                        item_timestamps["sharpa_hand_desired_command"] = (
                            sharpa_hand_desired_timestamps
                        )
                    native_marker = None
                    if native_capture_client is not None and active_native_capture_id is not None:
                        native_marker = {
                            "protocol": SHARPA_CAPTURE_PROTOCOL,
                            "capture_id": active_native_capture_id,
                            "workstation_monotonic_ns": sample_monotonic_ns,
                            "workstation_realtime_ns": sample_realtime_ns,
                        }

                    tactiles = tactile_sub.snapshot() if tactile_sub is not None else None
                    sim_state = sim_state_subscriber.read_data() if args.sim else None
                    item_idx = recorder.add_item(
                        colors=colors,
                        depths=depths,
                        states=states,
                        actions=actions,
                        tactiles=tactiles,
                        sim_state=sim_state,
                        timestamps=item_timestamps,
                        native_capture=native_marker,
                    )
                    if native_marker is not None:
                        marker_accepted = native_capture_client.enqueue_sample(
                            active_native_capture_id,
                            item_idx,
                            workstation_monotonic_ns=sample_monotonic_ns,
                            workstation_realtime_ns=sample_realtime_ns,
                        )
                        if not marker_accepted:
                            logger_mp.error(
                                "[SharpaCapture] SAMPLE marker dropped; episode will be marked "
                                f"invalid (idx={item_idx})"
                            )

            current_time = time.time()
            time_elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - time_elapsed)
            time.sleep(sleep_time)
            logger_mp.debug(f"main process sleep: {sleep_time}")

    except KeyboardInterrupt:
        logger_mp.info("⛔ KeyboardInterrupt, exiting program...")
    except Exception:
        import traceback

        logger_mp.error(traceback.format_exc())
    finally:
        # Queue native STOP before moving the robot home, but never wait for Thor
        # disk finalization on the control/shutdown path yet.  The worker runs in
        # parallel with the existing cleanup below.
        try:
            if native_capture_client is not None:
                capture_id_to_stop = (
                    active_native_capture_id or native_capture_client.active_capture_id()
                )
                if capture_id_to_stop is not None:
                    stop_queued = native_capture_client.stop_capture_async(
                        capture_id_to_stop,
                        workstation_monotonic_ns=time.monotonic_ns(),
                        workstation_realtime_ns=time.time_ns(),
                    )
                    if stop_queued:
                        if pending_native_episode_dir is None:
                            pending_native_episode_dir = active_native_episode_dir
                        active_native_capture_id = None
                        active_native_episode_dir = None
        except Exception as e:
            logger_mp.error(f"[SharpaCapture] failed to queue STOP during shutdown: {e}")

        try:
            arm_ctrl.ctrl_dual_arm_go_home()
        except Exception as e:
            logger_mp.error(f"Failed to ctrl_dual_arm_go_home: {e}")

        try:
            if args.ipc:
                ipc_server.stop()
            else:
                stop_listening()
                listen_keyboard_thread.join()
        except Exception as e:
            logger_mp.error(f"Failed to stop keyboard listener or ipc server: {e}")

        try:
            if img_client is not None:
                img_client.close()
        except Exception as e:
            logger_mp.error(f"Failed to close image client: {e}")

        try:
            tv_wrapper.close()
        except Exception as e:
            logger_mp.error(f"Failed to close televuer wrapper: {e}")

        try:
            if not args.motion:
                pass
                # status, result = motion_switcher.Exit_Debug_Mode()
                # logger_mp.info(f"Exit debug mode: {'Success' if status == 3104 else 'Failed'}")
        except Exception as e:
            logger_mp.error(f"Failed to exit debug mode: {e}")

        try:
            if args.sim:
                sim_state_subscriber.stop_subscribe()
        except Exception as e:
            logger_mp.error(f"Failed to stop sim state subscriber: {e}")

        try:
            if args.record:
                recorder.close()
        except Exception as e:
            logger_mp.error(f"Failed to close recorder: {e}")

        try:
            if native_capture_client is not None:
                finalized = native_capture_client.poll_finalized()
                if finalized is None and not native_capture_client.is_ready():
                    finalized = native_capture_client.wait_finalized(
                        args.sharpa_native_capture_stop_timeout + 0.5
                    )
                if finalized is not None:
                    pending_native_metadata = finalized
                if pending_native_metadata is None and pending_native_episode_dir is not None:
                    # Preserve a conservative failure record if Thor never
                    # acknowledged STOP before shutdown.
                    pending_native_metadata = native_capture_client.session_metadata() or {}
                    pending_native_metadata["status"] = "invalid"
                    pending_native_metadata["valid"] = False
                    reasons = list(pending_native_metadata.get("degraded_reasons", []))
                    reasons.append("shutdown before STOP/finalize acknowledgement")
                    pending_native_metadata["degraded_reasons"] = reasons
                if pending_native_metadata is not None and pending_native_episode_dir is not None:
                    metadata_path = write_capture_metadata(
                        pending_native_episode_dir,
                        _episode_capture_metadata(
                            pending_native_metadata,
                            pending_native_episode_dir,
                            args.task_name,
                        ),
                    )
                    logger_mp.info(f"[SharpaCapture] shutdown metadata: {metadata_path}")
                native_capture_client.close(timeout_s=1.0)
        except Exception as e:
            logger_mp.error(f"[SharpaCapture] failed to finalize local metadata: {e}")
        logger_mp.info("✅ Finally, exiting program.")
        exit(0)
