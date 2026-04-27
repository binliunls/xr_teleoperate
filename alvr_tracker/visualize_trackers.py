#!/usr/bin/env python3
"""
Real-time 3D visualization of ALVR Motion Trackers and H2 robot arm.

Shows tracker positions, robot URDF meshes with real-time IK, and target
end-effector positions in the Rerun 3D viewer.

Usage:
    python visualize_trackers.py              # Trackers only
    python visualize_trackers.py --robot      # Trackers + H2 robot arm with IK
"""

import sys
import os

# Conda envs ship a newer libstdc++ than the system default.  Prepend
# $CONDA_PREFIX/lib so the dynamic linker finds it before /lib/…, then
# re-exec so the change takes effect for all subsequent shared-library loads.
if os.environ.get("CONDA_PREFIX") and "_VIZ_LD_FIXED" not in os.environ:
    _conda_lib = os.path.join(os.environ["CONDA_PREFIX"], "lib")
    _ld = os.environ.get("LD_LIBRARY_PATH", "")
    if os.path.isdir(_conda_lib) and _conda_lib not in _ld.split(":"):
        os.environ["LD_LIBRARY_PATH"] = _conda_lib + (":" + _ld if _ld else "")
        os.environ["_VIZ_LD_FIXED"] = "1"
        os.execv(sys.executable, [sys.executable] + sys.argv)

import argparse
import time
import threading
import numpy as np

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

try:
    import rerun as rr
except ImportError:
    print("Error: rerun-sdk not installed. Install with: pip install rerun-sdk")
    sys.exit(1)

from alvr_tracker_bridge import ALVRTrackerBridge

CALIBRATE = False
START = False
STOP = False


def on_press(key):
    global CALIBRATE, START, STOP
    if key == "c":
        CALIBRATE = True
    elif key == "r":
        START = True
    elif key == "q":
        STOP = True


class RobotVisualizer:
    """H2 robot visualization using pinocchio FK and per-link STL meshes."""

    ARM_JOINT_NAMES = [
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]

    def __init__(self, urdf_path, position_scale=0.8):
        import pinocchio as pin

        self.pin = pin

        mesh_dir = os.path.dirname(urdf_path)
        self.full_robot = pin.RobotWrapper.BuildFromURDF(urdf_path, mesh_dir)

        self.visuals = []
        for geom in self.full_robot.visual_model.geometryObjects:
            mesh_path = geom.meshPath
            if not os.path.isabs(mesh_path):
                mesh_path = os.path.join(mesh_dir, mesh_path)
            self.visuals.append(
                {
                    "name": geom.name,
                    "frame_id": geom.parentFrame,
                    "placement": geom.placement,
                    "mesh_path": mesh_path,
                    "scale": geom.meshScale.copy(),
                }
            )

        self.arm_joint_idx_q = []
        for name in self.ARM_JOINT_NAMES:
            jid = self.full_robot.model.getJointId(name)
            self.arm_joint_idx_q.append(self.full_robot.model.joints[jid].idx_q)

        saved_cwd = os.getcwd()
        os.chdir(os.path.join(parent_dir, "teleop"))
        from teleop.robot_control.robot_arm_ik import H2_ArmIK

        self.arm_ik = H2_ArmIK()
        os.chdir(saved_cwd)

        self.position_scale = position_scale
        self.R_openvr_to_robot = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        self.calibrated = False
        self.ref_left_pos = None
        self.ref_right_pos = None
        self.ref_left_rot = None
        self.ref_right_rot = None
        self.init_left_wrist_pose = None
        self.init_right_wrist_pose = None

    def load_meshes(self):
        """Load all link STL meshes into Rerun (static)."""
        loaded = 0
        for info in self.visuals:
            if not os.path.exists(info["mesh_path"]):
                continue
            rr.log(f"robot/{info['name']}", rr.Asset3D(path=info["mesh_path"]), static=True)
            loaded += 1
        print(f"  Loaded {loaded}/{len(self.visuals)} robot meshes")
        self.update_robot_pose(np.zeros(14))

    def update_robot_pose(self, arm_q):
        """Recompute FK and position every mesh in the viewer."""
        pin = self.pin
        full_q = pin.neutral(self.full_robot.model)
        for i, idx_q in enumerate(self.arm_joint_idx_q):
            full_q[idx_q] = arm_q[i]

        pin.forwardKinematics(self.full_robot.model, self.full_robot.data, full_q)
        pin.updateFramePlacements(self.full_robot.model, self.full_robot.data)

        for info in self.visuals:
            link_se3 = self.full_robot.data.oMf[info["frame_id"]]
            mesh_se3 = link_se3 * info["placement"]
            rot = mesh_se3.rotation
            scale = info["scale"]
            if not np.allclose(scale, 1.0):
                rot = rot @ np.diag(scale)
            rr.log(
                f"robot/{info['name']}",
                rr.Transform3D(
                    translation=mesh_se3.translation.tolist(),
                    mat3x3=rot.flatten().tolist(),
                ),
            )

    def calibrate(self, arm_poses):
        """Auto-calibrate facing direction from tracker positions."""
        if not arm_poses["left"]["valid"] or not arm_poses["right"]["valid"]:
            return False

        self.ref_left_pos = arm_poses["left"]["position"].copy()
        self.ref_right_pos = arm_poses["right"]["position"].copy()
        self.ref_left_rot = arm_poses["left"]["rotation"].copy()
        self.ref_right_rot = arm_poses["right"]["rotation"].copy()

        lr_vec = self.ref_right_pos - self.ref_left_pos
        lr_horizontal = lr_vec.copy()
        lr_horizontal[1] = 0.0
        lr_norm = np.linalg.norm(lr_horizontal)

        if lr_norm > 0.1:
            right_dir = lr_horizontal / lr_norm
            up_dir = np.array([0.0, 1.0, 0.0])
            forward_dir = np.cross(up_dir, right_dir)
            forward_dir /= np.linalg.norm(forward_dir)
            right_dir = np.cross(forward_dir, up_dir)
            right_dir /= np.linalg.norm(right_dir)
            self.R_openvr_to_robot = np.array([forward_dir, -right_dir, up_dir])
            print(f"  Forward (OpenVR): [{forward_dir[0]:.3f}, {forward_dir[1]:.3f}, {forward_dir[2]:.3f}]")

        home_q = np.zeros(self.arm_ik.reduced_robot.model.nq)
        pin = self.pin
        pin.forwardKinematics(self.arm_ik.reduced_robot.model, self.arm_ik.reduced_robot.data, home_q)
        pin.updateFramePlacements(self.arm_ik.reduced_robot.model, self.arm_ik.reduced_robot.data)

        self.init_left_wrist_pose = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.L_hand_id].homogeneous.copy()
        self.init_right_wrist_pose = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.R_hand_id].homogeneous.copy()

        self.calibrated = True
        return True

    def tracker_to_robot_pos(self, tracker_pos, ref_pos, init_ee_pos):
        """Map a tracker position to robot-frame coordinates near the EE."""
        R = self.R_openvr_to_robot
        delta = R @ ((tracker_pos - ref_pos) * self.position_scale)
        return init_ee_pos + delta

    def compute_wrist_poses(self, arm_poses):
        """Compute IK target poses from tracker delta motion."""
        if not self.calibrated:
            return None, None
        if not arm_poses["left"]["valid"] or not arm_poses["right"]["valid"]:
            return None, None

        R = self.R_openvr_to_robot
        s = self.position_scale

        left_delta = R @ ((arm_poses["left"]["position"] - self.ref_left_pos) * s)
        right_delta = R @ ((arm_poses["right"]["position"] - self.ref_right_pos) * s)

        left_rel_rot = R @ (arm_poses["left"]["rotation"] @ self.ref_left_rot.T) @ R.T
        right_rel_rot = R @ (arm_poses["right"]["rotation"] @ self.ref_right_rot.T) @ R.T

        left_wrist = np.eye(4)
        left_wrist[:3, :3] = left_rel_rot @ self.init_left_wrist_pose[:3, :3]
        left_wrist[:3, 3] = self.init_left_wrist_pose[:3, 3] + left_delta

        right_wrist = np.eye(4)
        right_wrist[:3, :3] = right_rel_rot @ self.init_right_wrist_pose[:3, :3]
        right_wrist[:3, 3] = self.init_right_wrist_pose[:3, 3] + right_delta

        return left_wrist, right_wrist

    def solve_and_visualize(self, arm_poses):
        """Solve IK, update robot meshes, and show targets vs FK results."""
        left_wrist, right_wrist = self.compute_wrist_poses(arm_poses)
        if left_wrist is None:
            return None

        left_target = left_wrist[:3, 3]
        right_target = right_wrist[:3, 3]

        rr.log(
            "targets/left",
            rr.Points3D(
                [left_target],
                colors=[[0, 200, 255]],
                radii=[0.015],
            ),
        )
        rr.log(
            "targets/right",
            rr.Points3D(
                [right_target],
                colors=[[255, 150, 0]],
                radii=[0.015],
            ),
        )

        sol_q, _ = self.arm_ik.solve_ik(left_wrist, right_wrist)
        self.update_robot_pose(sol_q)

        pin = self.pin
        pin.forwardKinematics(
            self.arm_ik.reduced_robot.model,
            self.arm_ik.reduced_robot.data,
            np.array(sol_q).flatten(),
        )
        pin.updateFramePlacements(
            self.arm_ik.reduced_robot.model,
            self.arm_ik.reduced_robot.data,
        )
        left_fk = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.L_hand_id].translation.copy()
        right_fk = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.R_hand_id].translation.copy()

        rr.log(
            "robot/left_ee",
            rr.Points3D(
                [left_fk.tolist()],
                colors=[[0, 255, 100]],
                radii=[0.012],
            ),
        )
        rr.log(
            "robot/right_ee",
            rr.Points3D(
                [right_fk.tolist()],
                colors=[[0, 255, 100]],
                radii=[0.012],
            ),
        )

        rr.log(
            "targets/left_error_line",
            rr.LineStrips3D(
                [[left_target.tolist(), left_fk.tolist()]],
                colors=[[255, 50, 50, 180]],
            ),
        )
        rr.log(
            "targets/right_error_line",
            rr.LineStrips3D(
                [[right_target.tolist(), right_fk.tolist()]],
                colors=[[255, 50, 50, 180]],
            ),
        )

        left_err = np.linalg.norm(left_target - left_fk)
        right_err = np.linalg.norm(right_target - right_fk)
        rr.log("metrics/ik_error_left_mm", rr.Scalar(left_err * 1000))
        rr.log("metrics/ik_error_right_mm", rr.Scalar(right_err * 1000))

        for i, name in enumerate(self.ARM_JOINT_NAMES):
            short = name.replace("_joint", "").replace("left_", "L_").replace("right_", "R_")
            rr.log(f"joints/{short}", rr.Scalar(np.degrees(sol_q[i])))

        return sol_q


def main():
    global CALIBRATE, START, STOP

    parser = argparse.ArgumentParser(description="ALVR Tracker + H2 Robot Visualization")
    parser.add_argument("--robot", action="store_true", help="Show H2 robot arm with real-time IK")
    parser.add_argument("--position-scale", type=float, default=0.8, help="Position movement scale factor")
    args = parser.parse_args()

    print("=" * 60)
    print("ALVR Motion Tracker Visualization")
    print("=" * 60)

    bridge = ALVRTrackerBridge()
    if not bridge.initialize():
        print("Failed to initialize. Make sure ALVR is running.")
        sys.exit(1)

    robot_viz = None
    if args.robot:
        urdf_path = os.path.join(parent_dir, "h2_description", "H2.urdf")
        if not os.path.exists(urdf_path):
            print(f"URDF not found: {urdf_path}")
        else:
            try:
                robot_viz = RobotVisualizer(urdf_path, position_scale=args.position_scale)
                print("H2 robot model loaded.")
            except Exception as e:
                import traceback

                print(f"Failed to load robot: {e}")
                traceback.print_exc()

    use_robot_frame = robot_viz is not None

    rr.init("tracker_robot_viz", spawn=True)
    if use_robot_frame:
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    else:
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)

    grid_size = 1.5
    grid_lines = []
    for val in np.arange(-grid_size, grid_size + 0.25, 0.25):
        if use_robot_frame:
            grid_lines.append([[val, -grid_size, 0], [val, grid_size, 0]])
            grid_lines.append([[-grid_size, val, 0], [grid_size, val, 0]])
        else:
            grid_lines.append([[val, 0, -grid_size], [val, 0, grid_size]])
            grid_lines.append([[-grid_size, 0, val], [grid_size, 0, val]])
    rr.log("world/ground", rr.LineStrips3D(grid_lines, colors=[[100, 100, 100, 80]]), static=True)

    axis_len = 0.3
    rr.log(
        "world/origin",
        rr.Arrows3D(
            origins=[[0, 0, 0]] * 3,
            vectors=[[axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
        ),
        static=True,
    )

    if robot_viz:
        robot_viz.load_meshes()

    has_keyboard = False
    try:
        from sshkeyboard import listen_keyboard, stop_listening

        kb_thread = threading.Thread(
            target=listen_keyboard,
            kwargs={"on_press": on_press, "until": None, "sequential": False},
            daemon=True,
        )
        kb_thread.start()
        has_keyboard = True
    except ImportError:
        pass

    print("\nVisualization running!")
    if robot_viz:
        print("Controls:  [c] calibrate  [r] start IK  [q] quit")
    print("Press Ctrl+C to exit.\n")

    left_trail, right_trail = [], []
    max_trail = 100
    frame = 0
    calibrated = False

    try:
        while not STOP:
            rr.set_time_sequence("frame", frame)
            bridge.update()
            arm_poses = bridge.get_arm_poses()
            left = arm_poses["left"]
            right = arm_poses["right"]

            if CALIBRATE and robot_viz:
                CALIBRATE = False
                if robot_viz.calibrate(arm_poses):
                    calibrated = True
                    left_trail.clear()
                    right_trail.clear()
                    print("Calibrated! Press [r] to start IK.")

            tracking = robot_viz is not None and calibrated and START

            # --- Tracker display ---
            # In tracking mode: positions are shown relative to robot EE (aligned with the mesh)
            # Pre-tracking:     positions are absolute (transformed to robot frame if available)
            if left["valid"]:
                if tracking:
                    pos = robot_viz.tracker_to_robot_pos(
                        left["position"],
                        robot_viz.ref_left_pos,
                        robot_viz.init_left_wrist_pose[:3, 3],
                    ).tolist()
                elif use_robot_frame:
                    pos = (robot_viz.R_openvr_to_robot @ left["position"]).tolist()
                else:
                    pos = left["position"].tolist()

                rr.log("trackers/left/pos", rr.Points3D([pos], colors=[[0, 150, 255]], radii=[0.02]))
                left_trail.append(pos)
                if len(left_trail) > max_trail:
                    left_trail.pop(0)
                if len(left_trail) > 1:
                    rr.log("trackers/left/trail", rr.LineStrips3D([left_trail], colors=[[0, 150, 255, 100]]))

            if right["valid"]:
                if tracking:
                    pos = robot_viz.tracker_to_robot_pos(
                        right["position"],
                        robot_viz.ref_right_pos,
                        robot_viz.init_right_wrist_pose[:3, 3],
                    ).tolist()
                elif use_robot_frame:
                    pos = (robot_viz.R_openvr_to_robot @ right["position"]).tolist()
                else:
                    pos = right["position"].tolist()

                rr.log("trackers/right/pos", rr.Points3D([pos], colors=[[255, 150, 0]], radii=[0.02]))
                right_trail.append(pos)
                if len(right_trail) > max_trail:
                    right_trail.pop(0)
                if len(right_trail) > 1:
                    rr.log("trackers/right/trail", rr.LineStrips3D([right_trail], colors=[[255, 150, 0, 100]]))

            # Wrist connection line
            if left["valid"] and right["valid"]:
                if tracking:
                    lp = robot_viz.tracker_to_robot_pos(
                        left["position"],
                        robot_viz.ref_left_pos,
                        robot_viz.init_left_wrist_pose[:3, 3],
                    ).tolist()
                    rp = robot_viz.tracker_to_robot_pos(
                        right["position"],
                        robot_viz.ref_right_pos,
                        robot_viz.init_right_wrist_pose[:3, 3],
                    ).tolist()
                elif use_robot_frame:
                    R = robot_viz.R_openvr_to_robot
                    lp = (R @ left["position"]).tolist()
                    rp = (R @ right["position"]).tolist()
                else:
                    lp = left["position"].tolist()
                    rp = right["position"].tolist()
                rr.log("trackers/connection", rr.LineStrips3D([[lp, rp]], colors=[[200, 200, 200, 100]]))
                rr.log("metrics/wrist_distance", rr.Scalar(np.linalg.norm(np.array(lp) - np.array(rp))))

            if left["valid"]:
                rr.log("metrics/left_velocity", rr.Scalar(np.linalg.norm(left["velocity"])))
            if right["valid"]:
                rr.log("metrics/right_velocity", rr.Scalar(np.linalg.norm(right["velocity"])))

            # --- Robot IK ---
            if tracking and left["valid"] and right["valid"]:
                robot_viz.solve_and_visualize(arm_poses)

            frame += 1
            time.sleep(0.033)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if has_keyboard:
            try:
                stop_listening()
            except Exception:
                pass
        bridge.shutdown()
        print("Done.")


if __name__ == "__main__":
    main()
