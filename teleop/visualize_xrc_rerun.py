#!/usr/bin/env python3
"""
visualize_xrc_rerun.py

Real-time 3D visualization of Pico Motion Tracker data via the XRoboToolkit
SDK, using Rerun as the viewer.

Mirrors alvr_tracker/visualize_trackers.py exactly, with two differences:
  1. Data source: XrcWrapper (XRoboToolkit SDK over ADB) instead of
     ALVRTrackerBridge (OpenVR/SteamVR over Wi-Fi).
  2. No coordinate-frame rotation needed in the viz — XrcWrapper already
     converts poses to robot frame (Z-up, X-forward).

Usage:
    python visualize_xrc_rerun.py                    # trackers only
    python visualize_xrc_rerun.py --robot            # trackers + H2 robot arm + live IK
    python visualize_xrc_rerun.py --robot --arm G1_29
    python visualize_xrc_rerun.py --no-adb           # skip ADB (Pico on Wi-Fi)
    python visualize_xrc_rerun.py --left-serial ABC  --right-serial XYZ

Controls (when --robot is used):
    [c]  Calibrate — hold arms at robot home pose, press to capture reference
    [r]  Start IK  — robot mesh follows your tracker motion
    [q]  Quit
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import numpy as np

# ── libstdc++ shim (same fix as visualize_trackers.py) ──────────────────────
if os.environ.get("CONDA_PREFIX") and "_VIZ_LD_FIXED" not in os.environ:
    _conda_lib = os.path.join(os.environ["CONDA_PREFIX"], "lib")
    _ld = os.environ.get("LD_LIBRARY_PATH", "")
    if os.path.isdir(_conda_lib) and _conda_lib not in _ld.split(":"):
        os.environ["LD_LIBRARY_PATH"] = _conda_lib + (":" + _ld if _ld else "")
        os.environ["_VIZ_LD_FIXED"] = "1"
        os.execv(sys.executable, [sys.executable] + sys.argv)

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_THIS_DIR)
for _p in [_THIS_DIR, _REPO_ROOT]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    import rerun as rr
except ImportError:
    print("Error: rerun-sdk not installed.  Install with: pip install rerun-sdk")
    sys.exit(1)

from xrc_wrapper import XrcWrapper

# ── keyboard state ───────────────────────────────────────────────────────────
CALIBRATE = False
START     = False
STOP      = False


def on_press(key: str) -> None:
    global CALIBRATE, START, STOP
    if key == "c":
        CALIBRATE = True
    elif key == "r":
        START = True
    elif key == "q":
        STOP = True


# ── Robot visualizer (identical to visualize_trackers.py) ───────────────────

class RobotVisualizer:
    """
    Pinocchio FK + per-link STL meshes streamed to Rerun.

    Supports any robot whose URDF is under assets/ and whose IK class is in
    teleop/robot_control/robot_arm_ik.py.  Defaults to H2.
    """

    # Joint names ordered to match the IK sol_q vector (14 DOF dual arm)
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

    # Map CLI arm name → IK class import
    IK_CLASS_MAP = {
        "G1_29": ("robot_control.robot_arm_ik", "G1_29_ArmIK"),
        "G1_23": ("robot_control.robot_arm_ik", "G1_23_ArmIK"),
        "H1_2":  ("robot_control.robot_arm_ik", "H1_2_ArmIK"),
        "H1":    ("robot_control.robot_arm_ik", "H1_ArmIK"),
        "H2":    ("robot_control.robot_arm_ik", "H2_ArmIK"),
    }

    URDF_MAP = {
        "G1_29": "assets/g1/g1_29dof_rev_1_0.urdf",
        "G1_23": "assets/g1/g1_23dof_rev_1_0.urdf",
        "H1_2":  "assets/h1_2/h1_2.urdf",
        "H1":    "assets/h1/h1.urdf",
        "H2":    "h2_description/H2.urdf",
    }

    def __init__(self, arm: str = "H2", position_scale: float = 0.8) -> None:
        import pinocchio as pin
        self.pin = pin

        urdf_rel = self.URDF_MAP[arm]
        urdf_path = os.path.join(_REPO_ROOT, urdf_rel)
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF not found: {urdf_path}")

        mesh_dir = os.path.dirname(urdf_path)
        self.full_robot = pin.RobotWrapper.BuildFromURDF(urdf_path, mesh_dir)

        self.visuals = []
        for geom in self.full_robot.visual_model.geometryObjects:
            mesh_path = geom.meshPath
            if not os.path.isabs(mesh_path):
                mesh_path = os.path.join(mesh_dir, mesh_path)
            self.visuals.append({
                "name":      geom.name,
                "frame_id":  geom.parentFrame,
                "placement": geom.placement,
                "mesh_path": mesh_path,
                "scale":     geom.meshScale.copy(),
            })

        self.arm_joint_idx_q = []
        for name in self.ARM_JOINT_NAMES:
            jid = self.full_robot.model.getJointId(name)
            self.arm_joint_idx_q.append(self.full_robot.model.joints[jid].idx_q)

        # Load IK solver
        saved_cwd = os.getcwd()
        os.chdir(_THIS_DIR)
        mod_name, cls_name = self.IK_CLASS_MAP[arm]
        import importlib
        mod = importlib.import_module(mod_name)
        self.arm_ik = getattr(mod, cls_name)()
        os.chdir(saved_cwd)

        self.position_scale = position_scale

        # Calibration state
        self.calibrated      = False
        self.ref_left_pos:  np.ndarray | None = None
        self.ref_right_pos: np.ndarray | None = None
        self.ref_left_rot:  np.ndarray | None = None
        self.ref_right_rot: np.ndarray | None = None
        self.init_left_wrist:  np.ndarray | None = None  # (4,4)
        self.init_right_wrist: np.ndarray | None = None

    # ── mesh loading ──────────────────────────────────────────────────────

    def load_meshes(self) -> None:
        loaded = 0
        for info in self.visuals:
            if not os.path.exists(info["mesh_path"]):
                continue
            rr.log(f"robot/{info['name']}", rr.Asset3D(path=info["mesh_path"]), static=True)
            loaded += 1
        print(f"  Loaded {loaded}/{len(self.visuals)} robot meshes")
        self.update_robot_pose(np.zeros(14))

    def update_robot_pose(self, arm_q: np.ndarray) -> None:
        pin = self.pin
        full_q = pin.neutral(self.full_robot.model)
        for i, idx_q in enumerate(self.arm_joint_idx_q):
            full_q[idx_q] = arm_q[i]
        pin.forwardKinematics(self.full_robot.model, self.full_robot.data, full_q)
        pin.updateFramePlacements(self.full_robot.model, self.full_robot.data)
        for info in self.visuals:
            link_se3 = self.full_robot.data.oMf[info["frame_id"]]
            mesh_se3 = link_se3 * info["placement"]
            rot   = mesh_se3.rotation
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

    # ── calibration ───────────────────────────────────────────────────────

    def calibrate(self, arm_poses: dict) -> bool:
        """
        Capture reference tracker positions and current robot EE poses.

        Unlike visualize_trackers.py we do NOT compute R_openvr_to_robot here
        because XrcWrapper already outputs in robot frame.  The left→right
        vector is still used to print a sanity check.
        """
        if not arm_poses["left"]["valid"] or not arm_poses["right"]["valid"]:
            return False

        self.ref_left_pos  = arm_poses["left"]["position"].copy()
        self.ref_right_pos = arm_poses["right"]["position"].copy()
        self.ref_left_rot  = arm_poses["left"]["rotation"].copy()
        self.ref_right_rot = arm_poses["right"]["rotation"].copy()

        # Sanity: L→R vector should point roughly in +Y (robot right) in Z-up frame
        lr = self.ref_right_pos - self.ref_left_pos
        print(f"  L→R vector (robot frame): [{lr[0]:.3f}, {lr[1]:.3f}, {lr[2]:.3f}]")

        # FK at home pose → initial EE targets
        pin      = self.pin
        home_q   = np.zeros(self.arm_ik.reduced_robot.model.nq)
        pin.forwardKinematics(self.arm_ik.reduced_robot.model, self.arm_ik.reduced_robot.data, home_q)
        pin.updateFramePlacements(self.arm_ik.reduced_robot.model, self.arm_ik.reduced_robot.data)
        self.init_left_wrist  = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.L_hand_id].homogeneous.copy()
        self.init_right_wrist = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.R_hand_id].homogeneous.copy()

        self.calibrated = True
        return True

    # ── pose computation ──────────────────────────────────────────────────

    def tracker_to_robot_pos(
        self,
        tracker_pos: np.ndarray,
        ref_pos:     np.ndarray,
        init_ee_pos: np.ndarray,
    ) -> np.ndarray:
        """Map a tracker position delta to robot EE target (no frame rotation needed)."""
        delta = (tracker_pos - ref_pos) * self.position_scale
        return init_ee_pos + delta

    def compute_wrist_poses(
        self, arm_poses: dict
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        if not self.calibrated:
            return None, None
        if not arm_poses["left"]["valid"] or not arm_poses["right"]["valid"]:
            return None, None

        s = self.position_scale

        # Position delta (already in robot frame from XrcWrapper)
        left_delta  = (arm_poses["left"]["position"]  - self.ref_left_pos)  * s
        right_delta = (arm_poses["right"]["position"] - self.ref_right_pos) * s

        # Relative rotation (no additional frame rotation needed)
        left_rel_rot  = arm_poses["left"]["rotation"]  @ self.ref_left_rot.T
        right_rel_rot = arm_poses["right"]["rotation"] @ self.ref_right_rot.T

        left_wrist  = np.eye(4)
        left_wrist[:3, :3]  = left_rel_rot  @ self.init_left_wrist[:3, :3]
        left_wrist[:3,  3]  = self.init_left_wrist[:3, 3]  + left_delta

        right_wrist = np.eye(4)
        right_wrist[:3, :3] = right_rel_rot @ self.init_right_wrist[:3, :3]
        right_wrist[:3,  3] = self.init_right_wrist[:3, 3] + right_delta

        return left_wrist, right_wrist

    # ── IK + Rerun logging ────────────────────────────────────────────────

    def solve_and_visualize(self, arm_poses: dict) -> np.ndarray | None:
        left_wrist, right_wrist = self.compute_wrist_poses(arm_poses)
        if left_wrist is None:
            return None

        left_target  = left_wrist[:3,  3]
        right_target = right_wrist[:3, 3]

        rr.log("targets/left",  rr.Points3D([left_target],  colors=[[0,   200, 255]], radii=[0.015]))
        rr.log("targets/right", rr.Points3D([right_target], colors=[[255, 150,   0]], radii=[0.015]))

        sol_q, _ = self.arm_ik.solve_ik(left_wrist, right_wrist)
        self.update_robot_pose(sol_q)

        pin = self.pin
        pin.forwardKinematics(
            self.arm_ik.reduced_robot.model,
            self.arm_ik.reduced_robot.data,
            np.array(sol_q).flatten(),
        )
        pin.updateFramePlacements(self.arm_ik.reduced_robot.model, self.arm_ik.reduced_robot.data)
        left_fk  = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.L_hand_id].translation.copy()
        right_fk = self.arm_ik.reduced_robot.data.oMf[self.arm_ik.R_hand_id].translation.copy()

        rr.log("robot/left_ee",  rr.Points3D([left_fk.tolist()],  colors=[[0, 255, 100]], radii=[0.012]))
        rr.log("robot/right_ee", rr.Points3D([right_fk.tolist()], colors=[[0, 255, 100]], radii=[0.012]))

        rr.log("targets/left_error_line",  rr.LineStrips3D([[left_target.tolist(),  left_fk.tolist()]],  colors=[[255, 50, 50, 180]]))
        rr.log("targets/right_error_line", rr.LineStrips3D([[right_target.tolist(), right_fk.tolist()]], colors=[[255, 50, 50, 180]]))

        left_err  = np.linalg.norm(left_target  - left_fk)
        right_err = np.linalg.norm(right_target - right_fk)
        rr.log("metrics/ik_error_left_mm",  rr.Scalar(left_err  * 1000))
        rr.log("metrics/ik_error_right_mm", rr.Scalar(right_err * 1000))

        for i, name in enumerate(self.ARM_JOINT_NAMES):
            short = name.replace("_joint", "").replace("left_", "L_").replace("right_", "R_")
            rr.log(f"joints/{short}", rr.Scalar(float(np.degrees(sol_q[i]))))

        return sol_q


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    global CALIBRATE, START, STOP

    parser = argparse.ArgumentParser(
        description="XRoboToolkit SDK Motion Tracker Visualization (Rerun)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--robot",          action="store_true",       help="Show robot arm with real-time IK")
    parser.add_argument("--arm",            default="H2",              choices=list(RobotVisualizer.IK_CLASS_MAP), help="Robot arm type (only used with --robot)")
    parser.add_argument("--position-scale", type=float, default=0.8,   help="Scale factor for tracker position deltas")
    parser.add_argument("--no-adb",         action="store_true",        help="Skip ADB tunnel (Pico on Wi-Fi)")
    parser.add_argument("--left-serial",    type=str,   default=None,  help="Force serial for left wrist tracker")
    parser.add_argument("--right-serial",   type=str,   default=None,  help="Force serial for right wrist tracker")
    args = parser.parse_args()

    print("=" * 60)
    print("XRoboToolkit SDK — Motion Tracker Visualization (Rerun)")
    print("=" * 60)

    # ── Init XrcWrapper ──────────────────────────────────────────────────
    xrc = XrcWrapper(
        use_adb=not args.no_adb,
        left_serial=args.left_serial,
        right_serial=args.right_serial,
        position_scale=args.position_scale,
    )
    xrc.start()

    # ── Optionally load robot ────────────────────────────────────────────
    robot_viz = None
    if args.robot:
        try:
            robot_viz = RobotVisualizer(arm=args.arm, position_scale=args.position_scale)
            print(f"Robot {args.arm} loaded.")
        except Exception as e:
            import traceback
            print(f"Failed to load robot: {e}")
            traceback.print_exc()

    # ── Rerun scene setup ────────────────────────────────────────────────
    rr.init("xrc_tracker_viz", spawn=True)
    # XrcWrapper already outputs Z-up robot frame
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    grid_size = 1.5
    grid_lines = []
    for val in np.arange(-grid_size, grid_size + 0.25, 0.25):
        grid_lines.append([[val, -grid_size, 0], [val,  grid_size, 0]])
        grid_lines.append([[-grid_size, val, 0], [grid_size, val, 0]])
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

    # ── Keyboard listener ────────────────────────────────────────────────
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

    print("\nVisualization running in Rerun viewer.")
    if robot_viz:
        print("Controls:  [c] calibrate  [r] start IK  [q] quit")
    print("Press Ctrl+C to exit.\n")

    left_trail:  list[list] = []
    right_trail: list[list] = []
    max_trail = 100
    frame     = 0
    calibrated = False

    try:
        while not STOP:
            rr.set_time_sequence("frame", frame)

            xrc.update()
            arm_poses  = xrc.get_arm_poses()
            headset_T  = xrc.get_headset_pose()
            left       = arm_poses["left"]
            right      = arm_poses["right"]

            # ── Calibration ──────────────────────────────────────────
            if CALIBRATE and robot_viz:
                CALIBRATE = False
                if robot_viz.calibrate(arm_poses):
                    calibrated = True
                    left_trail.clear()
                    right_trail.clear()
                    print("Calibrated! Press [r] to start IK.")

            tracking = robot_viz is not None and calibrated and START

            # ── Tracker positions ─────────────────────────────────────
            # Poses are already in robot frame — no extra rotation needed.
            for side, pose, trail, color in (
                ("left",  left,  left_trail,  [0,   150, 255]),
                ("right", right, right_trail, [255, 150,   0]),
            ):
                if not pose["valid"]:
                    continue

                if tracking:
                    ref_pos     = robot_viz.ref_left_pos  if side == "left" else robot_viz.ref_right_pos
                    init_ee_pos = robot_viz.init_left_wrist[:3, 3] if side == "left" else robot_viz.init_right_wrist[:3, 3]
                    pos = robot_viz.tracker_to_robot_pos(pose["position"], ref_pos, init_ee_pos).tolist()
                else:
                    pos = pose["position"].tolist()

                rr.log(f"trackers/{side}/pos", rr.Points3D([pos], colors=[color], radii=[0.02]))
                trail.append(pos)
                if len(trail) > max_trail:
                    trail.pop(0)
                if len(trail) > 1:
                    rr.log(f"trackers/{side}/trail", rr.LineStrips3D([trail], colors=[color + [100]]))

            # ── Wrist connection line ─────────────────────────────────
            if left["valid"] and right["valid"]:
                if tracking:
                    lp = robot_viz.tracker_to_robot_pos(left["position"],  robot_viz.ref_left_pos,  robot_viz.init_left_wrist[:3, 3]).tolist()
                    rp = robot_viz.tracker_to_robot_pos(right["position"], robot_viz.ref_right_pos, robot_viz.init_right_wrist[:3, 3]).tolist()
                else:
                    lp = left["position"].tolist()
                    rp = right["position"].tolist()
                rr.log("trackers/connection", rr.LineStrips3D([[lp, rp]], colors=[[200, 200, 200, 100]]))
                rr.log("metrics/wrist_distance", rr.Scalar(np.linalg.norm(np.array(lp) - np.array(rp))))

            # ── Headset pose ──────────────────────────────────────────
            if headset_T is not None:
                hp = headset_T[:3, 3]
                rr.log("trackers/headset", rr.Points3D([hp.tolist()], colors=[[180, 100, 255]], radii=[0.025]))
                rr.log(
                    "trackers/headset_frame",
                    rr.Transform3D(
                        translation=hp.tolist(),
                        mat3x3=headset_T[:3, :3].flatten().tolist(),
                    ),
                )

            # ── Velocity metrics ──────────────────────────────────────
            if left["valid"]:
                rr.log("metrics/left_velocity",  rr.Scalar(np.linalg.norm(left["velocity"])))
            if right["valid"]:
                rr.log("metrics/right_velocity", rr.Scalar(np.linalg.norm(right["velocity"])))

            # ── Tracker count ─────────────────────────────────────────
            rr.log("metrics/num_trackers", rr.Scalar(xrc.num_trackers))

            # ── Robot IK ─────────────────────────────────────────────
            if tracking and left["valid"] and right["valid"]:
                robot_viz.solve_and_visualize(arm_poses)

            frame += 1
            time.sleep(0.033)   # ~30 Hz

    except KeyboardInterrupt:
        print("\nExiting…")
    finally:
        if has_keyboard:
            try:
                stop_listening()
            except Exception:
                pass
        xrc.stop()
        print("Done.")


if __name__ == "__main__":
    main()
