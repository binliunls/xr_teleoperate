#!/usr/bin/env python3
"""
Real-time 3D visualization of ALVR Motion Trackers using Rerun.

Usage:
    python visualize_trackers.py
"""

import time
import sys
import numpy as np

try:
    import rerun as rr
except ImportError:
    print("Error: rerun-sdk not installed. Install with: pip install rerun-sdk")
    sys.exit(1)

from alvr_tracker_bridge import ALVRTrackerBridge


def create_coordinate_axes(length: float = 0.1) -> tuple:
    """Create coordinate axis arrows for visualization"""
    origins = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
    vectors = np.array(
        [
            [length, 0, 0],  # X - red
            [0, length, 0],  # Y - green
            [0, 0, length],  # Z - blue
        ]
    )
    colors = np.array(
        [
            [255, 0, 0],  # X - red
            [0, 255, 0],  # Y - green
            [0, 0, 255],  # Z - blue
        ]
    )
    return origins, vectors, colors


def main():
    print("Initializing Motion Tracker Visualization...")
    print("=" * 60)

    # Initialize tracker bridge
    bridge = ALVRTrackerBridge()
    if not bridge.initialize():
        print("\nFailed to initialize tracker bridge!")
        print("Make sure ALVR is running and PICO is connected.")
        sys.exit(1)

    # Initialize Rerun with desktop viewer
    rr.init("motion_tracker_viz", spawn=True)

    # Set up the 3D view
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)

    # Log ground plane grid
    grid_size = 2.0
    grid_lines = []
    for i in np.linspace(-grid_size, grid_size, 21):
        grid_lines.append([[i, 0, -grid_size], [i, 0, grid_size]])
        grid_lines.append([[-grid_size, 0, i], [grid_size, 0, i]])
    rr.log("world/ground", rr.LineStrips3D(grid_lines, colors=[[100, 100, 100, 100]]), static=True)

    # Log origin axes
    rr.log(
        "world/origin",
        rr.Arrows3D(
            origins=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
            vectors=[[0.3, 0, 0], [0, 0.3, 0], [0, 0, 0.3]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
        ),
        static=True,
    )
    rr.log("world/origin/label", rr.TextLog("Origin"), static=True)

    print("\nVisualization started!")
    print("Move your wrist trackers to see them in 3D.")
    print("Press Ctrl+C to exit.\n")

    # Trail history for motion paths
    left_trail = []
    right_trail = []
    max_trail_length = 100

    frame = 0
    try:
        while True:
            rr.set_time_sequence("frame", frame)

            # Update tracker data
            bridge.update()
            arm_poses = bridge.get_arm_poses()

            # Visualize left wrist tracker
            left = arm_poses["left"]
            if left["valid"]:
                pos = left["position"]
                rot = left["rotation"]

                # Log position as point
                rr.log("trackers/left_wrist/position", rr.Points3D([pos], colors=[[0, 150, 255]], radii=[0.03]))

                # Log orientation axes
                axis_length = 0.08
                x_axis = rot @ np.array([axis_length, 0, 0])
                y_axis = rot @ np.array([0, axis_length, 0])
                z_axis = rot @ np.array([0, 0, axis_length])

                rr.log(
                    "trackers/left_wrist/axes",
                    rr.Arrows3D(
                        origins=[pos, pos, pos],
                        vectors=[x_axis, y_axis, z_axis],
                        colors=[[255, 100, 100], [100, 255, 100], [100, 100, 255]],
                    ),
                )

                # Add to trail
                left_trail.append(pos.copy())
                if len(left_trail) > max_trail_length:
                    left_trail.pop(0)

                # Log trail
                if len(left_trail) > 1:
                    rr.log("trackers/left_wrist/trail", rr.LineStrips3D([left_trail], colors=[[0, 150, 255, 150]]))

                # Log text info
                rr.log(
                    "trackers/left_wrist/info",
                    rr.TextLog(f"Left: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]"),
                )

            # Visualize right wrist tracker
            right = arm_poses["right"]
            if right["valid"]:
                pos = right["position"]
                rot = right["rotation"]

                # Log position as point
                rr.log("trackers/right_wrist/position", rr.Points3D([pos], colors=[[255, 150, 0]], radii=[0.03]))

                # Log orientation axes
                axis_length = 0.08
                x_axis = rot @ np.array([axis_length, 0, 0])
                y_axis = rot @ np.array([0, axis_length, 0])
                z_axis = rot @ np.array([0, 0, axis_length])

                rr.log(
                    "trackers/right_wrist/axes",
                    rr.Arrows3D(
                        origins=[pos, pos, pos],
                        vectors=[x_axis, y_axis, z_axis],
                        colors=[[255, 100, 100], [100, 255, 100], [100, 100, 255]],
                    ),
                )

                # Add to trail
                right_trail.append(pos.copy())
                if len(right_trail) > max_trail_length:
                    right_trail.pop(0)

                # Log trail
                if len(right_trail) > 1:
                    rr.log("trackers/right_wrist/trail", rr.LineStrips3D([right_trail], colors=[[255, 150, 0, 150]]))

                # Log text info
                rr.log(
                    "trackers/right_wrist/info",
                    rr.TextLog(f"Right: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]"),
                )

            # Log line connecting both wrists (if both valid)
            if left["valid"] and right["valid"]:
                rr.log(
                    "trackers/connection",
                    rr.LineStrips3D(
                        [[left["position"], right["position"]]],
                        colors=[[200, 200, 200, 150]],
                    ),
                )

                # Log distance between wrists
                distance = np.linalg.norm(left["position"] - right["position"])
                rr.log("metrics/wrist_distance", rr.Scalar(distance))

            # Log velocities
            if left["valid"]:
                left_vel = np.linalg.norm(left["velocity"])
                rr.log("metrics/left_velocity", rr.Scalar(left_vel))
            if right["valid"]:
                right_vel = np.linalg.norm(right["velocity"])
                rr.log("metrics/right_velocity", rr.Scalar(right_vel))

            frame += 1
            time.sleep(0.033)  # ~30 FPS

    except KeyboardInterrupt:
        print("\n\nExiting...")
    finally:
        bridge.shutdown()
        print("Visualization closed.")


if __name__ == "__main__":
    main()
