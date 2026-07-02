#!/usr/bin/env python3
"""Live OpenCV visualizer for the head and wrist camera ROS image topics.

Reuses the CycloneDDS configuration from `ros_image_monitor` so the receiver
matches the Jetson publisher out of the box. Run on the x86 host (or the
Jetson itself) with ROS 2 Jazzy sourced.
"""

import argparse
import math
import signal
import sys
import threading
import time

import numpy as np

from ros_image_monitor import (
    DEFAULT_IMAGE_TOPICS,
    configure_default_rmw,
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topics",
        nargs="+",
        default=list(DEFAULT_IMAGE_TOPICS),
        help="Image topics to visualize (default: all four head/wrist streams).",
    )
    parser.add_argument(
        "--separate",
        action="store_true",
        help="Show each topic in its own window instead of a tiled mosaic.",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Scale factor applied to each frame before display (default: 1.0).",
    )
    parser.add_argument(
        "--queue-size",
        type=int,
        default=2,
        help="rclpy subscriber queue depth (default: 2).",
    )
    return parser.parse_args(argv)


def image_msg_to_bgr(msg):
    """Convert a sensor_msgs/Image into an OpenCV BGR ndarray."""
    encoding = msg.encoding.lower()
    raw = np.frombuffer(msg.data, dtype=np.uint8)

    if encoding in ("rgb8", "bgr8"):
        channels = 3
        expected = msg.height * msg.step
        if raw.size < expected:
            return None
        # honor row stride (step) which may pad past width*channels
        frame = raw[:expected].reshape(msg.height, msg.step)[:, : msg.width * channels]
        frame = frame.reshape(msg.height, msg.width, channels)
        if encoding == "rgb8":
            import cv2

            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    if encoding in ("mono8", "8uc1"):
        expected = msg.height * msg.step
        if raw.size < expected:
            return None
        frame = raw[:expected].reshape(msg.height, msg.step)[:, : msg.width]
        import cv2

        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    return None


class FrameBuffer:
    """Thread-safe slot for the latest decoded frame per topic."""

    def __init__(self, topics):
        self.lock = threading.Lock()
        self.frames = {topic: None for topic in topics}
        self.counts = {topic: 0 for topic in topics}
        self.last_recv = {topic: 0.0 for topic in topics}

    def set(self, topic, frame):
        now = time.monotonic()
        with self.lock:
            self.frames[topic] = frame
            self.counts[topic] += 1
            self.last_recv[topic] = now

    def snapshot(self):
        with self.lock:
            return (
                {topic: frame for topic, frame in self.frames.items()},
                dict(self.counts),
                dict(self.last_recv),
            )


def annotate(frame, label, fps):
    import cv2

    overlay = frame.copy()
    text = f"{label}  {fps:5.1f} fps"
    cv2.rectangle(overlay, (0, 0), (frame.shape[1], 28), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
    cv2.putText(
        frame,
        text,
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return frame


def build_mosaic(frames_in_order):
    """Tile a list of (label, frame) into an approximately-square grid."""
    import cv2

    frames = [(label, f) for label, f in frames_in_order if f is not None]
    if not frames:
        return None
    n = len(frames)
    cols = int(math.ceil(math.sqrt(n)))
    rows = int(math.ceil(n / cols))

    cell_h = max(f.shape[0] for _, f in frames)
    cell_w = max(f.shape[1] for _, f in frames)
    canvas = np.zeros((rows * cell_h, cols * cell_w, 3), dtype=np.uint8)

    for idx, (_, frame) in enumerate(frames):
        r, c = divmod(idx, cols)
        if frame.shape[0] != cell_h or frame.shape[1] != cell_w:
            frame = cv2.resize(frame, (cell_w, cell_h))
        canvas[r * cell_h : (r + 1) * cell_h, c * cell_w : (c + 1) * cell_w] = frame
    return canvas


def main(argv=None):
    args = parse_args(argv)

    try:
        import cv2  # noqa: F401  (imported lazily inside helpers, but fail fast here)
    except ImportError:
        sys.exit("OpenCV is required for the visualizer. Install it with:\n" "  sudo apt install python3-opencv")

    configure_default_rmw()

    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image
    except ImportError as exc:
        sys.exit(
            "Failed to import ROS 2 Python modules. Source ROS first, e.g.:\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            f"Original import error: {exc}"
        )

    buffer = FrameBuffer(args.topics)

    rclpy.init()
    node = rclpy.create_node("camera_stream_visualizer")

    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=args.queue_size,
    )

    def make_callback(topic):
        def _cb(msg):
            frame = image_msg_to_bgr(msg)
            if frame is None:
                node.get_logger().warn(
                    f"Unsupported encoding on {topic}: {msg.encoding}",
                    throttle_duration_sec=5.0,
                )
                return
            if args.scale != 1.0:
                import cv2

                frame = cv2.resize(
                    frame,
                    None,
                    fx=args.scale,
                    fy=args.scale,
                    interpolation=cv2.INTER_AREA,
                )
            buffer.set(topic, frame)

        return _cb

    for topic in args.topics:
        node.create_subscription(Image, topic, make_callback(topic), qos)
        node.get_logger().info(f"Subscribed to {topic}")

    stop = threading.Event()

    def _request_stop(_signum=None, _frame=None):
        stop.set()

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), name="rclpy-spin", daemon=True)
    spin_thread.start()

    fps_window_start = time.monotonic()
    last_counts = {topic: 0 for topic in args.topics}
    fps = {topic: 0.0 for topic in args.topics}

    print(
        f"[viz] visualizing {len(args.topics)} topic(s); press q or Esc to quit",
        flush=True,
    )

    try:
        while not stop.is_set():
            frames, counts, _last_recv = buffer.snapshot()

            now = time.monotonic()
            elapsed = now - fps_window_start
            if elapsed >= 1.0:
                for topic in args.topics:
                    fps[topic] = (counts[topic] - last_counts[topic]) / elapsed
                last_counts = counts
                fps_window_start = now

            import cv2

            labeled = []
            for topic in args.topics:
                frame = frames.get(topic)
                if frame is None:
                    continue
                labeled.append((topic, annotate(frame.copy(), topic, fps[topic])))

            if args.separate:
                for topic, frame in labeled:
                    cv2.imshow(topic, frame)
            else:
                mosaic = build_mosaic(labeled)
                if mosaic is not None:
                    cv2.imshow("camera streams", mosaic)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                stop.set()
    finally:
        try:
            import cv2

            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
