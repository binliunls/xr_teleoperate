"""
ROS 2 image-topic client that mirrors ImageClient's interface.

Subscribes to four sensor_msgs/Image topics on CycloneDDS and exposes them
through `get_head_frame()` / `get_left_wrist_frame()` / `get_right_wrist_frame()`
so teleop_hand_and_arm.py can use it as a drop-in replacement for the ZMQ
ImageClient.

Topics (default):
    /head/left/image_raw
    /head/right/image_raw
    /wrist/left/image_raw
    /wrist/right/image_raw

The two head topics are stereo eyes; this client horizontally concatenates them
into one frame so the existing `binocular=True` recording path (which splits
`head_img.bgr` down the middle) keeps working without modification.
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


DEFAULT_HEAD_LEFT_TOPIC = "/head/left/image_raw"
DEFAULT_HEAD_RIGHT_TOPIC = "/head/right/image_raw"
DEFAULT_WRIST_LEFT_TOPIC = "/wrist/left/image_raw"
DEFAULT_WRIST_RIGHT_TOPIC = "/wrist/right/image_raw"

DEFAULT_RMW = "rmw_cyclonedds_cpp"


# ── frame wrapper, compatible with teleimager.TeleImage ──────────────────────


@dataclass
class ROSFrame:
    """Minimal duck-typed stand-in for teleimager.TeleImage."""

    fps: float = 0.0
    bgr_array: Optional[np.ndarray] = None
    jpg: Optional[bytes] = None  # not used by ROS path; kept for interface parity
    camera_timestamps: dict = field(default_factory=dict)

    @property
    def bgr(self) -> Optional[np.ndarray]:
        return self.bgr_array

    def __bool__(self) -> bool:
        return self.bgr_array is not None


# ── image decode (sensor_msgs/Image → BGR ndarray) ───────────────────────────


def _msg_to_bgr(msg) -> Optional[np.ndarray]:
    """Convert a sensor_msgs/Image into a contiguous BGR ndarray."""
    encoding = msg.encoding.lower()
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected = msg.height * msg.step

    if encoding in ("rgb8", "bgr8"):
        if raw.size < expected:
            return None
        frame = raw[:expected].reshape(msg.height, msg.step)[:, : msg.width * 3]
        frame = frame.reshape(msg.height, msg.width, 3)
        if encoding == "rgb8":
            frame = frame[..., ::-1]  # cheap RGB→BGR without cv2 dep
        return np.ascontiguousarray(frame)

    if encoding in ("mono8", "8uc1"):
        if raw.size < expected:
            return None
        gray = raw[:expected].reshape(msg.height, msg.step)[:, : msg.width]
        return np.repeat(gray[:, :, None], 3, axis=2)  # grayscale → 3-channel

    return None


# ── ROS subscriber thread ────────────────────────────────────────────────────


class _Subscriber:
    """Holds the latest frame for one topic, plus an FPS estimator."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.latest: Optional[np.ndarray] = None
        self.latest_timestamps: dict = {}
        self.timestamps: list = []
        self.height = 0
        self.width = 0

    def push(
        self,
        frame: np.ndarray,
        *,
        source_timestamp_ns: Optional[int],
        workstation_receive_monotonic_ns: int,
        workstation_receive_realtime_ns: int,
    ) -> None:
        now = time.monotonic()
        with self.lock:
            self.latest = frame
            self.latest_timestamps = {
                # Preserve the ROS header clock verbatim. It is not assumed to
                # share a clock domain with workstation monotonic/realtime.
                "ros_header_stamp_ns": source_timestamp_ns,
                "workstation_receive_monotonic_ns": workstation_receive_monotonic_ns,
                "workstation_receive_realtime_ns": workstation_receive_realtime_ns,
            }
            self.height, self.width = frame.shape[:2]
            self.timestamps.append(now)
            cutoff = now - 1.0
            while self.timestamps and self.timestamps[0] < cutoff:
                self.timestamps.pop(0)

    def snapshot(self) -> tuple:
        with self.lock:
            return self.latest, float(len(self.timestamps)), dict(self.latest_timestamps)


# ── client class ─────────────────────────────────────────────────────────────


class ROSImageClient:
    """Drop-in replacement for teleimager.ImageClient backed by ROS 2 topics."""

    def __init__(
        self,
        head_left_topic: str = DEFAULT_HEAD_LEFT_TOPIC,
        head_right_topic: str = DEFAULT_HEAD_RIGHT_TOPIC,
        wrist_left_topic: str = DEFAULT_WRIST_LEFT_TOPIC,
        wrist_right_topic: str = DEFAULT_WRIST_RIGHT_TOPIC,
        cyclonedds_uri: Optional[str] = None,
        queue_size: int = 2,
        warmup_timeout: float = 5.0,
        require_warmup: bool = False,
        head_mode: str = "binocular",
    ) -> None:
        if head_mode not in ("binocular", "mono"):
            raise ValueError(f"head_mode must be 'binocular' or 'mono', got {head_mode!r}")
        self._head_mode = head_mode

        # configure RMW before importing rclpy
        if cyclonedds_uri:
            # An explicit deployment mapping must override stale shell-wide DDS
            # settings left from the previous Thor address.
            os.environ["RMW_IMPLEMENTATION"] = DEFAULT_RMW
            os.environ["ROS_DOMAIN_ID"] = "0"
            os.environ["CYCLONEDDS_URI"] = cyclonedds_uri
        else:
            os.environ.setdefault("RMW_IMPLEMENTATION", DEFAULT_RMW)

        try:
            import rclpy
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            from sensor_msgs.msg import Image
        except ImportError as exc:
            raise RuntimeError(
                "ROSImageClient requires ROS 2 (rclpy + sensor_msgs). "
                "Source ROS first, e.g. `source /opt/ros/jazzy/setup.bash`."
            ) from exc

        # In mono mode, skip head/right entirely (no subscription, no multicast join).
        self._topics = {
            "head_left": head_left_topic,
            "wrist_left": wrist_left_topic,
            "wrist_right": wrist_right_topic,
        }
        if head_mode == "binocular":
            self._topics["head_right"] = head_right_topic
        self._subs: dict[str, _Subscriber] = {name: _Subscriber() for name in self._topics}

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._node = rclpy.create_node("xr_teleop_ros_image_client")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_size,
        )

        def _make_cb(name: str):
            def _cb(msg):
                receive_monotonic_ns = time.monotonic_ns()
                receive_realtime_ns = time.time_ns()
                source_timestamp_ns = None
                header = getattr(msg, "header", None)
                stamp = getattr(header, "stamp", None)
                if stamp is not None and hasattr(stamp, "sec") and hasattr(stamp, "nanosec"):
                    source_timestamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                frame = _msg_to_bgr(msg)
                if frame is None:
                    return
                self._subs[name].push(
                    frame,
                    source_timestamp_ns=source_timestamp_ns,
                    workstation_receive_monotonic_ns=receive_monotonic_ns,
                    workstation_receive_realtime_ns=receive_realtime_ns,
                )

            return _cb

        for name, topic in self._topics.items():
            self._node.create_subscription(Image, topic, _make_cb(name), qos)
            self._node.get_logger().info(f"[ros_image_client] subscribed: {topic}")

        self._stop = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin_loop, name="ros-image-client-spin", daemon=True)
        self._spin_thread.start()

        # Wait for at least one frame on each head topic so get_cam_config() can
        # report the actual image shape. Wrists are optional.
        if warmup_timeout > 0:
            self._warmup_head(warmup_timeout, require=require_warmup)

    # ── internal ─────────────────────────────────────────────────────────────

    def _spin_loop(self) -> None:
        while not self._stop.is_set():
            self._rclpy.spin_once(self._node, timeout_sec=0.1)

    def _warmup_head(self, timeout: float, require: bool) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            left, _, _ = self._subs["head_left"].snapshot()
            if self._head_mode == "mono":
                if left is not None:
                    return
            else:
                right, _, _ = self._subs["head_right"].snapshot()
                if left is not None and right is not None:
                    return
            time.sleep(0.05)
        if require:
            if self._head_mode == "mono":
                raise RuntimeError(
                    f"Did not receive head/left within {timeout:.1f}s "
                    f"(left={self._topics['head_left']})"
                )
            raise RuntimeError(
                f"Did not receive both head topics within {timeout:.1f}s "
                f"(left={self._topics['head_left']}, right={self._topics['head_right']})"
            )

    # ── public API (mirrors ImageClient) ─────────────────────────────────────

    def get_cam_config(self) -> dict:
        """Return a config dict shaped like teleimager.cam_config_client.yaml."""
        # Use head_left shape as the per-eye dimensions; binocular doubles the width.
        hl_frame, _, _ = self._subs["head_left"].snapshot()
        if hl_frame is not None:
            h, w = hl_frame.shape[:2]
        else:
            h, w = 480, 640
        if self._head_mode == "binocular":
            head_image_shape = [h, w * 2]  # stitched stereo width
            head_binocular = True
        else:  # mono
            head_image_shape = [h, w]
            head_binocular = False

        wl_frame, _, _ = self._subs["wrist_left"].snapshot()
        if wl_frame is not None:
            wh, ww = wl_frame.shape[:2]
            wrist_image_shape = [wh, ww]
        else:
            wrist_image_shape = [480, 640]

        return {
            "head_camera": {
                "enable_zmq": True,  # keeps existing teleop branches active
                "enable_webrtc": False,
                "binocular": head_binocular,
                "image_shape": head_image_shape,
                "fps": 30,
                "type": "ros",
                "zmq_port": 0,
                "webrtc_port": 0,
            },
            "left_wrist_camera": {
                "enable_zmq": True,
                "enable_webrtc": False,
                "binocular": False,
                "image_shape": wrist_image_shape,
                "fps": 30,
                "type": "ros",
                "zmq_port": 0,
                "webrtc_port": 0,
            },
            "right_wrist_camera": {
                "enable_zmq": True,
                "enable_webrtc": False,
                "binocular": False,
                "image_shape": wrist_image_shape,
                "fps": 30,
                "type": "ros",
                "zmq_port": 0,
                "webrtc_port": 0,
            },
        }

    def get_head_frame(self) -> ROSFrame:
        left_frame, l_fps, left_timestamps = self._subs["head_left"].snapshot()
        if self._head_mode == "mono":
            if left_frame is None:
                return ROSFrame(fps=0.0, bgr_array=None)
            return ROSFrame(
                fps=l_fps,
                bgr_array=left_frame,
                camera_timestamps={"head_left": left_timestamps},
            )
        right_frame, r_fps, right_timestamps = self._subs["head_right"].snapshot()
        if left_frame is None or right_frame is None:
            return ROSFrame(fps=0.0, bgr_array=None)
        # Resize right to match left if dims drift (shouldn't normally).
        if right_frame.shape != left_frame.shape:
            try:
                import cv2

                right_frame = cv2.resize(right_frame, (left_frame.shape[1], left_frame.shape[0]))
            except Exception:
                return ROSFrame(fps=0.0, bgr_array=None)
        stitched = np.concatenate((left_frame, right_frame), axis=1)
        return ROSFrame(
            fps=min(l_fps, r_fps),
            bgr_array=stitched,
            camera_timestamps={
                "head_left": left_timestamps,
                "head_right": right_timestamps,
            },
        )

    def get_left_wrist_frame(self) -> ROSFrame:
        frame, fps, timestamps = self._subs["wrist_left"].snapshot()
        return ROSFrame(
            fps=fps,
            bgr_array=frame,
            camera_timestamps={"wrist_left": timestamps} if frame is not None else {},
        )

    def get_right_wrist_frame(self) -> ROSFrame:
        frame, fps, timestamps = self._subs["wrist_right"].snapshot()
        return ROSFrame(
            fps=fps,
            bgr_array=frame,
            camera_timestamps={"wrist_right": timestamps} if frame is not None else {},
        )

    def close(self) -> None:
        self._stop.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass
