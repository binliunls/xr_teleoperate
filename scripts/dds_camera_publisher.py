#!/usr/bin/env python3
"""Publish H2's two Argus and two V4L2 cameras over native CycloneDDS.

Frames are JPEG compressed before DDS publication. QoS is best-effort with a
depth of one so slow consumers receive the newest image instead of a backlog.
This process does not import ROS or Unitree SDK2.
"""

import argparse
import os
import signal
import threading
import time
from dataclasses import dataclass
from xml.sax.saxutils import escape

import cv2
import numpy as np
from cyclonedds.core import Policy, Qos
from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import sequence, uint8, uint32, uint64
from cyclonedds.pub import DataWriter, Publisher
from cyclonedds.topic import Topic

try:
    import gi

    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
except (ImportError, ValueError) as exc:
    raise RuntimeError(
        "PyGObject/GStreamer introspection is required; install "
        "python3-gi and gir1.2-gstreamer-1.0"
    ) from exc


@dataclass
class CameraFrame(IdlStruct, typename="h2_camera::CameraFrame"):
    sequence_id: uint64
    timestamp_ns: uint64
    width: uint32
    height: uint32
    encoding: str
    data: sequence[uint8]


CAMERAS = {
    "head_left": "h2/camera/head_left/compressed",
    "head_right": "h2/camera/head_right/compressed",
    "wrist_left": "h2/camera/wrist_left/compressed",
    "wrist_right": "h2/camera/wrist_right/compressed",
}


def argus_branch(sensor_id: int, name: str, width: int, height: int, fps: int) -> str:
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM),width=2560,height=1984,format=NV12,"
        f"framerate={fps}/1 ! "
        f"nvvidconv ! video/x-raw,width={width},height={height},format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        f"appsink name={name} emit-signals=true drop=true max-buffers=1 "
        "sync=false async=false"
    )


class DualArgusCapture:
    """Read two Argus sensors from one GStreamer pipeline."""

    def __init__(
        self,
        left_sensor: int,
        right_sensor: int,
        width: int,
        height: int,
        fps: int,
    ) -> None:
        Gst.init(None)
        description = " ".join(
            (
                argus_branch(left_sensor, "head_left_sink", width, height, fps),
                argus_branch(right_sensor, "head_right_sink", width, height, fps),
            )
        )
        self.pipeline = Gst.parse_launch(description)
        self.sinks = {
            "head_left": self.pipeline.get_by_name("head_left_sink"),
            "head_right": self.pipeline.get_by_name("head_right_sink"),
        }
        if any(sink is None for sink in self.sinks.values()):
            raise RuntimeError("failed to create both Argus appsinks")
        self._condition = threading.Condition()
        self._latest: dict[str, np.ndarray | None] = {
            "head_left": None,
            "head_right": None,
        }
        self._sequence = {"head_left": 0, "head_right": 0}
        self._consumed = {"head_left": 0, "head_right": 0}
        self._release_lock = threading.Lock()
        self._released = False
        for name, sink in self.sinks.items():
            sink.connect("new-sample", self._on_new_sample, name)
        result = self.pipeline.set_state(Gst.State.PLAYING)
        if result == Gst.StateChangeReturn.FAILURE:
            self.pipeline.set_state(Gst.State.NULL)
            raise RuntimeError("failed to start the dual Argus GStreamer pipeline")

    def _on_new_sample(self, sink, name: str):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR
        caps = sample.get_caps().get_structure(0)
        width = caps.get_value("width")
        height = caps.get_value("height")
        buffer = sample.get_buffer()
        data = buffer.extract_dup(0, buffer.get_size())
        frame = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3)).copy()
        with self._condition:
            self._latest[name] = frame
            self._sequence[name] += 1
            self._condition.notify_all()
        return Gst.FlowReturn.OK

    def read(self, name: str) -> tuple[bool, np.ndarray | None]:
        deadline = time.monotonic() + 1.0
        with self._condition:
            while (
                self._sequence[name] <= self._consumed[name]
                and not self._released
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False, None
                self._condition.wait(remaining)
            if self._released or self._latest[name] is None:
                return False, None
            self._consumed[name] = self._sequence[name]
            return True, self._latest[name]

    def release(self) -> None:
        with self._release_lock:
            if not self._released:
                with self._condition:
                    self._released = True
                    self._condition.notify_all()
                self.pipeline.set_state(Gst.State.NULL)


class ArgusStream:
    def __init__(self, capture: DualArgusCapture, name: str) -> None:
        self.capture = capture
        self.name = name

    def read(self) -> tuple[bool, np.ndarray | None]:
        return self.capture.read(self.name)

    def release(self) -> None:
        self.capture.release()


def open_wrist(device: str, width: int, height: int, fps: int) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        raise RuntimeError(f"failed to open V4L2 device {device}")
    return capture


def publish_camera(
    name: str,
    capture: cv2.VideoCapture,
    writer: DataWriter,
    output_width: int,
    output_height: int,
    quality: int,
    stop: threading.Event,
) -> None:
    sequence_id = 0
    failures = 0
    report_at = time.monotonic() + 5.0
    report_count = 0
    while not stop.is_set():
        ok, frame = capture.read()
        if not ok or frame is None:
            failures += 1
            if failures >= 30:
                print(f"[ERROR] {name}: 30 consecutive capture failures", flush=True)
                stop.set()
                break
            time.sleep(0.01)
            continue
        failures = 0
        if frame.shape[1] != output_width or frame.shape[0] != output_height:
            frame = cv2.resize(frame, (output_width, output_height), interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            continue
        height, width = frame.shape[:2]
        writer.write(
            CameraFrame(
                sequence_id=sequence_id,
                timestamp_ns=time.time_ns(),
                width=width,
                height=height,
                encoding="jpeg",
                data=encoded.tobytes(),
            )
        )
        sequence_id += 1
        report_count += 1
        now = time.monotonic()
        if now >= report_at:
            print(f"[INFO] {name}: {report_count / 5.0:.1f} fps, last JPEG {encoded.size / 1024:.0f} KiB", flush=True)
            report_count = 0
            report_at = now + 5.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=10)
    parser.add_argument("--network-interface")
    parser.add_argument("--head-left-sensor", type=int, default=0)
    parser.add_argument("--head-right-sensor", type=int, default=1)
    parser.add_argument("--wrist-left-device", default="/dev/video2")
    parser.add_argument("--wrist-right-device", default="/dev/video3")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 1 <= args.jpeg_quality <= 100:
        raise SystemExit("--jpeg-quality must be in [1, 100]")

    if args.network_interface and "CYCLONEDDS_URI" not in os.environ:
        interface = escape(args.network_interface, {'"': "&quot;"})
        os.environ["CYCLONEDDS_URI"] = (
            "<CycloneDDS><Domain><General><Interfaces>"
            f'<NetworkInterface name="{interface}"/>'
            "</Interfaces></General></Domain></CycloneDDS>"
        )
    participant = DomainParticipant(args.domain)
    publisher = Publisher(participant)
    qos = Qos(Policy.Reliability.BestEffort, Policy.History.KeepLast(1))
    writers = {
        name: DataWriter(
            publisher,
            Topic(participant, topic_name, CameraFrame, qos=qos),
            qos=qos,
        )
        for name, topic_name in CAMERAS.items()
    }

    argus_capture = DualArgusCapture(
        args.head_left_sensor,
        args.head_right_sensor,
        args.width,
        args.height,
        args.fps,
    )
    captures = {
        "head_left": ArgusStream(argus_capture, "head_left"),
        "head_right": ArgusStream(argus_capture, "head_right"),
        "wrist_left": open_wrist(args.wrist_left_device, 1920, 1536, args.fps),
        "wrist_right": open_wrist(args.wrist_right_device, 1920, 1536, args.fps),
    }
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    threads = [
        threading.Thread(
            target=publish_camera,
            args=(name, capture, writers[name], args.width, args.height, args.jpeg_quality, stop),
            name=f"camera-{name}",
            daemon=True,
        )
        for name, capture in captures.items()
    ]
    print(
        f"[INFO] publishing {len(threads)} JPEG camera streams on CycloneDDS domain {args.domain}; "
        "Ctrl-C to stop",
        flush=True,
    )
    for thread in threads:
        thread.start()
    while not stop.wait(0.5):
        if any(not thread.is_alive() for thread in threads):
            stop.set()
    for thread in threads:
        thread.join(timeout=3.0)
    # Releasing multiple nvarguscamerasrc pipelines concurrently from their
    # worker threads can crash inside the Jetson GStreamer plugin. Release all
    # captures serially from the main thread instead.
    for capture in captures.values():
        capture.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
