"""ROS-free CycloneDDS client for compressed H2 camera frames.

The Thor publisher sends one JPEG-compressed ``CameraFrame`` per DDS sample on
four independent topics. This client keeps only the newest decoded frame and
mirrors the ``teleimager.ImageClient`` API used by teleop and policy eval.
"""

import threading
import time
import os
from xml.sax.saxutils import escape
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from cyclonedds.core import Policy, Qos
from cyclonedds.domain import DomainParticipant
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import sequence, uint8, uint32, uint64
from cyclonedds.sub import DataReader, Subscriber
from cyclonedds.topic import Topic
from cyclonedds.util import duration


@dataclass
class CameraFrame(IdlStruct, typename="h2_camera::CameraFrame"):
    sequence_id: uint64
    timestamp_ns: uint64
    width: uint32
    height: uint32
    encoding: str
    data: sequence[uint8]


DEFAULT_TOPICS = {
    "head_left": "h2/camera/head_left/compressed",
    "head_right": "h2/camera/head_right/compressed",
    "wrist_left": "h2/camera/wrist_left/compressed",
    "wrist_right": "h2/camera/wrist_right/compressed",
}


@dataclass
class DDSFrame:
    fps: float = 0.0
    bgr_array: Optional[np.ndarray] = None
    jpg: Optional[bytes] = None
    sequence_id: int = 0
    timestamp_ns: int = 0

    @property
    def bgr(self) -> Optional[np.ndarray]:
        return self.bgr_array

    def __bool__(self) -> bool:
        return self.bgr_array is not None


class _LatestFrame:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.frame: Optional[np.ndarray] = None
        self.jpg: Optional[bytes] = None
        self.sequence_id = 0
        self.timestamp_ns = 0
        self.arrivals: list[float] = []

    def push(self, msg: CameraFrame) -> None:
        if msg.encoding.lower() not in ("jpeg", "jpg"):
            return
        jpg = bytes(msg.data)
        decoded = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if decoded is None:
            return
        now = time.monotonic()
        with self.lock:
            self.frame = decoded
            self.jpg = jpg
            self.sequence_id = int(msg.sequence_id)
            self.timestamp_ns = int(msg.timestamp_ns)
            self.arrivals.append(now)
            cutoff = now - 1.0
            while self.arrivals and self.arrivals[0] < cutoff:
                self.arrivals.pop(0)

    def snapshot(self) -> DDSFrame:
        with self.lock:
            return DDSFrame(
                fps=float(len(self.arrivals)),
                bgr_array=self.frame,
                jpg=self.jpg,
                sequence_id=self.sequence_id,
                timestamp_ns=self.timestamp_ns,
            )


def _cyclonedds_config(network_interface: Optional[str]) -> str:
    """DDSI-level fragmentation keeps JPEG samples below the 1500 B Ethernet MTU.

    Without it a single frame becomes one ~13 kB datagram whose IP fragments the
    kernel must reassemble, and one lost fragment costs the whole frame.
    """
    interfaces = ""
    if network_interface:
        name = escape(network_interface, {'"': "&quot;"})
        interfaces = f'<Interfaces><NetworkInterface name="{name}"/></Interfaces>'
    return (
        "<CycloneDDS><Domain><General>"
        f"{interfaces}"
        "<MaxMessageSize>1400B</MaxMessageSize>"
        "<FragmentSize>1200B</FragmentSize>"
        "</General><Internal>"
        '<SocketReceiveBufferSize min="4MiB" max="16MiB"/>'
        # A 30 fps sample is overwritten in the writer history after 33 ms, so
        # the 100 ms default NackDelay asks for retransmits of frames that no
        # longer exist and caps delivery at ~10 fps.
        "<NackDelay>5ms</NackDelay>"
        # Reliable readers use the reliable defrag pool; four streams of ~25
        # fragments each overrun the default of 16.
        "<DefragReliableMaxSamples>64</DefragReliableMaxSamples>"
        "</Internal></Domain></CycloneDDS>"
    )


class DDSImageClient:
    """Latest-frame DDS camera client with the teleimager API."""

    def __init__(
        self,
        domain_id: int = 10,
        topics: Optional[dict[str, str]] = None,
        network_interface: Optional[str] = None,
        warmup_timeout: float = 5.0,
        require_warmup: bool = False,
    ) -> None:
        self.domain_id = domain_id
        self.topics = dict(topics or DEFAULT_TOPICS)
        self._latest = {name: _LatestFrame() for name in self.topics}
        self._stop = threading.Event()

        if "CYCLONEDDS_URI" not in os.environ:
            os.environ["CYCLONEDDS_URI"] = _cyclonedds_config(network_interface)
        self._participant = DomainParticipant(domain_id)
        self._subscriber = Subscriber(self._participant)
        # Reliable so CycloneDDS retransmits individual lost fragments; a JPEG
        # frame spans ~11 fragments and best-effort drops the whole sample when
        # any one of them is lost.
        qos = Qos(
            Policy.Reliability.Reliable(duration(milliseconds=100)),
            Policy.History.KeepLast(4),
        )
        self._readers: dict[str, DataReader] = {}
        for name, topic_name in self.topics.items():
            topic = Topic(self._participant, topic_name, CameraFrame, qos=qos)
            self._readers[name] = DataReader(self._subscriber, topic, qos=qos)

        self._thread = threading.Thread(target=self._read_loop, name="dds-image-client", daemon=True)
        self._thread.start()
        if warmup_timeout > 0:
            self._warmup(warmup_timeout, require=require_warmup)

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            received = False
            for name, reader in self._readers.items():
                for msg in reader.take(1):
                    # CycloneDDS yields InvalidSample when a remote writer is
                    # disposed or disappears. It is lifecycle metadata, not a
                    # camera frame, and must not terminate the receive thread.
                    if not isinstance(msg, CameraFrame):
                        continue
                    self._latest[name].push(msg)
                    received = True
            if not received:
                time.sleep(0.002)

    def _warmup(self, timeout: float, require: bool) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all(self._latest[name].snapshot().bgr is not None for name in self.topics):
                return
            time.sleep(0.05)
        if require:
            missing = [name for name in self.topics if self._latest[name].snapshot().bgr is None]
            raise RuntimeError(
                f"DDS camera warmup timed out after {timeout:.1f}s on domain {self.domain_id}; "
                f"missing={missing}"
            )

    def get_cam_config(self) -> dict:
        head = self._latest["head_left"].snapshot().bgr
        wrist = self._latest["wrist_left"].snapshot().bgr
        hh, hw = head.shape[:2] if head is not None else (480, 640)
        wh, ww = wrist.shape[:2] if wrist is not None else (480, 640)
        return {
            "head_camera": {
                "enable_zmq": True,
                "enable_webrtc": False,
                "binocular": True,
                "image_shape": [hh, hw * 2],
                "fps": 30,
                "type": "dds",
                "zmq_port": 0,
                "webrtc_port": 0,
            },
            "left_wrist_camera": {
                "enable_zmq": True,
                "enable_webrtc": False,
                "binocular": False,
                "image_shape": [wh, ww],
                "fps": 30,
                "type": "dds",
                "zmq_port": 0,
                "webrtc_port": 0,
            },
            "right_wrist_camera": {
                "enable_zmq": True,
                "enable_webrtc": False,
                "binocular": False,
                "image_shape": [wh, ww],
                "fps": 30,
                "type": "dds",
                "zmq_port": 0,
                "webrtc_port": 0,
            },
        }

    def get_left_head_frame(self) -> DDSFrame:
        return self._latest["head_left"].snapshot()

    def get_right_head_frame(self) -> DDSFrame:
        return self._latest["head_right"].snapshot()

    def get_head_frame(self) -> DDSFrame:
        left = self._latest["head_left"].snapshot()
        right = self._latest["head_right"].snapshot()
        if left.bgr is None or right.bgr is None:
            return DDSFrame()
        right_bgr = right.bgr
        if right_bgr.shape != left.bgr.shape:
            right_bgr = cv2.resize(right_bgr, (left.bgr.shape[1], left.bgr.shape[0]))
        return DDSFrame(
            fps=min(left.fps, right.fps),
            bgr_array=np.concatenate((left.bgr, right_bgr), axis=1),
            sequence_id=min(left.sequence_id, right.sequence_id),
            timestamp_ns=max(left.timestamp_ns, right.timestamp_ns),
        )

    def get_left_wrist_frame(self) -> DDSFrame:
        return self._latest["wrist_left"].snapshot()

    def get_right_wrist_frame(self) -> DDSFrame:
        return self._latest["wrist_right"].snapshot()

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)


def _main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Check native H2 DDS camera streams")
    parser.add_argument("--domain", type=int, default=10)
    parser.add_argument("--network-interface")
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()
    client = DDSImageClient(
        domain_id=args.domain,
        network_interface=args.network_interface,
        warmup_timeout=args.duration,
        require_warmup=True,
    )
    try:
        time.sleep(args.duration)
        frames = {
            "head_left+right": client.get_head_frame(),
            "wrist_left": client.get_left_wrist_frame(),
            "wrist_right": client.get_right_wrist_frame(),
        }
        failed = False
        for name, frame in frames.items():
            if frame.bgr is None or frame.fps < 1.0:
                print(f"[FAIL] {name}: no live frame")
                failed = True
                continue
            print(
                f"[ OK ] {name}: shape={frame.bgr.shape}, "
                f"receive_fps={frame.fps:.1f}, sequence={frame.sequence_id}"
            )
        return 1 if failed else 0
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(_main())
