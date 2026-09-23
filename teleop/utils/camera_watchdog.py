import threading
import time

import numpy as np


class CameraStreamLost(RuntimeError):
    """Raised when a camera required for recording stops producing fresh frames."""


class RecordingCameraWatchdog:
    """Fail closed when any DDS recording camera freezes or becomes empty."""

    def __init__(self, img_client, timeout_s: float, poll_s: float = 0.02, logger=None):
        if timeout_s <= 0:
            raise ValueError("camera watchdog timeout must be positive")
        self._streams = [
            ("head_left", img_client.get_left_head_frame),
            ("head_right", img_client.get_right_head_frame),
            ("wrist_left", img_client.get_left_wrist_frame),
            ("wrist_right", img_client.get_right_wrist_frame),
        ]
        self._timeout_s = float(timeout_s)
        self._poll_s = float(poll_s)
        self._logger = logger
        self._stop = threading.Event()
        self._failed = threading.Event()
        self._lock = threading.Lock()
        self._message = ""
        self._thread = None

    def start(self):
        self._thread = threading.Thread(
            target=self._loop,
            name="recording-camera-watchdog",
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self._timeout_s + self._poll_s))
            self._thread = None

    def failure(self):
        if not self._failed.is_set():
            return None
        with self._lock:
            return self._message

    def _fail(self, message: str):
        with self._lock:
            if not self._failed.is_set():
                self._message = message
                if self._logger is not None:
                    self._logger.critical(f"CAMERA RECORDING SAFETY STOP: {message}")
                self._failed.set()

    def _loop(self):
        now = time.monotonic()
        last_sequence = {}
        last_advance = {name: now for name, _ in self._streams}

        while not self._stop.is_set() and not self._failed.is_set():
            now = time.monotonic()
            for name, getter in self._streams:
                try:
                    frame = getter()
                except Exception as exc:
                    self._fail(f"{name}: frame getter failed: {exc}")
                    break

                image = None if frame is None else getattr(frame, "bgr", None)
                if image is None or not isinstance(image, np.ndarray) or image.size == 0:
                    self._fail(f"{name}: empty or invalid frame")
                    break

                sequence = getattr(frame, "sequence_id", None)
                if sequence is None:
                    self._fail(f"{name}: missing sequence_id")
                    break
                sequence = int(sequence)
                previous = last_sequence.get(name)
                if previous is None or sequence != previous:
                    last_sequence[name] = sequence
                    last_advance[name] = now
                elif now - last_advance[name] > self._timeout_s:
                    self._fail(
                        f"{name}: sequence {sequence} frozen for "
                        f"{now - last_advance[name]:.3f}s "
                        f"(limit {self._timeout_s:.3f}s)"
                    )
                    break

            self._stop.wait(self._poll_s)
