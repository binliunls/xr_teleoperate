"""Low-bandwidth control client for the Thor Sharpa native-rate recorder.

This module only sends episode lifecycle and 30 Hz sample-marker messages.  The
180 Hz tactile payload remains on Thor and never passes through this client.

Protocol: ``sharpa.capture.v1`` over a ZMQ REQ/REP socket.  The socket is owned
exclusively by a worker thread so SAMPLE and potentially slow STOP/fsync traffic
cannot block the teleop control loop.  START is the sole synchronous operation;
the caller supplies a short acknowledgement deadline and must not begin local
recording without a successful acknowledgement.
"""

from __future__ import annotations

import copy
import json
import logging
import os
import queue
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, Optional
from urllib.parse import urlsplit

import zmq


logger = logging.getLogger(__name__)

PROTOCOL = "sharpa.capture.v1"
DEFAULT_CONTROL_ADDRESS = "tcp://192.168.125.163:48010"
CAPTURE_METADATA_FILENAME = "sharpa_native_capture.json"
LEGACY_TACTILE_HOST = "127.0.0.1"


class CaptureControlError(RuntimeError):
    """A capture-control request was rejected or could not be acknowledged."""


def resolve_tactile_host(
    explicit_host: Optional[str], native_control_address: Optional[str]
) -> str:
    """Resolve the 30 Hz compatibility host without changing legacy mode."""
    if explicit_host:
        return explicit_host
    if native_control_address is None:
        return LEGACY_TACTILE_HOST

    parsed = urlsplit(native_control_address)
    if parsed.scheme != "tcp" or parsed.hostname is None:
        raise ValueError(
            "--sharpa-tactile-host is required when the native capture control "
            "address is not a TCP address"
        )
    return parsed.hostname


@dataclass
class _PendingRequest:
    request: Dict[str, Any]
    done: threading.Event = field(default_factory=threading.Event)
    reply: Optional[Dict[str, Any]] = None
    error: Optional[str] = None
    client_send_monotonic_ns: Optional[int] = None
    client_send_realtime_ns: Optional[int] = None
    client_receive_monotonic_ns: Optional[int] = None
    client_receive_realtime_ns: Optional[int] = None


class SharpaNativeCaptureClient:
    """Asynchronous client for the Thor native Sharpa capture sidecar.

    ``enqueue_sample`` and ``stop_capture_async`` never wait for network or disk
    I/O.  ``poll_finalized`` transfers the completed metadata back to the main
    thread.  Only ``start_capture`` waits, for at most ``start_timeout_s``.
    """

    def __init__(
        self,
        address: str = DEFAULT_CONTROL_ADDRESS,
        *,
        request_timeout_s: float = 0.35,
        stop_timeout_s: float = 30.0,
        sample_queue_size: int = 256,
        clock_sample_stride: int = 30,
        context: Optional[zmq.Context] = None,
    ) -> None:
        if not address:
            raise ValueError("capture control address must not be empty")
        if request_timeout_s <= 0:
            raise ValueError("request_timeout_s must be positive")
        if stop_timeout_s <= 0:
            raise ValueError("stop_timeout_s must be positive")
        if sample_queue_size < 1:
            raise ValueError("sample_queue_size must be at least 1")
        if clock_sample_stride < 1:
            raise ValueError("clock_sample_stride must be at least 1")

        self.address = address
        self.request_timeout_s = float(request_timeout_s)
        self.stop_timeout_s = float(stop_timeout_s)
        self.sample_queue_size = int(sample_queue_size)
        self.clock_sample_stride = int(clock_sample_stride)
        self._context = context if context is not None else zmq.Context.instance()

        # One reserved queue slot guarantees that STOP can be appended after all
        # accepted SAMPLE markers without blocking the caller.
        self._requests: queue.Queue = queue.Queue(maxsize=self.sample_queue_size + 1)
        self._sample_pending = 0
        self._state_lock = threading.Lock()
        self._state = "idle"
        self._active_capture_id: Optional[str] = None
        self._session: Optional[Dict[str, Any]] = None
        self._finalized_session: Optional[Dict[str, Any]] = None
        self._closed = False

        self._worker = threading.Thread(
            target=self._worker_loop,
            name="sharpa-native-capture-control",
            daemon=True,
        )
        self._worker.start()

    @staticmethod
    def new_capture_id() -> str:
        """Return a server-safe unique capture identifier."""
        return str(uuid.uuid4())

    def is_ready(self) -> bool:
        """Return True only after final metadata was consumed by the caller."""
        with self._state_lock:
            return self._state == "idle" and self._finalized_session is None

    def active_capture_id(self) -> Optional[str]:
        with self._state_lock:
            return self._active_capture_id

    def session_metadata(self) -> Optional[Dict[str, Any]]:
        """Return a thread-safe copy of the current/most recent session."""
        with self._state_lock:
            return copy.deepcopy(self._session)

    def start_capture(
        self,
        capture_id: str,
        *,
        workstation_monotonic_ns: Optional[int] = None,
        workstation_realtime_ns: Optional[int] = None,
        start_timeout_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Request START and synchronously require its short acknowledgement.

        On failure, a STOP cleanup is queued because START may have reached Thor
        even when its reply was lost.  No local episode should be created unless
        this method returns successfully.
        """
        self._validate_capture_id(capture_id)
        with self._state_lock:
            if self._closed:
                raise CaptureControlError("capture client is closed")
            if self._state != "idle" or self._finalized_session is not None:
                raise CaptureControlError(f"capture client is not ready (state={self._state})")
            self._state = "starting"
            self._active_capture_id = capture_id
            self._session = self._new_session(capture_id)

        pending = self._make_request(
            "START",
            capture_id,
            workstation_monotonic_ns=workstation_monotonic_ns,
            workstation_realtime_ns=workstation_realtime_ns,
        )
        self._put_control_request(pending)
        wait_s = self.request_timeout_s if start_timeout_s is None else float(start_timeout_s)
        try:
            reply = self._wait_for(pending, wait_s)
        except CaptureControlError as exc:
            self._mark_degraded(f"START failed: {exc}", invalid=True)
            # Do not block here.  The queued STOP follows START on the same worker
            # and cleans up a capture whose acknowledgement may merely be lost.
            self._queue_stop_after_failed_start(capture_id)
            raise

        with self._state_lock:
            self._state = "active"
            if self._session is not None:
                self._session["status"] = "active"
                self._session["valid"] = True
        return reply

    def enqueue_sample(
        self,
        capture_id: str,
        idx: int,
        *,
        workstation_monotonic_ns: int,
        workstation_realtime_ns: int,
    ) -> bool:
        """Enqueue one 30 Hz alignment marker without blocking.

        False means the marker was not accepted and the session is explicitly
        marked degraded/invalid in the final metadata.
        """
        with self._state_lock:
            if self._state != "active" or capture_id != self._active_capture_id:
                self._mark_degraded_locked(
                    f"SAMPLE idx={idx} rejected while state={self._state}", invalid=True
                )
                return False
            if self._sample_pending >= self.sample_queue_size:
                if self._session is not None:
                    self._session["counts"]["samples_queue_dropped"] += 1
                self._mark_degraded_locked(
                    f"SAMPLE queue full at idx={idx}", invalid=True
                )
                return False
            self._sample_pending += 1

        pending = self._make_request(
            "SAMPLE",
            capture_id,
            idx=int(idx),
            workstation_monotonic_ns=int(workstation_monotonic_ns),
            workstation_realtime_ns=int(workstation_realtime_ns),
        )
        try:
            self._requests.put_nowait(pending)
        except queue.Full:
            # The reserved STOP slot should make this unreachable, but fail
            # conservatively if another control request occupied it.
            with self._state_lock:
                self._sample_pending -= 1
                if self._session is not None:
                    self._session["counts"]["samples_queue_dropped"] += 1
                self._mark_degraded_locked(
                    f"control queue full at SAMPLE idx={idx}", invalid=True
                )
            return False

        with self._state_lock:
            if self._session is not None:
                self._session["counts"]["samples_enqueued"] += 1
        return True

    def stop_capture_async(
        self,
        capture_id: str,
        *,
        workstation_monotonic_ns: Optional[int] = None,
        workstation_realtime_ns: Optional[int] = None,
    ) -> bool:
        """Append STOP after all queued SAMPLEs and return immediately."""
        with self._state_lock:
            if self._state not in ("active", "starting"):
                return False
            if capture_id != self._active_capture_id:
                self._mark_degraded_locked(
                    f"STOP capture_id mismatch: active={self._active_capture_id}, got={capture_id}",
                    invalid=True,
                )
                return False
            self._state = "stopping"
            if self._session is not None:
                self._session["status"] = "stopping"

        pending = self._make_request(
            "STOP",
            capture_id,
            workstation_monotonic_ns=workstation_monotonic_ns,
            workstation_realtime_ns=workstation_realtime_ns,
        )
        try:
            self._requests.put_nowait(pending)
        except queue.Full:
            self._mark_degraded("STOP could not enter the control queue", invalid=True)
            self._finalize_session_locally("invalid")
            return False
        return True

    def ping(self, capture_id: str, *, timeout_s: Optional[float] = None) -> Dict[str, Any]:
        """Send a synchronous PING, primarily for diagnostics/clock anchors."""
        pending = self._make_request("PING", capture_id)
        self._put_control_request(pending)
        return self._wait_for(
            pending,
            self.request_timeout_s if timeout_s is None else float(timeout_s),
        )

    def poll_finalized(self) -> Optional[Dict[str, Any]]:
        """Return completed STOP metadata once, then make the client reusable."""
        with self._state_lock:
            if self._finalized_session is None:
                return None
            result = copy.deepcopy(self._finalized_session)
            self._finalized_session = None
            self._active_capture_id = None
            self._state = "idle"
            return result

    def wait_finalized(self, timeout_s: float) -> Optional[Dict[str, Any]]:
        """Wait outside the control loop for final metadata (used at shutdown)."""
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            result = self.poll_finalized()
            if result is not None:
                return result
            time.sleep(0.01)
        return self.poll_finalized()

    def close(self, timeout_s: float = 1.0) -> None:
        """Stop the worker.  Call STOP first if a capture is active."""
        with self._state_lock:
            if self._closed:
                return
            self._closed = True
        try:
            self._requests.put_nowait(None)
        except queue.Full:
            # STOP or samples still occupy the queue; wait only during shutdown.
            try:
                self._requests.put(None, timeout=max(0.0, float(timeout_s)))
            except queue.Full:
                logger.error("capture worker queue did not accept shutdown sentinel")
        self._worker.join(timeout=max(0.0, float(timeout_s)))
        if self._worker.is_alive():
            logger.error("capture control worker did not stop within %.3fs", timeout_s)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _validate_capture_id(capture_id: str) -> None:
        if not capture_id or len(capture_id) > 128:
            raise ValueError("capture_id must contain 1..128 characters")
        first = capture_id[0]
        if not first.isascii() or not first.isalnum():
            raise ValueError("capture_id must start with an ASCII letter or digit")
        allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-")
        if any(ch not in allowed for ch in capture_id):
            raise ValueError("capture_id contains unsupported characters")

    def _new_session(self, capture_id: str) -> Dict[str, Any]:
        return {
            "protocol": PROTOCOL,
            "capture_id": capture_id,
            "control_address": self.address,
            "status": "starting",
            "valid": False,
            "degraded_reasons": [],
            "counts": {
                "samples_enqueued": 0,
                "samples_acknowledged": 0,
                "samples_queue_dropped": 0,
                "samples_failed": 0,
                "requests_failed": 0,
            },
            "clock_samples": [],
            "clock_sample_stride": self.clock_sample_stride,
            "start_reply": None,
            "stop_reply": None,
            "thor_validation": None,
        }

    def _make_request(
        self,
        op: str,
        capture_id: str,
        *,
        idx: Optional[int] = None,
        workstation_monotonic_ns: Optional[int] = None,
        workstation_realtime_ns: Optional[int] = None,
    ) -> _PendingRequest:
        request: Dict[str, Any] = {
            "protocol": PROTOCOL,
            "request_id": str(uuid.uuid4()),
            "op": op,
            "capture_id": capture_id,
            "workstation_monotonic_ns": (
                time.monotonic_ns()
                if workstation_monotonic_ns is None
                else int(workstation_monotonic_ns)
            ),
            "workstation_realtime_ns": (
                time.time_ns() if workstation_realtime_ns is None else int(workstation_realtime_ns)
            ),
        }
        if idx is not None:
            request["idx"] = int(idx)
        return _PendingRequest(request=request)

    def _put_control_request(self, pending: _PendingRequest) -> None:
        try:
            self._requests.put_nowait(pending)
        except queue.Full as exc:
            raise CaptureControlError("capture control queue is full") from exc

    @staticmethod
    def _wait_for(pending: _PendingRequest, timeout_s: float) -> Dict[str, Any]:
        if timeout_s <= 0 or not pending.done.wait(timeout_s):
            raise CaptureControlError(
                f"timeout waiting for {pending.request['op']} acknowledgement"
            )
        if pending.error is not None:
            raise CaptureControlError(pending.error)
        if pending.reply is None:
            raise CaptureControlError(f"{pending.request['op']} completed without a reply")
        return pending.reply

    def _queue_stop_after_failed_start(self, capture_id: str) -> None:
        with self._state_lock:
            self._state = "stopping"
            if self._session is not None:
                self._session["status"] = "start_failed_cleanup"
        pending = self._make_request("STOP", capture_id)
        try:
            self._requests.put_nowait(pending)
        except queue.Full:
            self._mark_degraded("failed START cleanup STOP queue was full", invalid=True)
            self._finalize_session_locally("invalid")

    def _mark_degraded(self, reason: str, *, invalid: bool) -> None:
        with self._state_lock:
            self._mark_degraded_locked(reason, invalid=invalid)

    def _mark_degraded_locked(self, reason: str, *, invalid: bool) -> None:
        if self._session is None:
            return
        if reason not in self._session["degraded_reasons"]:
            self._session["degraded_reasons"].append(reason)
        if invalid:
            self._session["valid"] = False

    def _finalize_session_locally(self, status: str) -> None:
        with self._state_lock:
            if self._session is None:
                return
            self._session["status"] = status
            self._session["valid"] = False
            self._session["finalized_workstation_monotonic_ns"] = time.monotonic_ns()
            self._session["finalized_workstation_realtime_ns"] = time.time_ns()
            self._finalized_session = copy.deepcopy(self._session)
            self._state = "finalized"

    def _new_socket(self) -> zmq.Socket:
        sock = self._context.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.SNDHWM, 1)
        sock.setsockopt(zmq.RCVHWM, 1)
        sock.connect(self.address)
        return sock

    @staticmethod
    def _validate_reply(request: Dict[str, Any], reply: Any) -> Dict[str, Any]:
        if not isinstance(reply, dict):
            raise CaptureControlError("reply is not a JSON object")
        for key in ("protocol", "request_id", "op", "capture_id", "ok", "state", "error"):
            if key not in reply:
                raise CaptureControlError(f"reply missing {key!r}")
        if reply["protocol"] != PROTOCOL:
            raise CaptureControlError(f"reply protocol mismatch: {reply['protocol']!r}")
        if reply["request_id"] != request["request_id"]:
            raise CaptureControlError("reply request_id mismatch")
        if reply["op"] != request["op"]:
            raise CaptureControlError("reply op mismatch")
        if reply["capture_id"] != request["capture_id"]:
            raise CaptureControlError("reply capture_id mismatch")
        for key in (
            "server_receive_monotonic_ns",
            "server_receive_realtime_ns",
            "server_send_monotonic_ns",
            "server_send_realtime_ns",
        ):
            value = reply.get(key)
            if isinstance(value, bool) or not isinstance(value, int):
                raise CaptureControlError(f"reply missing integer {key!r}")
        if reply["ok"] is not True:
            error = reply.get("error")
            raise CaptureControlError(str(error) if error else f"{request['op']} rejected")
        if request["op"] == "SAMPLE" and reply.get("idx") != request.get("idx"):
            raise CaptureControlError("SAMPLE reply idx mismatch")
        expected_state = {
            "START": "recording",
            "SAMPLE": "recording",
            "STOP": "finalized",
        }.get(request["op"])
        if expected_state is not None and reply["state"] != expected_state:
            raise CaptureControlError(
                f"{request['op']} reply state mismatch: expected {expected_state!r}, "
                f"got {reply['state']!r}"
            )
        return reply

    def _record_success(self, pending: _PendingRequest) -> None:
        request = pending.request
        reply = pending.reply
        if reply is None:
            return
        exchange = {
            "op": request["op"],
            "request_id": request["request_id"],
            "idx": request.get("idx"),
            "workstation_event_monotonic_ns": request["workstation_monotonic_ns"],
            "workstation_event_realtime_ns": request["workstation_realtime_ns"],
            "workstation_send_monotonic_ns": pending.client_send_monotonic_ns,
            "workstation_send_realtime_ns": pending.client_send_realtime_ns,
            "server_receive_monotonic_ns": reply["server_receive_monotonic_ns"],
            "server_receive_realtime_ns": reply["server_receive_realtime_ns"],
            "server_send_monotonic_ns": reply["server_send_monotonic_ns"],
            "server_send_realtime_ns": reply["server_send_realtime_ns"],
            "workstation_receive_monotonic_ns": pending.client_receive_monotonic_ns,
            "workstation_receive_realtime_ns": pending.client_receive_realtime_ns,
        }
        if (
            pending.client_send_monotonic_ns is not None
            and pending.client_receive_monotonic_ns is not None
        ):
            server_elapsed = (
                reply["server_send_monotonic_ns"] - reply["server_receive_monotonic_ns"]
            )
            exchange["network_round_trip_ns"] = max(
                0,
                pending.client_receive_monotonic_ns
                - pending.client_send_monotonic_ns
                - server_elapsed,
            )

        with self._state_lock:
            if self._session is None or request["capture_id"] != self._session["capture_id"]:
                return
            keep_clock_sample = (
                request["op"] != "SAMPLE"
                or int(request.get("idx", 0)) % self.clock_sample_stride == 0
            )
            if keep_clock_sample:
                self._session["clock_samples"].append(exchange)
            if request["op"] == "START":
                self._session["start_reply"] = copy.deepcopy(reply)
            elif request["op"] == "SAMPLE":
                self._session["counts"]["samples_acknowledged"] += 1
            elif request["op"] == "STOP":
                self._session["stop_reply"] = copy.deepcopy(reply)
                raw_thor_reasons = reply.get("invalid_reasons")
                if isinstance(raw_thor_reasons, list) and all(
                    isinstance(reason, str) for reason in raw_thor_reasons
                ):
                    thor_reasons = list(raw_thor_reasons)
                    reasons_well_formed = True
                else:
                    thor_reasons = []
                    reasons_well_formed = False

                thor_valid = reply.get("valid") is True
                manifest_before_reply = reply.get("manifest_authoritative_before_reply")
                manifest_after_reply = reply.get("manifest_authoritative_after_reply")
                response_is_preliminary = reply.get("response_valid_is_preliminary")
                self._session["thor_validation"] = {
                    # The recorder closes/fsyncs its clock and writes the final
                    # manifest before replying. Fetch still re-verifies every
                    # artifact; it is not needed to repair a preliminary result.
                    "source": "stop_reply_authoritative",
                    "valid": reply.get("valid") if isinstance(reply.get("valid"), bool) else None,
                    "invalid_reasons": thor_reasons,
                    "response_valid_is_preliminary": response_is_preliminary,
                    "manifest_authoritative_before_reply": manifest_before_reply,
                    "manifest_authoritative_after_reply": manifest_after_reply,
                    "requires_manifest_fetch": True,
                }

                if not reasons_well_formed:
                    self._mark_degraded_locked(
                        "Thor STOP invalid_reasons is missing or not list[str]",
                        invalid=True,
                    )
                elif thor_reasons:
                    for reason in thor_reasons:
                        self._mark_degraded_locked(f"Thor STOP: {reason}", invalid=True)
                if not thor_valid and not thor_reasons:
                    self._mark_degraded_locked(
                        f"Thor STOP did not report valid=true (got {reply.get('valid')!r})",
                        invalid=True,
                    )
                if manifest_after_reply is not True:
                    self._mark_degraded_locked(
                        "Thor STOP did not report manifest_authoritative_after_reply=true",
                        invalid=True,
                    )
                if manifest_before_reply is not True:
                    self._mark_degraded_locked(
                        "Thor STOP did not report manifest_authoritative_before_reply=true",
                        invalid=True,
                    )
                if response_is_preliminary is not False:
                    self._mark_degraded_locked(
                        "Thor STOP validity was preliminary or unspecified",
                        invalid=True,
                    )
                if self._session["valid"] and not self._session["degraded_reasons"]:
                    self._session["status"] = "complete"
                else:
                    self._session["status"] = "degraded"
                    self._session["valid"] = False
                self._session["finalized_workstation_monotonic_ns"] = time.monotonic_ns()
                self._session["finalized_workstation_realtime_ns"] = time.time_ns()
                self._finalized_session = copy.deepcopy(self._session)
                self._state = "finalized"

    def _record_failure(self, pending: _PendingRequest, error: str) -> None:
        op = pending.request["op"]
        with self._state_lock:
            if self._session is not None:
                self._session["counts"]["requests_failed"] += 1
                if op == "SAMPLE":
                    self._session["counts"]["samples_failed"] += 1
                self._mark_degraded_locked(f"{op} failed: {error}", invalid=True)
                if op == "STOP":
                    self._session["status"] = "invalid"
                    self._session["finalized_workstation_monotonic_ns"] = time.monotonic_ns()
                    self._session["finalized_workstation_realtime_ns"] = time.time_ns()
                    self._finalized_session = copy.deepcopy(self._session)
                    self._state = "finalized"

    def _worker_loop(self) -> None:
        sock: Optional[zmq.Socket] = None
        try:
            while True:
                pending = self._requests.get()
                try:
                    if pending is None:
                        return
                    if sock is None:
                        sock = self._new_socket()

                    request_timeout_s = (
                        self.stop_timeout_s
                        if pending.request["op"] == "STOP"
                        else self.request_timeout_s
                    )
                    pending.client_send_monotonic_ns = time.monotonic_ns()
                    pending.client_send_realtime_ns = time.time_ns()
                    sock.send_json(pending.request)
                    if not sock.poll(max(1, int(request_timeout_s * 1000)), zmq.POLLIN):
                        raise CaptureControlError(
                            f"timeout waiting for {pending.request['op']} reply"
                        )
                    raw_reply = sock.recv_json()
                    pending.client_receive_monotonic_ns = time.monotonic_ns()
                    pending.client_receive_realtime_ns = time.time_ns()
                    pending.reply = self._validate_reply(pending.request, raw_reply)
                    self._record_success(pending)
                except Exception as exc:
                    pending.error = str(exc)
                    self._record_failure(pending, pending.error)
                    if sock is not None:
                        sock.close(linger=0)
                        sock = None
                    logger.error(
                        "Sharpa capture %s failed: %s",
                        pending.request.get("op"),
                        pending.error,
                    )
                finally:
                    if pending is not None:
                        if pending.request.get("op") == "SAMPLE":
                            with self._state_lock:
                                self._sample_pending = max(0, self._sample_pending - 1)
                        pending.done.set()
                    self._requests.task_done()
        finally:
            if sock is not None:
                sock.close(linger=0)


def write_capture_metadata(
    episode_dir: str,
    metadata: Dict[str, Any],
    *,
    filename: str = CAPTURE_METADATA_FILENAME,
) -> str:
    """Atomically write local capture/control metadata beside ``data.json``."""
    if not os.path.isdir(episode_dir):
        raise FileNotFoundError(f"episode directory does not exist: {episode_dir}")
    path = os.path.join(episode_dir, filename)
    tmp_path = f"{path}.{uuid.uuid4().hex}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as handle:
        json.dump(metadata, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp_path, path)
    return path
