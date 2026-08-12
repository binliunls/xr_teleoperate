import json
import tempfile
import threading
import time
import unittest
import uuid

import zmq

from teleop.utils.sharpa_native_capture import (
    PROTOCOL,
    CaptureControlError,
    SharpaNativeCaptureClient,
    resolve_tactile_host,
    write_capture_metadata,
)


class _FakeCaptureServer:
    def __init__(
        self,
        *,
        reject_start=False,
        sample_delay_s=0.0,
        stop_delay_s=0.0,
        stop_valid=True,
        stop_invalid_reasons=None,
        omit_stop_valid=False,
        stop_valid_is_preliminary=False,
    ):
        self.context = zmq.Context.instance()
        self.address = f"inproc://sharpa-capture-test-{uuid.uuid4()}"
        self.reject_start = reject_start
        self.sample_delay_s = sample_delay_s
        self.stop_delay_s = stop_delay_s
        self.stop_valid = stop_valid
        self.stop_invalid_reasons = list(stop_invalid_reasons or [])
        self.omit_stop_valid = omit_stop_valid
        self.stop_valid_is_preliminary = stop_valid_is_preliminary
        self.requests = []
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()
        if not self._ready.wait(1.0):
            raise RuntimeError("fake capture server did not start")

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self):
        socket = self.context.socket(zmq.REP)
        socket.setsockopt(zmq.LINGER, 0)
        socket.bind(self.address)
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)
        state = "idle"
        self._ready.set()
        try:
            while not self._stop.is_set():
                if socket not in dict(poller.poll(20)):
                    continue
                request = socket.recv_json()
                self.requests.append(request)
                op = request["op"]
                receive_monotonic_ns = time.monotonic_ns()
                receive_realtime_ns = time.time_ns()
                if op == "SAMPLE" and self.sample_delay_s:
                    time.sleep(self.sample_delay_s)
                elif op == "STOP" and self.stop_delay_s:
                    time.sleep(self.stop_delay_s)

                ok = not (op == "START" and self.reject_start)
                error = None if ok else "injected START rejection"
                if op == "START" and ok:
                    state = "recording"
                elif op == "STOP":
                    # STOP is idempotent and finalizes failed/lost START too.
                    state = "finalized"
                    ok = True
                    error = None

                reply = {
                    "protocol": PROTOCOL,
                    "request_id": request["request_id"],
                    "op": op,
                    "capture_id": request["capture_id"],
                    "ok": ok,
                    "state": state,
                    "error": error,
                    "server_receive_monotonic_ns": receive_monotonic_ns,
                    "server_receive_realtime_ns": receive_realtime_ns,
                    "server_send_monotonic_ns": time.monotonic_ns(),
                    "server_send_realtime_ns": time.time_ns(),
                }
                if op == "SAMPLE":
                    reply["idx"] = request["idx"]
                elif op == "START" and ok:
                    reply["capture_path"] = f"/recordings/{request['capture_id']}"
                    reply["manifest_path"] = f"/recordings/{request['capture_id']}/manifest.partial.json"
                elif op == "STOP":
                    reply["finalized_path"] = f"/recordings/{request['capture_id']}"
                    reply["manifest_path"] = f"/recordings/{request['capture_id']}/manifest.json"
                    reply["counts"] = {"tactile": 123}
                    if not self.omit_stop_valid:
                        reply["valid"] = self.stop_valid
                    reply["invalid_reasons"] = self.stop_invalid_reasons
                    reply["response_valid_is_preliminary"] = self.stop_valid_is_preliminary
                    reply["manifest_authoritative_before_reply"] = True
                    reply["manifest_authoritative_after_reply"] = True
                socket.send_json(reply)
        finally:
            socket.close(linger=0)


class SharpaNativeCaptureClientTest(unittest.TestCase):
    def _wait_result(self, client, timeout_s=2.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            result = client.poll_finalized()
            if result is not None:
                return result
            time.sleep(0.005)
        self.fail("capture did not finalize")

    def test_start_samples_stop_are_ordered_and_timestamped(self):
        server = _FakeCaptureServer()
        server.start()
        client = SharpaNativeCaptureClient(
            server.address,
            request_timeout_s=0.5,
            stop_timeout_s=1.0,
            sample_queue_size=8,
            clock_sample_stride=2,
        )
        capture_id = client.new_capture_id()
        try:
            start_reply = client.start_capture(capture_id, start_timeout_s=0.6)
            self.assertEqual(start_reply["state"], "recording")
            for idx in range(5):
                self.assertTrue(
                    client.enqueue_sample(
                        capture_id,
                        idx,
                        workstation_monotonic_ns=1_000 + idx,
                        workstation_realtime_ns=2_000 + idx,
                    )
                )
            self.assertTrue(
                client.stop_capture_async(
                    capture_id,
                    workstation_monotonic_ns=3_000,
                    workstation_realtime_ns=4_000,
                )
            )
            result = self._wait_result(client)

            self.assertEqual(
                [request["op"] for request in server.requests],
                ["START", "SAMPLE", "SAMPLE", "SAMPLE", "SAMPLE", "SAMPLE", "STOP"],
            )
            self.assertEqual(
                [request.get("idx") for request in server.requests if request["op"] == "SAMPLE"],
                list(range(5)),
            )
            self.assertTrue(result["valid"])
            self.assertEqual(result["status"], "complete")
            self.assertEqual(result["counts"]["samples_enqueued"], 5)
            self.assertEqual(result["counts"]["samples_acknowledged"], 5)
            self.assertEqual(result["stop_reply"]["counts"]["tactile"], 123)
            self.assertEqual(
                result["thor_validation"],
                {
                    "source": "stop_reply_authoritative",
                    "valid": True,
                    "invalid_reasons": [],
                    "response_valid_is_preliminary": False,
                    "manifest_authoritative_before_reply": True,
                    "manifest_authoritative_after_reply": True,
                    "requires_manifest_fetch": True,
                },
            )
            self.assertEqual(
                [(sample["op"], sample["idx"]) for sample in result["clock_samples"]],
                [("START", None), ("SAMPLE", 0), ("SAMPLE", 2), ("SAMPLE", 4), ("STOP", None)],
            )
            for sample in result["clock_samples"]:
                self.assertIn("workstation_send_monotonic_ns", sample)
                self.assertIn("server_receive_monotonic_ns", sample)
                self.assertIn("server_send_monotonic_ns", sample)
                self.assertIn("workstation_receive_monotonic_ns", sample)
                self.assertIn("network_round_trip_ns", sample)

            with tempfile.TemporaryDirectory() as episode_dir:
                path = write_capture_metadata(episode_dir, result)
                with open(path, "r", encoding="utf-8") as handle:
                    saved = json.load(handle)
                self.assertEqual(saved["capture_id"], capture_id)
                self.assertEqual(saved["status"], "complete")
            self.assertTrue(client.is_ready())
        finally:
            client.close()
            server.close()

    def test_sample_queue_overflow_marks_episode_invalid(self):
        server = _FakeCaptureServer(sample_delay_s=0.1)
        server.start()
        client = SharpaNativeCaptureClient(
            server.address,
            request_timeout_s=0.5,
            stop_timeout_s=1.0,
            sample_queue_size=1,
        )
        capture_id = client.new_capture_id()
        try:
            client.start_capture(capture_id, start_timeout_s=0.6)
            accepted = [
                client.enqueue_sample(
                    capture_id,
                    idx,
                    workstation_monotonic_ns=10_000 + idx,
                    workstation_realtime_ns=20_000 + idx,
                )
                for idx in range(5)
            ]
            self.assertIn(False, accepted)
            self.assertTrue(client.stop_capture_async(capture_id))
            result = self._wait_result(client)
            self.assertFalse(result["valid"])
            self.assertEqual(result["status"], "degraded")
            self.assertGreater(result["counts"]["samples_queue_dropped"], 0)
            self.assertTrue(any("queue full" in reason for reason in result["degraded_reasons"]))
        finally:
            client.close()
            server.close()

    def test_rejected_start_never_becomes_active_and_queues_cleanup(self):
        server = _FakeCaptureServer(reject_start=True)
        server.start()
        client = SharpaNativeCaptureClient(
            server.address,
            request_timeout_s=0.5,
            stop_timeout_s=1.0,
        )
        capture_id = client.new_capture_id()
        try:
            with self.assertRaises(CaptureControlError):
                client.start_capture(capture_id, start_timeout_s=0.6)
            result = self._wait_result(client)
            self.assertFalse(result["valid"])
            self.assertIn(result["status"], ("degraded", "invalid"))
            self.assertEqual([request["op"] for request in server.requests], ["START", "STOP"])
            self.assertTrue(client.is_ready())
        finally:
            client.close()
            server.close()

    def test_stop_finalization_is_async_and_blocks_reuse_until_consumed(self):
        server = _FakeCaptureServer(stop_delay_s=0.2)
        server.start()
        client = SharpaNativeCaptureClient(
            server.address,
            request_timeout_s=0.5,
            stop_timeout_s=1.0,
        )
        capture_id = client.new_capture_id()
        try:
            client.start_capture(capture_id, start_timeout_s=0.6)

            started = time.monotonic()
            self.assertTrue(client.stop_capture_async(capture_id))
            self.assertLess(time.monotonic() - started, 0.05)
            self.assertFalse(client.is_ready())
            with self.assertRaises(CaptureControlError):
                client.start_capture(client.new_capture_id(), start_timeout_s=0.1)

            result = self._wait_result(client)
            self.assertEqual(result["status"], "complete")
            self.assertEqual(result["stop_reply"]["state"], "finalized")
            self.assertTrue(client.is_ready())
        finally:
            client.close()
            server.close()

    def test_thor_invalid_stop_makes_local_session_invalid(self):
        server = _FakeCaptureServer(
            stop_valid=False,
            stop_invalid_reasons=["tactile sequence gap"],
        )
        server.start()
        client = SharpaNativeCaptureClient(server.address, request_timeout_s=0.5)
        capture_id = client.new_capture_id()
        try:
            client.start_capture(capture_id, start_timeout_s=0.6)
            self.assertTrue(client.stop_capture_async(capture_id))
            result = self._wait_result(client)

            self.assertFalse(result["valid"])
            self.assertEqual(result["status"], "degraded")
            self.assertIn("Thor STOP: tactile sequence gap", result["degraded_reasons"])
            self.assertFalse(result["thor_validation"]["valid"])
            self.assertTrue(result["thor_validation"]["requires_manifest_fetch"])
        finally:
            client.close()
            server.close()

    def test_missing_thor_valid_is_conservatively_invalid(self):
        server = _FakeCaptureServer(omit_stop_valid=True)
        server.start()
        client = SharpaNativeCaptureClient(server.address, request_timeout_s=0.5)
        capture_id = client.new_capture_id()
        try:
            client.start_capture(capture_id, start_timeout_s=0.6)
            self.assertTrue(client.stop_capture_async(capture_id))
            result = self._wait_result(client)

            self.assertFalse(result["valid"])
            self.assertEqual(result["status"], "degraded")
            self.assertIsNone(result["thor_validation"]["valid"])
            self.assertTrue(
                any("did not report valid=true" in reason for reason in result["degraded_reasons"])
            )
        finally:
            client.close()
            server.close()

    def test_preliminary_stop_validity_is_conservatively_invalid(self):
        server = _FakeCaptureServer(stop_valid_is_preliminary=True)
        server.start()
        client = SharpaNativeCaptureClient(server.address, request_timeout_s=0.5)
        capture_id = client.new_capture_id()
        try:
            client.start_capture(capture_id)
            self.assertTrue(client.stop_capture_async(capture_id))
            result = self._wait_result(client)
            self.assertFalse(result["valid"])
            self.assertIn(
                "Thor STOP validity was preliminary or unspecified",
                result["degraded_reasons"],
            )
        finally:
            client.close()
            server.close()


class TactileHostResolutionTest(unittest.TestCase):
    def test_legacy_default_stays_localhost(self):
        self.assertEqual(resolve_tactile_host(None, None), "127.0.0.1")

    def test_native_default_follows_control_tcp_host(self):
        self.assertEqual(
            resolve_tactile_host(None, "tcp://192.168.125.163:48010"),
            "192.168.125.163",
        )
        self.assertEqual(
            resolve_tactile_host(None, "tcp://thor-capture.local:49000"),
            "thor-capture.local",
        )

    def test_explicit_tactile_host_wins(self):
        self.assertEqual(
            resolve_tactile_host("10.0.0.7", "tcp://192.168.125.163:48010"),
            "10.0.0.7",
        )

    def test_non_tcp_control_requires_explicit_tactile_host(self):
        with self.assertRaises(ValueError):
            resolve_tactile_host(None, "inproc://capture-test")


if __name__ == "__main__":
    unittest.main()
