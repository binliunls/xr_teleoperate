import hashlib
import json
import shutil
import tempfile
import unittest
import uuid
from pathlib import Path
from unittest import mock

from teleop.utils.fetch_sharpa_native_capture import (
    CAPTURE_METADATA_FILENAME,
    DEFAULT_HOST_CAPTURE_ROOT,
    CaptureFetchError,
    fetch_episode_capture,
    load_capture_reference,
    verify_capture_directory,
)


def _sha256(data):
    return hashlib.sha256(data).hexdigest()


class _CaptureFixture:
    def __init__(self, root, *, capture_id=None, aliases=False):
        self.root = Path(root)
        self.capture_id = capture_id or str(uuid.uuid4())
        self.capture_dir = self.root / self.capture_id
        self.capture_dir.mkdir()

        chunk_0 = b"chunk-zero\x00" * 7
        chunk_1 = b"chunk-one\x01" * 5
        (self.capture_dir / "chunk-000000.shc").write_bytes(chunk_0)
        nested = self.capture_dir / "chunks"
        nested.mkdir()
        (nested / "chunk-000001.shc").write_bytes(chunk_1)
        clock = b'{"op":"START"}\n{"op":"STOP"}\n'
        (self.capture_dir / "clock_samples.jsonl").write_bytes(clock)

        chunks = [
            {
                "index": 0,
                "path": "chunk-000000.shc",
                "sha256": _sha256(chunk_0),
                ("size_bytes" if aliases else "stored_bytes"): len(chunk_0),
            },
            {
                "index": 1,
                "path": "chunks/chunk-000001.shc",
                "sha256": _sha256(chunk_1),
                ("size_bytes" if aliases else "stored_bytes"): len(chunk_1),
            },
        ]
        control_clock = {
            "closed": True,
            "writer_error": None,
        }
        manifest = {
            "format": "sharpa.capture.manifest.v1",
            "capture_id": self.capture_id,
            "state": "finalized",
            "valid": True,
            "invalid_reasons": [],
            "manifest_sha256_path": f"/recordings/{self.capture_id}/manifest.sha256",
            ("chunk_files" if aliases else "chunks"): chunks,
            "counts": {"chunks": 2},
            "control_clock": control_clock,
        }
        if aliases:
            control_clock["path"] = "/recordings/elsewhere/clock_samples.jsonl"
            manifest["clock_samples_sha256"] = _sha256(clock)
        else:
            manifest["clock_samples_path"] = (
                f"/recordings/{self.capture_id}/clock_samples.jsonl"
            )
            control_clock["sha256"] = _sha256(clock)
        self.manifest = manifest
        self.write_manifest()

    def write_manifest(self):
        manifest_bytes = (json.dumps(self.manifest, indent=2, sort_keys=True) + "\n").encode()
        (self.capture_dir / "manifest.json").write_bytes(manifest_bytes)
        (self.capture_dir / "manifest.sha256").write_text(
            f"{_sha256(manifest_bytes)}  manifest.json\n",
            encoding="ascii",
        )


def _write_episode_metadata(episode_dir, capture_id, *, include_host_paths=True):
    host_capture_path = f"{DEFAULT_HOST_CAPTURE_ROOT}/{capture_id}"
    stop_reply = {
        "protocol": "sharpa.capture.v1",
        "op": "STOP",
        "capture_id": capture_id,
        "ok": True,
        "state": "finalized",
        "valid": True,
        "invalid_reasons": [],
        "response_valid_is_preliminary": False,
        "manifest_authoritative_before_reply": True,
        "manifest_authoritative_after_reply": True,
    }
    if include_host_paths:
        stop_reply.update(
            {
                "host_capture_path": host_capture_path,
                "host_manifest_path": f"{host_capture_path}/manifest.json",
            }
        )
    metadata = {
        "protocol": "sharpa.capture.v1",
        "capture_id": capture_id,
        "status": "complete",
        "valid": True,
        "stop_reply": stop_reply,
        "thor_validation": {
            "source": "stop_reply_authoritative",
            "valid": True,
            "invalid_reasons": [],
            "response_valid_is_preliminary": False,
            "manifest_authoritative_before_reply": True,
            "manifest_authoritative_after_reply": True,
            "requires_manifest_fetch": True,
        },
    }
    Path(episode_dir, CAPTURE_METADATA_FILENAME).write_text(
        json.dumps(metadata),
        encoding="utf-8",
    )
    return metadata


class SharpaNativeFetchVerificationTest(unittest.TestCase):
    def test_valid_capture_verifies_all_chunks_manifest_and_clock(self):
        with tempfile.TemporaryDirectory() as root:
            fixture = _CaptureFixture(root)

            result = verify_capture_directory(fixture.capture_dir, fixture.capture_id)

            self.assertEqual(result.capture_id, fixture.capture_id)
            self.assertEqual(result.chunk_count, 2)
            self.assertGreater(result.chunk_bytes, 0)
            self.assertEqual(result.clock_path, "clock_samples.jsonl")

    def test_documented_aliases_verify(self):
        with tempfile.TemporaryDirectory() as root:
            fixture = _CaptureFixture(root, aliases=True)

            result = verify_capture_directory(fixture.capture_dir, fixture.capture_id)

            self.assertEqual(result.chunk_count, 2)
            self.assertEqual(result.clock_path, "clock_samples.jsonl")

    def test_manifest_hash_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            fixture = _CaptureFixture(root)
            with (fixture.capture_dir / "manifest.json").open("ab") as handle:
                handle.write(b" ")

            with self.assertRaisesRegex(CaptureFetchError, "manifest.json SHA-256 mismatch"):
                verify_capture_directory(fixture.capture_dir, fixture.capture_id)

    def test_chunk_hash_or_size_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            fixture = _CaptureFixture(root)
            (fixture.capture_dir / "chunk-000000.shc").write_bytes(b"corrupt")

            with self.assertRaisesRegex(CaptureFetchError, "chunk 0 size mismatch"):
                verify_capture_directory(fixture.capture_dir, fixture.capture_id)

    def test_chunk_path_traversal_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            fixture = _CaptureFixture(root)
            fixture.manifest["chunks"][0]["path"] = "../outside.shc"
            fixture.write_manifest()

            with self.assertRaisesRegex(CaptureFetchError, "path traversal"):
                verify_capture_directory(fixture.capture_dir, fixture.capture_id)

    def test_nonfinal_or_invalid_manifest_is_rejected(self):
        for key, value, message in (
            ("state", "recording", "state is not finalized"),
            ("valid", False, "manifest is not valid"),
            ("invalid_reasons", ["gap"], "invalid_reasons"),
        ):
            with self.subTest(key=key), tempfile.TemporaryDirectory() as root:
                fixture = _CaptureFixture(root)
                fixture.manifest[key] = value
                fixture.write_manifest()
                with self.assertRaisesRegex(CaptureFetchError, message):
                    verify_capture_directory(fixture.capture_dir, fixture.capture_id)

    def test_clock_hash_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            fixture = _CaptureFixture(root)
            fixture.manifest["control_clock"]["sha256"] = "0" * 64
            fixture.write_manifest()

            with self.assertRaisesRegex(CaptureFetchError, "clock_samples file SHA-256"):
                verify_capture_directory(fixture.capture_dir, fixture.capture_id)

    def test_clock_path_traversal_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            fixture = _CaptureFixture(root)
            fixture.manifest["clock_samples_path"] = "../clock_samples.jsonl"
            fixture.write_manifest()

            with self.assertRaisesRegex(CaptureFetchError, "path traversal"):
                verify_capture_directory(fixture.capture_dir, fixture.capture_id)

    def test_unlisted_chunk_or_partial_manifest_is_rejected(self):
        for name in ("chunk-999999.shc", "manifest.partial.json", "tail.shc.partial"):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as root:
                fixture = _CaptureFixture(root)
                (fixture.capture_dir / name).write_bytes(b"unlisted")

                with self.assertRaisesRegex(CaptureFetchError, "unlisted capture artifact"):
                    verify_capture_directory(fixture.capture_dir, fixture.capture_id)


class SharpaNativeFetchReferenceTest(unittest.TestCase):
    def test_stop_reply_host_paths_are_required_and_preferred(self):
        with tempfile.TemporaryDirectory() as episode:
            capture_id = str(uuid.uuid4())
            _write_episode_metadata(episode, capture_id)

            reference = load_capture_reference(episode)

            self.assertEqual(
                reference.host_capture_path,
                f"{DEFAULT_HOST_CAPTURE_ROOT}/{capture_id}",
            )
            self.assertFalse(reference.used_default_host_path)

    def test_default_host_path_requires_explicit_opt_in(self):
        with tempfile.TemporaryDirectory() as episode:
            capture_id = str(uuid.uuid4())
            _write_episode_metadata(episode, capture_id, include_host_paths=False)

            with self.assertRaisesRegex(CaptureFetchError, "allow-default-host-path"):
                load_capture_reference(episode)
            reference = load_capture_reference(episode, allow_default_host_path=True)
            self.assertTrue(reference.used_default_host_path)

    def test_noncanonical_uuid_and_remote_traversal_are_rejected(self):
        with tempfile.TemporaryDirectory() as episode:
            _write_episode_metadata(episode, "not-a-uuid")
            with self.assertRaisesRegex(CaptureFetchError, "canonical UUID"):
                load_capture_reference(episode)

        with tempfile.TemporaryDirectory() as episode:
            capture_id = str(uuid.uuid4())
            metadata = _write_episode_metadata(episode, capture_id)
            metadata["stop_reply"]["host_capture_path"] = (
                f"{DEFAULT_HOST_CAPTURE_ROOT}/../{capture_id}"
            )
            Path(episode, CAPTURE_METADATA_FILENAME).write_text(
                json.dumps(metadata),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(CaptureFetchError, "canonical absolute POSIX path"):
                load_capture_reference(episode)

    def test_preliminary_invalid_stop_is_rejected_before_transfer(self):
        with tempfile.TemporaryDirectory() as episode:
            capture_id = str(uuid.uuid4())
            metadata = _write_episode_metadata(episode, capture_id)
            metadata["stop_reply"]["valid"] = False
            metadata["stop_reply"]["invalid_reasons"] = ["chunk writer failure"]
            Path(episode, CAPTURE_METADATA_FILENAME).write_text(
                json.dumps(metadata),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(CaptureFetchError, "STOP reply"):
                load_capture_reference(episode)

    def test_local_fetch_uses_argv_verifies_then_atomically_installs(self):
        with tempfile.TemporaryDirectory() as root:
            root_path = Path(root)
            source_root = root_path / "source"
            source_root.mkdir()
            fixture = _CaptureFixture(source_root)
            episode = root_path / "episode_0001"
            episode.mkdir()
            _write_episode_metadata(episode, fixture.capture_id)
            calls = []

            def fake_scp(command, *, check, shell):
                calls.append((command, check, shell))
                destination = Path(command[-1]) / fixture.capture_id
                shutil.copytree(fixture.capture_dir, destination)

            with mock.patch(
                "teleop.utils.fetch_sharpa_native_capture.subprocess.run",
                side_effect=fake_scp,
            ):
                installed, result = fetch_episode_capture(episode)

            self.assertEqual(installed, episode / "sharpa_native_capture")
            self.assertTrue(installed.is_dir())
            self.assertEqual(result.capture_id, fixture.capture_id)
            self.assertEqual(len(calls), 1)
            command, check, shell = calls[0]
            self.assertIsInstance(command, list)
            self.assertEqual(command[:2], ["scp", "-r"])
            self.assertTrue(check)
            self.assertFalse(shell)

            with self.assertRaisesRegex(CaptureFetchError, "refusing to overwrite"):
                fetch_episode_capture(episode)


if __name__ == "__main__":
    unittest.main()
