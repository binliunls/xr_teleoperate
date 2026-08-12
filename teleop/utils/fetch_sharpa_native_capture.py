"""Fetch and verify one finalized Thor-native Sharpa capture.

Run manually after an episode has stopped and ``sharpa_native_capture.json``
has been finalized::

    python -m teleop.utils.fetch_sharpa_native_capture EPISODE_DIR

Only ``scp`` lifecycle traffic crosses the workstation link, after teleop has
ended.  All fetched data is verified in a unique partial directory before an
atomic rename to ``EPISODE_DIR/sharpa_native_capture``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import tempfile
import uuid
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence


CAPTURE_METADATA_FILENAME = "sharpa_native_capture.json"
FETCHED_CAPTURE_DIRNAME = "sharpa_native_capture"
DEFAULT_THOR = "unitree@192.168.125.163"
DEFAULT_HOST_CAPTURE_ROOT = "/home/unitree/sharpa_recordings"
MANIFEST_FORMAT = "sharpa.capture.manifest.v1"
_SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")
_MANIFEST_SHA_LINE_RE = re.compile(
    r"^([0-9a-fA-F]{64})(?:[ \t]+\*?manifest\.json)?[ \t]*$"
)
_SCP_TARGET_RE = re.compile(
    r"^(?:[A-Za-z_][A-Za-z0-9_.-]*@)?[A-Za-z0-9][A-Za-z0-9_.-]*$"
)
_REMOTE_PATH_RE = re.compile(r"^/[A-Za-z0-9._/-]+$")


class CaptureFetchError(RuntimeError):
    """The capture reference, transfer, or integrity verification failed."""


@dataclass(frozen=True)
class CaptureReference:
    capture_id: str
    host_capture_path: str
    host_manifest_path: str
    used_default_host_path: bool


@dataclass(frozen=True)
class VerificationResult:
    capture_id: str
    manifest_sha256: str
    chunk_count: int
    chunk_bytes: int
    clock_path: Optional[str]


def _require_regular_file(path: Path, label: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise CaptureFetchError(f"{label} is missing or is not a regular file: {path}")


def _load_json_object(path: Path, label: str) -> Dict[str, Any]:
    _require_regular_file(path, label)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CaptureFetchError(f"could not read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise CaptureFetchError(f"{label} must contain a JSON object")
    return value


def _canonical_uuid(value: Any, label: str = "capture_id") -> str:
    if not isinstance(value, str):
        raise CaptureFetchError(f"{label} must be a canonical UUID string")
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError) as exc:
        raise CaptureFetchError(f"{label} must be a canonical UUID string") from exc
    canonical = str(parsed)
    if value != canonical:
        raise CaptureFetchError(f"{label} must use canonical lowercase UUID form")
    return canonical


def _validated_remote_path(value: Any, label: str) -> PurePosixPath:
    if not isinstance(value, str) or not _REMOTE_PATH_RE.fullmatch(value):
        raise CaptureFetchError(f"{label} is not a safe absolute POSIX path")
    path = PurePosixPath(value)
    if not path.is_absolute() or path.as_posix() != value or ".." in path.parts:
        raise CaptureFetchError(f"{label} is not a canonical absolute POSIX path")
    if path == PurePosixPath("/"):
        raise CaptureFetchError(f"{label} must not be the filesystem root")
    return path


def _alias_value(
    mapping: Mapping[str, Any],
    keys: Sequence[str],
    label: str,
    *,
    required: bool = True,
) -> Any:
    present = [(key, mapping[key]) for key in keys if key in mapping]
    if not present:
        if required:
            raise CaptureFetchError(f"{label} is missing (accepted keys: {', '.join(keys)})")
        return None
    first_value = present[0][1]
    if any(value != first_value for _, value in present[1:]):
        raise CaptureFetchError(f"conflicting aliases for {label}")
    return first_value


def load_capture_reference(
    episode_dir: os.PathLike[str] | str,
    *,
    allow_default_host_path: bool = False,
    expected_host_root: str = DEFAULT_HOST_CAPTURE_ROOT,
) -> CaptureReference:
    """Load and strictly validate the finalized workstation-side reference."""
    episode = Path(episode_dir)
    if episode.is_symlink() or not episode.is_dir():
        raise CaptureFetchError(f"episode directory is missing or unsafe: {episode}")
    episode = episode.resolve(strict=True)
    metadata = _load_json_object(
        episode / CAPTURE_METADATA_FILENAME,
        CAPTURE_METADATA_FILENAME,
    )

    if metadata.get("protocol") != "sharpa.capture.v1":
        raise CaptureFetchError("capture metadata protocol is not sharpa.capture.v1")
    capture_id = _canonical_uuid(metadata.get("capture_id"))
    if metadata.get("status") != "complete" or metadata.get("valid") is not True:
        reasons = metadata.get("degraded_reasons")
        if not isinstance(reasons, list):
            reasons = metadata.get("invalid_reasons")
        reason_text = ", ".join(str(reason) for reason in reasons or []) or "unspecified"
        raise CaptureFetchError(
            "capture is not eligible for verified fetch "
            "(status={!r}, valid={!r}; reasons: {})".format(
                metadata.get("status"), metadata.get("valid"), reason_text
            )
        )

    stop_reply = metadata.get("stop_reply")
    if not isinstance(stop_reply, dict):
        raise CaptureFetchError("capture metadata has no finalized STOP reply")
    if (
        stop_reply.get("protocol") != "sharpa.capture.v1"
        or stop_reply.get("op") != "STOP"
        or stop_reply.get("capture_id") != capture_id
        or stop_reply.get("ok") is not True
        or stop_reply.get("state") != "finalized"
        or stop_reply.get("valid") is not True
    ):
        raise CaptureFetchError("STOP reply is not a successful finalized reply for this capture")
    stop_invalid_reasons = stop_reply.get("invalid_reasons")
    if not isinstance(stop_invalid_reasons, list) or stop_invalid_reasons:
        raise CaptureFetchError("STOP reply has invalid or non-empty invalid_reasons")
    if stop_reply.get("manifest_authoritative_after_reply") is not True:
        raise CaptureFetchError("STOP reply does not declare its manifest authoritative after reply")
    if stop_reply.get("manifest_authoritative_before_reply") is not True:
        raise CaptureFetchError("STOP reply does not declare pre-reply manifest finalization")
    if stop_reply.get("response_valid_is_preliminary") is not False:
        raise CaptureFetchError("STOP reply validity is preliminary or unspecified")

    thor_validation = metadata.get("thor_validation")
    if not isinstance(thor_validation, dict):
        raise CaptureFetchError("capture metadata has no preliminary Thor validation record")
    if (
        thor_validation.get("valid") is not True
        or thor_validation.get("response_valid_is_preliminary") is not False
        or thor_validation.get("manifest_authoritative_before_reply") is not True
        or thor_validation.get("manifest_authoritative_after_reply") is not True
        or thor_validation.get("requires_manifest_fetch") is not True
    ):
        raise CaptureFetchError("preliminary Thor validation does not permit authoritative fetch")

    host_root = _validated_remote_path(expected_host_root, "expected host capture root")
    expected_capture_path = host_root / capture_id
    expected_manifest_path = expected_capture_path / "manifest.json"

    raw_capture_path = stop_reply.get("host_capture_path")
    raw_manifest_path = stop_reply.get("host_manifest_path")
    used_default = False
    if raw_capture_path is None or raw_manifest_path is None:
        if not allow_default_host_path:
            raise CaptureFetchError(
                "STOP reply lacks host_capture_path/host_manifest_path; rerun with "
                "--allow-default-host-path only for the documented Thor layout"
            )
        raw_capture_path = expected_capture_path.as_posix()
        raw_manifest_path = expected_manifest_path.as_posix()
        used_default = True

    host_capture_path = _validated_remote_path(raw_capture_path, "host_capture_path")
    host_manifest_path = _validated_remote_path(raw_manifest_path, "host_manifest_path")
    if host_capture_path != expected_capture_path:
        raise CaptureFetchError(
            f"host_capture_path must be exactly {expected_capture_path.as_posix()}"
        )
    if host_manifest_path != expected_manifest_path:
        raise CaptureFetchError(
            f"host_manifest_path must be exactly {expected_manifest_path.as_posix()}"
        )

    return CaptureReference(
        capture_id=capture_id,
        host_capture_path=host_capture_path.as_posix(),
        host_manifest_path=host_manifest_path.as_posix(),
        used_default_host_path=used_default,
    )


def _safe_relative_path(value: Any, label: str) -> PurePosixPath:
    if not isinstance(value, str) or not value or "\\" in value:
        raise CaptureFetchError(f"{label} is not a safe relative POSIX path")
    path = PurePosixPath(value)
    if path.is_absolute() or path.as_posix() != value:
        raise CaptureFetchError(f"{label} is not a canonical relative POSIX path")
    if not path.parts or any(part in ("", ".", "..") for part in path.parts):
        raise CaptureFetchError(f"{label} contains path traversal")
    return path


def _safe_declared_basename(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or "\\" in value:
        raise CaptureFetchError(f"{label} is not a safe path string")
    path = PurePosixPath(value)
    if any(part in ("", ".", "..") for part in path.parts):
        raise CaptureFetchError(f"{label} contains path traversal")
    if not path.is_absolute() and len(path.parts) != 1:
        raise CaptureFetchError(f"{label} must be an absolute path or a basename")
    basename = path.name
    if basename in ("", ".", "..") or "/" in basename:
        raise CaptureFetchError(f"{label} has no safe basename")
    return basename


def _local_declared_file(root: Path, relative: PurePosixPath, label: str) -> Path:
    current = root
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise CaptureFetchError(f"{label} contains a symbolic link: {relative.as_posix()}")
    _require_regular_file(current, label)
    try:
        current.resolve(strict=True).relative_to(root.resolve(strict=True))
    except (OSError, ValueError) as exc:
        raise CaptureFetchError(f"{label} escapes the capture directory") from exc
    return current


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        while True:
            block = handle.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _expected_sha256(value: Any, label: str) -> str:
    if not isinstance(value, str) or not _SHA256_RE.fullmatch(value):
        raise CaptureFetchError(f"{label} must be a 64-digit SHA-256 hex string")
    return value.lower()


def _expected_size(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise CaptureFetchError(f"{label} must be a non-negative integer")
    return value


def _verify_size_and_hash(
    path: Path,
    *,
    expected_size: Optional[int],
    expected_sha256: str,
    label: str,
) -> int:
    size_before = path.stat().st_size
    if expected_size is not None and size_before != expected_size:
        raise CaptureFetchError(
            f"{label} size mismatch: expected {expected_size}, got {size_before}"
        )
    actual_sha256 = _sha256_file(path)
    size_after = path.stat().st_size
    if size_after != size_before:
        raise CaptureFetchError(f"{label} changed while it was being verified")
    if actual_sha256 != expected_sha256:
        raise CaptureFetchError(
            f"{label} SHA-256 mismatch: expected {expected_sha256}, got {actual_sha256}"
        )
    return size_before


def _parse_manifest_sha256(path: Path) -> str:
    _require_regular_file(path, "manifest.sha256")
    try:
        text = path.read_text(encoding="ascii")
    except (OSError, UnicodeDecodeError) as exc:
        raise CaptureFetchError(f"could not read manifest.sha256: {exc}") from exc
    lines = [line for line in text.splitlines() if line.strip()]
    if len(lines) != 1:
        raise CaptureFetchError("manifest.sha256 must contain exactly one non-empty line")
    match = _MANIFEST_SHA_LINE_RE.fullmatch(lines[0])
    if match is None:
        raise CaptureFetchError(
            "manifest.sha256 must contain a hash, optionally followed by manifest.json"
        )
    return match.group(1).lower()


def _reject_symlinks_and_special_files(root: Path) -> None:
    for directory, dirnames, filenames in os.walk(root, followlinks=False):
        directory_path = Path(directory)
        for name in [*dirnames, *filenames]:
            path = directory_path / name
            if path.is_symlink():
                raise CaptureFetchError(f"fetched capture contains a symbolic link: {path}")
            if name in filenames and not path.is_file():
                raise CaptureFetchError(f"fetched capture contains a special file: {path}")


def _reject_unlisted_capture_artifacts(root: Path, expected_files: set[str]) -> None:
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        relative = path.relative_to(root).as_posix()
        if relative in expected_files:
            continue
        name = path.name.lower()
        if (
            name.endswith(".shc")
            or name.endswith(".partial")
            or ".partial." in name
            or name.startswith("manifest")
        ):
            raise CaptureFetchError(f"unlisted capture artifact: {relative}")


def verify_capture_directory(
    capture_dir: os.PathLike[str] | str,
    expected_capture_id: str,
) -> VerificationResult:
    """Verify a fetched capture entirely locally; performs no network access."""
    capture_id = _canonical_uuid(expected_capture_id, "expected capture_id")
    root = Path(capture_dir)
    if root.is_symlink() or not root.is_dir():
        raise CaptureFetchError(f"capture directory is missing or unsafe: {root}")
    _reject_symlinks_and_special_files(root)

    manifest_path = root / "manifest.json"
    manifest_sha_path = root / "manifest.sha256"
    _require_regular_file(manifest_path, "manifest.json")
    _require_regular_file(manifest_sha_path, "manifest.sha256")
    expected_manifest_sha = _parse_manifest_sha256(manifest_sha_path)
    actual_manifest_sha = _sha256_file(manifest_path)
    if actual_manifest_sha != expected_manifest_sha:
        raise CaptureFetchError(
            "manifest.json SHA-256 mismatch: "
            f"expected {expected_manifest_sha}, got {actual_manifest_sha}"
        )

    manifest = _load_json_object(manifest_path, "manifest.json")
    if manifest.get("format") != MANIFEST_FORMAT:
        raise CaptureFetchError(f"manifest format is not {MANIFEST_FORMAT}")
    if manifest.get("capture_id") != capture_id:
        raise CaptureFetchError("manifest capture_id does not match the episode capture_id")
    if manifest.get("state") != "finalized":
        raise CaptureFetchError("manifest state is not finalized")
    if manifest.get("valid") is not True:
        raise CaptureFetchError("authoritative manifest is not valid")
    invalid_reasons = manifest.get("invalid_reasons")
    if not isinstance(invalid_reasons, list) or invalid_reasons:
        raise CaptureFetchError("authoritative manifest has invalid or non-empty invalid_reasons")

    declared_manifest_sha_path = manifest.get("manifest_sha256_path")
    if _safe_declared_basename(
        declared_manifest_sha_path,
        "manifest_sha256_path",
    ) != "manifest.sha256":
        raise CaptureFetchError("manifest_sha256_path does not name manifest.sha256")

    chunks = _alias_value(manifest, ("chunks", "chunk_files"), "chunk list")
    if not isinstance(chunks, list) or not chunks:
        raise CaptureFetchError("manifest chunk list must be a non-empty list")

    seen_paths = set()
    seen_indices = set()
    total_chunk_bytes = 0
    for position, chunk in enumerate(chunks):
        if not isinstance(chunk, dict):
            raise CaptureFetchError(f"chunk entry {position} is not an object")
        relative = _safe_relative_path(chunk.get("path"), f"chunk {position} path")
        relative_text = relative.as_posix()
        if relative.suffix != ".shc":
            raise CaptureFetchError(f"chunk {position} path must end in .shc")
        if relative_text in seen_paths:
            raise CaptureFetchError(f"duplicate chunk path: {relative_text}")
        seen_paths.add(relative_text)

        index = chunk.get("index")
        if isinstance(index, bool) or not isinstance(index, int) or index < 0:
            raise CaptureFetchError(f"chunk {position} index is invalid")
        if index in seen_indices:
            raise CaptureFetchError(f"duplicate chunk index: {index}")
        seen_indices.add(index)

        expected_size = _expected_size(
            _alias_value(chunk, ("stored_bytes", "size_bytes"), f"chunk {index} size"),
            f"chunk {index} size",
        )
        expected_hash = _expected_sha256(chunk.get("sha256"), f"chunk {index} sha256")
        chunk_path = _local_declared_file(root, relative, f"chunk {index}")
        total_chunk_bytes += _verify_size_and_hash(
            chunk_path,
            expected_size=expected_size,
            expected_sha256=expected_hash,
            label=f"chunk {index}",
        )

    if seen_indices != set(range(len(chunks))):
        raise CaptureFetchError("chunk indices must be contiguous from zero")
    counts = manifest.get("counts")
    if isinstance(counts, dict) and "chunks" in counts:
        declared_count = _expected_size(counts["chunks"], "counts.chunks")
        if declared_count != len(chunks):
            raise CaptureFetchError("counts.chunks does not match the chunk list")

    control_clock = manifest.get("control_clock")
    if control_clock is None:
        control_clock = {}
    if not isinstance(control_clock, dict):
        raise CaptureFetchError("control_clock must be an object when declared")
    clock_path_value = manifest.get("clock_samples_path", control_clock.get("path"))
    clock_hash_value = control_clock.get("sha256", manifest.get("clock_samples_sha256"))
    clock_path_result = None
    if clock_path_value is not None or clock_hash_value is not None:
        if clock_path_value is None or clock_hash_value is None:
            raise CaptureFetchError("clock path and SHA-256 must be declared together")
        clock_basename = _safe_declared_basename(clock_path_value, "clock_samples_path")
        if clock_basename != "clock_samples.jsonl":
            raise CaptureFetchError("clock_samples_path does not name clock_samples.jsonl")
        relative_clock = _safe_relative_path(clock_basename, "clock_samples basename")
        clock_path = _local_declared_file(root, relative_clock, "clock_samples file")
        clock_hash = _expected_sha256(clock_hash_value, "control_clock.sha256")
        clock_size_value = _alias_value(
            control_clock,
            ("stored_bytes", "size_bytes"),
            "control clock size",
            required=False,
        )
        clock_size = (
            None
            if clock_size_value is None
            else _expected_size(clock_size_value, "control clock size")
        )
        _verify_size_and_hash(
            clock_path,
            expected_size=clock_size,
            expected_sha256=clock_hash,
            label="clock_samples file",
        )
        if control_clock.get("closed") is not True:
            raise CaptureFetchError("control clock is not marked closed")
        if control_clock.get("writer_error") is not None:
            raise CaptureFetchError("control clock reports a writer error")
        clock_path_result = clock_basename

    expected_files = {"manifest.json", "manifest.sha256", *seen_paths}
    if clock_path_result is not None:
        expected_files.add(clock_path_result)
    _reject_unlisted_capture_artifacts(root, expected_files)

    return VerificationResult(
        capture_id=capture_id,
        manifest_sha256=actual_manifest_sha,
        chunk_count=len(chunks),
        chunk_bytes=total_chunk_bytes,
        clock_path=clock_path_result,
    )


def _validate_scp_target(target: str) -> str:
    if not isinstance(target, str) or not _SCP_TARGET_RE.fullmatch(target):
        raise CaptureFetchError(
            "--thor must be a plain [user@]hostname or IPv4 address without options"
        )
    return target


def fetch_episode_capture(
    episode_dir: os.PathLike[str] | str,
    *,
    thor: str = DEFAULT_THOR,
    allow_default_host_path: bool = False,
    expected_host_root: str = DEFAULT_HOST_CAPTURE_ROOT,
) -> tuple[Path, VerificationResult]:
    """Fetch, verify, and atomically install one episode's native capture."""
    episode = Path(episode_dir)
    if episode.is_symlink() or not episode.is_dir():
        raise CaptureFetchError(f"episode directory is missing or unsafe: {episode}")
    episode = episode.resolve(strict=True)
    reference = load_capture_reference(
        episode,
        allow_default_host_path=allow_default_host_path,
        expected_host_root=expected_host_root,
    )
    target = _validate_scp_target(thor)
    final_path = episode / FETCHED_CAPTURE_DIRNAME
    if os.path.lexists(final_path):
        raise CaptureFetchError(f"refusing to overwrite existing path: {final_path}")

    partial_root = Path(
        tempfile.mkdtemp(prefix=f".{FETCHED_CAPTURE_DIRNAME}.partial-", dir=episode)
    )
    try:
        source = f"{target}:{reference.host_capture_path}"
        command = ["scp", "-r", source, str(partial_root)]
        try:
            subprocess.run(command, check=True, shell=False)
        except (OSError, subprocess.CalledProcessError) as exc:
            raise CaptureFetchError(f"scp transfer failed: {exc}") from exc

        entries = list(partial_root.iterdir())
        if len(entries) != 1 or entries[0].name != reference.capture_id:
            raise CaptureFetchError(
                "scp result must contain exactly one directory named by capture_id"
            )
        fetched_path = entries[0]
        result = verify_capture_directory(fetched_path, reference.capture_id)

        if os.path.lexists(final_path):
            raise CaptureFetchError(f"refusing to overwrite existing path: {final_path}")
        os.rename(fetched_path, final_path)
        partial_root.rmdir()
        return final_path, result
    finally:
        if partial_root.exists():
            shutil.rmtree(partial_root)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Fetch and verify a finalized Thor-native Sharpa capture.",
    )
    parser.add_argument("episode_dir", help="Episode directory containing sharpa_native_capture.json")
    parser.add_argument(
        "--thor",
        default=DEFAULT_THOR,
        help=f"SCP target as [user@]host (default: {DEFAULT_THOR})",
    )
    parser.add_argument(
        "--expected-host-root",
        default=DEFAULT_HOST_CAPTURE_ROOT,
        help="Required Thor host capture root "
        f"(default: {DEFAULT_HOST_CAPTURE_ROOT}).",
    )
    parser.add_argument(
        "--allow-default-host-path",
        action="store_true",
        help="If old metadata lacks host paths, explicitly allow the documented fallback "
        f"{DEFAULT_HOST_CAPTURE_ROOT}/<capture_id>.",
    )
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        installed_path, result = fetch_episode_capture(
            args.episode_dir,
            thor=args.thor,
            allow_default_host_path=args.allow_default_host_path,
            expected_host_root=args.expected_host_root,
        )
    except CaptureFetchError as exc:
        raise SystemExit(f"error: {exc}") from exc

    print(
        json.dumps(
            {
                "capture_id": result.capture_id,
                "installed_path": str(installed_path),
                "manifest_sha256": result.manifest_sha256,
                "chunk_count": result.chunk_count,
                "chunk_bytes": result.chunk_bytes,
                "clock_path": result.clock_path,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
