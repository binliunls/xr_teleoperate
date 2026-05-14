"""
Compute the average "initial state" across a set of xr_teleoperate datasets.

For each dataset directory passed in, walks every `episode_*/data.json`,
takes the first --first-n frames' `states` from each episode, accumulates
them, and writes the per-joint mean as a YAML file shaped like the source
state dict.

Each xr_teleoperate `data.json` looks like:

    {
      "info": {...},
      "text": {...},
      "data": [
        { "idx": 0, "states": { "left_arm": {"qpos": [...]}, ... }, ... },
        { "idx": 1, "states": { ... }, ... },
        ...
      ]
    }

Output YAML:

    first_n_frames: 10
    num_episodes: 42
    num_frames: 420
    states:
      left_arm:
        qpos: [..., ..., ...]
      right_arm:
        qpos: [..., ..., ...]
      left_ee:
        qpos: [..., ..., ...]
      ...

Usage:
    python -m teleop.utils.compute_average_initial_state \\
        --dataset-dirs ~/Assemble_trocar_h2_sharpa/assemble_trocar_05121740 \\
                       ~/Assemble_trocar_h2_sharpa/assemble_trocar_05131200 \\
        --first-n 10 \\
        --output ~/average_initial_state.yaml
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml


# State sub-keys we expect to average. Any sub-key present in the data is
# averaged automatically; this list just controls the output ordering.
STATE_GROUPS = ("left_arm", "right_arm", "left_ee", "right_ee", "body")
JOINT_FIELDS = ("qpos", "qvel", "torque")  # which list-of-floats to average per group


def _iter_episode_jsons(dataset_dirs: Iterable[Path]) -> Iterable[Path]:
    for d in dataset_dirs:
        d = Path(d).expanduser().resolve()
        if not d.is_dir():
            print(f"[skip] not a directory: {d}", file=sys.stderr)
            continue
        for episode in sorted(d.glob("episode_*")):
            data_json = episode / "data.json"
            if data_json.is_file():
                yield data_json
            else:
                print(f"[skip] no data.json under {episode}", file=sys.stderr)


def _accumulate_episode(
    data_json: Path,
    first_n: int,
    sums: dict[tuple[str, str], np.ndarray],
    counts: dict[tuple[str, str], int],
) -> int:
    """Read first_n frames from one episode, add to sums/counts. Returns frames added."""
    try:
        with open(data_json, "r", encoding="utf-8") as f:
            obj = json.load(f)
    except Exception as e:
        print(f"[skip] failed to read {data_json}: {e}", file=sys.stderr)
        return 0

    frames = obj.get("data") or []
    if not frames:
        print(f"[skip] empty data: {data_json}", file=sys.stderr)
        return 0

    n_frames = 0
    for frame in frames[:first_n]:
        states = frame.get("states") or {}
        for group, group_state in states.items():
            if not isinstance(group_state, dict):
                continue
            for field in JOINT_FIELDS:
                values = group_state.get(field)
                if not values:  # empty list or missing → skip
                    continue
                arr = np.asarray(values, dtype=np.float64)
                key = (group, field)
                if key not in sums:
                    sums[key] = arr.copy()
                    counts[key] = 1
                else:
                    if arr.shape != sums[key].shape:
                        print(
                            f"[warn] shape mismatch in {data_json} for {group}.{field}: "
                            f"got {arr.shape}, expected {sums[key].shape}; skipping frame.",
                            file=sys.stderr,
                        )
                        continue
                    sums[key] += arr
                    counts[key] += 1
        n_frames += 1
    return n_frames


def compute(dataset_dirs: list[Path], first_n: int) -> tuple[dict, int, int]:
    sums: dict[tuple[str, str], np.ndarray] = {}
    counts: dict[tuple[str, str], int] = {}
    n_episodes = 0
    total_frames = 0
    for ep_json in _iter_episode_jsons(dataset_dirs):
        added = _accumulate_episode(ep_json, first_n, sums, counts)
        if added > 0:
            n_episodes += 1
            total_frames += added

    if n_episodes == 0:
        raise RuntimeError("No usable episodes found.")

    # Build nested averages preserving STATE_GROUPS ordering.
    avg: dict = {}
    seen_groups = {g for (g, _) in sums.keys()}
    ordered_groups = [g for g in STATE_GROUPS if g in seen_groups] + sorted(seen_groups - set(STATE_GROUPS))
    for group in ordered_groups:
        avg[group] = {}
        for field in JOINT_FIELDS:
            key = (group, field)
            if key in sums:
                avg[group][field] = (sums[key] / counts[key]).tolist()

    return avg, n_episodes, total_frames


def write_yaml(out_path: Path, avg: dict, first_n: int, n_episodes: int, total_frames: int):
    payload = {
        "first_n_frames": first_n,
        "num_episodes": n_episodes,
        "num_frames": total_frames,
        "states": avg,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, default_flow_style=None, sort_keys=False, allow_unicode=True)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--dataset-dirs",
        nargs="+",
        required=True,
        type=Path,
        help="One or more dataset directories. Each must contain episode_*/data.json subfolders.",
    )
    p.add_argument(
        "--first-n",
        type=int,
        default=10,
        help="Number of initial frames per episode to include (default: 10).",
    )
    p.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output YAML path.",
    )
    return p.parse_args()


def main():
    args = parse_args()
    if args.first_n <= 0:
        sys.exit(f"--first-n must be > 0 (got {args.first_n})")

    avg, n_eps, n_frames = compute(args.dataset_dirs, args.first_n)
    write_yaml(args.output.expanduser().resolve(), avg, args.first_n, n_eps, n_frames)
    print(f"[ok] {n_eps} episodes, {n_frames} frames → {args.output}")
    for group, fields in avg.items():
        for field, vals in fields.items():
            print(f"  {group:10s}.{field:7s}  dim={len(vals)}  " f"min={min(vals):+.4f}  max={max(vals):+.4f}")


if __name__ == "__main__":
    main()
