#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u

usage() {
    echo "Usage: bash teleop/run_teleop.sh TASK [TELEOP_ARGS...]" >&2
}

if [[ $# -eq 0 ]]; then
    usage
    exit 2
fi
if [[ $1 == "-h" || $1 == "--help" ]]; then
    usage
    exit 0
fi
if [[ -z $1 ]]; then
    echo "error: TASK must not be empty" >&2
    usage
    exit 2
fi

task_name=$1
shift

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd -- "$script_dir"
exec python teleop_hand_and_arm.py \
    --arm H2 --ee sharpa --motion \
    --camera-source ros --head-mode mono \
    --record --task-dir "$HOME/datasets" --task-name "$task_name" \
    "$@"
