#!/usr/bin/env bash
# Average the first N frames of `states` across one or more xr_teleoperate datasets.
#
# Usage:
#   bash run_compute_average_initial_state.sh <dataset_dir> [<dataset_dir>...] [-- extra args]
#
# Examples:
#   # Single dataset, defaults: first 10 frames, output ./average_initial_state.yaml
#   bash run_compute_average_initial_state.sh ~/Assemble_trocar_h2_sharpa/assemble_trocar_05121740
#
#   # Several datasets pooled together
#   bash run_compute_average_initial_state.sh \
#       ~/Assemble_trocar_h2_sharpa/assemble_trocar_05121740 \
#       ~/Assemble_trocar_h2_sharpa/assemble_trocar_05131200
#
#   # Override first-n / output by appending extra args after a `--` separator
#   bash run_compute_average_initial_state.sh ~/datasets/foo -- --first-n 20 --output ~/avg.yaml
set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <dataset_dir> [<dataset_dir>...] [-- --first-n N] [-- --output PATH]" >&2
    exit 1
fi

# Split the positional dataset dirs from any extra flags after "--".
DATASETS=()
EXTRA=()
seen_sep=0
for arg in "$@"; do
    if [[ "$arg" == "--" ]]; then
        seen_sep=1
        continue
    fi
    if (( seen_sep )); then
        EXTRA+=("$arg")
    else
        DATASETS+=("$arg")
    fi
done

if [[ ${#DATASETS[@]} -eq 0 ]]; then
    echo "ERROR: no dataset directories given." >&2
    exit 1
fi

# Defaults — only applied if the user didn't pass them via "-- ...".
have_first_n=0
have_output=0
for a in "${EXTRA[@]:-}"; do
    [[ "$a" == "--first-n" ]] && have_first_n=1
    [[ "$a" == "--output"  ]] && have_output=1
done
(( have_first_n )) || EXTRA+=(--first-n 10)
(( have_output  )) || EXTRA+=(--output "./average_initial_state.yaml")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Run under the "tv" conda env (where PyYAML / numpy are installed).
# Override with $PYTHON if you want a different interpreter.
PYTHON="${PYTHON:-conda run -n tv --live-stream python}"

$PYTHON "${SCRIPT_DIR}/compute_average_initial_state.py" \
    --dataset-dirs "${DATASETS[@]}" \
    "${EXTRA[@]}"
