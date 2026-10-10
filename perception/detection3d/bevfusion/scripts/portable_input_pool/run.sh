#!/usr/bin/env bash
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Vast is already a container: run in it directly, never nested Docker.
if [[ -f /.dockerenv ]] || [[ "${AJOU_DIRECT_CONTAINER:-0}" == 1 ]]; then
    python "$here/run_inside.py" "$@"
else
    docker run --rm --gpus all --shm-size=4g \
        -v "$here:/bench" \
        -v "$HOME/.cache/torch_extensions:/root/.cache/torch_extensions" \
        -e MAX_JOBS=2 --entrypoint python \
        "${AJOU_BENCH_IMAGE:-ajounice/bevfusion:team-cu128-v1}" \
        /bench/run_inside.py "$@"
fi
