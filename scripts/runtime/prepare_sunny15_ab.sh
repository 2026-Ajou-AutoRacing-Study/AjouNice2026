#!/usr/bin/env bash
# Execute inside the qualified Vast Python environment. Default: prepare only.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ "$#" -lt 3 ]]; then
  echo "Usage: bash $0 cosine|restarts JPEG_PORTABLE WORK_ROOT [extra runner arguments]" >&2
  exit 2
fi
variant="$1"; data="$2"; work_root="$3"; shift 3
case "$variant" in
  cosine) schedule=cosine ;;
  restarts) schedule=warmup-restarts ;;
  *) echo "Unknown variant" >&2; exit 2 ;;
esac
initializer="${BASE5_INITIALIZER:-/workspace/base5_nuscenes_full_xy_metric_v1_261011.pth}"
export PYTHONPATH="$ROOT:$ROOT/third_party/mmdetection3d:$ROOT/docker/bevfusion${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1
export WANDB_ENTITY=AjouNice2026 WANDB_MODE=online WANDB_DISABLE_CODE=true
exec "${BASE5_TRAINING_PYTHON:-python}" \
  "$ROOT/scripts/runtime/base5_distance_diagnostics.py" \
  --frozen-entrypoint "$ROOT/perception/detection3d/bevfusion/tools/run_base5_direct_training.py" \
  --config "$ROOT/perception/detection3d/bevfusion/configs/bevfusion_base5_fine_512_depth25_next_training.py" \
  --data "$data" --work-dir "$work_root/$variant" \
  --profile 512x1056 --epochs 8 --batch 4 --accumulate 1 --workers 4 \
  --lr 1e-4 --warmup-updates 500 --seed 261018 \
  --lr-schedule "$schedule" --restart-cycle-epochs 4 --restart-warmup-updates 500 \
  --init-checkpoint "$initializer" --wandb --wandb-group base5-sunny15-ab-261018 "$@"
