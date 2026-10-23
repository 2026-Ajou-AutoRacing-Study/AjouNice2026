#!/usr/bin/env bash
# Existing team Docker/ROS installation -> selected pinned weights + incremental build.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
install_args=()
skip_build=false
skip_docker=false
skip_acceleration=false
model=e12
while (($#)); do
  case "$1" in
    --model) model="${2:?Missing model: e12, e24, or e25}"; shift 2 ;;
    --archive|--remote) install_args+=("$1" "${2:?Missing option value}"); shift 2 ;;
    --skip-build) skip_build=true; shift ;;
    --skip-docker-check) skip_docker=true; shift ;;
    --skip-acceleration) skip_acceleration=true; shift ;;
    --check-only) skip_build=true; skip_docker=true; skip_acceleration=true; install_args+=("$1"); shift ;;
    --help|-h)
      echo "Usage: bash scripts/runtime/install_base5_production.sh [--model e12|e24|e25] [--remote ajou_drive] [--archive ZIP] [--skip-build] [--skip-acceleration] [--check-only]"
      echo "기본 E12 ZIP/가중치를 설치하며 기존 E24/E25는 덮어쓰지 않습니다. --model e24는 보존된 E24 ZIP, e25는 기존 legacy 가중치를 확인합니다."
      echo "Drive ZIP 검증·자동 설치와 기존 ROS workspace 증분 빌드. MORAI/주행은 시작하지 않습니다."
      echo "팀 공용 Docker 이미지와 ROS workspace는 기존 설치가 필요합니다. sudo/Google 로그인은 자동 수행하지 않습니다."
      echo "E12는 기본적으로 고정 TRT overlay 이미지를 로컬 빌드하고 이 PC GPU의 engine을 1회 준비합니다. --skip-acceleration은 같은 E12 원형 경로를 준비합니다."
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
case "$model" in e12|e24|e25) ;; *) echo '[ERROR] --model must be e12, e24, or e25.' >&2; exit 2 ;; esac
cd "$ROOT"
if [[ "$skip_build" != true ]]; then
  for path in base_ws/src/ekf-multi-object-tracking third_party/mmdetection3d; do
    if [[ -e "$path/.git" && -n "$(git -C "$path" status --porcelain)" ]]; then
      echo "[ERROR] Modified submodule: $path. Preserve its changes before updating." >&2
      exit 1
    fi
  done
  git submodule update --init base_ws/src/ekf-multi-object-tracking third_party/mmdetection3d
fi
python3 scripts/runtime/base5_production_bundle.py install --model "$model" "${install_args[@]}"
if [[ "$skip_docker" != true ]]; then
  image="${BEVFUSION_IMAGE:-ajounice/bevfusion:team-cu128-v1}"
  docker image inspect "$image" >/dev/null
  docker compose version
  # Only imports and a tiny CUDA tensor; does not load a detector or claim FPS.
  docker run --rm --gpus all --entrypoint python "$image" -c \
    'import torch, mmcv.ops, mmdet3d, spconv.pytorch; assert torch.cuda.is_available(); print(torch.cuda.get_device_name()); print((torch.ones(2, device="cuda") + 1).cpu()); print("Base5 Docker dependencies: OK")'
fi
if [[ "$skip_build" != true ]]; then
  # build_all alone can skip an already-installed planning workspace. Refresh
  # planning explicitly so a full-branch update also installs its new C++ code.
  bash "$ROOT/planning_ws/tools/build.sh"
  bash "$ROOT/scripts/workspace/build_all.sh"
fi
if [[ "$model" == e12 && "$skip_acceleration" != true && "$skip_docker" != true ]]; then
  trt_image=ajounice/bevfusion:team-trt-cu128-261023-v1
  if docker build --pull=false -f "$ROOT/docker/bevfusion/Dockerfile.production-tensorrt" \
      -t "$trt_image" "$ROOT" && \
      python3 "$ROOT/docker/bevfusion/scripts/setup_production_tensorrt.py" --execute --image "$trt_image"; then
    echo '[ACCELERATION READY] 이 PC의 E12 camera TRT + SDPA/cache 준비 완료. 실제 활성 상태는 서버 status로 확인하세요.'
  else
    echo '[WARNING] 선택 TRT 준비를 완료하지 못했습니다. 기존 성공 receipt는 보존하며, receipt가 없으면 같은 E12 eager로 시작합니다.' >&2
    echo '[RETRY] python3 docker/bevfusion/scripts/setup_production_tensorrt.py --execute' >&2
  fi
fi
echo "[READY] 선택 모델 $model 준비 완료. 새로운 터미널에서 기존 풀스택 순서대로 실행하세요."
echo "[DETECTOR] bevfusion 또는 bash scripts/runtime/run_bevfusion.sh"
echo "[VALIDATOR] Base5 E12 production; E24/E25 복귀는 명시적으로 선택."
echo "[ROLLBACK] bevfusion --model e24 또는 bevfusion --model e25"
echo "[PREFLIGHT] python3 scripts/runtime/preflight_bevfusion_competition.py --safe"
echo "[SAFE START] bevfusion --safe (same selected weights; optional accelerators disabled)"
