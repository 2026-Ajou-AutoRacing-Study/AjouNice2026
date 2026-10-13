#!/usr/bin/env bash
# Existing team Docker/ROS installation -> pinned E24 weights + incremental build.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
install_args=()
skip_build=false
skip_docker=false
while (($#)); do
  case "$1" in
    --archive|--remote) install_args+=("$1" "${2:?Missing option value}"); shift 2 ;;
    --skip-build) skip_build=true; shift ;;
    --skip-docker-check) skip_docker=true; shift ;;
    --check-only) skip_build=true; install_args+=("$1"); shift ;;
    --help|-h)
      echo "Usage: bash scripts/runtime/install_base5_production.sh [--remote ajou_drive] [--archive ZIP] [--skip-build] [--check-only]"
      echo "Drive ZIP 검증·자동 설치와 기존 ROS workspace 증분 빌드. MORAI/주행은 시작하지 않습니다."
      echo "팀 공용 Docker 이미지와 ROS workspace는 기존 설치가 필요합니다. sudo/Google 로그인은 자동 수행하지 않습니다."
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
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
python3 scripts/runtime/base5_production_bundle.py install "${install_args[@]}"
if [[ "$skip_docker" != true ]]; then
  image="${BEVFUSION_IMAGE:-ajounice/bevfusion:team-cu128-v1}"
  docker image inspect "$image" >/dev/null
  docker compose version
  # Only imports and a tiny CUDA tensor; does not load a detector or claim FPS.
  docker run --rm --gpus all --entrypoint python "$image" -c \
    'import torch, mmcv.ops, mmdet3d, spconv.pytorch; assert torch.cuda.is_available(); print(torch.cuda.get_device_name()); print((torch.ones(2, device="cuda") + 1).cpu()); print("Base5 Docker dependencies: OK")'
fi
if [[ "$skip_build" != true ]]; then
  set +u
  source "$ROOT/scripts/workspace/source_aw149.sh"
  set -u
  catkin_make_isolated --install \
    --source "$ROOT/planning_ws/src" --build "$ROOT/planning_ws/build_isolated" \
    --devel "$ROOT/planning_ws/devel_isolated" --install-space "$ROOT/planning_ws/install_isolated" \
    --pkg autoware_launch -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    --make-args -j4 -l4
  catkin_make_isolated --install \
    --source "$ROOT/base_ws/src" --build "$ROOT/base_ws/build_aw149" \
    --devel "$ROOT/base_ws/devel_aw149" --install-space "$ROOT/base_ws/install_aw149" \
    --pkg perception_object_pipeline ekf_multi_object_tracking bevfusion_ros planning_scenario_validator \
    -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 --make-args -j4 -l4
fi
echo "[READY] Base5 E24 준비 완료. 새로운 터미널에서 기존 풀스택 순서대로 실행하세요."
echo "[DETECTOR] bevfusion 또는 bash scripts/runtime/run_bevfusion.sh"
echo "[VALIDATOR] Base5 E24 production; E25 복귀는 명시적으로 e25 선택."
echo "[ROLLBACK] bevfusion --model e25"
