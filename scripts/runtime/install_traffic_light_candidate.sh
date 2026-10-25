#!/usr/bin/env bash
# Existing ROS workspace -> pinned opt-in models + isolated inference env.
# No nodes, scenario loads, driving commands, public sharing or production switch.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODEL_ARGS=()
PREFIX="$HOME/.local/share/ajounice/traffic_light_gpu"
EXISTING_PYTHON=""
DEVICE=0
CHECK=false
MODELS_ONLY=false
SKIP_BUILD=false
usage() {
  echo '사용: bash scripts/runtime/install_traffic_light_candidate.sh [--archive TAR] [--remote RCLONE_NAME] [--models-dir DIR] [--env-prefix DIR] [--existing-python PYTHON] [--device 0|cpu] [--models-only] [--skip-build] [--check]'
  echo '기존 ROS/풀스택 workspace가 필요합니다. 기본값은 새 모델 설치 + 전용 Python 환경 준비 + 두 모델 GPU 검사 + 관련 패키지 증분 설치입니다.'
  echo '승인된 모델 압축본은 Drive 로그인/rclone 없이 받습니다. 오프라인은 Downloads/final_weights_bundle.tar.gz 또는 --archive를 사용하세요. 공유 권한·시스템 Python·기존 모델 기본값은 바꾸지 않습니다.'
  echo '--models-only: 모델 파일만 준비, 환경 설치/추론/빌드 없음. --existing-python: 지정 interpreter를 변경하지 않고 검사. --check: 다운로드/설치/빌드 없이 기존 모델·환경 검사.'
}
while (($#)); do
  case "$1" in
    --archive|--remote|--models-dir) MODEL_ARGS+=("$1" "${2:?Missing option value}"); shift 2 ;;
    --env-prefix) PREFIX="${2:?Missing prefix}"; shift 2 ;;
    --existing-python) EXISTING_PYTHON="${2:?Missing Python}"; shift 2 ;;
    --device) DEVICE="${2:?Missing device}"; shift 2 ;;
    --models-only) MODELS_ONLY=true; SKIP_BUILD=true; shift ;;
    --skip-build) SKIP_BUILD=true; shift ;;
    --check) CHECK=true; SKIP_BUILD=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
[[ "$DEVICE" == cpu || "$DEVICE" =~ ^[0-9]+$ ]] || { echo '장치는 cpu 또는 GPU 번호입니다' >&2; exit 2; }
if $CHECK; then MODEL_ARGS+=(--check); fi
/usr/bin/python3 "$ROOT/scripts/runtime/traffic_light_candidate.py" "${MODEL_ARGS[@]}"
if $MODELS_ONLY; then
  echo '[MODELS VERIFIED ONLY] GPU/ROS runtime은 아직 검사하지 않았습니다. 기존 운영 설정은 유지됩니다.'
  exit 0
fi
mapfile -t MODEL_INFO < <(/usr/bin/python3 - "$ROOT" "${MODEL_ARGS[@]}" <<'PY'
import json,sys
sys.path.insert(0,sys.argv[1]+'/scripts/runtime')
from traffic_light_candidate import load_release,paths
args=sys.argv[2:];directory=args[args.index('--models-dir')+1] if '--models-dir' in args else None
doc=load_release();models=paths(directory,doc)
for value in (models['yolo'],models['cnn'],doc['models']['yolo']['sha256'],doc['models']['cnn']['sha256']):print(value)
PY
)
[[ ${#MODEL_INFO[@]} == 4 ]] || { echo '모델 경로 확인 실패' >&2; exit 1; }
CHECK_ARGS=(--device "$DEVICE" --weights "${MODEL_INFO[0]}" --cnn-checkpoint "${MODEL_INFO[1]}" --imgsz 960
  --expected-weights-sha256 "${MODEL_INFO[2]}" --expected-cnn-sha256 "${MODEL_INFO[3]}")
if [[ -n "$EXISTING_PYTHON" ]]; then
  export AJOUNICE_ROOT="$ROOT"
  set +u
  source "$ROOT/scripts/workspace/source_aw149.sh"
  set -u
  PYTHONNOUSERSITE=1 "$EXISTING_PYTHON" -s "$ROOT/scripts/runtime/check_traffic_light_env.py" \
    "${CHECK_ARGS[@]}" --check-ros --require-team-versions --isolate-python
else
  ENV_ARGS=(--prefix "$PREFIX")
  if $CHECK; then ENV_ARGS+=(--check); fi
  bash "$ROOT/scripts/runtime/setup_traffic_light_env.sh" "${ENV_ARGS[@]}" "${CHECK_ARGS[@]}"
fi
if ! $SKIP_BUILD; then
  if [[ -f "$ROOT/base_ws/build_aw149/traffic_light_detection/CMakeCache.txt" && -f "$ROOT/base_ws/build_aw149/planning_scenario_validator/CMakeCache.txt" ]]; then
    for package in traffic_light_detection planning_scenario_validator; do
      cmake --build "$ROOT/base_ws/build_aw149/$package" --target install -- -j2
    done
  else
    export AJOUNICE_ROOT="$ROOT"
    set +u
    source "$ROOT/scripts/workspace/source_aw149.sh"
    set -u
    catkin_make_isolated --install --source "$ROOT/base_ws/src" --build "$ROOT/base_ws/build_aw149" \
      --devel "$ROOT/base_ws/devel_aw149" --install-space "$ROOT/base_ws/install_aw149" \
      --pkg traffic_light_detection planning_scenario_validator -DCMAKE_BUILD_TYPE=Release \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 --make-args -j2 -l2
  fi
fi
echo '[CANDIDATE READY] 두 모델과 선택 장치 추론·ROS import 검사 완료. 주행/production 전환은 시작하지 않았습니다.'
echo '[GUI] 신호등 모델에서 v5 지도+CNN 후보를 선택하고 새 실험을 시작하세요. 기존 실행 사본은 수정하지 마세요.'
