#!/usr/bin/env bash
# Separate host inference environment. Never upgrades system Python or existing Conda envs.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PREFIX="$HOME/.local/share/ajounice/traffic_light_gpu"
CONDA_BIN=""
CHECK_ONLY=false
DEVICE=0
usage() {
  echo "사용: bash scripts/runtime/setup_traffic_light_env.sh [--prefix 경로] [--conda-exe 경로] [--device 0|cpu] [--check]"
  echo "기본: ~/.local/share/ajounice/traffic_light_gpu (Python 3.11 / Torch 2.7.1 cu128)"
  echo "--check: 설치/변경 없이 해당 환경의 모델·GPU·ROS import만 검사"
}
while (($#)); do
  case "$1" in
    --prefix|--conda-exe|--device)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      case "$1" in
        --prefix) PREFIX="$2" ;;
        --conda-exe) CONDA_BIN="$2" ;;
        --device) DEVICE="$2" ;;
      esac
      shift 2 ;;
    --check) CHECK_ONLY=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
[[ "$DEVICE" == cpu || "$DEVICE" =~ ^[0-9]+$ ]] || { echo "장치는 cpu 또는 GPU 번호입니다" >&2; exit 2; }
PREFIX="$(realpath -m -- "$PREFIX")"
if ! $CHECK_ONLY; then
  [[ "$PREFIX" == "$HOME/"* && "$PREFIX" != "$HOME" ]] || { echo "설치 경로는 홈 폴더 아래 새 전용 경로여야 합니다" >&2; exit 2; }
  [[ "$(uname -m)" == x86_64 ]] || { echo "이 설치기는 x86_64 Linux용입니다" >&2; exit 2; }
  mkdir -p "$(dirname "$PREFIX")"
  exec 9>"${PREFIX}.install.lock"
  flock -n 9 || { echo "같은 환경 설치가 진행 중입니다" >&2; exit 2; }
  MARKER="${PREFIX}.ajounice-managed"
  if [[ -e "$PREFIX" && ! -f "$MARKER" ]]; then
    echo "기존 환경은 변경하지 않습니다: $PREFIX. --check로 검사하거나 다른 --prefix를 지정하세요" >&2
    exit 2
  fi
  if [[ ! -f "$PREFIX/.ajounice-team-v1-ready" ]]; then
    # No activation, bashrc edits or global pip installations.
    if [[ -z "$CONDA_BIN" ]]; then
      for candidate in "${CONDA_EXE:-}" "$HOME/miniconda3/bin/conda" "$HOME/miniforge3/bin/conda" "$HOME/anaconda3/bin/conda" "$HOME/.local/share/ajounice/miniforge3/bin/conda"; do
        if [[ -n "$candidate" && -x "$candidate" ]]; then CONDA_BIN="$candidate"; break; fi
      done
    fi
    if [[ -z "$CONDA_BIN" ]]; then
      command -v curl >/dev/null || { echo "sudo apt install curl 로 curl을 먼저 설치하세요" >&2; exit 2; }
      MANAGER="$HOME/.local/share/ajounice/miniforge3"
      [[ ! -e "$MANAGER" ]] || { echo "불완전한 Miniforge가 있습니다: $MANAGER. --conda-exe로 기존 Conda를 지정하세요" >&2; exit 2; }
      TEMP="$(mktemp -d)"
      trap 'rm -rf -- "$TEMP"' EXIT
      URL="https://github.com/conda-forge/miniforge/releases/download/25.3.1-0/Miniforge3-25.3.1-0-Linux-x86_64.sh"
      curl -fL --retry 3 "$URL" -o "$TEMP/miniforge.sh"
      echo "376b160ed8130820db0ab0f3826ac1fc85923647f75c1b8231166e3d559ab768  $TEMP/miniforge.sh" | sha256sum -c -
      bash "$TEMP/miniforge.sh" -b -p "$MANAGER"
      CONDA_BIN="$MANAGER/bin/conda"
    fi
    [[ -x "$CONDA_BIN" ]] || { echo "Conda 실행 파일이 없습니다: $CONDA_BIN" >&2; exit 2; }
    # A marker outside the prefix allows resuming an interrupted installation.
    touch "$MARKER"
    if [[ ! -x "$PREFIX/bin/python" ]]; then
      env -u PYTHONPATH -u PYTHONHOME "$CONDA_BIN" create --yes --prefix "$PREFIX" --override-channels -c conda-forge python=3.11 pip
    fi
    env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 "$PREFIX/bin/python" -m pip --isolated install \
      torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cu128
    env -u PYTHONPATH -u PYTHONHOME PYTHONNOUSERSITE=1 "$PREFIX/bin/python" -m pip --isolated install \
      -r "$ROOT/base_ws/src/perception/traffic_light_detection/requirements-runtime.txt" \
      rospkg==1.6.1 catkin-pkg==1.1.0 PyYAML==6.0.3
    env -u PYTHONPATH -u PYTHONHOME "$PREFIX/bin/python" -m pip --isolated check
  fi
fi
[[ -x "$PREFIX/bin/python" ]] || { echo "Python이 없습니다: $PREFIX/bin/python" >&2; exit 2; }
# Source ROS only after package installation; pure-Python ROS messages can be
# imported by Python 3.11. Lanelet2 stays in its separate system-Python process.
export AJOUNICE_ROOT="$ROOT"
set +u
source "$ROOT/scripts/workspace/source_aw149.sh"
set -u
export PYTHONNOUSERSITE=1
"$PREFIX/bin/python" -s "$ROOT/scripts/runtime/check_traffic_light_env.py" \
  --device "$DEVICE" --check-ros --require-team-versions --isolate-python
if ! $CHECK_ONLY; then touch "$PREFIX/.ajounice-team-v1-ready"; fi
echo "설치/검사 완료. GUI에서 가상환경 Python 선택: $PREFIX/bin/python (장치: $DEVICE)"
