#!/usr/bin/env bash
# ROS Noetic가 준비된 팀 PC에서 도구를 확인하고 MORAI Network를 준비한다.
# 기본은 도구 검사만. --bevfusion은 Docker/Drive image/E25 자동 준비를 선택한다.
set -euo pipefail
REQUIRE_GUI=false
CHECK_ONLY=false
BEVFUSION=false
DRY_RUN=false
while (($#)); do
case "$1" in
  --gui) REQUIRE_GUI=true; shift ;;
  --check) REQUIRE_GUI=true; CHECK_ONLY=true; shift ;;
  --bevfusion) REQUIRE_GUI=true; BEVFUSION=true; shift ;;
  --dry-run) DRY_RUN=true; shift ;;
  --help|-h)
    echo "Usage: bash install.sh [--gui|--check|--bevfusion [--dry-run]]"
    echo "도구 확인 + validator Ego Network 생성. --gui는 tkinter도 필수로 검사합니다."
    echo "--check는 도구만 확인하며 Network·설정을 변경하지 않습니다."
    echo "--bevfusion: Ubuntu 20.04 Docker/Compose/Toolkit + rclone + 공용 image/E25 준비."
    echo "본인 Google 로그인/sudo가 필요하며 Docker 그룹 신규 추가 시 재로그인합니다."
    echo "--bevfusion --dry-run은 변경 없이 설치 계획만 표시합니다."
    echo "처음 설치: docs/planning/SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md"
    exit 0 ;;
  *) echo "Usage: bash install.sh [--gui|--check|--bevfusion [--dry-run]]" >&2; exit 2 ;;
esac
done
if [[ "$CHECK_ONLY" == true && ( "$BEVFUSION" == true || "$DRY_RUN" == true ) ]]; then
  echo "Usage: --check와 자동 설치 옵션은 같이 쓰지 않습니다." >&2; exit 2
fi
if [[ "$DRY_RUN" == true && "$BEVFUSION" != true ]]; then
  echo "Usage: --dry-run은 --bevfusion과 같이 사용합니다." >&2; exit 2
fi
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$BEVFUSION" == true ]]; then
  if [[ "$DRY_RUN" == true ]]; then
    exec /usr/bin/python3 "$PACKAGE_ROOT/scripts/setup_bevfusion_inference.py" --dry-run
  fi
  /usr/bin/python3 "$PACKAGE_ROOT/scripts/setup_bevfusion_inference.py"
fi
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi
MISSING=()
for command_name in ffmpeg xdpyinfo roslaunch rosbag python3; do
  if ! command -v "$command_name" >/dev/null 2>&1; then MISSING+=("$command_name"); fi
done
if ! /usr/bin/python3 -c 'import yaml, rospy, rosgraph' >/dev/null 2>&1; then
  MISSING+=("python3-yaml/rospy/rosgraph")
fi
if ! /usr/bin/python3 -c 'import gi; gi.require_version("Wnck", "3.0"); from gi.repository import Wnck' >/dev/null 2>&1; then
  MISSING+=("python3-gi/gir1.2-wnck-3.0")
fi
if ! /usr/bin/python3 -c 'import tkinter' >/dev/null 2>&1; then
  if [[ "$REQUIRE_GUI" == true ]]; then
    MISSING+=("python3-tk")
  else
    echo "참고: GUI를 쓰려면 sudo apt install python3-tk (CLI는 그대로 사용 가능)" >&2
  fi
fi
if ((${#MISSING[@]})); then
  echo "필요한 도구가 없습니다: ${MISSING[*]}" >&2
  echo "Ubuntu: sudo apt install ffmpeg x11-utils python3-yaml python3-tk python3-gi gir1.2-wnck-3.0" >&2
  echo "ROS Noetic는 별도 설치가 필요합니다. 설치 후 scripts/workspace/setup_ws.sh로 workspace를 빌드하세요." >&2
  echo "처음 설치: docs/planning/SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md" >&2
  exit 1
fi
if ! test -f "$PACKAGE_ROOT/../../../install_aw149/setup.bash"; then
  echo "주의: AW149 overlay가 없습니다. 저장소 루트에서 scripts/workspace/setup_ws.sh를 실행하세요." >&2
fi
if [[ "$CHECK_ONLY" == true ]]; then
  echo "GUI·녹화 도구 검사 완료. Network·설정은 변경하지 않았습니다."
  echo "GUI 고급/네트워크 탭에서 환경 검사 → 필요 시 실제 GPU 검사를 실행하세요."
  exit 0
fi
NETWORK_PROFILE="$(/usr/bin/python3 "$PACKAGE_ROOT/scripts/generate_network_profile.py")"
echo "MORAI GUI에서 로드할 Ego Network: $NETWORK_PROFILE"
echo "기본 도구 확인 완료. Google Drive 자동 업로드를 사용하려면 rclone 설치와 remote 로그인이 추가로 필요합니다."
