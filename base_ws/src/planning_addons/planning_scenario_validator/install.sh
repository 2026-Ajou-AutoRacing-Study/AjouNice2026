#!/usr/bin/env bash
# ROS Noetic가 준비된 팀 PC에서 도구를 확인하고 MORAI Network를 준비한다.
# ROS/Docker 설치, workspace 빌드, 맵/가중치 다운로드는 수행하지 않는다.
set -euo pipefail
REQUIRE_GUI=false
case "${1:-}" in
  --gui) REQUIRE_GUI=true; shift ;;
  --help|-h)
    echo "Usage: bash install.sh [--gui]"
    echo "도구 확인 + validator Ego Network 생성. --gui는 tkinter도 필수로 검사합니다."
    echo "처음 설치: docs/planning/SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md"
    exit 0 ;;
  "") ;;
  *) echo "Usage: bash install.sh [--gui]" >&2; exit 2 ;;
esac
if (($#)); then echo "Usage: bash install.sh [--gui]" >&2; exit 2; fi
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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
NETWORK_PROFILE="$(/usr/bin/python3 "$PACKAGE_ROOT/scripts/generate_network_profile.py")"
echo "MORAI GUI에서 로드할 Ego Network: $NETWORK_PROFILE"
echo "기본 도구 확인 완료. Google Drive 자동 업로드를 사용하려면 rclone 설치와 remote 로그인이 추가로 필요합니다."
