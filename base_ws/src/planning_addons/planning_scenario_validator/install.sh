#!/usr/bin/env bash
# Ubuntu 20.04/22.04 팀 PC에서 녹화·대시보드 실행 도구를 확인한다.
set -euo pipefail
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
if ((${#MISSING[@]})); then
  echo "필요한 도구가 없습니다: ${MISSING[*]}" >&2
  echo "Ubuntu: sudo apt install ffmpeg x11-utils python3-yaml python3-gi gir1.2-wnck-3.0" >&2
  echo "ROS Noetic와 AjouNice AW149 overlay는 scripts/workspace/setup_ws.sh로 준비하세요." >&2
  exit 1
fi
if ! test -f "$PACKAGE_ROOT/../../../install_aw149/setup.bash"; then
  echo "주의: AW149 overlay가 없습니다. 저장소 루트에서 scripts/workspace/setup_ws.sh를 실행하세요." >&2
fi
NETWORK_PROFILE="$(/usr/bin/python3 "$PACKAGE_ROOT/scripts/generate_network_profile.py")"
echo "MORAI GUI에서 로드할 Ego Network: $NETWORK_PROFILE"
echo "기본 도구 확인 완료. Google Drive 자동 업로드를 사용하려면 rclone 설치와 remote 로그인이 추가로 필요합니다."
