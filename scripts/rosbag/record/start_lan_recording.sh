#!/usr/bin/env bash
# One command on the algorithm PC; never starts/restarts the driving stack.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ "${1:-}" == --help ]]; then
  echo '사용: bash scripts/rosbag/record/start_lan_recording.sh [녹화기 옵션]'
  echo '평소처럼 풀스택을 실행한 뒤 사용. Ctrl+C는 녹화만 마감합니다.'
  echo 'Windows 보조 계측도 start_morai_recording.ps1 하나로 실행합니다.'
  exit 0
fi
if [[ "${1:-}" == --list ]]; then
  exec /usr/bin/python3 "$ROOT/scripts/rosbag/record/record_development_bag.py" --list
fi
set +u
source "$ROOT/scripts/workspace/source_aw149.sh"
set -u
exec /usr/bin/python3 "$ROOT/scripts/rosbag/record/record_development_bag.py" --session-id lan_debug "$@"
