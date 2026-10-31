#!/usr/bin/env bash
# 빌린 PC의 안전 인계된 새 출력. 무인자는 상태만 확인한다.
set -euo pipefail
BASE="$HOME/ajounice_base5_capture/work/unified-runtime-v1"
if [ "$#" -eq 0 ]; then set -- status; fi
exec /usr/bin/python3 -B -u "$BASE/recover_unified_borrowed.py" "$@"
