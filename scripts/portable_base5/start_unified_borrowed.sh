#!/usr/bin/env bash
# 빌린 PC의 승인된 작업 폴더에서만 실행한다. 기본은 읽기 전용 계획이다.
set -euo pipefail
BASE="$HOME/ajounice_base5_capture/work/unified-runtime-v1"
if [ "$#" -eq 0 ]; then set -- plan; fi
exec /usr/bin/python3 -B -u "$BASE/checks/unified_borrowed.py" "$@"
