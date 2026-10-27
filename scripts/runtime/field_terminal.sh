#!/usr/bin/env bash
# Display-only wrapper. Alias arguments and all ROS/control policies are unchanged.
set -uo pipefail

export AJOUNICE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$AJOUNICE_ROOT/docs/common_alias.txt"

if [[ $# -eq 0 ]]; then
  echo '사용법: field rc | udp_lan | sensing_lan | localization | bevfusion | tracking | traffic | planning | planviz | traj_fol_a [기존 인자...]' >&2
  echo '각 명령은 별도 터미널에서 실행. 제어(traj_fol_a)는 안전 확인 후 마지막에 직접 실행.' >&2
  exit 2
fi

entry="$1"
shift
case "$entry" in
  rc) component=roscore ;;
  udp|udp_lan) component="$entry" ;;
  sensing|sensing_lan) component=sensing ;;
  localization|localization_mrp) component=localization ;;
  bevfusion|tracking|traffic|planning|planviz) component="$entry" ;;
  traj_fol_a|traj_fol_global) component=control ;;
  *) echo "현장 요약에서 지원하지 않는 명령: $entry (기존 alias로 직접 실행 가능)" >&2; exit 2 ;;
esac

if [[ "$entry" == rc ]]; then
  field_log_new || exit
fi
prefix="$(field_log_prefix)" || exit

# Expand only a fixed, validated alias name, never user-provided shell text.
# "$@" remains positional argv even with spaces, quotes or shell metacharacters.
# run pre-opens the raw log before this child shell can start any ROS process.
exec /usr/bin/python3 "$AJOUNICE_ROOT/scripts/runtime/field_console.py" run \
  --component "$component" --log-dir "${prefix}_console" \
  --interval "${AJOUNICE_FIELD_LOG_INTERVAL:-1}" \
  --stop-timeout "${AJOUNICE_FIELD_STOP_TIMEOUT:-20}" \
  --width "${AJOUNICE_FIELD_LOG_WIDTH:-120}" -- \
  /bin/bash --noprofile --norc -O expand_aliases -c '
    source "$AJOUNICE_ROOT/docs/common_alias.txt"
    entry="$1"
    shift
    eval "$entry \"\$@\""
  ' field-alias "$entry" "$@"
