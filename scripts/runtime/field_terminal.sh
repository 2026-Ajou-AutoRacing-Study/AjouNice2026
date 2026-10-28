#!/usr/bin/env bash
# Display-only wrapper. Alias arguments and all ROS/control policies are unchanged.
set -uo pipefail

export AJOUNICE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$AJOUNICE_ROOT/docs/common_alias.txt"

if [[ $# -eq 0 ]]; then
  echo 'field udp_lan|sensing_lan|localization|bevfusion|tracking|traffic [args]' >&2
  echo 'rc/planning/planviz/control: original output' >&2
  exit 2
fi

entry="$1"
shift
case "$entry" in
  rc|planning|planviz|traj_fol_a|traj_fol_global) component= ;;
  udp|udp_lan) component="$entry" ;;
  sensing|sensing_lan) component=sensing ;;
  localization|localization_mrp) component=localization ;;
  bevfusion|tracking|traffic) component="$entry" ;;
  *) echo "unsupported command: $entry" >&2; exit 2 ;;
esac

if [[ -z "$component" ]]; then
  # Other teams own these consoles: use the exact original alias and output.
  exec /bin/bash --noprofile --norc -O expand_aliases -c '
    source "$AJOUNICE_ROOT/docs/common_alias.txt"
    entry="$1"
    shift
    eval "$entry \"\$@\""
  ' field-original "$entry" "$@"
fi
prefix="$(field_log_prefix 2>/dev/null)" || {
  field_log_new >/dev/null || exit
  prefix="$(field_log_prefix)" || exit
}

# Expand only a fixed, validated alias name, never user-provided shell text.
# "$@" remains positional argv even with spaces, quotes or shell metacharacters.
# run pre-opens the raw log before this child shell can start any ROS process.
display_args=()
if [[ -n "${AJOUNICE_FIELD_LOG_WIDTH:-}" ]]; then
  display_args=(--width "$AJOUNICE_FIELD_LOG_WIDTH")
fi
if [[ "${AJOUNICE_FIELD_TOPIC_STATUS:-1}" == 0 ]]; then
  display_args+=(--no-topic-status)
fi
exec /usr/bin/python3 "$AJOUNICE_ROOT/scripts/runtime/field_console.py" run \
  --component "$component" --log-dir "${prefix}_console" \
  --interval "${AJOUNICE_FIELD_LOG_INTERVAL:-1}" \
  --stop-timeout "${AJOUNICE_FIELD_STOP_TIMEOUT:-20}" \
  "${display_args[@]}" -- \
  /bin/bash --noprofile --norc -O expand_aliases -c '
    source "$AJOUNICE_ROOT/docs/common_alias.txt"
    entry="$1"
    shift
    eval "$entry \"\$@\""
  ' field-alias "$entry" "$@"
