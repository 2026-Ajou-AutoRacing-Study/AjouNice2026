#!/usr/bin/env bash
# Only selects existing profiles and disables forbidden receive channels.
# Does not source aliases, change NICs, load a scene, or control the vehicle itself.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
profile=local
print_only=false
args=()
while (($#)); do
  case "$1" in
    --profile)
      [[ $# -ge 2 ]] || { echo 'Missing --profile value' >&2; exit 2; }
      profile="$2"; shift 2 ;;
    profile:=*) profile="${1#profile:=}"; shift ;;
    --print-command) print_only=true; shift ;;
    -h|--help)
      echo 'run_competition_udp.sh --profile local|lan|teamlead [roslaunch args]'
      echo 'Ego and Object receive channels are always disabled. --print-command launches nothing.'
      exit 0 ;;
    enable_ego_vehicle_status:=*|enable_object_info:=*)
      case "${1#*:=}" in
        false|False|0) ;;
        *) echo "Competition entry requires ${1%%:=*}:=false; use explicit development entry for GT" >&2; exit 2 ;;
      esac
      shift ;;
    *) args+=("$1"); shift ;;
  esac
done
case "$profile" in
  local|lan)
    sensor="$ROOT/configs/sensors/Sensor_competition_${profile}.json"
    network="$ROOT/configs/network/ego_network_competition_${profile}.json" ;;
  teamlead)
    sensor="$ROOT/configs/sensors/Sensor_competition_lan_jihun.json"
    network="$ROOT/configs/network/ego_network_competition_lan_jihun.json" ;;
  *) echo "Unknown UDP profile: $profile (local|lan|teamlead)" >&2; exit 2 ;;
esac
command=(roslaunch sensor_interface morai_udp_interface.launch
  "sensor_config:=$sensor" "network_config:=$network"
  enable_ego_vehicle_status:=false enable_object_info:=false "${args[@]}")
if $print_only; then
  printf '%q ' "${command[@]}"; printf '\n'
  exit 0
fi
printf 'Competition UDP profile=%s; Ego=OFF Object=OFF\n' "$profile" >&2
# bash function fallback also allows isolated tests to intercept roslaunch.
if [[ "$(type -t roslaunch || true)" == function ]]; then
  roslaunch "${command[@]:1}"
else
  exec "${command[@]}"
fi
