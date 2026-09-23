#!/usr/bin/env bash

set -Eeuo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/common.sh"

usage() {
  echo "usage: $0 [--mr-root /path/to/MoraiLauncher_Lin]" >&2
}

mr_root=""
if [[ $# -gt 0 ]]; then
  if [[ $# -ne 2 || "$1" != "--mr-root" ]]; then
    usage
    exit 2
  fi
  mr_root="$(realpath "$2")"
fi

find_scenario_roots() {
  find "$HOME" -maxdepth 5 -type d \
    -path '*/MoraiLauncher_Lin_Data/SaveFile/Scenario' \
    -print 2>/dev/null | sort -u
}

if [[ -n "$mr_root" ]]; then
  scenario_root="$mr_root/MoraiLauncher_Lin_Data/SaveFile/Scenario"
  if [[ ! -d "$scenario_root" ]]; then
    echo "ERROR: not an MR root (Scenario directory missing): $mr_root" >&2
    exit 2
  fi
else
  mapfile -t candidates < <(find_scenario_roots)
  if [[ ${#candidates[@]} -ne 1 ]]; then
    echo "ERROR: expected exactly one MR Scenario directory, found ${#candidates[@]}." >&2
    printf '  %s\n' "${candidates[@]:-<none>}" >&2
    echo "Retry with: $0 --mr-root /path/to/MoraiLauncher_Lin" >&2
    exit 2
  fi
  scenario_root="${candidates[0]}"
fi

map_dir="$scenario_root/R_KR_PR_K-city_2025"
if [[ ! -d "$map_dir" ]]; then
  echo "ERROR: K-City scenario directory is missing: $map_dir" >&2
  exit 2
fi

count=0
for source in "$scenario_source_dir"/*.json; do
  install -m 0644 "$source" "$map_dir/$(basename "$source")"
  ((count += 1))
done

echo "Installed $count human-evaluation scenarios into:"
echo "  $map_dir"
echo "In MR Scenario Load, search for: ajounice_eval_"
