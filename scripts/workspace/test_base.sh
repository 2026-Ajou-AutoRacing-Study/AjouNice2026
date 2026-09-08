#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export AJOUNICE_ROOT="$REPO_ROOT"
timeout_s="${AW149_PACKAGE_TEST_TIMEOUT_S:-600}"

if (( $# != 0 )); then
  echo "Usage: AW149_PACKAGE_TEST_TIMEOUT_S=600 $0" >&2
  exit 2
fi
if ! [[ "$timeout_s" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: AW149_PACKAGE_TEST_TIMEOUT_S must be a positive integer." >&2
  exit 2
fi

source "$REPO_ROOT/scripts/workspace/source_aw149.sh"
mapfile -t packages < <(catkin_topological_order --only-names "$REPO_ROOT/base_ws/src")
if (( ${#packages[@]} != 24 )); then
  echo "ERROR: expected 24 base packages, found ${#packages[@]}." >&2
  printf '  %s\n' "${packages[@]}" >&2
  exit 1
fi

RUN_DIR="$(mktemp -d /tmp/aw149-base-tests.XXXXXX)"
for package in "${packages[@]}"; do
  build_dir="$REPO_ROOT/base_ws/build_aw149/$package"
  if [[ ! -d "$build_dir" ]]; then
    echo "ERROR: missing build directory for $package; run setup_ws.sh first." >&2
    exit 1
  fi
  printf 'TEST_START %s\n' "$package"
  if timeout "$timeout_s" make -C "$build_dir" run_tests -j2 >"$RUN_DIR/$package.log" 2>&1; then
    printf 'TEST_TARGET_PASS %s\n' "$package"
  else
    result=$?
    printf 'TEST_TARGET_FAIL %s rc=%s log=%s\n' \
      "$package" "$result" "$RUN_DIR/$package.log" >&2
    tail -80 "$RUN_DIR/$package.log" >&2
    exit "$result"
  fi
done

catkin_test_results "$REPO_ROOT/base_ws/build_aw149" --verbose
printf 'AW149_BASE_TEST_TARGETS_PASS=%d/%d\n' "${#packages[@]}" "${#packages[@]}"
printf 'Logs: %s\n' "$RUN_DIR"
