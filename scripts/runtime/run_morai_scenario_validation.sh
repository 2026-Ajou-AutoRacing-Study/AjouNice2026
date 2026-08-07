#!/usr/bin/env bash
set -euo pipefail

# Host-side entry point.  MORAI itself runs on the host; the ROS1 validation
# stack runs in the project Noetic image with host networking.
REPO_ROOT="${AJOUNICE_REPO_ROOT:-$HOME/AjouNice2026}"
SCENARIO_DIR="${MORAI_SCENARIO_DIR:-$HOME/다운로드/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025}"
IMAGE="${AJOUNICE_ROS1_IMAGE:-ajounice2026-ros1:noetic}"
PROFILE_FILE="${AJOUNICE_ROS1_PROFILE:-$HOME/.local/share/ajounice2026/ros1_noetic/ajou_ros1.bash}"

if [[ ! -d "$REPO_ROOT" ]]; then
  echo "AjouNice2026 repository not found: $REPO_ROOT" >&2
  exit 1
fi
if [[ ! -d "$SCENARIO_DIR" ]]; then
  echo "MORAI scenario directory not found: $SCENARIO_DIR" >&2
  exit 1
fi
if [[ ! -f "$PROFILE_FILE" ]]; then
  echo "ROS1 container profile not found: $PROFILE_FILE" >&2
  exit 1
fi
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "ROS1 image not found: $IMAGE" >&2
  echo "Build it with ros1_noetic_build_image.sh first." >&2
  exit 1
fi

tty_args=(-i)
if [[ -t 0 && -t 1 ]]; then
  tty_args=(-it)
fi

exec docker run --rm "${tty_args[@]}" \
  --init \
  --network host \
  --ipc host \
  -e AJOUNICE_ROOT=/home/ajou/AjouNice2026 \
  -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}" \
  -v "$REPO_ROOT:/home/ajou/AjouNice2026:rw" \
  -v "$SCENARIO_DIR:/morai_scenarios:ro" \
  -v "$PROFILE_FILE:/etc/profile.d/ajou_ros1.bash:ro" \
  -v /etc/localtime:/etc/localtime:ro \
  -w /home/ajou/AjouNice2026 \
  "$IMAGE" \
  python3 scripts/runtime/morai_scenario_validation.py \
    --scenario-dir /morai_scenarios \
    "$@"
