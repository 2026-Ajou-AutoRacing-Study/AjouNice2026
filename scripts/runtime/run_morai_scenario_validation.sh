#!/usr/bin/env bash
set -euo pipefail

# Host-side entry point. MORAI and this workspace normally run on native ROS1
# Noetic. Docker is retained as an explicit fallback for a separately prepared
# ROS1 image.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
REPO_ROOT="${AJOUNICE_REPO_ROOT:-$DEFAULT_REPO_ROOT}"
SCENARIO_DIR="${MORAI_SCENARIO_DIR:-$HOME/Desktop/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025}"
MODE="${AJOUNICE_ROS1_MODE:-auto}"
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
if [[ "$MODE" == "auto" ]]; then
  if [[ -f /opt/ros/noetic/setup.bash && -f "$REPO_ROOT/base_ws/devel/setup.bash" ]]; then
    MODE="native"
  else
    MODE="docker"
  fi
fi

if [[ "$MODE" == "native" ]]; then
  if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
    echo "Native ROS Noetic setup not found: /opt/ros/noetic/setup.bash" >&2
    exit 1
  fi
  if [[ ! -f "$REPO_ROOT/base_ws/devel/setup.bash" ]]; then
    echo "Workspace setup not found: $REPO_ROOT/base_ws/devel/setup.bash" >&2
    echo "Build it with scripts/workspace/build_all.sh first." >&2
    exit 1
  fi
  source /opt/ros/noetic/setup.bash
  source "$REPO_ROOT/base_ws/devel/setup.bash"
  cd "$REPO_ROOT"
  exec python3 scripts/runtime/morai_scenario_validation.py \
    --scenario-dir "$SCENARIO_DIR" \
    "$@"
fi

if [[ "$MODE" != "docker" ]]; then
  echo "Unknown AJOUNICE_ROS1_MODE: $MODE (use native, docker, or auto)" >&2
  exit 2
fi
if [[ ! -f "$PROFILE_FILE" ]]; then
  echo "ROS1 container profile not found: $PROFILE_FILE" >&2
  exit 1
fi
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "ROS1 image not found: $IMAGE" >&2
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
