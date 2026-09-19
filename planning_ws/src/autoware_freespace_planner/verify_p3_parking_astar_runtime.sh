#!/usr/bin/env bash
set -euo pipefail

# Candidate-local, foreground A* parking ingress gate. It does not cover
# obstacle replanning, RRT*, TF goal transforms, or full freespace policy.
ROOT=/home/jihun/Autoware2ros1
WORK="$ROOT/porting/work/autoware-freespace-planner-v1-p2-001/candidate"
PROVIDER="$ROOT/porting/work/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated"
PORT="${AW149_FREESPACE_P3_PORT:-11495}"

source /opt/ros/noetic/setup.bash
cmake --build "$WORK/build-r004" --target test_freespace_planner_ros1_node -j2
cmake --install "$WORK/build-r004"
export LD_LIBRARY_PATH="$WORK/install-r004/lib:$WORK/build-r004/devel/lib:$PROVIDER/lib:/home/jihun/AjouNice2026/base_ws/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
test_binary="$WORK/build-r004/devel/lib/autoware_freespace_planner/test_freespace_planner_ros1_node"
if ldd "$test_binary" | grep -q 'not found'; then
  echo 'P3_FREESPACE_PARKING_ASTAR_LINKER_CLOSURE_FAIL' >&2
  ldd "$test_binary" | grep 'not found' >&2
  exit 1
fi

export ROS_MASTER_URI="http://127.0.0.1:${PORT}"
master_pid=''
cleanup() {
  if [[ -n "$master_pid" ]]; then
    kill "$master_pid" 2>/dev/null || true
    wait "$master_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

rosmaster --core -p "$PORT" -w 3 >/tmp/aw149-freespace-p3-rosmaster.log 2>&1 &
master_pid=$!
for _ in $(seq 1 60); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.1
done
rosparam list >/dev/null
"$test_binary" --gtest_color=no
echo 'P3_FREESPACE_PARKING_ASTAR_RUNTIME_PASS'
