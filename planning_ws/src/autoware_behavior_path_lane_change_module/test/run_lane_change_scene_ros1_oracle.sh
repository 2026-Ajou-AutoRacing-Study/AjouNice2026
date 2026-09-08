#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash

readonly src_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly work_root="$(cd "${src_root}/.." && pwd)"
readonly build_root="${work_root}/configure-probe-r001"
readonly port="${AW149_LANE_CHANGE_SCENE_PORT:-11510}"
readonly test_utils="/home/jihun/Autoware2ros1/porting/work/autoware-test-utils-v1-p2-001/candidate-install-r001"
readonly route_root="/home/jihun/Autoware2ros1/porting/work/autoware-route-handler-v1-p2-001/source-closure-001"

export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_PACKAGE_PATH="${src_root}:${test_utils}/share:${route_root}/src:${route_root}/install/share:${ROS_PACKAGE_PATH:-}"
export LD_LIBRARY_PATH="${build_root}/devel/lib:${build_root}/gtest/lib:${test_utils}/lib:/home/jihun/Autoware2ros1/porting/work/autoware-behavior-path-planner-common-v1-p2-001/source-closure-001/install-host-plugin-r025/lib:/home/jihun/Autoware2ros1/porting/work/autoware-frenet-planner-v1-p2/attempt-006/candidate/install_isolated/lib:/home/jihun/Autoware2ros1/porting/work/autoware-lanelet2-extension-v1-p2-001/source-closure-001/install/lib:/home/jihun/Autoware2ros1/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install/lib:/home/jihun/Autoware2ros1/porting/work/autoware-motion-utils-v1-p2-001/source-closure/install/lib:/home/jihun/Autoware2ros1/porting/work/autoware-universe-utils-v1-p2-001/source-closure-facade-002/install/lib:/home/jihun/Autoware2ros1/porting/work/autoware-planning-factor-interface-v1-p2-001/source-closure-003-ros1-abi/install/lib:/home/jihun/Autoware2ros1/porting/work/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated/lib:/home/jihun/Autoware2ros1/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install/lib:/home/jihun/Autoware2ros1/porting/work/autoware-rtc-interface-v1-p2-010/install/lib:/home/jihun/Autoware2ros1/porting/work/autoware-sampler-common-v1-p2/validation/p2-sealed-005-source-closure/candidate-workspace/install_isolated/lib:/home/jihun/Autoware2ros1/porting/work/autoware-object-recognition-utils-v1/candidate-r016/install-r016/lib:/home/jihun/Autoware2ros1/porting/work/autoware-objects-of-interest-marker-interface-v1-p2-001/attempt-010/install/lib:/home/jihun/Autoware2ros1/porting/work/autoware-traffic-light-utils-v1-p2-001/source-closure-001/install_isolated/lib:/home/jihun/Autoware2ros1/porting/work/autoware-deprecated-boundary-departure-checker-v1-p2-001/source-closure-full-001/catkin_ws/install/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

roscore -p "${port}" >"/tmp/aw149-lane-change-scene-${port}.log" 2>&1 &
master_pid=$!
cleanup() {
  kill "${master_pid}" 2>/dev/null || true
  wait "${master_pid}" 2>/dev/null || true
}
trap cleanup EXIT

for _ in $(seq 1 40); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.25
done

python3 "${src_root}/scripts/load_ros1_parameters.py" /test_lane_change_scene \
  --source "${test_utils}/share/autoware_test_utils/config/test_common.param.yaml" \
  --source "${test_utils}/share/autoware_test_utils/config/test_nearest_search.param.yaml" \
  --source "${test_utils}/share/autoware_test_utils/config/test_vehicle_info.param.yaml" \
  --source "/home/jihun/Autoware2ros1/autoware_original/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/config/behavior_path_planner.param.yaml" \
  --source "${src_root}/config/lane_change.param.yaml"
cmake --build "${build_root}" --target test_lane_change_scene_ros1_oracle -j2
"${build_root}/devel/lib/autoware_behavior_path_lane_change_module/test_lane_change_scene_ros1_oracle" "$@"
