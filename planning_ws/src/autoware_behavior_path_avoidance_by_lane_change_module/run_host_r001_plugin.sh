#!/usr/bin/env bash
set -euo pipefail

candidate_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_root="$(cd "${candidate_root}/../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "${candidate_root}/build-host-r001/catkin_generated/setup_cached.sh"

export ROS_PACKAGE_PATH="${candidate_root}/install-host-r001/share:${workspace_root}/autoware_behavior_path_static_obstacle_avoidance_module/source-closure-001/install-host-r023-tests/share:${workspace_root}/autoware-behavior-path-lane-change-module-v1-p3-001/configure-probe-r001/install-isolated-export-r001/share:${workspace_root}/autoware-behavior-path-planner-v1-p2-001/source-closure-001/src:${ROS_PACKAGE_PATH}"
export CMAKE_PREFIX_PATH="${candidate_root}/install-host-r001:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${candidate_root}/build-host-r001/gtest/lib:${candidate_root}/install-host-r001/lib:${workspace_root}/autoware_behavior_path_static_obstacle_avoidance_module/source-closure-001/install-host-r023-tests/lib:${workspace_root}/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install/lib:${workspace_root}/autoware-lanelet2-extension-v1-p2-001/source-closure-001/install/lib:${workspace_root}/autoware-route-handler-v1-p2-001/source-closure-001/install/lib:${workspace_root}/autoware-vehicle-info-utils-v1-20260830/retry006/catkin_ws/install_isolated/lib:${workspace_root}/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated/lib:${workspace_root}/autoware-deprecated-boundary-departure-checker-v1-p2-001/source-closure-full-001/catkin_ws/install/lib:${workspace_root}/autoware-traffic-light-utils-v1-p2-001/source-closure-001/install_isolated/lib:${LD_LIBRARY_PATH}"

rospack profile >/dev/null
rospack plugins --attrib=plugin autoware_behavior_path_planner
"${candidate_root}/build-host-r001/devel/lib/autoware_behavior_path_avoidance_by_lane_change_module/test_avoidance_by_lane_change_plugin_discovery"
rostest "${candidate_root}/install-host-r001/share/autoware_behavior_path_avoidance_by_lane_change_module/avoidance_by_lane_change_manager_init.test"
