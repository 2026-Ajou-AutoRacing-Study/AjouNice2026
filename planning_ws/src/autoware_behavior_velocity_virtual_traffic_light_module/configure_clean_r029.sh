#!/usr/bin/env bash
set -euo pipefail

root=/home/jihun/Autoware2ros1
work="$root/porting/work/autoware-behavior-velocity-virtual-traffic-light-module-v1-p2-001/candidate-r028-full-lifecycle"
build="$work/build-clean-r029"
install="$work/install-clean-r029"
prefixes=(
  "$root/porting/work/tier4-v2x-msgs-v1-p3-clean-r001/install-clean"
  "$root/porting/work/autoware-behavior-velocity-planner-common-v1-p2-001/source-closure-001/install"
  "$root/porting/work/autoware-trajectory-v1-p2-001/source-closure-003/install"
  "$root/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install"
  "$root/porting/work/autoware-motion-utils-v1-p2-001/source-closure/install"
  "$root/porting/work/autoware-planning-factor-interface-v1-p2-001/source-closure-003-ros1-abi/install"
  "$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install"
  "$root/porting/work/autoware-velocity-smoother-v1-p2-001/ros1-runtime-candidate/catkin_ws_r002/install"
  "$root/porting/work/autoware-osqp-interface-v1-p2-001/source-closure/install"
  "$root/porting/work/autoware-qp-interface-v1-p2-001/source-closure/install"
  "$root/porting/work/osqp-v0.6.3-causal-execution-v17-20260831-001/install"
  "$root/porting/work/autoware-objects-of-interest-marker-interface-v1-p2-001/attempt-010/install"
  "$root/porting/work/autoware-universe-utils-v1-p2-001/source-closure-facade-002/install"
  "$root/porting/work/autoware-lanelet2-extension-v1-p2-001/source-closure-001/install"
  "$root/porting/work/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated"
  "$root/porting/work/autoware-component-interface-specs-v1-p2-retry006/catkin_ws/devel"
  /opt/ros/noetic
)
for p in "${prefixes[@]}"; do test -d "$p"; done
cmake_prefix=$(IFS=';'; printf '%s' "${prefixes[*]}")
export CMAKE_PREFIX_PATH=$(IFS=:; printf '%s' "${prefixes[*]}")
unset ROS_PACKAGE_PATH
cmake -S "$work" -B "$build" -DCMAKE_PREFIX_PATH="$cmake_prefix" -DCMAKE_INSTALL_PREFIX="$install" \
  -DBVPC_COMMON_INCLUDE_DIR="${prefixes[1]}/include" -DBVPC_COMMON_LIBRARY="${prefixes[1]}/lib/libautoware_behavior_velocity_planner_common.so" \
  -DTRAJECTORY_INCLUDE_DIR="${prefixes[2]}/include" -DTRAJECTORY_LIBRARY="${prefixes[2]}/lib/libautoware_trajectory.so" \
  -DROUTE_HANDLER_INCLUDE_DIR="${prefixes[3]}/include" -DROUTE_HANDLER_LIBRARY="${prefixes[3]}/lib/libautoware_route_handler.so" \
  -DMOTION_UTILS_INCLUDE_DIR="${prefixes[4]}/include" -DMOTION_UTILS_LIBRARY="${prefixes[4]}/lib/libautoware_motion_utils.so" -DMOTION_UTILS_GEOMETRY_LIBRARY="${prefixes[4]}/lib/libautoware_utils_geometry.so" \
  -DPLANNING_FACTOR_INTERFACE_LIBRARY="${prefixes[5]}/lib/libautoware_planning_factor_interface.so" -DLANELET_UTILS_LIBRARY="${prefixes[6]}/lib/libautoware_lanelet2_utils.so" \
  -DVELOCITY_SMOOTHER_LIBRARY="${prefixes[7]}/lib/libautoware_velocity_smoother.so" -DOSQP_INTERFACE_LIBRARY="${prefixes[8]}/lib/libautoware_osqp_interface.so" \
  -DQP_INTERFACE_LIBRARY="${prefixes[9]}/lib/libautoware_qp_interface.so" -DOSQP_LIBRARY="${prefixes[10]}/lib/libosqp.so" \
  -DOBJECTS_INCLUDE_DIR="${prefixes[11]}/include" -DOBJECTS_LIBRARY="${prefixes[11]}/lib/libautoware_objects_of_interest_marker_interface.so" \
  -DUTILS_INCLUDE_DIR="${prefixes[12]}/include" -DUTILS_DEBUG_LIBRARY="${prefixes[12]}/lib/libautoware_utils_debug.so" \
  -DLANELET_EXTENSION_INCLUDE_DIR="${prefixes[13]}/include" -DLANELET_EXTENSION_LIBRARY="${prefixes[13]}/lib/libautoware_lanelet2_extension_lib.so" \
  -DVEHICLE_INFO_INCLUDE_DIR="${prefixes[14]}/include" -DVEHICLE_INFO_LIBRARY="${prefixes[14]}/lib/libvehicle_info_utils.so" \
  -DMAP_MSGS_INCLUDE_DIR="${prefixes[15]}/include"
