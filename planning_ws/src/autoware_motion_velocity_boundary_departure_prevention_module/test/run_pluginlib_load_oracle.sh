#!/usr/bin/env bash
set -euo pipefail

root=/home/jihun/Autoware2ros1
candidate="$root/porting/work/autoware_motion_velocity_boundary_departure_prevention_module/candidate-r002-transport-lifecycle"
install="${AW149_BDP_INSTALL_OVERRIDE:-$root/porting/work/autoware_motion_velocity_boundary_departure_prevention_module/candidate-r002-transport-lifecycle-install}"
oracle_build="${AW149_BDP_ORACLE_BUILD_OVERRIDE:-$root/porting/work/autoware_motion_velocity_boundary_departure_prevention_module/candidate-r002-transport-lifecycle-install-build}"
mvp_prefix="${AW149_MVP_PREFIX_OVERRIDE:-$root/porting/work/autoware-motion-velocity-planner-v1-p2-001/source-closure-001/install}"
smoother_prefix="${AW149_SMOOTHER_PREFIX_OVERRIDE:-$root/porting/work/autoware-velocity-smoother-v1-p2-001/ros1-runtime-candidate/catkin_ws_r002/install}"
common_prefix="${AW149_MVP_COMMON_PREFIX_OVERRIDE:-$root/porting/work/autoware-motion-velocity-planner-common-v1-p2-001/source-closure-001/install}"
planning_msgs_prefix="${AW149_PLANNING_MSG_PREFIX_OVERRIDE:-/home/jihun/AjouNice2026/base_ws/devel}"
export ROS_HOME=/tmp/bdp-r002-roshome
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11488}"
export CMAKE_PREFIX_PATH="$planning_msgs_prefix:$install:$root/porting/work/osqp-v0.6.3-causal-execution-v17-20260831-001/install:$root/porting/work/proxsuite-v0.6.5-provider-001/install:$mvp_prefix:$common_prefix:$smoother_prefix:$root/porting/work/autoware-object-recognition-utils-v1/candidate-r012/install-r001:$root/porting/work/autoware-osqp-interface-v1-p2-001/source-closure/install:$root/porting/work/autoware-qp-interface-v1-p2-001/source-closure/install:$root/porting/work/autoware-deprecated-boundary-departure-checker-v1-p2-001/source-closure-full-001/catkin_ws/install:$root/porting/work/autoware-motion-utils-v1-p2-001/source-closure/install:$root/porting/work/autoware-universe-utils-v1-p2-001/source-closure-facade-002/install:$root/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install:$root/porting/work/autoware-trajectory-v1-p2-001/source-closure-003/install:$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install:$root/porting/work/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated:/opt/ros/noetic"
export ROS_PACKAGE_PATH="$planning_msgs_prefix/share:$install/share:$mvp_prefix/share:$common_prefix/share:$smoother_prefix/share:$root/porting/work/autoware-object-recognition-utils-v1/candidate-r012/install-r001/share:$root/porting/work/autoware-osqp-interface-v1-p2-001/source-closure/install/share:$root/porting/work/autoware-qp-interface-v1-p2-001/source-closure/install/share:$root/porting/work/autoware-deprecated-boundary-departure-checker-v1-p2-001/source-closure-full-001/catkin_ws/install/share:$root/porting/work/autoware-motion-utils-v1-p2-001/source-closure/install/share:$root/porting/work/autoware-universe-utils-v1-p2-001/source-closure-facade-002/install/share:$root/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install/share:$root/porting/work/autoware-trajectory-v1-p2-001/source-closure-003/install/share:$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install/share:$root/porting/work/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated/share:/opt/ros/noetic/share"
export LD_LIBRARY_PATH="$install/lib:$smoother_prefix/lib:$root/porting/work/autoware-object-recognition-utils-v1/candidate-r012/install-r001/lib:$root/porting/work/autoware-osqp-interface-v1-p2-001/source-closure/install/lib:$root/porting/work/autoware-qp-interface-v1-p2-001/source-closure/install/lib:$root/porting/work/osqp-v0.6.3-causal-execution-v17-20260831-001/install/lib:$root/porting/work/proxsuite-v0.6.5-provider-001/install/lib:$root/porting/work/autoware-deprecated-boundary-departure-checker-v1-p2-001/source-closure-full-001/catkin_ws/install/lib:$root/porting/work/autoware-motion-utils-v1-p2-001/source-closure/install/lib:$root/porting/work/autoware-universe-utils-v1-p2-001/source-closure-facade-002/install/lib:$mvp_prefix/lib:$common_prefix/lib:$root/porting/work/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated/lib:$root/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install/lib:$root/porting/work/autoware-trajectory-v1-p2-001/source-closure-003/install/lib:$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install/lib:$root/porting/work/autoware-utils-diagnostics-v2-cxx17export/catkin_ws/install/lib:$root/porting/work/autoware-vehicle-info-utils-v1-20260830/retry006/catkin_ws/install_isolated/lib:$root/porting/work/tier4-adapi-rviz-plugin-v1-p2-001/retry014/ws/install_isolated/lib:$planning_msgs_prefix/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu"

if ldd -r "$install/lib/libautoware_motion_velocity_boundary_departure_prevention_module.so" | grep -E 'undefined symbol|not found'; then
  echo BDP_DYNAMIC_PROVIDER_CLOSURE_FAILED >&2
  exit 10
fi
echo BDP_DYNAMIC_PROVIDER_CLOSURE_PASS

rospack plugins --attrib=plugin autoware_motion_velocity_planner | grep -F "$install/share/autoware_motion_velocity_boundary_departure_prevention_module/plugins.xml"
if ! rosparam list >/dev/null 2>&1; then
  echo "BDP_R003_NO_FOREGROUND_ROSCORE uri=$ROS_MASTER_URI" >&2
  exit 2
fi
"$oracle_build/devel/lib/autoware_motion_velocity_boundary_departure_prevention_module/bdp_pluginlib_load_oracle"
"$oracle_build/devel/lib/autoware_motion_velocity_boundary_departure_prevention_module/bdp_input_gate_oracle"
"$oracle_build/devel/lib/autoware_motion_velocity_boundary_departure_prevention_module/bdp_planning_fixture_oracle"
