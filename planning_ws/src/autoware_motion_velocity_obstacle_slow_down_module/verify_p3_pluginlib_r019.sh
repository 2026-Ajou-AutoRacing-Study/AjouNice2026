#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/jihun/Autoware2ros1
MODULE="$ROOT/porting/work/autoware-motion-velocity-obstacle-slow-down-module-v1-p2-001/candidate-r019-build/install"
COMMON="$ROOT/porting/work/autoware-motion-velocity-planner-common-v1-p2-001/source-closure-001/install"
MVP="$ROOT/porting/work/autoware-motion-velocity-planner-v1-p2-001/source-closure-001/install"
MOTION="$ROOT/porting/work/autoware-motion-utils-v1-p2-001/source-closure/install"
OBJECTS="$ROOT/porting/work/autoware-object-recognition-utils-v1/candidate-r012/install-r001"
VEHICLE="$ROOT/porting/work/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated"
UNIVERSE="$ROOT/porting/work/autoware-universe-utils-v1-p2-001/source-closure-facade-002/install"
PLANNING_FACTOR="$ROOT/porting/work/autoware-planning-factor-interface-v1-p2-001/source-closure-003-ros1-abi/install"
MARKER="$ROOT/porting/work/autoware-objects-of-interest-marker-interface-v1-p2-001/attempt-010/install"
SIGNAL="$ROOT/porting/work/autoware-signal-processing-v1-actual-render-r003/source-closure-001/install"
ROUTE="$ROOT/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install"

source /opt/ros/noetic/setup.bash
prefixes=("$MODULE" "$MVP" "$COMMON" "$MOTION" "$OBJECTS" "$VEHICLE" "$UNIVERSE" "$PLANNING_FACTOR" "$MARKER" "$SIGNAL" "$ROUTE")
for prefix in "${prefixes[@]}"; do test -d "$prefix"; done
export ROS_PACKAGE_PATH="$(printf '%s/share:' "${prefixes[@]}")${ROS_PACKAGE_PATH:-}"
export CMAKE_PREFIX_PATH="$(IFS=:; echo "${prefixes[*]}")${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export ROS_HOME="$MODULE/.ros"
mkdir -p "$ROS_HOME"
export LD_LIBRARY_PATH="$MODULE/lib:$MVP/lib:$COMMON/lib:$MOTION/lib:$OBJECTS/lib:$VEHICLE/lib:$UNIVERSE/lib:$PLANNING_FACTOR/lib:$MARKER/lib:$SIGNAL/lib:$ROUTE/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
rospack profile >/dev/null
rospack plugins --attrib=plugin autoware_motion_velocity_planner | grep -Fqx "autoware_motion_velocity_obstacle_slow_down_module $MODULE/share/autoware_motion_velocity_obstacle_slow_down_module/plugins.xml"
"$ROOT/porting/work/autoware-motion-velocity-obstacle-slow-down-module-v1-p2-001/candidate-r019-build/devel/lib/autoware_motion_velocity_obstacle_slow_down_module/test_obstacle_slow_down_plugin" --gtest_color=no
echo P3_R019_OBSTACLE_SLOW_DOWN_PLUGINLIB_PASS
