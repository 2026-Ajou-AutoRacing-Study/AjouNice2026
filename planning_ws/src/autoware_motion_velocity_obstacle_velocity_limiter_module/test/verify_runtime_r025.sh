#!/usr/bin/env bash
set -euo pipefail

root=/home/jihun/Autoware2ros1
candidate="$root/porting/work/autoware-motion-velocity-obstacle-velocity-limiter-module-v1-p2-001"
export ROS_HOME=/tmp/aw149-r025-ros-home
export ROS_LOG_DIR=/tmp/aw149-r025-ros-log
export ROS_MASTER_URI=http://127.0.0.1:11525
export CMAKE_PREFIX_PATH="$candidate/candidate-r025-install:$root/porting/work/autoware-motion-velocity-planner-v1-p2-001/source-closure-001/install:$root/porting/work/autoware-motion-velocity-planner-common-v1-p2-001/source-closure-001/install:$root/porting/work/autoware-motion-utils-v1-p2-001/source-closure/install:$root/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install:$root/porting/work/autoware-vehicle-info-utils-v1-20260830/retry006/catkin_ws/install_isolated:$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install:/opt/ros/noetic"
export ROS_PACKAGE_PATH="$candidate/candidate-r025-install/share:$root/porting/work/autoware-motion-velocity-planner-v1-p2-001/source-closure-001/install/share:$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install/share:/opt/ros/noetic/share"
export LD_LIBRARY_PATH="$candidate/candidate-r025-install/lib:$root/porting/work/autoware-motion-velocity-planner-common-v1-p2-001/source-closure-001/install/lib:$root/porting/work/autoware-motion-utils-v1-p2-001/source-closure/install/lib:$root/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install/lib:$root/porting/work/autoware-vehicle-info-utils-v1-20260830/retry006/catkin_ws/install_isolated/lib:$root/porting/work/autoware-universe-utils-v1-p2-001/source-closure-facade-002/install/lib:$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:/opt/ros/noetic/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cleanup() { [[ -n "${master_pid:-}" ]] && kill "$master_pid" 2>/dev/null || true; }
trap cleanup EXIT
roscore -p 11525 >/tmp/aw149-r025-roscore.log 2>&1 &
master_pid=$!
for _ in $(seq 1 40); do rosparam list >/dev/null 2>&1 && break; sleep 0.1; done
rosparam list >/dev/null
rosparam load "$candidate/candidate-r025/config/runtime_r025_ros1.yaml" /r025_positive
"$candidate/candidate-r025-build/devel/lib/autoware_motion_velocity_obstacle_velocity_limiter_module/verify_obstacle_velocity_limiter_init" /r025_positive
rosparam load "$candidate/candidate-r025/config/runtime_r025_ros1.yaml" /r025_negative
rosparam set /r025_negative/simulation/nb_points invalid_string
if "$candidate/candidate-r025-build/devel/lib/autoware_motion_velocity_obstacle_velocity_limiter_module/verify_obstacle_velocity_limiter_init" /r025_negative; then
  echo 'negative typed-parameter gate unexpectedly accepted invalid_string' >&2
  exit 1
fi
echo P3_R025_OBSTACLE_VELOCITY_LIMITER_TYPED_PARAMETER_REJECTION_PASS
