#!/usr/bin/env bash
set -euo pipefail

root=/home/jihun/Autoware2ros1
work="$root/porting/work/autoware-behavior-path-external-request-lane-change-module-v1-p3-001/source-closure-001"
lane="$root/porting/work/autoware-behavior-path-lane-change-module-v1-p3-001/configure-probe-r001"
common="$root/porting/work/autoware-behavior-path-planner-common-v1-p2-001/source-closure-001/install-isolated-export-r007"
port="${AW149_EXTERNAL_REQUEST_LC_PORT:-11559}"
source /opt/ros/noetic/setup.bash
cmake --build "$work/build-r001" --target test_external_request_lane_change_plugins -j2
export ROS_MASTER_URI="http://127.0.0.1:${port}"
export ROS_HOME="/tmp/aw149-external-request-lc-${port}-home"
export ROS_LOG_DIR="/tmp/aw149-external-request-lc-${port}-log"
export CMAKE_PREFIX_PATH="$work/install-r001:$lane/install-isolated-export-r001:$common:${CMAKE_PREFIX_PATH:-}"
export ROS_PACKAGE_PATH="$work/install-r001/share:$lane/install-isolated-export-r001/share:$common/share:$root/porting/work/autoware-behavior-path-planner-v1-p2-001/source-closure-001/src:${ROS_PACKAGE_PATH:-}"
export LD_LIBRARY_PATH="$work/install-r001/lib:$work/build-r001/devel/lib:$lane/install-isolated-export-r001/lib:$common/lib:$root/porting/work/autoware-frenet-planner-v1-p2/attempt-006/candidate/install_isolated/lib:$root/porting/work/autoware-sampler-common-v1-p2/validation/p2-sealed-005-source-closure/candidate-workspace/install_isolated/lib:$root/porting/work/autoware-object-recognition-utils-v1/candidate-r016/install-r016/lib:$root/porting/work/autoware-traffic-light-utils-v1-p2-001/source-closure-001/install_isolated/lib:$root/porting/work/autoware-freespace-planning-algorithms-v1-p2-r013/catkin_ws/install_isolated/lib:$root/porting/work/autoware-motion-utils-v1-p2-001/source-closure/install/lib:$root/porting/work/autoware-universe-utils-v1-p2-001/source-closure-facade-002/install/lib:$root/porting/work/autoware-lanelet2-utils-v1-p2-001/source-closure-001/install/lib:$root/porting/work/autoware-route-handler-v1-p2-001/source-closure-001/install/lib:$root/porting/work/autoware-rtc-interface-v1-p2-010/install/lib:$root/porting/work/autoware-objects-of-interest-marker-interface-v1-p2-001/attempt-010/install/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
roscore -p "$port" >"/tmp/aw149-external-request-lc-${port}.log" 2>&1 &
master_pid=$!
cleanup(){ kill "$master_pid" 2>/dev/null || true; wait "$master_pid" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
for _ in $(seq 1 40); do rosparam list >/dev/null 2>&1 && break; sleep 0.1; done
rosparam list >/dev/null
ldd -r "$work/install-r001/lib/libautoware_behavior_path_external_request_lane_change_module.so" | tee "/tmp/aw149-external-request-lc-${port}-ldd.log"
if rg -q 'not found|undefined symbol' "/tmp/aw149-external-request-lc-${port}-ldd.log"; then exit 1; fi
rospack profile >/dev/null
rospack plugins --attrib=plugin autoware_behavior_path_planner | grep -F "$work/install-r001/share/autoware_behavior_path_external_request_lane_change_module/plugins.xml"
"$work/build-r001/devel/lib/autoware_behavior_path_external_request_lane_change_module/test_external_request_lane_change_plugins" --gtest_color=no
rosparam get /rosdistro >/dev/null
test "$(rosnode list)" = "/rosout"
echo P3_R002_EXTERNAL_REQUEST_LANE_CHANGE_SCOPED_MASTER_PASS
