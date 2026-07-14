#!/usr/bin/env bash
set -u

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"

source /opt/ros/noetic/setup.bash
if [[ -f "$ROOT/base_ws/devel/setup.bash" ]]; then
  source "$ROOT/base_ws/devel/setup.bash"
fi

section() {
  printf '\n== %s ==\n' "$1"
}

section "listeners"
ss -ltnp | grep -E ':(11311|9090)' || true
ss -lunp | grep -E ':(2368|2370)' || true

section "ros params"
rosparam get /use_sim_time 2>/dev/null || true

section "ros nodes"
rosnode list 2>/dev/null | grep -E 'rosbridge|rosapi|ego_tf|tf_map|lidar|morai_sensor_static_tf|morai_camera|robot_state|rviz|lanelet' || true

section "bridge clients"
rostopic echo -n 1 /connected_clients 2>/dev/null || true

section "critical topics"
for topic in \
  /Ego_topic \
  /sensor/imu \
  /sensor/lidar1 \
  /sensor/lidar2 \
  /sensor/lidar1/deskewed \
  /sensor/lidar2/deskewed \
  /sensor/lidar_concat/deskewed \
  /tf \
  /tf_static; do
  printf '\n-- %s --\n' "$topic"
  rostopic info "$topic" 2>/dev/null || true
done

section "processes"
ps -ef | grep -E 'roscore|rosmaster|rosout|roslaunch|rosbridge_websocket|rosapi_node|rviz|nodelet|lidar_distortion|lidar_advanced|ego_tf|static_transform|morai_sensor_static_tf|Simulator.x86_64' || true
