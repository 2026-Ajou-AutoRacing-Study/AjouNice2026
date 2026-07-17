#!/usr/bin/env bash
set -u

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"
source /opt/ros/noetic/setup.bash
if [[ -f "$ROOT/base_ws/devel/setup.bash" ]]; then
  source "$ROOT/base_ws/devel/setup.bash"
fi

section() { printf '\n== %s ==\n' "$1"; }

section "listeners"
ss -ltnp | grep -E ':(11311)' || true
ss -lunp | grep -E ':(2368|9291|9293|9295|9297|9301|9303|9311|9313|9314|9317|9319)' || true

section "ros nodes"
rosnode list 2>/dev/null | grep -E 'morai_.*udp|ego_tf|tf_map|lidar|morai_sensor_static_tf|robot_state|rviz|lanelet' || true

section "critical topics"
for topic in \
  /Ego_topic /Competition_topic /Object_topic /CollisionData /ctrl_cmd \
  /sensor/gps /sensor/imu /sensor/lidar1 /sensor/lidar1/deskewed \
  /sensor/camera/front/compressed /sensor/camera/front_left/compressed \
  /sensor/camera/front_right/compressed /sensor/camera/back/compressed \
  /tf /tf_static; do
  printf '\n-- %s --\n' "$topic"
  rostopic info "$topic" 2>/dev/null || true
done

section "processes"
ps -ef | grep -E 'roscore|rosmaster|rosout|roslaunch|morai_.*udp|rviz|nodelet|lidar_distortion|ego_tf|static_transform|morai_sensor_static_tf|Simulator.x86_64' || true
