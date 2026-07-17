#!/usr/bin/env bash
set -euo pipefail

ROOT="${AJOUNICE_ROOT:-$HOME/AjouNice2026}"

source /opt/ros/noetic/setup.bash
if [[ -f "$ROOT/base_ws/devel/setup.bash" ]]; then
  source "$ROOT/base_ws/devel/setup.bash"
fi

kill_by_pattern() {
  local signal="$1"
  shift
  local pattern
  for pattern in "$@"; do
    pkill "-$signal" -f "$pattern" 2>/dev/null || true
  done
}

if rosnode list >/dev/null 2>&1; then
  nodes=$(rosnode list | grep -E '^/(ego_tf_broadcaster|tf_map_to_morai_map|lidar|lidar1|morai_sensor_static_tf|morai_sensor_udp_receiver|morai_ego_udp_receiver|morai_ctrl_cmd_udp_sender|ioniq5_robot_state_publisher|rviz_ioniq5|lanelet2_osm_marker_publisher|mgeo_json_marker_publisher)$' || true)
  if [[ -n "$nodes" ]]; then
    rosnode kill $nodes >/dev/null 2>&1 || true
  fi
fi

kill_by_pattern TERM \
  '[/]opt/ros/noetic/bin/roslaunch' \
  '[/]opt/ros/noetic/bin/rosmaster' \
  '[/]opt/ros/noetic/lib/rosout/rosout' \
  '[/]opt/ros/noetic/lib/nodelet/nodelet' \
  '[/]opt/ros/noetic/lib/rviz/rviz' \
  '[/]opt/ros/noetic/lib/tf2_ros/static_transform_publisher' \
  "$ROOT/base_ws/devel/lib/tf_pub/ego_tf_broadcaster" \
  "$ROOT/base_ws/devel/lib/sensor_interface/lidar_distortion_corrector" \
  "$ROOT/base_ws/devel/lib/sensor_interface/morai_sensor_udp_receiver" \
  "$ROOT/base_ws/devel/lib/sensor_interface/morai_ego_udp_receiver" \
  "$ROOT/base_ws/devel/lib/sensor_interface/morai_ctrl_cmd_udp_sender" \
  "$ROOT/base_ws/devel/lib/ioniq5/morai_sensor_static_tf.py" \
  "$ROOT/base_ws/devel/lib/lanelet_map_visualizer/"

sleep 1

kill_by_pattern KILL \
  "$ROOT/base_ws/devel/lib/sensor_interface/morai_sensor_udp_receiver" \
  "$ROOT/base_ws/devel/lib/sensor_interface/morai_ego_udp_receiver" \
  "$ROOT/base_ws/devel/lib/sensor_interface/morai_ctrl_cmd_udp_sender"

echo "remaining ROS/UDP listeners:"
ss -ltnp | grep -E ':(11311)' || true
ss -lunp | grep -E ':(2368|9291|9293|9295|9297|9301|9303|9311|9313|9314|9317|9319)' || true
