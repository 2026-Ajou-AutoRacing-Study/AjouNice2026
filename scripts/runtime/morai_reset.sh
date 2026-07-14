#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-stack}"
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

cleanup_master() {
  if rosnode list >/dev/null 2>&1; then
    yes y | rosnode cleanup >/dev/null 2>&1 || true
  fi
}

kill_bridge() {
  rosnode kill /rosbridge_websocket /rosapi >/dev/null 2>&1 || true
  kill_by_pattern TERM \
    '[r]oslaunch .*bringup morai_bridge.launch' \
    '[r]oslaunch rosbridge_server rosbridge_websocket.launch' \
    '[/]rosbridge_server/rosbridge_websocket' \
    '[/]rosapi/rosapi_node'
  sleep 1
  kill_by_pattern KILL \
    '[r]oslaunch .*bringup morai_bridge.launch' \
    '[r]oslaunch rosbridge_server rosbridge_websocket.launch' \
    '[/]rosbridge_server/rosbridge_websocket' \
    '[/]rosapi/rosapi_node'
}

kill_stack() {
  if rosnode list >/dev/null 2>&1; then
    rosnode kill $(rosnode list | rg '^/(ego_tf_broadcaster|tf_map_to_morai_map|rosbridge_websocket|rosapi|lidar|lidar1|lidar2|morai_sensor_static_tf|morai_camera_udp_receiver|ioniq5_robot_state_publisher|rviz_ioniq5|lanelet2_osm_marker_publisher|mgeo_json_marker_publisher)$') >/dev/null 2>&1 || true
  fi

  kill_by_pattern TERM \
    '[/]opt/ros/noetic/bin/roslaunch' \
    '[/]opt/ros/noetic/bin/rosmaster' \
    '[/]opt/ros/noetic/lib/rosout/rosout' \
    '[/]opt/ros/noetic/lib/rosbridge_server/rosbridge_websocket' \
    '[/]opt/ros/noetic/lib/rosapi/rosapi_node' \
    '[/]opt/ros/noetic/lib/nodelet/nodelet' \
    '[/]opt/ros/noetic/lib/rviz/rviz' \
    '[/]opt/ros/noetic/lib/tf2_ros/static_transform_publisher' \
    "$ROOT/base_ws/devel/lib/tf_pub/ego_tf_broadcaster" \
    "$ROOT/base_ws/devel/lib/sensor_interface/lidar_distortion_corrector" \
    "$ROOT/base_ws/devel/lib/sensor_interface/lidar_advanced_concat_filter" \
    "$ROOT/base_ws/devel/lib/sensor_interface/morai_camera_udp_receiver" \
    "$ROOT/base_ws/devel/lib/ioniq5/morai_sensor_static_tf.py" \
    "$ROOT/base_ws/devel/lib/lanelet_map_visualizer/"
  sleep 2
  kill_by_pattern KILL \
    '[/]opt/ros/noetic/bin/roslaunch' \
    '[/]opt/ros/noetic/bin/rosmaster' \
    '[/]opt/ros/noetic/lib/rosout/rosout' \
    '[/]opt/ros/noetic/lib/rosbridge_server/rosbridge_websocket' \
    '[/]opt/ros/noetic/lib/rosapi/rosapi_node' \
    '[/]opt/ros/noetic/lib/nodelet/nodelet' \
    '[/]opt/ros/noetic/lib/rviz/rviz' \
    '[/]opt/ros/noetic/lib/tf2_ros/static_transform_publisher' \
    "$ROOT/base_ws/devel/lib/tf_pub/ego_tf_broadcaster" \
    "$ROOT/base_ws/devel/lib/sensor_interface/lidar_distortion_corrector" \
    "$ROOT/base_ws/devel/lib/sensor_interface/lidar_advanced_concat_filter" \
    "$ROOT/base_ws/devel/lib/sensor_interface/morai_camera_udp_receiver" \
    "$ROOT/base_ws/devel/lib/ioniq5/morai_sensor_static_tf.py" \
    "$ROOT/base_ws/devel/lib/lanelet_map_visualizer/"
}

case "$MODE" in
  bridge)
    kill_bridge
    cleanup_master
    ;;
  stack)
    kill_stack
    ;;
  *)
    echo "usage: $0 [stack|bridge]" >&2
    exit 2
    ;;
esac

echo "remaining listeners:"
ss -ltnp | rg ':(11311|9090)' || true
ss -lunp | rg ':(2368|2370)' || true
