#!/usr/bin/env bash

set -Eeuo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/common.sh"
source_capture_workspace

sensor_config="$AJOUNICE_ROOT/configs/sensors/Sensor_competition_local.json"
network_config="$AJOUNICE_ROOT/configs/network/ego_network_competition_local.json"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
rosparam set /use_sim_time false

echo "Starting competition-local UDP acquisition without ROS control output."
echo "Sensor config : $sensor_config"
echo "Ego config    : $network_config"
echo "In MR, load/connect these same two JSON files before recording."

exec roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:="$sensor_config" \
  network_config:="$network_config" \
  enable_sensors:=true \
  enable_lidar:=true \
  enable_ego_network:=true \
  enable_ego_vehicle_status:=true \
  enable_control:=false
