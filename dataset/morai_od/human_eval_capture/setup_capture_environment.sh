#!/usr/bin/env bash

# Build only the ROS messages and UDP acquisition gateway.  This script never
# builds planning, localization, BEVFusion, acados, or install_aw149.

set -Eeuo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/common.sh"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ERROR: ROS Noetic is not installed under /opt/ros/noetic." >&2
  exit 2
fi
if ! command -v catkin >/dev/null 2>&1; then
  echo "ERROR: catkin-tools is missing." >&2
  echo "Install once: sudo apt update && sudo apt install python3-catkin-tools" >&2
  exit 2
fi

echo "Initializing only the MORAI message submodule..."
git -C "$AJOUNICE_ROOT" submodule update --init base_ws/src/morai/morai_msgs

source /opt/ros/noetic/setup.bash
for package in velodyne_pointcloud nodelet; do
  if ! rospack find "$package" >/dev/null 2>&1; then
    echo "ERROR: ROS package '$package' is missing." >&2
    echo "Install once: sudo apt install ros-noetic-velodyne-pointcloud ros-noetic-nodelet" >&2
    exit 2
  fi
done

cd "$AJOUNICE_ROOT/base_ws"
catkin config \
  --extend /opt/ros/noetic \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin build morai_msgs sensor_interface --no-status

source "$AJOUNICE_ROOT/base_ws/devel/setup.bash"
rospack find morai_msgs
rospack find sensor_interface
rosmsg show morai_msgs/EgoVehicleStatus >/dev/null
rosmsg show morai_msgs/ObjectStatusList >/dev/null

echo
echo "PASS: minimal raw-capture environment is ready."
echo "Generated overlay: $AJOUNICE_ROOT/base_ws/devel/setup.bash"
echo "install_aw149 was not created or used."
