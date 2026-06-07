#!/usr/bin/env bash
echo "record start: $(date '+%Y-%m-%d %H:%M:%S')"

rosbag record \
  /Ego_topic \
  /Object_topic \
  # /ctrl_cmd \
  /lidar3D \
  /image_jpeg/compressed \
  /imu \
  /gps \
  /tf \
  /clock \
  -O morai_local_valid_ego_sensors.bag \
  --duration=10 &

REC_PID=$!

for i in $(seq 0 10); do
  echo "recording... ${i}s / 10s"
  sleep 1
done

wait $REC_PID

echo "record done: $(date '+%Y-%m-%d %H:%M:%S')"
rosbag info morai_local_valid_ego_sensors.bag
