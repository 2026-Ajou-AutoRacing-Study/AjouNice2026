# MORAI PointPainting object pipeline live runbook

The sensor interface, static TF and localization terminals are identical to
`morai_fastbev_object_pipeline_live.md`. Select the PointPainting detector only
in terminal 5.

## Terminal 1 — ROS master

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
roscore
```

## Terminal 2 — MORAI Sensor/Ego UDP

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
rosparam set /use_sim_time false
roslaunch sensor_interface morai_udp_interface.launch \
  sensor_config:="$HOME/AjouNice2026/configs/sensors/Sensor_6.json" \
  network_config:="$HOME/AjouNice2026/configs/network/ego_network_1.json" \
  enable_control:=false \
  lidar_rpm:=900.0
```

## Terminal 3 — Sensor static TF

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
roslaunch bringup sensing.launch \
  sensor_config:="$HOME/AjouNice2026/configs/sensors/Sensor_6.json" \
  start_rviz:=false \
  enable_vehicle_model:=false \
  enable_ego_tf:=false
```

## Terminal 4 — MORAI GT localization and Lanelet2 v60

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
roslaunch bringup localization_morai_gt.launch \
  use_sim_time:=false \
  start_lanelet_map:=true \
  map_file:="$HOME/AjouNice2026/kcity_map/lanelet_map/lanelet2_map_v60.osm" \
  projector_file:="$HOME/AjouNice2026/kcity_map/lanelet_map/map_projector_info_v1.yaml"
```

## Terminal 5 — PointPainting, tracking and Map-CV prediction

```bash
cd ~/AjouNice2026
POINTPAINTING_WITH_TP=true \
POINTPAINTING_PREDICTION_MODEL=map_based \
CONDA_ENV=openpcdet_rtx50 \
base_ws/src/morai_pointpainting_ros/scripts/run_pointpainting.sh live
```

## Terminal 6 — Shared object-pipeline RViz

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
roslaunch perception_object_pipeline object_pipeline_visualization.launch
```

## Terminal 7 — Input/output rates

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
rostopic hz --wall-time \
  /sensor/lidar1 \
  /perception/object_recognition/detection/objects \
  /perception/object_recognition/tracking/objects \
  /perception/object_recognition/objects
```

## Terminal 8 — Verify the detector contract

```bash
cd ~/AjouNice2026
source /opt/ros/noetic/setup.bash
source base_ws/devel/setup.bash
rostopic type /perception/object_recognition/detection/objects
rostopic echo -n 1 /perception/object_recognition/detection/objects
```
