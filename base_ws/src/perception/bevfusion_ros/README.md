# bevfusion_ros

Host-side ROS1 bridge for the Dockerized four-camera BEVFusion runtime.

- Sensor topics are parsed from `configs/sensors/Sensor.json`.
- LiDAR timestamps select the nearest image from every camera buffer.
- Only the latest complete frame is retained while inference is busy.
- The Docker connection is restricted to `127.0.0.1` TCP.
- Output `MarkerArray` is an RViz debugging contract, not the final Planning
  perception message.

Run calibration projection first:

```bash
# Publish LiDAR points over the four camera images.
roslaunch bevfusion_ros projection_check.launch
```

Then run zero-shot detection:

```bash
# Publish BEVFusion 3D boxes and labels for RViz.
roslaunch bevfusion_ros zero_shot.launch
```

See `docs/bevfusion/260720_morai_four_camera_zero_shot_short.md` for the full
terminal-by-terminal MORAI procedure.

