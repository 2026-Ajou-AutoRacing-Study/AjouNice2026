# bevfusion_ros

Host-side ROS1 bridge for the Dockerized Base3 four-camera BEVFusion runtime.

- Sensor topics are parsed from `configs/sensors/Sensor.json`.
- LiDAR timestamps select the nearest image from every camera buffer.
- Camera decode/resize/crop runs in a four-thread host stage.
- CPU preprocessing and GPU inference overlap. Each stage retains only its
  latest complete frame, so latency cannot grow into a FIFO backlog.
- PointCloud2 conversion is vectorized and the runtime contract is exactly
  `float32 [x, y, z, intensity]`.
- The Docker connection is restricted to `127.0.0.1` TCP.
- Output is published both as RViz-only `MarkerArray` and the canonical
  `autoware_perception_msgs/DetectedObjects` detector contract. The latter is
  consumed by the common rosbag evaluator and downstream tracking pipeline.

Run calibration projection first:

```bash
# Publish LiDAR points over the four camera images.
roslaunch bevfusion_ros projection_check.launch
```

Then run Base3 detection:

```bash
# Publish BEVFusion 3D boxes and labels for RViz.
roslaunch bevfusion_ros base3_online.launch
```

`zero_shot.launch` remains only as a compatibility alias. Runtime timing and
drop counters are published in `/perception/bevfusion/status`; total sensor to
result latency is published in `/perception/bevfusion/latency_ms`.

See `docs/bevfusion/260720_morai_four_camera_zero_shot_short.md` for the full
terminal-by-terminal MORAI procedure.
