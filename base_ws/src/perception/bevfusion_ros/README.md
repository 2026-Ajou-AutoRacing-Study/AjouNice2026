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
# Publish BEVFusion 3D boxes and labels. Use planviz in a separate terminal.
roslaunch bevfusion_ros base3_online.launch
```

On the deployed AjouNice workstation, the `bevfusion` alias starts the Docker
inference server, waits for checkpoint readiness, launches this ROS bridge,
and stops the server when the bridge exits. It refuses to start while another
detector already publishes the canonical detection topic.

The default production operating point is class-aware: Vehicle `0.40`,
Pedestrian `0.15`, and Obstacle `0.10`. The bridge asks the Docker model for
all boxes down to `0.10` and applies the class thresholds before publishing the
canonical object topic. Override the corresponding `*_score_threshold` launch
arguments only for a controlled evaluation.

After class thresholds, production also applies class-aware rotated BEV NMS
at IoU `0.50`. This removes a lower-confidence box only when it substantially
overlaps a higher-confidence box of the same class; nearby distinct objects
and different classes are preserved. Set `rotated_bev_nms_enabled:=false`
only for a controlled A/B. The cumulative removal count is exposed as
`nms_suppressed_total` in `/perception/bevfusion/status`.

`zero_shot.launch` remains only as a compatibility alias. Runtime timing and
drop counters are published in `/perception/bevfusion/status`; total sensor to
result latency is published in `/perception/bevfusion/latency_ms`.

See `docs/bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md` for the production
deployment procedure. The older zero-shot documents are retained only as
experiment history and use the untrained nuScenes initialization.
