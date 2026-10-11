# bevfusion_ros

## Base5 별도 시험 경로 — 2026-10-12

`base5_trial.launch`는 E25 production을 교체하지 않는다. Sensor_4, 원본 JPEG 전달,
학습된 `512×1056 / resize 0.825` 입력 계약, 별도 서버 `29515`와
`/perception/base5_trial/detection/objects`를 선택한다. host는 원본 0~255 XYZI와
JPEG를 정규화·재압축하지 않는다. Base5 서버의
`--base5-runtime --point-intensity-source ros255`가 강도를 정확히 한 번 ÷255 처리한다.
`--fp32-reference`는 입력·출력 대조용이지 최적화된 실시간 설정이 아니다.
서버와 bridge의 강도·크기 계약이 다르면 요청을 거부한다. 기존 E25 launch와 기본값은
유지한다. 시험 경로의 threshold는 출발값이며 Base5 production 승인값이 아니다.
status·latency·marker·projection도 `/perception/base5_trial/`로 분리한다.
[현재 계획 허브](../../../../docs/bevfusion/261009_BASE5_INITIALIZATION_AND_RUNTIME_OPTIMIZATION_PLAN.md#base5-current-plan-hub)와
[EVIDENCE 35절](../../../../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-ros-input-continuity-261012)에서 검증 범위·결과를 확인한다.

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
at IoU `0.50`. This remains active for temporal experiments so their detector
output keeps the same duplicate-suppression contract as production. Disable it
only for a controlled A/B. The cumulative removal count is exposed as
`nms_suppressed_total` in `/perception/bevfusion/status`.

`zero_shot.launch` remains only as a compatibility alias. Runtime timing and
drop counters are published in `/perception/bevfusion/status`; total sensor to
result latency is published in `/perception/bevfusion/latency_ms`.

See `docs/bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md` for the production
deployment procedure. The older zero-shot documents are retained only as
experiment history and use the untrained nuScenes initialization.
