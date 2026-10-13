# AjouNice BEVFusion runtime

현재 production은 E24이며 선택·입력·가중치·최적화의 단일 기준은
[모델 CURRENT](../../docs/bevfusion/CURRENT.md)다. 실행은
[E24 handoff](../../docs/bevfusion/BASE5_BEVFUSION_PRODUCTION_HANDOFF.md)의
`scripts/runtime/run_bevfusion.sh`를 따른다. 이 디렉터리는 Base3 호환 환경과 Base5
학습·추론 서버를 함께 관리한다. 파일명에 Base3가 있어도 현재 실행값이라고 가정하지 않는다.
팀 CUDA 12.8 이미지의 하드웨어 지원과 새 호스트 qualification은 구분한다.

The ROS host connects to a port published only on `127.0.0.1`. The simulator
boundary remains the competition-provided UDP interface. E25 is an explicit
[rollback profile](../../docs/bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md), not
the current default. The older `260720_*zero_shot*` documents are history only.

Team training and artifact installation are documented in
`docs/bevfusion/TEAM_DOCKER_TRAINING_HANDOFF.md`.

```bash
bash docker/bevfusion/scripts/build_team_image.sh
bash docker/bevfusion/scripts/verify_team_image.sh
```

```bash
# Create the local Compose environment file once.
cd ~/AjouNice2026/docker/bevfusion
cp .env.example .env

# Start the model server.
docker compose up -d inference
docker compose logs -f inference

# Check protocol and model readiness.
PYTHONPATH=. python3 scripts/healthcheck.py
```

`.env.example` expects the verified inference-only B3-BF0-F40 E25 checkpoint at
`$HOME/bevfusion_artifacts/checkpoints/base3_bf0_f40_e25_inference.pth`. A local
`.env` is optional and ignored by Git. For a portable command that does not
depend on `.env`, use the production handoff document.

Set `PROJECTION_ONLY=true` in `.env` to validate calibration without loading
the model checkpoint.

## Online execution contract

아래 RGB8·비용 표는 초기 Base3 경로의 재현 기록이다. 현재 Base5 호스트는 원본 JPEG와
0~255 XYZI를 전달하고 서버가 JPEG decode/resize와 고정 강도 ÷255를 수행한다.
현재 512×1056·5cm/20cm와 기존 표의 작은 모델을 혼용하지 않는다. 실제 흐름은
[ROS 패키지](../../base_ws/src/perception/bevfusion_ros/README.md)를 따른다.
서버 raw box는 `lidar_bottom_v1`로 선언하며 원본 tensor·GT·가중치를 바꾸지 않는다.
ROS의 `box_center_output_origin:=gravity_center_v1`은 명시적인 시험용 어댑터다.
기본 `legacy_passthrough`를 유지하고, 선언 없는 이전 서버는 시험 모드에서 거부한다.
[Z 계약·reset·TF 검증 경계](../../docs/bevfusion/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#box-origin)를 먼저 확인한다.

- ROS chooses the closest four camera messages for each LiDAR timestamp.
- The ROS host decodes and resizes the four images concurrently with OpenCV.
- It sends four contiguous RGB8 tensors and one contiguous Nx4 XYZI tensor.
- The server constructs the MMDetection3D batch in memory; no temporary files
  are written or reread.
- FP16 autocast is enabled by default. Host preprocessing and server inference
  are separate latest-only stages, so they overlap without an unbounded queue.

The online OpenCV tensor was compared against the Base3 offline test transform:
image pixels, points and augmentation matrices match exactly (floating-point
matrix inverse differences were below `1e-6`).

## RTX 5060 Ti feasibility result

The initial 20-sample compatible-checkpoint benchmark was:

| Segment | Mean | p95 | Interpretation |
|---|---:|---:|---|
| Host four-camera preprocessing | 19.1 ms | 22.1 ms | Runs concurrently with inference |
| Docker tensor preprocessing | 2.6 ms | 3.3 ms | No filesystem round trip |
| FP16 model inference | 61.4 ms | 64.3 ms | Preloaded samples |
| Persistent TCP server round trip | 72.3 ms | 76.5 ms | 13.8 Hz server-side capacity |

The staged steady-state capacity is governed by the slower stage, currently
the 72.3 ms server round trip, rather than by adding 19.1 ms serially. This is
enough to consume a nominal 12 Hz LiDAR stream with modest margin. Individual
sensor-to-result latency remains roughly 90 ms plus ROS scheduling jitter.
The earlier B3-BF0 E10 rosbag runtime measured roughly
`78–84 ms` p50/p95 end-to-end latency and `12.5–12.8 Hz` processing capacity.
RTX 4090 measurements must still be repeated on the target PC. Monitor
`dropped_raw` and `dropped_prepared` in the status topic.

With the server running, the real-data TCP benchmark is reproducible with:

```bash
cd ~/AjouNice2026/docker/bevfusion
PYTHONPATH=. python3 scripts/benchmark_base3_tcp.py \
  --info ~/AjouNice2026/artifacts/datasets/morai_base3_bevfusion_v1/morai_base3_infos_val.pkl \
  --sensor-config ~/AjouNice2026/configs/sensors/Sensor.json \
  --samples 50
```
