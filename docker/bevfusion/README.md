# AjouNice BEVFusion runtime

현재 production은 전체 30,750장 E12 + camera TRT + head SDPA·좌표 cache이며 선택·입력·가중치·최적화의 단일 기준은
[모델 CURRENT](../../docs/bevfusion/CURRENT.md)다. 실행은
[production handoff](../../docs/bevfusion/BASE5_BEVFUSION_PRODUCTION_HANDOFF.md)의
`scripts/runtime/run_bevfusion.sh`를 따른다. 이 디렉터리는 Base3 호환 환경과 Base5
학습·추론 서버를 함께 관리한다. 파일명에 Base3가 있어도 현재 실행값이라고 가정하지 않는다.
팀 CUDA 12.8 이미지의 하드웨어 지원과 새 호스트 qualification은 구분한다.

The ROS host connects to a port published only on `127.0.0.1`. The simulator
boundary remains the competition-provided UDP interface. E25 is an explicit
[rollback profile](../../docs/bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md), not
the current default. The older `260720_*zero_shot*` documents are history only.

Team training and artifact installation are documented in
`docs/bevfusion/TEAM_DOCKER_TRAINING_HANDOFF.md`.

## Vast 오프라인 학습 이미지 — 2026-10-18

대상 private 저장소는 `fluffyluv/ajounice2026-bevfusion-train`이다.
태그는 `base5-cu128-261018-v1`이며 기존 production 서버 이미지를 교체하지 않는다.
업로드를 완료했으며 고정 registry 참조는 다음과 같다.

```text
fluffyluv/ajounice2026-bevfusion-train@sha256:85185232c0a53fb2640c411b04002fa3a98631408d3cdb5f668d5c15a40fd959
```

- 로컬에서 확인한 팀 base image ID는
  `sha256:13488ccac149f41b070e618db5db91bfa3bf2d3d1a6de3dac80aa2e2c948ec46`이다.
- 기존 CUDA 확장을 재사용하고 현재 fork의 추적 소스를 overlay한다. W&B는 원격 정상
  환경과 같은 `0.22.3`으로 고정한다. 설치된 모든 패키지가 원격과 완전히 같다는 뜻은 아니다.
- 빌드 context는 명시적으로 허용한 추적 소스만 포함한다. 데이터·가중치·Git 이력·
  Docker 로그인·Drive/W&B/SSH 인증정보는 포함하지 않는다.
- 이미지 안의 `/opt/ajounice/source_inventory.json`에 1,792개 source SHA를 기록한다.
- 다음 학습의 compile OFF·3클래스·512×1056 config 및 CUDA 확장 CPU import를 확인했다.
  GPU forward/backward·실제 checkpoint load·저장/재개·온라인 백업 인증은 별도로 수행한다.
- 재시작형 코사인은 아직 연결하지 않았다. 이 이미지를 선택해도 학습은 자동 시작하지 않는다.

기존에 검증한 base image가 있는 로컬에서 아래로 준비한다. context는 새 경로여야 한다.

```bash
python3 docker/bevfusion/scripts/build_vast_training_image.py \
  --context /tmp/ajounice_base5_vast_training_NEW --build
```

위 명령은 push하지 않는다. 이미지 검증 후 `docker push`를 별도로 실행한다.
처음부터 base를 빌드하려면 기존 `build_team_image.sh`를 사용하고 ID가 달라지면
새 환경을 검증한 뒤 새 base pin·태그로 준비한다. 기존 pin 검사를 우회하지 않는다.

Vast의 새 template에서 Image Path:Tag에
`fluffyluv/ajounice2026-bevfusion-train:base5-cu128-261018-v1`을 넣고 SSH launch mode를 선택한다.
Private repository 인증은 Docker Hub 사용자 `fluffyluv`와 읽기 권한 token을 UI의
Repository Authentication에 직접 입력한다. token을 Git·이미지·채팅에 넣지 않는다.
기존 Vast 인스턴스의 이미지가 자동으로 바뀌는 것은 아니다.

SSH 접속 후 `/opt/ajounice`에서 아래를 먼저 확인한다. Vast 내부에서 Docker를 중첩 실행하지 않는다.

```bash
python /opt/ajounice/verify_vast_training_image.py
python perception/detection3d/bevfusion/tools/run_base5_direct_training.py --help
```

데이터와 초기화 파일은 `/workspace` 아래 별도로 준비한다. 기존 완료 run의 실행 명령으로
새 데이터를 강제 resume하지 않으며 새 계약·work-dir를 만든 뒤 학습을 승인한다.

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
[Z 계약·reset·TF 검증 경계](../../docs/bevfusion/evidence/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#box-origin)를 먼저 확인한다.

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
  --sensor-config ~/AjouNice2026/configs/sensors/Sensor_competition_local.json \
  --samples 50
```
