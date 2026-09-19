# PointPainting ROS runtime benchmark (2026-08-16)

## 목적

PointPainting 온라인 파이프라인에서 다음 항목을 서로 분리해 측정했다.

- 입력률과 detection 발행률(throughput)
- LiDAR 입력 한 프레임부터 detection 발행까지의 지연(latency)
- 최신 프레임 교체(drop), 처리 실패(skip), queue 지연
- 카메라 decode부터 PointPillars postprocess까지의 단계별 시간
- 카메라 CPU 병렬화와 OpenCV 전처리의 효과

Throughput과 latency는 같은 값이 아니다. 파이프라인 병렬화는 결과 간격을
줄일 수 있지만, 한 프레임의 입력부터 출력까지 걸리는 시간은 그대로일 수
있다. 따라서 두 지표를 별도로 기록한다.

## 환경과 입력

- GPU: NVIDIA GeForce RTX 5060 Ti
- PointPainting release: `260816_sunny13_1500_9feat_12e`
- semantic: LRASPP MobileNetV3, MORAI 4-class
- detector: PointPillars, 9-feature / 3-class
- AMP: enabled
- semantic short side: 384 px
- 입력 bag:
  - `sensor_cam_raw_roundabout1.bag`
  - `sensor_cam_raw_highway1.bag`
- 원본 bag: camera 약 20 Hz, LiDAR 약 8.67 Hz
- 1.73배속 stress: camera 약 34.6 Hz, LiDAR 약 15.0 Hz

1.73배속은 대회 설정인 camera 30 Hz / LiDAR 15 Hz보다 카메라 쪽은 약간 더
가혹한 조건이다.

## 지표 정의

- `total_compute_ms`: worker가 한 LiDAR 프레임의 처리를 시작한 시점부터
  detection publish가 끝날 때까지
- `queue_delay_ms`: LiDAR callback 도착 후 worker가 처리를 시작할 때까지
- `arrival_to_publish_ms`: `queue_delay_ms + total_compute_ms`
- `drop`: queue가 이미 차 있을 때 더 최신 LiDAR가 들어와 이전 scan을 교체한 수
- `skip`: camera sync 실패, empty LiDAR 또는 처리 예외로 publish하지 못한 수

## 전체 결과

각 latency는 `p50 / p95`이다.

| Bag / 설정 | LiDAR 입력 | Detection 출력 | Drop | Compute ms | Queue ms | Arrival→publish ms |
|---|---:|---:|---:|---:|---:|---:|
| Roundabout 1×, PIL 1 worker | 8.668 Hz | 8.668 Hz | 0.00% | 71.53 / 75.47 | 0.04 / 4.42 | 72.22 / 77.09 |
| Roundabout 1.73×, PIL 1 worker | 14.990 Hz | 13.924 Hz | 6.91% | 70.07 / 73.37 | 30.01 / 66.45 | 100.09 / 137.30 |
| Roundabout 1.73×, PIL 1 worker, 상세 동기화 off | 14.990 Hz | 14.022 Hz | 6.26% | 69.86 / 73.55 | 31.86 / 64.80 | 101.83 / 134.49 |
| Roundabout 1.73×, **PIL 4 workers** | 14.999 Hz | **14.965 Hz** | **0.22%** | **54.89 / 59.51** | **0.05 / 16.11** | **58.51 / 72.48** |
| Roundabout 1.73×, OpenCV 1 worker | 14.999 Hz | 14.927 Hz | 0.22% | 59.49 / 63.66 | 0.04 / 21.92 | 64.22 / 81.82 |
| Roundabout 1.73×, **OpenCV 4 workers** | 14.999 Hz | **14.965 Hz** | **0.00%** | **45.21 / 48.87** | **0.04 / 6.09** | **45.97 / 52.97** |
| Highway 1×, PIL 1 worker | 8.668 Hz | 8.669 Hz | 0.00% | 70.65 / 74.71 | 0.04 / 4.41 | 71.18 / 75.87 |
| Highway 1.73×, PIL 1 worker | 14.994 Hz | 14.185 Hz | 5.31% | 69.00 / 72.92 | 31.19 / 64.14 | 100.37 / 133.11 |

상세 CUDA 동기화 on/off 차이가 작으므로, 기존 70 ms 병목은 계측 오버헤드가
만든 착시가 아니다. Roundabout과 Highway 결과도 유사하므로 객체 밀도보다
고정적인 4-camera 처리 비용이 주 병목이다.

## 단계별 시간

Roundabout 1.73× 기준 `p50 / p95`이다.

| 단계 | PIL 1 worker | PIL 4 workers | OpenCV 1 worker | OpenCV 4 workers |
|---|---:|---:|---:|---:|
| 4-camera JPEG decode | 27.25 / 28.82 ms | **14.21 / 16.03 ms** | 27.66 / 31.20 ms | **13.66 / 15.89 ms** |
| Resize + tensor + normalize | 15.11 / 16.35 ms | 11.99 / 14.42 ms | 4.64 / 6.48 ms | **3.24 / 3.84 ms** |
| LRASPP inference | 9.02 / 9.76 ms | 10.36 / 11.59 ms | 9.18 / 9.83 ms | 9.21 / 10.69 ms |
| Painting | 1.60 / 1.99 ms | 1.54 / 2.21 ms | 1.52 / 1.73 ms | 1.55 / 2.23 ms |
| PointPillars total | 12.10 / 12.63 ms | 11.81 / 12.34 ms | 11.71 / 12.16 ms | 11.95 / 12.81 ms |
| 전체 compute | 70.07 / 73.37 ms | 54.89 / 59.51 ms | 59.49 / 63.66 ms | **45.21 / 48.87 ms** |

기존 직렬 경로에서는 JPEG decode와 PIL/torchvision 전처리가 전체 시간의 약
60%를 차지했다. 모델 추론 자체보다 CPU 이미지 경로가 더 큰 병목이었다.

## Drop 및 queue 안정성

- PIL 1 worker, 15 Hz 상당: Roundabout 32/463 drop, Highway 약 5.3% drop
- PIL 4 workers: 1/464 drop, skip 0, queue 중앙값 0.05 ms
- PIL 4 workers에서 queue의 마지막 25% 중앙값은 처음 25%보다 0.01 ms 낮아
  지속적인 backlog는 없었다.
- 남은 한 번의 drop 직전 프레임은 compute 146.4 ms의 일회성 stall이었다.
  정상 p99 compute는 61.6 ms였으나 max stall 때문에 최신 scan 교체가 한 번
  발생했다.

결론적으로 15 Hz 평균 처리율은 충족하지만, 실차에서는 p99/max stall과 다른
GPU 노드의 동시 실행 영향을 추가로 관찰해야 한다.

## 병렬화 판단

현재 적용한 `camera_cpu_workers=4`는 한 프레임 안에서 네 카메라의 JPEG decode와
동일한 PIL transform을 병렬 실행한다. 학습 때 사용한 전처리 의미를 바꾸지
않으면서 latency와 throughput을 동시에 개선한다.

추가로 프레임 간 2-stage 파이프라인도 구현 가능하다.

1. stage A: camera match, JPEG decode, resize/normalize, LiDAR decode
2. stage B: LRASPP, painting, PointPillars, publish

`n+1` 프레임의 stage A를 `n` 프레임의 stage B와 겹치면 결과 발행 간격은
줄어든다. 그러나 `n` 프레임 자체의 latency는 stage A+B이므로 크게 줄지 않는다.
PIL 4-worker만으로 15 Hz를 충족하므로, 구조 복잡도가 큰 2-stage는 다른 GPU
모델을 동시에 실행했을 때 다시 필요성을 판단한다.

## OpenCV 판단

OpenCV backend는 opt-in으로 구현했다.

- 순수 4-camera transform microbenchmark:
  - torchvision/PIL p50 약 16.6 ms
  - 최적화된 OpenCV resize + Torch normalize p50 약 3.4 ms
- 온라인 전체 pipeline에서는 preprocess p50 15.1 → 4.6 ms
- OpenCV 4-worker 조합에서는 preprocess p50 3.24 ms, 전체 compute p50
  45.21 ms를 기록했다.
- 하지만 동일 이미지 tensor 차이는 normalized tensor 기준:
  - front/back MAE 약 0.018~0.020
  - side MAE 약 0.010

PIL antialiased bilinear와 OpenCV `INTER_LINEAR`의 resampling 결과가 완전히 같지
않다. 현재 segmentation checkpoint는 PIL 경로로 학습됐으므로, OpenCV를
production 기본값으로 바꾸기 전에는 segmentation mIoU와 최종 3D AP를 다시
평가해야 한다. 현재 production 기본값은 `torchvision + 4 workers`이다.

## 실행 명령

```bash
cd ~/AjouNice2026

# 원본 속도 상세 계측
perception/pointpainting/tools/run_rosbag_runtime_benchmark.sh \
  rosbags/raw_sensor_w_cam/sensor_cam_raw_roundabout1.bag \
  1.0 \
  artifacts/detection3d/pointpainting/benchmarks/roundabout_x1 \
  roundabout_x1

# LiDAR 15 Hz 상당 stress, production PIL 4-worker
POINTPAINTING_CAMERA_CPU_WORKERS=4 \
perception/pointpainting/tools/run_rosbag_runtime_benchmark.sh \
  rosbags/raw_sensor_w_cam/sensor_cam_raw_roundabout1.bag \
  1.73 \
  artifacts/detection3d/pointpainting/benchmarks/roundabout_x1p73 \
  roundabout_x1p73

# OpenCV 실험 backend
POINTPAINTING_PREPROCESS_BACKEND=opencv \
perception/pointpainting/tools/run_rosbag_runtime_benchmark.sh \
  rosbags/raw_sensor_w_cam/sensor_cam_raw_roundabout1.bag \
  1.73 \
  artifacts/detection3d/pointpainting/benchmarks/roundabout_opencv \
  roundabout_opencv
```

각 출력 폴더에는 다음 파일이 생긴다.

- `summary.json`: 먼저 확인할 작은 요약
- `frame_metrics.csv`: 프레임별 상세 timing/drop/queue
- `pointpainting.log`, `rosbag.log`, `roscore.log`: 디버깅용 원문

## 결론

1. 기존 1-worker는 LiDAR 15 Hz에서 약 5~7% scan을 교체하므로 부족하다.
2. PIL 4-worker는 학습 전처리를 유지하면서 14.97 Hz와 0.22% drop을 달성했다.
3. 현재 production 기본은 PIL/torchvision + camera CPU workers 4가 적절하다.
4. OpenCV 4-worker가 가장 빠르지만 최종 AP 재검증 전까지 실험 옵션으로 둔다.
5. 2-stage는 구현 가능하지만 현재 단독 PointPainting 15 Hz에는 필수는 아니다.
