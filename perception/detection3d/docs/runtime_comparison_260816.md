# Fast-BEV / PointPainting ROS runtime comparison (2026-08-16)

## 비교 기준

- GPU: NVIDIA GeForce RTX 5060 Ti
- 입력: `sensor_cam_raw_roundabout1.bag`, 1.73배속
- camera 입력: 약 34.6 Hz
- PointPainting LiDAR 입력: 약 15.0 Hz
- image short side: 384 px
- camera CPU workers: 4
- AMP/FP16: enabled
- 첫 detection 10개는 latency 통계에서 제외
- 백분위는 선형 보간 방식 사용

`compute`는 실제 연산 단계의 합이고, `input→publish latency`는 해당 detector의
입력 callback부터 detection 발행까지의 wall-clock 시간이다. Fast-BEV는 동기화된
camera set 도착, PointPainting은 LiDAR 도착이 시작점이다. 센서가 MORAI에서 ROS로
전달되기 전의 시간은 포함하지 않는다.

## 동일 재생 조건 결과

| Detector / backend | Camera input | LiDAR input | Detection output | steady drop / skip | Compute mean | Compute p50 | Compute p95 | Compute p99 | Input→publish mean | Input→publish p50 | Input→publish p95 | Input→publish p99 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Fast-BEV, OpenCV, staged | 34.600 Hz | — | **34.196 Hz** | 0.74% / 0% | **37.58 ms** | **36.96 ms** | **41.25 ms** | **56.09 ms** | **41.95 ms** | **40.08 ms** | **52.24 ms** | **74.85 ms** |
| PointPainting, OpenCV | 34.596 Hz | 14.999 Hz | 14.965 Hz | 0% / 0.22% | 45.50 ms | 45.21 ms | 48.87 ms | 56.86 ms | 46.73 ms | 45.97 ms | 52.97 ms | 60.69 ms |
| PointPainting, PIL | 34.596 Hz | 14.999 Hz | 14.965 Hz | 0.22% / 0% | 55.25 ms | 54.89 ms | 59.51 ms | 61.61 ms | 60.71 ms | 58.51 ms | 72.48 ms | 77.22 ms |

Fast-BEV 출력률과 PointPainting 출력률의 직접 비교에는 센서 차이가 있다.
PointPainting은 LiDAR 15 Hz에 의해 출력 상한이 정해지고, Fast-BEV는 camera-only라
34.6 Hz 입력을 그대로 처리한다. 따라서 출력률은 모델 우열만이 아니라 센서 발행률도
반영한다. 반면 latency 열은 각 detector가 자기 입력을 받은 뒤 결과를 내기까지의
시간을 같은 단위로 비교한 값이다.

## 단계별 평균 시간

| Detector / backend | Decode-stage timer | Transform-stage timer | H2D | Main inference | Painting | PointPillars | Queue | Publish | Compute total | Input→publish |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Fast-BEV, OpenCV, staged | 3.12 ms | 8.86 ms | 0.51 ms | 25.09 ms | — | — | 3.57 ms | 0.63 ms | 37.58 ms | 41.95 ms |
| PointPainting, OpenCV | 13.64 ms | 3.35 ms | 1.27 ms | 9.39 ms (LRASPP) | 1.63 ms | 11.99 ms | 1.23 ms | 2.13 ms | 45.50 ms | 46.73 ms |
| PointPainting, PIL | 14.17 ms | 12.08 ms | 1.28 ms | 10.22 ms (LRASPP) | 1.62 ms | 11.89 ms | 5.47 ms | 2.07 ms | 55.25 ms | 60.71 ms |

PointPainting의 `compute total`에는 표에 생략한 camera matching, LiDAR 전처리,
segmentation postprocess 및 PointPillars 내부 전처리/후처리도 포함되므로 표의 일부
열을 단순 합산한 값과 정확히 같지 않다. Fast-BEV의 `main inference`는 4-camera
BEV 추론 전체다.

두 camera 타이머의 코드 경계도 완전히 같지는 않다. Fast-BEV의 decode-stage는
`cv2.imdecode`만 포함하고 BGR→RGB 변환은 축소된 영상의 transform-stage에서 한다.
PointPainting의 decode-stage는 `cv2.imdecode` 뒤 원본 해상도 BGR→RGB
`copy()`까지 포함한다. 또한 Fast-BEV는 네 영상을 모두 `384×704`로 만들지만,
PointPainting은 short side 384와 원 종횡비를 유지하여 front/back은 약
`384×683`, side는 `384×512`로 처리한다. 따라서 위 두 stage를 개별적으로
detector 사이에 비교하기보다 두 단계의 합과 전체 compute를 중심으로 해석한다.

## 대회 목표 30 Hz Fast-BEV 참고값

| Camera input | Detection output | steady drop | Compute mean / p50 / p95 / p99 | Input→publish mean / p50 / p95 / p99 |
|---:|---:|---:|---:|---:|
| 29.999 Hz | 29.870 Hz | 0% | 37.01 / 36.66 / 39.77 / 47.68 ms | 38.99 / 38.08 / 45.25 / 56.20 ms |

30 Hz에서 queue의 steady-state 증가는 없었으며 실제 내부 drop도 0이었다.

## 결론

- Fast-BEV는 30 Hz를 drop 없이 처리하고 34.6 Hz stress도 약 34.3 Hz로 추종했다.
- 동일 34.6 Hz camera 입력에서 Fast-BEV latency p50은 40.08 ms다.
- PointPainting OpenCV 4-worker latency p50은 45.97 ms, PIL 4-worker는 58.51 ms다.
- PointPainting은 대회 LiDAR 15 Hz를 둘 다 따라가지만, OpenCV가 PIL보다 latency와
  tail latency를 크게 줄였다.
- 실전 배치는 평균뿐 아니라 p95/p99와 다른 GPU 노드 동시 실행 시 drop을 함께 봐야
  한다.
