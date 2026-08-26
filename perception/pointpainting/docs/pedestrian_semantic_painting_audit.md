# Pedestrian rosbag semantic-painting audit.

## 목적.

- 구 Pedestrian rosbag에서 PointPainting confidence가 낮은 원인이 LRASPP
  semantic painting 단계부터 시작되는지 확인함.
- Production LRASPP checkpoint와 online 전처리 조건을 그대로 사용함.
  - checkpoint: `260817_sunny13_1500_9feat_80e/lraspp_4class_best_miou.pth`.
  - torchvision short-side resize `384`.
  - ImageNet normalization.

## 방법.

- `/Object_topic` Pedestrian 3D BBox를 `/Ego_topic`과 `Sensor_6.json`으로 전방
  카메라에 투영함.
- 투영 BBox가 실제 사람 실루엣과 일치하는지 영상으로 검증함.
- 두 지표를 따로 측정함.
  - 2D BBox 전체 영역의 Pedestrian probability.
  - 3D BBox 안의 실제 LiDAR return이 카메라에 투영된 픽셀에서 받은
    Pedestrian probability.
- 두 번째 지표가 PointPainting 9-feature 입력에 직접 들어가는 값임.
- 도로면 contamination을 피하기 위해 BBox 바닥에서 0.15 m 이내인 point를
  제외함.
- 재현 도구는
  `perception/pointpainting/tools/audit_rosbag_pedestrian_semantics.py`임.

## 결과.

| Bag | 표본 | 사람 point 수 p50 | Ped probability mean / p50 | Ped argmax 비율 mean / p50 |
|---|---:|---:|---:|---:|
| Crossing | 96 | 12.5 | 0.539 / 0.503 | 56.4% / 60.0% |
| Distance | 488 | 29.0 | 0.366 / 0.342 | 39.5% / 41.7% |
| Base2 Train | 1,188 | 24.0 | 0.563 / 0.629 | 60.3% / 70.0% |
| Base2 Val | 246 | 14.0 | 0.615 / 0.651 | 68.9% / 75.0% |

- Crossing의 실제 사람 point 중 평균 43.6%가 Background argmax였음.
- Distance bag은 평균 60.1%가 Background argmax였음.
- Vehicle·Obstacle confusion은 거의 없었고 주로 Pedestrian과 Background 사이의
  혼동이었음.
- Distance bag의 거리는 횡방향 위치와 scale이 함께 달라지는 통제되지 않은
  배치이므로 순수한 거리 효과로 해석하지 않음.
  - 5 m actor는 카메라 좌측에 크게 잘린 close-up으로 나타나 Ped probability가
    특히 낮았음.
  - 10 m가 가장 높고 15 m, 18 m 순으로 하락함.
- camera--LiDAR timestamp 절대차 p50은 Crossing 0.48 ms, Distance 2.05 ms였음.
  - 일부 최대 34--37 ms 표본은 있으나 이 bag의 주된 semantic 저하를 고정
    sensor 비동기로 설명하기는 어려움.

## 판정.

- LRASPP가 같은 `Man1`을 못 알아보는 asset mismatch 가설은 기각함.
- 대신 online-like rosbag의 scale·화면 위치·배경 조건에서 사람 point를
  Background로 칠하는 semantic domain gap이 실제로 존재함.
- 이 결과는 semantic stage가 Pedestrian confidence 저하에 기여한다는 직접
  증거임.
- 다만 LRASPP가 3D detection miss의 유일한 원인인지는 아직 증명하지 않음.
  - 완전한 causal 분리는 GT BBox 안 point의 Pedestrian channel을 1로 바꾸는
    oracle-painting ablation과 원본 painting 결과를 같은 PointPillars checkpoint에
    넣어 비교해야 함.

## Obstacle에 대한 후속 원칙.

- 단순 positive frame 증량을 우선하지 않음.
- 최신 competition sensor bag에서 Obstacle GT point semantic audit과 oracle
  painting ablation을 먼저 수행함.
- LRASPP 병목이면 Base2/Base3 current-JPEG semantic data로 segmentation을
  재학습함.
- PointPillars 병목이면 compact/long/tall shape supervision을 분리하고 ROS
  출력에서 다시 `Obstacle`로 collapse하는 구조를 검토함.
- RoadFurniture와 collision obstacle의 label policy 및 asset별 BBox--LiDAR
  geometry를 다시 감사함.
- GT database sampling은 무조건 10개를 복사하지 않고 shape·거리·context 및
  collision-aware sampling으로 교체함.
- competition full-route false positive를 이용한 hard-negative mining과
  class-specific threshold/NMS calibration을 별도로 수행함.
