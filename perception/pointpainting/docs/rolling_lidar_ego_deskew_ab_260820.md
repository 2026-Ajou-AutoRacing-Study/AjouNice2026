# Rolling LiDAR ego-motion deskew A/B

## 결론

- PointPainting production에서는 ego-motion deskew를 사용하지 않는다.
- production 기본값은 `lidar_deskew_mode=off`이다.
- 기존 `object_points_median` measurement timestamp 보정은 그대로 유지한다.
- deskew 구현은 opt-in 실험 기능으로 남긴다. 현재 checkpoint에 바로 적용하면 정적 배경의 기하 정합은 좋아지지만 실제 detection, tracking, prediction은 악화됐다.

## 구현 범위

- `/Ego_topic`의 50 Hz position과 heading을 시간 보간한다.
- LiDAR의 point별 `time`을 이용해 각 point를 공통 LiDAR pose로 변환한다.
- LiDAR extrinsic의 translation과 yaw를 함께 반영한다.
- detector 출력 BBox를 최종 measurement timestamp의 LiDAR frame으로 다시 표현한다.
- reference는 `header` 또는 `point_median`을 선택할 수 있다.
- 카메라 선택 시각은 `header` 또는 deskew `reference`를 선택할 수 있다. 물리적으로 일관된 기본값은 `reference`이다.

이 구현은 Ego motion만 보정한다. 움직이는 NPC가 한 scan 동안 이동하면서 생기는 object-motion distortion은 보정하지 않는다.

## 기하 검증

Synthetic unit test에서 다음을 확인했다.

- 순수 translation 시 동일 world point가 공통 frame에서 일치한다.
- yaw rotation 시 동일 world point가 공통 frame에서 일치한다.
- BBox의 source/target LiDAR-time 변환이 일관된다.
- pose coverage 부족과 제한된 causal extrapolation을 구분한다.

Highway 1의 정적 구조를 연속 scan끼리 world frame에서 비교한 결과는 다음과 같다.

| 지표 | Raw scan | Ego deskew | 변화 |
| --- | ---: | ---: | ---: |
| Static nearest-neighbor p50 | 0.498 m | **0.449 m** | -9.9% |
| Static nearest-neighbor p90 | 1.222 m | **1.180 m** | -3.4% |

따라서 좌표 변환 방향 자체는 맞다.

## 실제 pipeline A/B

주 비교는 전체 441 LiDAR frame을 모두 처리한 Highway 1 결과이다.

- Baseline: deskew off, production `object_points_median` timestamp.
- Deskew: point-median reference, reference-time camera matching, output BBox를 object-point measurement time으로 변환.
- 평가: BBox-center-calibrated GT, CV-stable slice, Map-CV minFDE-K.

| 지표 | Baseline | Ego deskew | 변화 |
| --- | ---: | ---: | ---: |
| Detection precision | **0.873** | 0.749 | -0.123p |
| Detection recall | **0.870** | 0.853 | -0.017p |
| Tracking center p50 | **0.532 m** | 0.562 m | +5.7% |
| Tracking center p90 | **0.904 m** | 1.130 m | +25.0% |
| Speed error p50 | **0.504 m/s** | 0.668 m/s | +32.6% |
| 1 s minFDE-K p50 | **0.610 m** | 0.774 m | +26.9% |
| 2 s minFDE-K p50 | **0.983 m** | 1.207 m | +22.8% |
| 3 s minFDE-K p50 | **1.448 m** | 1.705 m | +17.7% |

카메라를 scan-header에 고정하거나, LiDAR와 카메라를 모두 scan-header reference로 맞춘 보조 실험도 성능 저하 방향을 뒤집지 못했다.

Highway 1에서 point-median deskew의 좌표 변화량은 작지 않았다.

- point displacement frame mean의 p50은 약 0.442 m이다.
- frame별 point displacement p95의 p50은 약 1.213 m이다.
- LiDAR preprocess overhead는 p50 약 4.32 ms, p95 약 5.13 ms이다.

Roundabout 1은 Ego가 정지해 있어 deskew의 유효 A/B 장면이 아니다.

- Ego XY peak-to-peak 이동은 약 0.000072 m이다.
- heading peak-to-peak 변화는 약 0.0012 deg이다.
- 이 bag의 moving-NPC distortion은 ego-motion deskew로 해결되지 않는다.

## 해석

- 기하학적으로 더 반듯한 point cloud가 현재 checkpoint에 더 좋은 입력이라는 보장은 없다.
- 현재 PointPillars checkpoint는 MORAI의 raw rolling scan으로 학습됐다. 약 0.4~1.2 m 규모로 point 분포를 바꾸면 학습 분포와 달라진다.
- PointPainting은 point와 semantic camera score의 결합 결과에 민감하다. LiDAR reference와 카메라 시각을 맞춰도 학습 때 없던 보정 분포 자체가 남는다.
- FP가 크게 증가했으므로 단순 evaluator timestamp 문제만으로 설명할 수 없다.

## 다시 시도할 조건

다음 조건을 모두 갖춘 별도 실험에서만 production 승격을 재검토한다.

- 학습 데이터에도 동일한 deskew를 적용한다.
- 학습과 online inference가 동일한 LiDAR reference와 camera matching 정책을 사용한다.
- moving-object distortion은 별도 문제로 분리한다.
- Highway처럼 Ego가 실제 이동하는 bag에서 detection AP, tracking, prediction을 함께 비교한다.

## 향후 deskew 재학습 계획

현재 결과는 deskew 자체의 최종 기각이 아니다. raw rolling scan으로 학습한
checkpoint에 deskew 좌표를 inference에서만 적용해 입력 분포가 달라진 실험이다.
다음 대규모 데이터 수집에서 학습과 inference의 계약을 일치시킨 별도 A/B를 다시
수행한다.

모델 구조 변경은 필수가 아니다. 기존 9-feature PointPainting과 PointPillars head를
그대로 유지하고, semantic painting 전에 LiDAR XYZ만 공통 reference pose로 변환한다.
raw와 deskew 데이터는 같은 원본 frame에서 함께 생성해 split, seed, optimizer와
epoch를 동일하게 비교한다.

### 현재 SaveSensorData의 한계

실제 정적·동적 capture의 `LIDAR_5/*_Intensity.bin`은 307,200 byte이며,
`19,200 point x 4 float32`인 `x,y,z,intensity`만 저장한다. point별 `time`과 scan
구간의 Ego pose trajectory는 포함하지 않는다. 따라서 현재 3,000-frame 패키지는
정확한 point-time feature 또는 point별 deskew 재학습에 바로 사용할 수 없다.

저장 순서가 `1,200 azimuth x 16 channel`이라는 사실과 scan 시작 방향을 live
PointCloud2와 대조해 검증하면 상대시각을 근사 복원할 여지는 있다. 검증되지 않은
index 기반 시간을 production GT처럼 사용하지 않는다.

### 다음 수집의 sidecar 계약

SaveSensorData를 Camera·LiDAR·GT의 canonical snapshot으로 계속 사용하면서 다음
정보만 작은 sidecar로 저장한다.

- `/sensor/lidar1` PointCloud2의 point별 `time` 또는 원본 field layout.
- scan 시작·종료·reference timestamp.
- scan 구간을 덮는 `/Ego_topic` pose samples.
- 적용한 LiDAR extrinsic과 deskew contract version.
- SaveSensorData XYZI와 ROS scan의 point count, timestamp, coordinate/hash 대조 결과.

동일 scan임을 검증하지 못한 frame은 deskew 학습 대상으로 승인하지 않는다. rosbag을
주 수집 수단으로 되돌리는 계획은 아니며, SaveSensorData snapshot에 timing sidecar만
보강하는 방식이다.

### 실험 순서와 채택 기준

1. Live smoke 한 장에서 SaveSensorData XYZI와 `/sensor/lidar1`이 동일 scan인지
   좌표·순서·개수로 검증한다.
2. 100--300 frame pilot에서 raw 9-feature와 deskew 9-feature를 동시에 만든다.
3. 같은 초기 weight와 학습 설정으로 짧은 A/B를 수행한다.
4. Detection center AP·IoU AP·mATE·mAOE뿐 아니라 Tracking center/speed error와
   Map-CV 1/2/3초 minFDE-K를 함께 비교한다.
5. p50만 좋아지고 precision, p90/p95, recall 또는 coverage가 악화하면 production에
   적용하지 않는다.
6. 효과가 확인된 뒤에만 전체 수집과 online launch의 기본값을 deskew로 변경한다.

Point-time을 열 번째 feature로 추가해 object-motion 단서를 학습하는 실험과
`vx,vy` velocity head는 별도 ablation이다. Ego deskew만으로 움직이는 NPC의 scan
내부 distortion이나 실제 속도가 자동으로 해결된다고 가정하지 않는다.
