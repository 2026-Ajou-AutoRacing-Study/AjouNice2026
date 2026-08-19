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
