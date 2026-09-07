# Map-CV production handoff

이 문서는 2026-08-20 기준 객체 Detection → Tracking → Prediction 파이프라인의
production 설정, 검증 근거, 기각한 실험, 알려진 한계와 재개 순서를 한 곳에 정리한다.
한동안 Prediction 작업을 중단한 뒤 다시 시작할 때 이 문서를 먼저 읽는다.

## 1. Production 데이터 흐름

```text
PointPainting DetectedObjects
  약 15 Hz, 실제 bag에서는 약 8.7 Hz
        ↓
EKF multi-object tracking
  Detection 시각에 measurement update
  내부 scheduler에서 30 Hz predicted state 발행
        ↓
Map-CV prediction
  Lanelet2 v60 중심선과 연결 graph 사용
  0.0~3.0 s, 0.1 s 간격
  서로 다른 plausible route Top-K 발행
        ↓
/perception/object_recognition/objects
```

공식 ROS contract는 다음과 같다.

- Detection: `/perception/object_recognition/detection/objects`.
- Tracking: `/perception/object_recognition/tracking/objects`.
- Prediction: `/perception/object_recognition/objects`.
- 메시지: `DetectedObjects → TrackedObjects → PredictedObjects`.

Detector는 PointPainting과 Fast-BEV 중 하나를 선택할 수 있고, Tracking 이후는
detector 종류를 알 필요가 없다.

## 2. Production 설정

### PointPainting

- Checkpoint: `pointpainting_B0_best_epoch65.pth`.
- 입력 좌표: MORAI raw rolling LiDAR point를 그대로 사용한다.
- Detection timestamp: `object_points_median`.
  - 각 BBox 내부 point의 time 중앙값을 구한다.
  - 객체별 중앙값들을 다시 중앙값 내 메시지 대표시각으로 사용한다.
  - 객체 point를 충분히 찾지 못하면 scan 전체 `point_median`으로 fallback한다.
- Ego-motion coordinate deskew: `off`.
- Timestamp 보정은 모델 입력을 바꾸지 않으므로 재학습이 필요 없다.

### EKF Tracking

- Detection timestamp와 Tracking output timestamp를 분리한다.
- Vehicle orientation은 실제 이동 벡터와 Lanelet 방향을 이용해 canonicalize한다.
- 방향 반전 진입/해제 기준은 각각 약 `100/80 deg` hysteresis를 사용한다.
- 고정 `+0.11 s` 지연값은 사용하지 않는다.
- `compensate_output_to_current_time=false`이다.
- EKF 내부 scheduler는 competition launch에서 `30 Hz`이다.
- 마지막 measurement가 `0.25 s`보다 오래되면 fixed-rate prediction을 중단한다.
- 외부 fixed-rate adapter는 production에서 사용하지 않는다.

30 Hz 출력은 새 센서 정보를 생성하지 않는다. 약 15 Hz Detection 사이를 EKF
motion state로 짧게 외삽해 Planning 입력의 timestamp와 cadence를 일정하게 만드는
기능이다.

### Map-CV Prediction

- Motion budget은 CV, 즉 현재 종방향 속도 유지이다.
- CA는 로터리와 고주로 모두 악화해 사용하지 않는다.
- 시작 위치는 Tracking BBox를 순간이동시키지 않는다.
- 미래 경로가 `1.5 s` 동안 Lanelet centerline으로 점진 수렴한다.
- Lanelet graph를 따라 successor route를 확장한다.
- 최대 raw 후보 수는 6개이다.
- 3초 미래 궤적이 `0.75 m` 이내로 겹치고 downstream Lanelet이 같은 후보는 하나의
  물리적 hypothesis로 묶는다.
- 같은 hypothesis에서는 최고 raw score만 대표로 사용한다. 중복 confidence 합산은
  아직 하지 않는다.
- Prediction horizon은 `0.0~3.0 s`, 간격은 `0.1 s`이다.
- Lanelet map은 `lanelet2_map_v60.osm`을 사용한다.

### Map/TF

- Map-CV는 v60 Lanelet graph를 유지한다.
- MORAI와 Lanelet map 사이의 고정 횡방향 bias는
  `configs/tf/morai_lanelet_static_tf.yaml`의 runtime-origin TF를 통해 보정한다.
- Legacy 결과 재현이 필요할 때만
  `configs/tf/morai_lanelet_static_tf_v23_refined_legacy.yaml`을 사용한다.
- 수정 TF는 Prediction 횡방향 p50을 약 `3~9 cm` 개선했지만, 종방향 오차가 더
  크기 때문에 일반 FDE 변화는 작았다.

## 3. Production 평가 계약

### GT 기준

- 주 지표는 `bbox-center-calibrated-v1`이다.
- MORAI ObjectStatus position과 실제 3D BBox center의 기준점 차이를 asset별 크기로
  보정한다.
- ObjectStatus 원점을 그대로 사용한 약 `1.5 m` 오차는 Tracking 오차로 해석하지
  않는다.
- `origin_aligned`는 현재 위치 오차를 완전히 제거한 motion-only 진단 상한이며
  production 주 지표가 아니다.

### Prediction 지표

- Planning이 Top-K 후보를 사용할 예정이므로 `minFDE-K`를 주 지표로 유지한다.
- Top-1 FDE는 confidence calibration 진단에 사용한다.
- horizon별 mean, p50, p90, p95, MR@2m, coverage를 함께 본다.
- CV-stable slice를 모델 선택 주 지표로 사용하고 전체 구간은 spawn/급가감속
  robustness 보조 지표로 유지한다.
- CV-stable은 예측 시작부터 해당 horizon 끝까지 GT 종방향 가속도 절댓값이 계속
  `0.5 m/s²` 이하인 표본이다. Online에서 알 수 있는 조건이 아니라 offline 평가용이다.

### 현재 대표 성능

PointPainting epoch-65, acquisition-time Detection, EKF internal 30 Hz,
Map-CV, CV-stable 기준이다.

| 장면 | Tracking center p50 | 속도오차 p50 | 1초 minFDE-K p50 | 2초 | 3초 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Roundabout 1 | 0.526 m | 0.422 m/s | 0.628 m | 0.980 m | 1.384 m |
| Highway 1 | 0.532 m | 0.504 m/s | 0.610 m | 0.983 m | 1.448 m |

- 1초는 Planning 주 판단에 사용할 수 있는 수준이다.
- 2초는 보조 판단에 사용한다.
- 3초는 단일 확정 궤적보다 보수적 위험 확인과 불확실성 margin에 사용한다.
- Roundabout 2는 정지선 위치와 원거리 Detection intermittency로 coverage가 낮아
  production 대표 bag으로 사용하지 않는다.

## 4. 채택한 개선

- Detector-neutral Autoware object message contract.
- Vehicle yaw 반전 억제와 이동방향 canonicalization.
- Map-CV Lanelet graph 경로 생성.
- 갈림길 Top-K route hypothesis 발행.
- 동일 물리 경로의 중복 hypothesis clustering.
- v60 runtime-origin TF 정렬.
- BBox-center-calibrated evaluator.
- rolling LiDAR `object_points_median` Detection timestamp.
- EKF 내부 30 Hz output scheduler.
- horizon별 minFDE-K와 종·횡방향 FDE 분해.

## 5. 기각하거나 보류한 실험

### 고정 `+0.11 s` 보정

- 겉으로 보인 약 0.11초 차이의 상당 부분은 MORAI ObjectStatus 원점과 BBox center
  기준점 차이였다.
- 모든 객체에 고정 지연을 더하는 방식은 사용하지 않는다.
- 실제 rolling scan timing은 point별 time으로 처리한다.

### CA motion budget

- Spawn 직후 가속을 일부 설명할 수 있지만 CV 안정 구간과 일반 구간을 모두
  안정적으로 개선하지 못했다.
- 대회 상호작용 구간에서는 NPC가 대체로 순항에 도달한 뒤 중요해진다.
- production은 CV를 유지한다.

### Tracking/Prediction 즉시 centerline snap

- 횡방향 오차는 줄일 수 있지만 현재 위치를 순간이동시키고 잘못된 Lanelet 선택을
  고착할 위험이 있다.
- Tracking BBox는 보존하고 미래 경로만 1.5초에 걸쳐 수렴시킨다.

### 외부 15/30 Hz resampling adapter

- 출력 cadence는 증가했지만 Tracking과 Prediction 정확도가 소폭 악화했다.
- 내부 EKF state를 직접 예측하는 scheduler가 더 일관적이므로 외부 adapter는
  production에서 사용하지 않는다.

### Ego-motion coordinate deskew

- 정적 구조 scan 정합 p50은 `0.498 → 0.449 m`로 좋아졌다.
- 현재 raw-scan checkpoint에서는 Detection precision이 `0.873 → 0.749`, Tracking
  center p50이 `0.532 → 0.562 m`, 3초 minFDE-K p50이
  `1.448 → 1.705 m`로 악화했다.
- 따라서 coordinate deskew는 `off`이다.
- 상세 내용은
  `perception/pointpainting/docs/rolling_lidar_ego_deskew_ab_260820.md`를 참고한다.

## 6. 알려진 한계

- 현재 PointPainting OD는 velocity를 직접 제공하지 않는다. EKF는 연속 위치 변화로
  속도를 추정하므로 Track 생성 직후 수렴 시간이 필요하다.
- EKF 30 Hz 출력은 15 Hz measurement 사이를 외삽할 뿐 새 정보를 추가하지 않는다.
- Rolling LiDAR ego-motion deskew는 production에서 꺼져 있다.
- 움직이는 NPC의 scan 내부 object-motion distortion은 해결하지 않았다.
- Map-CV의 route confidence는 정식 확률이 아니라 거리, yaw, 연결성, 전환 곡률을
  이용한 heuristic이다.
- 갈림길에서 관측만으로 의도를 알 수 없으면 Top-1을 완벽히 맞히는 것은 불가능하다.
  따라서 plausible Top-K를 Planning이 활용해야 한다.
- Planning의 모든 모듈이 아직 PredictedPath Top-K와 confidence를 동일하게 활용하는
  것은 아니다.
- 3초 종방향 오차는 횡방향 오차보다 크며, 주로 속도 state 오차가 누적된 결과다.

## 7. OD 동적 데이터 이후 재개 순서

1. 동적 Vehicle과 Pedestrian 데이터셋을 수집한다.
   - Spawn 직후 가속, 순항, 감속, 횡단을 구분한다.
   - 정적 프레임만 반복하지 않고 연속 frame과 정확한 sensor/ego timestamp를 남긴다.
2. Single-frame OD에서 velocity regression을 별도 실험한다.
   - 이것은 Temporal modeling과 다른 실험이다.
   - OD velocity를 EKF measurement update에 선택적으로 융합하고 위치 차분 속도와
     비교한다.
3. 동적 데이터 single-frame baseline과 실제 multi-frame Temporal model을 구분해
   비교한다.
4. 최근 `0.3~0.5 s` 위치 이력의 robust 종방향 속도 fitting을 EKF ablation으로
   검증한다.
5. Roundabout branch history와 confidence calibration을 개선한다.
6. Planning에 horizon별 uncertainty와 safety margin contract를 전달한다.
7. Training과 inference 모두 동일하게 변환한 데이터가 준비됐을 때만
   ego-motion deskew 재학습을 재검토한다.

## 8. 재개 시 검증 순서

- 먼저 `tracking_prediction_evaluation/README.md`의 evaluator 계약을 확인한다.
- Roundabout 1과 Highway 1을 주 bag으로 사용한다.
- Detection AP만 보지 말고 Tracking center/speed/yaw와 Map-CV 1/2/3초 결과를 같이
  비교한다.
- 전체 구간과 CV-stable을 모두 저장하되 production 선택은 CV-stable을 우선한다.
- 새 변경은 p50만 개선하고 p90/p95, recall, coverage를 손상하지 않는지 확인한다.
- RViz에서는 Top-1과 sibling route를 모두 표시해 Lanelet 선택과 직선 fallback을
  정성 확인한다.
