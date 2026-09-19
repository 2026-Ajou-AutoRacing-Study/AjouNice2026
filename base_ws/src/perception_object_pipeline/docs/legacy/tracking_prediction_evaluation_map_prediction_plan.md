# Tracking·Prediction 평가 재설계와 지도 기반 예측 도입 계획

## 목적

현재 표준 객체 파이프라인은 다음 인터페이스를 사용한다.

```text
DetectedObjects
  -> EKF multi-object tracker
  -> TrackedObjects
  -> CTRV/CV predictor
  -> PredictedObjects
```

이 문서는 다음 두 작업의 기준을 정한다.

1. 현재 tracking과 CTRV/CTRA prediction의 실제 성능을 재평가한다.
2. K-City Lanelet2 지도를 이용하는 map-based prediction을 ROS1 시스템에 도입한다.

대회 규정의 끼어들기 의미, v60의 실제 로터리·고주로 Lanelet 관계, 최신
Planning의 모듈별 다중 predicted path 소비 정책과 합류 기능 공백은
[`competition_merge_prediction_planning_contract.md`](competition_merge_prediction_planning_contract.md)에
별도로 정리한다.

## 현재 코드 감사 결과

### 런타임 파이프라인

- 차량 prediction은 CTRV를 사용한다.
- 보행자·자전거·unknown은 Cartesian constant velocity를 사용한다.
- prediction horizon은 3.0초, 간격은 0.1초이며 현재 pose를 포함해 31개 pose를 발행한다.
- predictor는 `map` frame의 `TrackedObjects`를 요구한다.
- 모든 객체는 현재 하나의 predicted path와 confidence 1.0만 가진다.
- EKF tracker는 CV·CTRV·CA·CTRA 상태 천이 모델을 지원하며 현재 기본값은 CTRV이다.
- 현재 association은 모든 후보 쌍을 거리 순으로 정렬해 선택하는 greedy 방식이다. 전역 최적 Hungarian association은 아니다.

### 기존 평가 코드의 한계

Git 이력의 구 `tracking_prediction` 평가기는 현재 표준 파이프라인의 정확도
평가기와 호환되지 않는다. 감사 후 package·스크립트·결과 문서를 작업트리에서
제거했다.

- 대부분 MORAI `/Object_topic`의 GT 위치·heading·속도를 prediction 초기 상태로 직접 사용한다.
- 실제 EKF가 추정한 속도·yaw rate·covariance를 입력으로 사용하지 않는다.
- detector의 false positive, false negative, classification error를 반영하지 않는다.
- association 실패, ID switch, track 생성 지연·소멸·재생성을 반영하지 않는다.
- PointPainting 오차를 하나의 평균 중심 오차에서 역산한 등방 Gaussian 노이즈로 대체한다.
- 일부 과거 CTRA smoothing 실험은 온라인에서 사용할 수 없는 미래 표본을 포함했다.
- bag에 따라 `ObjectStatus.velocity` 단위가 m/s와 km/h로 다르지만 레거시는
  일괄적으로 3.6으로 나눴다.
- MORAI `unique_id`가 시간에 따라 서로 다른 actor에 재사용되지만 영구 GT ID로
  취급했다.

따라서 기존 수치는 현재 실제 시스템의 CTRV/CTRA 성능으로 인용하지 않는다. 새
평가기에는 레거시 구현을 복사하지 않고, GT 궤적 재구성·시간 정렬·metric을 독립적으로
다시 구현했다. 과거 내용은 필요할 때만 Git 이력으로 확인한다.

## 평가를 세 층으로 분리한다

### E0: Oracle motion-model benchmark

```text
MORAI GT의 과거 궤적 -> CTRV / CTRA / map-based -> 미래 GT
```

목적은 tracker와 detector 영향을 제거하고 prediction 모델 자체의 상한을 비교하는 것이다. 모든 상태 추정은 causal history만 사용한다.

E0에는 runtime tracker 출력이 없으므로 tracking accuracy를 평가할 수 없다. GT
프레임 간 correspondence는 predictor 입력 궤적을 만들기 위한 전처리일 뿐이며,
이를 tracking 성능으로 보고하지 않는다. Tracking 평가는 실제 `TrackedObjects`를
기록하는 E1 이후로 미룬다.

### E1: Tracker-state prediction benchmark

```text
동일 GT 장면 -> 실제 EKF TrackedObjects -> 각 predictor -> 미래 GT
```

목적은 EKF 상태 추정 오차와 prediction 모델의 합성 성능을 측정하는 것이다. CTRV/CTRA/map-based predictor는 동일한 `TrackedObjects`를 받아야 한다.

### E2: Full detector-to-prediction benchmark

```text
Fast-BEV 또는 PointPainting
  -> DetectedObjects
  -> EKF tracker
  -> predictor
  -> 미래 GT
```

실제 대회 성능을 나타내는 최종 평가다. detector miss, false positive, class error, association, track latency와 prediction을 모두 포함한다.

현재 우선순위는 E0로 motion model 상한을 고정한 뒤 P1 map-based predictor를
검증하는 것이다. E1과 E2는 detector·tracker 성능이 더 안정화된 뒤 같은 장면
split과 horizon에서 추가한다. Prediction이 존재한 표본만 계산한 조건부 오차와,
prediction 누락까지 실패로 포함한 전체 성능을 반드시 분리한다.

## Tracking 평가 지표

### 주 지표

- HOTA, DetA, AssA, LocA
- IDF1
- track precision·recall
- ID switch와 fragmentation
- track initialization delay
- occlusion 후 ID 유지율과 track survival time

HOTA는 detection과 association 성능을 함께 보되 하위 지표로 원인을 분리할 수 있어 주 지표로 적합하다. MOTA·MOTP와 nuScenes 방식 AMOTA·AMOTP는 보조 지표로 유지한다.

### 상태 추정 지표

- ground-plane center error
- longitudinal·lateral position error
- speed·velocity-vector error
- yaw·yaw-rate error
- class·거리·속도별 p50, p90, p95
- covariance calibration은 후속 단계에서 NIS/NEES로 검사

MORAI GT unique ID와 tracker UUID를 직접 동일시하지 않는다. 각 timestamp에서 class와 거리 gate를 적용한 Hungarian matching으로 correspondence를 만든 뒤 시간 association을 평가한다.

## Prediction 평가 지표

### 단일 경로 CTRV·CTRA

- ADE와 FDE: 0.5, 1.0, 1.5, 2.0, 3.0초
- p50, p90, p95와 RMSE
- horizon별 Miss Rate: threshold는 1m·2m와 planning safety requirement에서 정한 값을 함께 사용
- Lanelet2 기준 longitudinal·lateral ADE/FDE
- off-road rate와 lane-boundary violation rate

### 다중 경로 map-based prediction

- confidence가 가장 높은 경로의 top-1 ADE/FDE/MR
- minADE-K, minFDE-K, MR-K
- 하나 이상의 mode가 GT를 포함하는 path recall
- Brier-minFDE 또는 calibration 지표
- predicted path confidence 합과 reliability

`minFDE-K`만 사용하면 많은 경로를 무분별하게 생성하는 모델이 유리하다. 따라서 planning이 실제로 사용할 top-1 성능, 후보 coverage, confidence calibration을 함께 보고한다.

### 분석 bucket

- vehicle / pedestrian / bicycle / unknown
- straight / curve / intersection / roundabout / lane change / crosswalk
- stationary / low-speed / moving
- object distance와 speed
- TTC가 짧은 planning-critical 표본

## 데이터와 rosbag 요구사항

현재 `roundabout1.bag`, `roundabout2.bag`, `sensor_cam_raw_highway1.bag`의
`/Object_topic`은 E0에 재사용할 수 있다. `highway2.bag`과
`competition_route_full_highspeed.bag`에는 평가할 ObjectStatus가 없어 제외한다.
과거 legacy node 출력 bag은 새 표준 tracking topic이 없고 고주기로 중복 발행한
기록이므로 삭제했으며 현재 파이프라인 기준값으로 사용하지 않는다.

필요한 신규 기록은 다음 topic을 같은 bag에 담아야 한다.

```text
/Object_topic
/Ego_topic
/tf
/tf_static
/perception/object_recognition/detection/objects
/perception/object_recognition/tracking/objects
/perception/object_recognition/objects
```

MORAI 객체 ID는 영구 ID가 아니므로 class별 causal position prediction과 Hungarian
assignment로 내부 GT 궤적을 재구성한다. Spawn·despawn·teleport는 trajectory
segment 경계로 자동 분리하고, 경계 주변 미래 horizon을 정상 prediction 실패로
세지 않는다. E0 속도·가속도·yaw rate는 unit이 불일치하는 message velocity가
아니라 과거 0.5초 GT pose의 causal polynomial fit으로 추정한다.

## Map-based prediction 포팅 판단

### 가능한 이유

- 현재 runtime 메시지가 Autoware 계열 `TrackedObjects -> PredictedObjects`로 이미 통일돼 있다.
- ROS Noetic에 Lanelet2 core, IO, matching, routing, traffic rules가 설치돼 있다.
- mission planner에 K-City OSM과 projector를 직접 읽는 ROS1 Lanelet2 코드가 이미 있다.
- K-City v60과 projector가 현재 작업트리에 있으며 최신 planning 검증 브랜치에서 사용된다.

### 그대로 빌드할 수 없는 이유

최신 `autoware_map_based_prediction`은 ROS2/ament, rclcpp components, Agnocast, 최신 Autoware utility와 message API에 의존한다. 소스 디렉터리를 catkin workspace에 복사하는 방식은 유지보수성과 검증 가능성이 낮다.

따라서 최신 구현의 알고리즘과 테스트를 기준으로 삼되, ROS1 표준 메시지와 현재 Lanelet2 loader에 맞는 독립 predictor로 단계적으로 이식한다.

## 권장 포팅 단계

### P0: E0 평가기를 먼저 구축 — 완료

독립 ROS package `tracking_prediction_evaluation`에 causal CTRV/CTRA E0 평가기를
구축했다. 합성 단위 테스트와 roundabout/highway 원시 bag 검증을 통과했으며,
상세 기준선은
[`tracking_prediction_evaluation/docs/e0_oracle_baseline.md`](../../../tracking_prediction_evaluation/docs/e0_oracle_baseline.md)에
기록한다. E1은 P1 이후로 연기한다.

### P1: 차량 lane-constrained predictor

- 객체 위치·yaw·이력으로 호환 lanelet 후보를 선택한다.
- 현재 lane의 following route와 routable left/right neighbour를 열거한다.
- 3초 horizon의 차선 중심/Frenet path를 생성한다.
- lane distance·yaw error·history consistency로 confidence를 정규화한다.
- 차선 밖, 저속, 매칭 불확실 객체는 현재 CTRV/CV로 fallback한다.
- 출력 contract는 기존과 동일한 0.1초 간격 `PredictedObjects`다.

이 단계만으로 직선·곡선·로터리에서 핵심 가설을 검증한다.

#### 2026-08-15 구현 현황

P1의 runtime prototype을 `perception_object_pipeline`에 구현했다.

- `map_based_prediction.py`: ROS message와 분리된 lane matching, routing graph,
  다중 forward route, centerline 경로와 confidence core
- `map_based_prediction_node.py`: `TrackedObjects -> PredictedObjects` ROS1 adapter
- `prediction_model:=ctrv|map_based`: 기존 CTRV를 보존하는 launch 선택 구조
- 차량만 map-based를 적용하고 pedestrian·obstacle·unknown은 CV를 유지한다.
- 저속, 3 m 밖, yaw 45도 초과, dead-end 길이 부족은 CTRV로 fallback한다.
- 현재 위치에서 centerline으로 1.5초 동안 수렴시켜 첫 sample의 순간이동을
  방지한다.
- v60 map hash를 node 시작 로그에 남긴다.

검증 결과는 다음과 같다.

- Python 단위 테스트와 기존 CTRV 회귀 테스트 통과
- catkin build 및 Detection -> Tracking -> CTRV 통합 rostest 통과
- v60 road lanelet 200개 중심 표본 중 199개에서 map path 생성(99.5%)
- 실제 ROS message smoke test에서 31 pose, 0.1초 간격, confidence 합 1.0 확인
- 20 vehicle core benchmark 평균 35.2 ms, p95 36.4 ms

아직 E0 bag 기준 CTRV 대비 ADE/FDE 개선을 검증하지 않았으므로 production
기본값은 계속 `ctrv`다. 현재 branch confidence는 지도 geometry와 lane matching
score 기반이며 실제 maneuver frequency로 calibration된 확률은 아니다. 객체별 lane
history와 lane-change intention은 P2 범위로 남긴다.

#### 2026-08-15 P1 E0 평가

raw `/Object_topic`은 `morai_map` 좌표이므로
`configs/tf/morai_lanelet_static_tf.yaml`의 고정 변환으로 Lanelet `map` 좌표에
정렬한 뒤 평가했다. 변환을 생략하면 v60 중심선과 약 20 m 어긋나 모든 표본이
CTRV fallback이 되므로, 평가 report에 transform 경로·map hash·mapped/fallback
origin 수를 기록한다.

3초 vehicle FDE는 다음과 같다.

| Bag | Model | mean | p50 | p90 | p95 |
|---|---|---:|---:|---:|---:|
| roundabout1 | CTRV | 2.245 | 1.263 | 6.795 | 7.847 |
| roundabout1 | Map P1 | 1.869 | 0.449 | 7.149 | 8.445 |
| roundabout2 | CTRV | 3.114 | 1.962 | 8.156 | 8.789 |
| roundabout2 | Map P1 | 2.840 | 1.518 | 8.244 | 8.797 |
| highway1 | CTRV | 2.136 | 1.201 | 5.770 | 8.459 |
| highway1 | Map P1 | 1.569 | 0.460 | 5.673 | 8.394 |

Map P1은 세 bag 모두 mean과 p50을 개선했다. Highway에서는 p90/p95도 소폭
개선했지만 roundabout에서는 p90/p95가 악화했다. 이는 단순 centerline 수렴이
다수 표본에는 유리하지만 잘못된 lane/branch 선택과 횡운동 모델 부재가 tail
error를 키운다는 증거다. 다음 predictor 개선은 자체 휴리스틱보다 Autoware의
Frenet polynomial path generator, object history와 maneuver prediction을 우선
포팅하고 동일 E0 표본으로 ablation한다.

### P2: History와 lane-change maneuver

- 객체별 lane association history를 유지한다.
- follow / left lane change / right lane change mode를 만든다.
- lateral displacement 또는 time-to-lane-change 조건을 비교한다.
- Frenet minimum-jerk lateral transition과 confidence calibration을 추가한다.

### P3: VRU와 crosswalk

- pedestrian·bicycle은 기본 CV와 reachable crosswalk 후보를 함께 생성한다.
- 신호등 integration은 traffic-light pipeline의 안정화 이후 선택적으로 연결한다.
- crossing intention과 stationary timeout은 실제 K-City rosbag으로 조정한다.

### P4: 성능 최적화와 production 전환

- Python prototype의 p95 latency가 허용 범위를 넘을 때만 C++로 이전한다.
- 기존 CTRV node와 map-based node를 launch parameter로 즉시 전환 가능하게 유지한다.
- planning topic과 message contract는 변경하지 않는다.

## 예상 효과와 한계

### 개선 가능성이 큰 구간

- 곡선과 로터리의 1.5~3.0초 lateral/FDE
- 교차로 branch에서 다중 경로 recall
- lane-boundary violation과 off-road rate
- 횡단보도 주변 VRU path coverage

### 개선이 작거나 악화할 수 있는 구간

- 단순 직선 도로는 CTRV/CV와 차이가 작다.
- 잘못된 yaw·velocity 또는 ID switch는 잘못된 lane matching을 만든다.
- branch의 top-1 path는 maneuver probability가 부정확하면 minFDE는 좋아도 악화할 수 있다.
- 정지·극저속 객체는 지도 기반 예측의 정보 이득이 작다.
- map geometry·projection·version 불일치는 prediction 전체를 체계적으로 틀리게 만든다.

따라서 수치 향상 폭은 새 evaluator로 측정하기 전에는 약속하지 않는다. 가장 현실적인 기대는 전체 평균의 일괄 상승보다, 곡선·로터리·분기에서 tail error와 off-road prediction을 줄이는 것이다.

## Tracker 후속 개선 후보

다음 tracker ablation은 E0/P1 완료 후 E1 단계에서 진행한다.

1. greedy association 대 Hungarian global assignment
2. center distance뿐 아니라 Mahalanobis, BEV IoU, class와 shape gate 결합
3. vehicle CTRV와 pedestrian CV의 class-aware process noise
4. track 생성·삭제·occlusion 파라미터 조정
5. timestamp age와 detector latency 보상
6. detector covariance를 사용할 수 있을 때 measurement covariance 연결

Map-based prediction 결과는 먼저 E0에서 비교하고, 이후 E1에서 tracker 입력 오차에
대한 강건성을 별도로 검증한다.

## 지도 버전 결정

삭제한 legacy launch는 v44/v45를 가리켰지만,
`origin/planning/sub/morai_validation_hj`의 competition route, planner와 MORAI
localization launch는 v60을 사용한다. Planning의 prediction 입력 contract는
여전히 `/perception/object_recognition/objects`의 `PredictedObjects`다. Prediction만
다른 map version을 사용하면 planning 경로와 lanelet ID가 어긋날 수 있으므로
신규 benchmark와 map-based predictor는 v60을 기준으로 한다.

Production 이전에 localization, prediction, planning의 기준 map을 하나로 통일한다.
초기 연구에서는 v60을 사용하고 결과 report에 map hash와 projector 설정을 기록한다.

## 구현 파일 구조 제안

```text
base_ws/src/tracking_prediction_evaluation/
  scripts/evaluate_oracle_prediction.py
  src/tracking_prediction_evaluation/
    trajectory.py
    models.py
    evaluator.py
  test/

base_ws/src/perception_object_pipeline/
  scripts/map_based_prediction_node.py
  config/map_based_prediction.yaml
  launch/object_prediction.launch
  test/test_map_based_prediction.py
```

평가 결과는 작은 `summary.json`, scene별 `scenes.jsonl`, 실패·outlier용 `failures.jsonl`로 나눈다. 큰 rosbag이나 전체 객체 dump를 report JSON에 중복 저장하지 않는다.

## 구현 승인 전 결정할 항목

1. 초기 map version은 v60을 사용한다.
2. 첫 benchmark는 기존 roundabout/highway bag의 E0이며 E1은 연기한다.
3. E0/P1 주 지표는 prediction 1·2·3초 top-1 FDE mean/p50/p90/p95다.
4. map-based 다중 경로는 top-1과 min-K를 동시에 사용한다.
5. 초기 포팅은 차량 P1까지만 구현하고 이득을 확인한 뒤 lane-change와 VRU로 확장한다.
