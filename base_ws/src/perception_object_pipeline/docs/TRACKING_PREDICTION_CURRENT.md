# Tracking·Prediction production 현재 상태

상태: **CURRENT**
마지막 문서·launch 대조: 2026-10-14. 아래 9월 bag 실측은 당시 이력이다.
현재 detector 선택은 [모델 CURRENT](../../../../docs/bevfusion/CURRENT.md#runtime)를 따른다.

## Production 계약

```text
BEVFusion Base5 E24 DetectedObjects
  -> EKF tracker
  -> UNKNOWN stationary policy
  -> Lanelet2 Map-CV predictor
  -> /perception/object_recognition/objects
```

운영 진입점은 `docs/common_alias.txt`의 `tracking`이다. detector와 localization은 별도
프로세스이며 동시에 다른 detector나 tracker를 같은 canonical topic에 발행하지 않는다.

| 구간 | 현재값 |
|---|---|
| Detection | `/perception/object_recognition/detection/objects` |
| Measurement track | `/perception/object_recognition/tracking/measurement_objects` |
| Motion-policy track | `/perception/object_recognition/tracking/objects` |
| Prediction | `/perception/object_recognition/objects` |
| World frame | `map` |

## Tracker

- Production ROS bridge는 tracker 입력 전에 같은 class의 rotated BEV IoU `0.50` NMS를
  적용한다. Barricade duplicate가 unmatched detection으로 남아 새 ID를 만드는 현상을
  막기 위한 production 기본값이며 tracker association gate 변경은 아니다.
- Confirmed-track 삭제는 detector frame 수가 아니라 message stamp 경과시간을 사용한다.
  moving track은 `0.50 s`, UNKNOWN 및 2 m/s 이하 CAR/TRUCK은 `1.00 s`까지 bounded
  coasting한다. Tentative track은 기존 strict 정책을 유지하며 pedestrian에는 긴 정적
  lifetime을 적용하지 않는다.
- 이미 mature track이 association된 frame에서는 중심 `0.75 m` 이내의 유사한 큰
  unmatched box가 두 번째 ID를 만들지 못하게 한다. 이는 detector NMS를 통과한
  low-IoU shape outlier에 대한 tracker-side 안전장치이며 전역 association gate `5 m`를
  넓힌 것이 아니다.
- 같은 E25 detection을 고정해 1배속 회귀한 결과, ID switch는 회전교차로
  `1 -> 0`, 고주로 합류 `0 -> 0`, 고주로 순항 `0 -> 0`이었다. 고주로 순항은
  coasting에 따른 FP가 `19 -> 26`으로 소폭 증가했으므로 false occupancy를 계속
  감시하되, 로터리·고주로에 대한 유의미한 ID 연속성 회귀는 확인되지 않았다.
- Pedestrian confirmed track은 정적·동적 모두 `0.50 s`만 coasting한다. 저속
  CAR/TRUCK/UNKNOWN용 `1.00 s` lifetime과 최소 크기 `1.0 m`인 duplicate-birth guard를
  pedestrian에 확대하지 않는다. 거리 변화 bag의 4명은 4개 ID로 안정적이었지만,
  횡단 bag의 1명은 3개 ID가 발생했다. 실제 횡단보도 planning 실패가 재현되기 전에는
  가까운 두 사람을 합칠 위험이 있는 pedestrian 전용 suppression을 추가하지 않는다.

- motion model: CTRV(`prediction_model=1`).
- detector position measurement noise: `0.40 m`.
- detector velocity fusion: `off`; production E24도 7D detector다.
- detector measurement timestamp state를 current time으로 강제 보상하지 않는다.
- EKF 내부 scheduler가 30 Hz로 prediction copy를 발행한다.
- production `tracking` alias의 maximum measurement age는 `0.50 s`다. generic
  `object_pipeline.launch`와 standalone `tracking_prediction.launch` 기본값 `0.25 s`는
  연구/직접 실행 호환값이며 운영 alias를 대체하지 않는다.
- Vehicle yaw는 1.0 m/s 이상에서 motion 방향과 canonicalize하고 enter/exit
  hysteresis는 `100/80 deg`다.
- 외부 fixed-rate adapter는 `0 Hz`, 즉 비활성이다.

0.50초는 detector frame 지연이나 간헐적 skip에서 track이 즉시 끊기지 않게 하는
운영 허용치다. 0.50초 동안 새 관측 없이 정확하다는 뜻이 아니므로 detector age와
prediction coverage를 함께 감시한다.

## Obstacle/UNKNOWN 정책

`force_unknown_stationary=true`다. 최신 관측은 pose와 box를 갱신하지만 tracker가 만든
가짜 속도·가속도·yaw rate는 prediction 전에 0으로 만든다. 따라서 정적 obstacle은
현재 위치가 반복되는 stationary path가 되고, Vehicle/Pedestrian은 이 정책의 영향을
받지 않는다.

대회에서는 `/Object_topic`이 없으므로 `od_gt`는 production fallback이 아니다. 로컬
부하 분리와 통제 실험에서만 detector replacement로 사용한다.

## Map-CV

- `tracking` alias가 사용하는 `object_pipeline.launch`의 map 기본값:
  `kcity_map/lanelet_map/lanelet2_map_v85.osm`. 실제 실행의 `map_file` override도 확인한다.
- horizon/time step: `3.0 s / 0.1 s`, 현재 pose 포함 31개 pose.
- minimum map-following speed: `0.5 m/s`.
- lane match: 거리 최대 `3.0 m`, yaw 오차 최대 `45 deg`.
- centerline lateral convergence: `1.5 s`.
- start lanelet 최대 3개, 최종 path 최대 6개.
- 같은 downstream Lanelet이며 horizon 전체가 `0.75 m` 이내인 중복 궤적은 하나의
  물리 hypothesis로 묶는다.
- confidence는 heuristic route score를 남은 hypothesis 사이에서 정규화한다. 실제
  calibrated probability로 해석하지 않는다.
- 저속, off-lane, wrong-direction, non-vehicle은 Cartesian CV fallback을 사용한다.

현재 predictor는 CTRV 곡률로 lane rank를 고르는 구조가 아니다. 현재 위치·yaw로
start Lanelet을 점수화하고 routing graph successor와 연결성을 따라 경로를 열거한다.
따라서 고주로 인접 차선 Top-1 의도 추정은 tracker history 없이 완전히 해결되지 않는다.

모든 planner가 Top-1만 쓰지는 않는다. 현재 통합 설정의 LC는 최고점 정확한 동점 전체,
Cruise inside/outside는 1개, 로터리는 confidence 0.05 이상 경로를 검사 대상으로 삼는다.
[49절](../../../../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-map-path-consumers-261014)의
시작 lanelet 이력은 진단 실행기에만 있고 발행 경로·confidence·production은 변경하지 않았다.

## 검증과 미구현 항목

모델 선택은 Detection만 보지 않고 같은 mission bag의 Tracking center/AVE, speed error,
Map-CV 1/2/3초 top-1 및 minFDE-K를 함께 본다. 최신 detector 및 tracker 근거는
[근거 인덱스](../../../../docs/bevfusion/README.md#evaluation)를 따른다.
과거 velocity A/B는 [9월 근거](../../../../docs/bevfusion/260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md)에서만 재현한다.

2026-10-12 원거리 실제 ROS 기억 A/B는 현재 bounded coasting을 유지하고 1/2초 확대를
채택하지 않았다. 자세한 분모·중복·잔류 제한은
[41절](../../../../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-far-bag-coasting-261012)을 따른다.
실제 planner의 기억이나 안전성을 이번 tracker 시험으로 인증하지 않는다.

2026-10-13 E24 고정 실제 ROS A/B에서 중복 birth guard OFF는 근접 대응 887/897을
늘리지 못하고 O02 전체 미대응을 531→748로 늘렸다. 빠른 차량 승격 ON은 R01 관측
1건만 개선해 기존 OFF를 유지한다. [bag·planning 검토 46절](../../../../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-tracker-planning-policy-review-261013)을 따른다.
30Hz tracking의 새 출력 stamp를 최신 detector 관측으로 해석하지 않는다. 설치 binary의
소스 반영 여부와 실제 관측 age를 별도로 확인한다.

다음은 현재 production 기본 경로로 채택하지 않은 후속 과제다.
일부 시험 코드·옵션이 존재하는 것과 production 활성화는 다르다.

[원거리 개선 계획](../../../../docs/bevfusion/261013_BASE5_RANGE_AND_TRACKING_IMPROVEMENT_PLAN.md)에 따라
저장 검출 다음에 실제 EKF의 독립 코어 복사본으로 낮은 score 갱신 12프로필을 비교했다.
이동·클래스 안정·XY 갱신 원형만 후보로 유지하며 [48절](../../../../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-low-score-core-probe-261013)에 효과와 한계를 기록했다.
설치된 코어·ROS node·launch는 미변경이고 production은 여전히 높은 score만 입력한다. 지도 보조는 미구현이다.
현재 Map-CV는 검출 삭제 필터가 아니며, 현재 Lane Change의 최고 confidence 동점 처리를 실제 실행값과 대조한다.
현재 코어의 association은 cost 정렬 후 greedy matching이다. Hungarian으로 설명하지 않는다.

- 기본 horizon 5초와 로터리 3~4초의 map-zone-aware 전환.
- tracker history를 이용한 temporal lane association과 Top-1 안정화.
- detector 9D velocity를 tracker update에 fusion.
- Planning의 추월 gap 판단을 위한 track-age/confidence gate.

위 항목은 localization 및 Base4 velocity pilot과 동시에 바꾸지 않는다. 기존 v60
PointPainting/Map-CV 수치와 초기 도입 계획은 [`legacy/`](legacy/)에만 보존한다.
