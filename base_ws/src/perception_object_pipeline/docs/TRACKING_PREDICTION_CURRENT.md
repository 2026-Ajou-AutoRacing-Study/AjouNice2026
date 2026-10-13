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
- 경로 arc index와 고정 지도 segment·bounded route geometry를 재사용하고 독립 메시지
  복사·동일 순서 DFS로 계산 비용을 줄인다. 6 bag의 10,433 tracking frame·11,021객체를
  두 번 비교해 직렬화 출력이 정확히 같다. 예측 정책 변경이 아니며 설치본은 아직 별도다.

현재 predictor는 CTRV 곡률로 lane rank를 고르는 구조가 아니다. 현재 위치·yaw로
start Lanelet을 점수화하고 routing graph successor와 연결성을 따라 경로를 열거한다.
따라서 고주로 인접 차선 Top-1 의도 추정은 tracker history 없이 완전히 해결되지 않는다.

모든 planner가 Top-1만 쓰지는 않는다. 현재 통합 설정의 LC는 최고점 정확한 동점 전체,
Cruise inside/outside는 1개, 로터리는 confidence 0.05 이상 경로를 검사 대상으로 삼는다.
[49절](../../../../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-map-path-consumers-261014)의
시작 lanelet 이력은 진단 실행기에만 있고 발행 경로·confidence·production은 변경하지 않았다.
추가 관측 적합도·횡이동 후보는 [50절](../../../../docs/bevfusion/261010_BASE5_INPUT_POOL_OPTIMIZATION_EVIDENCE.md#base5-map-cv-candidates-261014)의
별도 bag/30Hz 회귀까지 확인했다. 소수 큰 순위 악화 때문에 운영 정책에는 적용하지 않았다.
[오프라인 검토](../../../../docs/bevfusion/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#cpu)의
계산 최적화는 소스에 반영했지만 실행 중 프로세스나 팀장님 배포본은 교체하지 않았다.

## 최신 팀장님 풀주행 확인 (2026-10-14)

`full_baseline_00_sample_20261014_012302_01`의 bag·영상·시나리오·실행 파라미터를
`rosbags/eval/perception/261014_teammate_fullstack/`에 복사했다. Music 원본은 보존했고
bag SHA256은 `15698fa7c6ec264775c3b0e004980ad46a70cdf057f15e344416a7fd06dfb4d8`이다.
개별 근거는 `planner_logs/analysis/full_baseline_00_sample_20261014_012302_01/`의
`audit_v3/audit.json`, `map_v4/geometry_and_node_cost.json`,
`map_v1/frozen_history_candidate.json`에 둔다. 학습·생성기·운영 정책은 변경하지 않았다.

- 제어 시작/종료 events 사이 약 162.3초에 완주했고, 저장된 CollisionData의 접촉 목록은
  비어 있다. 해당 구간의 detection은 9.93 Hz, tracking/prediction은 30.00 Hz,
  최종 trajectory는 9.80 Hz이며 이 토픽들에 0.25초 초과 수신 공백은 없다.
- 저장된 실행 파라미터에서 LC `use_all_predicted_paths=false`, Cruise outside 1개,
  로터리 confidence 0.05를 확인했다. 정확한 최고점 동점은 0건이다. 이는 실행 설정 확인이며,
  모듈이 실제 선택한 object/path index를 기록한 것은 아니다. Top-2가 RViz에 보이는 것과
  planner가 그것을 소비하는 것은 여전히 구분한다.
- Scalar/arc-index를 전체 5,461 tracking frame에서 비교해 ROS payload가 정확히 같았다.
  기록본과 대응 가능한 5,417 prediction payload도 publisher별 `header.seq`만 정규화하면
  정확히 같다. 소스가 최적화된 버전으로 배포됐음을 이 사실만으로 추정하지 않는다.
- 로컬 CPU의 arc-index 기준 이후 추가 최적화는 평균 `2.160 -> 0.982 ms`, p95
  `6.002 -> 2.576 ms`다. 팀장님 PC의 live/LAN 전체 지연이 아니다. 별도 후보 프로세스
  반복의 최대는 86.976ms이며 GC가 85.285ms를 차지했다. 최대 지연 해결이나 GC OFF를
  주장하지 않는다. 소스·지도·bag 해시와 분모는 [근거](../../../../docs/bevfusion/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#cpu)에 둔다.
- 같은 Man1에 낮은-IoU 중복 UUID가 붙은 50/4,870프레임과 5개 ID 분절을 확인했다.
  원본 코어에서 중복 검출 birth→관측 경쟁→기존 ID coasting 만료를 재현했다. 가까운 두
  사람을 합칠 반례 때문에 threshold/guard를 강화하지 않는다. association 후보는
  [별도 검토](../../../../docs/bevfusion/261014_OFFLINE_SENSOR_TRACKING_NODE_REVIEW.md#tracking)이며 production은 미변경이다.
- 고정한 lane-history 후보는 3초 조건부 위치 오차 평균 `4.625 -> 4.623 m`로 거의
  같고, 변경된 30표본 중 5m 초과 개선 12·악화 9였다. 새 bag으로 재튜닝하지 않았으며
  기존 운영 미적용 결론을 유지한다. 원거리 저점수 갱신 후보는 raw 센서/저점수 검출이
  없어서 이 bag만으로 검증할 수 없다.
- 주행 중 trajectory validator 1,589건 중 7건이 invalid였다(최대 감속 4, 전방 길이 3).
  주행 중 latency invalid는 0건이고, 시작 전 초기 궤적에서 1건 있었다. MPT 실패 진단도
  있으므로 완주를 모든 중간 계산의 정상 판정으로 해석하지 않는다. 음영 구간에 배치
  객체가 없는 이 주행은 그 구간의 객체 상호작용을 검증하지 않는다.

다음 latency 관측에는 `/perception/base5_trial/status`와
`/perception/base5_trial/latency_ms`도 필요하다. Base5 trial launch가 canonical 상태
토픽을 이 namespace로 remap하지만 이번 녹화는 canonical 이름만 요청해 둘 다 빠졌다.
Detection header→bag 수신 age 평균 237 ms는 scan 누적·전처리·큐·추론 등을 포함할 수
있으며, 순수 GPU 추론 시간으로 부르거나 고정값을 빼서 보정하지 않는다. 정확한
Map-CV 입력/출력 stamp·frame·UUID 대응 4,870건의 수신 차이는 평균 3.91 ms지만,
subscriber별 녹화 순서 차이로 음수 7건도 있어 callback CPU와 같지 않다.

추가 녹화는 [노드 지연 실행기](../../../../scripts/README.md#후단과-노드-지연-분석)를
따른다. Map-CV/UNKNOWN callback 진단은 `publish_processing_diagnostics:=true`로 선택하며
기본 OFF다. 기존 validator는 수정하지 않았고 디버그를 자동 활성화하지도 않는다.
C++ 포팅은 lane projection/route geometry 등 CPU 비용이 큰 부분부터 검토하되,
동일 입력에서 UUID·class·shape·시각·경로 순서·confidence·31개 pose 동등성,
empty/저속/off-lane fallback, 전체 콜백의 p95/p99·최대 지연·관측 age·queue/drop을
확인한다. 이미 작은 직렬화 비용만 줄이는 작업보다 센서 누적과 큐, planner 개별
processing-time 진단을 먼저 분리한다.

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
