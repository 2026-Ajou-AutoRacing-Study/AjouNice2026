# AjouNice2026 활성 최우선 계획

마지막 갱신: 2026-10-05.

이 문서는 **아직 끝나지 않았고 다음 작업에서 반드시 다시 확인할 항목만** 둔다.
완료된 수집·학습·실험 수치는 각 CURRENT/EVIDENCE 문서에 보존하며 여기로 다시
복사하지 않는다.

## 현재 production 기준

- Detector: `B3-BF0-F40 E25`, threshold Vehicle/Pedestrian/Obstacle
  `0.40/0.15/0.10`.
- Tracker: CTRV, position measurement noise `0.40 m`.
- E25는 7D detector이므로 detector velocity fusion은 기본 `off`.
- Prediction: Lanelet Map-CV, 현재 3초 horizon.
- Localization production: `/Competition_topic`, heading correction `-1.347 deg`,
  sensor TF 중복 발행 금지. Planning/TF는 Global state, controller는 Smooth
  `/localization/control_kinematic_state`를 사용한다. MRP는 `localization_mrp`.

실행 계약은
[`bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md`](bevfusion/BASE3_BEVFUSION_PRODUCTION_HANDOFF.md)와
[`localization/README.md`](localization/README.md)를 우선한다.

## P0. 대회 측위 종·횡 안정화와 제어 채터링 제거

- 현장점검에서 fusion localization의 steering-rate absolute p95가 약 `1.59 rad/s`,
  GT localization은 약 `0.34 rad/s`였다. 시각화 문제가 아니라 제어 안정성 문제다.
- 정상 구간 횡 채터링은 약 `14.8/18.5 cm`로 양자화된 GNSS update를 Global EKF가
  강하게 따라가고 controller가 Global state를 직접 소비하는 구조가 주원인이다.
- 로컬 한-PC full-stack의 velocity 과적분은 production 속도 bias로 취급하지 않는다.
  현장 bag의 이동거리/속도 적분 비율은 정상이며, 로컬 MORAI 실시간율과 receipt-time
  artifact를 분리해야 한다.
- 경량 GT-object 폐루프에서 production Q/R은 localization `50 Hz`, 횡 frame-jump
  p95 약 `1.31 cm`, steering-rate p95 약 `0.54 rad/s`를 유지했다. 이 test bed 확보는
  완료됐고 `sigma=0.05, Qxy=0.05` preset은 기각한다.
- Global/Local state 분리와 Smooth controller 입력은 경량 전체 경로 폐루프 A/B를
  통과해 production에 반영했다. Quantization-aware 후보는 동일 기록 controller
  A/B에서 Smooth보다 우위가 없었고, 폐루프 승격 전이므로 기본 OFF다.
- 공식 START 고정 위치 + 보정된 Competition heading 초기화는 MR 로컬 대조를
  통과했다. 대회 LAN 출발·GPS 음영 복귀·checkpoint 순간이동 검증은 남아 있다.
  공식 START는 그때까지 opt-in이며 개발 기본 초기 위치는 `/Ego_topic`이다.

현재 production 계약과 개선 순서:
[`localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md`](localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md).
누적 문제 근거:
[`localization/260918_competition_localization_critical_issue_status.md`](localization/260918_competition_localization_critical_issue_status.md).
최신 폐루프 근거:
[`localization/260920_lightweight_closed_loop_evidence.md`](localization/260920_lightweight_closed_loop_evidence.md).

## P1. 근접 장애물·원거리 차량의 빠른 인지 전달

- Base5 E2 학습 및 [E1 보행자·장애물9-bag A/B](bevfusion/261005_BASE5_E1_MISSION_ROSBAG_AB.md)는 완료됐다. E1 장애물 소폭 개선에도 0–2m 장애물·20–30m 보행자 공백은 미해결이다. 다음은 해당 거리·부분 시야 표적 보강 + replay다. Production E25 유지, Far ROI와 고해상도는 별도 A/B로 유지한다.
- 전방은 빠른 존재 전달을, 후방/로터리는 위치·속도·궤적 안정성을 함께 평가한다.
- 현재 E25의 근접 장애물 누락은 tracker만으로 해결하지 않는다. 부분 시야·얇은
  방향·통과 중 raw/GT를 보강하고 동일 held-out bag에서 재학습 후보를 비교한다.
- 원거리 검증은 40/60/80/100m 가시성부터 시작한다. 현 ±30m config와 향후 모델의
  능력을 구분하며, ROI/학습/평가 범위를 함께 설계한다.
- 고해상도 320×896은 5060 Ti GPU smoke를 통과했다. 정확도·4090 속도는 미측정이다.
- 전후방 ±60m/좌우 ±30m ROI 후보는 축소 입력 forward와 checkpoint 기하 복원 검증을
  통과했다. 확장 GT 재변환·full-input 학습·출력/평가 필터 연계는 아직 미완료다.
- confirmed UNKNOWN→차량 빠른 승격은 opt-in 후보이며 기본 OFF다. 신규 track
  확인을 무조건 생략하거나 검출 없는 객체를 차량으로 만드는 기능은 아니다.
- Production E25 교체는 Detection/Tracking/Map-CV 동일 bag 회귀 검증 후 결정한다.

최신 근거·실행 경계:
[`bevfusion/261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md`](bevfusion/261004_PERCEPTION_RANGE_NEARFIELD_AND_PROMOTION_PILOT.md).

## 후속 연구. Base4/Q1 velocity·temporal

- 목적은 100 km/h 고주로 추월에 필요한 Vehicle velocity와 1/2/3초 미래경로를
  개선하는 것이다.
- 과거 개발 bag의 Position-only/GT oracle 숫자를 현재 production 성능으로 재사용하지
  않는다. 최신 팀원 bag의 보정 GT 비교는 BEVFusion index의 2026-10-04 EVIDENCE를 따른다.
- 이미 수행한 `B4T-HWY-P0` 및 Q1 이력을 참고하되, 새 근거 없이 초기 pilot을 자동 재개하지 않는다.
- 새 데이터는 실제 LiDAR cadence의 연속 sequence로 수집한다. LiDAR timestamp에
  Ego/Object pose를 보간하고 학습 GT는
  `velocity_target_ego_mps=[vx,vy]` 하나만 노출한다.
- 과거 sweep은 Ego motion만 현재 frame에 정합한다. 추론에 존재하지 않는 GT Object
  motion compensation은 input에 사용하지 않는다.
- single-frame, 3-sweep, dense temporal BEV, query memory는 이미 실험 이력이 있다.
  다음 architecture는 Q1 결과와 새 bag의 실패 분해를 보고 선택한다.
- Pilot dataset은 sequence 단위 train/val만 사용한다. 기존 고주로 합류·고주로1
  bag은 반복 사용된 dev 평가이며 최종 test가 아니다. 최종 후보는 새 held-out 고주로
  bag 최소 2개에서 한 번 평가한다.
- 각 모델은 tracker velocity `off/on`을 모두 평가한다. 첫 velocity covariance는
  `0.5 m/s`이며 geometry, association, 2D AVE와 Map-CV FDE를 함께 본다.

실험 이력:
[`bevfusion/260923_BASE4_Q1_EXECUTION_ROADMAP.md`](bevfusion/260923_BASE4_Q1_EXECUTION_ROADMAP.md).

## P2. Localization과 velocity pilot 이후 Prediction 개선

현재 구현값과 미구현 경계는
[`../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md`](../base_ws/src/perception_object_pipeline/docs/TRACKING_PREDICTION_CURRENT.md)를
따른다.

- 기본 prediction horizon을 5초로 확장하되 로터리에서는 3--4초로 줄이는
  map-zone-aware 정책을 검토한다.
- 고주로 추월을 방해하는 다중 후보 문제는 단순 `top_k=1` 강제가 아니라 tracker
  history, Lanelet 연결성, 횡위치와 차선 변경 가능성을 결합한 temporal lane
  association으로 Top-1 정확도를 먼저 높인다.
- 같은 차량의 인접 차선 점유 판단이 일정 시간 안정적으로 유지될 때만 Planning이
  추월 gap으로 사용하도록 track age와 prediction confidence 계약을 정한다.
- Localization과 detector velocity 변경을 동시에 섞지 않는다. P0와 P1의 동일 bag
  회귀 검증이 각각 끝난 뒤 별도 브랜치에서 수행한다.

## 완료되어 자동 재개하지 않는 항목

- Base3-Static 10K 수집, validator 소급 감사와 5-point GT 정책.
- Base3 LRASPP, PointPainting PP0/PP1과 FastBEV 연구.
- Base3 BEVFusion E40 학습과 production E25 선택.
- Base3-Online 고주로 334-scene 수집과 E25 fine-tuning.
- Tracker motion-model/covariance sweep과 2D AVE evaluator 수정.

과거 결과가 필요하면 [`bevfusion/README.md`](bevfusion/README.md),
[`../dataset/morai_od/docs/README.md`](../dataset/morai_od/docs/README.md) 및 각
`legacy/README.md`에서 찾는다. 새 근거 없이 완료 항목을 최우선 계획으로 되돌리지 않는다.
