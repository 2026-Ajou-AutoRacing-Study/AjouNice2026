# AjouNice2026 활성 최우선 계획

마지막 갱신: 2026-09-22.

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
  sensor TF 중복 발행 금지. MRP 로컬 실행은 `localization_mrp`.

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
- 다음 gate는 최신 Pure Pursuit limiter를 포함한 LAN-to-LAN closed-loop와 GPS 음영
  복귀 검증이다. 그 전에는 quantization-aware update나 global/local 제어 상태 분리를
  추가하지 않는다.

최신 문제 정의와 검증 계약:
[`localization/260918_competition_localization_critical_issue_status.md`](localization/260918_competition_localization_critical_issue_status.md).
최신 폐루프 근거:
[`localization/260920_lightweight_closed_loop_evidence.md`](localization/260920_lightweight_closed_loop_evidence.md).

## P1. Base4-Temporal 고주로 Velocity Pilot

- 목적은 100 km/h 고주로 추월에 필요한 Vehicle velocity와 1/2/3초 미래경로를
  개선하는 것이다.
- Point-time Ego deskew의 geometry/latency P0는 완료했다. 48-scene 고속 연속속도
  파일럿도 raw와 scan-end deskew 두 파생 dataset으로 변환·검증했다. Deskew는 merge
  clip을 개선했지만 cruise 한 clip을 악화한 근거가 있으므로, 동일 학습 조건 A/B 전에는
  production 기본값을 `off`로 유지한다.
- Position-only tracker의 고주로 2D AVE는 약 `0.96 m/s`다. GT velocity oracle은
  `0.259 m/s`, stable 3초 FDE `0.594 m`를 보여 실제 velocity observation의 가치가
  충분하다.
- `B4T-HWY-P0` 수집, learnability gate, 수정 LR full train과 두 고주로 rosbag 평가는
  완료됐다. T1 E6는 V1의 큰 velocity bias를 줄였지만 인접 frame error-step p95가
  `3.81--4.22 m/s`로 커서 매 detection velocity EKF update는 실패했다. Low-LR
  continuation은 평균 speed MAE만 소폭 개선하고 jitter를 악화시켜 추가 epoch도 기각했다.
  다음은 동일분포 수집 증량이 아니라 temporal stability 구조 변경이다.
- 다음 통제 실험은 아래 네 조합이다.
  1. E25-7D production 기준선.
  2. raw B4H-V1 single-frame 9D `vx,vy` head.
  3. raw B4H-T1 causal 3-sweep 9D `vx,vy` head.
  4. scan-end Ego-deskew V1/T1 입력 계약 A/B.
- 새 데이터는 실제 LiDAR cadence의 연속 sequence로 수집한다. LiDAR timestamp에
  Ego/Object pose를 보간하고 학습 GT는
  `velocity_target_ego_mps=[vx,vy]` 하나만 노출한다.
- 과거 sweep은 Ego motion만 현재 frame에 정합한다. 추론에 존재하지 않는 GT Object
  motion compensation은 input에 사용하지 않는다.
- Architecture 순서는 single-frame 9D -> native 3-sweep -> lightweight temporal BEV
  -> 마지막에 StreamPETR-inspired query memory다. Sweep-separated BEV spatial gate인
  T2 P0는 구현 및 16-frame smoke를 통과했다. 아직 3회 LiDAR encoding을 사용하므로
  production 실시간 구조가 아니며, 동일 4-epoch T1/T2 비교에서 rosbag velocity jitter와
  downstream FDE가 개선될 때만 recurrent cache 최적화로 진행한다.
- Pilot dataset은 sequence 단위 train/val만 사용한다. 기존 고주로 합류·고주로1
  bag은 반복 사용된 dev 평가이며 최종 test가 아니다. 최종 후보는 새 held-out 고주로
  bag 최소 2개에서 한 번 평가한다.
- Tracker velocity mode 2는 noise `1.0`과 `4.0 m/s` 모두 기각했다. Mode 1
  (track 초기화만)은 두 bag에서 소폭 이득이 있었지만 production 기본은 계속 mode 0이다.
  geometry, association, temporal velocity jitter, 2D AVE와 Map-CV FDE를 함께 본다.

상세 실행 계획:
[`bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`](bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md).
최신 수집 결과:
[`../dataset/morai_od/docs/260922_base4_highspeed_jitter_pilot_results.md`](../dataset/morai_od/docs/260922_base4_highspeed_jitter_pilot_results.md).

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
