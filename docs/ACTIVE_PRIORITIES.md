# AjouNice2026 활성 최우선 계획

마지막 갱신: 2026-09-23.

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

현재 production 계약과 개선 순서:
[`localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md`](localization/COMPETITION_LOCALIZATION_CURRENT_STATE.md).
누적 문제 근거:
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
  -> 마지막에 StreamPETR-inspired query memory다. Sweep-separated softmax gate T2는
  current sweep에 거의 고정되고 T1보다 속도가 나빠 기각했다. Ordered BEV delta를
  current BEV에 더하는 T2R E1은 두 dev bag에서 T1 대비 detector speed MAE,
  error-step p95, position-only tracking AVE와 Map-CV FDE를 모두 개선했다. 다만 3회
  LiDAR encoding을 사용하고 held-out 검증이 없으므로 production은 E25/mode 0을 유지한다.
  5060 Ti의 보고 latency도 mean `128 ms`, p95 `170 ms`로 12 Hz LiDAR 추종에 부족했다.
  Recurrent cached-BEV P0는 encoder를 frame당 한 번만 실행해 mean `80.3 ms`, p95
  `87.1 ms`로 12 Hz gate에 도달했지만, 기존 T2R checkpoint를 그대로 적용하면
  train/runtime temporal 의미 불일치로 speed MAE가 `2.873 -> 3.701 m/s`로 악화했다.
  source-frame encode + feature-warp로 학습 의미를 맞춘 T2C도 cached merge bag에서
  mean `79.1 ms`, p95 `84.6 ms`로 실시간 gate와 spatial AP는 통과했지만 speed MAE
  `3.627 m/s`, error-step p95 `4.599 m/s`로 T2R보다 나빠 기각했다. Cache/Ego-warp
  인프라는 보존하되 다음 architecture gate는 object/query memory와 명시적
  delta-time·displacement supervision이다. 최종 후보 판정에는 새 고주로 bag이 필요하다.
- Annotation learnability audit에서 GT center 3-frame (`~0.42 s`) displacement oracle의
  val speed MAE는 `0.033 m/s`로, sequence 자체에 속도 정보가 있고 label/time
  계약도 정상임을 확인했다. 반면 159 scene 중 scene 내 speed span이
  `0.5 m/s`를 넘는 것은 11개뿐이다. 64-sample aggressive overfit에서 T1과
  single-frame V1이 모두 `0.5 m/s` 아래로 내려가므로 head/loss의 표현
  불능은 아니지만, 현재 overfit은 temporal motion보다 scene prior 암기를 포함한다.
  따라서 다음은 동일 외형·위치의 속도만 바꾸는 counterfactual 수집과
  `0.3--0.6 s` object/query memory + displacement consistency를 같이 검증한다.
- 기존 detection bag의 predicted center를 0.45초 causal association한 probe에서 speed
  MAE는 Production E25 `0.29/0.64 m/s`, V1 `0.38/0.67 m/s`, T1
  `0.52/0.71 m/s`(고주로1/합류)였다.
  같은 pair의 velocity head `2.53--4.23 m/s`보다 크게 낮고, GT-ID 대비
  causal association 추가 손실도 `0.00--0.09 m/s`였다. 이후 `B4H-Q0` center-motion을
  실제 tracker와 Map-CV까지 구현·검증했지만 accepted slice에서도 기존 EKF보다
  speed/종방향 오차가 악화되어 production 후보에서 제외했다. Quality gate와 진단
  토픽만 연구 도구로 유지한다. 다음 architecture gate는 Vehicle query memory,
  명시적 delta-time과 displacement consistency를 결합한 `B4H-Q1`이다. 현재 bag의
  identity switch 0건은 쉽게 분리된 차량 분포의 결과이므로 새 다중차량 held-out gate가
  필수다.
- `B4H-Q1-P0` sparse Vehicle query-memory는 tensor/metadata/full pipeline smoke와
  matched A/B를 통과했다. Hardcase V2 scene-disjoint best speed MAE는 memory-off
  `4.075` 대비 memory-on `3.854 m/s`로 `5.4%` 개선됐지만 `<1.0 m/s` gate에는
  실패했다. 이어서 P1은 motion을 attention logit뿐 아니라 value에 직접 넣고 temporal
  velocity auxiliary head를 추가해 best speed MAE를 `3.133 m/s`까지 낮췄다. P0
  memory-on 대비 `18.7%` 개선이고 공간 mAP은 보존됐다. 이후 P2 감사에서 현재 frame의
  decoder 전 center와 memory의 decoder 후 center를 비교해 평균 `~0.48 m`의 decoder
  correction이 가짜 motion으로 들어가던 문제를 찾았다. 양쪽을 post-decoder center로
  통일하고 temporal velocity fusion을 90%로 둔 결과 hardcase V2 scene-disjoint speed
  MAE가 `3.133 → 1.070 m/s`로 `65.8%` 개선됐고 공간 mAP도 보존됐다. 99% 강제에 가까운
  fusion은 exact overfit에서 악화되어 기각했다. `<1.0 m/s` gate를 근소하게 넘었으므로
  production 적용 없이 memory-active/fallback, jitter, 개발 rosbag과 latency를 다음
  gate로 둔다.
- Q1-P2 gate 90%를 실제 Docker online context로 두 개발 bag에 적용했다. speed MAE는
  T1 대비 합류 `3.419 → 1.277 m/s`, highway1 `2.589 → 2.288 m/s`로 개선됐지만,
  highway1 adjacent error-step p95가 `8.351 m/s`로 악화했다. 5060 Ti mean latency도
  `93.5--95.3 ms`로 12 Hz budget을 넘었다. 평균 성능만으로 detector velocity fusion을
  열지 않으며, 다음은 다중-frame robust motion으로 query switch/jitter를 줄이는 P3다.
- Q1-P3는 최대 4개 memory frame의 attention-weighted center displacement를 직접 velocity
  기준값으로 사용하고 learned residual만 보정한다. Hardcase V2 scene-disjoint E1에서
  speed MAE `0.845 m/s`, center mAP `0.9106`으로 첫 `<1.0 m/s` gate를 통과했다. 개발
  bag에서도 highway1 `1.582 m/s`, merge `1.021 m/s`까지 개선됐고 error-step p95도
  P2 대비 각각 `8.351 → 5.569`, `4.030 → 3.367 m/s`로 줄었다. 다만 highway1 jitter는
  T1 `3.810 m/s`보다 크고 latency는 `92.8--95.2 ms`다. P3를 다음 연구 baseline으로
  삼되 production/fusion은 보류하고 robust association outlier gate를 P4로 진행한다.
- Pilot dataset은 sequence 단위 train/val만 사용한다. 기존 고주로 합류·고주로1
  bag은 반복 사용된 dev 평가이며 최종 test가 아니다. 최종 후보는 새 held-out 고주로
  bag 최소 2개에서 한 번 평가한다.
- Tracker velocity mode 2는 noise `1.0`과 `4.0 m/s` 모두 기각했다. Mode 1
  (track 초기화만)은 두 bag에서 소폭 이득이 있었지만 production 기본은 계속 mode 0이다.
  geometry, association, temporal velocity jitter, 2D AVE와 Map-CV FDE를 함께 본다.

상세 실행 계획:
[`bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md`](bevfusion/260920_BASE4_TEMPORAL_HIGHWAY_PILOT_PLAN.md).
Q1 실행 순서와 중단 기준:
[`bevfusion/260923_BASE4_Q1_EXECUTION_ROADMAP.md`](bevfusion/260923_BASE4_Q1_EXECUTION_ROADMAP.md).
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
