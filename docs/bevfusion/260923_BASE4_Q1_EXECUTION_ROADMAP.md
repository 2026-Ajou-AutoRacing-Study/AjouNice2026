# Base4 Q1 실행 로드맵

> 날짜: 2026-09-23.
> 상태: Q1-P1 motion-value gate까지 완료한 CURRENT 계획.
> 목표: 고주로 Vehicle velocity와 downstream 추월 판단을 개선하되 E25 production을
> 흔들지 않는다.

## 현재 결론

- Production은 `B3-BF0-F40 E25`, 7D position-only tracker 입력을 유지한다.
- V1/T1의 9D velocity head는 학습은 가능하지만 rosbag 일반화와 frame 안정성이 부족하다.
- T2/T2R/T2C dense BEV 계열은 정확도 또는 latency gate를 통과하지 못했다.
- Q0 causal center-motion 직접 출력은 accepted slice에서도 종방향 오차를 악화시켜
  기각했다.
- 다음 유효 가설은 같은 객체의 이력을 보존하는 sparse object/query memory다.

## 장기 실행 단계

### Phase A — 평가 계약 동결

기존 `highway1`과 `highway_merge`는 개발 bag으로만 사용한다. 모델 선택 전에 다음 두
평가군을 분리한다.

1. 개발군: 기존 두 bag. 빠른 회귀와 원인 분석용.
2. 최종 held-out군: 다중차량, 인접·후측방 차량, 합류, 추월, 가감속을 포함한 새 고주로
   bag 최소 2개. 모델과 threshold를 확정한 뒤 한 번만 평가한다.

학습 dataset은 같은 asset·거리·차선·현재 pose에서 NPC speed만 달라지는
counterfactual scene을 포함해야 한다. 일정 속도만 늘리지 않고 sequence 내부 가속·감속,
Ego/NPC 속도 독립 조합, 인접 차선 후측방 차량을 넣는다. Split은 frame이 아닌 scene
단위다.

### Phase B — Q1-P0 구조 검증

StreamPETR 전체를 포팅하지 않는다. 기존 TransFusion을 유지하고 Vehicle query에만 다음
최소 memory를 붙인다.

- current query feature, center, score와 timestamp를 노출한다.
- 과거 3~4 frame, 총 `0.3~0.6 s` memory를 보존한다.
- Ego pose로 과거 reference center를 current LiDAR frame에 정합한다.
- 거리·class·age gate 뒤 current query와 memory query 사이 cross-attention을 한 층만 둔다.
- 첫 구현은 memory를 detach하여 긴 BPTT와 VRAM 증가를 막는다.
- scene 변경, timestamp 역행, 큰 frame gap과 teleport에서 즉시 reset한다.
- 기존 9D velocity loss에 GT center displacement consistency 보조 loss를 추가한다.

GT identity는 학습 target 구성과 진단에만 사용하고 inference association에는 사용하지
않는다. Q0처럼 계산한 velocity를 최종 출력에 직접 덮어쓰지 않는다.

### Phase C — 단계별 학습 gate

1. deterministic tensor/unit test: Ego warp, dt, reset, mask와 gradient 확인.
2. 64-sample exact overfit: wiring과 optimization만 확인. 일반화 주장 금지.
3. counterfactual mini split: 같은 외형/위치의 다른 속도를 구분하는지 V1/T1과 비교.
4. full train/scene-disjoint val: spatial과 velocity를 같이 비교.
5. 기존 두 개발 bag: detection, temporal jitter, tracking, Map-CV end-to-end 평가.
6. 모든 설정을 고정한 뒤 새 held-out bag을 한 번 평가한다.

첫 현실적 통과 기준은 다음과 같다.

- scene-disjoint val Vehicle speed MAE `< 1.0 m/s`.
- adjacent-frame velocity error-step p95가 T1 대비 명확히 감소.
- center mAP/mATE가 대응 baseline 대비 5% 이상 악화하지 않음.
- 개발 bag에서 tracker 2D AVE와 Map-CV FDE@1/2/3s가 동시에 개선.
- 12 Hz 입력 기준 mean latency `< 83 ms`; 최종 production latency는 4090에서 재측정.

`0.5 m/s`는 최종 공격 목표이며 P0의 즉시 합격선으로 사용하지 않는다.

### Phase D — Tracker와 Prediction 통합

Q1 detector가 Phase C를 통과한 뒤에만 detector velocity fusion을 다시 연다.

- 매 frame 강제 fusion 대신 track age, detector velocity covariance와 innovation gate를 쓴다.
- position-only EKF를 항상 fallback으로 유지한다.
- tracker AVE와 speed가 개선된 뒤 Map-CV horizon 5초 및 temporal lane Top-1을 검토한다.
- Planning 추월 gap에는 track age, lane occupancy 안정 시간과 prediction confidence 계약을
  둔다.

Q1과 tracker/prediction 변경을 한 실험에서 동시에 튜닝하지 않는다.

## 즉시 수행할 작업

다음 작업 묶음은 학습을 오래 돌리기 전에 끝낸다.

1. TransFusion query 입출력과 checkpoint key를 고정하는 Q1-P0 wrapper 설계.
2. memory transform/reset/attention의 CPU tensor 단위시험 구현.
3. 기존 Base4 clip metadata가 Q1의 sequence ID, timestamp, Ego pose를 모두 제공하는지
   audit하고 누락 시 info builder만 보강.
4. 64-sample 및 counterfactual mini recipe 생성.
5. forward/backward smoke 한 번 후 parameter count, VRAM과 latency를 기록.

이 단계 전에는 대규모 재수집이나 장시간 full training을 시작하지 않는다.

## 2026-09-23 진행 상태

- sparse Vehicle query-memory, Ego pose 정합, mask와 reset 구현 완료.
- tensor/gradient 단위시험 및 실제 Base4 metadata audit 통과.
- batch 1, scene-order sampler, identity global augmentation 전용 config 구현 완료.
- 실제 4-camera+LiDAR 4-frame full forward/backward/checkpoint smoke 통과.
- memory 사용률, 평균 age와 residual gate를 W&B epoch metric으로 추가.
- 첫 E4는 velocity loss/LR control이 기존 learnability 계약과 달라 무효 처리했다.
- velocity weight 1.0, velocity/memory branch 5x LR의 64-sample E12에서 memory-on이
  memory-off보다 speed MAE를 `3.9%` 낮췄다.
- hardcase V2 scene-disjoint E4 matched A/B에서도 best speed MAE가
  `4.075 → 3.854 m/s`로 `5.4%` 개선됐고 공간 mAP은 보존됐다.
- query memory 신호는 확인했지만 `<1.0 m/s` 절대 gate에는 실패했다. P0 checkpoint는
  rosbag/production 후보가 아니며 P1 displacement auxiliary supervision으로 진행한다.
- P1에서 `(delta center / delta-time, age)`를 memory value에 명시적으로 넣고 temporal
  velocity auxiliary head를 추가했다. Hardcase V2 scene-disjoint best speed MAE는
  `3.854 → 3.133 m/s`로 P0보다 `18.7%` 개선됐고 공간 mAP도 보존됐다.
- 다만 `<1.0 m/s` gate에는 여전히 실패했다. P1 checkpoint도 rosbag/production 후보가
  아니며, 다음은 GT identity를 학습/진단에만 사용하는 실제 displacement-consistency
  target과 counterfactual mini split이다.

구현 및 결과 문서는
[`260923_BASE4_Q1_P0_QUERY_MEMORY_RESULTS.md`](260923_BASE4_Q1_P0_QUERY_MEMORY_RESULTS.md)를
따른다. P1 구현 및 결과는
[`260923_BASE4_Q1_P1_MOTION_VALUE_RESULTS.md`](260923_BASE4_Q1_P1_MOTION_VALUE_RESULTS.md)를
따른다. P1의 auxiliary target은 기존 velocity annotation이며 GT identity 기반
displacement-consistency loss는 아직 구현하지 않았다.

## 중단·전환 기준

- exact overfit도 실패: implementation/loss 연결 문제부터 수정한다.
- exact overfit은 성공하지만 counterfactual mini split에서 V1과 차이가 없음: query
  memory가 motion identity를 쓰지 못하는 것이므로 full train을 중단한다.
- val은 좋아도 개발 rosbag jitter/AVE가 악화: loss calibration 또는 online memory
  계약 문제로 분리하고 production 통합을 중단한다.
- Q1도 실패: architecture를 더 키우기 전에 새 실제 주행형 dataset의 motion diversity와
  timestamp/annotation 계약을 다시 감사한다.

## 병렬 우선순위

Perception Q1과 별개로 localization P0는 계속 더 높은 대회 안전 우선순위다. 최신
Pure Pursuit limiter를 포함한 LAN-to-LAN closed-loop와 GPS 음영 복귀 검증을 먼저
통과해야 한다. Localization 파라미터 변경과 Q1 detector 비교를 같은 주행에서 동시에
바꾸지 않는다.
