# Base4 Q1 실행 로드맵

> 날짜: 2026-09-24.
> 상태: Q1-P9 uncertainty-aware velocity fusion gate까지 완료한 CURRENT 계획.
> 목표: 고주로 Vehicle velocity와 downstream 추월 판단을 개선하되 E25 production을
> 흔들지 않는다.

## 현재 결론

- Production은 `B3-BF0-F40 E25`, 7D position-only tracker mode 0을 유지한다.
- V1/T1의 9D velocity head는 학습은 가능하지만 rosbag 일반화와 frame 안정성이 부족하다.
- T2/T2R/T2C dense BEV 계열은 정확도 또는 latency gate를 통과하지 못했다.
- Q0 causal center-motion 직접 출력은 accepted slice에서도 종방향 오차를 악화시켜
  기각했다.
- Q1 sparse query memory의 detector baseline은 P7 free-intercept다.
- P8에서 detector velocity의 누락됐던 LiDAR→map 회전을 수정했다.
- 초기 종방향 output-only 결합은 speed/AVE를 소폭 개선했지만 모든 Map-CV horizon을
  동시에 개선하지 못해 production에는 올리지 않는다.

## P0--P9 실험 ledger

| 단계 | 핵심 가설·변경 | 대표 결과 | 판정 |
| --- | --- | --- | --- |
| P0 | Ego-warped sparse Vehicle query memory | val speed `4.075→3.854` | 신호 확인, 절대오차 실패 |
| P1 | displacement/dt를 value에 넣고 auxiliary velocity head 추가 | `3.854→3.133` | 개선, center 기준 오류 발견 |
| P2 | current와 memory center를 모두 post-decoder로 통일 | `3.133→1.070` | 구조 오류 수정, `<1` 근소 실패 |
| P3 | 여러 causal pair의 direct motion aggregation | val `0.845`, 첫 `<1` | detector baseline 채택 |
| P4 | mutual matching과 hard robust consensus | `1.274~1.998` | 기각: 유효 pair 과도 제거 |
| P5 | robust 후보 soft blend | `0.845→0.847~0.851` | 기각: 단조 악화 |
| P6 | GT-ID 진단 후 `dt²` age-weighted slope | val `0.738`, highway1 `0.872` | center jitter 병목 완화 |
| P7 | intercept와 slope 동시 적합 | val `0.676`, bags `0.811/0.779` | detector 연구 baseline 갱신 |
| P8 | velocity frame 수정 + 초기 종방향 output-only | bags AVE `0.589/1.009` | 가능성 확인, Map-CV gate 미통과 |
| P9 | detector covariance 전달 + uncertainty-aware output | 초기 speed/all-FDE 일부 개선 | stable FDE 불일치, continuous 결합 기각 |

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

## 다음 수행 gate

1. 기존 두 development bag의 추가 파라미터 튜닝은 중단한다.
2. 가속 합류, 로터리 가속, 일정 순항과 후측방 차량을 포함한 새 held-out bag을 확보한다.
3. P7 detector와 position-only/P8을 고정 설정으로 한 번 비교한다.
4. P8이 tracking AVE와 Map-CV 전 horizon을 함께 개선할 때만 uncertainty-aware P9로 간다.
5. 통과하지 못하면 detector velocity는 confidence 참고값으로만 남기고 position tracker와
   temporal lane association 개선으로 전환한다.

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
- P1은 현재 frame의 decoder 전 proposal center와 memory의 decoder 후 center를 비교했다.
  decoder refinement 평균 `~0.48 m`가 `~0.15 s` 간격에서 `~3.2 m/s`의 가짜 motion이 될
  수 있는 구조 오류였다.
- P2는 양쪽 center를 decoder 후 기준으로 통일했다. Hardcase V2 scene-disjoint에서
  temporal velocity 90% fusion은 best speed MAE를 `3.133 → 1.070 m/s`로 `65.8%`
  개선했고 center mAP도 보존했다. 99% gate는 exact overfit에서 90%보다 악화되어
  기각했다.
- `<1.0 m/s` gate에는 근소하게 실패했으므로 아직 production 후보는 아니다. 다음은
  memory-active/fallback 분리 진단, adjacent-frame jitter, 개발 rosbag과 latency gate다.

구현 및 결과 문서는
[`260923_BASE4_Q1_P0_QUERY_MEMORY_RESULTS.md`](260923_BASE4_Q1_P0_QUERY_MEMORY_RESULTS.md)를
따른다. P1 구현 및 결과는
[`260923_BASE4_Q1_P1_MOTION_VALUE_RESULTS.md`](260923_BASE4_Q1_P1_MOTION_VALUE_RESULTS.md)를
따른다. P1의 auxiliary target은 기존 velocity annotation이며 GT identity 기반
displacement-consistency loss는 아직 구현하지 않았다.
P2의 center-contract 수정과 gate A/B는
[`260923_BASE4_Q1_P2_POSTDECODER_MOTION_RESULTS.md`](260923_BASE4_Q1_P2_POSTDECODER_MOTION_RESULTS.md)를
따른다.
P3의 다중-frame direct motion은 scene-disjoint speed MAE `0.845 m/s`로 첫 절대 gate를
통과했고 두 개발 bag에서도 평균과 jitter를 모두 개선했다. 다만 highway1 error-step p95
`5.569 m/s`와 mean latency `92.8 ms`가 남아 production 후보는 아니다. 상세 결과는
[`260923_BASE4_Q1_P3_ROBUST_MOTION_RESULTS.md`](260923_BASE4_Q1_P3_ROBUST_MOTION_RESULTS.md)를
따른다.

P4의 mutual matching + robust consensus는 기각했다. 엄격 2-frame consensus는 exact64
speed MAE가 `1.998 m/s`로 붕괴했고, 단일 mutual pair를 35 m/s cap 아래에서 허용한
P4b도 hardcase best `1.274 m/s`로 P3 `0.845 m/s`보다 악화했다. 동일 P3 가중치의
재학습 없는 비교에서도 P4b velocity만 `0.845 → 1.419 m/s`로 악화해 association hard
gate가 원인임을 분리했다. rosbag/tracker gate는 열지 않았으며 P3를 연구 baseline으로
유지한다. 상세 결과는
[`260923_BASE4_Q1_P4_ROBUST_ASSOCIATION_RESULTS.md`](260923_BASE4_Q1_P4_ROBUST_ASSOCIATION_RESULTS.md)를
따른다.

P5는 P3 출력을 보존한 채 robust 후보를 confidence로 작게 섞었다. 동일 P3 E1
checkpoint의 재학습 없는 hardcase 비교에서 blend `0.00/0.10/0.25`의 speed MAE는 각각
`0.8451/0.8472/0.8508 m/s`였다. 공간 mAP과 mATE는 완전히 동일했지만 robust 후보의
비중을 높일수록 velocity만 단조롭게 악화했다. 따라서 rosbag/tracker gate는 열지 않고,
현재 위치 기반 mutual/consensus threshold 튜닝을 종료한다. 다음 association 연구는
query feature similarity 또는 tracker innovation처럼 identity 정보가 추가될 때만 재개한다.
상세 결과는
[`260923_BASE4_Q1_P5_SHADOW_SOFT_BLEND_RESULTS.md`](260923_BASE4_Q1_P5_SHADOW_SOFT_BLEND_RESULTS.md)를
따른다.

P6 진단에서는 hardcase val의 association 후보가 p50/p95 모두 `1`이었고, GT identity로
사후 검증한 distance/feature matching도 `98/98`로 정확했다. 반면 정답 identity query
center displacement 자체의 speed MAE가 `0.846 m/s`여서, 오매칭보다 짧은 `dt`의 center
jitter가 병목임을 확인했다. Pair weight에 `dt^2`를 적용한 fixed-intercept WLS는 재학습
없이 hardcase speed MAE를 `0.845 → 0.738 m/s`로 낮췄다. 개발 bag에서도 highway1
speed MAE `1.581 → 0.872 m/s`, error-step p95 `5.569 → 1.492 m/s`로 개선했고 합류도
동시에 개선했다.

그러나 downstream tracker A/B에서는 position-only가 F1 `0.9329`, speed MAE
`0.671 m/s`, AVE `0.821 m/s`와 stable minFDE-K 전 horizon에서 가장 좋았다. Detector
velocity 매-frame update와 init-only 모두 이를 넘지 못했다. 따라서 P6는 detector 연구
후보로 채택하되 production의 detector velocity fusion은 계속 끈다. 상세 결과는
[`260923_BASE4_Q1_P6_AGE_WEIGHTED_MOTION_RESULTS.md`](260923_BASE4_Q1_P6_AGE_WEIGHTED_MOTION_RESULTS.md)를
따른다.

P7은 P6가 현재 query center를 정확한 intercept로 고정하던 가정을 제거하고, 현재와 과거
center 전체에서 intercept와 velocity를 함께 적합했다. 재학습 없이 hardcase speed MAE를
`0.738 → 0.676 m/s`로 낮췄고, 개발 bag도 highway1 `0.872 → 0.811 m/s`, 합류
`0.829 → 0.779 m/s`로 모두 개선했다. 가속도 근사 slice에서도 highway1 moderate/high와
합류 steady/moderate가 개선되어 등속 구간에만 맞춘 결과로 보이지 않는다.

다만 track 생성 후 1초 동안 P7의 `vx, vy`를 EKF에 직접 넣은 실험은 종방향 speed만
개선하고 center, lateral error, 2D AVE와 stable minFDE-K를 악화시켰다. Production은
계속 position-only mode 0을 유지한다. 다음 P8은 track/lane 종방향 성분만 조건부로
사용하고 lateral/spatial state를 건드리지 않는 결합을 검토한다. 상세 결과는
[`260924_BASE4_Q1_P7_FREE_INTERCEPT_RESULTS.md`](260924_BASE4_Q1_P7_FREE_INTERCEPT_RESULTS.md)를
따른다.

P8 감사에서 tracker가 detector position/yaw는 map frame으로 바꾸면서 LiDAR-frame
velocity를 회전하지 않는 계약 오류를 발견해 수정했다. 이후 EKF state는 position-only로
유지하고 초기 1초의 종방향 output만 선형 감쇠 결합했다. 두 bag에서 speed와 2D AVE는
소폭 개선됐고 stable 3초 FDE도 개선됐지만, highway1 all-FDE와 stable 1/2초는 소폭
악화해 production gate에는 미달했다. mode 4는 기본 off 연구 옵션이며 상세 결과는
[`260924_BASE4_Q1_P8_LONGITUDINAL_OUTPUT_RESULTS.md`](260924_BASE4_Q1_P8_LONGITUDINAL_OUTPUT_RESULTS.md)를
따른다.

P9-A는 이미 메시지에 존재하던 detector twist covariance가 tracker에서 버려지던 경로를
복구하고, tracker 종방향 covariance와 역분산 결합하는 mode 5를 추가했다. 초기 1초로
제한하면 두 개발 bag의 speed/AVE와 all-FDE는 소폭 개선됐지만 stable FDE는 일관되지
않았다. track-age 제한을 제거한 continuous 결합은 highway1 speed MAE `0.445 → 0.695`,
stable FDE@3s `0.905 → 2.478 m`로 명확히 실패해 기각했다. Production은 mode 0이며,
query별 temporal uncertainty를 추가하는 P9-B는 새 held-out bag 이후에만 연다. 상세는
[`260924_BASE4_Q1_P9_UNCERTAINTY_FUSION_RESULTS.md`](260924_BASE4_Q1_P9_UNCERTAINTY_FUSION_RESULTS.md)를
따른다.

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
