# Base4-Temporal 고주로 Velocity Pilot 계획

> **문서 상태: PILOT VALIDATED / PRODUCTION CAPTURE READY.** 전체 Base4 데이터셋을 확정하는 문서가 아니라, 고주로에서
> BEVFusion velocity observation의 실효성을 가장 작게 검증하기 위한 통제 실험이다.

## 한 줄 결론

Production E25의 공간 인식 능력을 초기값으로 사용하고, 새 고주로 연속 sequence에서
Vehicle `vx, vy`를 학습한다. 기존 고주로 두 rosbag은 반복 사용된 **development
evaluation**으로만 쓰고, 최종 성능 주장은 새 held-out 고주로 rosbag으로 확인한다.

## 왜 지금 이 실험인가

- Production E25 + EKF position-only tracking의 고주로 2D AVE는 약 `0.96 m/s`, 전체
  3초 Top-1 FDE는 약 `4.66 m`다.
- Tracker motion model과 covariance sweep으로 얻은 개선은 유효하지만 작았다.
- GT velocity oracle을 매 update에 보수적으로 융합했을 때 2D AVE는
  `0.852 -> 0.259 m/s`, stable 3초 FDE는 `1.617 -> 0.594 m`로 낮아졌다.
- 따라서 다음 큰 폭의 motion 개선 후보는 tracker 복잡도 증가가 아니라 detector가
  실제 velocity observation을 제공하는 것이다.

Oracle 수치는 production 예상치가 아니라 **velocity 정보가 충분히 정확할 때의 잠재
상한**이다. 실제 head는 반드시 velocity 미사용 tracker와 같은 조건에서 비교한다.

## 범위와 명칭

- Dataset/실험명: `Base4-Temporal-Highway-Pilot`, 축약 `B4T-HWY-P0`.
- 대상 class: 첫 pilot은 `Vehicle`만.
- 대상 구간: 고주로 ramp, merge, mainline, taper, exit.
- 목적: 전체 대회 분포 학습이 아니라 velocity head와 temporal input의 효과 분리.
- 이 결과만으로 전역 production을 교체하지 않는다.

## 수집 계약

### 필요한 motion family

- Ego/NPC 동일 속도 순항.
- Ego가 선행차를 따라잡거나 멀어지는 상대속도 변화.
- NPC 가속·감속과 spawn 직후 안정화 구간.
- 같은 차선 선행/후행 차량.
- 양쪽 인접 차선의 병렬·추월 차량.
- 합류·차선 변경으로 횡속도가 생기는 차량.
- 주 평가 거리 `10--30 m`; 가까운 차량과 30 m 경계 표본도 유지.

### 첫 고주로 campaign의 구간·속도 배분

조기 ramp 가속 상태는 어렵지만 planning이 아직 즉시 반응할 필요가 작다. 따라서 첫
pilot은 합류 이후의 실제 추월·상대속도 분포에 집중한다.

| 구간 | 목표 비율 | Ego 속도 | NPC 상태 |
|---|---:|---|---|
| ramp 초기 접근 | 0% | - | live pilot에서 제어권 전환 전 actor 이탈로 제외 |
| 실제 merge/conflict | 25% | 50/54/58 km/h | 59 km/h 미만, 40/50/60 NPC 순항 |
| post-merge 순항·추월 | 75% | 70/85/100/115/120 km/h | 시작속도=목표속도인 40/50/60 NPC 순항 |

합치면 실제 합류 25%, 합류 이후 75%다. `115/120 km/h`는 소수 hard case로만 둔다.
고속 Ego는 capture 시작 전에 충분히 상류에서 출발해 목표속도에 도달시킨다. 같은 차선의
큰 상대속도는 충돌 위험 때문에 짧은 window로 제한하고, 큰 closing/opening speed는 주로
인접 차선에서 만든다. 첫 pilot에는 실제 lane-change controller를 넣지 않으며, 이후
lateral-motion addon으로 분리한다. 큰 상대속도는 인접 차선에만 배정하고
restricted solid-separated lane은 계속 제외한다.

2026-09-20 MR live pilot에서 merge conflict `2/2`, post-merge `9/9`가 완료됐고
`120 km/h` command scene도 16 trigger를 완주했다. 성공 window는 LiDAR `23--35 frame`을
보존했다. 이 결과를 본 campaign 진입 gate로 사용한다.

### sequence 단위

- 독립 snapshot이 아니라 실제 LiDAR cadence의 연속 frame을 저장한다.
- 첫 목표는 약 `250--350 sequence`, sequence당 최소 `12`개 유효 LiDAR frame이다.
  최종 수량은 raw scene 수가 아니라 필터 후 유효 Vehicle track과 motion-family
  coverage로 결정한다.
- `/dataset/scenario_marker` 밖의 load/teleport 구간은 제거한다.
- scene/sequence를 split 단위로 사용하고 인접 frame을 train/val에 나누지 않는다.

### Gated raw 기록과 저장 용량

- 센서는 끊지 않고 계속 수신하되 ROS snapshot ring buffer를 사용한다.
- 정상 `capture_end`에서만 scene bag을 저장하며 scene-load/reconnect 대기는 버린다.
- 2026-09-20 MR 실측에서 32 MiB는 start marker가 잘려 기각했고, 48 MiB는
  `2.71--2.76 s` window를 완전히 보존했다.
- 현재 48 MiB 설정의 실제 파일은 scene당 약 `50.5 MB`다.
- 1,200 scene 약 `61 GB`, 1,500 scene 약 `76 GB`를 예상한다.
- 첫 20--30 scene에서 marker, sensor rate, camera age와 scene-to-bag sidecar를 audit한
  뒤 unattended campaign으로 넘어간다.
- continuous recorder는 진단 fallback이며, raw bag을 녹화 중 삭제하면서 converter를
  병렬 실행하는 방식은 crash 시 source/metadata가 어긋나므로 기본 경로로 쓰지 않는다.

### timestamp와 좌표

- LiDAR header timestamp를 sample 기준시각으로 사용한다.
- Camera는 causal tolerance 안의 가장 가까운 frame을 사용하고 age를 기록한다.
- Ego/Object pose는 LiDAR 시각 앞뒤 상태로 보간한다.
- 학습 label은 LiDAR/Ego 축의 절대 planar vector
  `velocity_target_ego_mps=[vx, vy]` 하나만 노출한다.
- 위치 미분값은 필요할 때 `diagnostics` namespace에서 GT sanity check에만 사용하며,
  학습 target 이름으로 노출하지 않는다.
- runtime `DetectedObject`에서는 detector/LiDAR `vx, vy`를 box yaw로 회전해 object-body
  twist로 변환하고, tracker가 이를 다시 map `vx, vy` observation으로 사용한다.

### temporal 입력의 motion compensation

- 과거 sweep은 Ego pose로 현재 LiDAR frame에 정합한다.
- GT Object pose로 동적 객체 point를 현재 위치에 강제 정합하지 않는다. 실제 추론에는
  object GT가 없고, 이를 적용하면 velocity 단서를 제거하거나 label leakage가 된다.
- 단순 point concatenation의 object trail이 geometry를 해칠 수 있으므로 single-frame
  velocity head를 먼저 검증한 뒤 temporal feature fusion으로 넘어간다.
- Object pose/velocity는 input 보정이 아니라 label 생성과 감사에 사용한다.

## 학습·평가 split

Pilot dataset은 다음 두 split만 만든다.

- `train`: 약 80%, sequence 단위.
- `val`: 약 20%, 다른 scenario seed·route segment·NPC motion 조합.

별도 dataset `test` split은 만들지 않는다. 작은 pilot에서 표본을 다시 떼기보다 val로
checkpoint를 고르고 rosbag에서 downstream을 검증한다.

단, 현재 `vehicle_highway_merge`와 `vehicle_highway1`은 이미 반복적으로 확인하고
파라미터 선택에 사용했으므로 엄밀한 test가 아니다.

- 기존 두 bag: development rosbag, 빠른 A/B와 epoch 선택 보조.
- 새 고주로 bag 최소 2개: 마지막에 한 번 여는 held-out rosbag.
- held-out은 다른 NPC seed, 속도, spawn timing과 주행을 사용한다.

새 held-out 없이 기존 두 bag만으로 얻은 결과는 `pilot/dev result`라고 보고하며 최종
일반화 성능이라고 표현하지 않는다.

## 비교할 모델

| ID | 초기값 | 입력 | 출력 | 목적 |
|---|---|---|---|---|
| E25-7D | Production E25 | single frame | 7D box | 고정 기준선 |
| B4H-V1 | E25 spatial weights | single frame | 7D + `vx,vy` | head/supervision 효과 |
| B4H-T1 | B4H-V1 또는 E25 | causal 3-sweep | 7D + `vx,vy` | 실제 temporal 이득 |

처음부터 3-sweep 하나만 학습하지 않는다. B4H-V1이 있어야 velocity GT 자체의 품질과
temporal 입력의 추가 이득을 분리할 수 있다.

## Temporal architecture 판단과 확장 순서

### 결론

현재 코드에서 **최소 수정으로 최대 실험 효율**을 노리는 첫 temporal 방식은
StreamPETR query memory가 아니라 원본 BEVFusion의 causal multi-sweep 입력을 복구하는
것이다. StreamPETR는 유력한 후속 후보지만, B4H-T1보다 먼저 구현하지 않는다.

권장 확장 순서는 다음과 같다.

| 단계 | ID | 방식 | 역할 |
|---|---|---|---|
| 0 | E25-7D | 현재 single-frame 7D | 고정 production 기준선 |
| 1 | B4H-V1 | single-frame 9D velocity head | velocity label·head 자체의 효과 분리 |
| 2 | B4H-T1 | Ego 정합 3-sweep + point `dt` | 가장 작은 temporal baseline |
| 3 | B4H-T2 | 직전 fused-BEV ego warp + gated fusion | raw point ghosting을 피하는 feature temporal |
| 4 | B4H-Q1 | StreamPETR-inspired sparse query memory | 긴 객체 이력과 object-centric motion |

앞 단계가 채택 gate를 넘지 못했다고 무조건 다음 단계를 구현하지 않는다. 특히 V1만으로
tracker AVE와 Map-CV FDE가 충분히 개선되면 temporal 구조의 production 복잡도를 추가할
이유가 없다.

### B4H-T1이 첫 temporal baseline인 이유

현재 MORAI Base3 config는 single-frame 실험을 위해 원본 MMDetection3D BEVFusion에서
다음 기능을 의도적으로 제거한 상태다.

- `LoadPointsFromMultiSweeps`와 여러 과거 sweep.
- 다섯 번째 point feature인 time lag.
- TransFusion의 `vel=[2,2]` head와 10D box coder.

반대로 upstream BEVFusion config는 이미 9개 과거 sweep, 5-feature LiDAR와 velocity
head를 지원한다. 따라서 B4H-T1은 새 detector를 이식하는 작업이 아니라 기존 경로를
MORAI의 실제 연속 sequence와 3-sweep 계약에 맞게 복구하는 작업에 가깝다. 현재
`adapt_nuscenes_checkpoint.py`에서 제거하는 fifth-channel sparse-conv weight와 velocity
head도 이 단계에서 다시 사용할 수 있다.

과거 sweep은 Ego motion만 현재 LiDAR frame으로 정합하고 point마다 실제 `dt`를 둔다.
GT Object pose를 이용한 object compensation은 online 추론에서 재현할 수 없고 velocity
단서를 지우므로 계속 금지한다.

### B4H-T2: lightweight temporal BEV

B4H-T1에서 동적 객체 trail 때문에 center/AP가 나빠지거나 velocity 개선이 작으면 다음
후보는 직전 fused-BEV 한 장을 쓰는 경량 recurrent fusion이다.

1. 직전 fused-BEV와 그 시각의 Ego pose를 cache한다.
2. 현재 Ego frame으로 과거 BEV를 warp한다.
3. current/previous feature를 concatenate한다.
4. 작은 `1x1 convolution` 또는 learned gate로 다시 기존 head channel에 투영한다.

온라인에서는 직전 BEV를 재사용하므로 매 frame 과거 encoder를 다시 돌리지 않는다.
다만 학습 loader는 순서가 보장된 clip, 동일 clip augmentation, scene reset과 frame-drop
정책을 제공해야 한다. 이 방식도 단순 config 변경은 아니지만 현재 BEVFusion encoder와
TransFusion head를 유지하므로 query-memory 이식보다 변경 범위가 작다.

### StreamPETR를 그대로 첫 선택으로 삼지 않는 이유

StreamPETR의 핵심은 이전 BEV feature의 `grid_sample` cache가 아니라 **object-query
memory**다. 과거 frame의 Top-K query feature, reference point, confidence, timestamp와
Ego pose를 FIFO memory에 보관하고, 현재 query가 propagation transformer로 과거 query와
상호작용한다. 따라서 단순히 `extract_feat` 뒤에 BEV warp 하나를 붙이는 방식과 다르다.

우리 TransFusion head도 query를 사용하므로 개념적으로는 결합 가능하지만 현재 query는
매 frame dense heatmap에서 새로 생성된다. B4H-Q1에는 최소한 다음 변경이 필요하다.

- 최종 Top-K query feature·center·score·velocity의 외부 노출과 저장.
- timestamp/Ego pose 기반 reference point 변환.
- current query와 memory query 사이 temporal attention.
- batch sample별 sequence ID, memory validity와 scene/teleport reset.
- ordered streaming 또는 clip 학습, augmentation 일관성, memory detach/BPTT 정책.
- 초기 miss와 오검출 query가 memory에 누적되는 경우의 age/confidence gate.
- runtime frame drop과 비정상 `dt` 처리.

따라서 sparse memory의 online 계산량은 매력적이지만 구현·학습 위험은 B4H-T1/T2보다
높다. T1 또는 T2로 temporal 정보의 실효성이 확인되고 dense feature memory의 지연이나
velocity 한계가 남을 때 `B4H-Q1`로 통제 실험한다. StreamPETR 전체를 포팅하지 않고
Vehicle Top-K query만 보존하는 작은 memory bank부터 시작한다.

공식 StreamPETR는 camera-only PETR 계열이며 논문 결과는 8-frame sliding-window 학습을
사용했다. 공식 저장소의 streaming-video 학습은 메모리·시간을 줄이지만 더 늦게
수렴한다고 명시한다. 또한 공식 환경은 PyTorch 1.9와 MMDetection3D 1.0.0rc6 계열이라
현재 팀 Docker/MMDetection3D에 파일을 그대로 복사하지 않는다.

- Paper: <https://arxiv.org/abs/2303.11926>
- Official repository: <https://github.com/exiawsh/StreamPETR>
- Official training notes: <https://github.com/exiawsh/StreamPETR/blob/main/docs/training_inference.md>

### SOLOFusion과의 관계

SOLOFusion은 camera-only BEV 계열에서 긴 image history의 coarse cost volume과 짧은
history의 fine-grained stereo를 결합한다. 긴 history와 2-stage training/data sampler가
필요하여 현재 LiDAR-camera BEVFusion velocity pilot의 최소 변경 목표와 맞지 않는다.
아이디어 비교 대상으로는 유지하지만 B4T-HWY-P0 구현 후보에서는 제외한다.

- Paper/repository: <https://github.com/Divadi/SOLOFusion>

### T1 이후 선택 gate

- T1이 V1 대비 velocity와 downstream FDE를 개선하고 geometry gate도 통과하면 T1을
  우선 채택하고 구조 확장을 멈춘다.
- T1 velocity는 좋아지지만 center/AP가 ghosting으로 퇴행하면 T2를 수행한다.
- T2가 정확하지만 latency/VRAM이 production 범위를 넘거나 장기 객체 motion이 여전히
  부족하면 Q1을 수행한다.
- T1과 T2 모두 V1보다 유의미하지 않으면 temporal architecture보다 velocity label,
  motion-family coverage와 loss/covariance 계약을 먼저 재감사한다.

## Tracker A/B

각 velocity 모델을 동일 detection threshold에서 두 번 평가한다.

1. Detector velocity를 tracker에 넣지 않음.
2. 매 measurement update에 detector velocity를 넣음.
   - 첫 covariance: `0.5 m/s`.
   - 비정상값, `30 m/s` 초과와 비-Vehicle velocity는 거부.

Fusion을 켰을 때 OD geometry, track association 또는 center가 악화되면 velocity
covariance를 키우거나 head를 기각한다. 모델이 velocity를 출력한다는 이유만으로
production fusion을 켜지 않는다.

## 지표와 선택 기준

### OD

- Center-distance AP/precision/recall/F1.
- Center p50/p90, mATE, mASE, mAOE.
- Vehicle speed MAE, 실제 2D AVE, GT-heading 종/횡 velocity MAE.

### Tracking

- F1/MOTA와 track continuity.
- Center p50/p90.
- Speed MAE와 실제 2D AVE.
- Track 생성 후 `0--0.5 s`, 안정 track을 분리해 velocity 수렴시간 확인.

### Prediction/Planning proxy

- Map-CV Top-1 FDE와 minFDE-K `@1/2/3 s`.
- CV-stable과 전체 motion slice를 모두 보고한다.
- 고주로 추월 관점에서 인접 차선 occupancy, track age, 경로 coverage를 함께 본다.

첫 채택 gate는 다음과 같다.

- 기존 E25 대비 Vehicle F1 `-0.01` 이내, center p50 `+0.05 m` 이내.
- Velocity fusion이 velocity 미사용 대비 tracking 2D AVE 또는 stable FDE@3s를
  상대 `10%` 이상 개선.
- 두 development bag 모두 같은 방향으로 개선하며 한 bag 평균으로 다른 bag 퇴행을
  숨기지 않음.
- 전역 production 후보가 되려면 새 held-out 고주로와 기존 Vehicle/Pedestrian/
  Obstacle regression suite를 추가 통과.

## 실행 순서

1. 현재 Base3-Online raw bag/index가 true consecutive temporal sample을 보존하는지
   감사하고, single-frame velocity-head smoke에 재사용 가능한 범위를 확정한다.
2. 새 B4T-HWY-P0 연속 수집기의 marker/timestamp/velocity schema를 2-scene pilot로
   검증한다.
3. 20--30 sequence pilot을 수집해 track continuity, velocity target, camera age,
   point count를 검사한다.
4. B4H-V1 single-frame 9D smoke와 짧은 학습을 먼저 수행한다.
5. B4H-T1 causal 3-sweep을 같은 split과 loss 계약으로 학습한다.
6. 기존 고주로 두 dev bag에서 OD -> tracker off/on -> Map-CV를 순서대로 평가한다.
7. 명확한 후보만 새 held-out bag에서 한 번 평가한다.

## 명시적 비목표

- Base2-Temporal을 최종 production dataset으로 복구하지 않는다.
- Base3-Static 독립 frame을 가짜 연속 sequence로 묶지 않는다.
- GT Object motion compensation을 online input에 사용하지 않는다.
- 기존 고주로 두 bag의 반복 점수를 최종 test 성능으로 부르지 않는다.
- pilot 결과만으로 Pedestrian/Obstacle 또는 전체 route production을 교체하지 않는다.

## 관련 근거

- Tracker·oracle: [`260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md`](260919_HIGHWAY_TRACKER_VELOCITY_ABLATION.md)
- Base3-Online 결과: [`260919_BASE3_ONLINE_E25_FINETUNE_RESULTS.md`](260919_BASE3_ONLINE_E25_FINETUNE_RESULTS.md)
- Dataset 현재 상태: [`../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`](../../dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md)
- 기존 Base2 이력: [`../../dataset/morai_od/docs/legacy/base2_10k_d3_temporal_label_index_audit.md`](../../dataset/morai_od/docs/legacy/base2_10k_d3_temporal_label_index_audit.md)
