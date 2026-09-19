# Base4-Temporal 고주로 Velocity Pilot 계획

> **문서 상태: PLAN.** 전체 Base4 데이터셋을 확정하는 문서가 아니라, 고주로에서
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

### sequence 단위

- 독립 snapshot이 아니라 실제 LiDAR cadence의 연속 frame을 저장한다.
- 첫 목표는 약 `250--350 sequence`, sequence당 최소 `12`개 유효 LiDAR frame이다.
  최종 수량은 raw scene 수가 아니라 필터 후 유효 Vehicle track과 motion-family
  coverage로 결정한다.
- `/dataset/scenario_marker` 밖의 load/teleport 구간은 제거한다.
- scene/sequence를 split 단위로 사용하고 인접 frame을 train/val에 나누지 않는다.

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
