# Base3-Static에서 Base4-Temporal로 넘어가는 조건부 실험 로드맵.

마지막 갱신: 2026-08-29.

## 목적.

Base3-Static 10K로 single-frame 정적 3D OD의 class·위치·크기·yaw·배경 일반화를
먼저 최대한 높인다. 이후 같은 OD를 현재 EKF tracker에 연결해 tracker 자체의 위치 이력
기반 velocity가 대회 mission에 충분한지 확인한다. Tracking velocity와 Map-CV prediction이
충분하면 동적 OD velocity 개발을 production 필수사항으로 만들지 않는다. 부족할 때만
Base4-Temporal 데이터셋과 temporal OD를 진행한다.

## 단계별 의사결정.

1. Base3-Static 10K capture·정제·학습을 완료한다.
2. Offline validation과 mission rosbag에서 Base3 single-frame OD를 평가한다.
3. Base3 OD는 velocity 없이 BBox만 EKF tracker에 전달한다.
4. Tracking Vehicle AVE·speed error·center error와 Map-CV 1/2/3초 FDE를 평가한다.
5. Tracker-only velocity가 충분하면 Base3 기반 pipeline을 production 후보로 유지한다.
6. 고주로·로터리에서 tracker 수렴 지연이나 2~3초 종방향 누적 오차가 계속 크면
   Base4-Temporal을 시작한다.

Base4 착수 여부는 OD AVE 하나가 아니라 다음 두 pipeline의 end-to-end 비교로 결정한다.

- Base3 BBox → tracker 자체 velocity → Map-CV prediction.
- Base4 BBox+velocity → gated EKF velocity update → Map-CV prediction.

## Base3-Static 학습 계약.

- 약 10,002 frames를 각각 독립 single-frame sample로 사용한다.
- 7D BBox `x, y, z, dx, dy, dz, yaw`를 학습한다.
- Velocity regression을 수행하지 않는다.
- `time_lag_sec`, sweep index, 과거 sweep을 입력하지 않는다.
- Capture clip의 3 frames는 약 3 m 이동 간격의 서로 다른 single-frame 관측이다.
  이를 12 Hz temporal sweep으로 취급하지 않는다.
- 0-point GT는 제거하고 1--4-point GT는 positive loss에서 제외한다.

## Base4-Temporal 데이터 계약.

- 원본 sensor observation을 목표 12 Hz로 연속 저장한다.
- Clip당 원본 8 frames와 각 frame의 실제 timestamp를 보존한다.
- 학습 baseline은 causal 3-sweep `t, t-1, t-2`로 시작한다.
- 실제 `time_lag_sec`, Ego pose, sweep validity mask를 저장한다.
- Sweep 간 Ego motion을 현재 frame으로 정합한다.
- Velocity GT source·freshness·quality mask를 명시한다.
- Dynamic Vehicle과 Pedestrian뿐 아니라 static object와 empty background도 일부 포함한다.
- Base3의 3 m 간격 frame으로 누락된 12 Hz history를 합성하지 않는다.

### Base4 착수 시 velocity metadata 재검토 항목.

Base3-Static Batch 001--007은 capture 도중 schema를 바꾸지 않고 현재처럼
`morai_velocity_raw`와 `derived_velocity_mps`를 모두 보존한다. 다만 이는 raw capture
archive의 일관성을 위한 결정이며, `derived_velocity_mps`를 학습용 GT로 승인한다는 뜻이
아니다.

Base4-Temporal 수집기와 dataset builder를 구현하기 전에 다음 결정을 다시 명시적으로
검토한다.

- 학습용 공개 label은 변환이 끝난 `velocity_target_ego_mps` 하나로 제한한다.
- `derived_velocity_mps`는 위치 차분 진단값이며 GT로 사용하지 않는다.
- Base3 학습·팀원 공유 package에서는 혼동을 막기 위해 derived 값을 제외한다.
- Base4에서 raw velocity의 단위·좌표계·freshness 검증이 다시 필요할 경우, 연속 위치와
  실제 timestamp로 derived velocity를 builder/audit 단계에서 필요할 때만 재계산한다.
- derived 값을 영구 metadata로 저장할지, audit report에만 남길지는 새 12 Hz pilot에서
  실제로 추가 오류를 잡는지 확인한 뒤 결정한다.
- 저장한다면 `diagnostics.position_difference_velocity_mps`처럼 진단 전용 namespace와
  `label_eligible: false`를 사용해 학습 label과 분리한다.

과거 Base2 D3에서는 derived 비교가 MORAI Object velocity의 km/h/local-frame 해석을
확인하고 raw-derived 불일치 59건과 무효 label 5건을 찾는 데 실제로 쓰였다. 반면 초기
derived 계산은 wall time과 simulator time 차이로 자체 scale이 틀린 적도 있었다. 따라서
Base4에서는 과거 필드를 관성적으로 유지하지 않고, 정확한 timestamp 기반 audit 효용을
새로 입증해야 한다.

Base3 최종 replacement audit에서는 pre-capture gate를 통과한 TrafficBarrel이 실제 세
frame 동안 약 2.4 m 이동한 것을 위치 차분으로 발견해 제외했다. 따라서 공유용 label에서
derived 필드를 제거하더라도, raw 위치 이력으로 motion consistency를 재계산하는 post-hoc
audit은 Base4에서도 유지한다.

## Pretraining과 full fine-tuning.

Base3 checkpoint는 temporal motion 자체를 pretrain하지 않는다. Spatial encoder와 3D
detection을 pretrain한다.

Base3에서 이전하는 가중치:

- LiDAR·Camera encoder.
- PointPainting과 BEV backbone.
- Class head.
- Position·size·yaw head.

Base4에서 새로 초기화하는 가중치:

- Temporal fusion과 time embedding.
- Velocity head.
- Velocity confidence·uncertainty head.

Base4 학습에서는 shared backbone까지 unfreeze한 full fine-tuning을 수행한다. 초기에는
pretrained backbone에 새 module보다 작은 learning rate를 적용하고, 장기간 동결하지 않는다.
Base3 sample을 Base4 temporal input으로 억지 변환하지 않는다.

## 예정 ablation.

| 실험 | 초기화 | Fine-tuning 데이터 | 목적 |
| --- | --- | --- | --- |
| T0 | Random | Base4 only | Temporal from-scratch 기준. |
| T1 | Base3 pretrained | Base4 only | Spatial pretraining의 순수 효과. |
| T2 | Base3 pretrained | Base4 temporal + 별도 Base3 single-frame auxiliary branch | Static detection 망각 방지. 구조 지원 시에만 수행. |

첫 실험은 T1로 한다. Base3 sample을 temporal loader에 섞지 않고 Base4-only로 full
fine-tuning해 temporal·velocity 효과를 분리한다. Base3 offline validation이나 empty-route
false positive가 유의미하게 악화될 때만 T2 구현 가치를 검토한다.

Base3는 약 3 m 간격의 독립 single-frame 자료이고 Base4는 고정 주기의 연속 3-sweep이다.
따라서 두 데이터를 같은 temporal input batch에 그대로 섞으면 안 된다. 과거 frame을
복제하거나 0으로 채우는 것도 실제 Base4 input distribution과 다르므로 기본 전략으로
사용하지 않는다.

T2는 모델이 다음 구조를 명시적으로 지원할 때만 가능하다.

- Base4 batch는 기존 temporal branch에서 3-sweep fusion과 detection·velocity loss를 계산한다.
- Base3 batch는 별도 single-frame auxiliary branch에서 spatial encoder와 detection head만
  사용한다.
- 두 branch는 spatial encoder와 detection head weight를 공유한다.
- Base3 branch에서는 temporal module과 velocity head를 호출하지 않으며 velocity loss도
  계산하지 않는다.

이 분기나 dual-loader가 없다면 T2는 보류하고 T1을 production 실험으로 사용한다. 더 단순한
망각 대응은 pretrained backbone learning rate를 낮추고 Base4 detection loss를 유지하거나,
Base4 자체에 충분한 static·empty sequence를 포함하는 것이다.

## Tracker 통합 계약.

- 현재 Base3 production 후보는 OD velocity를 발행하지 않는다.
- Base4 velocity는 EKF control input이 아니라 optional measurement로 전달한다.
- Velocity confidence·uncertainty와 measurement covariance를 함께 전달한다.
- Class, temporal history, 물리 범위, Lanelet 방향, tracker innovation gate를 통과할 때만
  velocity update를 사용한다.
- Gate를 통과하지 못하면 현재처럼 연속 위치 변화로 tracker가 velocity를 추정한다.

## 평가 지표와 production 판정.

Offline OD:

- Class별 AP·recall·precision.
- mATE·mASE·mAOE.
- Vehicle AVE, speed MAE, 종·횡방향 velocity MAE.

Mission rosbag end-to-end:

- OD Vehicle AVE와 speed error.
- Tracking Vehicle AVE, speed p50/p90, center error, ID switch.
- Map-CV minFDE-K 1/2/3초와 signed longitudinal/lateral FDE.
- 로터리·고주로 합류·고주로 추월 slice.

OD velocity가 존재한다는 이유만으로 production에 넣지 않는다. BBox-only tracker 대비
Tracking AVE와 2--3초 prediction이 실제로 좋아질 때만 활성화한다. 대략적인 OD Vehicle
AVE 해석은 다음과 같다.

- 0.5 m/s 이하: 적극적인 gated integration 후보.
- 0.5--0.8 m/s: 큰 covariance를 쓰는 보조 measurement 후보.
- 0.8--1.0 m/s 이상: tracker-only보다 좋아지는지 반드시 rosbag A/B로 확인.

## 현재 상태.

- Base3-Static corrective topology 30/120 live gate를 통과했다.
- Batch 005--007 source 생성과 offline validator·topology audit을 완료했다.
- 다음 작업은 Base3-Static capture 완료, 전체 10K 정제, single-frame 학습과
  OD→tracking→prediction 평가다.
- Base4-Temporal은 위 평가에서 tracker velocity가 아쉬울 때 시작하는 조건부 후속 작업이다.
