# Fast-BEV obstacle regression 개선 로드맵

## 목표

대회에 등장할 obstacle asset을 사전에 완전히 알 수 없으므로 길거나 높거나 작은
객체를 학습 데이터에서 제거하지 않는다. 최종 출력 class도 계속 `vehicle`,
`pedestrian`, `obstacle`을 기본 계약으로 유지한다. 핵심 목표는 Planning이 정적
장애물을 안정적으로 회피할 수 있도록 obstacle의 중심 위치와 box를 정확히
추정하는 것이다.

## 현재 기준과 문제 가설

동일한 1,500-frame dataset과 222-frame held-out test에서 OpenCV/flip 0.5
3-class baseline은 다음 결과를 냈다.

- center mAP: `0.5919`
- obstacle mean center AP: `0.4194`
- obstacle mATE: `0.3704 m`
- obstacle mASE: `0.2379`
- IoU mAP@0.25: `0.5744`
- IoU mAP@0.50: `0.2751`
- inference: `41.48 FPS`

Obstacle의 크기 분포가 multimodal하여 하나의 regression 문제로 학습하기 어렵다는
가설은 타당하다. 그러나 최초 Shape5 실험은 label만 나눴고 regression은
class-agnostic anchor별 출력을 공유했다. 따라서 아래 두 실험으로 anchor 다양성과
class별 회귀 전문화를 분리해서 검증한다.

## H1: 3-class + five shared anchor sizes

Class와 dataset label은 기존과 동일하게 유지하고 anchor 크기 후보만 3개에서
5개로 늘린다.

```text
classes: vehicle, pedestrian, obstacle
sizes: vehicle, pedestrian, compact, tall, long representative sizes
rotations: 0, 90 degrees
anchors per BEV cell: 6 -> 10
```

목적은 표본을 분할하지 않고 각 anchor index의 독립적인 regression 출력을 통해
multimodal 형상에 더 가까운 초기 box를 제공하는 것이다. 모든 anchor가 모든
3개 class score를 낸다는 점은 유지되므로 classification 출력은 위치당
`18 -> 30`, 전체 head 출력은 `84 -> 140` channel로 증가한다.

예상 효과는 center AP보다 box scale과 IoU에서 먼저 나타나야 한다. H1이 유효하다는
최소 증거는 obstacle mASE 또는 IoU AP가 반복 가능한 수준으로 좋아지는 것이다.
Center mAP만 소폭 움직이고 IoU가 그대로라면 anchor 형상은 핵심 병목이 아니다.

## H2: 5-class + class-dedicated anchors/regression

내부 class를 vehicle, pedestrian, compact, tall, long으로 유지하되 각 class는
자기 대표 크기의 두 yaw anchor만 사용한다.

```text
vehicle          -> vehicle size anchors only
pedestrian       -> pedestrian size anchors only
compact_obstacle -> compact size anchors only
tall_obstacle    -> tall size anchors only
long_obstacle    -> long size anchors only
```

현재 `FreeAnchor3DHead`의 `assign_per_class` config만 켜서는 구현되지 않는다.
FreeAnchor loss가 일반 assigner 경로를 우회하고 각 GT에 대해 모든 anchor를 직접
top-k matching하기 때문이다. 별도 `ClassAwareFreeAnchor3DHead`가 필요하다.

- 학습 matching에서 GT class에 해당하는 anchor 두 개만 후보로 사용
- classification도 anchor마다 의미가 정해진 한 class logit만 출력
- bbox와 direction regression을 class 전용 anchor branch에서 출력
- inference에서 branch의 class를 암시적으로 부여
- compact/tall/long을 obstacle로 병합한 뒤 rotated NMS 적용

위치당 출력은 현재 shared Shape5의 `160` channel에서 dedicated Shape5의
`120` channel로 줄일 수 있다. 후보 class-anchor score도 `2,000,000`에서
`400,000`으로 감소한다. 다만 subclass별 표본 분할과 임의적인 형상 경계 문제는
남는다.

## 공정한 실험 규약

두 실험 모두 다음을 고정한다.

- 동일 train/val/test split: `1050/228/222`
- 동일 OpenCV backend와 flip 확률 `0.5/0.5`
- 동일 seed `0`, 12 epochs, FP16, optimizer와 learning rate
- best checkpoint는 validation center mAP
  - H1: `center_mAP`
  - H2: 최종 3-class로 병합한 `merged_center_mAP`
- NMS threshold 선택은 validation에서만 수행
- 최종 test는 선택 완료 후 한 번 평가
- 기존 3-class, shared Shape5, H1, H2의 FPS와 p95 latency도 함께 비교

주요 판단 지표 우선순위는 다음과 같다.

1. obstacle mATE와 AP@0.5m/1m
2. obstacle IoU AP와 mASE
3. 전체 center mAP
4. inference FPS와 p95 latency

## H1과 H2가 효과 없을 때의 개선 우선순위

### 1. 오류 분해 평가

학습 class는 3개로 유지하되 test obstacle을 사후 분석에서만 compact/tall/long,
거리, 가림 정도, 크기로 나눈다. 검출 누락인지 중심·크기·yaw 중 무엇이 병목인지
먼저 분리한다. Shape label은 진단용이며 model output contract에는 넣지 않는다.

### 2. 3-class main task + auxiliary shape head

메인 obstacle classification에는 모든 obstacle 표본을 사용하고, obstacle positive에
대해서만 compact/tall/long auxiliary loss를 준다. Auxiliary shape를 틀려도
obstacle 검출 자체는 실패하지 않는다. 필요하면 auxiliary feature로 세 regression
expert를 soft gating하되 최종 ROS 출력은 obstacle 하나로 유지한다.

### 3. Position-oriented class-aware regression loss

Planning 목적에 맞춰 obstacle의 x/y 중심 오차에 더 높은 가중치를 주고 size,
yaw, velocity 가중치를 별도로 조정한다. 기존 bbox loss에 BEV center L1 또는
center-distance loss를 병행한다. IoU loss만 높이면 크기별 불균형이 다시 커질 수
있으므로 중심 loss를 우선한다.

### 4. 제거 없는 균형 수집과 sampling

모든 asset을 유지하면서 compact/tall/long, `0-10/10-20/20-30 m`, 가림 정도,
전·측·후방 조합의 quota를 둔다. 같은 시나리오의 유사한 6 frame 반복보다 서로
다른 asset과 pose를 가진 scene 수를 늘린다. 희귀 형상은 scene/object-aware
sampler로 oversampling한다.

### 5. Anchor-free CenterHead

Fast-BEV의 camera-to-BEV feature는 유지하고 detection head를 CenterPoint 계열로
바꾼다. BEV heatmap에서 중심을 직접 검출하고 offset, 크기, yaw를 회귀하므로
형상이 다양한 obstacle을 고정 anchor에 맞출 필요가 없다. Anchor 실험이 모두
실패하면 가장 우선적인 구조 변경 후보다.

### 6. 비대칭 고해상도 BEV 또는 refinement

현재 약 `0.5 m` BEV cell은 작은 obstacle 위치에 거칠 수 있다. 전방 40 m,
후방 15 m, 좌우 25 m의 비대칭 관심 범위를 사용해 같은 tensor 비용으로 해상도를
높이거나, 0.5 m 전역 BEV 뒤 obstacle 주변만 고해상도로 보정하는 2-stage
refinement를 검토한다.

### 7. Prediction fusion과 temporal smoothing

가까운 중복 box의 최고 score 하나만 선택하지 않고 validation으로 안전성을 확인한
weighted box fusion을 적용할 수 있다. 정적 obstacle은 Tracking에서 velocity를
0 근처로 제한하고 여러 frame 위치를 EKF로 평활화해 Planning에 전달한다.

### 8. LiDAR 보조 또는 BEV fusion

정확한 3D 중심 거리가 최종 병목이면 camera-only depth 한계를 인정해야 한다.
16-channel LiDAR라도 camera box와 depth를 결합하거나 PointPainting/BEVFusion으로
확장하면 정적 obstacle 거리 보정에 직접적인 정보를 제공한다.

## 현재 권장 실험 순서

```text
H1 3-class + five shared anchors
 -> H2 5-class + dedicated anchors/regression
 -> 오류 분해
 -> auxiliary shape + center loss
 -> anchor-free CenterHead
 -> 고해상도 BEV
 -> temporal/LiDAR fusion
```

어느 단계에서도 대회에 나올 수 있는 obstacle asset을 단순 삭제해 성능을 높이지
않는다. 분포 제어가 필요하면 제거 대신 균형 수집과 sampling으로 해결한다.
