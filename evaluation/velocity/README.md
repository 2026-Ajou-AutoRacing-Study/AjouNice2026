# MORAI Velocity Evaluation

MORAI PointPainting 기반 3D object detector의
object velocity estimation 성능을 평가하기 위한 evaluator입니다.

## Evaluation Overview

Detection prediction과 GT를 먼저 매칭한 뒤,
Detection True Positive(TP)에 대해서 velocity error를 계산합니다.

평가되는 velocity는 현재 ego-oriented coordinate frame에서 표현된
object velocity입니다.

- vx: x-axis object velocity
- vy: y-axis object velocity

현재 모델이 직접 예측하는 것은 relative velocity가 아닙니다.

Relative velocity가 필요한 경우 다음과 같이 계산할 수 있습니다.

```text
v_rel = v_obj - v_ego
```

## Main Velocity Metric

### Vector AVE / mAVE

각 matched object에 대해 2D velocity vector의 Euclidean error를 계산합니다.

```text
velocity_error =
sqrt(
    (vx_pred - vx_gt)^2
    +
    (vy_pred - vy_gt)^2
)
```

전체 matched object에 대한 평균:

```text
mAVE = mean(velocity_error)
```

단위는 m/s입니다.

## Additional Metrics

Evaluator에서는 다음 metric도 함께 출력합니다.

- mAVE
- MAE vx
- MAE vy
- Speed MAE
- Zero-Velocity Baseline

Zero-Velocity Baseline은 모든 객체의 velocity를

```text
vx = 0
vy = 0
```

으로 예측했을 때의 error입니다.

## Evaluation Groups

다음 그룹별로 velocity metric을 계산합니다.

- All
- Static
- Moving
- Pedestrian
- Vehicle
- Obstacle

### Static / Moving Definition

GT object speed를 기준으로 구분합니다.

```text
Static : speed < 0.5 m/s
Moving : speed >= 0.5 m/s
```

## Distance Evaluation

거리별 velocity 성능도 출력합니다.

```text
0-10 m
10-20 m
20-30 m
30+ m
```

## Important Metric for Temporal Modeling

전체 mAVE는 static object가 많을 경우 매우 낮게 나타날 수 있습니다.

Temporal motion modeling 성능을 비교할 때는 특히 다음 지표를 확인합니다.

```text
Moving mAVE
Vehicle mAVE
Pedestrian mAVE
```

특히 Moving mAVE를 temporal motion estimation의 핵심 지표로 사용합니다.

## Current Reference Result

3-sweep PointPainting + velocity regression model  
T1 / Epoch 80 / Validation set / IoU >= 0.50

```text
All mAVE        : 0.2110 m/s
Static mAVE     : 0.0095 m/s
Moving mAVE     : 1.1708 m/s
Pedestrian mAVE : 0.3006 m/s
Vehicle mAVE    : 1.1050 m/s

Moving matched  : 355 / 362 (98.07%)
```

Moving object의 Zero-Velocity Baseline:

```text
6.1126 m/s
```

T1 Moving mAVE:

```text
1.1708 m/s
```

따라서 현재 temporal model은 zero-velocity prediction과 비교했을 때
moving object velocity를 실제로 학습하고 있음을 확인할 수 있습니다.

## Evaluation Note

본 evaluator에서 표시하는 mAVE는
MORAI validation dataset의 detection TP에 대해 계산한
velocity vector error입니다.

따라서 nuScenes 공식 TP metric의 AVE와
평가 protocol이 완전히 동일하다는 의미는 아닙니다.

논문 및 보고서에서는 본 문서에 정의된 metric을 기준으로
평가 방법을 명시해야 합니다.
