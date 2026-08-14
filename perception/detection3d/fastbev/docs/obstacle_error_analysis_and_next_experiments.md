# Fast-BEV obstacle 오류 분해와 다음 실험

## 기준 모델과 진단 방법

기준 모델은 OpenCV, BEV/image flip `0.5/0.5`, 3-class, three-anchor로 학습한
`full1500_stride1_fp16_opencv_flip50_50_v1`의 validation-selected epoch 12이다.
222-frame held-out test의 375개 obstacle을 분석했다.

- score threshold: validation epoch 12의 obstacle 1 m best-F1 값 `0.59619`
- matching: score 내림차순, frame별 GT-prediction 일대일 center-distance matching
- 형상: 기존 진단 taxonomy와 같은 compact/tall/long, ratio threshold `2.0`
- 거리: `0-10`, `10-20`, `20-30`, `30-50 m`
- mATE/mASE/mAOE: score threshold를 통과하고 2 m 안에서 매칭된 box의 기술 통계

이는 모델 선택용 test metric을 새로 정의한 것이 아니라 실패 원인을 분해하는
진단이다. 원본 metadata에는 MORAI asset 이름과 camera occlusion이 없으므로 현재
결과로 asset별 실패와 가림 정도는 계산할 수 없다. 다음 데이터 변환부터
`asset_name`, camera별 visibility/2D box 면적을 frame metadata에 보존해야 한다.
`num_lidar_pts`도 현재 converter가 유효 box에 상수 1을 넣으므로 진단값으로 쓰지
않는다.

## 오류 분해 결과

Validation threshold 이상 obstacle prediction은 342개였다. 2 m matching 기준
205개 GT가 매칭됐고 GT 170개가 누락됐으며 unmatched prediction은 137개였다.

### 형상별

| Shape | GT | Recall@0.5m | Recall@1m | Recall@2m | mATE | mASE | raw mAOE | π-sym mAOE |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Compact | 90 | 48.9% | 56.7% | 61.1% | 0.452 | 0.386 | 0.823 | 0.231 |
| Tall | 136 | 45.6% | 52.2% | 55.9% | 0.403 | 0.055 | 0.778 | 0.211 |
| Long | 149 | 36.9% | 46.3% | 49.7% | 0.424 | 0.347 | 0.432 | 0.198 |

Tall obstacle의 mASE `0.055`는 이미 좋다. 형상 다양성 전체가 동일하게 회귀를
실패하는 것이 아니라 compact와 long의 크기 회귀가 병목이다. 2 m 안에서 매칭된
long 74개 중 43개가 predicted size taxonomy상 compact로 축소됐고 long으로 남은
것은 31개뿐이었다.

| GT shape / predicted box shape | Compact | Tall | Long |
|---|---:|---:|---:|
| Compact | 50 | 2 | 3 |
| Tall | 0 | 76 | 0 |
| Long | 43 | 0 | 31 |

### 거리별

| Distance | GT | Recall@0.5m | Recall@1m | Recall@2m | mATE | mASE |
|---|---:|---:|---:|---:|---:|---:|
| 0-10 m | 147 | 67.3% | 72.8% | 75.5% | 0.285 | 0.246 |
| 10-20 m | 188 | 31.4% | 41.5% | 46.3% | 0.563 | 0.252 |
| 20-30 m | 40 | 7.5% | 15.0% | 17.5% | 0.903 | 0.263 |

크기 오차는 거리에 따라 급격히 악화되지 않지만 recall과 중심 오차는 크게
악화된다. 따라서 현재 최우선 표현 병목은 중·원거리 image-to-BEV 공간 해상도와
camera-only depth다. 20-30 m에서 compact는 Recall@0.5m `0%`, tall은 `14.3%`,
long은 `11.1%`였다.

### Yaw 해석

전체 matched obstacle의 raw mAOE는 `0.665 rad`지만 π 대칭을 인정하면
`0.212 rad`로 내려간다. 많은 obstacle box가 180도 반대로 예측됐을 뿐 점유
방향축 자체는 유사하다. 정적 obstacle 평가와 loss에는 asset별 방향 의미 또는
최소한 π-periodic yaw를 적용하는 것이 타당하다. 이는 실제 검출 recall 개선과는
별도 과제다.

## 결론

1. 하나의 obstacle class 자체가 불가능한 문제라는 증거는 없다. Tall 형상은 같은
   class 안에서도 크기를 정확히 회귀한다.
2. 주요 병목은 long을 compact로 축소하는 회귀, compact/long의 높은 mASE,
   10 m 이후 급격한 recall·중심 정확도 하락이다.
3. H1 five shared anchors는 이 병목을 개선하지 못했으므로 anchor 수를 늘리지 않는다.
4. H2 dedicated subclass는 현재 보류한다. 데이터/해상도와 class 유지형 loss를 먼저
   검증한다.

진단 산출물은 Git 밖 다음 경로에 둔다.

```text
artifacts/detection3d/fastbev/diagnostics/obstacle_baseline_v1/
├── evaluation_metrics.json
├── test_predictions.pkl
├── obstacle_error_summary.json
└── obstacle_error_summary.md
```

## 1순위 후속 과제: 데이터 확대

현재는 추가 수집이 필요하므로 구현을 미루되 반드시 수행한다.

- train 1,050 frame에서 우선 3,000-5,000 frame으로 확대
- compact/tall/long을 삭제하지 않고 scene quota로 균형화
- 특히 long과 compact의 asset/pose/거리 다양성 보강
- `0-10/10-20/20-30 m` 거리 quota를 형상별로 적용
- 같은 scene의 유사 연속 frame보다 서로 다른 scene 수를 우선
- 새 metadata에 asset 이름, camera visibility, 2D projected area 보존
- 기존 baseline과 architecture/config를 고정한 data-scale ablation 수행

## 2순위 계획: 관심 범위 기반 고해상도 BEV

현재 Fast-BEV volume은 `200 x 200 x 4`, horizontal voxel `0.5 m`, XY 범위
`[-50, 50] x [-50, 50] m`다. detection head stride는 이미 1이므로 더 줄일
stride는 없다.

초기 제안 `전방 40/후방 15/좌우 25 m`는 0.25 m에서 `220 x 200`으로 현재보다
XY cell이 10%만 늘어난다는 장점이 있다. 그러나 실제 split에서 vehicle의 약
20%가 후방 15 m 밖에 있어 끼어들기용 후방 인지를 지나치게 줄인다. 그대로
채택하지 않는다.

### 단계 A: 같은 연산량의 0.3 m grid

```text
point cloud range: x [-20, 40], y [-30, 30], z [-5, 3]
n_voxels:          [200, 200, 4]
voxel_size:        [0.3, 0.3, 1.5]
XY cells:          40,000 -> 40,000
```

Train GT 보존율은 vehicle `90.5%`, pedestrian `96.6%`, obstacle `98.0%`다.
Test obstacle은 100% 보존된다. 계산량을 거의 고정하면서 horizontal resolution을
40% 높이는 첫 smoke/12-epoch ablation이다. 다만 rear vehicle 9-10% 손실을
class/range metric에 명시해야 한다.

### 단계 B: 보존 범위를 넓힌 0.25 m grid

단계 A에서 obstacle의 10-30 m recall 또는 center/IoU가 개선되면 다음 후보를
비교한다.

```text
candidate B1: x [-20, 40], y [-30, 30], 240 x 240, 0.25 m
candidate B2: x [-30, 30], y [-25, 25], 240 x 200, 0.25 m
```

- B1: 전방 40 m와 좌우 30 m 유지, XY cell 44% 증가
- B2: 대부분의 현재 GT와 후방 30 m 유지, XY cell 20% 증가, 전방은 30 m 제한

대회 obstacle 회피와 후방 끼어들기를 함께 고려하면 A 이후 B1을 기본 후보로
삼고, FPS가 30 Hz 미만으로 내려갈 때 B2를 검토한다.

### 구현 및 검증 순서

1. 파생 config에서 `point_cloud_range`, `model.n_voxels`, `model.voxel_size`,
   anchor range를 함께 변경한다.
2. train/test pipeline의 `ObjectRangeFilter`와 `KittiSetOrigin`도 같은 range로
   변경한다. 비대칭 range의 중심이 Fast-BEV backprojection origin이 된다.
3. annotation 보존율 검사와 model tensor shape 정적 검증을 추가한다.
4. 한 batch forward/backward와 1-epoch smoke를 수행한다.
5. 동일 seed/data/augmentation으로 12 epochs 학습한다.
6. validation으로 NMS/score를 선택한 뒤 test를 한 번 평가한다.
7. 전체 metric 외에 이 문서의 shape-distance 오류 분해와 FPS/p95를 비교한다.

채택 조건은 baseline 대비 다음을 모두 만족하는 것이다.

- obstacle 10-20 m 및 20-30 m Recall@1m 개선
- obstacle center AP와 IoU AP@0.25 개선
- vehicle/pedestrian 전체 AP의 큰 회귀 없음
- online target 30 Hz를 감당할 inference throughput과 안정적인 p95

## 15 Hz VLP16 LiDAR fusion 판단

규정상 camera는 최대 30 Hz, VLP16 한 대는 최대 15 Hz이며 10 Hz 이하가
권장된다. LiDAR callback을 기준으로 모든 sensor를 동기화하는 monolithic
BEVFusion은 출력도 10-15 Hz로 제한된다. 60 km/h에서는 15 Hz scan 사이 Ego가
약 `1.11 m`, 30 Hz camera 사이에는 약 `0.56 m` 이동하므로 timestamp와 tracking
보상이 없으면 고속 구간 latency가 커진다.

그럼에도 LiDAR는 사용할 이유가 있다. 대회 규정은 임의 obstacle의 **각 객체별
크기**를 고려한 회피를 요구하며, 현재 진단에서 camera-only의 10 m 이후 중심
오차와 recall이 급격히 나빠졌다. 16채널이라도 가까운 정적 obstacle의 거리와
점유 폭을 보정하는 정보는 camera보다 직접적이다. 반면 작은 물체와 먼 물체에는
점이 없을 수 있으므로 semantic recall 전체를 해결하지는 못한다.

따라서 최종 추천은 `30 Hz camera pipeline을 15 Hz에 종속시키지 않는 dual-rate
late fusion`이다.

```text
Camera Fast-BEV detection:  최대 30 Hz 지속
LiDAR geometry correction: 10-15 Hz 도착 시 최근 detection의 depth/size 보정
Tracking/Prediction:        timestamp 기반으로 30 Hz 상태 예측·발행
```

대안으로 최근 LiDAR BEV feature를 ego-motion 보정해 다음 camera frame 한두 개에
재사용할 수 있지만 neural asynchronous fusion 구현 비용이 크다. 우선순위는
camera-only 데이터/BEV 개선 뒤이며, monolithic 15 Hz BEVFusion보다 detection
box와 LiDAR cluster를 결합하는 late fusion PoC가 먼저다.

채택 판단은 다음 두 조건으로 한다.

- 15 Hz 보정이 obstacle mATE/크기 오차와 collision-distance 안정성을 유의미하게
  개선하는가
- LiDAR가 없는 중간 camera frame에서 tracker propagation으로 30 Hz Planning
  interface를 유지할 수 있는가

즉 VLP16의 낮은 채널·주기 때문에 fusion을 포기할 필요는 없지만, LiDAR를 전체
perception clock으로 삼아서는 안 된다.

## 주요 설계 질문에 대한 확정 사항

### Pedestrian과 obstacle의 yaw 의미

MORAI pedestrian의 yaw가 실제 보행 방향과 일관되는 것을 육안으로 확인했으므로
pedestrian은 vehicle과 같이 `2π` 방향성을 유지한다. 즉 0도와 180도는 서로 다른
정답이며 기존 direction classification을 유지한다.

Obstacle은 Planning이 사용하는 충돌 점유 box 관점에서 앞뒤가 뒤집힌 `yaw + π`가
동일한 경우가 대부분이다. 현재 Fast-BEV의 yaw 회귀는 `sin(pred-target)` 차이를
사용해 연속 yaw 항은 이미 π 차이에 관대하지만, 별도의 2-bin direction
classification loss는 0도와 180도를 구분한다. 평가만 π 대칭으로 바꾸면 metric은
공정해지지만 학습 중 direction loss에는 불필요한 충돌이 남는다.

따라서 다음 정책을 사용한다.

| 대상 | yaw 주기 | 학습 정책 |
|---|---|---|
| Vehicle | `2π` | 기존 yaw regression + direction loss 유지 |
| Pedestrian | `2π` | 실제 보행 방향이므로 기존 loss 유지 |
| Long/tall obstacle | `π` | 점유 방향축은 학습하되 앞뒤 direction penalty 제거 |
| 회전 대칭 compact obstacle | yaw 무의미 | yaw/direction loss mask 후보 |

형상별 loss는 구현 가능하다. FreeAnchor loss에는 GT별 `gt_bboxes_`, `gt_labels_`와
각 GT에 매칭된 top-k anchor가 동시에 존재하므로 GT의 `w/l/h` 비율 또는 별도
metadata에서 symmetry type을 구한 뒤 다음과 같이 적용할 수 있다.

1. obstacle matched GT에 대해서만 direction CE를 mask한다.
2. 회전 대칭 compact asset에는 bbox yaw channel weight도 0으로 둔다.
3. long/tall에는 π-periodic axis yaw regression을 유지한다.
4. vehicle/pedestrian loss는 변경하지 않는다.

다만 stock `FreeAnchor3DHead` config만으로 GT별 weight를 줄 수는 없다. 현재
`loss_dir`와 box code loss가 positive bag probability로 합쳐지기 전에
shape/symmetry mask를 적용하는 `SymmetryAwareFreeAnchor3DHead` 같은 작은 custom
head가 필요하다. `w/l/h` 비율만으로 원통과 정육면체를 완전히 구별할 수 없으므로
최종적으로는 asset metadata에 `yaw_symmetry: directional | axis | none`을 저장하는
것이 가장 정확하다. Metadata가 없는 현재 1,500-frame dataset에서는 dimensions
기반 규칙으로 먼저 ablation할 수 있다.

### Symmetry-aware yaw ablation 결과 (2026-08-15)

동일한 1,500-frame split, OpenCV backend, BEV/image flip `0.5/0.5`, seed 0,
FP16, 12 epoch 조건에서 다음 세 실험을 수행했다.
여기서 H1-A/H1-B는 symmetry ablation의 실행명이며,
`obstacle_regression_roadmap.md`에서 이미 사용한 five-anchor H1과는 별개다.

| 실험 | 변경점 |
|---|---|
| C0 | 기존 3-class, shared 3-anchor loss |
| H1-A | obstacle의 2-bin direction CE만 mask |
| H1-B | H1-A + taxonomy와 무관하게 XY aspect ratio `<= 1.2`인 obstacle의 연속 yaw regression mask |

H1-B의 구현 조건은 높이를 보지 않는다. Train obstacle 1,521개 중 467개(30.7%)가
마스킹됐으며, 진단 taxonomy로 나누면 compact 297개, tall 127개, long 43개다.
따라서 이 실험은 원통형일 가능성이 있는 일부 tall도 이미 포함하지만 tall 전체의
yaw를 제거한 실험은 아니다.

222-frame held-out test 결과는 다음과 같다. 진단 yaw 값은 세 모델 모두 동일한
score threshold `0.59619`와 2 m center matching을 사용했다.

| 지표 | C0 | H1-A | H1-B |
|---|---:|---:|---:|
| 전체 center mAP | **0.5919** | 0.5897 | 0.5874 |
| obstacle center mean AP | 0.4194 | 0.4055 | **0.4240** |
| obstacle AP@0.5 m | 0.3104 | 0.2895 | **0.3181** |
| obstacle IoU AP@0.25 | **0.3412** | 0.3130 | 0.3271 |
| obstacle IoU AP@0.50 | **0.1020** | 0.0652 | 0.0558 |
| obstacle mATE | 0.3704 | 0.3632 | **0.3468** |
| obstacle mASE | **0.2379** | 0.2667 | 0.2597 |
| raw `2π` mAOE | **0.6655** | 1.4770 | 0.9402 |
| `π`-symmetric mAOE | 0.2116 | **0.1982** | 0.2002 |
| 2 m recall | **0.5467** | 0.5040 | 0.5120 |
| inference FPS | 41.48 | 41.90 | 42.18 |

결론은 다음과 같다.

- H1-A는 `π`-symmetric yaw만 `0.0134 rad` 개선했지만 obstacle mean AP,
  IoU AP, recall이 모두 하락했다. raw yaw가 크게 악화된 것은 direction supervision을
  제거한 실험의 예상된 결과지만, 검출·box 품질 개선으로 이어지지 않았다.
- H1-B는 center AP와 mATE는 소폭 개선했으나 IoU AP@0.50이 C0 대비 약 45% 낮고,
  mASE와 recall도 나빠졌다. XY가 정사각형에 가까운 obstacle 30.7%의 yaw
  supervision 제거가 shape fitting과 positive matching까지 불안정하게 만든 것으로
  본다.
- 속도 차이는 2% 이내라 loss mask의 실시간 연산 이득은 의미가 없다.
- 따라서 **현재 production checkpoint와 다음 실험의 대조군은 C0를 유지**한다.
  H1-A/H1-B는 채택하지 않으며, obstacle 평가는 raw mAOE와 함께
  `π`-symmetric mAOE를 진단 지표로만 병기한다.
- 향후 asset별 `yaw_symmetry` metadata가 확보되더라도 곧바로 production loss에
  적용하지 않고, 더 큰 데이터에서 독립 ablation으로 재검증한다.

### 현재 velocity GT와 `vx`, `vy` 회귀의 한계

현재 1,500-frame MORAI 변환 데이터는 각 객체의 실제 속도를 저장하지 않는다.
dataset info 생성기가 모든 GT box에 다음 값을 기록한다.

```python
gt_velocity = [[0.0, 0.0], ...]
```

Fast-BEV box code는 `[x, y, z, w, l, h, yaw, vx, vy]` 9개 연속값을
예측하므로, 이 데이터로 학습하면 동적 차량과 보행자를 포함한 모든 객체의
`vx`, `vy` 정답이 0인 것으로 간주된다. 현재 config는 마지막 두 velocity code에
각각 `0.2` weight를 주기 때문에 velocity loss가 완전히 빠진 것도 아니다. 결과적으로
모델은 `vx`, `vy`를 실제 속도가 아니라 **0에 가깝게 회귀하도록 학습**된다.

이 문제의 의미는 다음과 같다.

- 현재 정적 데이터에서 velocity residual은 빠르게 작아지므로 전체 학습 loss를 크게
  지배할 가능성은 낮다.
- 그러나 추론된 `vx`, `vy`는 실제 동적 객체 속도로 해석할 수 없다.
- 현행 ROS adapter는 Fast-BEV box의 `vx`, `vy`를 `DetectedObjects`에 싣지 않고
  `has_twist=false`로 발행한다.
- 현행 EKF tracker의 measurement는 detection의 `x`, `y`, `yaw`뿐이며, tracker
  state의 `vx`, `vy`는 연속 detection 위치 변화와 motion model을 통해 자체 추정한다.
- 따라서 현재 T&P는 잘못 학습된 Fast-BEV velocity를 직접 사용하지 않는다. 하지만
  향후 detector velocity를 tracker 초기값·measurement로 쓰려면 실제 frame-level
  actor velocity GT를 수집하거나 velocity channels/loss를 비활성화한 뒤 다시
  학습해야 한다.

다음 데이터셋에서는 MORAI actor state와 캡처 timestamp를 함께 저장해 실제 ego-frame
`vx`, `vy`를 만들고, temporal sequence의 object identity까지 유지하는 것이 원칙이다.
그 전까지는 현재 checkpoint의 velocity 출력을 downstream contract에 노출하지 않는다.

### Fast-BEV에서 voxel이라는 용어의 의미

`voxel`이라는 용어는 맞다. Fast-BEV 논문과 원본 코드가 multi-view 2D image
feature를 `3D voxel space`로 backprojection한다고 정의하고, 실제 구현도
`n_voxels`, `voxel_size`, `get_points()`라는 이름을 사용한다. 다만 이는 LiDAR
point를 binning하는 point-cloud voxelization이 아니라 **카메라 특징을 샘플링하는
고정 3D feature voxel/grid**다.

현재 설정은 다음 순서로 동작한다.

```text
4-camera image features
  -> 200 x 200 x 4개의 3D sampling point 생성
  -> 각 point를 camera feature map에 투영하여 3D feature volume 생성
  -> 3D/BEV encoder
  -> 각 BEV head cell에서 anchors와 class/box residual 예측
  -> rotated NMS
```

YOLO grid와는 `공간 cell마다 후보를 예측한다`는 점만 비슷하다. 차이는 Fast-BEV
cell feature가 원근 image를 단순 분할한 것이 아니라 calibration으로 3D 공간에
투영된 multi-camera feature라는 점이다. Anchor가 놓이는 단위는 최종 BEV feature
cell이고, 현재 head stride가 1이므로 대략 각 0.5 m XY cell마다 3 sizes x 2 yaw,
총 6개 anchor 후보가 있다.

0.5 m는 하드 출력 양자화가 아니다.

- center는 anchor로부터 continuous offset을 회귀하므로 sub-cell 위치가 가능하다.
- width/length/height도 continuous scale을 회귀하므로 0.5 m보다 작은 box가 가능하다.
- 한 cell에도 여러 anchor 후보가 있으므로 엄밀히 객체 하나로 제한되지 않는다.
- 그러나 같은 cell 후보들이 coarse feature를 공유하므로 작은 객체, 가까운 두 객체,
  정확한 center 회귀에는 0.5 m feature resolution이 병목이 될 수 있다.

### 균일 고해상도와 가변 해상도

현재 구현에서 voxel spacing은 한 volume 안에서 균일하다. `0.3 m grid` 후보는 선택한
관심 범위 전체가 0.3 m이고, 범위 밖은 `ObjectRangeFilter`와 detection range 밖이라
검출하지 않는다. 근거리 0.3 m, 원거리 0.5 m를 동시에 쓰는 non-uniform grid는
현재 구현되어 있지 않으며 multi-resolution branch 또는 별도 refinement가 필요하다.

100 x 100 m 전체를 0.3 m로 만들면 XY cell이 40,000개에서 약 111,556개로 2.8배
증가한다. 30 Hz 목표를 고려하면 먼저 관심 범위를 줄이는 균일 grid를 실험한다.

기존 단계 A 외에 후방 coverage를 보존하는 다음 후보도 추가한다.

```text
candidate A-rear30:
  range:       x [-30, 40], y [-30, 30]
  n_voxels:    [224, 192, 4]
  voxel_size:  [0.3125, 0.3125, 1.5]
  XY cells:    43,008 (현재 대비 +7.5%)
```

이는 전방 40 m, 후방 30 m, 좌우 30 m를 유지하면서 약 0.31 m 해상도를 확보한다.
비정사각형 tensor와 비대칭 origin은 코드상 가능하지만 model shape, anchor range,
memory, FPS를 smoke test로 확인한 뒤 채택한다.

### VLP16의 10 m 이후 역할

VLP16은 10 m 이후 무용하지 않다. 약 2도인 수직 beam 간격 때문에 인접 beam의
높이 간격은 10 m에서 약 0.35 m, 20 m에서 0.70 m, 30 m에서 1.05 m가 된다.
따라서 작은·낮은 obstacle의 원거리 standalone detection은 불안정하지만, 점이 맞은
vehicle/tall/long obstacle의 직접 거리와 점유 폭은 camera-only depth보다 유용할 수
있다.

첫 fusion PoC는 별도 LiDAR detector보다 Fast-BEV proposal 기반 geometry
refinement를 우선한다.

```text
Fast-BEV box
  -> timestamp/ego-motion으로 LiDAR 정렬
  -> 확장 box 또는 frustum 안의 점 선택
  -> ground 제거와 point-count gate
  -> 충분한 점이 있을 때만 depth/center/size 보정
  -> 점이 없으면 원래 camera box 유지
```

이 방식은 LiDAR가 class를 분류하거나 독립적으로 객체를 검출할 필요가 없다. 다만
Fast-BEV가 객체를 완전히 누락한 경우에는 복구할 수 없으므로, 실제 point-hit 통계와
LiDAR-only recall이 충분할 때만 PointPillars 같은 별도 detector를 후속 검토한다.

### Dual-rate late fusion 구조

연구 모델은 joint feature를 학습할 수 있는 mid-level/BEV fusion을 많이 사용하지만,
우리 센서는 camera 최대 30 Hz와 LiDAR 10-15 Hz로 주기가 다르다. 단일 BEVFusion
callback을 LiDAR에 맞추면 camera measurement도 10-15 Hz에 종속된다. 우선 다음
비동기 구조를 사용한다.

```text
Fast-BEV DetectedObjects:       camera frame마다 갱신
LiDAR geometry correction:      scan 도착 시에만 선택적 갱신
Tracker/PredictedObjects:        timestamp 기반 일정 주기로 예측·발행
```

구현 후보는 세 단계다.

1. proposal-conditioned raw-point refinement: 별도 LiDAR detector 없음, 첫 추천
2. track-level asynchronous update: camera와 LiDAR correction을 EKF에 별도 measurement로 입력
3. detection-level fusion: Fast-BEV와 PointPillars box를 association, 필요할 때만 도입

Autoware의 object/tracking merger는 association, Mahalanobis/IoU gate, 서로 다른 sensor
rate 처리의 참고 구현으로 사용할 수 있지만 현재 ROS1 pipeline에 바로 끼우는 완제품은
아니다. 우리 표준 `DetectedObjects -> TrackedObjects -> PredictedObjects` 계약 사이에
adapter/refiner node를 구현하는 방식이 적합하다.

### Tall 형상과 center 오차 해석

mATE는 box 크기로 정규화하지 않은 center-to-center 거리이므로 객체가 얇거나 작다고
center 오차의 가능한 범위가 자동으로 작아지지 않는다. 현재 tall mATE `0.403 m`는
compact `0.452 m`, long `0.424 m`와 비슷하다. Tall의 뚜렷한 장점은 mATE가 아니라
mASE `0.055`이며, 이는 tall asset의 크기·외형 분포가 비교적 일정한 결과로 본다.

Vehicle center 오차가 더 크다면 크기 자체보다 거리 분포, occlusion, visible surface와
실제 3D center의 차이를 먼저 의심한다. Class별 원인을 비교할 때는 반드시 같은
`0-10/10-20/20-30 m` bin에서 비교한다.

### 대회 기준 후방 detection 범위

대회의 회전교차로·합류 끼어들기 때문에 후방 차량을 제거할 수는 없다. 권장 중요도는
다음과 같다.

- rear 0-15 m: 정확한 box, velocity, track이 중요
- rear 15-20 m: 끼어들기 판단 핵심 범위
- rear 20-30 m: 접근 차량 조기 track용, 존재와 상대속도 우선
- rear 30 m 초과: 검출되면 좋지만 대회 우선순위는 낮음

기존 데이터에서 vehicle 약 20%가 rear 15 m 밖, 약 9-10%가 rear 20 m 밖에 있다.
따라서 rear 15 m cutoff는 과도하고, 최소 rear 20 m accuracy와 rear 30 m coverage를
목표로 한다. Planning에서는 고정 거리보다 TTC와 상대속도로 위험도를 판단한다.

## 추가 데이터 수집을 제외한 실행 우선순위

추가 데이터 수집은 장기 필수 과제로 유지하되, 현재 보유 데이터와 rosbag만으로는
아래 순서로 진행한다.

1. **Symmetry-aware yaw 진단·loss ablation: 완료, production 미채택**
   - H1-A/H1-B 모두 `π`-symmetric mAOE는 소폭 개선
   - IoU AP와 recall 손실 때문에 C0 유지
2. **관심 범위 고해상도 BEV: 다음 실험**
   - 먼저 `0.3 m, rear20, 200x200`과 `0.3125 m, rear30, 224x192` 정적 검증
   - 1-epoch smoke 후 동일 조건 12-epoch 비교
   - 거리별 recall/mATE, IoU, FPS, p95로 선택
3. **Position-oriented obstacle loss**
   - 기존 3-class/3-anchor 유지
   - obstacle x/y center L1 가중치를 높이는 최소 ablation
   - size/yaw와 positive-bag 균형 붕괴 여부 확인
4. **Auxiliary shape head**
   - 최종 class는 obstacle 유지
   - obstacle positive에만 compact/tall/long auxiliary supervision
   - 효과가 있을 때만 soft regression expert 검토
5. **Anchor-free CenterHead 타당성·smoke**
   - 고정 anchor 경쟁을 제거하고 BEV center heatmap을 직접 학습
   - 위의 작은 변경들이 실패할 때 본 실험 진행
6. **기존 LiDAR rosbag point-hit upper-bound 분석**
   - GT box별 거리·형상별 LiDAR point 수와 빈 box 비율 계산
   - GT 주변 point cluster로 얻을 수 있는 center/size 보정 상한 측정
7. **Dual-rate raw-LiDAR geometry refinement PoC**
   - 6번에서 10-20 m 개선 가능성이 확인될 때만 구현
   - 별도 LiDAR detector 없이 시작
8. **Temporal smoothing 검증**
   - 정적 obstacle velocity prior와 EKF 평활화
   - 단일-frame metric과 Planning 입력 jitter를 함께 비교

즉 symmetry-aware yaw 실험은 끝났고 현재 다음 구현 1순위는 두 고해상도 BEV 후보
비교다. LiDAR fusion은 먼저 기존 rosbag의 point-hit upper bound를 계산해 효과
가능성을 확인한 뒤 구현한다.

## 참고 자료

- Fast-BEV 논문: <https://arxiv.org/abs/2301.12511>
- Ouster/Velodyne VLP16 공식 자료: <https://ouster.com/ja/downloads/velodyne-downloads>
- Autoware object merger: <https://autowarefoundation.github.io/autoware_universe/latest/perception/autoware_object_merger/>
- Autoware tracking object merger: <https://autowarefoundation.github.io/autoware.universe_planning/pr-5553/perception/tracking_object_merger/>
- Autoware image projection fusion: <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/>
- CLOCs 공식 구현: <https://github.com/pangsu0613/CLOCs>
