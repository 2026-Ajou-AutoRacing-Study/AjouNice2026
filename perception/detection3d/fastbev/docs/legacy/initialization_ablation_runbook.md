# Fast-BEV 공식 가중치 초기화 ablation

## 목적

역사적 대조군 `C0`는 ImageNet ResNet-18에서 시작한다. 1차 실험은 데이터,
증강, seed, 12 epoch를 고정하고 공식 사전학습 가중치의 초기화 범위만 바꿔 비교한다.
학습 실행 전 원본 체크포인트를 목표 MORAI 모델과 key·shape 단위로 대조하고, 호환되는
모듈만 별도 체크포인트로 만든다.

## 준비된 후보

| 후보 | 원본 | 실제 로드 범위 | target parameter coverage |
| --- | --- | --- | ---: |
| I0 | 기존 C0 | ImageNet ResNet-18 | 대조군 |
| I1 | COCO→nuImages Cascade Mask R-CNN R18/FPN | ResNet-18 backbone | 73.37% |
| I2 | nuScenes Fast-BEV M0 epoch 20 | backbone + 64-channel FPN | 74.75% |
| I3 | nuScenes Fast-BEV M0 epoch 20 | backbone + FPN + 호환 BEV neck | 99.46% |

I1 원본 FPN은 256 channel이고 MORAI Fast-BEV M0 FPN은 64 channel이므로 I1에서
FPN은 로드하지 않는다. I3의 temporal fuse weight는 원본 4-frame 입력과 현재
single-frame 입력의 shape이 다르다. 준비 도구는 이 weight뿐 아니라 같은 layer의
bias도 함께 제외해 학습된 적 없는 혼합 layer가 생기지 않게 한다. 10-class detection
head도 모두 제외하고 MORAI 3-class head를 새로 학습한다.

## 다운로드와 무결성

원본과 생성물은 Git에 넣지 않고 아래 경로에 둔다.

```text
artifacts/detection3d/fastbev/pretrained/
├── cascade_mask_rcnn_r18_fpn_coco-mstrain_3x_20e_nuim_bbox_mAP_0.5110_segm_mAP_0.4070.pth
├── fastbev_m0_r18_s256x704_v200x200x4_c192_d2_f4_epoch20_nuscenes.pth
├── prepared/
│   ├── i1_nuimages_backbone.pth
│   ├── i2_nuscenes_image.pth
│   └── i3_nuscenes_image_bev.pth
└── reports/
    ├── i1_nuimages_backbone.json
    ├── i2_nuscenes_image.json
    ├── i3_nuscenes_image_bev.json
    └── sha256sums.txt
```

다른 PC에서 같은 준비를 재현하려면 다음 두 명령을 실행한다.

```bash
cd ~/AjouNice2026
perception/detection3d/fastbev/scripts/download_initialization_weights.sh
perception/detection3d/fastbev/scripts/prepare_initialization_experiments.sh
```

다운로드 스크립트는 공식 Fast-BEV Google Drive file ID를 사용하고 고정 SHA-256을
검증한다. 준비 도구는 각 실험에서 실제로 선택·제외된 tensor와 family별 coverage를
JSON으로 남긴다.

## 1차 실행

아래 명령은 I1, I2, I3을 순서대로 각각 seed 0, 12 epoch 학습하며 W&B group
`sunny13-1500-initialization-phase1`에 기록한다. 실수로 시작하지 않도록
`--execute`가 없으면 학습하지 않는다.

```bash
cd ~/AjouNice2026
perception/detection3d/fastbev/scripts/run_initialization_phase1.sh --execute
```

각 run은 매 epoch validation을 수행하고 기존 설정의 `center_mAP` 기준 best
checkpoint를 저장한다. 후보 비교 중에는 test split을 보지 않는다. I1-I3의
validation 결과를 C0와 비교해 다음 단계로 넘길 후보를 고른 뒤, 선택된 후보만
seed 1/2 및 24 epoch 재검증과 held-out test 평가를 수행한다.

## 준비 완료 조건

- 두 공식 원본 체크포인트 SHA-256 검증 통과
- I1/I2/I3 shape-safe 체크포인트와 audit JSON 생성
- 각 config가 해당 체크포인트를 실제 `load_from`으로 로드
- 세 config 모두 실제 MORAI train batch CUDA forward smoke 통과
- 준비 단계에서는 본 학습과 W&B run을 시작하지 않고 별도 승인 후 실행

## Seed 0, 12 epoch 결과

2026-08-15에 I1-I3을 같은 split, augmentation, seed와 12 epoch로 실행했다. 후보
선택에는 validation만 사용했고 세 run 모두 epoch 12가 best였다.

| 후보 | val center mAP | vehicle | pedestrian | obstacle |
| --- | ---: | ---: | ---: | ---: |
| C0 | 0.5943 | 0.6790 | 0.6758 | 0.4279 |
| I1 | 0.6078 | 0.6917 | 0.6911 | 0.4405 |
| I2 | 0.6075 | 0.6989 | 0.7018 | 0.4218 |
| I3 | **0.6331** | **0.7198** | **0.7040** | **0.4754** |

Validation으로 I3을 선택한 뒤에만 held-out test 222 frames를 한 번 평가했다.

| test metric | C0 | I3 | 절대 변화 |
| --- | ---: | ---: | ---: |
| center mAP | 0.5919 | **0.6459** | +0.0540 |
| vehicle mean AP | 0.6938 | **0.7523** | +0.0585 |
| pedestrian mean AP | 0.6625 | **0.7013** | +0.0389 |
| obstacle mean AP | 0.4194 | **0.4839** | +0.0645 |
| IoU mAP@0.25 | 0.5744 | **0.6014** | +0.0270 |
| IoU mAP@0.50 | 0.2751 | **0.2879** | +0.0128 |

거리별로는 I3이 10-20 m의 세 class와 20-30 m의 세 class를 모두 개선했다.
특히 vehicle 20-30 m는 `0.4522 -> 0.5679`, pedestrian 20-30 m는
`0.0502 -> 0.0997`이다. 반면 vehicle 0-10 m는 `0.7791 -> 0.6847`로
하락했으므로 다음 재현 실험에서 반드시 확인한다.

I3은 obstacle mATE `0.3704 -> 0.3482`, mASE `0.2379 -> 0.2133`도
개선했다. Pedestrian과 obstacle raw mAOE는 악화됐지만 두 class의 orientation GT와
형상 대칭성 문제가 이미 확인되어 있어 production 선택의 단독 기준으로 쓰지 않는다.

결론은 **I3을 seed 0의 승자이자 다음 재현 후보로 선택하되 아직 production C0를
교체하지 않는 것**이다. 다음 gate는 I3 seed 1/2 재현이며, 평균 개선이 유지되면
24 epoch 실험으로 넘어간다. Test 결과는 아래에 저장됐다.

```text
artifacts/detection3d/fastbev/runs/init_i3_nuscenes_bev_seed0_12e/
├── best_center_mAP_epoch_12.pth
├── test_metrics.json
└── test_evaluation.log
```

## I3 seed 1 재현 결과

I3의 seed 0 상승이 우연인지 확인하기 위해 seed만 1로 바꾸고 같은 12 epoch
validation을 반복했다. Test split은 다시 사용하지 않았다.

| validation metric | C0 seed 0 | I3 seed 0 | I3 seed 1 | I3 두 seed 평균 |
| --- | ---: | ---: | ---: | ---: |
| center mAP | 0.5943 | 0.6331 | **0.6388** | **0.6360** |
| vehicle mean AP | 0.6790 | 0.7198 | 0.7198 | **0.7198** |
| pedestrian mean AP | 0.6758 | 0.7040 | 0.7058 | **0.7049** |
| obstacle mean AP | 0.4279 | 0.4754 | **0.4909** | **0.4832** |
| IoU mAP@0.25 | 0.5542 | 0.6090 | 0.5985 | **0.6038** |
| IoU mAP@0.50 | 0.2331 | 0.2819 | 0.2657 | **0.2738** |

두 I3 seed의 center mAP 차이는 0.0057에 불과하고, 둘 다 C0 seed 0보다
0.0388 이상 높다. Vehicle/pedestrian AP는 거의 동일하고 obstacle도 두 seed 모두
C0보다 높다. Obstacle mATE도 `0.3571`에서 각각 `0.2912`, `0.3004`로 반복
개선됐다. 따라서 I3 seed 0의 전체 개선은 단순 seed 행운일 가능성이 낮다.

다만 완전한 paired causal comparison에는 C0 seed 1도 필요하다. 또한 validation의
obstacle 20-30 m AP는 I3 두 seed 모두 C0보다 낮아 원거리 소표본 문제는 해결되지
않았다. Raw yaw 지표도 seed 편차가 크므로 기존 방침대로 보조 진단으로만 쓴다.

## 기준 전환 결정

2026-08-15부터 **새 Fast-BEV 실험의 기본 초기화는 I3**이다. Seed 0과 seed 1에서
center mAP 및 obstacle mATE 개선이 반복됐고, seed 0 held-out test에서도 C0보다
center mAP `+0.0540`을 확인했기 때문이다.

- `scripts/train.sh`와 `scripts/smoke_train.py`의 기본 config: I3
- 기본 비교 기준: I3, OpenCV, BEV/image flip `0.5/0.5`, FP16, 12 epochs
- C0: 역사적 결과 비교와 rollback 재현에만 사용
- I1/I2: 초기화 범위 ablation 기록으로 보존
- ROS 실시간 배포 checkpoint: 별도 latency·출력 검증 전까지 자동 교체하지 않음

즉 이후 Dataset V2, backbone capacity, 고해상도 image feature 및 LiDAR 실험은
특별한 이유가 없으면 I3에서 시작한다. 실험 보고서에는 I3 seed 0/1 중 사용한 seed와
checkpoint를 명시한다.
