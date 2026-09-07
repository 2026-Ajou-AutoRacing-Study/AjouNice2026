# Base3 segmentation 학습 기록.

마지막 갱신: 2026-08-30.

이 문서는 Base3-Static 10K의 LRASPP 4-class 학습 계획, 데이터 준비, smoke test,
본학습 결과와 후속 online 검증을 한곳에 누적한다. 1--3 epoch 파이프라인 확인 run은
결과 리더보드에 올리지 않고 smoke 항목에만 기록한다.

## 목표와 가설.

- Base2 LRASPP는 offline mIoU가 높았지만 실제 로터리·고주로 Vehicle semantic을 대부분
  Background로 분류했다.
- 기존 positive scene을 반복 sampling하는 것만으로는 해결되지 않았다.
- Base3는 Vehicle/Pedestrian/Obstacle의 asset, 거리, yaw, camera view와 route background를
  새로 수집했으므로 semantic painting의 online domain gap을 줄일 가능성이 있다.
- 먼저 LRASPP 자체를 검증하고, online semantic gate를 통과한 checkpoint만 PointPainting
  재학습에 사용한다.

## 고정 데이터 계약.

- Dataset: `Base3-Static 10K`.
- 3,634 clips, 10,902 independent single frames.
- Train/Val/Test: 2,544/545/545 clips.
- 4 camera 전체를 사용하므로 43,608 segmentation samples다.
- 원본은 `_Intensity.png`, GT는 `_Semantic.png`다.
- `_Depth.png`, `_Instance.png`는 학습에 쓰지 않으며 최종 source에서 삭제했다.
- split은 frame이 아니라 clip 단위다. 같은 actor layout의 세 frame은 항상 같은 split에
  남는다.
- 최종 선택 파일은
  `dataset/morai_od/configs/base3_static_lraspp_selection_v1.json`이다.
- rejected run과 superseded static-motion clip은 selection 단계에서 제외한다. capture
  directory 전체 recursive glob은 금지한다.

## Semantic GT 계약.

| Class ID | Class | MORAI RGB |
| ---: | --- | --- |
| 0 | Background | 아래 세 foreground palette 외 모든 색 |
| 1 | Pedestrian | `(98, 2, 255)` |
| 2 | Vehicle | generic `(255, 2, 2)`, Sedan `(255, 60, 60)`, SUV `(255, 75, 75)`, Truck `(255, 90, 90)`, Bus `(255, 105, 105)`, Wagon `(255, 135, 135)`, MPV `(255, 150, 150)` |
| 3 | Obstacle | `(236, 255, 2)` |

- RGBA source는 RGB로 변환해 alpha를 버린 뒤 exact palette matching한다.
- mask resize는 nearest-neighbor만 사용한다.
- batch padding은 class ID 255이며 CrossEntropyLoss와 metric에서 ignore한다.
- 2026-08-30 Base3 800개 image/1,170개 2D BBox 표본 audit에서 모든 BBox 영역에 기대
  class pixel이 존재했다. BBox 중심 일치율 89.9%는 occlusion과 box 여백 때문에 더 낮다.
- 최초 Base2 mapper는 generic Vehicle과 Bus 색만 포함해 Base3의 다양한 Vehicle subtype을
  Background로 잘못 바꿨다. Base3 9,191개 Vehicle-family BBox image를 전수 검사해 subtype별
  중심 RGB가 95.2--99.5% 일관됨을 확인하고 위 palette를 복구했다. Sedan/SUV/Truck/Wagon/MPV
  색이 빠진 mask와 audit은 폐기하고 `--overwrite`로 전량 재생성한다.

## 데이터 준비.

```bash
cd ~/AjouNice2026

source ~/miniconda3/etc/profile.d/conda.sh
conda activate openpcdet_rtx50

python tools/segmentation/prepare_base3_lraspp_4class.py \
  --capture-root /home/luv/morai_cap/260829_034057_base3_static_10k \
  --selection dataset/morai_od/configs/base3_static_lraspp_selection_v1.json \
  --output-root artifacts/datasets/morai_base3_static_10k_lraspp_4class_v1
```

산출물은 class-ID mask, `train.csv`, `val.csv`, `test.csv`, split scene 목록과
`audit.json`이다. 원본 image는 복사하지 않는다.

### 2026-08-30 생성 결과.

| Split | Clips | Frames | Camera samples |
| --- | ---: | ---: | ---: |
| Train | 2,544 | 7,632 | 30,528 |
| Val | 545 | 1,635 | 6,540 |
| Test | 545 | 1,635 | 6,540 |
| 합계 | 3,634 | 10,902 | 43,608 |

- 모든 clip이 정확히 `3 frames × 4 cameras = 12 samples`다.
- image/mask 누락, 중복 image, split scene leakage, rejected-run 유입은 모두 0이다.
- class-ID mask artifact는 43,608장, 278 MiB다.
- Batch008의 retry 전 frame이 camera directory에 남아 한 scene에 glob 결과가 4장이 되는
  사례를 찾았다. builder는 glob을 폐기하고 각 clip의 `frames.jsonl[].new_files`에 기록된
  정확한 세 frame만 사용한다.

Train의 최종 자연분포는 다음과 같다. 한 image에 여러 foreground class가 동시에 있을 수
있으므로 positive 비율 합은 100%일 필요가 없다.

| Source pool | Positive images | Train 전체 비율 | Eligible clips |
| --- | ---: | ---: | ---: |
| Vehicle | 6,548 | 21.45% | 934 |
| Pedestrian | 4,214 | 13.80% | 556 |
| Obstacle | 7,354 | 24.09% | 706 |
| Empty | 12,412 | 40.66% | 2,230 |

최초 두 색짜리 Vehicle palette에서는 Vehicle-positive가 잘못 `1,176장/3.85%`로
집계됐다. subtype palette 복구 후 `6,548장/21.45%`가 되어 Base3의 class-centric 수집
의도가 실제 segmentation target에도 반영됐다.

## 학습 계획.

- Architecture: torchvision LRASPP-MobileNetV3-Large, VOC pretrained 초기값.
- 4 classes: Background/Pedestrian/Vehicle/Obstacle.
- Input: short side 384, aspect ratio 유지.
- Optimizer: AdamW, LR `3e-4`, weight decay `1e-4`.
- Scheduler: 1-epoch linear warm-up 후 cosine annealing, 총 30 epochs,
  minimum LR `3e-6`.
- Batch size: 12, workers: 6, seed: 42.
- Geometric/photometric augmentation: 사용하지 않음.
- Validation: 매 epoch 수행한다.
  - `best_foreground_miou.pth`: PointPainting 채택의 primary checkpoint.
  - `best_4class_miou.pth`: 일반 segmentation 비교용 checkpoint.
- AMP 사용.
- 매 epoch 전체 Val과 W&B logging을 수행한다.
- W&B는 `epoch`을 공통 X축으로 사용하고 다음을 기록한다.
  - Train loss, 실제 사용 LR, images/s, peak CUDA memory.
  - Val loss, pixel accuracy, 4-class mIoU, foreground-only mIoU.
  - Background/Pedestrian/Vehicle/Obstacle별 IoU, precision, recall, Dice.
  - True class와 predicted class의 pixel count 및 row-normalized confusion table.
- 첫 baseline은 Base3 자연분포와 ENet pixel weight를 사용한다.
- Base3의 자연 positive-image 분포는 Vehicle/Pedestrian/Obstacle/Empty
  `21.45/13.80/24.09/40.66%`로 이미 충분히 균형적이다.
- ENet weight까지 적용한 상태에서 class-aware sampling을 동시에 쓰면 dataset 자체 효과와
  resampling 효과가 섞인다. 첫 run은 Base2 natural E30과 같은 최적화 조건을 사용해 Base3
  dataset과 수정 palette의 효과를 분리한다.
- class-aware `25/20/25/30%`는 첫 run의 Pedestrian 또는 Vehicle recall이 부족할 때만 두 번째
  ablation으로 수행한다.
- 본학습 전 1 train batch + 1 val batch smoke를 통과해야 한다.

## 실행 결과.

아직 본학습 전이다.

- Dataset selection contract: pass.
- Train/Val/Test sample 수: 30,528/6,540/6,540.
- Class-positive image 비율: 위 데이터 준비 표와 같음.
- Unit test: 3 passed.
- Loader/model GPU smoke: pass.
  - GPU: RTX 5060 Ti.
  - 최종 설정: natural sampling, ENet weight, batch size 12.
  - Tensor: image `(12, 3, 384, 683)`, target `(12, 384, 683)`, output
    `(12, 4, 384, 683)`.
  - 1 train batch forward/backward와 1 val batch metric 계산, checkpoint 저장을 완료했다.
  - ENet loss weight는 Background/Pedestrian/Vehicle/Obstacle
    `0.0415/1.4005/1.2382/1.3197`이다.
  - smoke의 IoU는 무작위에 가까운 초기 모델을 한 batch만 갱신한 값이므로 성능 지표로
    사용하지 않는다.
- W&B metric smoke: pass.
  - 새 class별 precision/recall/Dice, foreground mIoU, 처리량, peak VRAM 및 confusion table이
    한 epoch record로 정상 직렬화됐다.
- 본학습 checkpoint: pending.
- Offline best epoch와 class IoU: pending.
- Online rosbag semantic audit: pending.
- PointPainting 재학습 채택 여부: pending.

### Epoch 3 중간 결과와 resume.

- Epoch 3: 4-class mIoU `0.7342`, foreground mIoU `0.6465`.
- Background/Pedestrian/Vehicle/Obstacle IoU:
  `0.9970/0.5833/0.8134/0.5428`.
- Epoch 3은 두 기준 모두 현재 best라 기존 `best_miou.pth`를 두 명시적 checkpoint로
  이관했다.
- 중단된 `last.pth`에는 model, optimizer, scheduler가 있으므로 Epoch 4부터 resume한다.
- 기존 checkpoint는 scheduler step 이전 저장 형식이다. Resume loader가 compatibility step을
  한 번 적용해 Epoch 4 LR `2.965282e-4`를 복구한다.
- W&B run id `9fdubfb6`를 이어서 사용한다.

## 본학습 명령.

Smoke와 audit을 통과한 뒤 실행한다. 이 문서 작성 단계에서는 실행하지 않는다.

```bash
cd ~/AjouNice2026
source ~/miniconda3/etc/profile.d/conda.sh
conda activate openpcdet_rtx50

OUT=artifacts/detection3d/pointpainting/training/base3_static_lraspp4class_natural_enet_bs12_e30
mkdir -p "$OUT"

python tools/segmentation/train_lraspp_morai_4class_v2.py \
  --dataset-root / \
  --train-csv artifacts/datasets/morai_base3_static_10k_lraspp_4class_v1/train.csv \
  --val-csv artifacts/datasets/morai_base3_static_10k_lraspp_4class_v1/val.csv \
  --output-dir "$OUT" \
  --epochs 30 \
  --batch-size 12 \
  --workers 6 \
  --short-side 384 \
  --lr 3e-4 \
  --weight-decay 1e-4 \
  --class-weight-mode enet \
  --scheduler cosine \
  --warmup-epochs 1 \
  --sampling-mode natural \
  --checkpoint-interval 5 \
  --amp \
  --wandb-project morai-3d-object-detection \
  --wandb-run-name base3-static-lraspp4class-natural-enet-bs12-e30 \
  --wandb-mode online \
  2>&1 | tee "$OUT/train.log"
```

Epoch 3 중단점에서 이어갈 때는 같은 명령에 다음을 추가한다.

```text
--resume "$OUT/last.pth"
--wandb-run-id 9fdubfb6
```

RTX 5060 Ti에서 batch 순서를 무작위화하고, 각 run의 첫 20 iteration을 제외한 100 iteration을
3회 반복했다. 다음 값은 end-to-end 처리량의 중앙값이다.

| Batch | Throughput | Compute-only | Peak CUDA memory |
| ---: | ---: | ---: | ---: |
| 8 | **154.1 image/s** | **164.7 image/s** | 1.33 GiB |
| 12 | 153.3 image/s | 160.8 image/s | 1.97 GiB |
| 16 | 153.6 image/s | 159.8 image/s | 2.60 GiB |
| 24 | 151.0 image/s | 157.1 image/s | 3.88 GiB |
| 32 | 146.5 image/s | 151.5 image/s | 5.13 GiB |

DataLoader wait는 모든 설정에서 step당 `0.16--0.23ms`라 병목이 아니다. LRASPP-MobileNetV3의
depthwise convolution은 batch 8에서 이미 병렬성이 충분하고, 이후에는 working set과 memory
traffic이 커져 image당 GPU compute가 조금씩 느려진다. 최초 단발 측정에서 큰 batch가 절반까지
느려 보인 것은 CUDA/cuDNN warm-up 시간을 측정에 포함한 오류였다.

Batch 8/12/16의 처리량 차이는 0.5% 이내다. 첫 Base3 baseline은 Base2 natural run과 같은
batch 12와 LR `3e-4`를 유지해 비교 가능성과 기존 검증값을 보존한다. Batch 24/32는 VRAM만 더
사용하고 처리량·일반화·wall time 어느 쪽에도 확인된 이점이 없어 사용하지 않는다. 첫 epoch는
`3e-5`에서 `3e-4`까지 warm-up하고 이후 cosine으로 감소한다. 전체 학습은 약 `1.8--2.2시간`을
예상하며 첫 epoch ETA로 다시 보정한다.

## 채택 gate.

- Offline: 모든 foreground IoU를 보고 mIoU 하나로 결정하지 않는다.
- Online: 로터리, 고주로, Pedestrian, Obstacle rosbag에서 semantic point/pixel recall을
  기존 production LRASPP와 비교한다.
- 새 LRASPP가 offline만 좋아지고 online Vehicle/Pedestrian/Obstacle이 개선되지 않으면
  PointPainting 재학습에 사용하지 않는다.
- gate 통과 시 동일 checkpoint로 painted Base3 single-frame dataset을 만들고 PointPainting
  S1 계열과 같은 rosbag leaderboard 기준으로 비교한다.

## 다음 segmentation 개선 실험.

현재 Base3 LRASPP natural/ENet/short-side 384 학습은 controlled baseline이므로 중간에
설정을 바꾸지 않는다. 학습 종료 후 다음 순서로 개선한다.

1. Metric gate.
   - 일반 pixel metric은 foreground mIoU와 class별 IoU/precision/recall을 모두 저장한다.
   - PointPainting 채택의 주 semantic 지표는 LiDAR projected-point 기준
     precision/recall/IoU다.
   - 높은 pixel recall만으로 채택하지 않는다. 객체 밖 배경을 크게 칠하면 해당 위치의
     LiDAR point도 foreground semantic feature를 받아 3D false positive와 box 중심·크기
     오차를 만들 수 있다.
2. Loss A/B.
   - A: 현재 ENet-weighted cross entropy.
   - B: ENet-weighted cross entropy + `0.3 * Dice`.
   - B가 Pedestrian/Obstacle precision과 projected-point IoU를 개선하면서 recall을
     과도하게 떨어뜨리지 않는지 확인한다.
3. Augmentation A/B.
   - full image sample을 주로 유지한다.
   - object-aware crop/scale sample을 약 20--30% 섞고 mild color, blur, JPEG 변형을
     적용한다.
   - 원거리 객체 분포를 훼손하지 않도록 과도한 object crop은 사용하지 않는다.
4. Resolution A/B.
   - short-side `384 / 448 / 512`를 같은 checkpoint selection 기준으로 비교한다.
   - 4-camera resize/normalize, LRASPP inference, 전체 PointPainting compute의
     mean/p50/p95/p99, throughput과 peak VRAM을 함께 측정한다.
   - 기존 production LRASPP 순수 추론 약 9 ms를 latency 기준점으로 사용하되 새 환경에서
     다시 측정한다.
5. Architecture gate.
   - loss, augmentation, resolution 개선 후에도 mission rosbag의 Pedestrian/Obstacle
     projected-point 성능이 부족할 때만 SegFormer-B0 등 경량 대체 모델을 비교한다.
   - 정확도 개선량보다 4-camera online latency 증가가 큰 모델은 채택하지 않는다.
6. End-to-end gate.
   - segmentation checkpoint만 교체한 semantic rosbag audit과, 그 checkpoint로 Base3를
     다시 painting하고 PointPainting을 재학습한 end-to-end OD 평가를 구분한다.
   - 최종 판단은 로터리·고주로·Pedestrian·Obstacle rosbag의 class별 AP, false positive,
     mATE/mASE/mAOE와 전체 latency로 한다.

## Base3 natural/ENet/384 E30 최종 결과.

- Run: `base3-static-lraspp4class-natural-enet-bs12-e30`.
- Primary checkpoint: `best_foreground_miou.pth`, Epoch 30.
- Validation:
  - 4-class mIoU `0.8315`, foreground mIoU `0.7759`.
  - Pedestrian/Vehicle/Obstacle IoU `0.7065 / 0.9008 / 0.7203`.
  - Pedestrian/Vehicle/Obstacle precision `0.7113 / 0.9068 / 0.7578`.
  - Pedestrian/Vehicle/Obstacle recall `0.9905 / 0.9927 / 0.9357`.
- Frozen test split:
  - 4-class mIoU `0.8362`, foreground mIoU `0.7821`.
  - Pedestrian/Vehicle/Obstacle IoU `0.6978 / 0.8999 / 0.7485`.
  - Pedestrian/Vehicle/Obstacle precision `0.7023 / 0.9058 / 0.7619`.
  - Pedestrian/Vehicle/Obstacle recall `0.9908 / 0.9928 / 0.9771`.
- Validation과 test가 유사하므로 split 과적합 징후는 작다. Vehicle은 강하고,
  Pedestrian과 Obstacle은 높은 recall 대비 과대 painting precision이 다음 개선 대상이다.

### 15 Hz 상당 4-camera runtime.

입력은 `sensor_cam_raw_roundabout1.bag`을 `1.73x`로 재생했으며 LiDAR 입력은 약
`15.00 Hz`다. 두 backend 모두 464/464 frame을 발행하고 drop/skip은 0이었다.

| Backend | LRASPP inference p50 / p95 | 전체 compute p50 / p95 / p99 | 입력--발행 p50 / p95 / p99 |
|---|---:|---:|---:|
| torchvision, 4 workers | 10.03 / 11.52 ms | 52.21 / 57.59 / 62.56 ms | 55.42 / 68.17 / 72.15 ms |
| OpenCV, 4 workers | 9.41 / 10.93 ms | 43.46 / 48.31 / 50.96 ms | 44.44 / 50.18 / 54.02 ms |

- 같은 LRASPP-MobileNetV3/384 구조이므로 새 weight로 인한 latency 증가는 없다.
- OpenCV는 resize/tensor/normalize p50이 `3.28 ms`로 torchvision의 `11.02 ms`보다
  빠르다. 다만 최종 production backend는 새 LRASPP로 다시 painting한 PointPainting의
  정확도 A/B까지 확인한 뒤 결정한다.

### 기존 mission rosbag semantic A/B.

동일 rosbag, 동일 GT box와 LiDAR point, 동일 short-side 384에서 기존 production
LRASPP와 비교했다. `GT-point argmax`는 각 GT 3D box 안에서 카메라로 투영 가능한
LiDAR point 중 target class가 argmax인 비율의 객체별 평균이다.

| Scene | Class | Production target probability | Base3 target probability | Production GT-point argmax | Base3 GT-point argmax |
|---|---|---:|---:|---:|---:|
| 로터리 | Vehicle | 0.421 | **0.743** | 41.9% | **75.5%** |
| 고주로 | Vehicle | 0.435 | **0.706** | 43.0% | **70.9%** |
| 횡단 | Pedestrian | 0.539 | **0.886** | 56.4% | **90.3%** |
| 5--18 m 거리 | Pedestrian | 0.366 | **0.830** | 39.5% | **86.3%** |
| Obstacle3 | Obstacle | 0.080 | **0.418** | 7.3% | **41.2%** |
| Ped-negative 장면의 obstacle | Obstacle | 0.006 | **0.988** | 0.0% | **100.0%** |

- Vehicle과 Pedestrian의 기존 online semantic domain gap은 크게 줄었다.
- Obstacle도 크게 개선됐지만 Obstacle3는 16 m 이상에서 GT-point argmax가 평균
  `34.4%`이고, 일부 가늘고 긴 size family는 `0--18%`라 아직 production 약점이다.
- Obstacle3를 Base3 production obstacle allowlist의 실제 BBox size와 일치하는 형상으로
  다시 자르면 다음과 같다.
  - 카메라 투영 가능한 196개 중 105개가 CargoBox, drum/barrel 및 Base3 barrier
    8종의 size와 일치한다. Obstacle3에는 Base3 10종 중 8종이 등장한다.
  - 전체 거리에서 production/Base3 GT-point argmax는 `10.5% / 56.1%`다.
  - Base3 수집 정책의 주 거리인 30 m 이내에서는 Base3 GT-point argmax가 `81.3%`다.
    `0--16 m`는 `79.0%`, `16--30 m`는 `83.7%`다.
  - 30 m 초과 39개는 `13.3%`로 급락해, 얇은 제외 asset뿐 아니라 극원거리까지 기존
    Obstacle3 전체 평균을 낮춘다.
  - 따라서 대회 주 판단은 `Base3 allowlist + 30 m 이내` slice로 하고, Obstacle3
    global은 out-of-domain/long-range stress 지표로 유지한다.
- 이 audit은 positive GT point에 대한 recall 성격의 지표다. rosbag 전체 배경 point의
  정답 semantic label이 없으므로 full precision/IoU를 의미하지 않는다.
- 현재 PointPillars는 구 LRASPP probability로 painting된 데이터로 학습됐다. 따라서 새
  LRASPP checkpoint만 끼운 3D AP를 production 성능으로 해석하지 않고, Base3 재painting과
  PointPainting controlled retraining 후 end-to-end OD를 평가한다.

### Base3 재painting 전 PIL--OpenCV A/B.

동일 Base3 E30 checkpoint, short-side 384, 동일 test/rosbag 표본에서 전처리 backend만
바꿨다. OpenCV는 online node와 같은 `INTER_LINEAR`, RGB uint8 tensor 변환과 ImageNet
normalization을 사용한다.

| Frozen test | torchvision/PIL | OpenCV | OpenCV 변화 |
|---|---:|---:|---:|
| Foreground mIoU | 0.7821 | 0.7735 | -0.0086 |
| Pedestrian IoU | 0.6978 | 0.6864 | -0.0114 |
| Vehicle IoU | 0.8999 | 0.8947 | -0.0052 |
| Obstacle IoU | 0.7485 | 0.7393 | -0.0093 |

| Rosbag GT-point target argmax | torchvision/PIL | OpenCV | OpenCV 변화 |
|---|---:|---:|---:|
| 로터리 Vehicle | 75.5% | 76.5% | +1.0%p |
| 고주로 Vehicle | 70.9% | 74.0% | +3.1%p |
| 횡단 Pedestrian | 90.3% | 86.0% | -4.3%p |
| 5--18 m Pedestrian | 86.3% | 88.1% | +1.8%p |
| Obstacle3 global | 41.2% | 44.8% | +3.7%p |
| Base3 allowlist obstacle, 30 m 이내 | 81.3% | 82.1% | +0.8%p |
| Ped-negative 장면 obstacle | 100.0% | 100.0% | 0.0%p |

- OpenCV의 frozen test foreground mIoU 손실은 `0.86%p`이고 class별 손실도
  `0.52--1.14%p`다. 횡단 Pedestrian slice만 객체평균 `4.3%p` 하락하지만 기존
  production `56.4%`보다 여전히 크게 높다.
- 반면 4-camera preprocess p50은 `11.02 -> 3.28 ms`, 전체 PointPainting compute
  p50은 `52.21 -> 43.46 ms`, 입력--발행 p50은 `55.42 -> 44.44 ms`로 줄었다.
- backend 전체에 일관된 online 정확도 하락은 없고 latency 이득이 크므로 Base3
  재painting과 online production target은 **OpenCV**로 고정한다.
- PointPillars 학습용 painted feature와 online 추론 backend를 반드시 OpenCV로
  동일하게 유지한다. PIL은 정확도 상한과 회귀 진단용 A/B로만 보존한다.
