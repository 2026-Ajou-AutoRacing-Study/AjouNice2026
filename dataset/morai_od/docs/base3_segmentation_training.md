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
