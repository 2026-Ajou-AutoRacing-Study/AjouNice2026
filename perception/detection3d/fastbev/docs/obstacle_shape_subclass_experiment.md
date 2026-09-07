# Fast-BEV obstacle shape subclass 실험

## 목적과 결론

MORAI의 `obstacle`은 작은 정육면체형 물체부터 가늘고 높은 표지물, 길쭉한
물체까지 크기 분포가 넓다. 하나의 class와 하나의 대표 anchor만 사용하는 것이
학습을 방해하는지 확인하기 위해 내부 학습 class를 3개에서 5개로 확장했다.

- 내부 학습: `vehicle`, `pedestrian`, `compact_obstacle`, `tall_obstacle`,
  `long_obstacle`
- 평가·배포 경계: 세 obstacle subclass를 다시 `obstacle`로 병합
- 최종 판단: 구현과 학습은 정상 동작했지만 기존 3-class baseline보다 성능과
  속도가 낮아 현재 production 및 ROS 기본 모델에는 적용하지 않는다.

## 파생 라벨 규칙

기존 MORAI annotation을 다시 수작업하지 않는다. 원본 3D box의
`(length, width, height)`만 사용하여 상호 배타적으로 파생한다. 비율 기준은
`2.0`이다.

1. `height > 2 * max(length, width)`이면 `tall_obstacle`
2. tall이 아니면서 `max(length, width) > 2 * min(length, width)` 또는
   `max(length, width) > 2 * height`이면 `long_obstacle`
3. 나머지는 `compact_obstacle`

원본 이미지와 box는 변경하거나 복사하지 않고, Git 밖 원본 데이터를 참조하는
작은 info pickle만 `artifacts/` 아래에 생성한다.

| Split | Compact | Tall | Long |
|---|---:|---:|---:|
| Train | 351 | 662 | 508 |
| Validation | 107 | 206 | 114 |
| Test | 90 | 136 | 149 |
| Total | 548 | 1,004 | 771 |

전체 중앙값 `(length, width, height)`은 compact `(0.749, 0.749, 0.919)`,
tall `(0.316, 0.600, 3.011)`, long `(0.617, 1.358, 0.900)`이다.

## 구현

- `tools/build_obstacle_shape_infos.py`: 5-class metadata 파생 및 정합성 검사
- `configs/fastbev_m0_morai_4cam_s384x704_shape5.py`: 5-class head와 shape별
  anchor, OpenCV/flip 0.5 조건
- `common/evaluation/obstacle_shape_merge.py`: 세 shape prediction을 obstacle로
  병합하고 class-agnostic rotated NMS 수행
- `plugin/morai_dataset.py`: raw 5-class와 merged 3-class 평가를 동시에 출력
- best checkpoint 기준: validation `merged_center_mAP`

Subclass별 NMS 뒤 obstacle로 병합하면 서로 다른 subclass의 중복 box가 남을 수
있다. 따라서 병합 후 rotated BEV NMS를 한 번 더 적용한다. Validation-only sweep
결과 IoU threshold `0.1`을 선택했다.

| Merge NMS IoU | Validation merged center mAP |
|---:|---:|
| 0.1 | 0.5726 |
| 0.2 | 0.5569 |
| 0.3 | 0.5568 |
| 0.5 | 0.5568 |
| 0.8 | 0.5562 |

## 재현 명령

```bash
cd ~/AjouNice2026

python3 perception/detection3d/fastbev/tools/build_obstacle_shape_infos.py

FASTBEV_CONFIG="$PWD/perception/detection3d/fastbev/configs/fastbev_m0_morai_4cam_s384x704_shape5.py" \
FASTBEV_IMAGE_BACKEND=opencv \
FASTBEV_BEV_FLIP_PROB=0.5 \
FASTBEV_IMAGE_FLIP_PROB=0.5 \
WANDB_RUN_GROUP=sunny13-1500-obstacle-shape \
perception/detection3d/fastbev/scripts/train.sh \
  full1500_stride1_fp16_opencv_flip50_50_shape5_v1
```

실험 산출물은 다음 경로에 있다.

- Run: `artifacts/detection3d/fastbev/runs/full1500_stride1_fp16_opencv_flip50_50_shape5_v1/`
- Best: `best_merged_center_mAP_epoch_12.pth`
- Test: `test_metrics.json`
- 비교표: `comparison.md`
- W&B: `AjouNice2026/morai-fastbev/full1500_stride1_fp16_opencv_flip50_50_shape5_v1`

## Held-out test 결과

동일한 222-frame test split에서 기존 OpenCV/flip 0.5 3-class baseline과
비교했다. NMS threshold 선택은 validation에서 끝낸 뒤 test를 평가했다.

| Metric | 3-class | Shape5 후 3-class 병합 | Delta |
|---|---:|---:|---:|
| Center mAP | 0.5919 | 0.5856 | -0.0063 |
| Vehicle mean AP | 0.6938 | 0.6867 | -0.0072 |
| Pedestrian mean AP | 0.6625 | 0.6667 | +0.0042 |
| Obstacle mean AP | 0.4194 | 0.4034 | -0.0160 |
| IoU mAP@0.25 | 0.5744 | 0.5368 | -0.0376 |
| IoU mAP@0.50 | 0.2751 | 0.2634 | -0.0118 |
| Inference FPS | 41.48 | 36.77 | -4.71 |

Raw shape mean AP는 compact `0.2901`, tall `0.3976`, long `0.2680`이다.
특히 compact와 long은 표본 수와 외형 내부 분산이 아직 크다. 5-class head는
anchor 수를 위치당 6개에서 10개로 늘려 추론도 약 11.4% 느려졌다. 병합
obstacle mAOE가 크게 나빠진 것은 서로 다른 shape hypothesis의 yaw 의미가
병합 과정에서 일관되지 않은 영향도 있으므로 production 채택 근거로 쓰지 않는다.

## W&B 패널

Shape 진단용 그룹 `VAL — Shape5 / Merged`를 만들고 X축은 `epoch`로 둔다.

- `Merged center mAP`: `val/merged_center_mAP`
- `Merged class mean AP`: `val/merged/center/vehicle_mean_AP`,
  `val/merged/center/pedestrian_mean_AP`,
  `val/merged/center/obstacle_mean_AP`
- `Raw obstacle shape mean AP`: `val/center/compact_obstacle_mean_AP`,
  `val/center/tall_obstacle_mean_AP`, `val/center/long_obstacle_mean_AP`
- `Merged obstacle distance AP`: `val/merged/center/obstacle_AP_0.5m`,
  `val/merged/center/obstacle_AP_1m`, `val/merged/center/obstacle_AP_2m`,
  `val/merged/center/obstacle_AP_4m`
- `Merged obstacle translation / scale`: `val/merged/tp/obstacle_mATE`,
  `val/merged/tp/obstacle_mASE`
- `Merged obstacle yaw`: `val/merged/tp/obstacle_mAOE`

기존 baseline에는 `merged/*` key가 없다. 같은 W&B panel에서 직접 겹쳐 볼 때는
baseline의 `val/center_mAP`와 후보의 `val/merged_center_mAP`를 함께 Y축에 넣고,
class AP도 같은 방식으로 원래 key와 `merged/` key를 함께 선택한다. Raw shape
패널은 shape5 run에만 값이 나타나는 것이 정상이다.

## ROS 적용 설계와 보류 이유

채택한다면 ROS node 내부에서 5-class config/checkpoint로 추론한 뒤 shape label
2/3/4를 `obstacle`로 remap하고, IoU `0.1` class-agnostic rotated NMS를 적용한
3-class `DetectedObjects`만 외부에 발행한다. 디버그 marker에는 원래 shape를
남길 수 있다. Tracking·Planning은 내부 subclass를 알 필요가 없다.

그러나 held-out test에서 기존 3-class보다 obstacle AP와 IoU가 낮고 추론도
느렸으므로 이 ROS 변환은 현재 구현하지 않는다. 다음 재시도는 단순 class 분할보다
shape-aware regression loss, anchor-only ablation, 더 많은 obstacle 표본을 먼저
검토한다.

## 결과 해석: Shape5가 회귀 분포를 분리했는가

이번 실험은 obstacle label을 세 subclass로 나눴지만 bbox regression까지
class별로 나눈 실험은 아니다. 현재 `FreeAnchor3DHead`의 출력은 다음과 같다.

```text
classification = num_anchors * num_classes
bbox regression = num_anchors * box_code_size
direction       = num_anchors * 2
```

따라서 regression 출력에는 class 차원이 없다. Shape5에서도 모든 class가 모든
anchor의 regression 결과를 공유한다. 결과적으로 표본과 classification 문제는
세분화했지만, 애초 가설이었던 compact/tall/long별 회귀 전문화는 강제하지 않았다.
이는 현재 방식이 다음 두 비용을 부담하면서 회귀 전문화 이득은 충분히 얻지 못한
이유다.

- train obstacle 표본이 compact `351`, tall `662`, long `508`로 분산됨
- BEV 위치당 classification 출력이 기존 `6 anchors * 3 classes = 18`에서
  `10 anchors * 5 classes = 50`으로 증가함

기존 3-class 대비 obstacle mATE가 `0.3704 -> 0.3944`, mASE가
`0.2379 -> 0.2410`으로 모두 나빠졌다. 이는 하나의 obstacle class가 반드시
단일 평균 크기로만 회귀한다는 뜻은 아니다. 회귀는 입력 feature와 anchor별로
조건화되며 연속적인 box residual을 예측한다. 다만 넓은 형상 분포가 병목일
가능성은 여전히 있으므로, 아래 두 통제 실험으로 다시 검증한다.
