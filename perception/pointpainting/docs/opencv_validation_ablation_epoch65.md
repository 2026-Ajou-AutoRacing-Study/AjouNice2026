# PointPainting Epoch-65 OpenCV validation ablation

## Contract

- checkpoint: `pointpainting_B0_best_epoch65.pth` from the 80-epoch run.
- split: the same 228-frame validation split used by the model owner.
- GT counts: Pedestrian 851, Vehicle 444, Obstacle 427.
- LRASPP checkpoint and PointPillars weights are unchanged.
- short-side resize: 384.
- the only intended change is camera preprocessing from the training-compatible
  torchvision/PIL path to RGB OpenCV `INTER_LINEAR` resize, tensor conversion
  and ImageNet normalization.
- all 9-feature painted point clouds were regenerated before evaluation.
- this is an inference compatibility ablation, not an OpenCV retraining run.

## Center AP result

| Metric | PIL baseline | OpenCV inference | Delta |
| --- | ---: | ---: | ---: |
| Overall center mAP | 0.9358 | **0.9364** | +0.0006 |
| Pedestrian mean AP | 0.9420 | **0.9436** | +0.0016 |
| Vehicle mean AP | **0.9610** | 0.9608 | -0.0002 |
| Obstacle mean AP | 0.9040 | **0.9049** | +0.0009 |
| Pedestrian AP@0.5m | 0.9410 | **0.9414** | +0.0004 |
| Vehicle AP@0.5m | **0.9290** | 0.9170 | -0.0120 |
| Obstacle AP@0.5m | **0.8710** | 0.8675 | -0.0035 |

## Distance summary

| Range | PIL overall mAP | OpenCV overall mAP | Delta |
| --- | ---: | ---: | ---: |
| 0--10 m | **0.9500** | 0.9406 | -0.0094 |
| 10--20 m | **0.9323** | 0.9264 | -0.0059 |
| 20--30 m | **0.8163** | 0.8109 | -0.0054 |

## Box quality at 2 m

| Class | PIL mATE | OpenCV mATE | PIL mASE | OpenCV mASE |
| --- | ---: | ---: | ---: | ---: |
| Pedestrian | **0.0400 m** | 0.0425 m | **0.1310** | 0.1387 |
| Vehicle | **0.0880 m** | 0.0965 m | **0.0730** | 0.0796 |
| Obstacle | **0.0710 m** | 0.0806 m | **0.0640** | 0.0825 |

## Decision

The global center mAP difference is negligible, so OpenCV preprocessing does
not cause a general recognition collapse. However, the distance-binned metrics,
Vehicle AP@0.5m and all three mATE/mASE pairs are consistently worse. Keep the
PIL/torchvision path as the accuracy reference. OpenCV remains a viable runtime
candidate when throughput is required, but it should not become the final
production default solely from global mAP; confirm the trade-off with the
online latency benchmark and, if selected, retrain the LRASPP/painted dataset
with the OpenCV contract.

Generated artifacts:

- `artifacts/detection3d/datasets/morai_pointpainting_sunny13_1500_9feat_opencv_val_v1/`.
- `artifacts/detection3d/pointpainting/releases/260817_sunny13_1500_9feat_80e/opencv_val_metrics.json`.
- `artifacts/detection3d/pointpainting/releases/260817_sunny13_1500_9feat_80e/opencv_val_evaluation.log`.
