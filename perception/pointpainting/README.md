# MORAI PointPainting 3-Class

Competition PointPainting pipeline for:

- Pedestrian
- Vehicle
- Obstacle

## Pipeline

4-camera RGB
-> LRASPP 4-class semantic segmentation
-> LiDAR projection / multi-camera fusion
-> 9-feature PointPainting
-> PointPillars 3D detection

Point feature order:

[x, y, z, intensity, background, pedestrian, vehicle, obstacle, unknown]

`unknown` indicates a LiDAR point that is not visible from any camera.

## Main files

- `tools/build_sunny13_1500_pointpainting_9feat.py`
  - Generates the 9-feature PointPainting dataset from 4-camera RGB and LiDAR.

- `openpcdet_patch/tools/cfgs/custom_models/pointpillar_morai_pointpainting_sunny13_1500_9feat.yaml`
  - PointPillars 3-class model configuration.

- `openpcdet_patch/tools/cfgs/dataset_configs/morai_pointpainting_sunny13_1500_9feat.yaml`
  - MORAI 9-feature dataset configuration.

- `openpcdet_patch/pcdet/datasets/custom/custom_dataset.py`
- `openpcdet_patch/pcdet/datasets/custom/morai_eval.py`
  - MORAI CustomDataset/evaluation support for OpenPCDet.

- `openpcdet_patch/tools/eval_morai_competition_metrics.py`
  - Center-distance AP evaluation.

## Weights

Weights are distributed separately and are not committed to Git.

- `lraspp_4class_best_miou.pth`
  - LRASPP semantic segmentation checkpoint.

- `pointpainting_B0_epoch11.pth`
  - PointPillars detector checkpoint.
  - 12-epoch comparison best checkpoint: epoch 11.

## OpenPCDet patch

Files under `openpcdet_patch/` should be copied to the corresponding paths
inside the OpenPCDet repository.

The PointPainting dataset builder requires the input dataset path, output path,
and LRASPP checkpoint path to be supplied as command-line arguments.
