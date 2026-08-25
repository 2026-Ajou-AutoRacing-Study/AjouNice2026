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
- `openpcdet_patch/pcdet/datasets/custom/morai_center_metrics.py`
  - MORAI CustomDataset/evaluation support for OpenPCDet.
  - Fast-BEV and PointPainting share the same model-neutral W&B validation
    keys for center AP, box quality, operating points, distance bins and IoU.

- `openpcdet_patch/tools/eval_morai_competition_metrics.py`
  - Center-distance AP evaluation.

- `tools/run_rosbag_runtime_benchmark.sh`
  - Measures input/output Hz, end-to-end latency, stage timing, queue delay,
    dropped LiDAR scans, and skipped frames without loading the full log into
    the analysis context.

- `docs/runtime_benchmark_260816.md`
  - RTX 5060 Ti runtime results and the production decision to use four
    camera CPU workers with the training-compatible torchvision backend.

- `docs/opencv_validation_ablation_epoch65.md`
  - Same-checkpoint, same-validation-split PIL versus OpenCV preprocessing
    accuracy comparison for the 80-epoch production release.

- `docs/velocity_regression_9d.md`
  - 3,000-frame 9D velocity GT contract, Epoch65 checkpoint migration,
    training smoke validation, full-training command, metrics, and ROS opt-in.

- `docs/temporal_sweep_alignment_azimuth_deskew_ab.md`
  - Existing dynamic data reuse, 3-sweep Ego alignment, physical simulation
    time recovery, azimuth Ego-deskew A/B, smoke results, and training commands.

- `docs/base2_temporal3_velocity_d4_readiness.md`
  - Base2 10K D4 materialization, 3-sweep/time-lag contract, weighted velocity
    loss, online ROS parity, GPU/W&B smoke results, interleaved epoch train/val,
    and the approved full-run command. Full training is intentionally not
    started by this preparation.

## Weights

Weights are distributed separately and are not committed to Git.

Runtime release location in this workspace:

`artifacts/detection3d/pointpainting/releases/260825_base2_s1_singleframe9_geom_aug_e38/`

- `pointpainting_base2_s1_best_epoch38.pth`
  - Base2 single-sweep 7D detector with geometric augmentation.
  - Current Vehicle production comparator and default ROS checkpoint.
  - Detector velocity publication remains disabled.
- `pointpillar_morai_pointpainting_base2_singleframe9_geom_aug.yaml`
  - Exact S1 training/runtime model configuration.

The shared LRASPP checkpoint remains at:

`artifacts/detection3d/pointpainting/releases/260817_sunny13_1500_9feat_80e/`

- `lraspp_4class_best_miou.pth`
  - LRASPP semantic segmentation checkpoint.

- `pointpainting_B0_best_epoch65.pth`
  - Legacy E65 detector retained as the historical static-data baseline.

## OpenPCDet patch

Training/evaluation patches under `openpcdet_patch/` can be copied to the
corresponding paths inside OpenPCDet. The ROS runtime loads the committed model
and dataset YAML directly, so it does not require an ad-hoc config copy.

The PointPainting dataset builder requires the input dataset path, output path,
and LRASPP checkpoint path to be supplied as command-line arguments.
