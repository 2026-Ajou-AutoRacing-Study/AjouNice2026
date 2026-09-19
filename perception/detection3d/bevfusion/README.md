# Base3 BEVFusion experiment

> **문서 상태: IMPLEMENTATION REFERENCE + HISTORICAL EXPERIMENT LOG.** 아래 초기
> feasibility 서술의 “production replacement가 아님”은 현재 결론이 아니다. 현재
> production/학습 문서는 `docs/bevfusion/README.md`에서 선택한다. Base3-Online은
> `dataset/morai_od/docs/BASE3_ONLINE_CURRENT_STATE.md`가 canonical이다.

The shared multi-GPU training entry point is:

```bash
bash perception/detection3d/bevfusion/scripts/configure_team_machine.sh --operator sw
bash perception/detection3d/bevfusion/scripts/run_team_training.sh fresh-e12 --dry-run
bash perception/detection3d/bevfusion/scripts/run_team_training.sh fresh-e12
```

See `docs/bevfusion/TEAM_DOCKER_TRAINING_HANDOFF.md`. Existing Base3 train,
smoke, rosbag evaluation, and online scripts remain reproducibility tools; the
team runner reuses the same MMEngine config and plugin rather than defining a
second model pipeline.

This experiment asks one narrow question: does native camera–LiDAR feature
fusion retain Base3 offline accuracy better than PointPainting on asynchronous
MORAI rosbag input? It is not a production replacement yet.

## Contract

- Data: `Base3-Static 10K`, 10,902 frames and the existing scene-level split.
- Input: four intensity cameras plus one current LiDAR frame (four features).
- Range: the same `[-30, 30] m` XY and `[-3, 2] m` Z contract as Base3
  PointPainting.
- Labels: Vehicle, Pedestrian, Obstacle; boxes with fewer than five LiDAR
  points are removed from training and evaluation using a 3 cm counting
  tolerance, matching the Base3 PointPainting contract.
- No semantic images, velocity labels, temporal sweeps, or GT sampling.
- Validation runs every epoch and uses the same epoch-axis W&B contract as
  PointPainting: `val/center/*`, `val/tp/*`, `val/operating/*`,
  `val/range/*`, and oriented-BEV `val/iou/*`. Both `val/center_mAP` and
  `val/center/mAP` are retained so existing panels work unchanged. The
  single-frame head has no velocity output, so fake `val/velocity/*` values
  are deliberately not emitted.
- Training scalars, including `train/loss`, component losses, `train/lr`, and
  the epoch-end `train/lr_epoch`, also use `epoch` as their W&B x-axis. The
  PointPainting aliases `epoch/train_loss_mean` and
  `epoch/learning_rate_last` are emitted once per epoch; BEVFusion-specific
  heatmap, classification, and box-loss epoch means remain available beside
  them.
- W&B source snapshots are disabled because generated catkin devel trees can
  contain stale symlinks; the resolved config and all scalar metrics are still
  logged.
- The smoke wrapper adapts the trusted nuScenes BEVFusion checkpoint: its fifth
  LiDAR time-lag channel is removed, and its ten class heads are grouped into
  MORAI Vehicle, Pedestrian, and Obstacle. Thus the compatible camera, LiDAR,
  view-transform, fusion, and detection weights all provide initialization.
- The adapter preserves spconv's per-module version metadata. Without it,
  MMEngine mistakes current-layout sparse kernels for legacy kernels and skips
  the LiDAR sparse encoder after an unnecessary axis conversion.

The modern MMDetection3D BEVFusion project is used. The legacy MIT submodule
requires a PyTorch/CUDA stack unsuitable for the RTX 5060 Ti.

## Preparation and smoke test

Build the CUDA 12.8 image if it is absent:

```bash
cd ~/AjouNice2026
docker build -f docker/bevfusion/Dockerfile -t ajounice/bevfusion:dev-cu128 .
```

The source checkpoint defaults to
`~/bevfusion_artifacts/checkpoints/bevfusion_lidar_cam_nuscenes.pth`. Override
it with `BEVFUSION_NUSCENES_CHECKPOINT=/absolute/path/model.pth` when needed.
The adapted checkpoint is generated under ignored `artifacts/`; it is not a
Git-tracked model binary.

Run the bounded class-stratified 16-frame-per-split, one-epoch smoke test:

```bash
cd ~/AjouNice2026
WANDB_MODE=online \
  bash perception/detection3d/bevfusion/scripts/run_base3_smoke.sh
```

Generate the complete info files only after the smoke test passes:

```bash
bash perception/detection3d/bevfusion/scripts/prepare_base3_full_infos.sh
```

The adapted-pretrain online smoke run is
[`1ef341d5`](https://wandb.ai/AjouNice2026/morai-3d-object-detection/runs/1ef341d5).
It completed train, checkpoint save, and validation with all three classes and
empty frames. A later local contract smoke also verified the final AMP scale,
`val/*` namespace, `train/lr_epoch`, and best-checkpoint selection. The only
checkpoint-load omissions are `view_transform.dx/bx/nx`,
which must be regenerated for MORAI's 60 m BEV rather than loaded from the
nuScenes 108 m grid. After four optimizer steps the loss was 7.87 and the
16-frame smoke mAP was 0.379; these are pipeline checks, not model-quality
estimates. The main run preserves the team's existing `train/*`,
`val/center/*`, `val/tp/*`, and `val/range/*` W&B panel keys. The full info
audit is:

- train/val/test frames: 7,632 / 1,635 / 1,635.
- kept GT: 8,182 Vehicle, 6,600 Pedestrian, 15,183 Obstacle.
- rejected below five points: 98 / 219 / 1,206 respectively.

The raw-LiDAR recount differs from the prior painted-point audit by only seven
borderline Obstacle boxes at the four/five-point boundary. No split, label, or
sensor file is missing.

## Prepared main run (do not start during setup)

The main run is capped at 12 epochs, validates every epoch, and retains the
latest three checkpoints plus the best `val/center/mAP` checkpoint. This is a
full-detector fine-tune from the adapted nuScenes model, so 12 epochs is already
substantially more optimizer work than the official multi-GPU fine-tune; a
30-epoch default would waste several extra days and raise overfitting risk.

```bash
cd ~/AjouNice2026
WANDB_MODE=online \
  bash perception/detection3d/bevfusion/scripts/run_base3_train.sh
```

Do not run that command until the experiment is explicitly approved. Batch 4
uses about 10.6 GB reported model memory on the 16 GB RTX 5060 Ti. After the
runtime optimization, a full-run step is about 0.55 seconds: one 1,908-step
training epoch is roughly 17.5 minutes. A 1,635-frame validation is expected to
take about 11--12 minutes. Validating after every epoch therefore puts the
12-epoch wall-clock estimate near 5 hours 50 minutes, with roughly 20 minutes
of environment/checkpoint variance.
