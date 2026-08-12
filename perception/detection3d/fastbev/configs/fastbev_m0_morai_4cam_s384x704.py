_base_ = [
    '../../../../third_party/fastbev/configs/fastbev/exp/paper/fastbev_m0_r18_s256x704_v200x200x4_c192_d2_f4.py'
]

custom_imports = dict(
    imports=['perception.detection3d.fastbev.plugin'],
    allow_failed_imports=False)

class_names = ['vehicle', 'pedestrian', 'obstacle']
dataset_type = 'MoraiMultiViewDataset'
data_root = '/home/luv/ajounice_data/morai_od/processed/fastbev_sunny13_1500_v1/'
point_cloud_range = [-50, -50, -5, 50, 50, 3]

model = dict(
    # The upstream M0 config assumes 6 cameras over 4 timesteps.  MORAI uses
    # one timestep from a fixed four-camera rig, so make both dimensions
    # explicit instead of relying on Fast-BEV's historical 6-camera default.
    num_cameras=4,
    backbone=dict(
        init_cfg=dict(
            type='Pretrained',
            checkpoint=(
                'https://download.pytorch.org/models/'
                'resnet18-f37072fd.pth'))),
    neck_3d=dict(
        in_channels=64 * 4,
        # Preserve the native 0.5 m BEV grid for pedestrians and obstacles.
        # The upstream stride-2 head produces a 1.0 m output grid, which was
        # shown by the 21-frame ablation to be too coarse for MORAI small boxes.
        stride=1,
        fuse=dict(in_channels=64 * 4, out_channels=64 * 4)),
    bbox_head=dict(
        num_classes=3,
        anchor_generator=dict(
            ranges=[[-50, -50, -1.8, 50, 50, -1.8]],
            sizes=[[4.5, 1.9, 1.7], [0.6, 0.6, 1.75], [1.0, 1.0, 1.0]],
            custom_values=[0, 0],
            rotations=[0, 1.57],
            reshape_out=True)),
    test_cfg=dict(
        nms_type_list=['rotate', 'rotate', 'rotate'],
        nms_thr_list=[0.2, 0.2, 0.2],
        nms_radius_thr_list=[4.0, 0.5, 1.0],
        nms_rescale_factor=[1.0, 1.0, 1.0]))

input_modality = dict(
    use_lidar=False,
    use_camera=True,
    use_radar=False,
    use_map=False,
    use_external=False)

# Width-based resize followed by centre crop preserves each camera's aspect
# ratio.  Front/back and side cameras therefore reach the same 384x704 canvas
# without stretching their different native resolutions.
data_config = dict(
    input_size=(384, 704),
    resize=(-0.02, 0.02),
    crop=(-0.02, 0.02),
    rot=(-2.0, 2.0),
    flip=True,
    test_input_size=(384, 704),
    test_resize=0.0,
    test_rotate=0.0,
    test_flip=False,
    pad=(0, 0, 0, 0),
    pad_divisor=32,
    pad_color=(0, 0, 0))

file_client_args = dict(backend='disk')
img_norm_cfg = dict(
    mean=[123.675, 116.28, 103.53],
    std=[58.395, 57.12, 57.375],
    to_rgb=True)

train_pipeline = [
    dict(type='MoraiMultiViewPipeline', n_images=4, transforms=[
        dict(type='LoadImageFromFile', file_client_args=file_client_args)]),
    dict(type='LoadAnnotations3D', with_bbox_3d=True, with_label_3d=True,
         with_bbox=False, with_label=False),
    dict(type='LoadPointsFromFile', dummy=True, coord_type='LIDAR', load_dim=5, use_dim=5),
    dict(type='RandomFlip3D', flip_2d=False, sync_2d=False,
         flip_ratio_bev_horizontal=0.5, flip_ratio_bev_vertical=0.0,
         update_img2lidar=True),
    dict(type='GlobalRotScaleTrans', rot_range=[-0.20, 0.20],
         scale_ratio_range=[0.95, 1.05], translation_std=[0.05, 0.05, 0.05],
         update_img2lidar=True),
    dict(type='RandomAugImageMultiViewImage', data_config=data_config),
    dict(type='ObjectRangeFilter', point_cloud_range=point_cloud_range),
    dict(type='KittiSetOrigin', point_cloud_range=point_cloud_range),
    dict(type='NormalizeMultiviewImage', **img_norm_cfg),
    dict(type='DefaultFormatBundle3D', class_names=class_names),
    dict(type='Collect3D', keys=['img', 'gt_bboxes_3d', 'gt_labels_3d'])]

test_pipeline = [
    dict(type='MoraiMultiViewPipeline', n_images=4, transforms=[
        dict(type='LoadImageFromFile', file_client_args=file_client_args)]),
    dict(type='LoadPointsFromFile', dummy=True, coord_type='LIDAR', load_dim=5, use_dim=5),
    dict(type='RandomAugImageMultiViewImage', data_config=data_config, is_train=False),
    dict(type='KittiSetOrigin', point_cloud_range=point_cloud_range),
    dict(type='NormalizeMultiviewImage', **img_norm_cfg),
    dict(type='DefaultFormatBundle3D', class_names=class_names, with_label=False),
    dict(type='Collect3D', keys=['img'])]

common_dataset = dict(
    type=dataset_type,
    data_root=data_root,
    classes=class_names,
    modality=input_modality,
    box_type_3d='LiDAR',
    load_interval=1,
    sequential=False,
    n_times=1,
    with_velocity=True,
    use_valid_flag=True)

data = dict(
    samples_per_gpu=1,
    workers_per_gpu=2,
    train=dict(_delete_=True, **common_dataset,
               pipeline=train_pipeline, test_mode=False,
               ann_file=data_root + 'fastbev_infos/morai_infos_train.pkl'),
    val=dict(_delete_=True, **common_dataset,
             pipeline=test_pipeline, test_mode=True,
             ann_file=data_root + 'fastbev_infos/morai_infos_val.pkl'),
    test=dict(_delete_=True, **common_dataset,
              pipeline=test_pipeline, test_mode=True,
              ann_file=data_root + 'fastbev_infos/morai_infos_test.pkl'))

# Upstream AdamW2 calls torch.optim._functional.adamw with the pre-PyTorch-2
# positional API.  Standard AdamW has the same intended optimizer semantics
# and is maintained by PyTorch/MMCV for the installed runtime.
optimizer = dict(
    _delete_=True,
    type='AdamW',
    lr=0.0004,
    weight_decay=0.01,
    paramwise_cfg=dict(
        custom_keys={'backbone': dict(lr_mult=0.1, decay_mult=1.0)}))

# Keep per-epoch validation diagnostic but dashboard-sized: center AP, TP box
# errors, a 1 m operating point, IoU AP/AR, and range summaries through 30 m.
# Runtime and the verbose per-threshold range breakdown remain test-only.
evaluation = dict(
    interval=1,
    metric=('center', 'tp', 'iou'),
    center_thresholds=(0.5, 1.0, 2.0, 4.0),
    distance_bins=((0.0, 10.0), (10.0, 20.0), (20.0, 30.0)),
    compact=True,
    save_best='center_mAP',
    rule='greater')
total_epochs = 12
runner = dict(type='EpochBasedRunner', max_epochs=total_epochs)
checkpoint_config = dict(
    interval=1,
    max_keep_ckpts=3,
    save_last=True)

# MMCV's logger hook sends the same train/validation scalars that appear in
# the text log to W&B.  Runtime identity comes from environment variables so
# teammates can reuse this config without committing credentials or personal
# settings.  Checkpoints remain local by default; they are large and the best
# model is already selected by CheckpointHook above.
wandb_tags = [
    tag.strip() for tag in __import__('os').getenv(
        'WANDB_TAGS',
        'fastbev,morai,4cam,single-frame,s384x704,fp16').split(',')
    if tag.strip()
]
wandb_config = dict(
    architecture='FastBEV-M0-ResNet18',
    dataset='MORAI-fastbev-sunny13-1500-v1',
    train_frames=1050,
    val_frames=228,
    test_frames=222,
    cameras=4,
    input_height=384,
    input_width=704,
    bev_range_m='[-50,50]x[-50,50]',
    bev_grid_m=0.5,
    batch_size_per_gpu=1,
    optimizer='AdamW',
    learning_rate=0.0004,
    weight_decay=0.01,
    max_epochs=total_epochs,
    fp16=True,
    selection_metric='center_mAP')
log_config = dict(
    interval=10,
    hooks=[
        dict(type='TextLoggerHook'),
        dict(type='TensorboardLoggerHook'),
        dict(
            type='MoraiWandbLoggerHook',
            # MMCV's explicit W&B ``step=runner.iter`` can move backwards
            # across epoch-end validation.  Let W&B keep its own monotonic
            # _step and use our explicit global_step/epoch axes instead.
            with_step=False,
            init_kwargs=dict(
                entity=__import__('os').getenv(
                    'WANDB_ENTITY', 'AjouNice2026'),
                project=__import__('os').getenv(
                    'WANDB_PROJECT', 'morai-fastbev'),
                name=__import__('os').getenv(
                    'WANDB_NAME', 'fastbev_m0_morai_baseline'),
                id=__import__('os').getenv('WANDB_RUN_ID'),
                resume=__import__('os').getenv('WANDB_RESUME', 'allow'),
                group=__import__('os').getenv(
                    'WANDB_RUN_GROUP', 'sunny13-1500-single-frame'),
                job_type=__import__('os').getenv(
                    'WANDB_JOB_TYPE', 'train'),
                tags=wandb_tags,
                notes=__import__('os').getenv(
                    'WANDB_NOTES',
                    'MORAI 4-camera Fast-BEV single-frame baseline'),
                config=wandb_config),
            interval=10)
    ])

# The inherited config points to an upstream checkpoint path that is not part
# of this repository.  Keep a clean from-scratch default; an explicit MORAI or
# Fast-BEV checkpoint can still be supplied with --load-from/--resume-from.
load_from = None
resume_from = None
