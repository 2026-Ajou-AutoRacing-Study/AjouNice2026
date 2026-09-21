"""Base4-Temporal-3S: four current cameras + three ego-aligned LiDAR sweeps."""

_base_ = [
    '../../../../third_party/mmdetection3d/projects/BEVFusion/configs/'
    'bevfusion_lidar-cam_voxel0075_second_secfpn_8xb4-cyclic-20e_nus-3d.py'
]

custom_imports = dict(
    imports=[
        'projects.BEVFusion.bevfusion',
        'perception.detection3d.bevfusion.plugin',
    ],
    allow_failed_imports=False,
)

classes = ('vehicle', 'pedestrian', 'obstacle')
class_names = classes
metainfo = dict(classes=classes)
dataset_type = 'MoraiBase4Temporal3SweepDataset'
point_cloud_range = [-30.0, -30.0, -3.0, 30.0, 30.0, 2.0]
# Keep the upstream 40-bin vertical sparse geometry so its 256-channel LiDAR
# neck remains unchanged; the pipeline still filters actual points to Base3's
# tighter Z range above.
voxelization_range = [-30.0, -30.0, -5.0, 30.0, 30.0, 3.0]
repo_root = '/workspace/AjouNice2026'
info_root = repo_root + '/artifacts/datasets/morai_base4_temporal_3sweep_v1'
pretrained = (
    repo_root + '/artifacts/detection3d/bevfusion/pretrained/'
    'base4_temporal_3sweep_epoch24_init.pth')

input_modality = dict(use_lidar=True, use_camera=True)
data_prefix = dict(pts='', img='')

model = dict(
    data_preprocessor=dict(
        voxelize_cfg=dict(point_cloud_range=voxelization_range)),
    pts_voxel_encoder=dict(type='HardSimpleVFE', num_features=5),
    pts_middle_encoder=dict(in_channels=5, sparse_shape=[800, 800, 41]),
    img_backbone=dict(
        convert_weights=False,
        init_cfg=None),
    view_transform=dict(
        xbound=[-30.0, 30.0, 0.3],
        ybound=[-30.0, 30.0, 0.3]),
    bbox_head=dict(
        type='MoraiTransFusionHead',
        num_classes=3,
        common_heads=dict(
            _delete_=True,
            center=[2, 2], height=[1, 2], dim=[3, 2], rot=[2, 2], vel=[2, 2]),
        train_cfg=dict(
            assigner=dict(type='HungarianAssigner3D'),
            grid_size=[800, 800, 41],
            point_cloud_range=point_cloud_range,
            code_weights=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.2, 0.2]),
        # The upstream head only defines class-safe local-NMS groups for its
        # 3-class Waymo branch. MORAI also has exactly three ordered classes,
        # so reuse that branch (names are not used by the computation).
        test_cfg=dict(
            dataset='Waymo',
            grid_size=[800, 800, 41],
            pc_range=[-30.0, -30.0],
            voxel_size=[0.075, 0.075],
            out_size_factor=8),
        bbox_coder=dict(
            code_size=10,
            pc_range=[-30.0, -30.0],
            post_center_range=[-30.0, -30.0, -3.0, 30.0, 30.0, 2.0]),
    ),
)

# Base4-Temporal-3S consumes the current camera set and an ego-motion-
# compensated three-sweep LiDAR cloud.  Point channel 4 stores the signed
# time lag in seconds: [x, y, z, intensity, dt].
train_pipeline = [
    dict(
        type='BEVLoadMultiViewImageFromFiles',
        to_float32=True,
        color_type='color',
        num_views=4),
    dict(
        type='LoadPointsFromFile',
        coord_type='LIDAR',
        load_dim=5,
        use_dim=5),
    dict(
        type='LoadAnnotations3D',
        with_bbox_3d=True,
        with_label_3d=True,
        with_attr_label=False),
    dict(
        type='MoraiImageAug3D',
        final_dim=[256, 704],
        resize_lim=[0.55, 0.55],
        bot_pct_lim=[0.0, 0.0],
        rot_lim=[0.0, 0.0],
        rand_flip=False,
        is_train=True),
    # Kept disabled in B0. Online-domain experiments enable it through their
    # runner without changing the reproduced baseline contract.
    dict(type='MoraiOnlineImageDomainAug3D', jpeg_prob=0.0,
         blur_prob=0.0, exposure_prob=0.0, noise_prob=0.0),
    dict(
        type='BEVFusionGlobalRotScaleTrans',
        scale_ratio_range=[0.95, 1.05],
        rot_range=[-0.10, 0.10],
        translation_std=0.2),
    dict(type='PointsRangeFilter', point_cloud_range=point_cloud_range),
    dict(type='ObjectRangeFilter', point_cloud_range=point_cloud_range),
    dict(type='ObjectNameFilter', classes=classes),
    dict(type='PointShuffle'),
    dict(
        type='Pack3DDetInputs',
        keys=['points', 'img', 'gt_bboxes_3d', 'gt_labels_3d'],
        meta_keys=[
            'cam2img', 'ori_cam2img', 'lidar2cam', 'lidar2img',
            'cam2lidar', 'ori_lidar2img', 'img_aug_matrix',
            'lidar_aug_matrix', 'box_type_3d', 'sample_idx', 'lidar_path',
            'img_path', 'num_pts_feats', 'num_views',
            'transformation_3d_flow', 'pcd_rotation', 'pcd_scale_factor',
            'pcd_trans',
        ]),
]

test_pipeline = [
    dict(
        type='BEVLoadMultiViewImageFromFiles',
        to_float32=True,
        color_type='color',
        num_views=4),
    dict(
        type='LoadPointsFromFile',
        coord_type='LIDAR',
        load_dim=5,
        use_dim=5),
    dict(
        type='MoraiImageAug3D',
        final_dim=[256, 704],
        resize_lim=[0.55, 0.55],
        bot_pct_lim=[0.0, 0.0],
        rot_lim=[0.0, 0.0],
        rand_flip=False,
        is_train=False),
    dict(type='PointsRangeFilter', point_cloud_range=point_cloud_range),
    dict(
        type='Pack3DDetInputs',
        keys=['points', 'img'],
        meta_keys=[
            'cam2img', 'ori_cam2img', 'lidar2cam', 'lidar2img',
            'cam2lidar', 'img_aug_matrix', 'box_type_3d', 'sample_idx',
            'lidar_path', 'img_path', 'num_pts_feats', 'num_views',
        ]),
]

train_dataloader = dict(
    batch_size=4,
    num_workers=4,
    persistent_workers=True,
    sampler=dict(type='DefaultSampler', shuffle=True),
    dataset=dict(
        _delete_=True,
        type=dataset_type,
        data_root='',
        ann_file=info_root + '/morai_base4_temporal_3sweep_infos_train.pkl',
        data_prefix=data_prefix,
        pipeline=train_pipeline,
        metainfo=metainfo,
        modality=input_modality,
        filter_empty_gt=False,
        test_mode=False,
        box_type_3d='LiDAR'))

val_dataloader = dict(
    batch_size=1,
    num_workers=2,
    persistent_workers=True,
    drop_last=False,
    sampler=dict(type='DefaultSampler', shuffle=False),
    dataset=dict(
        _delete_=True,
        type=dataset_type,
        data_root='',
        ann_file=info_root + '/morai_base4_temporal_3sweep_infos_val.pkl',
        data_prefix=data_prefix,
        pipeline=test_pipeline,
        metainfo=metainfo,
        modality=input_modality,
        filter_empty_gt=False,
        test_mode=True,
        load_eval_anns=True,
        box_type_3d='LiDAR'))
test_dataloader = dict(
    batch_size=1,
    num_workers=2,
    persistent_workers=True,
    drop_last=False,
    sampler=dict(type='DefaultSampler', shuffle=False),
    dataset=dict(
        _delete_=True,
        type=dataset_type,
        data_root='',
        ann_file=info_root + '/morai_base4_temporal_3sweep_infos_test.pkl',
        data_prefix=data_prefix,
        pipeline=test_pipeline,
        metainfo=metainfo,
        modality=input_modality,
        filter_empty_gt=False,
        test_mode=True,
        load_eval_anns=True,
        box_type_3d='LiDAR'))

val_evaluator = dict(
    _delete_=True,
    type='MoraiCenterMetric',
    prefix='val',
    point_cloud_range=point_cloud_range)
test_evaluator = dict(
    _delete_=True,
    type='MoraiCenterMetric',
    prefix='test',
    point_cloud_range=point_cloud_range)

train_cfg = dict(by_epoch=True, max_epochs=12, val_interval=1)
val_cfg = dict()
test_cfg = dict()
load_from = pretrained

optim_wrapper = dict(
    type='AmpOptimWrapper',
    # RTX 50-series smoke tests showed that PyTorch's 65536 default skips the
    # first eight fresh-head updates. Start at the empirically stable scale;
    # dynamic growth/backoff remains enabled.
    loss_scale=dict(init_scale=32.0, growth_interval=2000),
    optimizer=dict(type='AdamW', lr=1.0e-4, weight_decay=0.01),
    clip_grad=dict(max_norm=35, norm_type=2))

param_scheduler = [
    dict(
        type='LinearLR', start_factor=0.1, by_epoch=False, begin=0, end=500),
    dict(
        type='CosineAnnealingLR',
        begin=0,
        end=12,
        T_max=12,
        by_epoch=True,
        eta_min=1.0e-6,
        convert_to_iter_based=True),
]

default_hooks = dict(
    logger=dict(type='LoggerHook', interval=20),
    checkpoint=dict(
        type='CheckpointHook',
        interval=1,
        max_keep_ckpts=3,
        save_best='val/center/mAP',
        rule='greater'))
custom_hooks = [dict(type='MoraiEpochLrHook')]
log_processor = dict(window_size=20, by_epoch=True)

vis_backends = [
    dict(type='LocalVisBackend'),
    dict(
        type='MoraiWandbVisBackend',
        init_kwargs=dict(
            project='morai-3d-object-detection',
            entity='AjouNice2026',
            name='bevfusion-base3-b0',
            group='bevfusion-base3',
            job_type='train')),
]
visualizer = dict(
    type='Det3DLocalVisualizer', vis_backends=vis_backends, name='visualizer')

randomness = dict(seed=260913, deterministic=False)
auto_scale_lr = dict(enable=False, base_batch_size=1)
