"""Base4 Q1-P0: ordered, causal Vehicle query-memory experiment."""

_base_ = ['./bevfusion_base4_v1_4cam_lidar_velocity_w1_head5x.py']

info_root = "{{$MORAI_BASE4_INFO_ROOT:/workspace/AjouNice2026/artifacts/datasets/morai_base4_velocity_hardcase_v2_raw16}}"

model = dict(
    bbox_head=dict(
        type='MoraiQueryMemoryTransFusionHead',
        memory_max_frames=4,
        memory_max_queries_per_frame=64,
        memory_reset_gap_sec=0.30,
        query_memory=dict(
            channels=128,
            num_heads=4,
            vehicle_label=0,
            minimum_memory_score=0.20,
            maximum_memory_age_sec=0.60,
            maximum_association_distance_m=6.0,
            initial_residual_logit=-4.0,
        ),
    ),
)

# A recurrent Python memory must see one contiguous scene in chronological
# order. P0 therefore forbids shuffled and multi-sample batches. It also uses
# identity global augmentation: independent per-frame rotation/translation
# would invalidate cross-frame geometry. Shared clip augmentation is Q1-P1.
train_pipeline = [
    dict(
        type='BEVLoadMultiViewImageFromFiles',
        to_float32=True,
        color_type='color',
        num_views=4),
    dict(
        type='LoadPointsFromFile',
        coord_type='LIDAR',
        load_dim=4,
        use_dim=4),
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
    dict(
        type='MoraiOnlineImageDomainAug3D',
        jpeg_prob=0.0,
        blur_prob=0.0,
        exposure_prob=0.0,
        noise_prob=0.0),
    dict(
        type='BEVFusionGlobalRotScaleTrans',
        scale_ratio_range=[1.0, 1.0],
        rot_range=[0.0, 0.0],
        translation_std=0.0),
    dict(
        type='PointsRangeFilter',
        point_cloud_range=[-30.0, -30.0, -3.0, 30.0, 30.0, 2.0]),
    dict(
        type='ObjectRangeFilter',
        point_cloud_range=[-30.0, -30.0, -3.0, 30.0, 30.0, 2.0]),
    dict(
        type='ObjectNameFilter',
        classes=('vehicle', 'pedestrian', 'obstacle')),
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
            'pcd_trans', 'scene_id', 'timestamp', 'sequence_frame_index',
            'sequence_length', 'ego2global', 'lidar2global',
        ]),
]

train_dataloader = dict(
    batch_size=1,
    num_workers=2,
    persistent_workers=True,
    sampler=dict(type='DefaultSampler', shuffle=False),
    dataset=dict(
        pipeline=train_pipeline,
        ann_file=info_root + '/morai_base3_infos_train.pkl'))
val_dataloader = dict(
    batch_size=1,
    sampler=dict(type='DefaultSampler', shuffle=False),
    dataset=dict(ann_file=info_root + '/morai_base3_infos_val.pkl'))
test_dataloader = dict(
    batch_size=1,
    sampler=dict(type='DefaultSampler', shuffle=False),
    dataset=dict(ann_file=info_root + '/morai_base3_infos_val.pkl'))

# Preserve roughly the V1 batch-4 optimizer-step cadence without pretending
# four unrelated frames are one recurrent batch.
optim_wrapper = dict(
    accumulative_counts=4,
    paramwise_cfg=dict(
        custom_keys={
            'bbox_head.prediction_heads.0.vel': dict(lr_mult=5.0),
            'bbox_head.query_memory': dict(lr_mult=5.0),
        }))

custom_hooks = [
    dict(type='MoraiQueryMemoryStatsHook'),
    dict(type='MoraiEpochLrHook'),
]
