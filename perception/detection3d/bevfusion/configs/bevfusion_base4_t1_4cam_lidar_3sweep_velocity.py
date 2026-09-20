"""Base4 T1: causal three-sweep Ego-aligned BEVFusion velocity baseline."""

_base_ = ['./bevfusion_base4_v1_4cam_lidar_velocity.py']

load_from = "{{$BEVFUSION_BASE4_T1_INIT_CHECKPOINT:/workspace/AjouNice2026/artifacts/detection3d/bevfusion/base4_init/base3_e25_to_base4_t1.pth}}"

point_cloud_range = [-30.0, -30.0, -3.0, 30.0, 30.0, 2.0]
classes = ('vehicle', 'pedestrian', 'obstacle')

model = dict(
    pts_voxel_encoder=dict(type='HardSimpleVFE', num_features=5),
    pts_middle_encoder=dict(in_channels=5, sparse_shape=[800, 800, 41]))

train_pipeline = [
    dict(
        type='BEVLoadMultiViewImageFromFiles', to_float32=True,
        color_type='color', num_views=4),
    dict(
        type='LoadPointsFromFile', coord_type='LIDAR', load_dim=4,
        use_dim=4),
    dict(type='MoraiLoadPointsFromCausalSweeps', sweeps_num=2),
    dict(
        type='LoadAnnotations3D', with_bbox_3d=True, with_label_3d=True,
        with_attr_label=False),
    dict(
        type='MoraiImageAug3D', final_dim=[256, 704],
        resize_lim=[0.55, 0.55], bot_pct_lim=[0.0, 0.0],
        rot_lim=[0.0, 0.0], rand_flip=False, is_train=True),
    dict(
        type='MoraiOnlineImageDomainAug3D', jpeg_prob=0.0, blur_prob=0.0,
        exposure_prob=0.0, noise_prob=0.0),
    dict(
        type='BEVFusionGlobalRotScaleTrans', scale_ratio_range=[0.95, 1.05],
        rot_range=[-0.10, 0.10], translation_std=0.2),
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
            'pcd_trans'])]

test_pipeline = [
    dict(
        type='BEVLoadMultiViewImageFromFiles', to_float32=True,
        color_type='color', num_views=4),
    dict(
        type='LoadPointsFromFile', coord_type='LIDAR', load_dim=4,
        use_dim=4),
    dict(type='MoraiLoadPointsFromCausalSweeps', sweeps_num=2),
    dict(
        type='MoraiImageAug3D', final_dim=[256, 704],
        resize_lim=[0.55, 0.55], bot_pct_lim=[0.0, 0.0],
        rot_lim=[0.0, 0.0], rand_flip=False, is_train=False),
    dict(type='PointsRangeFilter', point_cloud_range=point_cloud_range),
    dict(
        type='Pack3DDetInputs', keys=['points', 'img'],
        meta_keys=[
            'cam2img', 'ori_cam2img', 'lidar2cam', 'lidar2img',
            'cam2lidar', 'img_aug_matrix', 'box_type_3d', 'sample_idx',
            'lidar_path', 'img_path', 'num_pts_feats', 'num_views'])]

train_dataloader = dict(dataset=dict(pipeline=train_pipeline))
val_dataloader = dict(dataset=dict(pipeline=test_pipeline))
test_dataloader = dict(dataset=dict(pipeline=test_pipeline))
