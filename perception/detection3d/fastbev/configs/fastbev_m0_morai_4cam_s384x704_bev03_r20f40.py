"""C0 detector with a 0.3 m BEV grid over rear20/front40/side30 m."""

_base_ = ['./fastbev_m0_morai_4cam_s384x704.py']

point_cloud_range = [-20.0, -30.0, -5.0, 40.0, 30.0, 3.0]
bev_n_voxels = [200, 200, 4]
bev_voxel_size = [0.3, 0.3, 1.5]

model = dict(
    n_voxels=[bev_n_voxels],
    voxel_size=[bev_voxel_size],
    bbox_head=dict(
        anchor_generator=dict(
            ranges=[[-20.0, -30.0, -1.8, 40.0, 30.0, -1.8]])))

# MMCV 1.4 cannot patch individual inherited list entries, so keep an explicit
# copy of the short C0 pipelines and change only the range-dependent stages.
image_transform_backend = __import__('os').environ.get(
    'FASTBEV_IMAGE_BACKEND', 'opencv').lower()
bev_horizontal_flip_prob = float(__import__('os').environ.get(
    'FASTBEV_BEV_FLIP_PROB', '0.5'))
image_horizontal_flip_prob = float(__import__('os').environ.get(
    'FASTBEV_IMAGE_FLIP_PROB', '0.5'))
class_names = ['vehicle', 'pedestrian', 'obstacle']
file_client_args = dict(backend='disk')
data_config = dict(
    input_size=(384, 704),
    resize=(-0.02, 0.02),
    crop=(-0.02, 0.02),
    rot=(-2.0, 2.0),
    flip=True,
    flip_prob=image_horizontal_flip_prob,
    test_input_size=(384, 704),
    test_resize=0.0,
    test_rotate=0.0,
    test_flip=False,
    pad=(0, 0, 0, 0),
    pad_divisor=32,
    pad_color=(0, 0, 0))
img_norm_cfg = dict(
    mean=[123.675, 116.28, 103.53],
    std=[58.395, 57.12, 57.375],
    to_rgb=True)

train_pipeline = [
    dict(type='MoraiMultiViewPipeline', n_images=4, transforms=[
        dict(type='LoadImageFromFile', file_client_args=file_client_args)]),
    dict(type='LoadAnnotations3D', with_bbox_3d=True, with_label_3d=True,
         with_bbox=False, with_label=False),
    dict(type='LoadPointsFromFile', dummy=True, coord_type='LIDAR',
         load_dim=5, use_dim=5),
    dict(type='RandomFlip3D', flip_2d=False, sync_2d=False,
         flip_ratio_bev_horizontal=bev_horizontal_flip_prob,
         flip_ratio_bev_vertical=0.0, update_img2lidar=True),
    dict(type='GlobalRotScaleTrans', rot_range=[-0.20, 0.20],
         scale_ratio_range=[0.95, 1.05],
         translation_std=[0.05, 0.05, 0.05], update_img2lidar=True),
    dict(type='MoraiAugImageMultiViewImage', data_config=data_config,
         backend=image_transform_backend),
    dict(type='ObjectRangeFilter', point_cloud_range=point_cloud_range),
    dict(type='KittiSetOrigin', point_cloud_range=point_cloud_range),
    dict(type='NormalizeMultiviewImage', **img_norm_cfg),
    dict(type='DefaultFormatBundle3D', class_names=class_names),
    dict(type='Collect3D',
         keys=['img', 'gt_bboxes_3d', 'gt_labels_3d'])]

test_pipeline = [
    dict(type='MoraiMultiViewPipeline', n_images=4, transforms=[
        dict(type='LoadImageFromFile', file_client_args=file_client_args)]),
    dict(type='LoadPointsFromFile', dummy=True, coord_type='LIDAR',
         load_dim=5, use_dim=5),
    dict(type='MoraiAugImageMultiViewImage', data_config=data_config,
         backend=image_transform_backend, is_train=False),
    dict(type='KittiSetOrigin', point_cloud_range=point_cloud_range),
    dict(type='NormalizeMultiviewImage', **img_norm_cfg),
    dict(type='DefaultFormatBundle3D', class_names=class_names,
         with_label=False),
    dict(type='Collect3D', keys=['img'])]

data = dict(
    train=dict(pipeline=train_pipeline),
    val=dict(pipeline=test_pipeline),
    test=dict(pipeline=test_pipeline))

wandb_config = dict(
    experiment='BEV A 0.3m rear20 front40 side30',
    bev_range_m='x[-20,40] y[-30,30]',
    bev_grid_m=0.3,
    bev_n_voxels=bev_n_voxels,
    bev_xy_cells=40000)
