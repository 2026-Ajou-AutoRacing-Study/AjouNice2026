_base_ = [
    '../../../../third_party/fastbev/configs/fastbev/exp/paper/fastbev_m0_r18_s256x704_v200x200x4_c192_d2_f4.py'
]

custom_imports = dict(imports=['fastbev_plugin'], allow_failed_imports=False)

class_names = ['vehicle', 'pedestrian', 'obstacle']
dataset_type = 'MoraiMultiViewDataset'
data_root = '/home/luv/ajounice_data/morai_od/processed/fastbev_sunny13_1500_v1/'
point_cloud_range = [-50, -50, -5, 50, 50, 3]

model = dict(
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
    dict(type='LoadAnnotations3D', with_bbox=True, with_label=True),
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
    with_box2d=False,
    box_type_3d='LiDAR',
    load_interval=1,
    sequential=False,
    n_times=1,
    with_velocity=True,
    use_valid_flag=True)

data = dict(
    samples_per_gpu=1,
    workers_per_gpu=2,
    train=dict(**common_dataset, pipeline=train_pipeline, test_mode=False,
               ann_file=data_root + 'fastbev_infos/morai_infos_train.pkl'),
    val=dict(**common_dataset, pipeline=test_pipeline, test_mode=True,
             ann_file=data_root + 'fastbev_infos/morai_infos_val.pkl'),
    test=dict(**common_dataset, pipeline=test_pipeline, test_mode=True,
              ann_file=data_root + 'fastbev_infos/morai_infos_test.pkl'))

evaluation = dict(interval=1)
total_epochs = 12
