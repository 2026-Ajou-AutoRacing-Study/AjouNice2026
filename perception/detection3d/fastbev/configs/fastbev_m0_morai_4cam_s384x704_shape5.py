"""Obstacle shape-subclass ablation with a merged three-class evaluator."""

_base_ = ['./fastbev_m0_morai_4cam_s384x704.py']

class_names = [
    'vehicle',
    'pedestrian',
    'compact_obstacle',
    'tall_obstacle',
    'long_obstacle',
]
shape_data_root = __import__('os').environ.get(
    'FASTBEV_SHAPE_DATA_ROOT',
    __import__('os').path.join(
        __import__('os').environ.get(
            'AJOUNICE_ROOT', '/home/luv/AjouNice2026'),
        'artifacts/detection3d/fastbev/data/'
        'fastbev_sunny13_1500_shape5_v1')) + '/'
image_transform_backend = __import__('os').environ.get(
    'FASTBEV_IMAGE_BACKEND', 'opencv').lower()
if image_transform_backend not in ('pil', 'opencv'):
    raise ValueError('FASTBEV_IMAGE_BACKEND must be pil or opencv')
bev_horizontal_flip_prob = float(__import__('os').environ.get(
    'FASTBEV_BEV_FLIP_PROB', '0.5'))
image_horizontal_flip_prob = float(__import__('os').environ.get(
    'FASTBEV_IMAGE_FLIP_PROB', '0.5'))

model = dict(
    bbox_head=dict(
        num_classes=5,
        anchor_generator=dict(
            ranges=[[-50, -50, -1.8, 50, 50, -1.8]],
            sizes=[
                [4.5, 1.9, 1.7],
                [0.6, 0.6, 1.75],
                [0.75, 0.75, 0.92],
                [0.32, 0.60, 3.01],
                [0.62, 1.79, 0.90],
            ],
            custom_values=[0, 0],
            rotations=[0, 1.57],
            reshape_out=True)),
    test_cfg=dict(
        nms_type_list=['rotate'] * 5,
        nms_thr_list=[0.2] * 5,
        nms_radius_thr_list=[4.0, 0.5, 0.9, 0.7, 1.6],
        nms_rescale_factor=[1.0] * 5))

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
file_client_args = dict(backend='disk')
img_norm_cfg = dict(
    mean=[123.675, 116.28, 103.53],
    std=[58.395, 57.12, 57.375],
    to_rgb=True)
point_cloud_range = [-50, -50, -5, 50, 50, 3]

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
    dict(type='Collect3D', keys=['img', 'gt_bboxes_3d', 'gt_labels_3d'])]

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

shape_dataset = dict(
    classes=class_names,
    obstacle_shape_merge=True,
    # Selected on validation only: 0.1 produced merged center mAP 0.5726,
    # versus 0.5569/0.5568/0.5562 at 0.2/0.5/0.8.
    merged_obstacle_nms_iou_threshold=0.1)
data = dict(
    train=dict(
        **shape_dataset,
        data_root=shape_data_root,
        pipeline=train_pipeline,
        ann_file=shape_data_root + 'fastbev_infos/morai_infos_train.pkl'),
    val=dict(
        **shape_dataset,
        data_root=shape_data_root,
        pipeline=test_pipeline,
        ann_file=shape_data_root + 'fastbev_infos/morai_infos_val.pkl'),
    test=dict(
        **shape_dataset,
        data_root=shape_data_root,
        pipeline=test_pipeline,
        ann_file=shape_data_root + 'fastbev_infos/morai_infos_test.pkl'))

evaluation = dict(
    interval=1,
    metric=('center', 'tp', 'iou'),
    center_thresholds=(0.5, 1.0, 2.0, 4.0),
    distance_bins=((0.0, 10.0), (10.0, 20.0), (20.0, 30.0)),
    compact=True,
    save_best='merged_center_mAP',
    rule='greater')

wandb_tags = [
    tag.strip() for tag in __import__('os').environ.get(
        'WANDB_TAGS',
        'fastbev,morai,4cam,shape5,opencv,flip50-50,fp16').split(',')
    if tag.strip()
]
wandb_config = dict(
    architecture='FastBEV-M0-ResNet18',
    experiment='obstacle-shape-subclasses-v1',
    internal_classes=class_names,
    deployment_classes=['vehicle', 'pedestrian', 'obstacle'],
    obstacle_shape_ratio_threshold=2.0,
    merged_obstacle_nms_iou_threshold=0.1,
    image_transform_backend=image_transform_backend,
    bev_horizontal_flip_prob=bev_horizontal_flip_prob,
    image_horizontal_flip_prob=image_horizontal_flip_prob,
    dataset='MORAI-fastbev-sunny13-1500-shape5-v1',
    train_frames=1050,
    val_frames=228,
    test_frames=222,
    anchor_sizes=model['bbox_head']['anchor_generator']['sizes'],
    selection_metric='merged_center_mAP')
log_hooks = [dict(type='TextLoggerHook'), dict(type='TensorboardLoggerHook')]
if __import__('os').environ.get('FASTBEV_DISABLE_WANDB', '0') != '1':
    log_hooks.append(dict(
        type='MoraiWandbLoggerHook',
        with_step=False,
        init_kwargs=dict(
            entity=__import__('os').environ.get(
                'WANDB_ENTITY', 'AjouNice2026'),
            project=__import__('os').environ.get(
                'WANDB_PROJECT', 'morai-fastbev'),
            name=__import__('os').environ.get(
                'WANDB_NAME', 'fastbev_morai_obstacle_shape5'),
            id=__import__('os').environ.get('WANDB_RUN_ID'),
            resume=__import__('os').environ.get('WANDB_RESUME', 'allow'),
            group=__import__('os').environ.get(
                'WANDB_RUN_GROUP', 'sunny13-1500-obstacle-shape'),
            job_type=__import__('os').environ.get(
                'WANDB_JOB_TYPE', 'train'),
            tags=wandb_tags,
            notes=__import__('os').environ.get(
                'WANDB_NOTES',
                'Five internal classes with merged three-class evaluation'),
            config=wandb_config),
        interval=10))
log_config = dict(interval=10, hooks=log_hooks)
