_base_ = ['./fastbev_m0_morai_4cam_s384x704.py']

# Preserve the original 1.0 m-grid diagnostic even though production now uses
# stride=1.  The sibling overfit20_stride1 config overrides this back to 1.
model = dict(neck_3d=dict(stride=2))

data_root = '/home/luv/ajounice_data/morai_od/processed/fastbev_sunny13_1500_v1/'

# Deterministic image preprocessing for a small-set memorization test.  The
# train split contains 1,050 frames; load_interval=50 selects 21 spread-out
# samples without creating another annotation file outside the repository.
data_config = dict(
    input_size=(384, 704),
    resize=(0.0, 0.0),
    crop=(0.0, 0.0),
    rot=(0.0, 0.0),
    flip=False,
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
class_names = ['vehicle', 'pedestrian', 'obstacle']
image_transform_backend = __import__('os').environ.get(
    'FASTBEV_IMAGE_BACKEND', 'pil').lower()

train_pipeline = [
    dict(type='MoraiMultiViewPipeline', n_images=4, transforms=[
        dict(type='LoadImageFromFile', file_client_args=file_client_args)]),
    dict(type='LoadAnnotations3D', with_bbox_3d=True, with_label_3d=True,
         with_bbox=False, with_label=False),
    dict(type='LoadPointsFromFile', dummy=True, coord_type='LIDAR',
         load_dim=5, use_dim=5),
    dict(type='MoraiAugImageMultiViewImage', data_config=data_config,
         backend=image_transform_backend, is_train=False),
    dict(type='ObjectRangeFilter',
         point_cloud_range=[-50, -50, -5, 50, 50, 3]),
    dict(type='KittiSetOrigin',
         point_cloud_range=[-50, -50, -5, 50, 50, 3]),
    dict(type='NormalizeMultiviewImage', **img_norm_cfg),
    dict(type='DefaultFormatBundle3D', class_names=class_names),
    dict(type='Collect3D',
         keys=['img', 'gt_bboxes_3d', 'gt_labels_3d'])]

data = dict(
    samples_per_gpu=1,
    workers_per_gpu=0,
    train=dict(load_interval=50, pipeline=train_pipeline),
    # Overfit evaluation intentionally uses exactly the same 21 train frames.
    # This is a memorization diagnostic, not a generalization score.
    val=dict(
        ann_file=data_root + 'fastbev_infos/morai_infos_train.pkl',
        load_interval=50),
    test=dict(
        ann_file=data_root + 'fastbev_infos/morai_infos_train.pkl',
        load_interval=50))

# The upstream 1,000-iteration warm-up would cover this entire tiny run.
# Disable it so a decreasing loss can be observed within 30 x 21 iterations.
lr_config = dict(
    _delete_=True,
    policy='step',
    warmup=None,
    step=[20],
    gamma=0.1)

runner = dict(type='EpochBasedRunner', max_epochs=30)
total_epochs = 30
checkpoint_config = dict(interval=10)
log_config = dict(
    interval=1,
    hooks=[dict(type='TextLoggerHook')])

# First prove deterministic FP32 memorization.  Mixed precision can be enabled
# for the full run after this baseline is known to converge.
fp16 = None
evaluation = dict(interval=10)
