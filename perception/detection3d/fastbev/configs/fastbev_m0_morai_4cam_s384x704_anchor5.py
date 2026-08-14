"""Three deployment classes with five shared anchor-size candidates."""

_base_ = ['./fastbev_m0_morai_4cam_s384x704.py']

anchor_sizes = [
    [4.5, 1.9, 1.7],
    [0.6, 0.6, 1.75],
    [0.75, 0.75, 0.92],
    [0.32, 0.60, 3.01],
    [0.62, 1.79, 0.90],
]

model = dict(
    bbox_head=dict(
        num_classes=3,
        anchor_generator=dict(
            ranges=[[-50, -50, -1.8, 50, 50, -1.8]],
            sizes=anchor_sizes,
            custom_values=[0, 0],
            rotations=[0, 1.57],
            reshape_out=True)))

wandb_tags = [
    tag.strip() for tag in __import__('os').environ.get(
        'WANDB_TAGS',
        'fastbev,morai,4cam,anchor5,opencv,flip50-50,fp16').split(',')
    if tag.strip()
]
wandb_config = dict(
    architecture='FastBEV-M0-ResNet18',
    experiment='obstacle-anchor5-shared-v1',
    internal_classes=['vehicle', 'pedestrian', 'obstacle'],
    deployment_classes=['vehicle', 'pedestrian', 'obstacle'],
    image_transform_backend=__import__('os').environ.get(
        'FASTBEV_IMAGE_BACKEND', 'opencv').lower(),
    bev_horizontal_flip_prob=float(__import__('os').environ.get(
        'FASTBEV_BEV_FLIP_PROB', '0.5')),
    image_horizontal_flip_prob=float(__import__('os').environ.get(
        'FASTBEV_IMAGE_FLIP_PROB', '0.5')),
    dataset='MORAI-fastbev-sunny13-1500-v1',
    train_frames=1050,
    val_frames=228,
    test_frames=222,
    anchor_sizes=anchor_sizes,
    anchors_per_bev_cell=10,
    selection_metric='center_mAP')

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
                'WANDB_NAME', 'fastbev_morai_anchor5'),
            id=__import__('os').environ.get('WANDB_RUN_ID'),
            resume=__import__('os').environ.get('WANDB_RESUME', 'allow'),
            group=__import__('os').environ.get(
                'WANDB_RUN_GROUP', 'sunny13-1500-obstacle-anchor'),
            job_type=__import__('os').environ.get(
                'WANDB_JOB_TYPE', 'train'),
            tags=wandb_tags,
            notes=__import__('os').environ.get(
                'WANDB_NOTES',
                'Three classes with five shared anchor-size candidates'),
            config=wandb_config),
        interval=10))
log_config = dict(interval=10, hooks=log_hooks)
