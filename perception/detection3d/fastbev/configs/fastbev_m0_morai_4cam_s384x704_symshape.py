"""H1-B: direction-mask obstacles and yaw-mask near-square XY footprints."""

_base_ = ['./fastbev_m0_morai_4cam_s384x704.py']

model = dict(
    bbox_head=dict(
        type='SymmetryAwareFreeAnchor3DHead',
        obstacle_label=2,
        mask_obstacle_direction=True,
        mask_near_square_obstacle_yaw=True,
        near_square_aspect_ratio=1.2))

wandb_config = dict(
    experiment='H1-B obstacle shape-aware yaw-mask',
    obstacle_direction_loss=False,
    near_square_obstacle_yaw_loss=False,
    near_square_obstacle_aspect_ratio=1.2)
