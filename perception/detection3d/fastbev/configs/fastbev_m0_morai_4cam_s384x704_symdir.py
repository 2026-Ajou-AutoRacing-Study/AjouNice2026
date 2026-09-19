"""H1-A: remove only obstacle front/rear direction supervision."""

_base_ = ['./fastbev_m0_morai_4cam_s384x704.py']

model = dict(
    bbox_head=dict(
        type='SymmetryAwareFreeAnchor3DHead',
        obstacle_label=2,
        mask_obstacle_direction=True,
        mask_near_square_obstacle_yaw=False,
        near_square_aspect_ratio=1.2))

wandb_config = dict(
    experiment='H1-A obstacle direction-mask',
    obstacle_direction_loss=False,
    near_square_obstacle_yaw_loss=True)
