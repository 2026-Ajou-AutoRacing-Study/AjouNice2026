"""C1-A: apply 1.5x x/y regression weight to obstacle positives."""

_base_ = ['./fastbev_m0_morai_4cam_s384x704.py']

model = dict(
    bbox_head=dict(
        type='ObstacleAwareFreeAnchor3DHead',
        obstacle_label=2,
        obstacle_xy_loss_weight=1.5,
        mask_obstacle_direction=False,
        mask_near_square_obstacle_yaw=False))

wandb_config = dict(
    experiment='C1-A obstacle XY loss 1.5x',
    obstacle_xy_loss_weight=1.5,
    obstacle_direction_loss=True,
    obstacle_yaw_loss=True)
