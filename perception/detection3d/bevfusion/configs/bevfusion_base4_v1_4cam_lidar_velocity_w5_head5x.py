"""Aggressive single-frame velocity overfit control for the Base4 T1 gate."""

_base_ = ['./bevfusion_base4_v1_4cam_lidar_velocity_w1_head5x.py']

model = dict(
    bbox_head=dict(
        train_cfg=dict(
            code_weights=[
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0,
                5.0, 5.0,
            ])))
