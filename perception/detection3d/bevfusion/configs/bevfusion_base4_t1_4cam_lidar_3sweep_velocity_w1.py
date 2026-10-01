"""Base4 T1 diagnostic with full-strength velocity regression weights."""

_base_ = ['./bevfusion_base4_t1_4cam_lidar_3sweep_velocity.py']

model = dict(
    bbox_head=dict(
        train_cfg=dict(
            code_weights=[
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0,
                1.0, 1.0,
            ])))
