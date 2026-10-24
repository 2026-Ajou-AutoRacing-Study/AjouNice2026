"""Aggressive Base4 T1 velocity learnability diagnostic.

This config is intentionally unsuitable for model promotion.  It raises only
the planar velocity loss and the newly introduced temporal/velocity parameter
learning rates so an exact-sample overfit can answer whether the current head
and input path are capable of representing the target at all.
"""

_base_ = ['./bevfusion_base4_t1_4cam_lidar_3sweep_velocity_w1_head5x.py']

model = dict(
    bbox_head=dict(
        train_cfg=dict(
            code_weights=[
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0,
                5.0, 5.0,
            ])))
