"""Q1-P3: multi-frame direct motion plus a learned residual velocity."""

_base_ = ['./bevfusion_base4_q1_p2_postdecoder_motion_gate90_4cam_lidar_velocity.py']

model = dict(
    bbox_head=dict(
        direct_motion_velocity_residual=True,
    ),
)
