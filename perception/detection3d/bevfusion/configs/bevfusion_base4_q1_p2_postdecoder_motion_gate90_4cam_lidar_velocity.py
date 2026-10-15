"""Q1-P2c: post-decoder motion with a 90% initial velocity fusion gate."""

_base_ = ['./bevfusion_base4_q1_p2_postdecoder_motion_4cam_lidar_velocity.py']

model = dict(
    bbox_head=dict(
        initial_velocity_fusion_logit=2.197224577,
    ),
)
