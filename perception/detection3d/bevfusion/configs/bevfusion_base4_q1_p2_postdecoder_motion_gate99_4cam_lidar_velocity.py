"""Q1-P2d: prefer temporal velocity whenever causal memory is available."""

_base_ = ['./bevfusion_base4_q1_p2_postdecoder_motion_4cam_lidar_velocity.py']

model = dict(
    bbox_head=dict(
        initial_velocity_fusion_logit=4.59511985,
    ),
)
