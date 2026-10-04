"""Q1-P2: compute temporal motion from decoder-refined query centres."""

_base_ = ['./bevfusion_base4_q1_p1_motion_value_4cam_lidar_velocity.py']

model = dict(
    bbox_head=dict(
        query_memory_after_decoder=True,
    ),
)
