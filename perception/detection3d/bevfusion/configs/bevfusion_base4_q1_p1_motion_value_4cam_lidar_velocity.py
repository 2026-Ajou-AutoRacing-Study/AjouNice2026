"""Q1-P1: explicit delta-position/time values and temporal velocity head."""

_base_ = ['./bevfusion_base4_q1_query_memory_4cam_lidar_velocity.py']

model = dict(
    bbox_head=dict(
        temporal_velocity_aux_weight=0.5,
        initial_velocity_fusion_logit=-2.0,
        query_memory=dict(
            use_motion_value=True,
        ),
    ),
)
