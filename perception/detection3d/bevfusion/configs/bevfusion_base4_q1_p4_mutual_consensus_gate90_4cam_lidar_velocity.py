"""Q1-P4: mutual per-frame association and robust velocity consensus."""

_base_ = ['./bevfusion_base4_q1_p3_robust_motion_gate90_4cam_lidar_velocity.py']

model = dict(
    bbox_head=dict(
        query_memory=dict(
            robust_direct_velocity=True,
            robust_min_consensus_frames=2,
            robust_consensus_threshold_mps=3.0,
            robust_allow_single_mutual_pair=True,
            robust_single_pair_max_speed_mps=35.0,
        ),
    ),
)
