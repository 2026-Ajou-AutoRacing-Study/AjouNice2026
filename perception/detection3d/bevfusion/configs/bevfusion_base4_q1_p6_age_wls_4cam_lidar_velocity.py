"""Q1-P6: reduce short-dt center jitter with fixed-intercept WLS motion."""

_base_ = ['./bevfusion_base4_q1_p3_robust_motion_gate90_4cam_lidar_velocity.py']

model = dict(
    bbox_head=dict(
        query_memory=dict(
            # Pair velocity is delta/dt. Reweighting attention by dt^2 is
            # equivalent to a weighted least-squares slope with the current
            # center fixed as the trajectory intercept.
            direct_velocity_age_power=2.0,
        ),
    ),
)
