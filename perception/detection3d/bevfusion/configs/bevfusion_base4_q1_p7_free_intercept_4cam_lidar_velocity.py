"""Q1-P7: fit a free-intercept line through causal query centers."""

_base_ = ['./bevfusion_base4_q1_p6_age_wls_4cam_lidar_velocity.py']

model = dict(
    bbox_head=dict(
        query_memory=dict(
            # P6's dt^2 pair-slope weighting assumes the current center is an
            # exact intercept.  P7 instead fits intercept and slope jointly.
            direct_velocity_age_power=0.0,
            direct_velocity_fit_intercept=True,
            direct_velocity_current_weight=1.0,
        ),
    ),
)
