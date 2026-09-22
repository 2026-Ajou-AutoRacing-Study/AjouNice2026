"""Base4 T1 with 5x LR on both newly introduced temporal parameters."""

_base_ = ['./bevfusion_base4_t1_4cam_lidar_3sweep_velocity_w1.py']

optim_wrapper = dict(
    paramwise_cfg=dict(
        custom_keys={
            'bbox_head.prediction_heads.0.vel': dict(lr_mult=5.0),
            # The adapted V1 checkpoint copies XYZI weights and initializes
            # the fifth point-age input at zero.  Accelerate this first sparse
            # layer as well, otherwise the temporal cue learns at the small
            # spatial-backbone fine-tuning LR.
            'pts_middle_encoder.conv_input.0': dict(lr_mult=5.0),
        }))
