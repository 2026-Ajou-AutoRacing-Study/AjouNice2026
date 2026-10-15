"""Base4 T2: sweep-separated, Ego-aligned temporal LiDAR BEV fusion."""

_base_ = ['./bevfusion_base4_t1_4cam_lidar_3sweep_velocity_w1.py']

model = dict(
    type='MoraiTemporalBEVFusion',
    temporal_fusion=dict(
        type='MoraiTemporalBEVGate',
        channels=256,
        num_sweeps=3,
        current_logit=4.0,
        past_logit=-2.0))

optim_wrapper = dict(
    paramwise_cfg=dict(
        custom_keys={
            'bbox_head.prediction_heads.0.vel': dict(lr_mult=5.0),
            'pts_middle_encoder.conv_input.0': dict(lr_mult=5.0),
            'temporal_fusion': dict(lr_mult=5.0),
        }))

default_hooks = dict(
    checkpoint=dict(
        save_best=['val/center/mAP', 'val/tp/vehicle_speed_MAE'],
        rule=['greater', 'less']))
