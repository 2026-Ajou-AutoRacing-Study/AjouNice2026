"""Base4 T2.1: ordered temporal-difference residual BEV fusion."""

_base_ = ['./bevfusion_base4_t2_4cam_lidar_temporal_bev_velocity.py']

model = dict(
    temporal_fusion=dict(
        _delete_=True,
        type='MoraiTemporalBEVResidual',
        channels=256,
        num_sweeps=3,
        hidden_channels=128))
