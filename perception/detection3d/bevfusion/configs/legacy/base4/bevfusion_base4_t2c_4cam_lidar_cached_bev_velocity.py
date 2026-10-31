"""Base4 T2C: train T2R with encode-then-warp cached-BEV semantics."""

_base_ = ['./bevfusion_base4_t2r_4cam_lidar_temporal_bev_velocity.py']

point_cloud_range = [-30.0, -30.0, -3.0, 30.0, 30.0, 2.0]

model = dict(
    temporal_feature_warp=True,
    temporal_point_cloud_range=point_cloud_range)
