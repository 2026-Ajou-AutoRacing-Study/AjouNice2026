"""Base4 V1 diagnostic with unit-weight planar velocity regression."""

_base_ = ['./bevfusion_base4_v1_4cam_lidar_velocity.py']

# The inherited MORAI geometry channels remain unchanged.  Only the final
# vx/vy weights differ from the upstream nuScenes convention (0.2, 0.2), so
# this config can isolate whether velocity is underweighted for the highway
# vehicle-only corpus.
model = dict(
    bbox_head=dict(
        train_cfg=dict(
            code_weights=[1.0, 1.0, 1.0, 1.0, 1.0,
                          1.0, 1.0, 1.0, 1.0, 1.0])))
