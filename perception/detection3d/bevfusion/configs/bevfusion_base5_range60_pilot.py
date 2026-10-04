"""Opt-in +/-60m longitudinal, +/-30m lateral ROI wiring pilot.

Same .075m voxels and .6m output cells as E25; NOT trained for far vehicles.
Camera depth remains [1,60,.5] to preserve its learned depth-head shape.
Thus outer-ROI image evidence is not guaranteed. Retraining/design required.
"""
from copy import deepcopy
_base_ = './bevfusion_base3_b0_4cam_lidar.py'
point_cloud_range = [-60., -30., -3., 60., 30., 2.]
voxelization_range = [-60., -30., -5., 60., 30., 3.]
grid_size = [1600, 800, 41]
custom_imports = dict(imports=list(_base_.custom_imports.imports) + [
    'perception.detection3d.bevfusion.plugin.morai_view_geometry_hook'],
    allow_failed_imports=False)
custom_hooks = deepcopy(_base_.custom_hooks) + [dict(type='MoraiConfigViewGeometryHook')]
model = dict(
    data_preprocessor=dict(voxelize_cfg=dict(point_cloud_range=voxelization_range)),
    pts_middle_encoder=dict(sparse_shape=grid_size),
    view_transform=dict(xbound=[-60., 60., .3], ybound=[-30., 30., .3]),
    bbox_head=dict(
        train_cfg=dict(grid_size=grid_size, point_cloud_range=point_cloud_range),
        test_cfg=dict(grid_size=grid_size, pc_range=[-60., -30.]),
        bbox_coder=dict(pc_range=[-60., -30.], post_center_range=point_cloud_range)))
train_pipeline = deepcopy(_base_.train_pipeline)
test_pipeline = deepcopy(_base_.test_pipeline)
for pipeline in (train_pipeline, test_pipeline):
    for transform in pipeline:
        if transform['type'] in ('PointsRangeFilter', 'ObjectRangeFilter'):
            transform['point_cloud_range'] = point_cloud_range
train_dataloader = dict(batch_size=1, dataset=dict(pipeline=train_pipeline))
val_dataloader = dict(batch_size=1, dataset=dict(pipeline=test_pipeline))
test_dataloader = dict(batch_size=1, dataset=dict(pipeline=test_pipeline))
