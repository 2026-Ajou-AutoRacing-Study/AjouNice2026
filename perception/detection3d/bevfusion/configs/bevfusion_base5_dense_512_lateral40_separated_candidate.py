"""Cost-only lateral +/-40m counterpart; not a production/training decision.

Keep 512x1056, X +/-100m, .075m voxel/.30m camera BEV/.50m depth.
Minimal stride-compatible internal Y padding is +/-40.8m (1088 voxels).
Native +/-30m-derived GT is NOT sufficient to train this widened candidate.
"""
from copy import deepcopy

_base_ = './bevfusion_base5_dense_512_separated_candidate.py'
point_cloud_range = [-100., -40., -3., 100., 40., 2.]
origin = [-100.8, -40.8]
voxel_size = [.075, .075, .2]
grid = [2688, 1088, 41]
model = dict(
    data_preprocessor=dict(voxelize_cfg=dict(
        point_cloud_range=[*origin, -5., 100.8, 40.8, 3.])),
    pts_middle_encoder=dict(sparse_shape=[1088, 2688, 41]),
    view_transform=dict(ybound=[-40.8, 40.8, .3]),
    bbox_head=dict(
        train_cfg=dict(grid_size=grid, voxel_size=voxel_size,
                       point_cloud_range=[*origin, -3., 100.8, 40.8, 2.]),
        test_cfg=dict(grid_size=grid, pc_range=origin),
        bbox_coder=dict(pc_range=origin, post_center_range=point_cloud_range)))
train_pipeline = deepcopy(_base_.train_pipeline)
test_pipeline = deepcopy(_base_.test_pipeline)
for pipeline in (train_pipeline, test_pipeline):
    for transform in pipeline:
        if transform['type'] in ('PointsRangeFilter', 'ObjectRangeFilter'):
            transform['point_cloud_range'] = point_cloud_range
train_dataloader = dict(dataset=dict(pipeline=train_pipeline))
val_dataloader = dict(dataset=dict(pipeline=test_pipeline))
test_dataloader = dict(dataset=dict(pipeline=test_pipeline))
