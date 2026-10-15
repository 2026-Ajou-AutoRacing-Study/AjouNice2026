"""5cm/20cm, depth 25cm training candidate; not a main-run approval.

Same semantic X +/-100m, Y +/-30m ROI and source labels. Integral internal
padding matches the qualified common050 inference grid, not a wider task ROI.
The camera-only audited initializer is reused; LiDAR/depth/head start fresh.
"""
_base_ = './bevfusion_base5_dense_512_depth25_runtime_candidate.py'
voxel_size = [.05, .05, .2]
grid = [4032, 1344, 41]
origin = [-100.8, -33.6]
voxelization_range = [-100.8, -33.6, -5., 100.8, 33.6, 3.]
model = dict(
    data_preprocessor=dict(voxelize_cfg=dict(
        voxel_size=voxel_size, point_cloud_range=voxelization_range)),
    pts_middle_encoder=dict(sparse_shape=[1344, 4032, 41]),
    view_transform=dict(xbound=[-100.8, 100.8, .2], ybound=[-33.6, 33.6, .2]),
    bbox_head=dict(
        train_cfg=dict(grid_size=grid, voxel_size=voxel_size,
                       point_cloud_range=[-100.8, -33.6, -3., 100.8, 33.6, 2.]),
        test_cfg=dict(grid_size=grid, voxel_size=voxel_size[:2], pc_range=origin),
        bbox_coder=dict(voxel_size=voxel_size[:2], pc_range=origin)))
