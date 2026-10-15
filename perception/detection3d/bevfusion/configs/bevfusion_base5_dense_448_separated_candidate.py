"""2026-10-10 tested cost/wiring candidate, NOT selected production/training init.

4 classes, 7D, +/-100x30 semantic ROI; .075m voxel/.30m camera BEV/.50m depth.
448x896 common bottom crop + separated pooling; random weights for probes only.
Before full training: bind crop-specific infos, audited pretrained, real 4090 cost.
"""
from copy import deepcopy
_base_='./bevfusion_base5_independent_100m.py'
custom_imports=dict(imports=[
    'projects.BEVFusion.bevfusion','perception.detection3d.bevfusion.plugin',
    'perception.detection3d.bevfusion.plugin.morai_base5_dataset',
    'perception.detection3d.bevfusion.plugin.morai_rectangular_bev',
    'perception.detection3d.bevfusion.plugin.morai_separated_depth_lss'],allow_failed_imports=False)
origin=[-100.8,-32.4];voxel_size=[.075,.075,.2];grid=[2688,864,41]
model=dict(
    data_preprocessor=dict(voxelize_cfg=dict(voxel_size=voxel_size,
        point_cloud_range=[*origin,-5.,100.8,32.4,3.])),
    pts_middle_encoder=dict(sparse_shape=[864,2688,41]),
    view_transform=dict(type='MoraiSeparatedRectangularDepthLSSTransform',
        image_size=[448,896],feature_size=[56,112],xbound=[-100.8,100.8,.3],
        ybound=[-32.4,32.4,.3],dbound=[1.,100.,.5]),
    bbox_head=dict(train_cfg=dict(grid_size=grid,voxel_size=voxel_size,
        point_cloud_range=[*origin,-3.,100.8,32.4,2.]),
        test_cfg=dict(grid_size=grid,voxel_size=voxel_size[:2],pc_range=origin),
        bbox_coder=dict(voxel_size=voxel_size[:2],pc_range=origin)))
train_pipeline=deepcopy(_base_.train_pipeline);test_pipeline=deepcopy(_base_.test_pipeline)
for pipeline in (train_pipeline,test_pipeline):
    for transform in pipeline:
        if transform['type']=='MoraiImageAug3D':
            transform['final_dim']=[448,896];transform['resize_lim']=[.7,.7]
train_dataloader=dict(dataset=dict(pipeline=train_pipeline))
val_dataloader=dict(dataset=dict(pipeline=test_pipeline))
test_dataloader=dict(dataset=dict(pipeline=test_pipeline))
load_from=None
