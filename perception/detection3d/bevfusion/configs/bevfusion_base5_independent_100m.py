"""Base5 fresh four-class candidate. No E25/temporal/velocity initialization.

100m longitudinal / 30m lateral ROI, 0.10m voxels, 320x896 cameras.
Model geometry is a candidate until full GPU forward/backward validation.
"""
from copy import deepcopy

_base_='./bevfusion_base3_b0_4cam_lidar.py'
classes=('vehicle','pedestrian','obstacle','animal')
class_names=classes
metainfo=dict(classes=classes)
dataset_type='MoraiBase5Dataset'
custom_imports=dict(imports=['projects.BEVFusion.bevfusion','perception.detection3d.bevfusion.plugin',
    'perception.detection3d.bevfusion.plugin.morai_base5_dataset',
    'perception.detection3d.bevfusion.plugin.morai_rectangular_bev'],allow_failed_imports=False)
point_cloud_range=[-100.,-30.,-3.,100.,30.,2.]
voxelization_range=[-100.,-32.,-5.,100.,32.,3.]
voxel_size=[.1,.1,.2]
grid=[2000,640,41] # 600 is not divisible by the SECOND/FPN total XY stride 16.
load_from=None
model=dict(
    data_preprocessor=dict(voxelize_cfg=dict(voxel_size=voxel_size,point_cloud_range=voxelization_range)),
    pts_middle_encoder=dict(sparse_shape=[640,2000,41]),
    view_transform=dict(type='MoraiRectangularDepthLSSTransform',image_size=[320,896],feature_size=[40,112],
        xbound=[-100.,100.,.4],ybound=[-32.,32.,.4],dbound=[1.,100.,1.]),
    bbox_head=dict(type='MoraiRectangularTransFusionHead',num_classes=4,
        train_cfg=dict(grid_size=grid,voxel_size=voxel_size,point_cloud_range=[-100.,-32.,-3.,100.,32.,2.]),
        test_cfg=dict(grid_size=grid,pc_range=[-100.,-32.],voxel_size=voxel_size[:2],nms_type=None),
        bbox_coder=dict(pc_range=[-100.,-32.],voxel_size=voxel_size[:2],post_center_range=point_cloud_range)))
train_pipeline=deepcopy(_base_.train_pipeline)
test_pipeline=deepcopy(_base_.test_pipeline)
for pipeline in (train_pipeline,test_pipeline):
    for transform in pipeline:
        if transform['type']=='MoraiImageAug3D':
            transform['final_dim']=[320,896];transform['resize_lim']=[.70,.70]
        if transform['type'] in ('PointsRangeFilter','ObjectRangeFilter'):
            transform['point_cloud_range']=point_cloud_range
        if transform['type']=='ObjectNameFilter': transform['classes']=classes
info_root='/workspace/AjouNice2026/artifacts/datasets/base5_v2_pilot'
train_dataloader=dict(batch_size=1,num_workers=2,dataset=dict(type=dataset_type,metainfo=metainfo,
    ann_file=info_root+'/morai_base5_infos_train.pkl',pipeline=train_pipeline))
val_dataloader=dict(batch_size=1,num_workers=2,dataset=dict(type=dataset_type,metainfo=metainfo,
    ann_file=info_root+'/morai_base5_infos_val.pkl',pipeline=test_pipeline))
test_dataloader=dict(batch_size=1,num_workers=2,dataset=dict(type=dataset_type,metainfo=metainfo,
    ann_file=info_root+'/morai_base5_infos_test.pkl',pipeline=test_pipeline))
val_evaluator=dict(point_cloud_range=point_cloud_range)
test_evaluator=dict(point_cloud_range=point_cloud_range)
