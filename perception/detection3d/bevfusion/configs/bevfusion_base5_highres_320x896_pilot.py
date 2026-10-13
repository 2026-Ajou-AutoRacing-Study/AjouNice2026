"""High-resolution wiring pilot; NOT a trained production replacement.

Same +/-30m geometry/head as E25. Fine-tuning and rosbag A/B are required.
The image crop is close to baseline FoV. Dimensions must be multiples of 32:
320x880 fails depth/image concatenation because Swin pads its width to 896.
"""
from copy import deepcopy

_base_ = './bevfusion_base3_b0_4cam_lidar.py'

model = dict(view_transform=dict(image_size=[320, 896], feature_size=[40, 112]))
train_pipeline = deepcopy(_base_.train_pipeline)
test_pipeline = deepcopy(_base_.test_pipeline)
for pipeline in (train_pipeline, test_pipeline):
    for transform in pipeline:
        if transform['type'] == 'MoraiImageAug3D':
            transform['final_dim'] = [320, 896]
            transform['resize_lim'] = [0.70, 0.70]

train_dataloader = dict(batch_size=1, dataset=dict(pipeline=train_pipeline))
val_dataloader = dict(batch_size=1, dataset=dict(pipeline=test_pipeline))
test_dataloader = dict(batch_size=1, dataset=dict(pipeline=test_pipeline))
