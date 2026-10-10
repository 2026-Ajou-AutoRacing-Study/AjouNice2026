"""Current Base5 input, 2026-10-11: preencoded native Q90 RGB in every split.

No JPEG re-encoding in train/val/analysis. Geometry augmentation stays enabled.
PNG is a local conversion source only; auxiliary lossless labels stay unchanged.
Not a production E25 change. Use the guarded direct-training runner.
"""
from copy import deepcopy

_base_ = './bevfusion_base5_dense_512_separated_candidate.py'

required_camera_input_policy = 'native_jpeg90'
def _base5_input_pipeline(source):
    pipeline = []
    for transform in deepcopy(source):
        if transform['type'] == 'MoraiOnlineImageDomainAug3D':
            continue
        if transform['type'] == 'BEVLoadMultiViewImageFromFiles':
            transform['to_float32'] = False
        if transform['type'] == 'MoraiImageAug3D':
            transform['output_dtype'] = 'uint8'
        pipeline.append(transform)
        if transform['type'] == 'LoadPointsFromFile':
            pipeline.append(dict(type='MoraiFilterInvalidPoints'))
    return pipeline


train_pipeline = _base5_input_pipeline(_base_.train_pipeline)
test_pipeline = _base5_input_pipeline(_base_.test_pipeline)
train_dataloader = dict(dataset=dict(pipeline=train_pipeline))
val_dataloader = dict(dataset=dict(pipeline=test_pipeline))
test_dataloader = dict(dataset=dict(pipeline=test_pipeline))
