"""512x1056 counterpart of the tested 448 separated candidate; not production."""
from copy import deepcopy
_base_='./bevfusion_base5_dense_448_separated_candidate.py'
model=dict(view_transform=dict(image_size=[512,1056],feature_size=[64,132]))
train_pipeline=deepcopy(_base_.train_pipeline)
test_pipeline=deepcopy(_base_.test_pipeline)
for pipeline in (train_pipeline,test_pipeline):
    for transform in pipeline:
        if transform['type']=='MoraiImageAug3D':
            transform['final_dim']=[512,1056]
            transform['resize_lim']=[.825,.825]
train_dataloader=dict(dataset=dict(pipeline=train_pipeline))
val_dataloader=dict(dataset=dict(pipeline=test_pipeline))
test_dataloader=dict(dataset=dict(pipeline=test_pipeline))
