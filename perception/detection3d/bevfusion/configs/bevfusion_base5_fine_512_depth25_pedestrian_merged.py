"""User-selected Base5 three-class training; animal remains in raw metadata.

Fresh run only. Do not resume a four-class optimizer/head checkpoint. The
audited camera+sparse initializer is reusable; no E25 weights are introduced.
"""
from copy import deepcopy

_base_ = './bevfusion_base5_fine_512_depth25_training_compiled_candidate.py'
classes = ('vehicle', 'pedestrian', 'obstacle')
class_names = classes
metainfo = dict(classes=classes)
required_training_label_policy = 'animal_to_pedestrian_v1'
model = dict(bbox_head=dict(num_classes=3))
train_pipeline = deepcopy(_base_.train_pipeline)
test_pipeline = deepcopy(_base_.test_pipeline)
for pipeline in (train_pipeline, test_pipeline):
    for transform in pipeline:
        if transform['type'] == 'ObjectNameFilter':
            transform['classes'] = classes
for_split_type = 'MoraiBase5PedestrianDataset'
train_dataloader = dict(dataset=dict(type=for_split_type, metainfo=metainfo, pipeline=train_pipeline))
val_dataloader = dict(dataset=dict(type=for_split_type, metainfo=metainfo, pipeline=test_pipeline))
test_dataloader = dict(dataset=dict(type=for_split_type, metainfo=metainfo, pipeline=test_pipeline))
val_evaluator = dict(type='MoraiBase5MergedCenterMetric')
test_evaluator = dict(type='MoraiBase5MergedCenterMetric')
