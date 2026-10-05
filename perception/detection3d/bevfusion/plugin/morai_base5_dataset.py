"""Opt-in four-class Base5 adapter. Does not alter Base3/E25 registration."""
from mmdet3d.registry import DATASETS
from .morai_base3_dataset import MoraiBase3Dataset


@DATASETS.register_module()
class MoraiBase5Dataset(MoraiBase3Dataset):
    METAINFO = dict(classes=('vehicle','pedestrian','obstacle','animal'),
                    palette=((0,255,0),(255,128,0),(0,128,255),(255,0,180)))
