"""Opt-in four-class Base5 adapter. Does not alter Base3/E25 registration."""
from mmdet3d.registry import DATASETS
from .morai_base3_dataset import MoraiBase3Dataset


@DATASETS.register_module()
class MoraiBase5Dataset(MoraiBase3Dataset):
    METAINFO = dict(classes=('vehicle','pedestrian','obstacle','animal'),
                    palette=((0,255,0),(255,128,0),(0,128,255),(255,0,180)))


@DATASETS.register_module()
class MoraiBase5PedestrianDataset(MoraiBase3Dataset):
    """Three training classes, with immutable animal subtype in derived infos."""
    METAINFO = dict(classes=('vehicle', 'pedestrian', 'obstacle'),
                    palette=((0,255,0),(255,128,0),(0,128,255)))

    def parse_ann_info(self, info):
        import numpy as np
        instances = info.get('instances', [])
        if any(i.get('original_semantic_class') not in ('vehicle', 'pedestrian', 'obstacle', 'animal')
               for i in instances):
            raise ValueError('Merged Base5 requires original semantic metadata')
        # Det3DDataset remaps ANY field containing "label". Source label 3 is
        # provenance, not a training label in this three-class adapter.
        parsed_info = dict(info, instances=[{k: v for k, v in i.items()
                           if k != 'original_bbox_label_3d'} for i in instances])
        ann = super().parse_ann_info(parsed_info)
        ann['original_bbox_label_3d'] = np.asarray([i['original_bbox_label_3d'] for i in instances], dtype=np.int64)
        ann['original_semantic_class'] = np.asarray([i['original_semantic_class'] for i in instances])
        ann['original_asset'] = np.asarray([i['asset'] for i in instances])
        return ann
