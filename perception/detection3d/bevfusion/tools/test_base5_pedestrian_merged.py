from copy import deepcopy
import json
from pathlib import Path
import pickle
import tempfile
import unittest

from prepare_base5_pedestrian_merged import build, merge_infos, LABEL_POLICY
from prepare_base5_portable_dataset import validate
from test_base5_upright_portable import UprightPortableTest
from prepare_base5_upright_portable import build as upright_build
from run_base5_direct_training import prepare
from mmengine.config import Config
import argparse
import numpy as np
from types import SimpleNamespace
from perception.detection3d.common.evaluation.base5_subgroup import subgroup_recall


class PedestrianMergeTest(unittest.TestCase):
    def test_subgroup_matching_does_not_reuse_human_match(self):
        row = dict(info=dict(gt_boxes=np.array([[0, 0], [.5, 0]]),
                             gt_names=np.array(['pedestrian', 'pedestrian'])),
                   prediction=dict(boxes_3d=SimpleNamespace(tensor=np.array([[0, 0]])),
                                   scores_3d=np.array([.9]), labels_3d=np.array([1])),
                   original_semantic_class=np.array(['pedestrian', 'animal']),
                   assets=np.array(['Man1', 'Dog1']))
        scores = subgroup_recall([row])
        self.assertEqual(scores['subgroup/original_pedestrian/recall_1m_score010'], 1.)
        self.assertEqual(scores['subgroup/original_animal/recall_1m_score010'], 0.)
        row['prediction'] = dict(boxes_3d=SimpleNamespace(tensor=np.array([[0, 0], [.5, 0]])),
                                 scores_3d=np.array([.9, .09]), labels_3d=np.array([1, 1]))
        scores = subgroup_recall([row])
        self.assertEqual(scores['subgroup/original_animal/recall_1m_score005'], 1.)
        self.assertEqual(scores['subgroup/original_animal/recall_1m_score010'], 0.)
        row['prediction']['labels_3d'] = np.array([1, 2])
        self.assertEqual(subgroup_recall([row])['subgroup/original_animal/recall_1m_score005'], 0.)

    def test_dataset_provenance_label_is_not_remapped_twice(self):
        from perception.detection3d.bevfusion.plugin.morai_base5_dataset import MoraiBase5PedestrianDataset
        dataset = MoraiBase5PedestrianDataset.__new__(MoraiBase5PedestrianDataset)
        dataset._metainfo = dict(classes=('vehicle', 'pedestrian', 'obstacle'))
        dataset.label_mapping = {-1: -1, 0: 0, 1: 1, 2: 2}
        dataset.num_ins_per_cat = [0, 0, 0]
        from mmdet3d.structures import Box3DMode
        dataset.box_mode_3d = Box3DMode.LIDAR
        info, _ = merge_infos(self.infos())
        ann = dataset.parse_ann_info(info['data_list'][0])
        self.assertEqual(ann['gt_labels_3d'].tolist(), [0, 1, 2, 1])
        self.assertEqual(ann['original_bbox_label_3d'].tolist(), [0, 1, 2, 3])
        self.assertEqual(ann['original_semantic_class'][-1], 'animal')
        self.assertEqual(info['data_list'][0]['instances'][-1]['original_bbox_label_3d'], 3)

    def infos(self):
        return dict(metainfo=dict(classes=('vehicle', 'pedestrian', 'obstacle', 'animal')),
                    data_list=[dict(token='mixed', instances=[dict(
                        bbox_label_3d=i, asset='asset%d' % i, component_id='part%d' % i,
                        bbox_3d=[1, 2, 3, .5, .5, 1, 0], num_lidar_pts=i)
                        for i in range(4)])])

    def test_semantics_geometry_and_composite_boxes_preserved(self):
        source = self.infos()
        before = deepcopy(source)
        result, report = merge_infos(source)
        self.assertEqual(source, before)
        self.assertEqual([i['bbox_label_3d'] for i in result['data_list'][0]['instances']], [0, 1, 2, 1])
        self.assertEqual(report['merged_gt']['pedestrian'], 2)
        self.assertEqual(report['animal_assets'], {'asset3': 1})
        for original, merged in zip(source['data_list'][0]['instances'], result['data_list'][0]['instances']):
            self.assertEqual(merged.pop('original_bbox_label_3d'), original['bbox_label_3d'])
            self.assertEqual(merged.pop('original_semantic_class'), source['metainfo']['classes'][original['bbox_label_3d']])
            merged['bbox_label_3d'] = original['bbox_label_3d']
            self.assertEqual(original, merged)

    def test_rejects_wrong_order_unknown_labels_and_double_conversion(self):
        info = self.infos()
        info['metainfo']['classes'] = ('pedestrian', 'vehicle', 'obstacle', 'animal')
        with self.assertRaisesRegex(ValueError, 'order'):
            merge_infos(info)
        info = self.infos()
        info['data_list'][0]['instances'][0]['bbox_label_3d'] = -1
        with self.assertRaisesRegex(ValueError, 'Invalid'):
            merge_infos(info)
        info = self.infos()
        info['data_list'][0]['instances'][0]['original_bbox_label_3d'] = 0
        with self.assertRaisesRegex(ValueError, 'already exists'):
            merge_infos(info)
        result, _ = merge_infos(self.infos())
        with self.assertRaises(ValueError):
            merge_infos(result)

    def fixture(self, root):
        source = UprightPortableTest().fixture(root)
        upright_build(source, root / 'upright', hardlink=True)
        return root / 'upright'

    def test_portable_hashes_and_raw_source_unchanged(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = self.fixture(root)
            before = (source / 'manifest.json').read_bytes()
            manifest, report = build(source, root / 'merged', hardlink=True)
            self.assertEqual(manifest['training_label_policy'], LABEL_POLICY)
            self.assertEqual(manifest['split_counts'], json.loads(before)['split_counts'])
            self.assertEqual((source / 'manifest.json').read_bytes(), before)
            self.assertEqual((source / 'data/s0/audit.json').stat().st_ino,
                             (root / 'merged/data/s0/audit.json').stat().st_ino)
            source.rename(root / 'hidden_source')
            validate(root / 'merged', verify_hashes=True)
            with self.assertRaises(FileExistsError):
                build(root / 'hidden_source', root / 'merged')
            with self.assertRaisesRegex(ValueError, 'already has'):
                build(root / 'merged', root / 'duplicate')

    def test_config_and_fresh_resume_contract(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = self.fixture(root)
            build(source, root / 'merged', hardlink=True)
            config = Path(__file__).resolve().parents[1] / 'configs/bevfusion_base5_fine_512_depth25_pedestrian_merged.py'
            args = argparse.Namespace(data=root / 'merged', profile='512x1056', batch=1,
                accumulate=1, epochs=2, workers=0, lr=1e-4, config=config, warmup_updates=1,
                seed=1, work_dir=root / 'run', disable_val=False, wandb=False,
                init_checkpoint=None, resume=None, allow_random_init=True, run=False)
            cfg, manifest = prepare(args)
            self.assertEqual(cfg.model.bbox_head.num_classes, 3)
            self.assertEqual(tuple(cfg.val_dataloader.dataset.metainfo.classes), ('vehicle', 'pedestrian', 'obstacle'))
            self.assertEqual(cfg.val_dataloader.dataset.type, 'MoraiBase5PedestrianDataset')
            self.assertEqual(manifest['identity']['training_label_policy'], LABEL_POLICY)
            for pipeline in (cfg.train_dataloader.dataset.pipeline, cfg.val_dataloader.dataset.pipeline):
                for name_filter in (t for t in pipeline if t['type'] == 'ObjectNameFilter'):
                    self.assertEqual(tuple(name_filter['classes']), tuple(cfg.classes))
            checkpoint = args.work_dir / 'epoch_1.pth'
            checkpoint.write_bytes(b'placeholder')
            (args.work_dir / 'last_checkpoint').write_text(str(checkpoint))
            args.resume = 'auto'
            _, resumed = prepare(args)
            self.assertEqual(resumed['contract_sha256'], manifest['contract_sha256'])
            args.data = source
            with self.assertRaisesRegex(ValueError, 'label policy mismatch'):
                prepare(args)

    def test_three_class_cannot_silently_drop_animals(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = self.fixture(root)
            config = Path(__file__).resolve().parents[1] / 'configs/bevfusion_base5_fine_512_depth25_pedestrian_merged.py'
            args = argparse.Namespace(data=source, profile='512x1056', batch=1,
                accumulate=1, epochs=2, workers=0, lr=1e-4, config=config, warmup_updates=1,
                seed=1, work_dir=root / 'run', disable_val=False, wandb=False,
                init_checkpoint=None, resume=None, allow_random_init=True, run=False)
            with self.assertRaisesRegex(ValueError, 'label policy mismatch'):
                prepare(args)
            build(source, root / 'merged', hardlink=True)
            args.data = root / 'merged'
            args.config = config.parent / 'bevfusion_base5_fine_512_depth25_training_compiled_candidate.py'
            with self.assertRaisesRegex(ValueError, 'label policy mismatch'):
                prepare(args)


if __name__ == '__main__':
    unittest.main()
