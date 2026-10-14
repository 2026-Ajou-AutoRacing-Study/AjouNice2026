#!/usr/bin/env python3
import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_base5_offline_support import audit, point_bin, range_bin, sector, support_summary


def fixture():
    row = dict(token='t0', component_id='1:Obstacle:0', split='train', asset='Box',
               cls='obstacle', points=3, native_points=3, size_m=[1., 2., 3.],
               center_lidar_m=[25., 0., 0.], distance_m=25., direction='front',
               family='obstacle', mission=False, primary=True,
               original_semantic='obstacle', camera_gate=None, camera_rich_proxy=None,
               any_extra_crop_loss=None, best_visible_mask_pixels=None)
    inst = dict(component_id=row['component_id'], asset='Box', parent_uid=1,
                num_lidar_pts=3, bbox_3d=[25., 0., 0., 1., 2., 3., math.pi / 4])
    infos = dict(train=dict(data_list=[dict(token='t0', instances=[inst]),
                                       dict(token='empty', instances=[])]),
                 val=dict(data_list=[]))
    return row, infos


class SupportAuditTest(unittest.TestCase):
    def test_half_open_range(self):
        self.assertEqual(range_bin(19.9999), '10_20m')
        self.assertEqual(range_bin(20), '20_30m')
        self.assertEqual(range_bin(100.1), '100plus_corner')
        for x in (-1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                range_bin(x)

    def test_point_bins(self):
        self.assertEqual([point_bin(x) for x in (0, 1, 4, 5, 19, 20)],
                         ['0', '1_4', '1_4', '5_19', '5_19', '20plus'])
        with self.assertRaises(ValueError):
            point_bin(-1)

    def test_sector_wrap(self):
        for angle in (0., .4, -1.4, math.pi):
            self.assertEqual(sector(angle), sector(angle + 4 * math.pi))
        self.assertEqual(sector(math.pi), sector(-math.pi))

    def test_unknown_camera_is_not_failed(self):
        row, infos = fixture()
        report = audit([row], infos)
        self.assertEqual(report['groups']['train/obstacle']['camera_unknown'], 1)
        self.assertEqual(report['groups']['train/obstacle']['weak_support_known'], 0)
        self.assertEqual(report['groups']['train/obstacle']['rich_support_proxy'], 0)

    def test_empty_frames_retained(self):
        row, infos = fixture()
        report = audit([row], infos)
        self.assertEqual(report['frames']['train']['frames'], 2)
        self.assertEqual(report['frames']['train']['visible_parent_count_histogram'], {1: 1, 0: 1})

    def test_components_vs_parents(self):
        row, infos = fixture()
        other = copy.deepcopy(row)
        other['component_id'] = '1:Pedestrian:1'
        inst = copy.deepcopy(infos['train']['data_list'][0]['instances'][0])
        inst['component_id'] = other['component_id']
        infos['train']['data_list'][0]['instances'].append(inst)
        report = audit([row, other], infos)
        self.assertEqual(report['groups']['all']['gt_observations'], 2)
        self.assertEqual(report['groups']['all']['parent_observations'], 1)

    def test_reject_duplicate_and_unmatched(self):
        row, infos = fixture()
        with self.assertRaises(ValueError):
            audit([row, row], infos)
        with self.assertRaises(ValueError):
            audit([], infos)
        bad = dict(row, split='val')
        with self.assertRaises(ValueError):
            audit([bad], infos)

    def test_reject_asset_points_or_center_mismatch(self):
        row, infos = fixture()
        for field, value in [('asset', 'Other'), ('points', 5), ('center_lidar_m', [2., 0., 0.])]:
            bad = dict(row, **{field: value})
            with self.assertRaises(ValueError):
                audit([bad], infos)

    def test_split_overlap_rejected(self):
        row, infos = fixture()
        infos['val'] = copy.deepcopy(infos['train'])
        with self.assertRaises(ValueError):
            audit([row], infos)

    def test_animal_mapping_without_raw_label_mutation(self):
        row, infos = fixture()
        inst = infos['train']['data_list'][0]['instances'][0]
        inst['bbox_label_3d'] = 3
        infos['train']['metainfo'] = dict(classes=['vehicle', 'pedestrian', 'obstacle', 'animal'])
        row['cls'] = 'pedestrian'
        report = audit([row], infos)
        self.assertEqual(report['groups']['train/pedestrian']['gt_observations'], 1)
        self.assertEqual(inst['bbox_label_3d'], 3)
        with self.assertRaises(ValueError):
            audit([dict(row, cls='obstacle')], infos)

    def test_empty_summary(self):
        self.assertEqual(support_summary([]), {'gt_observations': 0})


if __name__ == '__main__':
    unittest.main()
