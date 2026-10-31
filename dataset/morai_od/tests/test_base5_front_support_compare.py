"""CPU-only comparison semantics; no ROS, simulator, GPU or label mutation."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import compare_base5_front_support as report


def fixture():
    spec = dict(seed=42, actors=2, primary_identity=11,
                secondary_obstacle_support=dict(new_actor_ordinals=[1]))
    actors = [dict(unique_id=8500 + i, data_id=11 + i, name='asset' + str(i),
                   source_list='objectList', motion='static', position=dict(x=i, y=0, z=28))
              for i in range(2)]
    before = dict(spec=spec, runtime_actors=actors, split='train', ego=dict(x=0, y=0, yaw=0))
    after = copy.deepcopy(before)
    after['spec'][report.PLAN_KEY] = dict(ordinal=1, data_id=12, selected=True,
                                         distance_bounds_m=[6, 10])
    after['obstacle_front_support_pair'] = dict(source_id='slot', source_scenario_sha256='sha',
                                               source_metadata_sha256='metadata-sha')
    return before, after


class FrontSupportComparisonTest(unittest.TestCase):
    def contract(self, before, after):
        return report.preserve_contract('slot', dict(scenario_sha256='sha'),
                                        dict(scenario_sha256='new'), before, after)

    def test_poses_can_change_without_claiming_pose_control(self):
        b, a = fixture()
        a['ego']['x'] = 4
        a['ego']['yaw'] = 359
        a['runtime_actors'][0]['position']['x'] = 3
        result = self.contract(b, a)
        self.assertEqual(result['ego_xy_change_m'], 4)
        self.assertEqual(result['ego_yaw_change_deg'], -1)
        self.assertFalse(result['ego_pose_equal'])
        self.assertFalse(result['original_actor_positions_equal'])
        self.assertEqual(result['added_uid'], 8501)

    def test_original_spec_change_rejected(self):
        b, a = fixture()
        a['spec']['seed'] = 43
        with self.assertRaisesRegex(ValueError, 'frozen spec'):
            self.contract(b, a)

    def test_changed_asset_or_motion_rejected(self):
        for key, value in [('data_id', 77), ('motion', 'moving')]:
            b, a = fixture()
            a['runtime_actors'][1][key] = value
            with self.assertRaisesRegex(ValueError, 'identities/count/motion'):
                self.contract(b, a)

    def test_changed_source_rejected(self):
        b, a = fixture()
        a['obstacle_front_support_pair']['source_scenario_sha256'] = 'wrong'
        with self.assertRaisesRegex(ValueError, 'checksum'):
            self.contract(b, a)

    def test_split_change_rejected(self):
        b, a = fixture()
        a['split'] = 'val'
        with self.assertRaisesRegex(ValueError, 'Split'):
            self.contract(b, a)

    def test_support_counts_are_not_acceptance(self):
        rows = [dict(parent_uid=8501, lidar_points=0, kept512=True, camera_gate512=True,
                     actual_ego_sector='front', range_ego_xy_m=8),
                dict(parent_uid=8501, lidar_points=20, kept512=True, camera_gate512=False,
                     actual_ego_sector='rear', range_ego_xy_m=9)]
        result = report.support_summary(rows)
        self.assertEqual(result['components'], 2)
        self.assertEqual(result['actors'], 1)
        self.assertEqual(result['lidar_zero'], 1)
        self.assertEqual(result['lidar_20_plus'], 1)
        self.assertEqual(result['front_under10m'], 1)

    def test_ego_frame_direction_is_not_lidar_frame_direction(self):
        capture = dict(components=[dict(component_id='8501:Obstacle:0', asset='box',
            **{'class': 'obstacle'}, points=1, camera_gate512=False, kept512=True,
            cameras=[], range=.3)])
        native = dict(components=[dict(component_id='8501:Obstacle:0', parent_uid=8501,
                                      center_ego_m=[.8, 0, 0])])
        row = report.component_rows(capture, native)[0]
        self.assertEqual(row['actual_ego_sector'], 'front')
        self.assertEqual(row['range_ego_xy_m'], .8)
        self.assertEqual(row['range_lidar_xy_m'], .3)

    def test_unknown_ego_center_rejected(self):
        capture = dict(components=[dict(component_id='x')])
        with self.assertRaisesRegex(ValueError, 'Ego-frame'):
            report.component_rows(capture, dict(components=[dict(component_id='x')]))

    def test_missing_shutdown_is_unknown_not_certified(self):
        with tempfile.TemporaryDirectory() as root:
            result = report.session_safety(Path(root) / 'ledger.jsonl')
        self.assertFalse(result['shutdown_record_exists'])
        self.assertIsNone(result['original_shutdown_park_verified'])

    def test_recorded_park_is_separate_from_recovery(self):
        with tempfile.TemporaryDirectory() as root:
            report.atomic_json(Path(root) / 'shutdown_safety.json', dict(park_verified=True))
            result = report.session_safety(Path(root) / 'ledger.jsonl')
        self.assertTrue(result['original_shutdown_park_verified'])

    def test_duplicate_slot_in_side_is_rejected(self):
        with patch.object(report, 'completed_lines', return_value=[dict(id='same'), dict(id='same')]), \
             patch.object(report, 'pilot_report', return_value=dict(errors=[], captures=[])):
            with self.assertRaisesRegex(ValueError, 'Duplicate slot'):
                report.load_side([Path('unused')])

    def test_actor_exposures_are_not_deduplicated_across_scenes(self):
        b, a = fixture()
        component = dict(parent_uid=8501, component_id='8501:Obstacle:0', lidar_points=2,
                         kept512=True, camera_gate512=True, actual_ego_sector='front',
                         range_ego_xy_m=8)
        sides = []
        for metadata, digest in [(b, 'sha'), (a, 'new')]:
            side = {}
            for slot in ('one', 'two'):
                m = copy.deepcopy(metadata)
                if 'obstacle_front_support_pair' in m:
                    m['obstacle_front_support_pair']['source_id'] = slot
                side[slot] = dict(metadata=m, row=dict(scenario_sha256=digest,
                    status='accepted_candidate'), capture=dict(review_reasons=[]),
                    components=[copy.deepcopy(component)], sidecar_sha256={})
            sides.append(side)
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'ledger'
            report.atomic_json(path, dict(test=True))
            with patch.object(report, 'load_side', side_effect=[(sides[0], []), (sides[1], [])]):
                result = report.compare([path], [path], Path(root) / 'new.json')
        self.assertEqual(result['added_actor_support']['before']['actors'], 2)
        self.assertEqual(result['paired_scene_count'], 2)
        self.assertTrue(result['diagnostic_only'])
        self.assertFalse(result['fills_training_quota'])

    def test_partial_pair_set_is_not_silently_selected(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / 'ledger'
            report.atomic_json(path, dict(test=True))
            with patch.object(report, 'load_side', side_effect=[({'one': {}}, []), ({}, [])]):
                with self.assertRaisesRegex(ValueError, 'slots differ'):
                    report.compare([path], [path], Path(root) / 'new.json')


if __name__ == '__main__':
    unittest.main()
