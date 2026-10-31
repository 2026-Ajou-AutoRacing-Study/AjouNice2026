"""CPU-only contract tests for the additional Base5 validation dashboard.

These scalars are diagnostics, not a second inference pass or a replacement
for the established center AP / best-checkpoint contract.
"""
import copy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import base5_val_logging as logging
from base5_distance_diagnostics import diagnostic_metrics
from perception.detection3d.common.evaluation.morai_detection import (
    DEFAULT_CENTER_THRESHOLDS, _evaluate_center, _filter_range,
)


def sample(xy=(), labels=None, prediction_xy=None, prediction_labels=None,
           scores=None):
    """Construct the canonical post-extraction sample without ROS or a model."""
    xy = list(xy)
    labels = [0] * len(xy) if labels is None else list(labels)
    prediction_xy = xy if prediction_xy is None else list(prediction_xy)
    prediction_labels = (labels if prediction_labels is None and prediction_xy == xy
                         else [0] * len(prediction_xy) if prediction_labels is None
                         else list(prediction_labels))
    boxes = lambda values: np.asarray(
        [[x, y, 0., 2., 2., 2., 0.] for x, y in values], dtype=float).reshape(-1, 7)
    return dict(gt_boxes=boxes(xy), gt_labels=np.asarray(labels, dtype=np.int64),
                pred_boxes=boxes(prediction_xy),
                pred_labels=np.asarray(prediction_labels, dtype=np.int64),
                pred_scores=np.asarray([.9] * len(prediction_xy) if scores is None
                                       else scores, dtype=float))


def all_classes(x=35., **kwargs):
    return sample([(x, 0), (x, 3), (x, 6)], [0, 1, 2], **kwargs)


class RangeContractTests(unittest.TestCase):
    def test_constants_match_the_approved_panel_names_and_thresholds(self):
        self.assertEqual(tuple(logging.CLASSES), ('vehicle', 'pedestrian', 'obstacle'))
        self.assertEqual(tuple(logging.ENVIRONMENTS),
                         ('sunny11', 'sunny13', 'sunny15',
                          'foggy11', 'foggy13', 'foggy15'))
        self.assertEqual(tuple(logging.TEN_METER_BINS),
                         tuple((float(x), float(x + 10)) for x in range(0, 100, 10)))
        self.assertEqual(logging.FIXED_THRESHOLDS,
                         {'vehicle': .4, 'pedestrian': .1, 'obstacle': .1})

    def test_half_open_ten_meter_boundaries_and_radial_distance(self):
        points = [(float(x), 0.) for x in range(0, 101, 10)]
        points += [(9.999, 0.), (6., 8.), (99.999, 0.)]
        result = logging.ten_meter_metrics([sample(points)])
        expected = {0: 2, 10: 2, 90: 2}
        for lower in range(0, 100, 10):
            prefix = f'range/{lower}_{lower + 10}m'
            self.assertEqual(result[prefix + '/vehicle_GT'], expected.get(lower, 1))
        self.assertNotIn('range/100_110m/vehicle_GT', result)

    def test_gt_zero_new_ranges_have_no_ap_but_positive_gt_miss_has_zero_ap(self):
        result = logging.ten_meter_metrics([sample([(35., 0.)], prediction_xy=[])])
        self.assertEqual(result['range/30_40m/vehicle_mean_AP'], 0.)
        self.assertEqual(result['range/30_40m/vehicle_available'], 1.)
        self.assertEqual(result['range/30_40m/pedestrian_GT'], 0.)
        self.assertEqual(result['range/30_40m/pedestrian_available'], 0.)
        self.assertNotIn('range/30_40m/pedestrian_mean_AP', result)
        self.assertNotIn('range/30_40m/mAP', result)
        self.assertEqual(result['range/30_40m/mAP_available'], 0.)
        self.assertEqual(result['range/30_40m/mAP_GT_classes'], 1.)
        self.assertNotIn('range/40_50m/vehicle_mean_AP', result)

    def test_perfect_predictions_have_equal_class_mean_ap(self):
        result = logging.ten_meter_metrics([all_classes()])
        for name in logging.CLASSES:
            self.assertAlmostEqual(result[f'range/30_40m/{name}_mean_AP'], 1.)
        self.assertAlmostEqual(result['range/30_40m/mAP'], 1.)
        self.assertEqual(result['range/30_40m/mAP_GT_classes'], 3.)

    def test_range_ap_reuses_all_four_original_xy_thresholds(self):
        rows = [sample([(35., 0.)], prediction_xy=[(35.75, 0.)]),
                sample([(35., 2.)], [1], [(36.5, 2.)], [1]),
                sample([(35., 4.)], [2], [(38., 4.)], [2])]
        ranged = _filter_range(rows, (30., 40.))
        original = _evaluate_center(ranged, list(logging.CLASSES),
                                   DEFAULT_CENTER_THRESHOLDS, 2., .1, .1,
                                   'range/30_40m', include_tp=False,
                                   include_operating_points=False)
        result = logging.ten_meter_metrics(rows)
        for name in logging.CLASSES:
            key = f'range/30_40m/{name}_mean_AP'
            self.assertEqual(result[key], original[key])
        expected = [original[f'range/30_40m/{name}_mean_AP']
                    for name in logging.CLASSES]
        self.assertEqual(result['range/30_40m/mAP'], float(np.mean(expected)))

    def test_fixed_thresholds_do_not_change_ap_predictions(self):
        rows = [sample([(35., 0.)], scores=[.001])]
        self.assertAlmostEqual(logging.ten_meter_metrics(rows)[
            'range/30_40m/vehicle_mean_AP'], 1.)
        fixed = logging.fixed_operating_metrics(rows)
        self.assertEqual(fixed['fixed/vehicle/recall'], 0.)

    def test_small_sample_boundary(self):
        self.assertEqual(logging.ten_meter_metrics([sample([(35., 0.)] * 19)])[
            'range/30_40m/vehicle_small_sample'], 1.)
        self.assertEqual(logging.ten_meter_metrics([sample([(35., 0.)] * 20)])[
            'range/30_40m/vehicle_small_sample'], 0.)

    def test_overlapping_legacy_supported_scalars_have_identical_values(self):
        rows = [all_classes(x=5.), all_classes(x=15.), all_classes(x=25.),
                all_classes(x=60.), all_classes(x=85.)]
        legacy = diagnostic_metrics(rows)
        extension = logging.ten_meter_metrics(rows)
        overlap = legacy.keys() & extension.keys()
        self.assertTrue(overlap)
        for key in overlap:
            self.assertEqual(extension[key], legacy[key], key)

    def test_existing_three_near_ranges_reuse_ap_without_recomputation(self):
        rows = [all_classes(x=lower + 5.) for lower in range(0, 100, 10)]
        legacy = {}
        for lower in (0, 10, 20):
            prefix = f'range/{lower}_{lower + 10}m'
            legacy.update(_evaluate_center(
                _filter_range(rows, (lower, lower + 10)), list(logging.CLASSES),
                DEFAULT_CENTER_THRESHOLDS, 2., .1, .1, prefix,
                include_tp=False, include_operating_points=False))
        expected = logging.ten_meter_metrics(rows)
        with patch.object(logging, '_class_ap', wraps=logging._class_ap) as calculate:
            actual = logging.ten_meter_metrics(rows, legacy_metrics=legacy)
        self.assertEqual(calculate.call_count, 7)
        self.assertEqual(actual, expected)
        broken = dict(legacy)
        broken['range/0_10m/vehicle_GT'] += 1.
        with self.assertRaises(AssertionError):
            logging.ten_meter_metrics(rows, legacy_metrics=broken)

    def test_samples_are_not_modified(self):
        rows = [all_classes()]
        before = copy.deepcopy(rows)
        logging.extension_metrics(rows, ['sunny13'])
        for actual, expected in zip(rows, before):
            self.assertEqual(set(actual), set(expected))
            for key in actual:
                np.testing.assert_array_equal(actual[key], expected[key])


class EnvironmentContractTests(unittest.TestCase):
    def test_six_base_environments_and_114_supplement_frames_are_separate(self):
        rows = [all_classes() for _ in logging.ENVIRONMENTS]
        cohorts = list(logging.ENVIRONMENTS)
        for name in ('supp_sunny11', 'supp_sunny13', 'supp_sunny15'):
            rows.extend(sample([(35., 0.)], [2]) for _ in range(38))
            cohorts.extend([name] * 38)
        result = logging.environment_metrics(rows, cohorts)
        for name in logging.ENVIRONMENTS:
            prefix = f'environment/{name}'
            self.assertEqual(result[prefix + '/frames'], 1.)
            self.assertAlmostEqual(result[prefix + '/mAP'], 1.)
            for class_name in logging.CLASSES:
                self.assertEqual(result[f'{prefix}/{class_name}_GT'], 1.)
        prefix = 'supplement/sunny_obstacle'
        self.assertEqual(result[prefix + '/frames'], 114.)
        self.assertEqual(result[prefix + '/obstacle_GT'], 114.)
        self.assertAlmostEqual(result[prefix + '/obstacle_mean_AP'], 1.)
        self.assertNotIn(prefix + '/mAP', result)
        self.assertFalse(any(key.startswith('environment/supp_') for key in result))

    def test_unknown_missing_and_misaligned_cohorts_fail_fast(self):
        rows = [all_classes()]
        for cohorts in ([], ['unknown'], [None], ['sunny11', 'sunny13']):
            with self.subTest(cohorts=cohorts), self.assertRaises((ValueError, TypeError)):
                logging.environment_metrics(rows, cohorts)

    def test_gt_zero_environment_has_no_class_or_aggregate_ap(self):
        result = logging.environment_metrics([sample()], ['foggy13'])
        prefix = 'environment/foggy13'
        self.assertEqual(result[prefix + '/frames'], 1.)
        self.assertNotIn(prefix + '/mAP', result)
        self.assertNotIn(prefix + '/mAP_available', result)
        for name in logging.CLASSES:
            self.assertEqual(result[f'{prefix}/{name}_GT'], 0.)
            self.assertNotIn(f'{prefix}/{name}_mean_AP', result)

    def test_environment_map_is_macro_not_weighted_by_gt_frequency(self):
        rows = [sample([(35., 0.)] * 4, [0] * 4,
                       [(35., 0.)] * 4, [0] * 4),
                sample([(35., 2.)], [1], [], []),
                sample([(35., 4.)], [2], [], [])]
        result = logging.environment_metrics(rows, ['sunny11'] * len(rows))
        prefix = 'environment/sunny11'
        self.assertAlmostEqual(result[prefix + '/mAP'], 1. / 3.)


class FixedOperatingPointTests(unittest.TestCase):
    def test_each_score_threshold_is_inclusive(self):
        rows = []
        for index, name in enumerate(logging.CLASSES):
            threshold = logging.FIXED_THRESHOLDS[name]
            rows.append(sample([(5., 0.), (5., 3.)], [index, index],
                               [(5., 0.), (5., 3.)], [index, index],
                               [threshold, np.nextafter(threshold, -np.inf)]))
        result = logging.fixed_operating_metrics(rows)
        for name in logging.CLASSES:
            prefix = 'fixed/' + name
            self.assertEqual(result[prefix + '/score_threshold'],
                             logging.FIXED_THRESHOLDS[name])
            self.assertEqual(result[prefix + '/precision'], 1.)
            self.assertEqual(result[prefix + '/recall'], .5)
            self.assertAlmostEqual(result[prefix + '/F1'], 2. / 3.)

    def test_exactly_one_meter_does_not_match_but_just_inside_does(self):
        boundary = sample([(0., 0.)], prediction_xy=[(1., 0.)])
        inside = sample([(0., 0.)], prediction_xy=[(np.nextafter(1., 0.), 0.)])
        result = logging.fixed_operating_metrics([boundary, inside])
        self.assertEqual(result['fixed/vehicle/precision'], .5)
        self.assertEqual(result['fixed/vehicle/recall'], .5)
        self.assertEqual(result['fixed/vehicle/fp_per_frame'], .5)

    def test_duplicate_predictions_are_false_positives(self):
        rows = [sample([(5., 0.)], prediction_xy=[(5., 0.), (5.1, 0.)],
                       scores=[.8, .9])]
        result = logging.fixed_operating_metrics(rows)
        self.assertEqual(result['fixed/vehicle/precision'], .5)
        self.assertEqual(result['fixed/vehicle/recall'], 1.)
        self.assertEqual(result['fixed/vehicle/fp_per_frame'], 1.)

    def test_matching_is_score_ranked_greedy_not_globally_optimized(self):
        # The high-score middle candidate takes the nearest GT at x=0; the
        # lower-score left candidate cannot then steal it or reuse that GT.
        rows = [sample([(0., 0.), (1.8, 0.)], prediction_xy=[(-.8, 0.), (.8, 0.)],
                       scores=[.8, .9])]
        result = logging.fixed_operating_metrics(rows)
        self.assertEqual(result['fixed/vehicle/precision'], .5)
        self.assertEqual(result['fixed/vehicle/recall'], .5)
        self.assertEqual(result['fixed/vehicle/fp_per_frame'], 1.)

    def test_predictions_cannot_match_another_class(self):
        rows = [sample([(5., 0.), (5., 3.)], [0, 1],
                       [(5., 0.), (5., 3.)], [1, 0])]
        result = logging.fixed_operating_metrics(rows)
        for name in ('vehicle', 'pedestrian'):
            self.assertEqual(result[f'fixed/{name}/precision'], 0.)
            self.assertEqual(result[f'fixed/{name}/recall'], 0.)
            self.assertEqual(result[f'fixed/{name}/fp_per_frame'], 1.)

    def test_background_frames_count_in_fp_per_frame_denominator(self):
        rows = [sample([(5., 0.)]), sample(),
                sample(prediction_xy=[(5., 0.), (7., 0.)])]
        result = logging.fixed_operating_metrics(rows)
        self.assertAlmostEqual(result['fixed/vehicle/fp_per_frame'], 2. / 3.)
        self.assertAlmostEqual(result['fixed/vehicle/precision'], 1. / 3.)
        self.assertEqual(result['fixed/vehicle/recall'], 1.)

    def test_a_supported_class_with_no_predictions_is_not_unavailable(self):
        result = logging.fixed_operating_metrics([
            sample([(5., 0.)], prediction_xy=[])])
        self.assertEqual(result['fixed/vehicle/recall'], 0.)
        self.assertEqual(result['fixed/vehicle/F1'], 0.)
        self.assertEqual(result['fixed/vehicle/fp_per_frame'], 0.)

    def test_zero_gt_and_zero_frames_are_finite_documented_zero_conventions(self):
        for rows in ([], [sample()], [sample(prediction_xy=[(5., 0.)])]):
            result = logging.fixed_operating_metrics(rows)
            self.assertTrue(all(np.isfinite(value) for value in result.values()))
            self.assertEqual(result['fixed/vehicle/recall'], 0.)
            self.assertEqual(result['fixed/vehicle/F1'], 0.)
        self.assertEqual(logging.fixed_operating_metrics([
            sample(prediction_xy=[(5., 0.)])])['fixed/vehicle/fp_per_frame'], 1.)


def complete_configuration():
    counts = {name: 750 for name in logging.ENVIRONMENTS}
    counts.update({name: 38 for name in logging.SUPPLEMENTS})
    mapping = {f'{name}_token_{index:04d}': name for name, count in counts.items()
               for index in range(count)}
    return dict(token_to_cohort=mapping, expected_cohort_frames=counts)


class ProvenanceAndLegacyTests(unittest.TestCase):
    def test_full_val_mapping_has_exact_4614_unique_frames(self):
        config = complete_configuration()
        self.assertEqual(len(config['token_to_cohort']), 4614)
        self.assertEqual(logging._configuration(config), config)
        self.assertIsNone(logging._configuration(None))

    def test_mapping_counts_environment_sizes_and_unknown_names_fail_fast(self):
        good = complete_configuration()
        cases = []
        wrong = copy.deepcopy(good)
        wrong['token_to_cohort'].pop(next(iter(wrong['token_to_cohort'])))
        cases.append(wrong)
        wrong = copy.deepcopy(good)
        wrong['expected_cohort_frames']['sunny11'] = 749
        wrong['token_to_cohort'].pop('sunny11_token_0749')
        cases.append(wrong)
        wrong = copy.deepcopy(good)
        wrong['expected_cohort_frames']['sunny11'] = 750.
        cases.append(wrong)
        wrong = copy.deepcopy(good)
        wrong['token_to_cohort']['unlisted'] = 'foggy13_completion'
        cases.append(wrong)
        wrong = copy.deepcopy(good)
        wrong['extra'] = 'not part of the contract'
        cases.append(wrong)
        for config in cases:
            with self.subTest(config_keys=list(config)), self.assertRaises(ValueError):
                logging._configuration(config)

    def test_real_portable_lidar_path_resolves_parent_parent_token(self):
        mapping = {'sunny11_token_0001': 'sunny11'}
        for path in ('data/sunny11_token_0001/LIDAR_5/123_Intensity.bin',
                     '/workspace/a/dataset/data/sunny11_token_0001/LIDAR_5/123_Intensity.bin'):
            row = SimpleNamespace(metainfo=dict(sample_idx='sunny11_token_0001',
                                                lidar_path=path))
            self.assertEqual(logging._sample_token(row, mapping), 'sunny11_token_0001')

    def test_missing_unknown_conflicting_or_ambiguous_identity_fails_fast(self):
        mapping = {'a': 'sunny11', 'b': 'foggy13'}
        for meta in ({}, {'sample_idx': 'unlisted'},
                     {'sample_idx': 'a', 'lidar_path': 'data/b/LIDAR_5/x.bin'},
                     {'sample_idx': 'a', 'lidar_path': 'data/a/b/LIDAR_5/x.bin'}):
            with self.subTest(meta=meta), self.assertRaises(ValueError):
                logging._sample_token(SimpleNamespace(metainfo=meta), mapping)

    def test_numeric_or_missing_sample_index_uses_unique_explicit_path_mapping(self):
        mapping = {'a': 'sunny11', 'b': 'foggy13'}
        for meta in ({'sample_idx': 123, 'lidar_path': 'data/a/LIDAR_5/x.bin'},
                     {'lidar_path': 'data/a/LIDAR_5/x.bin'}):
            self.assertEqual(logging._sample_token(SimpleNamespace(metainfo=meta), mapping), 'a')

    def test_legacy_keys_values_and_gt_zero_scalars_are_preserved(self):
        original = {'center/mAP': .8, 'center/vehicle_mean_AP': .9,
                    'range/0_10m/pedestrian_mean_AP': 0.,
                    'range/0_10m/mAP': .2, 'range/0_10m/pedestrian_GT': 0.}
        extra = {'range/0_10m/pedestrian_GT': 0., 'fixed/vehicle/recall': .7}
        before = dict(original)
        merged = logging.merge_preserving_legacy(original, extra)
        self.assertEqual(original, before)
        for key, value in original.items():
            self.assertEqual(merged[key], value)
        self.assertEqual(merged['fixed/vehicle/recall'], .7)
        with self.assertRaises(AssertionError):
            logging.merge_preserving_legacy(original, {'center/mAP': .7})

    def test_legacy_gt_zero_aggregate_cannot_be_replaced_by_subset_mean(self):
        rows = [sample([(5., 0.)])]
        new = logging.ten_meter_metrics(rows)
        self.assertNotIn('range/0_10m/mAP', new)
        merged = logging.merge_preserving_legacy({'range/0_10m/mAP': 1. / 3.}, new)
        self.assertEqual(merged['range/0_10m/mAP'], 1. / 3.)


class DashboardSchemaTests(unittest.TestCase):
    def test_all_preconfigured_public_panel_keys_are_emitted_with_same_case(self):
        rows = []
        cohorts = []
        for environment in logging.ENVIRONMENTS:
            for lower in range(0, 100, 10):
                # A separate frame per class avoids the radial offset moving
                # an object at x=95 out of its last ten-metre bin.
                for class_index in range(3):
                    rows.append(sample([(lower + 5., 0.)], [class_index]))
                    cohorts.append(environment)
        rows.append(sample([(35., 0.)], [2]))
        cohorts.append('supp_sunny11')
        values = logging.extension_metrics(rows, cohorts)
        for lower in range(0, 100, 10):
            prefix = f'range/{lower}_{lower + 10}m/'
            for metric in ('mAP', *(name + '_mean_AP' for name in logging.CLASSES)):
                self.assertIn(prefix + metric, values)
        for environment in logging.ENVIRONMENTS:
            prefix = f'environment/{environment}/'
            for metric in ('mAP', 'frames',
                           *(name + '_mean_AP' for name in logging.CLASSES),
                           *(name + '_GT' for name in logging.CLASSES)):
                self.assertIn(prefix + metric, values)
        for name in logging.CLASSES:
            for metric in ('precision', 'recall', 'F1', 'fp_per_frame', 'score_threshold'):
                self.assertIn(f'fixed/{name}/{metric}', values)
        self.assertIn('supplement/sunny_obstacle/obstacle_mean_AP', values)
        self.assertNotIn('fixed/vehicle/f1', values)
        self.assertFalse(any(key.startswith('val/') for key in values))

    def test_existing_preview_schema_matches_real_metric_names_if_available(self):
        schema = Path(__file__).resolve().parents[2] / (
            'artifacts/evaluation/base5_val_panel_preview_261021_v1/METRIC_SCHEMA.json')
        if not schema.exists():
            self.skipTest('The optional local dummy-run receipt is not in Git')
        keys = set(json.loads(schema.read_text())['metric_keys'])
        expected = set()
        for environment in logging.ENVIRONMENTS:
            expected.update(f'val/environment/{environment}/{metric}' for metric in
                            ('mAP', 'frames', *(name + '_mean_AP' for name in logging.CLASSES),
                             *(name + '_GT' for name in logging.CLASSES)))
        for lower in range(0, 100, 10):
            expected.update(f'val/range/{lower}_{lower + 10}m/{metric}' for metric in
                            ('mAP', *(name + '_mean_AP' for name in logging.CLASSES)))
        for name in logging.CLASSES:
            expected.update(f'val/fixed/{name}/{metric}' for metric in
                            ('precision', 'recall', 'F1', 'fp_per_frame', 'score_threshold'))
        expected.add('val/supplement/sunny_obstacle/obstacle_mean_AP')
        self.assertTrue(expected <= keys, sorted(expected - keys))


@unittest.skipUnless(importlib.util.find_spec('mmengine') is not None,
                     'The real evaluator integration runs in the prepared training environment')
class RuntimeIntegrationTests(unittest.TestCase):
    """Real MMEngine metric/backend wiring, with W&B network calls mocked."""

    @classmethod
    def setUpClass(cls):
        from base5_distance_diagnostics import install as install_distance
        install_distance()
        logging.install()

    def test_wandb_public_keys_are_visible_and_use_epoch_without_double_prefix(self):
        from mmengine.visualization import WandbVisBackend
        from perception.detection3d.bevfusion.plugin.morai_wandb import MoraiWandbVisBackend
        backend = object.__new__(MoraiWandbVisBackend)
        backend._wandb = Mock()
        with patch.object(WandbVisBackend, '_init_env'):
            backend._init_env()
        for namespace in ('environment', 'range', 'supplement', 'fixed'):
            backend._wandb.define_metric.assert_any_call(
                f'val/{namespace}/*', step_metric='epoch', hidden=True)
        keys = []
        for environment in logging.ENVIRONMENTS:
            keys.extend(f'val/environment/{environment}/{metric}' for metric in
                        ('mAP', *(name + '_mean_AP' for name in logging.CLASSES)))
        for lower in range(0, 100, 10):
            keys.extend(f'val/range/{lower}_{lower + 10}m/{metric}' for metric in
                        ('mAP', *(name + '_mean_AP' for name in logging.CLASSES)))
        keys.append('val/supplement/sunny_obstacle/obstacle_mean_AP')
        for name in logging.CLASSES:
            keys.extend(f'val/fixed/{name}/{metric}' for metric in ('precision', 'recall', 'F1'))
            backend._wandb.define_metric.assert_any_call(
                f'val/fixed/{name}/fp_per_frame', step_metric='epoch', hidden=False,
                summary='min', overwrite=True)
        for key in keys:
            backend._wandb.define_metric.assert_any_call(
                key, step_metric='epoch', hidden=False, summary='max', overwrite=True)
        payload = backend._canonicalize(
            {'epoch': 7, 'val/environment/sunny11/mAP': .6,
             'val/fixed/vehicle/F1': .5}, step=6534)
        self.assertEqual(payload['epoch'], 7)
        self.assertEqual(payload['global_step'], 6534)
        self.assertEqual(payload['val/environment/sunny11/mAP'], .6)
        self.assertFalse(any(key.startswith('val/val/') for key in payload))

    def test_process_records_real_sample_identity_before_aggregation(self):
        import torch
        from perception.detection3d.bevfusion.plugin.morai_metric import (
            MoraiBase5MergedCenterMetric, _Boxes)
        config = complete_configuration()
        token = 'sunny11_token_0000'

        class DataSample(dict):
            @property
            def metainfo(self):
                return self['metainfo']

        row = sample([(5., 0.)])
        data_sample = DataSample(
            metainfo=dict(sample_idx=token,
                          lidar_path=f'data/{token}/LIDAR_5/native_Intensity.bin'),
            eval_ann_info=dict(gt_bboxes_3d=_Boxes(row['gt_boxes']),
                               gt_labels_3d=row['gt_labels'],
                               original_semantic_class=np.asarray(['Vehicle']),
                               original_asset=np.asarray(['test'])),
            pred_instances_3d=dict(bboxes_3d=_Boxes(row['pred_boxes']),
                                   scores_3d=torch.as_tensor(row['pred_scores']),
                                   labels_3d=torch.as_tensor(row['pred_labels'])))
        metric = MoraiBase5MergedCenterMetric(
            point_cloud_range=[-100, -30, -3, 100, 30, 2], base5_val_logging=config)
        metric.dataset_meta = dict(classes=logging.CLASSES)
        metric.process({}, [data_sample])
        self.assertEqual(len(metric.results), 1)
        self.assertEqual(metric.results[0]['_base5_val_token'], token)
        self.assertEqual(metric.results[0]['_base5_val_cohort'], 'sunny11')
        with self.assertRaises(ValueError):
            metric.compute_metrics(metric.results)

    def test_complete_cpu_evaluator_preserves_every_legacy_scalar(self):
        import torch
        from perception.detection3d.bevfusion.plugin.morai_metric import (
            MoraiBase5MergedCenterMetric, _Boxes)
        config = complete_configuration()
        rows = []
        for token, cohort in config['token_to_cohort'].items():
            index = int(token.rsplit('_', 1)[1])
            if index == 0:
                values = all_classes() if cohort in logging.ENVIRONMENTS else sample([(35., 0.)], [2])
            else:
                values = sample()
            labels = values['gt_labels']
            rows.append(dict(
                info=dict(gt_boxes=values['gt_boxes'],
                          gt_names=np.asarray(logging.CLASSES)[labels],
                          valid_flag=np.ones(len(labels), dtype=bool),
                          num_lidar_pts=np.ones(len(labels))),
                prediction=dict(boxes_3d=_Boxes(values['pred_boxes']),
                                scores_3d=torch.as_tensor(values['pred_scores']),
                                labels_3d=torch.as_tensor(values['pred_labels'])),
                original_semantic_class=np.asarray(['Vehicle', 'Pedestrian', 'Obstacle'])[labels],
                assets=np.asarray(['test'] * len(labels)),
                _base5_val_token=token, _base5_val_cohort=cohort))
        legacy = MoraiBase5MergedCenterMetric(point_cloud_range=[-100, -30, -3, 100, 30, 2])
        legacy.dataset_meta = dict(classes=logging.CLASSES)
        expected = legacy.compute_metrics(rows)
        metric = MoraiBase5MergedCenterMetric(
            point_cloud_range=[-100, -30, -3, 100, 30, 2], base5_val_logging=config)
        metric.dataset_meta = dict(classes=logging.CLASSES)
        actual = metric.compute_metrics(rows)
        for key, value in expected.items():
            self.assertEqual(actual[key], value, key)
        for cohort in logging.ENVIRONMENTS:
            self.assertEqual(actual[f'environment/{cohort}/frames'], 750.)
            self.assertAlmostEqual(actual[f'environment/{cohort}/mAP'], 1.)
        self.assertEqual(actual['supplement/sunny_obstacle/frames'], 114.)
        self.assertEqual(actual['supplement/sunny_obstacle/obstacle_GT'], 3.)
        self.assertEqual(actual['fixed/vehicle/recall'], 1.)
        self.assertEqual(actual['fixed/vehicle/fp_per_frame'], 0.)
        self.assertEqual(actual['range/30_40m/vehicle_GT'], 6.)
        self.assertNotIn('range/90_100m/vehicle_mean_AP', actual)

    def test_checkpoint_receipt_preserves_training_metadata_and_optimizer(self):
        from mmengine.registry import HOOKS
        hook = HOOKS.get('Base5ValLoggingReceiptHook')()
        checkpoint = dict(meta=dict(epoch=7), optimizer=dict(keep='unchanged'))
        hook.before_save_checkpoint(None, checkpoint)
        self.assertEqual(checkpoint['meta']['epoch'], 7)
        self.assertEqual(checkpoint['optimizer'], dict(keep='unchanged'))
        self.assertEqual(checkpoint['meta']['base5_val_logging']['version'], logging.VERSION)
        self.assertEqual(checkpoint['meta']['base5_val_logging']['schema_sha256'],
                         logging.SCHEMA_SHA256)


if __name__ == '__main__':
    unittest.main()
