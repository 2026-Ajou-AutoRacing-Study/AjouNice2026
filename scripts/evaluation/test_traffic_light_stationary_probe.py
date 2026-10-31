"""Pure stationary-probe regressions: never start ROS, MORAI, or child processes."""

import copy
import json
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import traffic_light_stationary_probe as probe


def example_scene():
    return {
        'egoVehicle': {
            'beforeControlMode': 1, 'currentControlMode': 1,
            'currentVelocity': 20, 'customValue': 30,
            'initPosition': {
                'gear': 4, 'pos': {'x': 1, 'y': 2, 'z': 3, '_x': '1'},
                'rot': {'roll': '0', 'pitch': '0', 'yaw': '90'},
                'vehicleSaveData': {
                    'velocity': 20,
                    'lastVelocity': {'x': 10, 'y': 5, 'z': 1},
                    'vehicle': {'velocity': 20, 'lastVelocity': {'x': 10, 'y': 5, 'z': 1}},
                    'inputData': [9, 8, 7, 6, 5, 4, 4, 2],
                },
            },
        },
        'vehicleList': [{'id': 1}], 'pedestrianList': [{'id': 2}],
        'obstacleList': [{'id': 3}], 'objectList': [{'id': 4}],
        'spawnPointList': [{'id': 5}], 'environment': {'weather': 'unchanged'},
    }


class SceneNormalizationTest(unittest.TestCase):
    def test_source_is_preserved_and_stationary_copy_clears_all_actors(self):
        source = example_scene()
        original = copy.deepcopy(source)
        result = probe.normalized_scene(source, {'x': 12.5, 'yaw': -30})
        self.assertEqual(source, original)
        ego = result['egoVehicle']
        self.assertEqual((ego['beforeControlMode'], ego['currentControlMode']), (2, 2))
        self.assertEqual((ego['currentVelocity'], ego['customValue']), (0, 0))
        initial = ego['initPosition']
        self.assertEqual(initial['gear'], 1)
        self.assertEqual(initial['pos']['x'], 12.5)
        self.assertEqual(initial['pos']['_x'], '12.5')
        self.assertEqual(initial['rot']['yaw'], '-30.0')
        saved = initial['vehicleSaveData']
        for container in (saved, saved['vehicle']):
            self.assertEqual(container['velocity'], 0)
            self.assertEqual(container['lastVelocity'], {'x': 0, 'y': 0, 'z': 0})
        self.assertEqual(saved['inputData'][6], 1)
        self.assertEqual(saved['inputData'][:6], original['egoVehicle']['initPosition']['vehicleSaveData']['inputData'][:6])
        for name in ('vehicleList', 'pedestrianList', 'obstacleList', 'objectList', 'spawnPointList'):
            self.assertEqual(result[name], [])
        self.assertEqual(result['environment'], original['environment'])

    def test_invalid_override_pose_is_rejected_without_mutation(self):
        source = example_scene()
        original = copy.deepcopy(source)
        for pose in ({'x': math.nan}, {'yaw': math.inf}, {'z': None}, {'x': True}, {'yaaw': 3}, ['x']):
            with self.subTest(pose=pose), self.assertRaises(ValueError):
                probe.normalized_scene(source, pose)
        self.assertEqual(source, original)

    def test_invalid_original_pose_is_not_hidden_by_partial_override(self):
        scene = example_scene()
        scene['egoVehicle']['initPosition']['pos']['y'] = float('nan')
        with self.assertRaises(ValueError):
            probe.normalized_scene(scene, {'x': 5})

    def test_native_json_file_bytes_are_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'native.json'
            original = json.dumps(example_scene(), indent=3).encode()
            source.write_bytes(original)
            selected = probe.validate_scenario_source(source, directory)
            probe.normalized_scene(json.loads(selected.read_bytes()), {'y': 42})
            self.assertEqual(source.read_bytes(), original)


class StdinReaderTest(unittest.TestCase):
    def setUp(self):
        self.read_fd, self.write_fd = os.pipe()
        self.reader = probe.CommandLineReader(self.read_fd)

    def tearDown(self):
        os.close(self.read_fd)
        if self.write_fd is not None:
            os.close(self.write_fd)

    def test_two_batched_json_lines_are_delivered_without_second_fd_readiness(self):
        os.write(self.write_fd, b'{"action":"status"}\n{"action":"stop"}\n')
        first = self.reader.poll(0.1)
        # Writer remains open and the pipe is empty. The second line must come
        # from our byte queue, not select readiness or TextIOWrapper buffering.
        with patch.object(probe.select, 'select', side_effect=AssertionError('must not wait on fd')):
            second = self.reader.poll(0.1)
        self.assertEqual(json.loads(first[1])['action'], 'status')
        self.assertEqual(json.loads(second[1])['action'], 'stop')
        self.assertEqual((first[0], second[0]), ('line', 'line'))

    def test_partial_line_then_eof_is_delivered_once_without_eof_spin(self):
        os.write(self.write_fd, b'{"action":"status"}')
        self.assertEqual(self.reader.poll(0.1), ('timeout', None))
        os.close(self.write_fd)
        self.write_fd = None
        self.assertEqual(self.reader.poll(0.1), ('line', '{"action":"status"}'))
        with patch.object(probe.select, 'select', side_effect=AssertionError('EOF must be sticky')):
            self.assertEqual(self.reader.poll(0.1), ('eof', None))
            self.assertEqual(self.reader.poll(0.1), ('eof', None))

    def test_empty_pipe_timeout_and_split_utf8(self):
        self.assertEqual(self.reader.poll(0), ('timeout', None))
        data = '{"note":"신호"}\n'.encode()
        os.write(self.write_fd, data[:10])
        self.assertEqual(self.reader.poll(0.1), ('timeout', None))
        os.write(self.write_fd, data[10:])
        kind, line = self.reader.poll(0.1)
        self.assertEqual(kind, 'line')
        self.assertEqual(json.loads(line)['note'], '신호')

    def test_oversized_command_is_rejected(self):
        reader = probe.CommandLineReader(self.read_fd, max_line_bytes=5)
        os.write(self.write_fd, b'123456\n')
        with self.assertRaises(ValueError):
            reader.poll(0.1)


class ScopeAndCleanupTest(unittest.TestCase):
    def test_output_scope_freshness_and_symlink_escape(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            allowed = root / 'evaluation'
            allowed.mkdir()
            self.assertEqual(probe.validate_output_directory(allowed / 'new', allowed), allowed / 'new')
            with self.assertRaises(FileExistsError):
                probe.validate_output_directory(allowed, allowed)
            with self.assertRaises(ValueError):
                probe.validate_output_directory(root / 'outside', allowed)
            outside = root / 'outside'
            outside.mkdir()
            (allowed / 'escape').symlink_to(outside, target_is_directory=True)
            with self.assertRaises(ValueError):
                probe.validate_output_directory(allowed / 'escape/new', allowed)

    def test_scenario_source_is_direct_json_in_selected_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            selected = root / 'selected'
            selected.mkdir()
            outside = root / 'outside.json'
            outside.write_text('{}')
            (selected / 'escape.json').symlink_to(outside)
            with self.assertRaises(ValueError):
                probe.validate_scenario_source(outside, selected)
            with self.assertRaises(ValueError):
                probe.validate_scenario_source(selected / 'escape.json', selected)

    def test_idle_timeout_is_finite_positive_and_bounded(self):
        self.assertEqual(probe.validate_idle_timeout(60), 60)
        for value in (0, -1, math.inf, math.nan, 3601):
            with self.assertRaises(ValueError):
                probe.validate_idle_timeout(value)

    def test_bootstrap_is_explicit_and_not_enabled_by_default(self):
        parser = probe.build_argument_parser()
        required = ['--output', '/unused/new', '--scenario-dir', '/unused/scenes']
        self.assertIsNone(parser.parse_args(required).bootstrap_scene)
        self.assertEqual(parser.parse_args(required + ['--bootstrap-scene', '/chosen.json']).bootstrap_scene,
                         Path('/chosen.json'))

    def test_failed_stationary_gate_is_recorded_and_only_owned_items_are_cleaned(self):
        hold, first, second, handle, unrelated = (Mock() for _ in range(5))
        events, stopped = [], []
        shutdown = Mock()
        failures = probe.cleanup_owned(
            hold, [first, second], [handle], Mock(side_effect=RuntimeError('no measured telemetry')),
            Mock(), events.append, stopped.append, shutdown,
        )
        self.assertTrue(failures)
        self.assertEqual(stopped, [second, first])
        self.assertNotIn(unrelated, stopped)
        hold.stop.assert_called_once()
        handle.close.assert_called_once()
        shutdown.assert_called_once()
        failed = [event for event in events if event['event'] == 'shutdown_stationary_failed']
        self.assertEqual(len(failed), 1)
        self.assertFalse(failed[0]['stationary_verified'])
        self.assertFalse(any(event['event'] == 'shutdown_stationary' for event in events))

    def test_cleanup_continues_after_individual_process_and_hold_failures(self):
        hold, first, second, handle = (Mock() for _ in range(4))
        hold.stop.side_effect = RuntimeError('hold failed')
        stop = Mock(side_effect=[RuntimeError('child failed'), None])
        shutdown = Mock()
        failures = probe.cleanup_owned(hold, [first, second], [handle], lambda: {'gear': 1},
                                       lambda: {}, lambda event: None, stop, shutdown)
        self.assertEqual(stop.call_count, 2)
        handle.close.assert_called_once()
        shutdown.assert_called_once()
        self.assertEqual(len(failures), 2)


if __name__ == '__main__':
    unittest.main()
