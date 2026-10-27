"""Synthetic-only recorder contracts: no live ROS, native API or ffmpeg process."""
import ast
import json
from pathlib import Path
import tempfile
import signal
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import record_traffic_light_evidence_video as video


def record(payload=None, source=10_000_000_000, receipt=10., sequence=1):
    return dict(payload=payload or {}, source_stamp_ns=source,
                receipt_monotonic=receipt, receipt_ros_ns=10_010_000_000, sequence=sequence)


class EvidenceVideoTest(unittest.TestCase):
    def test_footer_signal_depth_is_distinct_from_stopline_distance(self):
        runtime = record({'frame_decision': {'group_id': 201351,
            'observation_distance_gate': {'current_matched_camera_depth_m': 37.125}}})
        evidence = video.frame_evidence(0, 10, 10, 10.1, 10_100_000_000,
            {'runtime': runtime}, record(), False)
        lines = video.telemetry_lines('current', evidence)
        self.assertIn('신호등까지(전방 깊이) 37.12 m', lines[2])
        self.assertIn('정차 목표 ? m', lines[2])
        self.assertEqual(lines[-1], '')
        self.assertIn('판단 주기 아님', video.telemetry_lines('과거 판단 0.5초 표본', evidence)[-1])
        runtime['payload']['frame_decision']['observation_distance_gate'] = {}
        missing = video.frame_evidence(0, 10, 10, 10.1, 10_100_000_000,
            {'runtime': runtime}, record(), False)
        self.assertIn('전방 깊이) ? m', video.telemetry_lines('current', missing)[2])

    def test_footer_uses_actual_same_mission_decision_not_lamp_color(self):
        runtime = record({'frame_decision': {'group_id': 201326,
            'observed_tuples': [['green', 'circle', 1]]}})
        planner = record({'group_id': 201326, 'require_stop': True, 'committed': False})
        evidence = video.frame_evidence(0, 10, 10, 10.1, 10_100_000_000,
            {'runtime': runtime, 'planner_decision:201326': planner}, record(), False)
        lines = video.telemetry_lines('current', evidence)
        self.assertIn('미션 D', lines[0])
        self.assertIn('녹색', lines[0])
        self.assertIn('STOP 정지', lines[1])
        stale = video.frame_evidence(1, 10, 10, 11, 11_000_000_000,
            {'runtime': runtime, 'planner_decision:201326': planner}, record(), True)
        self.assertIn('판단 정보 없음', video.telemetry_lines('current', stale)[1])
        missing = video.frame_evidence(0, 10, 10, 10.1, 10_100_000_000,
            {'runtime': runtime, 'planner_decision:201351': planner}, record(), False)
        self.assertIn('판단 정보 없음', video.telemetry_lines('current', missing)[1])

    def test_time_stamps_and_nonfinite_values_preserve_unknowns(self):
        self.assertEqual(video.stamp_ns(SimpleNamespace(secs=10, nsecs=123)), 10_000_000_123)
        self.assertEqual(video.stamp_ns(SimpleNamespace(sec=10, nanosec=123)), 10_000_000_123)
        self.assertIsNone(video.stamp_ns(SimpleNamespace(secs=0, nsecs=0)))
        self.assertIsNone(video.stamp_ns(SimpleNamespace(secs=10, nsecs=10**9)))
        self.assertEqual(video.clean({'unknown': float('nan'), 'bits': [0, float('inf')]}),
                         {'unknown': None, 'bits': [0, None]})

    def test_output_fresh_and_in_scope_with_symlink_escape_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            parent = root/'artifacts/evaluation'
            parent.mkdir(parents=True)
            self.assertEqual(video.validate_output(parent/'fresh', root), parent/'fresh')
            with self.assertRaises(ValueError): video.validate_output(parent, root)
            with self.assertRaises(ValueError): video.validate_output(root/'outside', root)
            (parent/'escape').symlink_to(root)
            with self.assertRaises(ValueError): video.validate_output(parent/'escape/fresh', root)

    def test_mailbox_retains_only_latest_values_and_counters(self):
        latest = video.Latest()
        for index in range(100):
            latest.put('image', b'jpeg'+bytes([index]), index+1, index+2, index)
        latest.put('ego', {'speed_kph': 10}, None, 100, 101)
        latest.error('runtime')
        self.assertEqual(len(latest.snapshot()), 2)
        self.assertEqual(latest.snapshot()['image']['sequence'], 100)
        self.assertEqual(latest.snapshot()['image']['source_stamp_ns'], 100)
        self.assertEqual(latest.counters()['callback_errors'], {'runtime': 1})

    def test_frozen_ros_clock_is_stale_on_wall_clock_and_old_stamp_not_refreshed(self):
        image = record()
        status, ages = video.freshness(image, 10.6, 10_000_000_000)
        self.assertIn('STALE', status)
        self.assertEqual(ages['source_stamp_ns'], 10_000_000_000)
        self.assertEqual(ages['source_age_sec'], 0.)
        self.assertIn('WAITING', video.freshness(None, 10, 0)[0])
        self.assertIn('UNVERIFIED', video.freshness(record(source=None), 10, 0)[0])
        self.assertIn('FUTURE', video.freshness(image, 10.1, 9_900_000_000)[0])

    def test_planned_speeds_use_same_frame_map_pose_and_forward_arc_distance(self):
        trajectory = record({'frame_id': 'map', 'points': [[0, 0, 1], [3, 0, 2], [6, 0, 0], [12, 0, 4]]})
        odom = record({'frame_id': '/map', 'x': .1, 'y': 0})
        result = video.planned_speeds(trajectory, odom, 10.1)
        self.assertEqual([result[key] for key in ('nearest_mps', 'ahead_5m_mps', 'ahead_10m_mps')], [1, 0, 4])
        odom['payload']['frame_id'] = 'morai_map'
        self.assertIn('frame_mismatch', video.planned_speeds(trajectory, odom, 10.1)['reason'])
        odom['payload']['frame_id'] = 'map'
        self.assertIn('stale', video.planned_speeds(trajectory, odom, 12)['reason'])
        odom['payload']['x'] = 1000
        self.assertFalse(video.planned_speeds(trajectory, odom, 10.1)['available'])

    def test_runtime_match_requires_exact_camera_stamp_and_retains_candidate_hold(self):
        runtime = record({'frame_decision': {'observed_tuples': [['green', 'circle', 1]],
            'final_tuples': [], 'stamped_vote': {'enabled': True, 'winner_vote_count': 1},
            'observation_timing': {'output_origin': 'stamped_vote_unknown'}}})
        evidence = video.frame_evidence(3, 10, 10, 10.31, 10_310_000_000,
            {'runtime': runtime}, record(b'JPEG'), True)
        self.assertEqual(evidence['pts_sec'], .3)
        self.assertTrue(evidence['runtime_exact_image_stamp_match'])
        self.assertTrue(evidence['image_repeated'])
        self.assertEqual(evidence['image']['source_stamp_ns'], 10_000_000_000)
        self.assertEqual(evidence['telemetry']['runtime']['value']['frame_decision']['observation_timing']['output_origin'], 'stamped_vote_unknown')
        runtime['source_stamp_ns'] += 1
        newer = video.frame_evidence(4, 10, 10, 10.4, 10_400_000_000,
            {'runtime': runtime}, record(), True)
        self.assertFalse(newer['runtime_exact_image_stamp_match'])
        json.dumps(newer, allow_nan=False)

    def test_render_even_canvas_without_covering_original_camera(self):
        import numpy as np
        image = np.full((1013, 1280, 3), 75, dtype=np.uint8)
        evidence = video.frame_evidence(0, 10, 10, 10.1, 10_100_000_000, {}, record(), False)
        frame = video.render_frame(image, 'current', evidence)
        self.assertEqual(frame.shape, (960, 1280, 3))
        self.assertTrue(np.array_equal(frame[:720], image[:720]))
        stale = video.frame_evidence(1, 10, 10, 11, 11_000_000_000, {}, record(), True)
        self.assertLess(video.render_frame(image, 'current', stale)[0, 0, 0], 75)

    def test_encoder_arguments_are_explicit_bounded_and_never_overwrite(self):
        software = video.ffmpeg_arguments(Path('/new/video.mp4'), 10, 'libx264')
        self.assertIn('ultrafast', software)
        self.assertIn('-crf', software)
        self.assertEqual(software[software.index('-threads')+1], '2')
        self.assertIn('-n', software)
        self.assertNotIn('-y', software)
        hardware = video.ffmpeg_arguments(Path('/new/video.mp4'), 10, 'h264_nvenc')
        self.assertIn('h264_nvenc', hardware)
        self.assertIn('-cq', hardware)
        self.assertNotIn('-crf', hardware)

    def test_ffmpeg_close_uses_EOF_wait_and_reports_nonzero(self):
        writer = object.__new__(video.VideoWriter)
        writer.process = Mock()
        writer.process.wait.return_value = 0
        writer.close()
        writer.process.stdin.close.assert_called_once()
        writer.process.wait.assert_called_once_with(timeout=20)
        writer.process.terminate.assert_not_called()
        writer.process.wait.return_value = 2
        with self.assertRaisesRegex(RuntimeError, 'exit code 2'): writer.close()

    def test_source_has_no_publisher_native_vehicle_or_sensor_mutations(self):
        tree = ast.parse(Path(video.__file__).read_text())
        calls = {node.func.attr for node in ast.walk(tree)
                 if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
        self.assertIn('Subscriber', calls)
        self.assertFalse(calls.intersection({'Publisher', 'ServiceProxy', 'set_traffic_light', 'publish'}))
        self.assertNotIn('monitor_traffic_light_trial', Path(video.__file__).read_text())

    def run_mocked_main(self, directory, supply_image=True, fail_finalize=False):
        """Exercise production lifecycle with no ROS connection or subprocess."""
        import cv2
        import numpy as np
        output = Path(directory)/'fresh'
        fake_rospy = SimpleNamespace(init_node=Mock(), is_shutdown=lambda: False,
            Time=SimpleNamespace(now=lambda: SimpleNamespace(to_nsec=lambda: 10_000_000_000)))
        subscription = Mock()
        writers = []

        def subscriptions(_rospy, latest):
            if supply_image:
                ok, encoded = cv2.imencode('.jpg', np.full((1013, 1280, 3), 50, np.uint8))
                self.assertTrue(ok)
                latest.put('image', encoded.tobytes(), 10_000_000_000,
                           10_000_000_000, video.time.monotonic())
            return [subscription]

        class FakeWriter:
            def __init__(self, command, log):
                self.command, self.closed = command, False
                writers.append(self)

            def write(self, frame):
                self.shape = frame.shape
                # Invoke this test process's handler, never signal another node.
                signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)

            def close(self):
                self.closed = True
                if fail_finalize:
                    raise RuntimeError('synthetic finalize failure')
                (output/'front_evidence.mp4').write_bytes(b'SYNTHETIC TEST ONLY - NOT AN ACTUAL VIDEO')

        with patch.dict(sys.modules, {'rospy': fake_rospy}), \
                patch.object(video, 'subscribe', subscriptions), \
                patch.object(video, 'VideoWriter', FakeWriter), \
                patch.object(video.shutil, 'which', return_value='/synthetic/ffmpeg'), \
                patch.object(video, 'validate_output', return_value=output):
            result = video.main(['--output', str(output), '--label', 'vote3', '--fps', '10',
                                 '--max-seconds', '.1', '--encoder', 'h264_nvenc'])
        self.assertTrue(writers[0].closed)
        subscription.unregister.assert_called_once()
        return result, json.loads((output/'metadata.json').read_text()), output

    def test_mocked_main_signal_closes_video_and_persists_all_clock_domains(self):
        original_handler = signal.getsignal(signal.SIGTERM)
        with tempfile.TemporaryDirectory() as directory:
            result, metadata, output = self.run_mocked_main(directory)
            self.assertEqual(result, 0)
            self.assertEqual(metadata['status'], 'complete')
            self.assertEqual(metadata['stop_reason'], 'SIGTERM')
            self.assertEqual(metadata['encoder'], 'h264_nvenc')
            self.assertEqual(metadata['decoded_image_count'], 1)
            for key in ('started_ros_ns', 'ended_ros_ns', 'started_monotonic', 'ended_monotonic',
                        'started_utc', 'ended_utc', 'first_frame'):
                self.assertIn(key, metadata)
            frame = json.loads((output/'frames.jsonl').read_text())
            self.assertEqual(frame['frame_index'], 0)
            self.assertEqual(frame['pts_sec'], 0.)
            self.assertEqual(frame['image']['source_stamp_ns'], 10_000_000_000)
            self.assertIn('write_completed_monotonic', frame)
        self.assertEqual(signal.getsignal(signal.SIGTERM), original_handler)

    def test_mocked_waiting_and_ffmpeg_finalize_failure_are_unqualified_nonzero(self):
        with tempfile.TemporaryDirectory() as directory:
            result, metadata, output = self.run_mocked_main(directory, supply_image=False)
            self.assertEqual(result, 1)
            self.assertEqual(metadata['status'], 'failed')
            self.assertIn('No real annotated camera frame', metadata['error'])
            frame = json.loads((output/'frames.jsonl').read_text())
            self.assertIn('WAITING', frame['image_status'])
        with tempfile.TemporaryDirectory() as directory:
            result, metadata, _ = self.run_mocked_main(directory, fail_finalize=True)
            self.assertEqual(result, 1)
            self.assertIn('synthetic finalize failure', metadata['error'])


if __name__ == '__main__':
    unittest.main()
