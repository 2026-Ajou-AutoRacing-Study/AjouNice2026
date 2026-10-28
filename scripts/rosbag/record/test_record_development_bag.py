import contextlib
import importlib.util
import io
from pathlib import Path
import re
import json
import signal
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('recorder', Path(__file__).with_name('record_development_bag.py'))
recorder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(recorder)


class RecordingTests(unittest.TestCase):
    def test_core_and_diagnostics_are_selected(self):
        for topic in recorder.REQUIRED + recorder.DIAGNOSTICS + [
                '/sensor/lidar1/packets', '/sensor/camera/back_left/compressed',
                '/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/trajectory',
                '/planning/planning_factors/roundabout', '/control/command/control_cmd',
                '/perception/traffic_light_recognition/map_rois',
                '/traffic_light_node/debug_selection', '/traffic_light_node/debug_classifier',
                '/traffic_light_node/debug_runtime',
                '/perception/traffic_light_recognition/planner_decision_debug',
                '/map/vector_map', '/system/operation_mode/state',
                '/vehicle/status/velocity_status', '/vehicle/status/steering_status']:
            with self.subTest(topic=topic):
                self.assertTrue(any(re.match(p, topic) for p in recorder.TOPICS))

    def test_no_duplicate_decoded_images_by_default(self):
        self.assertFalse(any(re.match(p, '/sensor/camera/front') for p in recorder.TOPICS))
        self.assertFalse(any(re.match(p, '/sensor/camera/front/image_raw') for p in recorder.TOPICS))

    def test_placeholder_cruise_output_is_not_required(self):
        self.assertFalse(any(n.endswith('/obstacle_cruise/trajectory') for n in recorder.DIAGNOSTICS))

    def test_added_planner_context_is_recorded_without_new_mandatory_startup_gates(self):
        for topic in ('/map/vector_map', '/system/operation_mode/state',
                      '/vehicle/status/velocity_status', '/vehicle/status/steering_status'):
            with self.subTest(topic=topic):
                self.assertTrue(any(re.match(p, topic) for p in recorder.TOPICS))
                self.assertNotIn(topic, recorder.REQUIRED)

    def test_snapshot_covers_actual_signal_braking_and_stop_pipeline_namespaces(self):
        # The 261028 team LAN archive had signal messages but none of these
        # parameters, so current defaults could not prove the active brake model.
        for name in (
                '/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner',
                '/planning/scenario_planning/lane_driving/motion_planning/path_optimizer',
                '/planning/scenario_planning/velocity_smoother', '/longi_pi_autoware'):
            with self.subTest(namespace=name):
                self.assertIn(name, recorder.PARAMETERS)
        self.assertEqual(len(recorder.PARAMETERS), len(set(recorder.PARAMETERS)))

    def test_list_needs_neither_ros_nor_files(self):
        with patch.object(sys, 'argv', ['recorder', '--list']), contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(recorder.main(), 0)
        self.assertIn('obstacle_cruise', output.getvalue())

    def test_secret_parameters_redacted_recursively(self):
        self.assertEqual(recorder.scrub({'safe': [{'api_key': 'private', 'n': 2}], 'TOKEN': 'secret'}),
                         {'safe': [{'api_key': '[REDACTED]', 'n': 2}], 'TOKEN': '[REDACTED]'})

    def test_bad_options_fail_without_ros(self):
        for args in (['--duration', '-1'], ['--minimum-free-gib', 'nan'], ['--minimum-free-gib', '1'],
                     ['--host-interval', 'nan'], ['--host-interval', '0'],
                     ['--udp-port', '0'], ['--telemetry-pid', '-1'], ['--session-id', '../../bad']):
            with self.subTest(args=args), patch.object(sys, 'argv', ['recorder']+args), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    recorder.main()

    def test_internal_tracker_diagnostics_selected(self):
        self.assertTrue(any(re.match(p, '/perception/object_recognition/tracking/diagnostics/association')
                            for p in recorder.TOPICS))

    def test_audit_only_does_not_contact_master_or_write(self):
        with patch.object(sys, 'argv', ['recorder', '--audit-only', '/tmp/closed.bag']), \
                patch.object(recorder, 'audit', return_value={'offline': True}) as audit, \
                contextlib.redirect_stdout(io.StringIO()) as stream:
            self.assertEqual(recorder.main(), 0)
        self.assertEqual(json.loads(stream.getvalue()), {'offline': True})
        self.assertEqual(audit.call_args[0][0], Path('/tmp/closed.bag'))

    def test_telemetry_args_are_passed_without_shell(self):
        args = types.SimpleNamespace(host_interval=1., udp_port=[2368, 2368, 9090],
                                     telemetry_pid=[42], no_host_gpu=True)
        command = recorder.telemetry_command(Path('/tmp/new'), 'paired-run', args)
        self.assertEqual(command.count('2368'), 1)
        self.assertIn('paired-run', command)
        self.assertIn('--no-gpu', command)
        self.assertIn('--pid', command)

    def test_snapshot_is_read_only_and_handles_missing_parameters(self):
        class Master:
            def getPublishedTopics(self, namespace):
                return [['/sensor/lidar1', 'sensor_msgs/PointCloud2']]
            def getSystemState(self):
                return [[['/sensor/lidar1', ['/lidar']]], [], []]
        class Ros:
            @staticmethod
            def get_param(name):
                if name == '/use_sim_time':
                    return False
                if name == '/bevfusion_ros_bridge':
                    return {'publish_frame_timing': True, 'token': 'secret'}
                raise KeyError(name)
        state = recorder.snapshot(Master(), Ros())
        self.assertFalse(state['parameters']['/use_sim_time'])
        self.assertEqual(state['parameters']['/bevfusion_ros_bridge']['token'], '[REDACTED]')
        self.assertIsNone(state['parameters']['/traffic_light_node'])

    def test_only_own_host_child_is_terminated(self):
        class Process:
            returncode = None
            def poll(self):
                return self.returncode
            def send_signal(self, sig):
                self.signal = sig
            def wait(self, timeout=None):
                self.returncode = 0
        process = Process()
        result = recorder.stop_child(process)
        self.assertEqual(process.signal, signal.SIGTERM)
        self.assertFalse(result['forced'])

    def test_telemetry_audit_distinguishes_missing_and_incomplete(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'host.jsonl'
            self.assertEqual(recorder.telemetry_audit(path)['status'], 'missing')
            path.write_text('{"record_type":"sample"}\ninvalid\n')
            report = recorder.telemetry_audit(path)
            self.assertEqual(report['samples'], 1)
            self.assertEqual(report['invalid_lines'], 1)
            self.assertFalse(report['ended'])

    def test_host_child_exit_race_still_reaped(self):
        class Process:
            returncode = None
            waited = False
            def poll(self):
                return None
            def send_signal(self, sig):
                raise ProcessLookupError()
            def wait(self, timeout=None):
                self.waited = True
                self.returncode = 0
        process = Process()
        self.assertFalse(recorder.stop_child(process)['forced'])
        self.assertTrue(process.waited)

    def test_advertisement_does_not_claim_recording(self):
        state = {'advertised_topics': [['/perception/bevfusion/frame_timing', 'std_msgs/String']]}
        report = recorder.diagnostic_readiness(state)
        self.assertTrue(report['advertised_groups']['detector_stage_timing'])
        self.assertIn('Advertisement only', report['note'])

    def test_master_timeout_skips_remaining_snapshot_reads(self):
        class Ros:
            calls = 0
            def get_param(self, name):
                self.calls += 1
                raise TimeoutError()
        ros = Ros()
        state = recorder.snapshot(None, ros)
        self.assertEqual(ros.calls, 1)
        self.assertIn('snapshot_incomplete', state['errors'])

    def test_snapshot_http_transport_has_a_timeout(self):
        connection = recorder.SnapshotTransport().make_connection('127.0.0.1:11311')
        self.assertEqual(connection.timeout, 2.)


if __name__ == '__main__':
    unittest.main()
