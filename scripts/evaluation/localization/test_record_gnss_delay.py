#!/usr/bin/env python3
"""Recorder lifecycle and read-only ownership tests; no live ROS/vehicle."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch


SPEC = importlib.util.spec_from_file_location('gnss_delay_recorder', Path(__file__).with_name('record_gnss_delay.py'))
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


class RecorderTest(unittest.TestCase):
    def test_topics_include_raw_reference_and_canonical_pair_not_large_sensors(self):
        self.assertLessEqual(M.REQUIRED, set(M.TOPICS))
        self.assertIn('/tf_static', M.TOPICS)
        self.assertIn('/localization/gt/twist', M.TOPICS)
        self.assertIn('/localization/gt/orientation', M.TOPICS)
        self.assertFalse(any('camera' in topic or 'lidar' in topic for topic in M.TOPICS))
        self.assertNotIn('/ctrl_cmd', M.TOPICS)

    def test_duration_and_heading_are_bounded(self):
        for arguments in [['--duration','nan'], ['--duration','0'], ['--duration','601'],
                          ['--heading-offset-deg','inf']]:
            with self.subTest(arguments=arguments), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                M.parse_args(arguments)

    def test_dry_run_never_contacts_master_or_creates_output(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'not_created'
            with patch.object(M, 'inspect_master') as inspect, patch.object(M, 'allocate_output') as allocate:
                with contextlib.redirect_stdout(io.StringIO()) as stream:
                    self.assertEqual(M.main(['--dry-run','--output-dir',str(target)]),0)
                plan=json.loads(stream.getvalue())
                self.assertFalse(plan['controls_vehicle'])
                self.assertFalse(plan['starts_master'])
                inspect.assert_not_called();allocate.assert_not_called()
                self.assertFalse(target.exists())

    def test_output_never_overwrites_existing_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'run'
            self.assertEqual(M.allocate_output(target),target)
            with self.assertRaises(FileExistsError):M.allocate_output(target)

    def test_master_probe_is_allowlisted_read_only(self):
        proxy=Mock()
        proxy.getPid.return_value=(1,'',123)
        proxy.getPublishedTopics.return_value=(1,'',[[topic,'fixture'] for topic in M.REQUIRED])
        proxy.getParam.return_value=(-1,'missing',None)
        with patch.object(M.xmlrpc.client,'ServerProxy',return_value=proxy):
            result=M.inspect_master('http://127.0.0.1:11311')
        self.assertEqual(result['master_uri'],'http://127.0.0.1:11311')
        self.assertLessEqual({call[0] for call in proxy.method_calls},
                             {'getPid','getPublishedTopics','getParam'})

    def test_missing_reference_is_clear_failure(self):
        proxy=Mock();proxy.getPid.return_value=(1,'',123)
        proxy.getPublishedTopics.return_value=(1,'', [['/sensor/gps','fixture']])
        with patch.object(M.xmlrpc.client,'ServerProxy',return_value=proxy):
            with self.assertRaisesRegex(RuntimeError,'Ego oracle'):
                M.inspect_master('http://127.0.0.1:11311')

    def test_master_failure_does_not_create_files_or_start_ros(self):
        with patch.object(M,'inspect_master',side_effect=RuntimeError('master unavailable')):
            with patch.object(M,'allocate_output') as allocate, patch.object(M.subprocess,'Popen') as spawn:
                with self.assertRaisesRegex(RuntimeError,'master unavailable'):M.main([])
                allocate.assert_not_called();spawn.assert_not_called()

    def test_record_command_uses_bounded_topic_list_and_duration(self):
        command=M.recorder_command(Path('/tmp/path with spaces/data.bag'),90.)
        self.assertEqual(command[:2],['rosbag','record'])
        self.assertIn('--duration=90.0',command)
        self.assertIn('/tmp/path with spaces/data.bag',command)
        self.assertNotIn('-a',command)

    def test_cleanup_signals_only_owned_recorder_group(self):
        process=Mock(pid=4321);process.poll.return_value=None
        with patch.object(M.os,'killpg') as kill:
            M.stop_owned(process)
        kill.assert_called_once_with(4321,signal.SIGINT)

    def test_cleanup_escalates_only_owned_group(self):
        process=Mock(pid=4321);process.poll.return_value=None
        process.wait.side_effect=[subprocess.TimeoutExpired('recorder',1),None]
        with patch.object(M.os,'killpg') as kill:
            M.stop_owned(process,timeout=.1)
        self.assertEqual([call.args for call in kill.call_args_list],
                         [(4321,signal.SIGINT),(4321,signal.SIGTERM)])

    def test_completed_recorder_is_not_signaled(self):
        process=Mock();process.poll.return_value=0
        with patch.object(M.os,'killpg') as kill:M.stop_owned(process)
        kill.assert_not_called()

    def test_existing_bag_is_not_recorded_or_mutated(self):
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'old.bag';source.write_bytes(b'original')
            target=Path(directory)/'new'
            with patch.object(M,'inspect_master') as inspect,patch.object(M,'record') as record:
                with patch.object(M.subprocess,'run',return_value=Mock(returncode=2)) as run:
                    with contextlib.redirect_stdout(io.StringIO()):
                        code=M.main(['--bag',str(source),'--output-dir',str(target)])
            self.assertEqual(code,2)
            self.assertEqual(source.read_bytes(),b'original')
            inspect.assert_not_called();record.assert_not_called()
            self.assertIn('--heading-offset-deg',run.call_args.args[0])
            self.assertEqual(json.loads((target/'provenance.json').read_text())['mode'],'existing_bag')


if __name__=='__main__':unittest.main()
