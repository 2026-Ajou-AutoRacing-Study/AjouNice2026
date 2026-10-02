"""Real subprocess queue tests without ROS, MORAI or moving any vehicle."""
import copy
import fcntl
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import yaml
from planning_scenario_validator import batch, gui
from planning_scenario_validator.config import load_campaign
from planning_scenario_validator.runtime import RuntimeFailure


FAKE_RUNNER = '''#!/usr/bin/python3
import hashlib, json, os, signal, sys, time
from pathlib import Path
from planning_scenario_validator.config import load_campaign, EXECUTION_CONTRACT
cfg = load_campaign(sys.argv[1], check_install=False)
out = cfg['output']; out.mkdir(parents=True, exist_ok=True)
calls = out / 'calls'
calls.write_text(str(int(calls.read_text()) + 1) if calls.exists() else '1')
state = {'execution_contract': EXECUTION_CONTRACT, 'config_path': str(cfg['path']),
         'config_sha256': hashlib.sha256(cfg['path'].read_bytes()).hexdigest(), 'results': {}}
stopped = False
def stop(*args):
    global stopped
    stopped = True
signal.signal(signal.SIGINT, stop)
if os.environ.get('WAIT_CAMPAIGN') == cfg['name']:
    (out / 'waiting').touch()
    while not stopped: time.sleep(.02)
    sys.exit(130)
limit = 1 if os.environ.get('INCOMPLETE_CAMPAIGN') == cfg['name'] else len(cfg['cases'])
for case in cfg['cases'][:limit]:
    state['results'][case.name] = {'status': 'FAIL' if case.name.endswith('two') else 'PASS',
        'source_sha256': case.source_sha256, 'result_path': str(out / case.name)}
(out / 'campaign_state.json').write_text(json.dumps(state))
sys.exit(0)
'''


class BatchTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.queue_file = self.root / 'queue.json'
        self.scenes = self.root / 'scenarios'
        self.scenes.mkdir()
        for name in ('one', 'two'):
            (self.scenes / (name + '.json')).write_text(json.dumps({
                'mapInfo': {'mapName': 'R_KR_PR_K-city_2025'},
                'egoVehicle': {'initPosition': {'pos': {'x': 0, 'y': 0}}}}))
        self.paths = []
        doc = gui.load_document(gui.MISSIONS / 'highway/base/example.yaml')
        for name in ('batch_A', 'batch_B', 'batch_C'):
            value = copy.deepcopy(doc)
            value.update(campaign_name=name, output_root=str(self.root / 'results'))
            value['scenarios'] = {'directory': str(self.scenes), 'glob': '*.json'}
            path = self.root / (name + '.yaml')
            path.write_text(yaml.safe_dump(value))
            self.paths.append(path)
        self.fake = self.root / 'runner'
        self.fake.write_text(FAKE_RUNNER)
        self.fake.chmod(0o755)
        batch.write_queue({'schema_version': 1, 'items': [str(p) for p in self.paths], 'run_directory': None}, self.queue_file)
        root_patch = patch.object(batch, 'ROOT', self.root)
        root_patch.start()
        self.addCleanup(root_patch.stop)

    def run_queue(self, stop_event=None):
        return batch.run_batch(self.queue_file, runner=self.fake, stop_event=stop_event, check_install=False)

    def state(self):
        return batch.batch_state(batch.read_queue(self.queue_file))

    def test_fail_is_completed_and_next_yaml_runs(self):
        self.assertEqual(self.run_queue(), 0)
        state = self.state()
        self.assertEqual(state['status'], 'DONE')
        self.assertEqual([r['counts'] for r in state['items']], [{'PASS': 1, 'FAIL': 1, 'ERROR': 0}] * 3)
        self.assertTrue(all(r['status'] == 'DONE' for r in state['items']))
        directory = Path(batch.read_queue(self.queue_file)['run_directory'])
        self.assertTrue((directory / 'summary.csv').is_file())
        self.assertEqual(batch.read_queue(directory / 'queue.json'), batch.read_queue(self.queue_file))
        self.assertEqual(self.run_queue(), 0)
        self.assertEqual((self.root / 'results/batch_A/calls').read_text(), '1')

    def test_manual_interrupt_B_then_resume_A_is_not_repeated(self):
        stop = threading.Event()
        errors = []
        def interrupt():
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                if (self.root / 'results/batch_B/waiting').exists():
                    stop.set()
                    return
                time.sleep(.02)
            errors.append('B did not start')
            stop.set()
        thread = threading.Thread(target=interrupt)
        thread.start()
        with patch.dict(os.environ, {'WAIT_CAMPAIGN': 'batch_B'}):
            self.assertEqual(self.run_queue(stop), 130)
        thread.join(2)
        self.assertFalse(errors)
        self.assertEqual([r['status'] for r in self.state()['items']], ['DONE', 'INTERRUPTED', 'PENDING'])
        self.assertFalse((self.root / 'results/batch_C').exists())
        self.assertEqual(self.run_queue(), 0)
        self.assertEqual((self.root / 'results/batch_A/calls').read_text(), '1')
        self.assertEqual((self.root / 'results/batch_B/calls').read_text(), '2')

    def test_zero_exit_with_missing_scenes_stops_queue(self):
        with patch.dict(os.environ, {'INCOMPLETE_CAMPAIGN': 'batch_B'}):
            self.assertEqual(self.run_queue(), 1)
        self.assertEqual(self.state()['status'], 'ERROR')
        self.assertFalse((self.root / 'results/batch_C').exists())

    def test_changed_input_cannot_resume_and_old_state_is_preserved(self):
        self.assertEqual(self.run_queue(), 0)
        state_path = Path(batch.read_queue(self.queue_file)['run_directory']) / 'state.json'
        original = state_path.read_bytes()
        self.paths[1].write_text(self.paths[1].read_text() + '# changed\n')
        with self.assertRaisesRegex(ValueError, '바뀌었습니다'):
            self.run_queue()
        self.assertEqual(state_path.read_bytes(), original)

    def test_duplicate_and_network_mismatch_rejected(self):
        with self.assertRaisesRegex(ValueError, '중복'):
            batch.plan([self.paths[0], self.paths[0]], check_install=False)
        doc = yaml.safe_load(self.paths[1].read_text())
        doc['campaign_name'] = 'batch_A'
        self.paths[1].write_text(yaml.safe_dump(doc))
        with self.assertRaisesRegex(ValueError, '결과 폴더'):
            batch.plan(self.paths[:2], check_install=False)
        doc['campaign_name'] = 'batch_B'
        doc.setdefault('morai', {})['scenario_load_port'] = 9096
        self.paths[1].write_text(yaml.safe_dump(doc))
        with self.assertRaisesRegex(ValueError, '무인 배치'):
            batch.plan(self.paths[:2], check_install=False)

    def test_queue_lock_prevents_duplicate_run(self):
        with self.queue_file.with_suffix('.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with self.assertRaisesRegex(RuntimeFailure, '이미 실행'):
                self.run_queue()
        self.assertFalse(batch.read_queue(self.queue_file)['run_directory'])

    def test_malformed_queue_and_state_are_preserved(self):
        self.queue_file.write_text('[]')
        with self.assertRaisesRegex(ValueError, '형식'):
            batch.read_queue(self.queue_file)
        self.assertEqual(self.queue_file.read_text(), '[]')
        (self.root / 'state.json').write_text('{"schema_version":1,"items":[]}')
        with self.assertRaisesRegex(ValueError, 'state 형식'):
            batch.batch_state({'run_directory': str(self.root)})

    def test_changed_scene_cannot_resume(self):
        self.assertEqual(self.run_queue(), 0)
        state_path = Path(batch.read_queue(self.queue_file)['run_directory']) / 'state.json'
        original = state_path.read_bytes()
        (self.scenes / 'one.json').write_text((self.scenes / 'one.json').read_text() + '\n')
        with self.assertRaisesRegex(ValueError, '바뀌었습니다'):
            self.run_queue()
        self.assertEqual(state_path.read_bytes(), original)

    def test_ros_graph_lock_prevents_different_campaigns(self):
        import hashlib
        from planning_scenario_validator import runner
        directory = self.root / 'planner_logs/validation'
        directory.mkdir(parents=True)
        master = 'http://127.0.0.1:11311'
        key = hashlib.sha256(master.encode()).hexdigest()[:16]
        with patch.object(runner, 'ROOT', self.root), patch.dict(os.environ, {'ROS_MASTER_URI': master}):
            with (directory / ('.ros_graph_' + key + '.lock')).open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with patch.object(runner, '_run_campaign') as child:
                    with self.assertRaisesRegex(RuntimeFailure, '같은 ROS master'):
                        runner.run({})
                    child.assert_not_called()

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'GUI requires Xvfb')
    def test_gui_batch_runs_and_monitor_shows_yaml_and_scene_progress(self):
        executable = self.root / 'batch-entry'
        executable.write_text('''#!/usr/bin/python3
import signal, sys, threading
from planning_scenario_validator import batch
stop = threading.Event()
signal.signal(signal.SIGINT, lambda *_: stop.set())
sys.exit(batch.run_batch(sys.argv[1], runner=%r, stop_event=stop, check_install=False))
''' % str(self.fake))
        executable.chmod(0o755)
        captured = []
        with patch.dict(os.environ, {'XDG_CONFIG_HOME': str(self.root / 'gui')}):
            batch.write_queue(batch.read_queue(self.queue_file))
            def ready(app):
                deadline = time.monotonic() + 15
                def check():
                    try:
                        app['actions']['update_progress']()
                        state = batch.batch_state(batch.read_queue())
                        if state and state['status'] == 'DONE':
                            app['actions']['update_progress']()
                            self.assertIn('YAML 3/3', app['status'].get())
                            self.assertIn('실패 3', app['status'].get())
                            self.assertEqual(len(app['batch_table'].get_children()), 3)
                            captured.append(True)
                            app['root'].destroy()
                        elif time.monotonic() > deadline:
                            raise AssertionError('GUI batch did not finish')
                        else:
                            app['root'].after(50, check)
                    except BaseException as error:
                        captured.append(error)
                        app['actions']['stop']()
                        app['root'].destroy()
                def start():
                    with patch('tkinter.messagebox.askokcancel', return_value=True):
                        app['actions']['batch_start']()
                    self.assertEqual(app['active']['mode'], 'batch')
                    app['root'].after(50, check)
                app['root'].after(50, start)
            gui.main(on_ready=ready, batch_executable=executable)
        self.assertEqual(len(captured), 1)
        if isinstance(captured[0], BaseException):
            raise captured[0]

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'GUI requires Xvfb')
    def test_gui_rejected_resume_does_not_show_old_done_as_success(self):
        self.assertEqual(self.run_queue(), 0)
        previous = batch.read_queue(self.queue_file)
        self.paths[1].write_text(self.paths[1].read_text() + '# changed input\n')
        executable = self.root / 'rejected-batch'
        executable.write_text('''#!/usr/bin/python3
import sys
from planning_scenario_validator import batch
sys.exit(batch.run_batch(sys.argv[1], runner=%r, check_install=False))
''' % str(self.fake))
        executable.chmod(0o755)
        captured = []
        with patch.dict(os.environ, {'XDG_CONFIG_HOME': str(self.root / 'gui')}):
            batch.write_queue(previous)
            def ready(app):
                deadline = time.monotonic() + 10
                def check():
                    try:
                        if app['active'].get('batch_error'):
                            app['actions']['update_progress']()
                            self.assertIn('배치 실행 오류', app['status'].get())
                            self.assertEqual(batch.batch_state(batch.read_queue())['status'], 'DONE')
                            captured.append(True)
                            app['root'].destroy()
                        elif time.monotonic() > deadline:
                            raise AssertionError('rejection was not visible')
                        else:
                            app['root'].after(50, check)
                    except BaseException as error:
                        captured.append(error)
                        app['root'].destroy()
                def start():
                    with patch('tkinter.messagebox.askokcancel', return_value=True):
                        app['actions']['batch_start']()
                    app['root'].after(50, check)
                app['root'].after(50, start)
            gui.main(on_ready=ready, batch_executable=executable)
        self.assertEqual(len(captured), 1)
        if isinstance(captured[0], BaseException):
            raise captured[0]

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'GUI requires Xvfb')
    def test_gui_restore_and_new_copy_preserve_completed_inputs_and_results(self):
        self.assertEqual(self.run_queue(), 0)
        previous = batch.read_queue(self.queue_file)
        snapshot = Path(previous['run_directory']) / 'queue.json'
        original_inputs = [path.read_bytes() for path in self.paths]
        original_state = (Path(previous['run_directory']) / 'state.json').read_bytes()
        captured = []
        example = gui.MISSIONS / 'highway/base/example.yaml'
        with patch.dict(os.environ, {'XDG_CONFIG_HOME': str(self.root / 'gui')}), patch.object(gui, 'MISSIONS', self.root / 'missions'), patch.object(gui, 'campaign_examples', return_value=[example]):
            def ready(app):
                def exercise():
                    try:
                        with patch('tkinter.filedialog.askopenfilename', return_value=str(snapshot)):
                            app['actions']['batch_restore']()
                        self.assertEqual(batch.read_queue(), previous)
                        with patch('tkinter.messagebox.askyesno', return_value=True):
                            app['actions']['batch_clone']()
                        copied = batch.read_queue()
                        self.assertIsNone(copied['run_directory'])
                        self.assertEqual(len(copied['items']), 3)
                        for old, new in zip(self.paths, copied['items']):
                            self.assertNotEqual(str(old), new)
                            self.assertTrue(Path(new).is_file())
                            doc = gui.load_document(new)
                            self.assertNotEqual(doc['campaign_name'], old.stem)
                            self.assertEqual(doc['scenarios']['directory'], str(self.scenes))
                        self.assertEqual([path.read_bytes() for path in self.paths], original_inputs)
                        self.assertEqual((Path(previous['run_directory']) / 'state.json').read_bytes(), original_state)
                        captured.append(True)
                    except BaseException as error:
                        captured.append(error)
                    finally:
                        app['root'].destroy()
                app['root'].after(50, exercise)
            gui.main(on_ready=ready)
        self.assertEqual(len(captured), 1)
        if isinstance(captured[0], BaseException):
            raise captured[0]

    @unittest.skipUnless(os.environ.get('DISPLAY'), 'GUI requires Xvfb')
    def test_gui_add_reorder_persist_and_clear_keep_results(self):
        captured = []
        with patch.dict(os.environ, {'XDG_CONFIG_HOME': str(self.root / 'gui')}):
            def ready(app):
                def exercise():
                    try:
                        for path in self.paths[:2]:
                            with patch('tkinter.filedialog.askopenfilenames', return_value=(str(path),)):
                                app['actions']['batch_edit']('files')
                        app['batch_table'].selection_set('1')
                        app['actions']['batch_edit']('up')
                        self.assertEqual(batch.read_queue()['items'], [str(self.paths[1]), str(self.paths[0])])
                        self.assertIn('YAML 2개', app['batch_note'].get())
                        marker = self.root / 'old-result'
                        marker.write_text('keep')
                        with patch('tkinter.messagebox.askyesno', return_value=True):
                            app['actions']['batch_edit']('reset')
                        self.assertEqual(batch.read_queue()['items'], [])
                        self.assertEqual(marker.read_text(), 'keep')
                        captured.append(True)
                    except BaseException as error:
                        captured.append(error)
                    finally:
                        app['root'].destroy()
                app['root'].after(50, exercise)
            gui.main(on_ready=ready)
        self.assertEqual(len(captured), 1)
        if isinstance(captured[0], BaseException):
            raise captured[0]


if __name__ == '__main__':
    unittest.main()
