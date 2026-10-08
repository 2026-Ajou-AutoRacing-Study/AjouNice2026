import json
from pathlib import Path
import sys
import tempfile
import unittest
import shutil
import subprocess
import threading
from html.parser import HTMLParser
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_dashboard import status_payload,Dashboard,reason_counts,HTML


class DashboardTests(unittest.TestCase):
    def fixture(self,root):
        capture=root/'capture';manifest=root/'manifest.json'
        manifest.write_text(json.dumps(dict(dataset_id='test_sunny13',
            scenes=[dict(id='a',family='highway',motion='static'),dict(id='b',family='obstacle',motion='walking')],
            collection_order=dict(policy='stationary-first; animated-last; stable within each group',stationary=1,animated=1))))
        return capture,manifest

    def test_not_started_and_disk(self):
        with tempfile.TemporaryDirectory() as folder:
            result=status_payload(Path(folder)/'future')
            self.assertEqual(result['stage'],'not_started')
            self.assertGreater(result['disk_free_gib'],0)
            self.assertFalse(result['status_stale'])
            self.assertEqual(result['eta_label'],'수집 전')
            self.assertFalse((Path(folder)/'future').exists())

    def test_storage_estimate_and_zero_approved(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);capture,manifest=self.fixture(root);capture.mkdir()
            (capture/'status.json').write_text(json.dumps(dict(stage='running',requested=5000,
                finished=1000,accepted_candidate=900,elapsed_sec=600,updated_at=0)))
            d=Dashboard(capture,manifest)
            self.assertIsNone(d.payload()['storage_estimate']['total_gib'])
            d.storage_sample=dict(bytes=9*1024**3,accepted=900,observed_at=123)
            s=d.payload()['storage_estimate']
            self.assertAlmostEqual(s['total_gib'],50)
            self.assertAlmostEqual(s['additional_gib'],41)
            self.assertAlmostEqual(s['free_after_gib'],d.payload()['disk_free_gib']-41,places=3)
            d.storage_sample['accepted']=0
            self.assertIsNone(d.payload()['storage_estimate']['total_gib'])

    def test_storage_scan_counts_hardlinks_once_and_preserves_status(self):
        import os
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);capture,manifest=self.fixture(root);capture.mkdir()
            state=capture/'status.json';state.write_text('{"accepted_candidate":1}')
            before=state.read_bytes();blob=capture/'raw';blob.write_bytes(bytes(65536))
            d=Dashboard(capture,manifest);d.sample_storage();size=d.storage_sample['bytes']
            os.link(blob,capture/'qa_hardlink');d.sample_storage()
            self.assertEqual(d.storage_sample['bytes'],size)
            self.assertEqual(state.read_bytes(),before)

    def test_zero_eta_does_not_claim_completion_for_running_or_halted(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for stage,label,stale in [('running','계측 중',True),
                    ('halted','중단 · 확인 필요',False),('halted_exit_safety','중단 · 확인 필요',False),
                    ('paused','일시정지',False),('complete_with_deficit','부족분 남음',False),
                    ('complete','완료',False)]:
                (root/'status.json').write_text(json.dumps(dict(stage=stage,requested=2,finished=2,
                    accepted_candidate=2,eta_sec=0,updated_at=0)))
                result=status_payload(root)
                self.assertEqual(result['eta_label'],label)
                self.assertEqual(result['status_stale'],stale)

    def test_rate_and_internal_child_only(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);child=root/'child';child.mkdir()
            (child/'status.json').write_text('{"finished":2}')
            state=dict(current=str(child),elapsed_sec=60,accepted_candidate=2)
            (root/'status.json').write_text(json.dumps(state))
            result=status_payload(root)
            self.assertEqual(result['accepted_per_hour'],120)
            self.assertEqual(result['child_progress']['finished'],2)
            active=child/'calibration';active.mkdir()
            (active/'status.json').write_text('{"stage":"waiting_motion_target"}')
            (child/'status.json').write_text(json.dumps(dict(child_status=str(active/'status.json'))))
            self.assertEqual(status_payload(root)['active_capture']['stage'],'waiting_motion_target')
            (child/'status.json').write_text(json.dumps(dict(child_status=str(root.parent/'status.json'))))
            self.assertNotIn('active_capture',status_payload(root))
            state['current']=str(root.parent)
            (root/'status.json').write_text(json.dumps(state))
            self.assertNotIn('child_progress',status_payload(root))

    def test_replenishment_progress_distinguishes_finished_from_approved(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            state=dict(stage='replenishing',mode='replenishment',requested=5000,finished=5000,
                initial_accepted=4900,replenished=20,remaining_deficit=80,review=80,
                accepted_candidate=4920,elapsed_sec=36000,replenish_round=1,replenish_round_cap=2)
            (root/'status.json').write_text(json.dumps(state))
            result=status_payload(root)
            self.assertEqual(result['remaining_deficit'],80)
            self.assertEqual(result['replenished'],20)
            self.assertEqual(result['accepted_candidate'],4920)

    def test_plan_before_capture_is_read_only_and_has_denominators(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);capture,manifest=self.fixture(root)
            payload=Dashboard(capture,manifest).payload()
            self.assertEqual(payload['requested'],2)
            self.assertEqual(payload['motion_plan'],dict(stationary=1,animated=1))
            self.assertEqual(payload['family_progress'][0],dict(family='highway',planned=1,accepted=0))
            self.assertFalse(capture.exists());self.assertEqual(payload['history'],[])

    def test_moving_secondary_counts_as_animated_not_static_primary(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);capture,manifest=self.fixture(root)
            meta=root/'metadata.json';meta.write_text(json.dumps(dict(runtime_actors=[dict(motion='static'),dict(motion='walking')])))
            manifest.write_text(json.dumps(dict(scenes=[dict(family='obstacle',motion='static',metadata=str(meta))])))
            self.assertEqual(Dashboard(capture,manifest).payload()['motion_plan'],dict(animated=1))

    def test_recent_rate_uses_active_elapsed_not_offline_wait(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);dashboard=Dashboard(root)
            def status(elapsed,accepted,stage,wall):
                (root/'status.json').write_text(json.dumps(dict(stage=stage,elapsed_sec=elapsed,finished=accepted,accepted_candidate=accepted)))
                return dashboard.payload(wall)
            status(100,10,'running',1000)
            status(100,10,'paused',1100)
            result=status(400,40,'running',10000)
            self.assertEqual(len(result['history']),2)
            self.assertEqual(result['recent_accepted_per_hour'],360.)

    def test_history_restart_restores_and_capture_is_never_written(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);capture,manifest=self.fixture(root);capture.mkdir()
            state=dict(stage='complete',elapsed_sec=100,finished=2,accepted_candidate=2)
            (capture/'status.json').write_text(json.dumps(state));original=(capture/'status.json').read_bytes()
            journal=root/'cache/history.jsonl'
            Dashboard(capture,manifest,history_file=journal).payload(1000)
            with journal.open('a') as f:f.write('torn partial record')
            new=Dashboard(capture,manifest,history_file=journal)
            self.assertEqual(len(new.payload(2000)['history']),1)
            self.assertEqual((capture/'status.json').read_bytes(),original)
            self.assertEqual([p.name for p in capture.iterdir()],['status.json'])
            with self.assertRaises(ValueError):Dashboard(capture,history_file=capture/'history.jsonl')

    def test_counter_rollback_resets_instead_of_negative_rate(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);dashboard=Dashboard(root)
            for elapsed,n,wall in [(100,10,1000),(20,2,2000)]:
                (root/'status.json').write_text(json.dumps(dict(stage='running',elapsed_sec=elapsed,accepted_candidate=n,finished=n)))
                result=dashboard.payload(wall)
            self.assertEqual(len(result['history']),1)
            self.assertIsNone(result['recent_accepted_per_hour'])
            self.assertTrue(result['diagnostic_warnings'])

    def test_deficit_reasons_count_cells_not_repeated_attempts(self):
        cells=[dict(reasons=[dict(reasons=['ground','ground']),dict(review_reasons=['ground','visibility'])]),dict(reasons=['ground'])]
        self.assertEqual(reason_counts(cells),dict(ground=2,visibility=1))

    def test_checkpoint_failure_counts_do_not_claim_live_counts(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);capture,manifest=self.fixture(root);capture.mkdir()
            (capture/'status.json').write_text(json.dumps(dict(stage='running',requested=2,finished=1,accepted_candidate=1,
                accepted_bins={json.dumps(['highway',[10,30],'front']):1})))
            (capture/'quota.json').write_text(json.dumps(dict(target=2,pending=2,cells=[])))
            result=Dashboard(capture,manifest).payload()
            self.assertEqual(result['family_progress'][0]['accepted'],1)
            self.assertEqual(result['motion_checked'],dict(stationary=1))
            self.assertEqual(result['failure_checkpoint_finished'],0)
            self.assertEqual(result['direction_counts'],dict(front=1))

    def test_suite_handles_arbitrary_totals_without_starting_environments(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);capture,manifest=self.fixture(root);suite=root/'suite.json'
            suite.write_text(json.dumps(dict(environments=[dict(environment=str(i),expected_frames=5000,capture_output=str(root/f'env{i}')) for i in range(10)])))
            result=Dashboard(capture,manifest,suite).payload()
            self.assertEqual(result['suite_target'],50000);self.assertEqual(result['suite_accepted'],0)
            self.assertFalse((root/'env0').exists())

    def test_corrupt_status_is_not_reported_as_not_started_or_complete(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'status.json').write_text('not json')
            result=status_payload(root)
            self.assertEqual(result['stage'],'unreadable_status');self.assertTrue(result['error'])

    def test_replenishment_does_not_reuse_ordered_motion_checked_counts(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);capture,manifest=self.fixture(root);capture.mkdir()
            (capture/'status.json').write_text(json.dumps(dict(stage='replenishing',mode='replenishment',requested=2,
                finished=2,accepted_candidate=1,review=1)))
            self.assertIsNone(Dashboard(capture,manifest).payload()['motion_checked'])

    @unittest.skipUnless(shutil.which('google-chrome'),'Optional headless browser not installed')
    def test_browser_renders_graphs_and_escapes_diagnostic_text(self):
        # Isolated UI fixture, never written to a real campaign or dashboard journal.
        payload=dict(stage='running',requested=5000,finished=20,accepted_candidate=19,review=1,
            pending=4980,elapsed_sec=200,eta_sec=100,history=[dict(elapsed_sec=t,accepted=t/10,finished=t/10+1) for t in (100,150,200)],
            rate_history=[dict(elapsed_sec=t,rate=360) for t in (100,150,200)],
            error='<img src=x onerror=alert(1)>',dataset_id='UI_TEST_NOT_COLLECTION')
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                body=HTML.encode() if self.path=='/' else json.dumps(payload).encode()
                self.send_response(200);self.send_header('Content-Type','text/html' if self.path=='/' else 'application/json');self.end_headers();self.wfile.write(body)
            def log_message(self,*args):pass
        class Tags(HTMLParser):
            def __init__(self):super().__init__();self.tags=[]
            def handle_starttag(self,tag,attrs):self.tags.append(tag)
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
        try:
            with tempfile.TemporaryDirectory() as folder:
                result=subprocess.run(['google-chrome','--headless','--no-sandbox','--disable-gpu','--disable-dev-shm-usage',
                    '--user-data-dir='+folder,'--virtual-time-budget=3500','--dump-dom',f'http://127.0.0.1:{server.server_port}'],
                    capture_output=True,text=True,timeout=30,check=True)
            parser=Tags();parser.feed(result.stdout)
            self.assertEqual(parser.tags.count('svg'),2)
            self.assertEqual(parser.tags.count('polyline'),3)
            self.assertNotIn('img',parser.tags)
            self.assertIn('UI_TEST_NOT_COLLECTION',result.stdout)
            self.assertIn('19장',result.stdout)
        finally:server.shutdown();server.server_close();worker.join(timeout=2)


if __name__=='__main__':unittest.main()
