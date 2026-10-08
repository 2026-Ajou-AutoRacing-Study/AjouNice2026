import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from base5_capture_dashboard import status_payload


class DashboardTests(unittest.TestCase):
    def test_not_started_and_disk(self):
        with tempfile.TemporaryDirectory() as folder:
            result=status_payload(Path(folder)/'future')
            self.assertEqual(result['stage'],'not_started')
            self.assertGreater(result['disk_free_gib'],0)
            self.assertFalse(result['status_stale'])
            self.assertEqual(result['eta_label'],'수집 전')
            self.assertFalse((Path(folder)/'future').exists())

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


if __name__=='__main__':unittest.main()
