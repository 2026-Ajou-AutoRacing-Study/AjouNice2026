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


if __name__=='__main__':unittest.main()
