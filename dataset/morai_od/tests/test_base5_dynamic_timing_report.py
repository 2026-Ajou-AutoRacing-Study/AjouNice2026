import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from report_base5_dynamic_timing import report


class TimingReportTest(unittest.TestCase):
    def test_pivot_travel_and_review_not_approval(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);capture=root/'capture';capture.mkdir()
            meta=dict(runtime_actors=[dict(unique_id=8500,name='test',motion='walking',position=dict(x=1,y=2))])
            telemetry=dict(objects_before=dict(pedestrian_list=[dict(unique_id=8500,position=dict(x=4,y=6))]),
                primary_displacement_during_wait_m=.06)
            row=dict(status='review',id='test',capture_root=str(capture),distance_bounds=[3,5],
                audit=dict(components=[dict(parent_uid=8500,range_ego_xy_m=8,lidar_points=1,cameras=[],
                    positive_lidar1_camera0=True)],review_reasons=['primary_actual_distance_bin_mismatch']))
            for filename,data in [('metadata.json',meta),('telemetry.json',telemetry)]:
                (capture/filename).write_text(json.dumps(data))
            (root/'ledger.jsonl').write_text(json.dumps(row)+'\n')
            result=report(root)
            self.assertEqual(result['reviews'][0]['primary_pivot_travel_before_capture_m'],5.)
            self.assertIn('BULK_CAPTURE_BLOCKED',result['status'])
