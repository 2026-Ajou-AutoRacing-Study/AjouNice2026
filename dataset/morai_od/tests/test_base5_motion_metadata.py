import copy
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_base5_capture import object_motion_samples
from base5_capture_safety import motion_capture_review,vertical_motion_review


class MotionMetadataTest(unittest.TestCase):
    def message(self):
        return dict(header=dict(seq=0,stamp=dict(secs=1791429106,nsecs=123456789)),
            npc_list=[dict(unique_id=7,position=dict(x=1.,y=2.,z=3.),
                velocity=dict(x=7.92,y=-.4,z=.03),heading=61.3)],
            pedestrian_list=[dict(unique_id=8,position=dict(x=4.,y=5.,z=6.),
                velocity=dict(x=0.,y=0.,z=0.),heading=-45.)],obstacle_list=[])

    def test_raw_velocity_heading_clock_and_legacy_positions(self):
        message=self.message();original=copy.deepcopy(message)
        rows=object_motion_samples(message,42.)
        self.assertEqual(message,original)
        self.assertEqual(len(rows),2)
        self.assertEqual({k:rows[0][k] for k in ('t','uid','x','y','z')},
            dict(t=42.,uid=7,x=1.,y=2.,z=3.))
        self.assertEqual(rows[0]['velocity'],dict(x=7.92,y=-.4,z=.03))
        self.assertEqual(rows[0]['heading'],61.3)
        self.assertEqual(rows[0]['source_seq'],0)
        self.assertEqual(rows[0]['source_stamp_ros'],message['header']['stamp'])
        self.assertAlmostEqual(rows[0]['source_stamp'],1791429106.123456789)
        self.assertEqual(json.loads(json.dumps(rows)),rows)

    def test_missing_advisory_fields_never_fabricate_zero_velocity(self):
        row=object_motion_samples(dict(obstacle_list=[dict(unique_id=9,
            position=dict(x=0,y=0,z=0))]),2.)[0]
        for field in ('velocity','heading','source_seq','source_stamp','source_stamp_ros'):
            self.assertIsNone(row[field],field)
        self.assertEqual(object_motion_samples({},2.),[])

    def test_vectors_and_exact_stamps_are_independent_copies(self):
        message=self.message();rows=object_motion_samples(message,1.)
        rows[0]['velocity']['x']=123
        rows[0]['source_stamp_ros']['secs']=12
        self.assertEqual(message['npc_list'][0]['velocity']['x'],7.92)
        self.assertEqual(rows[1]['source_stamp_ros']['secs'],1791429106)
        self.assertEqual(message['header']['stamp']['secs'],1791429106)

    def test_invalid_stamp_does_not_break_capture_diagnostics(self):
        for stamp in ({},dict(secs='bad',nsecs=0),dict(secs=None,nsecs=0),dict(secs=float('nan'),nsecs=0)):
            message=self.message();message['header']['stamp']=stamp
            self.assertIsNone(object_motion_samples(message,1.)[0]['source_stamp'])

    def test_advisory_velocity_does_not_change_motion_or_z_gates(self):
        actor=dict(unique_id=7,motion='walking',source_list='pedestrianList',
            motion_path=dict(points=[dict(x=0,y=0),dict(x=4,y=0)]))
        rows=[];before=after=None
        for t in (.8,.9,1.):
            message=self.message()
            message['npc_list'][0]['position']=dict(x=t,y=0.,z=3.)
            message['npc_list'][0]['velocity']=dict(x=0.,y=0.,z=0.)
            rows.extend(object_motion_samples(message,t))
            if t==.8:before=message
            after=message
        legacy=[{k:r[k] for k in ('uid','t','x','y','z')} for r in rows]
        old=motion_capture_review(actor,before,after,legacy,1.)
        self.assertEqual(motion_capture_review(actor,before,after,rows,1.),old)
        self.assertFalse(old['review_reasons'])
        self.assertEqual(vertical_motion_review(actor,rows,.8,reference_z=3.),
            vertical_motion_review(actor,legacy,.8,reference_z=3.))


if __name__=='__main__':unittest.main()
