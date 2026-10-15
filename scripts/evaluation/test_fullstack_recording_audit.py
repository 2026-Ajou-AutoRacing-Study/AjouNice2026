import sys
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import re
import subprocess
import gc

sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_fullstack_recording import intervals, run_window, stamp_key
from profile_recorded_map_prediction import observe_gc


class AuditContractTest(unittest.TestCase):
    def message(self, seq=0, frame='map', ids=(1,), stamp=123):
        return NS(header=NS(seq=seq,frame_id=frame,stamp=NS(to_nsec=lambda:stamp)),
                  objects=[NS(object_id=NS(uuid=bytes([i]*16))) for i in ids])

    def test_publisher_seq_is_not_input_identity(self):
        self.assertEqual(stamp_key(self.message(1251)),stamp_key(self.message(1296)))

    def test_frame_and_ids_guard_matching(self):
        base=stamp_key(self.message())
        for msg in (self.message(frame='lidar'),self.message(ids=(2,)),self.message(stamp=124)):
            self.assertNotEqual(base,stamp_key(msg))

    def test_ordered_ids_not_accidentally_sorted(self):
        self.assertNotEqual(stamp_key(self.message(ids=(1,2))),stamp_key(self.message(ids=(2,1))))

    def test_empty_message_key_is_valid(self):
        self.assertEqual(stamp_key(self.message(ids=())),(123,'map',()))

    def test_empty_cadence(self):
        self.assertIsNone(intervals([])['hz'])

    def test_arrival_cadence_keeps_duplicates(self):
        result=intervals([1.,1.,1.1,1.2,1.8])
        self.assertEqual(result['messages'],5)
        self.assertEqual(result['gaps_gt05'],1)
        self.assertAlmostEqual(result['hz'],5.)

    def test_controller_window_not_inferred_from_speed(self):
        self.assertIsNone(run_window([]))
        value=run_window([dict(event='CONTROLLER_START',time='2026-10-13T16:23:35+00:00'),
                          dict(event='RUN_FINISHED',time='2026-10-14T01:26:17+09:00')])
        self.assertEqual(value[1]-value[0],162.)

    def test_recorder_covers_both_namespaces_and_planner_costs(self):
        script=Path(__file__).resolve().parents[1]/'rosbag/record/rosbag_for_perception_latency_eval.sh'
        result=subprocess.run(['bash',str(script),'--list'],check=True,text=True,capture_output=True)
        expressions=[re.compile(p) for p in result.stdout.splitlines()]
        for topic in ('/perception/bevfusion/status','/perception/base5_trial/status',
                      '/perception/base5_trial/latency_ms',
                      '/perception/object_recognition/tracking/measurement_objects',
                      '/planning/scenario_planning/lane_driving/motion_planning/path_optimizer/debug/processing_time_ms',
                      '/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/debug/obstacle_cruise/processing_time_ms'):
            self.assertTrue(any(p.fullmatch(topic) for p in expressions),topic)
        self.assertFalse(any(p.fullmatch('/sensor/lidar1') for p in expressions))
        self.assertFalse(any(p.fullmatch('/sensor/camera/front/compressed') for p in expressions))

    def test_recorder_rejects_invalid_duration_before_ros_start(self):
        script=Path(__file__).resolve().parents[1]/'rosbag/record/rosbag_for_perception_latency_eval.sh'
        for duration in ('0','-1','601','invalid'):
            result=subprocess.run(['bash',str(script),'--duration',duration,'--list'],text=True,capture_output=True)
            self.assertEqual(result.returncode,2)

    def test_gc_probe_restores_callbacks_even_on_error(self):
        before=list(gc.callbacks)
        with self.assertRaises(ValueError):
            with observe_gc() as pauses:
                gc.collect()
                self.assertTrue(pauses)
                self.assertEqual(pauses[-1]['generation'],2)
                raise ValueError('diagnostic failure')
        self.assertEqual(gc.callbacks,before)


if __name__=='__main__':unittest.main()
