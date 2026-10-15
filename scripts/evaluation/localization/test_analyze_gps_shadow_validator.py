#!/usr/bin/env python3
"""Focused numerical guard tests; no ROS graph or bag needed."""
import unittest
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import numpy as np
from analyze_gps_shadow_validator import (
    execution_evidence,
    gps_outages, interpolate_segments, nearest_bounded_path,
    phase_masks, recovery_summary, split_oracle, statistics, within_window,
)


class PhaseAnalysisTest(unittest.TestCase):
    def test_zero_gps_invalid_despite_status_one(self):
        gps=np.array([[0,1,37,127,1,0,0], [.1,0,0,0,1,.1,.1],
                      [.2,0,0,0,1,.2,.2], [.3,1,37,127,1,.3,.3]])
        intervals=gps_outages(gps,0,1)
        self.assertEqual(len(intervals),1)
        self.assertEqual(intervals[0]['start'],.1)
        self.assertEqual(intervals[0]['end'],.3)
        self.assertEqual(intervals[0]['reasons'],['raw_invalid'])

    def test_receipt_gap_is_not_invalid_coordinate(self):
        gps=np.array([[0,1,37,127,1,0,0],[2,1,37,127,1,2,2]])
        intervals=gps_outages(gps,0,2)
        self.assertEqual(intervals,[{'start':.5,'end':2.,'reasons':['receipt_stale']}])

    def test_teleport_gap_is_never_interpolated(self):
        ego=np.array([[0,0,0,0,0,0,0,0],[.1,0,0,0,0,0,0,.1],
                      [1,100,0,0,0,0,0,1],[1.1,100,0,0,0,0,0,1.1]])
        segments,events=split_oracle(ego,.25,0)
        oracle, ids=interpolate_segments(segments,[.05,.5,1.05])
        self.assertEqual(len(events),1)
        self.assertEqual(ids.tolist(),[0,-1,1])
        self.assertTrue(np.isnan(oracle[1]).all())
        self.assertAlmostEqual(oracle[2,0],100)

    def test_backwards_header_overlap_is_ambiguous(self):
        a=np.array([[0,0,0,0,0,0,0,0],[1,0,0,0,0,0,0,1]])
        b=np.array([[.5,10,0,0,0,0,0,2],[1.5,10,0,0,0,0,0,3]])
        values,ids=interpolate_segments([a,b],[.25,.75,1.25])
        self.assertEqual(ids.tolist(),[0,-2,1])
        self.assertTrue(np.isnan(values[1]).all())

    def test_path_is_bounded_and_rejects_long_discontinuous_segment(self):
        path=np.array([[0.,0.],[1.,0.]])
        self.assertEqual(nearest_bounded_path(np.array([.5,1]),path),(1.,1.,0.))
        self.assertIsNone(nearest_bounded_path(np.array([10.,0]),path))
        self.assertIsNone(nearest_bounded_path(np.array([5.,0]),np.array([[0.,0.],[10.,0.]])))

    def test_phase_boundary_has_no_double_count(self):
        masks=phase_masks(np.array([0,1,2,2.5,3,4]),[{'start':1,'end':2}],0,5)
        self.assertTrue(np.all(np.sum(list(masks.values()),axis=0)==1))
        self.assertEqual(masks['reacquisition_0_1s'].tolist(),[False,False,True,True,False,False])

    def test_recovery_does_not_bridge_long_missing_samples(self):
        result=recovery_summary(np.array([2.,2.1,5.,5.1]),np.zeros(4),[{'end':2}],hold=1)
        self.assertIsNone(result[0]['recovery_after_reacquisition_sec'])

    def test_signed_and_absolute_mean_are_distinct(self):
        result=statistics([-1,1,np.nan])
        self.assertEqual(result['count'],2)
        self.assertEqual(result['signed']['mean'],0)
        self.assertEqual(result['absolute']['mean'],1)

    def test_driving_window_excludes_park_preparation_and_shutdown(self):
        times=np.array([0,1,2,3,4,5,np.nan])
        mask=within_window(times,2,4)
        self.assertEqual(mask.tolist(),[False,False,True,True,True,False,False])
        phase=phase_masks(times,[{'start':2.5,'end':3.5}],0,6)
        self.assertEqual(sum(np.count_nonzero(p&mask) for p in phase.values()),3)

    def test_nonfinite_first_oracle_sample_is_not_interpolated(self):
        ego=np.array([[0,np.nan,0,0,0,0,0,0],[.1,1,0,0,0,0,0,.1],
                      [.2,2,0,0,0,0,0,.2]])
        segments,events=split_oracle(ego,.25,0)
        reference,ids=interpolate_segments(segments,[0,.05,.15])
        self.assertEqual(events[0]['reason'],'nonfinite_initial_oracle')
        self.assertEqual(ids.tolist(),[-1,-1,0])
        self.assertAlmostEqual(reference[2,0],1.5)

    def test_aborted_drive_stops_at_case_result_not_park_shutdown(self):
        with TemporaryDirectory() as directory:
            bag=Path(directory)/"case.bag"
            messages=[
                {"event":"CONTROLLER_START","time":"2026-10-15T00:00:00+00:00"},
                {"event":"CASE_RESULT","time":"2026-10-15T00:01:00+00:00","reason":"MANUAL_ABORT"},
                {"event":"SAFETY_STOP_CONFIRMED","time":"2026-10-15T00:01:05+00:00"},
            ]
            (bag.parent/"events.jsonl").write_text("\n".join(map(json.dumps,messages)))
            evidence=execution_evidence(bag)
            self.assertEqual(evidence["evaluation_stop_event"],"CASE_RESULT")
            self.assertEqual(evidence["run_finished_stamp"]-evidence["controller_start_stamp"],60)
            self.assertNotIn("status",evidence)

    def test_preparation_failure_is_not_a_drive_terminal_event(self):
        with TemporaryDirectory() as directory:
            bag=Path(directory)/"case.bag"
            messages=[
                {"event":"CASE_RESULT","time":"2026-10-14T23:59:00+00:00"},
                {"event":"CONTROLLER_START","time":"2026-10-15T00:00:00+00:00"},
                {"event":"RUN_FINISHED","time":"2026-10-15T00:01:00+00:00"},
                {"event":"CASE_RESULT","time":"2026-10-15T00:01:02+00:00"},
            ]
            (bag.parent/"events.jsonl").write_text("\n".join(map(json.dumps,messages)))
            evidence=execution_evidence(bag)
            self.assertEqual(evidence["evaluation_stop_event"],"RUN_FINISHED")
            self.assertEqual(evidence["run_finished_stamp"]-evidence["controller_start_stamp"],60)


if __name__=='__main__':
    unittest.main()
