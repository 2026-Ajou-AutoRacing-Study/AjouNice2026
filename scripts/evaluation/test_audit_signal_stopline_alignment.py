from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from audit_signal_stopline_alignment import alignment_verdict, same_group_geometry, stable_window_summary


def test_short_restop_or_insufficient_observations_do_not_qualify():
    assert stable_window_summary([], dict(start_stamp_ns=0,end_stamp_ns=2_000_000_000,
                                         duration_sec=2.)) is None
    assert stable_window_summary([], dict(start_stamp_ns=0,end_stamp_ns=10_000_000_000,
                                         duration_sec=10.)) is None


def test_stable_median_excludes_braking_and_departure_frames():
    frames=[dict(stamp=i*100_000_000,top=40. if i<20 or i>95 else 94.,head_id='A')
            for i in range(101)]
    result=stable_window_summary(frames,dict(start_stamp_ns=0,end_stamp_ns=10_000_000_000,
                                           duration_sec=10.))
    assert result['median_top_px']==94.
    assert result['minimum_top_px']==94.


def test_all_four_within_five_pixels_required():
    rows=[dict(mission=m,median_top_px=v) for m,v in zip('ABCD',[93.,94.,95.,96.])]
    assert alignment_verdict(rows)['status']=='aligned'
    rows[-1]['median_top_px']=99.
    assert alignment_verdict(rows)['status']=='adjustment_needed'
    rows[-1]['median_top_px']=None
    assert alignment_verdict(rows)['status']=='insufficient_stable_stop_evidence'
    assert alignment_verdict(rows[:3])['status']=='insufficient_stable_stop_evidence'


def test_one_mission_with_disagreeing_stops_cannot_hide_behind_average():
    rows=[dict(mission=m,median_top_px=95.) for m in 'ABCD']
    rows[1]['between_stop_median_range_px']=[85.,105.]
    assert alignment_verdict(rows)['status']=='adjustment_needed'


def test_compare_one_physical_head_not_a_mixture_of_C016_C018():
    frames=[dict(stamp=i*100_000_000,top=94.,head_id='C016') for i in range(20,40)]
    frames += [dict(stamp=i*100_000_000,top=200.,head_id='C018') for i in range(20,60)]
    window=dict(start_stamp_ns=0,end_stamp_ns=10_000_000_000,duration_sec=10.)
    result=stable_window_summary(frames,window,'C016')
    assert result['median_top_px']==94.
    assert result['heads']=={'C016':20}
    assert stable_window_summary(frames,window,'missing_head') is None


def test_cross_candidate_reuse_requires_exact_selected_mission_geometry():
    source=dict(frame='map',groups=[dict(group_id=201351,line=[1,2]),dict(group_id=201290,line=[3,4])])
    target=deepcopy(source)
    target['groups'][1]['line']=[5,6]
    assert same_group_geometry(source,target,'A')
    assert not same_group_geometry(source,target,'C')
    target['frame']='other'
    assert not same_group_geometry(source,target,'A')


def test_selected_manifest_meets_the_same_five_pixel_rule():
    root=Path(__file__).resolve().parents[2]
    cfg=yaml.safe_load((root/'configs/map/kcity_v93_stopline_candidate.yaml').read_text())
    values=list(cfg['stable_median_top_px'].values())
    assert max(values)-min(values) <= cfg['alignment_criterion']['maximum_between_mission_spread_px']==5.
    assert cfg['measured_spread_px']==pytest.approx(max(values)-min(values),abs=.001)
    assert cfg['all_four_pixel_alignment_verified']
    assert not cfg['runtime_default_promoted']
    assert not cfg['signal_selection_continuity_fixed']
