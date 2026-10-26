from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from audit_signal_stopline_standard import CRITERIA, check_report


def report():
    return dict(mission='A', group_id=201351, front_bumper_offset_m=3.845,
        approach_detector_top_margin_px=dict(min=81., p50=110., max=220.),
        stationary_windows=[dict(duration_sec=10.,
            stopped_detector_top_margin_px=dict(min=81., p50=95., max=99.),
            front_bumper_gap_to_map_line_m=dict(min=-.1, max=.2),
            selection_status=dict(matched=20, missing=3))])


def test_one_criterion_preserves_missing_observations():
    result = check_report(report(), 'A')
    assert result['status'] == 'geometry_pass'
    assert result['eligible_stops'][0]['selection_status']['missing'] == 3


@pytest.mark.parametrize('change', ['short', 'no_stop', 'no_box', 'low_top', 'early_stop', 'approach'])
def test_missing_or_bad_evidence_is_not_a_pass(change):
    r = deepcopy(report())
    w = r['stationary_windows'][0]
    if change == 'short':
        w['duration_sec'] = 2.
    elif change == 'no_stop':
        r['stationary_windows'] = []
    elif change == 'no_box':
        w['stopped_detector_top_margin_px'] = None
    elif change == 'low_top':
        w['stopped_detector_top_margin_px']['min'] = 79.
    elif change == 'early_stop':
        w['front_bumper_gap_to_map_line_m']['max'] = 2.
    else:
        r['approach_detector_top_margin_px']['min'] = 79.
    assert check_report(r, 'A')['status'] != 'geometry_pass'


def test_vehicle_reference_and_group_must_match():
    r = report()
    r['front_bumper_offset_m'] = 3.
    with pytest.raises(ValueError):
        check_report(r, 'A')
    with pytest.raises(ValueError):
        check_report(report(), 'B')


def test_documented_candidate_uses_the_same_numeric_criterion():
    root = Path(__file__).resolve().parents[2]
    config = yaml.safe_load((root/'configs/map/kcity_v91_stopline_candidate.yaml').read_text())
    common = config['common_visibility_criterion']
    assert common['image_size'] == CRITERIA['image_size']
    assert common['stop_duration_min_sec'] == CRITERIA['minimum_stop_duration_sec']
    assert common['stopped_median_top_band_px'] == CRITERIA['median_top_range_px']
    assert common['observed_stop_and_approach_min_top_px'] == CRITERIA['minimum_observed_top_px']
    assert common['maximum_abs_bumper_to_map_line_gap_m'] == CRITERIA['maximum_abs_line_gap_m']
    assert common['common_margin_m'] == config['stop_margin_m'] == 0.
    assert not common['signal_selection_continuity_fixed']
    assert not config['runtime_default_promoted']
