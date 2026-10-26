"""Pure optional original-paint route geometry contracts."""
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from signal_judging_line_geometry import parse_judging_lines, route_judging_progress


def valid():
    return dict(regulatory_id=201365,line_xy=[17.,-3.,17.,3.],approach_xy=[1.,0.])


def test_absent_and_malformed_optional_profile_do_not_raise():
    assert parse_judging_lines(None)==({},[])
    assert parse_judging_lines({})[0]=={}
    assert parse_judging_lines({})[1]
    assert parse_judging_lines([dict(valid(),line_xy=[0.,0.])])[0]=={}


def test_duplicate_or_invalid_direction_never_selects_one_flattering_row():
    assert parse_judging_lines([valid(),valid()])[0]=={}
    for direction in ([0.,0.],[0.,1.],['1',0.],[True,0.]):
        assert parse_judging_lines([dict(valid(),approach_xy=direction)])[0]=={}


def test_crossing_is_on_successor_lane_not_clamped_to_virtual_lane_end():
    line=parse_judging_lines([valid()])[0][201365]
    route=[(0,0.,10.,[(1,[(0.,0.),(10.,0.)],10.)]),
           (1,10.,10.,[(2,[(10.,0.),(20.,0.)],10.)])]
    assert route_judging_progress(line,route,10.)==17.


def test_finite_line_and_route_direction_must_match():
    line=parse_judging_lines([valid()])[0][201365]
    assert route_judging_progress(line,[(0,0.,20.,[(1,[(0.,8.),(20.,8.)],20.)])],10.) is None
    assert route_judging_progress(line,[(0,0.,20.,[(1,[(20.,0.),(0.,0.)],20.)])],10.) is None


def test_route_alternatives_use_normalized_preferred_metric():
    line=parse_judging_lines([valid()])[0][201365]
    route=[(0,0.,10.,[(1,[(0.,0.),(10.,0.)],10.)]),
           (1,10.,10.,[(2,[(10.,0.),(30.,0.)],20.)])]
    assert route_judging_progress(line,route,10.)==13.5
