import importlib.util
from pathlib import Path


spec=importlib.util.spec_from_file_location('hesitation_audit',Path(__file__).with_name('audit_traffic_light_hesitation_bag.py'))
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def lamp(color='GREEN',shape='CIRCLE',status='SOLID_ON'):
    return dict(color=color,shape=shape,status=status)


def row(t,phase,group=201351):
    return dict(t=t,phase=phase,group=group)


def test_direction_permission_does_not_treat_d_straight_green_as_left_permission():
    assert module.permission([lamp()],201351)=='GO'
    assert module.permission([lamp()],201326)=='STOP'
    assert module.permission([lamp('RED'),lamp(shape='LEFT_ARROW')],201326)=='GO'


def test_unknown_and_dark_or_unobserved_do_not_create_permission():
    assert module.permission([],201351)=='UNKNOWN'
    assert module.permission([lamp('UNKNOWN')],201351)=='UNKNOWN'
    assert module.permission([lamp(status='SOLID_OFF')],201351)=='UNKNOWN'


def test_yellow_is_not_relabelled_as_go():
    assert module.permission([lamp('AMBER')],201351)=='YELLOW'
    assert module.permission([lamp('RED'),lamp('AMBER')],201326)=='YELLOW'


def test_observed_tuple_labels_are_separate_from_synthetic_final_red():
    assert module.permission(module.tuple_lamps([]),201326)=='UNKNOWN'
    assert module.permission(module.tuple_lamps([['red','circle',None]]),201326)=='STOP'
    assert module.permission(module.tuple_lamps([['red','circle',None],['green','arrow',True]]),201326)=='GO'


def test_unknown_gap_is_bracketed_only_by_same_mission_go():
    result=module.unknown_gaps([row(0,'GO'),row(.1,'UNKNOWN'),row(.2,'UNKNOWN'),row(.3,'GO')])
    assert len(result)==1
    assert result[0]['frames']==2
    assert result[0]['bracketed_by_go']
    assert result[0]['go_to_go_receipt_gap_sec']==.3


def test_transport_gap_and_red_after_unknown_are_not_short_go_to_go_dropouts():
    result=module.unknown_gaps([row(0,'GO'),row(1,'UNKNOWN'),row(1.1,'GO'),
                              row(1.2,'UNKNOWN'),row(1.3,'STOP')])
    assert len(result)==2
    assert not any(r['bracketed_by_go'] for r in result)


def test_nonmission_and_other_group_go_do_not_bracket_a_mission_unknown():
    result=module.unknown_gaps([row(0,'GO',201365),row(.1,'UNKNOWN'),row(.2,'GO',201365),row(.3,'UNKNOWN',0)])
    assert len(result)==1
    assert not result[0]['bracketed_by_go']


def test_nearest_speed_requires_fresh_receipt_and_handles_edges():
    rows=[dict(t=0,speed=1),dict(t=1,speed=2)]
    assert module.closest(rows,.9)['speed']==2
    assert module.closest(rows,.5) is None
    assert module.closest([],0) is None


def test_stationary_brake_holding_and_missing_speed_are_not_moving_episodes():
    brakes=[dict(t=0,brake=.5),dict(t=.1,brake=.6),dict(t=3,brake=.5)]
    assert module.brake_episodes(brakes,[dict(t=0,speed=0),dict(t=.1,speed=0)])==[]
    assert module.brake_episodes(brakes,[dict(t=0,speed=float('nan'))])==[]
    assert module.brake_episodes([dict(t=0,brake=float('nan'))],[dict(t=0,speed=1)])==[]


def test_moving_brake_episodes_have_explicit_gap_threshold():
    brakes=[dict(t=t,brake=.5) for t in (0,.1,.8,.9)]
    speeds=[dict(t=t,speed=1) for t in (0,.1,.8,.9)]
    result=module.brake_episodes(brakes,speeds)
    assert len(result)==2
    assert result[0]['frames']==result[1]['frames']==2
