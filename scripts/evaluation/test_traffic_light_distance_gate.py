import sys
from pathlib import Path
import pytest

SCRIPTS = Path(__file__).resolve().parents[2]/'base_ws/src/perception/traffic_light_detection/scripts'
sys.path.insert(0, str(SCRIPTS))
from traffic_light_observation_handoff import ObservationDistanceGate


def selected(depth, stamp=1, group=201351):
    return dict(status='matched', source_stamp_ns=stamp, group_id=group,
                roi=dict(group_id=group, depth_m=depth))


def test_disabled_is_a_noop():
    assert not ObservationDistanceGate().update(0, 0, {})['suppressed']
    assert not ObservationDistanceGate(0,30).update(201326,1,selected(50,1,201326))['suppressed']


@pytest.mark.parametrize('group', [201351,201365,201290])
def test_abc40_d30_mission_thresholds(group):
    gate=ObservationDistanceGate(40,30)
    assert gate.update(group,1,selected(40.01,1,group))['suppressed']
    assert gate.update(group,2,selected(40,2,group))['just_opened']
    assert gate.update(201326,3,selected(35,3,201326))['suppressed']
    assert gate.update(201326,4,selected(30.01,4,201326))['suppressed']
    state=gate.update(201326,5,selected(30,5,201326))
    assert state['just_opened'] and state['threshold_m']==30
    assert not gate.update(201326,6,selected(31,6,201326))['suppressed']


@pytest.mark.parametrize('value',[-1,121,float('nan'),float('inf'),True])
def test_invalid_d_configuration(value):
    with pytest.raises(ValueError): ObservationDistanceGate(40,value)


def test_exact_boundary_opens_and_distance_or_missing_does_not_close():
    gate=ObservationDistanceGate(40)
    assert gate.update(201351,1,selected(40.01))['suppressed']
    assert gate.update(201351,2,selected(40,2))['just_opened']
    assert not gate.update(201351,3,selected(41,3))['suppressed']
    assert not gate.update(201351,4,{})['suppressed']


def test_group_change_and_clock_rewind_reset():
    gate=ObservationDistanceGate(40)
    gate.update(201351,10,selected(30,10))
    assert gate.update(201365,11,selected(60,11,201365))['suppressed']
    gate.update(201365,12,selected(30,12,201365))
    assert gate.update(201365,9,selected(60,9,201365))['suppressed']


@pytest.mark.parametrize('depth',[None,0,-1,float('nan'),float('inf'),True])
def test_invalid_geometry_cannot_activate(depth):
    assert ObservationDistanceGate(40).update(201351,1,selected(depth))['suppressed']


def test_wrong_group_or_stamp_does_not_activate():
    assert ObservationDistanceGate(40).update(201351,2,selected(20,1))['suppressed']
    assert ObservationDistanceGate(40).update(201351,1,selected(20,1,201365))['suppressed']


@pytest.mark.parametrize('value',[-1,121,float('nan'),float('inf'),True])
def test_invalid_configuration(value):
    with pytest.raises(ValueError): ObservationDistanceGate(value)
