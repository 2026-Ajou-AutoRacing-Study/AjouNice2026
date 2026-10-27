"""Real callback: brief dark crop, original timestamps and immediate new lamps."""
import copy
import pytest

import test_traffic_light_direct_node_handoff as rig_fixture
from test_traffic_light_direct_node_handoff import R, Y, G, L
from traffic_light_cnn_classifier import observation_from_probabilities
from traffic_light_observation_handoff import no_lamps_context
from signal_smoother import SignalSmoother


@pytest.fixture
def rig():
    case = rig_fixture.RealPolicyNodeHandoffTest()
    case.setUpClass()
    node = case.node()
    node.no_lamps_hold_sec = .2
    return case, node


def dark():
    return observation_from_probabilities((.01, .01, .01, .01))


@pytest.mark.parametrize('group,lamps', [(201351,[G]), (201365,[G]),
                                       (201290,[G,L]), (201326,[R,L])])
def test_same_housing_gap_keeps_source_then_sustained_darkness_expires(rig, group, lamps):
    case, node = rig
    node._resolve_group_id = lambda: group
    case.step(node, 10.01, 10., raw=lamps)
    for now, source in ((10.13,10.12), (10.21,10.20)):
        wire, result = case.step(node, now, source, observation=dark())
        assert result['no_lamps_hold']['eligible']
        assert result['final_tuples'] == [list(x) for x in lamps]
        assert result['observation_timing']['output_origin'] == 'no_lamps_hold'
        assert wire.stamp.to_nsec() == 10_000_000_000
    _, result = case.step(node, 10.210001, 10.200001, observation=dark())
    assert result['final_tuples'] == []
    _, result = case.step(node, 10.25, 10.24, observation=dark())
    assert result['final_tuples'] == []


@pytest.mark.parametrize('lamps', ([R], [Y], [R,Y]))
def test_new_red_yellow_replaces_held_permission_on_first_frame(rig, lamps):
    case, node = rig
    node._resolve_group_id = lambda: 201326
    case.step(node, 10.01, 10., raw=[R,L])
    case.step(node, 10.13, 10.12, observation=dark())
    wire, result = case.step(node, 10.16, 10.15, raw=lamps)
    assert result['final_tuples'] == [list(x) for x in lamps]
    assert not result['observation_timing']['output_is_held']
    assert wire.stamp.to_nsec() == 10_150_000_000


@pytest.mark.parametrize('failure', ('no_prior', 'other_head', 'missing_map', 'route_change', 'switch_off'))
def test_missing_identity_first_observation_and_zero_switch_do_not_inherit_green(rig, failure):
    case, node = rig
    if failure != 'no_prior': case.step(node, 10.01, 10.)
    kw = {}
    if failure == 'other_head': kw['head'] = 'other'
    if failure == 'missing_map': kw['ready'] = False
    if failure == 'route_change': node._resolve_group_id = lambda: 201365
    if failure == 'switch_off': node.no_lamps_hold_sec = 0.
    _, result = case.step(node, 10.13, 10.12, observation=dark(), **kw)
    assert result['final_tuples'] == []


def test_yellow_is_not_extended_and_stale_old_source_cannot_hold(rig):
    case, node = rig
    case.step(node, 10.01, 10., raw=[Y])
    assert case.step(node, 10.13, 10.12, observation=dark())[1]['final_tuples'] == []
    node = case.node(); node.no_lamps_hold_sec = .2
    case.step(node, 10.31, 10., raw=[G])
    assert case.step(node, 10.35, 10.34, observation=dark())[1]['final_tuples'] == []


def test_unrelated_unknown_clears_cache_instead_of_resurrecting_permission(rig):
    case, node = rig
    case.step(node, 10.01, 10.)
    case.step(node, 10.10, 10.09, unknown=True)
    assert case.step(node, 10.13, 10.12, observation=dark())[1]['final_tuples'] == []


def test_no_lamps_context_requires_current_normal_map_identity_and_exact_dark_bits():
    packet = dict(status='ready', group_id=201365, source_stamp_ns=10_000_000_000,
        generated_stamp_ns=10_010_000_000,
        rois=[dict(eligible=True, group_id=201365, head_id='B')])
    selection = dict(status='matched', reason='map_head_association', head_id='B')
    classifier = dark().to_dict()
    def eligible(p=packet, s=selection, c=classifier, stamp=10_000_000_000, now=10_050_000_000):
        return no_lamps_context(p,s,c,'B',201365,stamp,now)['eligible']
    assert eligible()
    assert not eligible(stamp=10_000_000_001)
    assert not eligible(now=11_000_000_000)
    assert not eligible(p=dict(packet, status='missing'))
    assert not eligible(s=dict(selection, reason='visual_continuation_match'))
    assert not eligible(c=dict(classifier, bits=dict(red=0,yellow=1,green=0,left_arrow=0)))
    assert not eligible(c=dict(classifier, bits=dict(red=False,yellow=0,green=0,left_arrow=0)))
    changed = copy.deepcopy(packet); changed['rois'][0]['head_id'] = 'other'
    assert not eligible(p=changed)


def test_full_phase_majority_delays_new_stop_but_no_lamps_retention_does_not(rig):
    case, node = rig
    voter = SignalSmoother(7,4)
    for i in range(7):
        source = 10.+i*.1
        case.step(node,source+.01,source,raw=[G])
        voter.update([G])
    for i in range(3):
        source=10.7+i*.1
        result=case.step(node,source+.01,source,raw=[Y])[1]
        assert result['final_tuples'] == [list(Y)]
        assert voter.update([Y]) == [G]
    assert voter.update([Y]) == [Y]
