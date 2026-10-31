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


def test_yellow_is_held_with_original_stamp_and_stale_old_source_cannot_hold(rig):
    case, node = rig
    case.step(node, 10.01, 10., raw=[Y])
    wire, result = case.step(node, 10.13, 10.12, observation=dark())
    assert result['final_tuples'] == [list(Y)]
    assert wire.stamp.to_nsec() == 10_000_000_000
    assert case.step(node, 10.22, 10.21, observation=dark())[1]['final_tuples'] == []
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


@pytest.mark.parametrize('lamps,probabilities', (
    ([R], (.5,.01,.01,.01)), ([G], (.01,.01,.5,.01)),
    ([Y], (.01,.5,.01,.01)), ([R,Y], (.99,.5,.01,.01))))
def test_alternating_soft_and_dark_share_one_nonrenewable_budget(rig, lamps, probabilities):
    case, node = rig
    node._resolve_group_id = lambda: 201326
    case.step(node, 10.01, 10., raw=lamps)
    soft = observation_from_probabilities(probabilities)
    assert not soft.known
    for now, source, observation in ((10.08,10.07,soft), (10.14,10.13,dark()),
                                      (10.20,10.19,soft)):
        wire,result=case.step(node,now,source,observation=observation)
        assert result['final_tuples'] == [list(x) for x in lamps]
        assert wire.stamp.to_nsec() == 10_000_000_000
    assert case.step(node,10.22,10.21,observation=dark())[1]['final_tuples'] == []
    assert case.step(node,10.24,10.23,observation=soft)[1]['final_tuples'] == []


@pytest.mark.parametrize('lamps', ([R], [G], [R,Y]))
def test_new_lamp_replaces_held_yellow_immediately(rig, lamps):
    case,node=rig
    node._resolve_group_id = lambda: 201326
    case.step(node,10.01,10.,raw=[Y])
    case.step(node,10.08,10.07,observation=dark())
    wire,result=case.step(node,10.14,10.13,raw=lamps)
    assert result['final_tuples'] == [list(x) for x in lamps]
    assert wire.stamp.to_nsec() == 10_130_000_000
    assert result['observation_timing']['output_origin'] == 'current_observation'


def test_confident_new_red_or_yellow_inside_soft_output_never_keeps_old_green(rig):
    case,node=rig
    for probabilities in ((.99,.5,.01,.01), (.5,.99,.01,.01)):
        node=case.node();node.no_lamps_hold_sec=.2
        case.step(node,10.01,10.,raw=[G])
        soft=observation_from_probabilities(probabilities)
        assert not soft.known
        assert case.step(node,10.10,10.09,observation=soft)[1]['final_tuples'] == []


def test_held_yellow_does_not_restart_native_planner_yellow_budget(tmp_path):
    """Execute the real header, not a Python imitation of planner state."""
    from pathlib import Path
    import subprocess
    root=Path(__file__).resolve().parents[5]
    include=root/'planning_ws/src/autoware_behavior_velocity_traffic_light_module/include'
    source=tmp_path/'yellow.cpp'
    source.write_text('''
#include <cassert>
#include "autoware/behavior_velocity_traffic_light_module/signal_compliance_guard.hpp"
using namespace autoware::behavior_velocity_planner::signal_compliance;
int main() {
  Guard g; g.setRequiredGreenObservations(1);
  assert(!g.observe(Observation::AuthorizedGreen, 10., 10.01, 1.));
  g.observe(Observation::Yellow, 10.1, 10.11, 1.);
  assert(g.yellowTransitionEligible());
  assert(*g.yellowStartLowerBoundSec() == 10.);
  for (double now : {10.18, 10.25}) {
    g.observe(Observation::Yellow, 10.1, now, 1.); // held ORIGINAL source
    auto d=timedYellowDecision(true,g.yellowTransitionEligible(),false,
      10.,0.,1.,5.,now,g.yellowStartLowerBoundSec(),2.,.2);
    assert(std::abs(d.remaining_sec-(2.-(now-10.))) < 1e-9);
    assert(d.allow);
    assert(*g.yellowStartLowerBoundSec() == 10.);
  }
  auto off=timedYellowDecision(false,true,false,10.,0.,1.,5.,10.25,10.,2.,.2);
  assert(!off.allow);
  auto expired=timedYellowDecision(true,true,false,10.,0.,1.,5.,11.9,10.,2.,.2);
  assert(!expired.allow);
  g.observe(Observation::Stop,10.26,10.27,1.);
  assert(!g.yellowTransitionEligible());
}
''')
    binary=tmp_path/'yellow'
    subprocess.run(['g++','-std=c++17','-I'+str(include),str(source),'-o',str(binary)],
                   check=True,timeout=30,capture_output=True)
    subprocess.run([str(binary)],check=True,timeout=10,capture_output=True)
