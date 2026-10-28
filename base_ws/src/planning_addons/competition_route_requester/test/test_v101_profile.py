"""v101 map identity and the direct 201439 -> 200360 route contract."""
from pathlib import Path
import copy
import hashlib
import xml.etree.ElementTree as ET

import unittest
import yaml
from competition_route_requester import policy

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[3]
SHA = '574d0db7ba7aa2fbca55253e1ed467bdb4bfb2afeea78dd0b760227d47edeb29'


def load(path):
    return yaml.safe_load(path.read_text())


def test_v101_route_preserves_v99_except_map_and_bottleneck_preferences():
    old = load(PACKAGE / 'config/kcity_v99_competition_route.yaml')
    new = load(PACKAGE / 'config/kcity_v101_competition_route.yaml')
    expected = copy.deepcopy(old)
    expected['map'].update(version=101, file='kcity_map/lanelet_map/lanelet2_map_v101.osm', sha256=SHA)
    for key in ('route_a', 'route_b'):
        expected[key]['preferred_lanelet_overrides'].update({201441: 201439, 201446: 201439})
    assert new == expected
    params = load(PACKAGE / 'config/competition_route_requester_v101.param.yaml')
    assert params['route_definition_file'] == 'kcity_v101_competition_route.yaml'
    assert params == load(PACKAGE / 'config/competition_route_requester.param.yaml')
    old_params = load(PACKAGE / 'config/competition_route_requester_v99.param.yaml')
    old_params['route_definition_file'] = params['route_definition_file']
    assert params == old_params


def test_alternative_bottleneck_branches_use_201439_then_200360():
    routes = load(PACKAGE / 'config/kcity_v101_competition_route.yaml')
    definition = routes['route_b']
    goal = routes['poses'][definition['goal_ref']]
    for branch in (201441, 201446):
        # A noncanonical upstream branch exercises the RouteSelector fallback,
        # rather than the exact frozen sequence mapping.
        observed = [201456, 201438, 201468, 200138, 201444,
                    201470, branch, 200185, 201448, 201451, 200088]
        expected = observed[:6] + [201439, 200360, 201479] + observed[9:]
        assert policy.preferred_sequence(routes, 'route_b', observed) == expected
        for sequence, accepted in ((observed, False), (expected, True),
                                   (observed[:7] + expected[7:], False)):
            assert policy.route_matches(
                routes, 'route_b', definition['uuid'], sequence,
                (goal['x'], goal['y']), .5) is accepted


def test_all_routes_and_waypoints_keep_direct_bottleneck_exit():
    routes = load(PACKAGE / 'config/kcity_v101_competition_route.yaml')
    for key in ('route_a', 'route_b', 'fallback_route'):
        definition = routes[key]
        sequence = definition['expected_path_lanelet_ids']
        index = sequence.index(201439)
        assert sequence[index:index + 2] == [201439, 200360]
        if key != 'fallback_route':
            refs = definition['waypoint_refs']
            index = refs.index('wp_201473')
            assert routes['poses'][refs[index]]['lanelet_id'] == 201439
            assert routes['poses'][refs[index + 1]]['lanelet_id'] == 200360


def test_manifest_and_map_sha_match_without_publishing_map():
    manifest = load(ROOT / 'configs/map/kcity_v101_shadow_shoulders.yaml')
    assert manifest['map_sha256'] == SHA
    route = load(PACKAGE / 'config/kcity_v101_competition_route.yaml')['map']
    assert route['version'] == manifest['map_version'] == 101
    assert route['file'] == manifest['map_file']
    assert route['sha256'] == SHA
    assert route['projector_file'] == manifest['projector_file']
    path = ROOT / manifest['map_file']
    if not path.exists():
        raise unittest.SkipTest('Provision v101 with scripts/map/prepare_kcity_v101.py')
    assert hashlib.sha256(path.read_bytes()).hexdigest() == SHA
    metadata = next(e for _, e in ET.iterparse(path, events=('start',)) if e.tag == 'MetaInfo')
    assert metadata.get('map_version') == '101'


def test_v101_mission_excludes_parallel_exit_and_preserves_other_v99_rules():
    integration = PACKAGE.parent / 'ajounice_planning_integration'
    expected_mission = load(integration / 'config/mission_planner_kcity_v99.yaml')
    overrides = expected_mission['route_section_primitive_overrides']
    overrides['200360'] = [200360]
    overrides['201479'] = [201479]
    del overrides['200185'], overrides['201448']
    expected_mission['route_section_excluded_lanelet_ids'][:0] = [200185, 201448]
    mission = load(integration / 'config/mission_planner_kcity_v101.yaml')
    assert mission == expected_mission
    excluded = set(mission['route_section_excluded_lanelet_ids'])
    for candidates in mission['route_section_primitive_overrides'].values():
        assert not excluded.intersection(candidates)
    routes = load(PACKAGE / 'config/kcity_v101_competition_route.yaml')
    for key in ('route_a', 'route_b', 'fallback_route'):
        assert not excluded.intersection(routes[key]['expected_path_lanelet_ids'])
    rule_dir = ROOT / 'base_ws/src/competition/competition_rule_monitor/config'
    old = load(rule_dir / 'kcity_v99_rules.yaml')
    expected = dict(old, map_version=101, map_sha256=SHA)
    assert load(rule_dir / 'kcity_v101_rules.yaml') == expected


def test_v101_requested_lanelet_speed_limit():
    path = ROOT / 'kcity_map/lanelet_map/lanelet2_map_v101.osm'
    if not path.exists():
        raise unittest.SkipTest('Provision v101 with scripts/map/prepare_kcity_v101.py')
    relation = next(e for _, e in ET.iterparse(path, events=('end',))
                    if e.tag == 'relation' and e.get('id') == '201096')
    assert {e.get('k'): e.get('v') for e in relation.findall('tag')}['speed_limit'] == '30'


def test_v101_adds_shared_boundary_shoulders_without_changing_road_relations():
    path = ROOT / 'kcity_map/lanelet_map/lanelet2_map_v101.osm'
    if not path.exists():
        raise unittest.SkipTest('Provision v101 for geometry contract checks')
    root = ET.parse(path).getroot()
    nodes = {n.get('id') for n in root.findall('node')}
    ways = {w.get('id'): w for w in root.findall('way')}
    relations = {r.get('id'): r for r in root.findall('relation')}
    def tags(element):
        return {t.get('k'): t.get('v') for t in element.findall('tag')}
    shoulders = {i: r for i, r in relations.items() if tags(r).get('subtype') == 'road_shoulder'}
    assert set(shoulders) == {'234976', '235226'}
    for rid, road_id, seam, role in [('234976', '200374', '100237', 'right'),
                                     ('235226', '200471', '100228', 'left')]:
        members = {m.get('role'): m.get('ref') for m in shoulders[rid].findall('member')}
        assert members[role] == seam
        assert any(m.get('ref') == seam for m in relations[road_id].findall('member'))
        assert tags(ways[seam]) == {'type': 'line_thin', 'subtype': 'solid', 'color': 'white'}
        outer = members['left' if role == 'right' else 'right']
        assert tags(ways[outer])['type'] == 'road_border'
        assert all(nd.get('ref') in nodes for key in members.values()
                   for nd in ways[key].findall('nd'))
    old_path = ROOT / 'kcity_map/lanelet_map/lanelet2_map_v99.osm'
    if old_path.exists():
        old = ET.parse(old_path).getroot()
        def semantic(e):
            return (e.tag, sorted(e.attrib.items()), tuple(semantic(c) for c in e))
        for relation in old.findall('relation'):
            assert semantic(relation) == semantic(relations[relation.get('id')])
