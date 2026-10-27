"""v99 route profile changes only map identity; route and mission policies remain v98-compatible."""
from pathlib import Path
import copy
import hashlib
import xml.etree.ElementTree as ET

import unittest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[3]
SHA = 'e9d28ebad8b5c1ed7b876c18702923e44b75a53d7f3db4e856eafe6ef756db30'


def load(path):
    return yaml.safe_load(path.read_text())


def test_v99_route_changes_map_identity_only():
    old = load(PACKAGE / 'config/kcity_v98_competition_route.yaml')
    new = load(PACKAGE / 'config/kcity_v99_competition_route.yaml')
    expected = copy.deepcopy(old)
    expected['map'].update(version=99, file='kcity_map/lanelet_map/lanelet2_map_v99.osm', sha256=SHA)
    assert new == expected
    params = load(PACKAGE / 'config/competition_route_requester_v99.param.yaml')
    assert params['route_definition_file'] == 'kcity_v99_competition_route.yaml'
    assert params == load(PACKAGE / 'config/competition_route_requester.param.yaml')
    old_params = load(PACKAGE / 'config/competition_route_requester_v98.param.yaml')
    old_params['route_definition_file'] = params['route_definition_file']
    assert params == old_params


def test_manifest_and_map_sha_match_without_publishing_map():
    manifest = load(ROOT / 'configs/map/kcity_v99_speed_limit.yaml')
    assert manifest['map_sha256'] == SHA
    route = load(PACKAGE / 'config/kcity_v99_competition_route.yaml')['map']
    assert route['version'] == manifest['map_version'] == 99
    assert route['file'] == manifest['map_file']
    assert route['sha256'] == SHA
    assert route['projector_file'] == manifest['projector_file']
    path = ROOT / manifest['map_file']
    if not path.exists():
        raise unittest.SkipTest('Provision v99 with scripts/map/prepare_kcity_v99.py')
    assert hashlib.sha256(path.read_bytes()).hexdigest() == SHA
    metadata = next(e for _, e in ET.iterparse(path, events=('start',)) if e.tag == 'MetaInfo')
    assert metadata.get('map_version') == '99'


def test_v99_mission_and_rule_policy_preserves_v98():
    integration = PACKAGE.parent / 'ajounice_planning_integration'
    assert load(integration / 'config/mission_planner_kcity_v99.yaml') == load(
        integration / 'config/mission_planner_kcity_v98.yaml')
    rule_dir = ROOT / 'base_ws/src/competition/competition_rule_monitor/config'
    old = load(rule_dir / 'kcity_v98_rules.yaml')
    expected = dict(old, map_version=99, map_sha256=SHA)
    assert load(rule_dir / 'kcity_v99_rules.yaml') == expected


def test_v99_requested_lanelet_speed_limit():
    path = ROOT / 'kcity_map/lanelet_map/lanelet2_map_v99.osm'
    if not path.exists():
        raise unittest.SkipTest('Provision v99 with scripts/map/prepare_kcity_v99.py')
    relation = next(e for _, e in ET.iterparse(path, events=('end',))
                    if e.tag == 'relation' and e.get('id') == '201096')
    assert {e.get('k'): e.get('v') for e in relation.findall('tag')}['speed_limit'] == '30'


def test_v99_changes_only_speed_limit_and_map_metadata():
    paths = [ROOT / f'kcity_map/lanelet_map/lanelet2_map_v{v}.osm' for v in (98, 99)]
    if not all(p.exists() for p in paths):
        raise unittest.SkipTest('Provision v98 and v99 for asset comparison')
    old, new = (ET.parse(p).getroot() for p in paths)
    def layer(root, kind):
        return {e.get('id'): ET.tostring(e) for e in root.findall(kind)}
    for kind in ('node', 'way'):
        assert layer(old, kind) == layer(new, kind)
    before, after = layer(old, 'relation'), layer(new, 'relation')
    assert set(before) == set(after)
    assert [key for key in before if before[key] != after[key]] == ['201096']
    relation = next(e for e in new.findall('relation') if e.get('id') == '201096')
    limit = next(e for e in relation.findall('tag') if e.get('k') == 'speed_limit')
    assert limit.get('v') == '30'
    limit.set('v', '59')
    assert ET.tostring(relation) == before['201096']
    assert new.find('MetaInfo').get('map_version') == '99'
