"""v97 changes only map identity; route and mission policies remain v96-compatible."""
from pathlib import Path
import copy
import hashlib
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[3]
SHA = 'da22c593a980a8fe02e136a004e1918ccf46a82381024854a4fa344eb992e0d9'


def load(path):
    return yaml.safe_load(path.read_text())


def test_v97_route_changes_map_identity_only():
    old = load(PACKAGE / 'config/kcity_v96_competition_route.yaml')
    new = load(PACKAGE / 'config/kcity_v97_competition_route.yaml')
    expected = copy.deepcopy(old)
    expected['map'].update(version=97, file='kcity_map/lanelet_map/lanelet2_map_v97.osm', sha256=SHA)
    assert new == expected
    params = load(PACKAGE / 'config/competition_route_requester_v97.param.yaml')
    assert params['route_definition_file'] == 'kcity_v97_competition_route.yaml'
    assert params == load(PACKAGE / 'config/competition_route_requester.param.yaml')
    old_params = load(PACKAGE / 'config/competition_route_requester_v96.param.yaml')
    old_params['route_definition_file'] = params['route_definition_file']
    assert params == old_params


def test_manifest_and_map_sha_match_without_publishing_map():
    manifest = load(ROOT / 'configs/map/kcity_v97_speed_limit.yaml')
    assert manifest['map_sha256'] == SHA
    route = load(PACKAGE / 'config/kcity_v97_competition_route.yaml')['map']
    assert route['version'] == manifest['map_version'] == 97
    assert route['file'] == manifest['map_file']
    assert route['sha256'] == SHA
    assert route['projector_file'] == manifest['projector_file']
    path = ROOT / manifest['map_file']
    if not path.exists():
        pytest.skip('v97 OSM is supplied separately')
    assert hashlib.sha256(path.read_bytes()).hexdigest() == SHA
    metadata = next(e for _, e in ET.iterparse(path, events=('start',)) if e.tag == 'MetaInfo')
    assert metadata.get('map_version') == '97'


def test_v97_mission_and_rule_policy_preserves_v96():
    integration = PACKAGE.parent / 'ajounice_planning_integration'
    assert load(integration / 'config/mission_planner_kcity_v97.yaml') == load(
        integration / 'config/mission_planner_kcity_v96.yaml')
    rule_dir = ROOT / 'base_ws/src/competition/competition_rule_monitor/config'
    old = load(rule_dir / 'kcity_v96_rules.yaml')
    expected = dict(old, map_version=97, map_sha256=SHA)
    assert load(rule_dir / 'kcity_v97_rules.yaml') == expected


def test_v97_requested_lanelet_speed_limit():
    path = ROOT / 'kcity_map/lanelet_map/lanelet2_map_v97.osm'
    if not path.exists():
        pytest.skip('v97 OSM is supplied separately')
    relation = next(e for _, e in ET.iterparse(path, events=('end',))
                    if e.tag == 'relation' and e.get('id') == '200968')
    assert {e.get('k'): e.get('v') for e in relation.findall('tag')}['speed_limit'] == '40'
