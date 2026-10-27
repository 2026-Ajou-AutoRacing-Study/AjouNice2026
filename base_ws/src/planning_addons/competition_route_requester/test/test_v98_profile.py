"""v98 route profile changes only map identity; route and mission policies remain v97-compatible."""
from pathlib import Path
import copy
import hashlib
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[3]
SHA = '25087b24cf9c2a65f366b33b885bcc07ff644ad7b241d8f671a24a03460dc7d0'


def load(path):
    return yaml.safe_load(path.read_text())


def test_v98_route_changes_map_identity_only():
    old = load(PACKAGE / 'config/kcity_v97_competition_route.yaml')
    new = load(PACKAGE / 'config/kcity_v98_competition_route.yaml')
    expected = copy.deepcopy(old)
    expected['map'].update(version=98, file='kcity_map/lanelet_map/lanelet2_map_v98.osm', sha256=SHA)
    assert new == expected
    params = load(PACKAGE / 'config/competition_route_requester_v98.param.yaml')
    assert params['route_definition_file'] == 'kcity_v98_competition_route.yaml'
    assert params == load(PACKAGE / 'config/competition_route_requester.param.yaml')
    old_params = load(PACKAGE / 'config/competition_route_requester_v97.param.yaml')
    old_params['route_definition_file'] = params['route_definition_file']
    assert params == old_params


def test_manifest_and_map_sha_match_without_publishing_map():
    manifest = load(ROOT / 'configs/map/kcity_v98_centerline_connection.yaml')
    assert manifest['map_sha256'] == SHA
    route = load(PACKAGE / 'config/kcity_v98_competition_route.yaml')['map']
    assert route['version'] == manifest['map_version'] == 98
    assert route['file'] == manifest['map_file']
    assert route['sha256'] == SHA
    assert route['projector_file'] == manifest['projector_file']
    path = ROOT / manifest['map_file']
    if not path.exists():
        pytest.skip('v98 OSM is supplied separately')
    assert hashlib.sha256(path.read_bytes()).hexdigest() == SHA
    metadata = next(e for _, e in ET.iterparse(path, events=('start',)) if e.tag == 'MetaInfo')
    assert metadata.get('map_version') == '98'


def test_v98_mission_and_rule_policy_preserves_v97():
    integration = PACKAGE.parent / 'ajounice_planning_integration'
    assert load(integration / 'config/mission_planner_kcity_v98.yaml') == load(
        integration / 'config/mission_planner_kcity_v97.yaml')
    rule_dir = ROOT / 'base_ws/src/competition/competition_rule_monitor/config'
    old = load(rule_dir / 'kcity_v97_rules.yaml')
    expected = dict(old, map_version=98, map_sha256=SHA)
    assert load(rule_dir / 'kcity_v98_rules.yaml') == expected


def test_v98_requested_lanelet_speed_limit():
    path = ROOT / 'kcity_map/lanelet_map/lanelet2_map_v98.osm'
    if not path.exists():
        pytest.skip('v98 OSM is supplied separately')
    relation = next(e for _, e in ET.iterparse(path, events=('end',))
                    if e.tag == 'relation' and e.get('id') == '200968')
    assert {e.get('k'): e.get('v') for e in relation.findall('tag')}['speed_limit'] == '40'


def test_v98_only_changes_approved_centerline_and_metadata():
    paths = [ROOT / f'kcity_map/lanelet_map/lanelet2_map_v{v}.osm' for v in (97, 98)]
    if not all(p.exists() for p in paths):
        pytest.skip('Provision v97 and v98 for full asset comparison')
    old, new = (ET.parse(p).getroot() for p in paths)
    def layer(root, kind):
        return {e.get('id'): ET.tostring(e) for e in root.findall(kind)}
    assert layer(old, 'node') == layer(new, 'node')
    assert layer(old, 'relation') == layer(new, 'relation')
    before, after = layer(old, 'way'), layer(new, 'way')
    assert set(before) == set(after)
    assert [key for key in before if before[key] != after[key]] == ['105522']
    assert after['105522'] == before['105522'].replace(b'ref="229565"', b'ref="233927"')
    way = next(w for w in new.findall('way') if w.get('id') == '234039')
    assert way.find('nd').get('ref') == '233927'
    assert new.find('MetaInfo').get('map_version') == '98'
