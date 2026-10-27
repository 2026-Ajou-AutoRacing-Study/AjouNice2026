"""v95 changes only map identity; route and mission policies remain v94-compatible."""
from pathlib import Path
import copy
import hashlib
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[3]
SHA = 'a9d931eca722ab81596bce0685147b337dd7fdc3320ebeae13d0ff03ef2f2b6e'


def load(path):
    return yaml.safe_load(path.read_text())


def test_v95_route_changes_map_identity_only():
    old = load(PACKAGE / 'config/kcity_v94_competition_route.yaml')
    new = load(PACKAGE / 'config/kcity_v95_competition_route.yaml')
    expected = copy.deepcopy(old)
    expected['map'].update(version=95, file='kcity_map/lanelet_map/lanelet2_map_v95.osm', sha256=SHA)
    assert new == expected
    params = load(PACKAGE / 'config/competition_route_requester_v95.param.yaml')
    assert params['route_definition_file'] == 'kcity_v95_competition_route.yaml'
    assert params == load(PACKAGE / 'config/competition_route_requester.param.yaml')
    old_params = load(PACKAGE / 'config/competition_route_requester_v94.param.yaml')
    old_params['route_definition_file'] = params['route_definition_file']
    assert params == old_params


def test_manifest_and_map_sha_match_without_publishing_map():
    manifest = load(ROOT / 'configs/map/kcity_v95_centerline_asset.yaml')
    assert manifest['map_sha256'] == SHA
    route = load(PACKAGE / 'config/kcity_v95_competition_route.yaml')['map']
    assert route['version'] == manifest['map_version'] == 95
    assert route['file'] == manifest['map_file']
    assert route['sha256'] == SHA
    assert route['projector_file'] == manifest['projector_file']
    path = ROOT / manifest['map_file']
    if not path.exists():
        pytest.skip('v95 OSM is supplied separately')
    assert hashlib.sha256(path.read_bytes()).hexdigest() == SHA
    metadata = next(e for _, e in ET.iterparse(path, events=('start',)) if e.tag == 'MetaInfo')
    assert metadata.get('map_version') == '95'
