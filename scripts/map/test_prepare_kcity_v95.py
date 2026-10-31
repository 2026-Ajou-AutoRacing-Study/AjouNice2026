"""Only temporary fixture assets; no downloads, ROS or installed workspace writes."""
import hashlib
from pathlib import Path

import pytest
import yaml

import prepare_kcity_v95 as tool


@pytest.fixture
def asset(tmp_path):
    content = b'<osm><MetaInfo map_version="95"/></osm>'
    source = tmp_path / 'separately_supplied.osm'
    source.write_bytes(content)
    manifest = tmp_path / tool.MANIFEST
    manifest.parent.mkdir(parents=True)
    manifest.write_text(yaml.safe_dump(dict(map_version=95,
        map_file='kcity_map/lanelet_map/lanelet2_map_v95.osm',
        map_sha256=hashlib.sha256(content).hexdigest())))
    return tmp_path, source


def test_missing_asset_requires_separate_supply_then_verifies(asset):
    root, source = asset
    with pytest.raises(FileNotFoundError, match='distributed separately'):
        tool.prepare(root=root)
    target = tool.prepare(root=root, source=source)
    assert target.read_bytes() == source.read_bytes()
    assert tool.prepare(root=root) == target


def test_rejects_bad_source_before_creating_target(asset):
    root, source = asset
    source.write_bytes(b'wrong map')
    with pytest.raises(ValueError, match='SHA mismatch'):
        tool.prepare(root=root, source=source)
    assert not (root / 'kcity_map/lanelet_map/lanelet2_map_v95.osm').exists()


def test_existing_map_is_never_replaced(asset):
    root, source = asset
    target = tool.prepare(root=root, source=source)
    target.write_bytes(b'existing different asset')
    with pytest.raises(ValueError, match='SHA mismatch'):
        tool.prepare(root=root, source=source)
    assert target.read_bytes() == b'existing different asset'
