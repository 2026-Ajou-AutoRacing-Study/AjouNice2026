"""Verify reproducible derivation and immutable asset handling using small fixtures."""
import hashlib
import pytest
import yaml
import prepare_kcity_v98 as tool


@pytest.fixture
def asset(tmp_path):
    data = b'<osm><MetaInfo map_version="97"/>\n  <way id="105522"><nd ref="229565"/></way></osm>'
    sha = hashlib.sha256(data).hexdigest()
    new = tool.derive(data, sha)
    manifest = tmp_path / tool.MANIFEST
    manifest.parent.mkdir(parents=True)
    source = tmp_path / 'kcity_map/lanelet_map/lanelet2_map_v97.osm'
    source.parent.mkdir(parents=True)
    source.write_bytes(data)
    manifest.write_text(yaml.safe_dump(dict(map_version=98,
        map_file='kcity_map/lanelet_map/lanelet2_map_v98.osm',
        map_sha256=hashlib.sha256(new).hexdigest(),
        source=dict(map_file=str(source.relative_to(tmp_path)), sha256=sha))))
    return tmp_path, source, new


def test_derives_from_pinned_local_v97_and_accepts_supplied_v98(asset, tmp_path):
    root, source, new = asset
    target = tool.prepare(root=root)
    assert target.read_bytes() == new
    assert tool.prepare(root=root, check=True) == target
    target.unlink()
    supplied = root / 'supplied_v98.osm'
    supplied.write_bytes(new)
    assert tool.prepare(root=root, source=supplied).read_bytes() == new


def test_check_does_not_create_asset_and_bad_source_is_rejected(asset):
    root, source, _ = asset
    with pytest.raises(FileNotFoundError, match='Provision'):
        tool.prepare(root=root, check=True)
    source.write_bytes(b'wrong source')
    with pytest.raises(ValueError, match='source SHA mismatch'):
        tool.prepare(root=root)
    assert not (root / 'kcity_map/lanelet_map/lanelet2_map_v98.osm').exists()


def test_never_overwrites_existing_different_map(asset):
    root, _, _ = asset
    target = tool.prepare(root=root)
    target.write_bytes(b'existing different bytes')
    with pytest.raises(ValueError, match='SHA mismatch'):
        tool.prepare(root=root)
    assert target.read_bytes() == b'existing different bytes'
