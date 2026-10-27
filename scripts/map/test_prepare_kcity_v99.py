"""Only temporary fixture assets; no downloads, ROS or installed workspace writes."""
import hashlib
from pathlib import Path

import pytest
import yaml

import prepare_kcity_v99 as tool


@pytest.fixture
def asset(tmp_path):
    content = b'<osm><MetaInfo map_version="99"/></osm>'
    source = tmp_path / 'separately_supplied.osm'
    source.write_bytes(content)
    manifest = tmp_path / tool.MANIFEST
    manifest.parent.mkdir(parents=True)
    manifest.write_text(yaml.safe_dump(dict(map_version=99,
        map_file='kcity_map/lanelet_map/lanelet2_map_v99.osm',
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
    assert not (root / 'kcity_map/lanelet_map/lanelet2_map_v99.osm').exists()


def test_existing_map_is_never_replaced(asset):
    root, source = asset
    target = tool.prepare(root=root, source=source)
    target.write_bytes(b'existing different asset')
    with pytest.raises(ValueError, match='SHA mismatch'):
        tool.prepare(root=root, source=source)
    assert target.read_bytes() == b'existing different asset'


@pytest.mark.parametrize('present', [False, True])
def test_clean_build_rejects_missing_or_bad_v99_before_deleting_install(asset, present):
    import shutil
    import subprocess
    root, _ = asset
    source_root = Path(__file__).resolve().parents[2]
    for relative in ('scripts/workspace/build_all.sh', 'scripts/map/prepare_kcity_v99.py'):
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source_root / relative, target)
    sentinel = root / 'base_ws/install_aw149/preserve_me'
    sentinel.parent.mkdir(parents=True)
    sentinel.write_text('existing build')
    if present:
        (root / 'kcity_map/lanelet_map').mkdir(parents=True)
        (root / 'kcity_map/lanelet_map/lanelet2_map_v99.osm').write_bytes(b'wrong asset')
    result = subprocess.run(['bash', str(root / 'scripts/workspace/build_all.sh'), '--clean'],
                            capture_output=True, text=True)
    assert result.returncode != 0
    assert sentinel.read_text() == 'existing build'
    assert 'v99' in result.stderr
