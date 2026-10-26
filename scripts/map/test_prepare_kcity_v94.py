import hashlib
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


SPEC = importlib.util.spec_from_file_location(
    'prepare_kcity_v94', Path(__file__).with_name('prepare_kcity_v94.py'))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
PAYLOAD = b'pinned v94 map fixture\n'


@pytest.fixture
def asset(tmp_path):
    manifest = tmp_path / MODULE.MANIFEST
    manifest.parent.mkdir(parents=True)
    manifest.write_text(yaml.safe_dump(dict(
        map_version=94, map_file=MODULE.MAP_FILE, drive_path=MODULE.MAP_FILE,
        map_sha256=hashlib.sha256(PAYLOAD).hexdigest(),
        drive_root_folder_id='private-folder-id', drive_file_id='pinned-file-id')))
    return tmp_path, tmp_path / MODULE.MAP_FILE


def no_network(*args, **kwargs):
    pytest.fail('Existing/check/offline operations must never invoke rclone')


def assert_no_temporary(target):
    assert not list(target.parent.glob('.prepare-v94-*'))


@pytest.mark.parametrize('check', [False, True])
def test_existing_verified_asset_never_uses_network(asset, check):
    root, target = asset
    target.parent.mkdir(parents=True)
    target.write_bytes(PAYLOAD)
    assert MODULE.prepare(root=root, check=check, run=no_network) == target
    assert target.read_bytes() == PAYLOAD
    assert_no_temporary(target)


@pytest.mark.parametrize('check', [False, True])
def test_existing_wrong_sha_is_never_overwritten(asset, check):
    root, target = asset
    target.parent.mkdir(parents=True)
    target.write_bytes(b'important existing map')
    with pytest.raises(ValueError, match='never overwritten'):
        MODULE.prepare(root=root, check=check, run=no_network)
    assert target.read_bytes() == b'important existing map'
    assert_no_temporary(target)


def test_check_missing_asset_is_read_only(asset):
    root, target = asset
    with pytest.raises(FileNotFoundError, match='v94 map missing'):
        MODULE.prepare(root=root, check=True, run=no_network)
    assert not target.parent.exists()


@pytest.mark.parametrize('correct', [True, False])
def test_offline_source_must_match_pinned_sha(asset, correct):
    root, target = asset
    source = root / 'offline.osm'
    source.write_bytes(PAYLOAD if correct else b'corrupt offline asset')
    if correct:
        assert MODULE.prepare(root=root, source=source, run=no_network) == target
        assert target.read_bytes() == PAYLOAD
    else:
        with pytest.raises(ValueError, match='SHA-256 mismatch'):
            MODULE.prepare(root=root, source=source, run=no_network)
        assert not target.exists()
    assert source.exists()
    assert_no_temporary(target)


@pytest.mark.parametrize('explicit,environment,expected', [
    (None, None, 'ajou_drive_private'),
    (None, 'teammate_private', 'teammate_private'),
    ('explicit_private', 'teammate_private', 'explicit_private'),
])
def test_download_is_single_file_with_explicit_root_and_remote(asset, monkeypatch,
                                                            explicit, environment, expected):
    root, target = asset
    if environment is None:
        monkeypatch.delenv('KCITY_MAP_RCLONE_REMOTE', raising=False)
    else:
        monkeypatch.setenv('KCITY_MAP_RCLONE_REMOTE', environment)
    calls = []

    def download(command, **options):
        calls.append(command)
        assert command[:3] == ['rclone', 'copyto', f'{expected}:{MODULE.MAP_FILE}']
        temporary = Path(command[3])
        assert temporary.parent == target.parent
        assert command[4:] == ['--drive-root-folder-id', 'private-folder-id']
        assert options == dict(stdout=MODULE.subprocess.PIPE, stderr=MODULE.subprocess.PIPE,
                               text=True, check=False)
        temporary.write_bytes(PAYLOAD)
        return SimpleNamespace(returncode=0)

    MODULE.prepare(root=root, remote=explicit, run=download)
    assert len(calls) == 1
    assert target.read_bytes() == PAYLOAD
    assert_no_temporary(target)


@pytest.mark.parametrize('exit_code', [1, 0])
def test_download_failure_or_corrupt_partial_never_publishes(asset, exit_code):
    root, target = asset

    def download(command, **options):
        Path(command[3]).write_bytes(b'partial/corrupt bytes')
        return SimpleNamespace(returncode=exit_code, stderr='SECRET_TOKEN_NOT_TO_PRINT')

    with pytest.raises((RuntimeError, ValueError)) as failure:
        MODULE.prepare(root=root, remote='private_test', run=download)
    assert 'SECRET_TOKEN' not in str(failure.value)
    if exit_code:
        assert 'rclone config reconnect private_test:' in str(failure.value)
        assert 'no public/shared-client fallback' in str(failure.value)
    assert not target.exists()
    assert_no_temporary(target)


def test_missing_rclone_explains_offline_alternative(asset):
    root, target = asset

    def missing(*args, **kwargs):
        raise FileNotFoundError('rclone')

    with pytest.raises(RuntimeError, match='use --source FILE'):
        MODULE.prepare(root=root, run=missing)
    assert not target.exists()
    assert_no_temporary(target)


@pytest.mark.parametrize('same', [False, True])
def test_racing_existing_file_is_preserved_and_verified(asset, monkeypatch, same):
    root, target = asset
    source = root / 'offline.osm'
    source.write_bytes(PAYLOAD)
    winner = PAYLOAD if same else b'other writer must survive'

    def racing_link(temporary, destination):
        destination.write_bytes(winner)
        raise FileExistsError(destination)

    monkeypatch.setattr(MODULE.os, 'link', racing_link)
    if same:
        assert MODULE.prepare(root=root, source=source, run=no_network) == target
    else:
        with pytest.raises(ValueError, match='never overwritten'):
            MODULE.prepare(root=root, source=source, run=no_network)
    assert target.read_bytes() == winner
    assert_no_temporary(target)


def test_dangling_symlink_is_existing_and_never_replaced(asset):
    root, target = asset
    target.parent.mkdir(parents=True)
    target.symlink_to(root / 'absent-map.osm')
    with pytest.raises(ValueError, match='never overwritten'):
        MODULE.prepare(root=root, run=no_network)
    assert target.is_symlink()
    assert_no_temporary(target)


def test_remote_is_a_configuration_name_not_a_url_or_path(asset):
    root, target = asset
    with pytest.raises(ValueError, match='explicit rclone configuration name'):
        MODULE.prepare(root=root, remote='https://public.example/file', run=no_network)
    assert not target.parent.exists()


@pytest.mark.parametrize('field', ['drive_root_folder_id', 'drive_file_id'])
@pytest.mark.parametrize('value', [None, False, 123])
def test_malformed_manifest_identity_is_rejected_before_any_write(asset, field, value):
    root, target = asset
    manifest_path = root / MODULE.MANIFEST
    manifest = yaml.safe_load(manifest_path.read_text())
    manifest[field] = value
    manifest_path.write_text(yaml.safe_dump(manifest))
    with pytest.raises(ValueError, match=field):
        MODULE.prepare(root=root, run=no_network)
    assert not target.parent.exists()
