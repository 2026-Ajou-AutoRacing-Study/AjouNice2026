import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_environment_rgb_pilot as pilot


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))
    return path


@pytest.fixture
def source(tmp_path, monkeypatch):
    ident = 'sunny13_scene_00001'
    scenario = write(tmp_path/'source.json', dict(unit=True))
    meta = write(tmp_path/'metadata.json', dict(runtime_actors=[dict(motion='static')], split='train'))
    root = tmp_path/'raw'; root.mkdir()
    (root/'scenario.json').write_bytes(scenario.read_bytes())
    for name in ('metadata.json', 'telemetry.json', 'sensor_reference.json', 'audit.json'):
        write(root/name, {})
    entry = dict(id=ident, scenario=str(scenario), metadata=str(meta), split='train',
        scenario_sha256=pilot.sha256(scenario), derived_layout_group_id='layout:original')
    deck = dict(config=dict(environment_tag='sunny13', **{pilot.shadow.KEY:dict(unused=True)}),
        count=1, scenes=[entry], bulk_ready=True)
    baseline = dict(id=ident, scenario_sha256=entry['scenario_sha256'],
        status='accepted_candidate', capture_root=str(root))
    ledger = tmp_path/'ledger.jsonl'; ledger.write_text(json.dumps(baseline)+'\n')
    manifest = write(tmp_path/'manifest.json', deck)
    monkeypatch.setattr(pilot.layout, 'validate_manifest_overlay', lambda *a, **k: None)
    return manifest, ledger, deck, baseline, meta


def test_fresh_diagnostic_preserves_geometry_metadata_split_and_source_bytes(source, tmp_path):
    manifest, ledger, deck, baseline, meta = source
    before = {p: p.read_bytes() for p in (manifest, ledger, meta)}
    result = pilot.prepare(manifest, ledger, tmp_path/'new', 'sunny15', 'sunny15', ('00001',))
    candidate = pilot.read(result); registration = pilot.read(result.with_name('registration.json'))
    assert candidate['scenes'] == deck['scenes']
    assert candidate['config']['environment_tag'] == 'sunny13'
    assert candidate['config'][pilot.shadow.KEY] == deck['config'][pilot.shadow.KEY]
    assert candidate['environment_rgb_pilot']['physical_capture_environment'] == 'sunny15'
    assert candidate['bulk_ready'] is False
    assert registration['independent_training_samples'] == 0
    assert registration['source_environment'] == 'sunny13'
    assert registration['capture_environment']['hour'] == 15
    assert all(p.read_bytes() == data for p, data in before.items())
    assert len(registration['pairs'][0]['baseline_files']) == 5
    with pytest.raises(FileExistsError):
        pilot.prepare(manifest, ledger, tmp_path/'new', 'sunny15', 'sunny15', ('00001',))


@pytest.mark.parametrize('motion', ['walking', 'running', 'unknown'])
def test_moving_or_unknown_baseline_cannot_be_misused_as_static(source, motion):
    _, _, deck, baseline, meta = source
    write(meta, dict(runtime_actors=[dict(motion=motion)]))
    with pytest.raises(ValueError, match='정적'):
        pilot.select(deck, [baseline], ('00001',))


def test_registration_tagged_shadow_is_not_silently_rebound(source):
    _, _, deck, baseline, meta = source
    value = pilot.read(meta); value[pilot.shadow.KEY] = dict(prepared=True); write(meta, value)
    with pytest.raises(ValueError, match='정적'):
        pilot.select(deck, [baseline], ('00001',))


@pytest.mark.parametrize('ids', [(), ('00001', '00001'), ('99999',)])
def test_missing_or_duplicate_ids_fail_before_output(source, ids):
    _, _, deck, baseline, _ = source
    with pytest.raises(ValueError):
        pilot.select(deck, [baseline], ids)


def test_payload_mismatch_cannot_be_called_same_geometry(source):
    _, _, deck, baseline, _ = source
    write(Path(baseline['capture_root'])/'scenario.json', dict(changed=True))
    with pytest.raises(ValueError, match='payload'):
        pilot.select(deck, [baseline], ('00001',))


def test_unconfirmed_environment_fails_without_writing(source, tmp_path):
    manifest, ledger, *_ = source
    with pytest.raises(ValueError, match='일치'):
        pilot.prepare(manifest, ledger, tmp_path/'new', 'sunny15', 'sunny13', ('00001',))
    assert not (tmp_path/'new').exists()


def test_symlink_parent_cannot_write_into_baseline_capture(source, tmp_path):
    manifest, ledger, _, baseline, _ = source
    alias = tmp_path/'alias'; alias.symlink_to(baseline['capture_root'], target_is_directory=True)
    with pytest.raises(ValueError, match='capture 내부'):
        pilot.prepare(manifest, ledger, alias/'new', 'sunny15', 'sunny15', ('00001',))
    assert not (Path(baseline['capture_root'])/'new').exists()
