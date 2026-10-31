"""CPU-only cohort identity tests; original frozen conversion APIs are reused."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import borrowed_training_export as original
import cohort_training_export as cohort
import pack_training


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) + '\n')
    return path


def snapshot(*roots):
    return {str(path): path.read_bytes() for root in roots for path in root.rglob('*') if path.is_file()}


def build(ledger, profiles, output, expected_scenes=5000, copy_mode='copy'):
    return copy_mode


@pytest.fixture
def setup(tmp_path, monkeypatch):
    repo = tmp_path / 'repo'
    repo.mkdir()
    monkeypatch.setattr(original, 'LOGICAL_REPO', repo)
    for relative in original.REFERENCE_PATHS:
        write(repo / relative, dict(source=relative))
    expected = {str(repo / name): original.sha(repo / name) for name in original.REFERENCE_PATHS}
    sensor = write(repo / 'configs/sensors/Sensor_competition_local.json', dict(cameras=4))
    reference = write(tmp_path / 'reference.json', dict(helper_sha256=expected,
        sensor_sha256=original.sha(sensor), conversion_adapter_sha256=original.HELPERS['training_convert_adapter.py']))
    final = tmp_path / 'collection/layout_split_final'
    final.mkdir(parents=True)
    rows = [dict(id=f'scene{i}', source_cell_id=f'cell{i}', capture_root=f'/native/{i}',
        scenario_sha256=hashlib.sha256(str(i).encode()).hexdigest(), status='accepted_candidate') for i in range(5000)]
    selected = final / 'derived_selected_ledger.jsonl'
    selected.write_text(''.join(json.dumps(row) + '\n' for row in rows))
    overlay = write(final / 'split_overlay.json', dict(fixture=True))
    manifest = write(final / 'manifest.json', dict(count=5000, config=dict(environment_tag='sunny11'),
        scenes=[dict(id=row['source_cell_id']) for row in rows]))
    receipt = write(final / 'result.json', dict(frames=5000,
        manifest=str(manifest), manifest_sha256=original.sha(manifest),
        derived_selected_ledger=str(selected), derived_selected_ledger_sha256=original.sha(selected),
        split_overlay=str(overlay), split_overlay_sha256=original.sha(overlay)))
    root = tmp_path / 'export'
    argv = ['--repo', str(repo), '--export-root', str(root), '--cohort', 'sunny11',
        '--archive-name', 'base5_sunny11_5k_jpeg90_upright3_261019.tar.zst',
        '--selected', str(selected), '--split-overlay', str(overlay),
        '--sensor', str(sensor), '--expected-helpers', str(reference)]
    monkeypatch.setattr(cohort, 'parent', Mock(return_value=original))
    return SimpleNamespace(repo=repo, root=root, final=final, rows=rows, reference=reference,
        sensor=sensor, manifest=manifest, selected=selected, overlay=overlay, receipt=receipt,
        argv=argv, expected=expected)


def refresh_receipt(setup):
    receipt = original.read(setup.receipt)
    receipt.update(manifest_sha256=original.sha(setup.manifest),
        derived_selected_ledger_sha256=original.sha(setup.selected))
    write(setup.receipt, receipt)


def fake_components(setup, monkeypatch):
    portable = SimpleNamespace(build=build,
        __file__=str(setup.repo / 'perception/detection3d/bevfusion/tools/prepare_base5_portable_dataset.py'))
    monkeypatch.setattr(original.inspect, 'getsourcefile', Mock(return_value=portable.__file__))
    adapter = SimpleNamespace(load_modules=Mock(return_value=(None, SimpleNamespace(portable=portable))))

    def run(cohorts, output, sensor, split_overlay):
        assert set(cohorts) == {'sunny11'} and cohorts['sunny11'] == setup.selected
        assert portable.build('ledger', {}, output, copy_mode='hardlink') == 'copy'
        write(output / 'report.json', dict(fixture='qualified'))
        write(output / 'sunny11/training3_jpeg_portable/manifest.json', dict(fixture='package'))
        return dict(helper_sha256=setup.expected, unique_raw_observations=5000)

    adapter.run = Mock(side_effect=run)
    packer = SimpleNamespace(finish=SimpleNamespace(validate_training_package=Mock()),
        qualification=Mock(), parser=pack_training.parser, pack=Mock(return_value=dict(event='packed')))
    monkeypatch.setattr(original, 'components', Mock(return_value=(adapter, packer)))
    return adapter, packer


def test_original_parent_bytes_are_pinned_and_loaded_without_rewriting():
    loaded = cohort.parent()
    assert cohort.sha(loaded.__file__) == cohort.PARENT_SHA256
    assert loaded.HELPERS == original.HELPERS


def test_plan_keeps_original_input_gates_and_does_not_write_or_mutate_args(setup, monkeypatch):
    loader = Mock()
    monkeypatch.setattr(original, 'components', loader)
    args = cohort.parser().parse_args(setup.argv)
    before = snapshot(setup.repo, setup.final), vars(args).copy(), original.scope
    result = cohort.execute(args)
    assert result['cohort'] == 'sunny11' and result['unchanged_conversion_rules']
    assert not result['files_written'] and not setup.root.exists()
    assert before == (snapshot(setup.repo, setup.final), vars(args), original.scope)
    loader.assert_not_called()


@pytest.mark.parametrize('environment', cohort.ENVIRONMENTS)
def test_all_six_cohorts_use_same_original_scope_and_explicit_identity(setup, environment):
    manifest = original.read(setup.manifest)
    manifest['config']['environment_tag'] = environment
    write(setup.manifest, manifest)
    refresh_receipt(setup)
    args = cohort.parser().parse_args(setup.argv)
    args.cohort = environment
    args.archive_name = f'base5_{environment}_5k_jpeg90_upright3_20261019.tar.zst'
    _, _, expected, contract = cohort.cohort_scope(original, args)
    assert contract['cohort'] == environment and contract['schema'] == cohort.SCHEMA
    assert contract['input_pins'][str(setup.manifest)] == original.sha(setup.manifest)
    assert expected == setup.expected and contract['original_png_deleted'] is False


@pytest.mark.parametrize('change', ['environment', 'manifest_hash', 'manifest_count', 'manifest_ids',
    'manifest_parent', 'rows_count', 'row_status', 'duplicate_id', 'duplicate_root',
    'duplicate_sha', 'duplicate_slot', 'no_newline', 'wrong_archive', 'incomplete_expected',
    'helper_changed', 'sensor_changed', 'overlay_changed'])
def test_wrong_cohort_partial_or_changed_input_is_never_exported(setup, change):
    args = cohort.parser().parse_args(setup.argv)
    manifest = original.read(setup.manifest)
    rows = setup.rows
    if change == 'environment':
        manifest['config']['environment_tag'] = 'sunny13'
    elif change == 'manifest_count':
        manifest['count'] = 4999
    elif change == 'manifest_ids':
        manifest['scenes'][0]['id'] = 'wrong-cell'
    elif change == 'rows_count':
        rows.pop()
    elif change == 'row_status':
        rows[0]['status'] = 'review'
    elif change.startswith('duplicate_'):
        field = dict(duplicate_id='id', duplicate_root='capture_root', duplicate_sha='scenario_sha256',
            duplicate_slot='source_cell_id')[change]
        rows[0][field] = rows[1][field]
    elif change == 'wrong_archive':
        args.archive_name = 'base5_sunny13_5k_jpeg90_upright3_20261019.tar.zst'
    elif change == 'incomplete_expected':
        args.expected_scenes = 4999
    elif change == 'helper_changed':
        write(setup.repo / original.REFERENCE_PATHS[0], dict(changed=True))
    elif change == 'sensor_changed':
        write(setup.sensor, dict(changed=True))
    elif change == 'overlay_changed':
        write(setup.overlay, dict(changed=True))
    write(setup.manifest, manifest)
    setup.selected.write_text(''.join(json.dumps(row) + '\n' for row in rows))
    if change == 'no_newline':
        setup.selected.write_text(setup.selected.read_text().rstrip('\n'))
    refresh_receipt(setup)
    if change in ('manifest_hash', 'manifest_parent'):
        receipt = original.read(setup.receipt)
        if change == 'manifest_hash':
            receipt['manifest_sha256'] = 'changed'
        else:
            other = write(setup.final.parent / 'manifest.json', manifest)
            receipt.update(manifest=str(other), manifest_sha256=original.sha(other))
        write(setup.receipt, receipt)
    frozen_scope = original.scope
    with pytest.raises(ValueError):
        cohort.execute(args)
    assert original.scope is frozen_scope and not setup.root.exists()


def test_qualification_validate_and_pack_call_unchanged_original_apis(setup, monkeypatch):
    adapter, packer = fake_components(setup, monkeypatch)
    before = snapshot(setup.repo, setup.final)
    result = cohort.execute(cohort.parser().parse_args(['qualify', *setup.argv, '--run']))
    assert result['frames'] == 5000 and result['cohort'] == 'sunny11'
    receipt = original.read(setup.root / 'QUALIFIED.json')
    assert receipt['contract']['driver_sha256'] == cohort.sha(cohort.__file__)
    assert receipt['contract']['cohort'] == 'sunny11' and receipt['storage_only_copy'][0]['copy_mode'] == 'copy'
    assert receipt['native_payload_deleted'] is False
    cohort.execute(cohort.parser().parse_args(['validate', *setup.argv]))
    cohort.execute(cohort.parser().parse_args(['pack', *setup.argv, '--run']))
    args = packer.pack.call_args.args[0]
    assert args.package == setup.root / 'qualified/sunny11/training3_jpeg_portable'
    assert args.output.name == 'base5_sunny11_5k_jpeg90_upright3_261019.tar.zst'
    assert args.compression_level == 6 and args.threads == 2
    assert before == snapshot(setup.repo, setup.final)
    adapter.run.assert_called_once()


def test_explicit_cohort_and_archive_identity_are_required():
    with pytest.raises(SystemExit):
        cohort.parser().parse_args([])
