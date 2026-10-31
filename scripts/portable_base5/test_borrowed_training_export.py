"""CPU-only isolated driver tests; no SSH, simulator, GPU or source deletion."""
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import borrowed_training_export as driver
import pack_training


def build(ledger, profiles, output, expected_scenes=5000, copy_mode='copy'):
    return dict(ledger=ledger, profiles=profiles, output=output,
                expected_scenes=expected_scenes, copy_mode=copy_mode)


def portable():
    return SimpleNamespace(build=build, __file__=__file__)


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def snapshot(path):
    return {str(p):p.read_bytes() for p in path.rglob('*') if p.is_file()}


def test_storage_adapter_changes_only_first_explicit_copy_mode_and_restores():
    module = portable(); qualification = SimpleNamespace(portable=module)
    expected = {str(Path(__file__).resolve()):driver.sha(__file__)}
    with driver.source_readonly_copy(qualification, expected) as calls:
        actual = module.build('ledger', {'512x1056':'infos'}, 'output', copy_mode='hardlink')
        assert actual == dict(ledger='ledger', profiles={'512x1056':'infos'}, output='output',
                              expected_scenes=5000, copy_mode='copy')
        assert len(calls) == 1
        with pytest.raises(ValueError, match='Only one'):
            module.build('ledger', {}, 'second', copy_mode='hardlink')
    assert module.build is build


def test_storage_adapter_restores_after_original_exception():
    module = portable(); qualification = SimpleNamespace(portable=module)
    expected = {str(Path(__file__).resolve()):driver.sha(__file__)}
    with pytest.raises(RuntimeError, match='upstream'):
        with driver.source_readonly_copy(qualification, expected):
            module.build('ledger', {}, 'output', copy_mode='hardlink')
            raise RuntimeError('upstream')
    assert module.build is build


@pytest.mark.parametrize('mode', ['source_sha', 'signature', 'not_used', 'wrong_request'])
def test_storage_adapter_rejects_unreviewed_function_or_unexpected_use(mode):
    module = portable(); qualification = SimpleNamespace(portable=module)
    expected = {str(Path(__file__).resolve()):driver.sha(__file__)}
    if mode == 'source_sha':expected[str(Path(__file__).resolve())] = 'changed'
    if mode == 'signature':module.build = lambda other:other
    with pytest.raises(ValueError):
        with driver.source_readonly_copy(qualification, expected):
            if mode == 'wrong_request':module.build('ledger', {}, 'output', copy_mode='copy')
    assert module.build is build or mode == 'signature'


@pytest.fixture
def setup(tmp_path, monkeypatch):
    repo = tmp_path/'repo'; repo.mkdir()
    monkeypatch.setattr(driver, 'LOGICAL_REPO', repo)
    for relative in driver.REFERENCE_PATHS:
        write(repo/relative, dict(source=relative))
    expected = {str(repo/p):driver.sha(repo/p) for p in driver.REFERENCE_PATHS}
    sensor = write(repo/'configs/sensors/Sensor_competition_local.json', dict(cameras=4))
    reference = write(tmp_path/'runtime/reference.json', dict(helper_sha256=expected,
        sensor_sha256=driver.sha(sensor), conversion_adapter_sha256=driver.HELPERS['training_convert_adapter.py']))
    final = tmp_path/'collections/terminal/layout_split_final'
    selected = write(final/'derived_selected_ledger.jsonl', dict(trusted='fixture'))
    overlay = write(final/'split_overlay.json', dict(split='fixture'))
    result = write(final/'result.json', dict(frames=5000,
        derived_selected_ledger=str(selected), derived_selected_ledger_sha256=driver.sha(selected),
        split_overlay=str(overlay), split_overlay_sha256=driver.sha(overlay)))
    root = tmp_path/'work/export'
    argv = ['--repo', str(repo), '--export-root', str(root), '--selected', str(selected),
        '--split-overlay', str(overlay), '--sensor', str(sensor), '--expected-helpers', str(reference)]
    return SimpleNamespace(repo=repo, root=root, selected=selected, overlay=overlay,
        reference=reference, result=result, sensor=sensor, argv=argv, expected=expected)


def fake_components(setup, monkeypatch):
    module = portable()
    module.__file__ = str(setup.repo/'perception/detection3d/bevfusion/tools/prepare_base5_portable_dataset.py')
    monkeypatch.setattr(driver.inspect, 'getsourcefile', Mock(return_value=module.__file__))
    qualification = SimpleNamespace(portable=module)
    adapter = SimpleNamespace(load_modules=Mock(return_value=(None, qualification)))

    def run(cohorts, output, sensor, split_overlay):
        module.build('native', {}, output/'png_portable', copy_mode='hardlink')
        write(output/'report.json', dict(fake='qualification'))
        write(output/'sunny13/training3_jpeg_portable/manifest.json', dict(fake='package'))
        return dict(helper_sha256=setup.expected, unique_raw_observations=5000)

    adapter.run = Mock(side_effect=run)
    packer = SimpleNamespace(finish=SimpleNamespace(validate_training_package=Mock()),
        qualification=Mock(), parser=pack_training.parser,
        pack=Mock(return_value=dict(event='packed_without_network')))
    monkeypatch.setattr(driver, 'components', Mock(return_value=(adapter, packer)))
    return adapter, packer


def test_plan_is_read_only_without_importing_conversion_modules(setup, monkeypatch):
    loader = Mock(); monkeypatch.setattr(driver, 'components', loader)
    before = snapshot(setup.repo), snapshot(setup.selected.parent), snapshot(setup.reference.parent)
    result = driver.execute(driver.parser().parse_args(setup.argv))
    assert not result['execute'] and not result['files_written'] and not setup.root.exists()
    assert before == (snapshot(setup.repo), snapshot(setup.selected.parent), snapshot(setup.reference.parent))
    loader.assert_not_called()


@pytest.mark.parametrize('kind', ['helper', 'sensor', 'adapter', 'overlay', 'finalized_sha'])
def test_reference_and_finalized_binding_must_match_exactly(setup, kind):
    reference = driver.read(setup.reference)
    if kind == 'helper':write(next(setup.repo.rglob('qualify_base5_training_inputs.py')), dict(changed=True))
    elif kind == 'sensor':write(setup.sensor, dict(changed=True))
    elif kind == 'adapter':
        reference['conversion_adapter_sha256'] = 'changed';write(setup.reference, reference)
    elif kind == 'overlay':write(setup.overlay, dict(changed=True))
    else:
        result = driver.read(setup.result); result['derived_selected_ledger_sha256'] = 'changed'
        write(setup.result, result)
    with pytest.raises(ValueError):driver.scope(driver.parser().parse_args(setup.argv))


def test_qualified_pack_and_validate_use_same_frozen_apis_and_preserve_sources(setup,monkeypatch):
    adapter, packer = fake_components(setup, monkeypatch)
    before = snapshot(setup.repo), snapshot(setup.selected.parent), snapshot(setup.reference.parent)
    (setup.root/'tmp').mkdir(parents=True)
    qualified = driver.execute(driver.parser().parse_args(['qualify', *setup.argv, '--run']))
    assert qualified['frames'] == 5000 and qualified['source_unchanged']
    receipt = driver.read(setup.root/'QUALIFIED.json')
    assert receipt['storage_only_copy'][0]['copy_mode'] == 'copy'
    assert receipt['native_payload_deleted'] is False
    result = driver.execute(driver.parser().parse_args(['validate', *setup.argv]))
    assert not result['files_written']
    result = driver.execute(driver.parser().parse_args(['pack', *setup.argv, '--run']))
    assert result['event'] == 'packed_without_network'
    args = packer.pack.call_args.args[0]
    assert args.output.parent == args.receipt_dir.parent == setup.root
    assert args.package == setup.root/'qualified/sunny13/training3_jpeg_portable'
    assert args.selected == setup.selected and args.split_overlay == setup.overlay
    assert args.compression_level == 6 and args.threads == 2 and args.run
    assert before == (snapshot(setup.repo), snapshot(setup.selected.parent), snapshot(setup.reference.parent))
    assert adapter.run.call_count == 1


def test_qualify_never_overwrites_existing_partial_product(setup,monkeypatch):
    adapter, _ = fake_components(setup, monkeypatch)
    write(setup.root/'INTENT.json', dict(previous='failure'))
    before = snapshot(setup.root)
    with pytest.raises(ValueError, match='Fresh empty export'):
        driver.execute(driver.parser().parse_args(['qualify', *setup.argv, '--run']))
    assert before == snapshot(setup.root)
    adapter.run.assert_not_called()


def test_validate_and_plan_cannot_start_mutating_action(setup):
    for stage in ('plan', 'validate'):
        with pytest.raises(ValueError, match='--run applies'):
            driver.execute(driver.parser().parse_args([stage, *setup.argv, '--run']))
    assert not setup.root.exists()
