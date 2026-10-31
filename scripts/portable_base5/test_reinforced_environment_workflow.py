import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('reinforced_environment_workflow', HERE/'reinforced_environment_workflow.py')
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


@pytest.mark.parametrize('environment', adapter.ENVIRONMENTS)
def test_explicit_environment(environment):
    actual, argv = adapter.parse_environment(['--environment', environment, 'register', '--output-root', '/tmp/new'])
    assert actual == environment
    assert argv == ['register', '--output-root', '/tmp/new']


@pytest.mark.parametrize('argv', [[], ['--environment', 'rainy13'], ['--environment=foggy15'],
    ['--environment', 'foggy15', '--environment', 'sunny11']])
def test_ambiguous_environment_rejected(argv):
    with pytest.raises((RuntimeError, SystemExit)):
        adapter.parse_environment(argv)


def test_worker_environment_is_only_provisional(tmp_path):
    intent = tmp_path/'intent.json'
    intent.write_text(json.dumps(dict(environment='foggy15')))
    argv = ['--worker', '/tmp/frozen.py', '--intent', str(intent), '--intent-sha256', '0'*64]
    assert adapter.parse_environment(argv) == ('foggy15', argv)
    intent.write_text(json.dumps(dict(environment='rainy13')))
    with pytest.raises(RuntimeError):
        adapter.parse_environment(argv)


def test_original_body_hash_checked_first(monkeypatch):
    monkeypatch.setattr(adapter, 'sha', lambda _: '0'*64)
    with pytest.raises(RuntimeError, match='bytes changed'):
        adapter.load_workflow('foggy15')


def test_same_body_new_environment_no_capture():
    original = adapter.TEMPLATE.read_bytes()
    workflow = adapter.load_workflow('foggy15')
    assert workflow.ENVIRONMENT == 'foggy15'
    assert workflow.INTENT_SCHEMA == adapter.INTENT_SCHEMA
    assert workflow.__file__ == str(HERE/'reinforced_environment_workflow.py')
    assert workflow.DEFAULT_CAPTURE.name == 'collection_5k_v1'
    assert 'foggy15' in str(workflow.DEFAULT_OUTPUT)
    assert adapter.TEMPLATE.read_bytes() == original


def test_intent_adds_template_pin_without_changing_native_scope(monkeypatch):
    # Replace only the loader in a fake import to exercise the adaptation seam.
    body = SimpleNamespace(intent_for=lambda *a, **k: dict(pins={'original': 'x'}, limit=32, bulk_authorized=False),
        verify_intent=lambda filename, expected: json.loads(Path(filename).read_text()), save=lambda p, d: d)
    spec = SimpleNamespace(loader=SimpleNamespace(exec_module=lambda module: None))
    monkeypatch.setattr(adapter.importlib.util, 'spec_from_file_location', lambda *a: spec)
    monkeypatch.setattr(adapter.importlib.util, 'module_from_spec', lambda *a: body)
    workflow = adapter.load_workflow('foggy15')
    result = workflow.intent_for(None)
    assert result['pins']['original'] == 'x'
    assert result['pins'][str(adapter.TEMPLATE)] == adapter.TEMPLATE_SHA256
    assert result['limit'] == 32 and result['bulk_authorized'] is False


def test_worker_cannot_omit_or_change_template_authority(monkeypatch, tmp_path):
    body = SimpleNamespace(intent_for=lambda *a, **k: dict(pins={}),
        verify_intent=lambda filename, expected: json.loads(Path(filename).read_text()), save=lambda p, d: d)
    spec = SimpleNamespace(loader=SimpleNamespace(exec_module=lambda module: None))
    monkeypatch.setattr(adapter.importlib.util, 'spec_from_file_location', lambda *a: spec)
    monkeypatch.setattr(adapter.importlib.util, 'module_from_spec', lambda *a: body)
    workflow = adapter.load_workflow('foggy15')
    intent = tmp_path/'intent.json'
    intent.write_text(json.dumps(dict(pins={})))
    with pytest.raises(RuntimeError, match='authority'):
        workflow.verify_intent(intent, 'unused')
    intent.write_text(json.dumps(workflow.intent_for()))
    assert workflow.verify_intent(intent, 'unused')['pins'][str(adapter.TEMPLATE)] == adapter.TEMPLATE_SHA256


def test_only_new_command_plan_identity_is_generalized(tmp_path):
    workflow = adapter.load_workflow('foggy15')
    path = tmp_path/'plan.json'
    workflow.save(path, dict(schema='base5_foggy13_common_command_plan_v1', commands_executed=False))
    assert json.loads(path.read_text()) == dict(schema='base5_reinforced_environment_common_command_plan_v1',
        environment='foggy15', commands_executed=False)
