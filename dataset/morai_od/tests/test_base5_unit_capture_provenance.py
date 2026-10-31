import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from base5_capture_safety import atomic_json, sha256
from build_base5_mr_asset_catalog import DEFAULT_CATALOG
import base5_unit_geometry_contract as unit
from base5_unit_capture_provenance import UnitCaptureProvenanceError, verify_capture


@pytest.fixture
def sample(tmp_path):
    size = [.5727, .5724, .9194]
    source = tmp_path / 'native_unit.json'
    atomic_json(source, dict(assets={'RedBarrel': dict(data_id=40100003, bbox_size_m=size)}))
    origin = dict(path=str(source), sha256=sha256(source), assets=['RedBarrel'])
    policy = tmp_path / 'unit_review.json'
    atomic_json(policy, dict(schema=unit.SCHEMA, training_gt_modified=False, sources=[origin],
        catalog_reference=dict(path=str(DEFAULT_CATALOG.resolve()), sha256=sha256(DEFAULT_CATALOG)),
        assets={'RedBarrel': dict(data_id=40100003, source_list='objectList', raw_class='Obstacle',
                                  native_size_lwh_m=size, reference_source=origin)}))
    reference = dict(path=str(policy), sha256=sha256(policy))
    metadata = dict(native_unit_geometry_reference=reference, runtime_actors=[
        dict(name='RedBarrel', unique_id=8500, data_id=40100003, source_list='objectList', motion='static')])
    planned = tmp_path / 'planned_metadata.json'
    atomic_json(planned, metadata)
    actual = copy.deepcopy(metadata)
    actual.update(lidar_to_ego=[[1, 0, 0, 0]], sensor_source_sha256='runtime sensor',
                  capture_safety=dict(runtime_diagnostics='extra fields are allowed'))
    actual['runtime_actors'][0]['runtime_diagnostic'] = dict(ground_support='observed')
    capture = tmp_path / 'capture'
    atomic_json(capture / 'metadata.json', actual)
    scenario = dict(objectList=[dict(UNIQUEID=8500, DataID=40100003, scale=dict(x=1., y=1., z=1.))])
    atomic_json(capture / 'scenario.json', scenario)
    source_scenario = tmp_path / 'planned_scenario.json'
    atomic_json(source_scenario, scenario)
    audit = dict(components=[dict(parent_uid=8500, component_id='8500:Obstacle:0',
        asset='RedBarrel', data_id=40100003, raw_class='Obstacle', size_m=size,
        center_lidar_m=[1., 2., 3.], rotation_rpy=[0., 0., .5])],
        accepted_candidate=True, review_reasons=[])
    audit['native_unit_geometry_review'] = unit.review(actual, audit)
    atomic_json(capture / 'audit.json', audit)
    annotation = capture / 'LIDAR_5/a_Instance.txt'
    annotation.parent.mkdir()
    annotation.write_text('Obstacle 1 2 3 0 0 0.5 .5727 .5724 .9194 0 0 0 0 8500\n')
    entry = dict(id='a', metadata=str(planned), scenario=str(source_scenario),
                 scenario_sha256=sha256(source_scenario))
    manifest = dict(config={unit.KEY: reference})
    ledger = dict(id='a', status='accepted_candidate', scenario_sha256=entry['scenario_sha256'],
                  audit=copy.deepcopy(audit))
    return entry, capture, manifest, ledger


def test_valid_unit_capture_allows_runtime_diagnostics_and_changes_no_source(sample):
    entry, capture, manifest, ledger = sample
    snapshot = {path: path.read_bytes() for path in capture.parent.rglob('*') if path.is_file()}
    result = verify_capture(entry, capture, manifest, ledger)
    assert result['verified'] and result['compared'] == 1 and result['ledger_audit_bound']
    assert not result['raw_or_status_modified']
    for path, original in snapshot.items():
        assert path.read_bytes() == original


@pytest.mark.parametrize('change', ['removed_inventory', 'name', 'uid', 'data_id', 'motion', 'scale'])
def test_completed_actor_identity_cannot_hide_behind_matching_reference_tag(sample, change):
    entry, capture, manifest, ledger = sample
    metadata = json.loads((capture / 'metadata.json').read_text())
    if change == 'removed_inventory':
        metadata['runtime_actors'] = []
    else:
        actor = metadata['runtime_actors'][0]
        if change == 'name': actor['name'] = 'IgnoredAsset'
        if change == 'uid': actor['unique_id'] = 8501
        if change == 'data_id': actor['data_id'] = 999
        if change == 'motion': actor['motion'] = 'walking'
        if change == 'scale': actor['asset_scale'] = dict(scale_xyz=[1.25] * 3)
    atomic_json(capture / 'metadata.json', metadata)
    with pytest.raises(UnitCaptureProvenanceError, match='identity/order/motion/scale'):
        verify_capture(entry, capture, manifest, ledger)


def test_removed_or_foreign_capture_reference_fails_closed(sample):
    entry, capture, manifest, ledger = sample
    metadata = json.loads((capture / 'metadata.json').read_text())
    metadata.pop(unit.KEY)
    atomic_json(capture / 'metadata.json', metadata)
    with pytest.raises(UnitCaptureProvenanceError, match='reference changed'):
        verify_capture(entry, capture, manifest, ledger)


@pytest.mark.parametrize('change', ['missing_review', 'forged_review', 'ledger_diff', 'native_size'])
def test_saved_unit_review_and_actual_native_geometry_are_bound(sample, change):
    entry, capture, manifest, ledger = sample
    audit = json.loads((capture / 'audit.json').read_text())
    if change == 'missing_review': audit.pop('native_unit_geometry_review')
    if change == 'forged_review': audit['native_unit_geometry_review']['actors'][0]['matched'] = False
    if change == 'ledger_diff': audit['extra_unrecorded_field'] = True
    if change == 'native_size': audit['components'][0]['size_m'][0] *= 1.25
    atomic_json(capture / 'audit.json', audit)
    with pytest.raises(UnitCaptureProvenanceError):
        verify_capture(entry, capture, manifest, ledger if change == 'ledger_diff' else None)


def make_legitimate_unit_mismatch(capture):
    metadata = json.loads((capture / 'metadata.json').read_text())
    audit = json.loads((capture / 'audit.json').read_text())
    audit['components'][0]['size_m'] = [value * 1.25 for value in audit['components'][0]['size_m']]
    annotation = capture / 'LIDAR_5/a_Instance.txt'
    fields = annotation.read_text().split()
    fields[7:10] = [str(value) for value in audit['components'][0]['size_m']]
    annotation.write_text(' '.join(fields) + '\n')
    audit['native_unit_geometry_review'] = unit.review(metadata, audit)
    audit['review_reasons'] = ['unit_native_bbox_size_mismatch', 'other_existing_QA']
    audit['accepted_candidate'] = False
    atomic_json(capture / 'audit.json', audit)
    return audit


def test_legitimate_review_remains_review_and_is_not_promoted(sample):
    entry, capture, manifest, ledger = sample
    audit = make_legitimate_unit_mismatch(capture)
    ledger.update(status='review', audit=audit)
    result = verify_capture(entry, capture, manifest, ledger)
    assert result['review_reasons'] == ['unit_native_bbox_size_mismatch']
    assert ledger['status'] == 'review'
    assert not audit['accepted_candidate']


@pytest.mark.parametrize('change', ['reason_removed', 'accepted_audit', 'accepted_ledger'])
def test_unit_mismatch_cannot_claim_accepted_or_drop_qa_reason(sample, change):
    entry, capture, manifest, ledger = sample
    audit = make_legitimate_unit_mismatch(capture)
    ledger.update(status='review', audit=audit)
    if change == 'reason_removed': audit['review_reasons'] = []
    if change == 'accepted_audit': audit['accepted_candidate'] = True
    if change == 'accepted_ledger': ledger['status'] = 'accepted_candidate'
    atomic_json(capture / 'audit.json', audit)
    with pytest.raises(UnitCaptureProvenanceError):
        verify_capture(entry, capture, manifest, ledger)


def test_policy_source_tampering_is_fatal(sample):
    entry, capture, manifest, ledger = sample
    Path(manifest['config'][unit.KEY]['path']).write_text('{}')
    with pytest.raises(UnitCaptureProvenanceError):
        verify_capture(entry, capture, manifest, ledger)
    assert issubclass(UnitCaptureProvenanceError, RuntimeError)


def test_untagged_legacy_adds_no_unit_qa_requirements(tmp_path):
    capture = tmp_path / 'legacy'
    atomic_json(capture / 'metadata.json', {})
    planned = tmp_path / 'planned.json'
    atomic_json(planned, {})
    assert verify_capture(dict(metadata=str(planned)), capture, dict(config={})) is None
    assert not (capture / 'audit.json').exists()


def test_tagged_capture_without_planned_source_is_rejected(sample):
    _, capture, manifest, _ = sample
    with pytest.raises(UnitCaptureProvenanceError, match='planned source'):
        verify_capture(None, capture, manifest)
