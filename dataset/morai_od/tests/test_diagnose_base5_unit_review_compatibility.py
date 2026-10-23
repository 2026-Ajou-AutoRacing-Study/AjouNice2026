import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from base5_capture_safety import atomic_json, sha256
from build_base5_mr_asset_catalog import DEFAULT_CATALOG
import base5_unit_geometry_contract as unit
from diagnose_base5_unit_review_compatibility import (diagnose, review_in_memory,
    summarize_reviews)


def policy(tmp_path):
    source = tmp_path / 'native_unit.json'
    size = [.5727, .5724, .9194]
    atomic_json(source, dict(assets={'RedBarrel': dict(data_id=40100003, bbox_size_m=size)}))
    origin = dict(path=str(source), sha256=sha256(source), assets=['RedBarrel'])
    path = tmp_path / 'unit_policy.json'
    atomic_json(path, dict(schema=unit.SCHEMA, training_gt_modified=False, sources=[origin],
        catalog_reference=dict(path=str(DEFAULT_CATALOG.resolve()), sha256=sha256(DEFAULT_CATALOG)),
        assets={'RedBarrel': dict(data_id=40100003, source_list='objectList', raw_class='Obstacle',
                                  native_size_lwh_m=size, reference_source=origin)}))
    return dict(path=str(path), sha256=sha256(path))


def capture(tmp_path, name, *, size_factor=1., scenario_scale=1.):
    root = tmp_path / name
    size = [value * size_factor for value in [.5727, .5724, .9194]]
    metadata = dict(split='train', runtime_actors=[dict(name='RedBarrel', data_id=40100003,
        source_list='objectList', motion='static', unique_id=8500)])
    if scenario_scale != 1.:
        metadata['runtime_actors'][0]['asset_scale'] = dict(scale_xyz=[scenario_scale] * 3)
    audit = dict(components=[dict(parent_uid=8500, raw_class='Obstacle', size_m=size)],
                 accepted_candidate=True, review_reasons=[])
    scenario = dict(objectList=[dict(UNIQUEID=8500, DataID=40100003,
                                    scale=dict(x=scenario_scale, y=scenario_scale, z=scenario_scale))])
    for filename, document in [('metadata.json', metadata), ('audit.json', audit), ('scenario.json', scenario)]:
        atomic_json(root / filename, document)
    return root, metadata, audit


def test_review_tag_is_ram_only_and_does_not_change_legacy_qa(tmp_path):
    reference = policy(tmp_path)
    root, metadata, audit = capture(tmp_path, 'raw')
    before = copy.deepcopy((metadata, audit))
    snapshot = {path: path.read_bytes() for path in root.iterdir()}
    assert not review_in_memory(metadata, audit, reference)['review_reasons']
    assert before == (metadata, audit)
    assert unit.KEY not in metadata
    for path, original in snapshot.items():
        assert path.read_bytes() == original


def test_existing_historical_tag_cannot_be_replaced(tmp_path):
    reference = policy(tmp_path)
    _, metadata, audit = capture(tmp_path, 'raw')
    metadata[unit.KEY] = dict(path='/tmp/other.json', sha256='different')
    with pytest.raises(unit.UnitGeometryContractError, match='existing historical'):
        review_in_memory(metadata, audit, reference)


def test_replay_full_path_keeps_unit_nonunit_and_canonical_separate(tmp_path):
    reference = policy(tmp_path)
    rows = []
    for index in range(2):
        root, _, audit = capture(tmp_path, 'canonical_' + str(index))
        rows.append(dict(id='scene' + str(index), capture_root=str(root), status='accepted_candidate',
                         scenario_sha256=sha256(root / 'scenario.json'), audit=audit))
    ledger = tmp_path / 'selected.jsonl'
    ledger.write_text(''.join(json.dumps(row) + '\n' for row in rows))
    observed, files = [], {}
    for index, (size_factor, scale) in enumerate([(1., 1.), (1.25, 1.), (1.5, 1.5)]):
        root, _, audit = capture(tmp_path, 'red_' + str(index), size_factor=size_factor, scenario_scale=scale)
        for path in root.iterdir():
            files[str(path)] = sha256(path)
        observed.append(dict(capture_root=str(root), parent_uid=8500, case=str(index), stage='historical',
            scenario_scale_xyz=[scale] * 3, native_summary_size_m=audit['components'][0]['size_m'],
            observed_summary_state='large_state' if size_factor == 1.25 else 'small_state'))
    evidence = tmp_path / 'red_evidence.json'
    atomic_json(evidence, dict(passed=True, asset=dict(name='RedBarrel'), source_files=files, observations=observed))
    snapshot = {path: path.read_bytes() for path in tmp_path.rglob('*') if path.is_file()}
    output = tmp_path / 'diagnostic'
    result = diagnose(ledger, reference, evidence, output, expected_frames=2,
                      expected_red_unit_mismatches=1)
    assert result['verification_passed']
    assert result['canonical']['totals']['matched'] == 2
    assert result['canonical']['totals']['mismatched'] == 0
    assert result['historical_red']['totals']['matched'] == 1
    assert result['historical_red']['totals']['mismatched'] == 1
    assert result['historical_red']['totals']['skipped_nonunit'] == 1
    assert not result['legacy_qa_upgraded']
    assert result['metadata_sidecars_created'] is False
    assert sorted(path.name for path in output.iterdir()) == ['canonical_reviews.json',
        'historical_red_observations.json', 'report.json', 'source_pins.json']
    for path, content in snapshot.items():
        assert path.read_bytes() == content
    with pytest.raises(ValueError, match='Fresh diagnostic'):
        diagnose(ledger, reference, evidence, output, expected_frames=2)


def test_summary_does_not_count_uncompared_component_change_as_match():
    records = [dict(scene_id='test', classification={8500: 'compound_parent'},
        registered_actors=[dict(name='RedBarrel', unique_id=8500)],
        review=dict(review_reasons=['unit_native_component_contract_changed'],
            actors=[dict(name='RedBarrel', parent_uid=8500, compared=False)]))]
    result = summarize_reviews(records, ['RedBarrel'])
    assert result['totals']['matched'] == result['totals']['compared'] == 0
    assert result['totals']['component_contract_changed'] == 1
