"""CPU-only immutable preparation reuse; never imports or launches ROS."""
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import base5_shadow_ground_reuse as reuse
import generate_dynamic_object_capture_set as geo
from base5_capture_safety import atomic_json, sha256


def ledger(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(r) + '\n' for r in rows))


def fixture(root, replacement=False):
    sensor = root / 'sensor.json'
    atomic_json(sensor, dict(sensor='fixture'))
    spec = dict(family='shadow', shadow_moving=True, seed=11, role='front')
    points = [dict(x=6., y=0., z=.5), dict(x=10., y=0., z=.5)]
    actor = dict(unique_id=8500, data_id=50300002, name='Dog1', source_list='pedestrianList',
        motion='walking', requested_motion='walking', population_pool='animal',
        position=points[0], motion_path=dict(policy='natural-target-v1', points=points,
            target_position=dict(x=8., y=0., z=.5), ground_path_required=True))
    original = dict(spec=spec, ego=dict(x=0., y=0., yaw=0.), split='train',
        runtime_actors=[actor], scene_group_id='group_a', logical_quota_slot_id='a',
        fresh_environment_provenance=dict(geometry_seed=11),
        environment_split_contract=dict(original_slot_split='train'))
    scene = dict(egoVehicle={}, pedestrianList=[dict(UNIQUEID=8500)],
        waypointDataList=[dict(pathName='PedPath - 8500', waypointData=[
            dict(pos=geo.world_to_morai_waypoint(p)) for p in points])])
    config = dict(environment_tag='sunny13', capture={})
    def deck(folder, ident, meta, payload):
        atomic_json(folder / 'scene.json', payload)
        atomic_json(folder / 'metadata.json', meta)
        row = dict(id=ident, scenario=str(folder / 'scene.json'), metadata=str(folder / 'metadata.json'),
            scenario_sha256=sha256(folder / 'scene.json'))
        atomic_json(folder / 'manifest.json', dict(config=config, count=1, scenes=[row]))
        return folder / 'manifest.json', row
    source, source_row = deck(root / 'source', 'a', original, scene)
    before = copy.deepcopy(original)
    input_scene = copy.deepcopy(scene)
    if replacement:
        input_scene['replacement_geometry'] = True
        before['ego']['x'] = 1.
    input_manifest, input_row = deck(root / 'historical/cell', 'a_replace' if replacement else 'a', before, input_scene)
    files = []
    def capture(session, ident, meta, payload, status='accepted_candidate'):
        capture_root = session / 'captures' / ident
        atomic_json(capture_root / 'scenario.json', payload)
        raw = copy.deepcopy(meta)
        raw['sensor_source_sha256'] = sha256(sensor)
        atomic_json(capture_root / 'metadata.json', raw)
        summary = dict(observed=True, no_collision_verified=True, non_ego_uids=[])
        audit = dict(review_reasons=[], capture_safety=dict(collision=summary))
        atomic_json(capture_root / 'audit.json', audit)
        atomic_json(capture_root / 'telemetry.json', {})
        atomic_json(capture_root / 'collision_history.json', dict(summary=summary))
        (capture_root / 'sensor_reference.json').write_bytes(sensor.read_bytes())
        launch = (prepared_manifest if session.name == 'initial'
            else deck(session.parent / (session.name + '_deck'), ident, meta, payload)[0])
        atomic_json(session / 'session_contract.json', dict(sensor_sha256=sha256(sensor),
            manifest_sha256=sha256(launch)))
        atomic_json(session / 'shutdown_safety.json', dict(park_verified=True))
        row = dict(id=ident, scenario_sha256=sha256(capture_root / 'scenario.json'),
            capture_root=str(capture_root), status=status, audit=audit)
        ledger(session / 'ledger.jsonl', [row])
        return capture_root, row
    empty = copy.deepcopy(before)
    empty.update(calibration_only=True, runtime_actors=[], ground_probe_source=dict(
        scenario_sha256=input_row['scenario_sha256'], targets=copy.deepcopy(before['runtime_actors'])))
    empty['spec'] = dict(spec, family='background', actors=0, motion='static')
    terrain, _ = capture(root / 'historical/cell/motion_terrain', 'a_terrain', empty, {})
    static = copy.deepcopy(before)
    static.update(calibration_only=True, static_motion_reference=True)
    static['runtime_actors'][0]['motion'] = 'standing'
    reference, _ = capture(root / 'historical/cell/motion_static', 'a_static', static, {})
    for folder, filename in ((terrain, 'cloud.bin'), (reference, 'instance.txt')):
        (folder / filename).write_bytes(b'independent native witness')
        files.extend(reuse.pin(folder / name) for name in ('metadata.json', 'telemetry.json', filename))
    prepared = copy.deepcopy(before)
    prepared['runtime_actors'][0]['motion_path']['ground_path_evidence'] = dict(
        policy=reuse.GROUND, files=files, samples=[dict(x=p['x'], y=p['y'], z=0.) for p in points],
        asset_pivot_offset_m=.5)
    prepared_scene = copy.deepcopy(input_scene)
    prepared_scene['ground_prepared'] = True
    prepared_manifest, prepared_row = deck(root / 'historical/cell/motion_ground_deck',
        input_row['id'] + '_groundpath', prepared, prepared_scene)
    approved, approved_row = capture(root / 'historical/cell/initial', prepared_row['id'], prepared, prepared_scene)
    approved_row['source_cell_id'] = 'a'
    selected = root / 'historical/selected_ledger.jsonl'
    ledger(selected, [approved_row])
    selection = dict(source_id='a', mode='same_quota_replacement' if replacement else 'exact_source',
        prepared_manifest=str(prepared_manifest), selected_ledger=str(selected))
    return SimpleNamespace(**locals())


def registered(tmp_path, replacement=False):
    f = fixture(tmp_path, replacement)
    f.manifest = reuse.register(f.source, [f.selection], tmp_path / 'registration', f.sensor)
    return f


def test_legacy_no_opt_in_does_not_touch_files(tmp_path):
    assert reuse.validate_manifest(dict(config={}, scenes=[dict(metadata='/does/not/exist')])) == {}
    p = tmp_path / 'legacy.json'
    atomic_json(p, dict(config={}, scenes=[]))
    assert reuse.adopt(p, tmp_path / 'out') is None
    assert not (tmp_path / 'out').exists()


@pytest.mark.parametrize('replacement', [False, True])
def test_registration_and_adoption_are_fresh_native_plans(tmp_path, replacement):
    f = registered(tmp_path, replacement)
    unchanged = {p: (p.read_bytes(), p.stat().st_mtime_ns, p.stat().st_ino)
        for p in (f.selected, f.source, f.approved.parent.parent / 'ledger.jsonl')}
    output = reuse.adopt(f.manifest, tmp_path / 'new_plan')
    d = reuse.read(output)
    m = reuse.read(d['scenes'][0]['metadata'])
    assert d['scenes'][0]['id'] == 'a'
    assert m[reuse.KEY]['mode'] == f.selection['mode']
    assert d['scenes'][0]['scenario_sha256'] == f.prepared_row['scenario_sha256']
    assert m[reuse.KEY]['historical_raw_is_not_new_training_data'] is True
    assert reuse.read(f.manifest)['scenes'] == reuse.read(f.source)['scenes']
    assert not list(output.parent.glob('**/ledger.jsonl'))
    files = {p: (p.read_bytes(), p.stat().st_mtime_ns) for p in output.parent.iterdir()}
    assert reuse.adopt(f.manifest, output.parent) == output
    assert files == {p: (p.read_bytes(), p.stat().st_mtime_ns) for p in output.parent.iterdir()}
    assert unchanged == {p: (p.read_bytes(), p.stat().st_mtime_ns, p.stat().st_ino) for p in unchanged}


@pytest.mark.parametrize('replacement,mode', [(True, 'exact_source'), (False, 'same_quota_replacement'), (False, 'auto')])
def test_source_mode_must_be_explicit_and_correct(tmp_path, replacement, mode):
    f = fixture(tmp_path, replacement)
    f.selection['mode'] = mode
    with pytest.raises(reuse.ShadowReuseContractError):
        reuse.register(f.source, [f.selection], tmp_path / 'reg', f.sensor)


@pytest.mark.parametrize('key,value', [('spec', dict(family='shadow', shadow_moving=True, seed=12)),
    ('split', 'val'), ('scene_group_id', 'foreign'), ('logical_quota_slot_id', 'foreign')])
def test_foreign_quota_and_group_fail_before_registration(tmp_path, key, value):
    f = fixture(tmp_path)
    meta = reuse.read(f.source_row['metadata'])
    meta[key] = value
    atomic_json(f.source_row['metadata'], meta)
    with pytest.raises(reuse.ShadowReuseContractError):
        reuse.register(f.source, [f.selection], tmp_path / 'reg', f.sensor)


@pytest.mark.parametrize('key,value', [('name', 'Dog2'), ('data_id', 77), ('motion', 'running'),
    ('requested_motion', 'running'), ('population_pool', 'pedestrian')])
def test_asset_motion_identity_is_not_transplanted(tmp_path, key, value):
    f = fixture(tmp_path)
    meta = reuse.read(f.source_row['metadata'])
    meta['runtime_actors'][0][key] = value
    atomic_json(f.source_row['metadata'], meta)
    with pytest.raises(reuse.ShadowReuseContractError):
        reuse.register(f.source, [f.selection], tmp_path / 'reg', f.sensor)


@pytest.mark.parametrize('target', ['sensor', 'selected', 'source', 'input_manifest', 'prepared_manifest',
    'terrain/cloud.bin', 'reference/instance.txt', 'terrain/metadata.json',
    'terrain/../../ledger.jsonl', 'approved/../../ledger.jsonl', 'approved/metadata.json',
    'approved/../../shutdown_safety.json', 'approved/../../session_contract.json'])
def test_changed_frozen_evidence_is_hard_failure_not_deficit(tmp_path, target):
    f = registered(tmp_path)
    if '/' in target:
        head, tail = target.split('/', 1)
        path = (getattr(f, head) / tail).resolve()
    else:
        path = getattr(f, target)
    path.write_bytes(path.read_bytes() + b' ')
    with pytest.raises(reuse.ShadowReuseContractError):
        reuse.validate_manifest(reuse.read(f.manifest))
    assert not issubclass(reuse.ShadowReuseContractError, ValueError)


@pytest.mark.parametrize('change', ['scenario', 'metadata', 'registry', 'environment', 'source_id', 'quota'])
def test_adoption_and_configuration_tamper_is_rejected(tmp_path, change):
    f = registered(tmp_path)
    path = reuse.adopt(f.manifest, tmp_path / 'new')
    d = reuse.read(path)
    if change in ('scenario', 'metadata'):
        p = Path(d['scenes'][0][change])
        p.write_bytes(p.read_bytes() + b' ')
        if change == 'metadata':
            m = reuse.read(p)
            m['runtime_actors'][0]['name'] = 'foreign'
            atomic_json(p, m)
    elif change == 'registry':
        p = Path(d['config'][reuse.KEY]['registration']['path'])
        p.write_bytes(p.read_bytes() + b' ')
    elif change == 'environment':
        d['config']['environment_tag'] = 'foggy13'
    elif change == 'source_id':
        d['scenes'][0]['id'] = 'foreign'
    else:
        d['scenes'][0]['distance_bounds'] = [80, 100]
    with pytest.raises(reuse.ShadowReuseContractError):
        reuse.validate_manifest(d)


def test_incomplete_or_preexisting_output_is_never_overwritten(tmp_path):
    f = registered(tmp_path)
    with pytest.raises(reuse.ShadowReuseContractError):
        reuse.register(f.source, [f.selection], f.manifest.parent, f.sensor)
    folder = tmp_path / 'partial'
    folder.mkdir()
    (folder / 'ledger.jsonl').write_text('old incomplete ledger')
    with pytest.raises(reuse.ShadowReuseContractError):
        reuse.adopt(f.manifest, folder)
    assert (folder / 'ledger.jsonl').read_text() == 'old incomplete ledger'
    planned = reuse.adopt(f.manifest, tmp_path / 'planned')
    (planned.parent / 'ledger.jsonl').write_text('foreign completed ledger\n')
    with pytest.raises(reuse.ShadowReuseContractError):
        reuse.adopt(f.manifest, planned.parent)


def test_unregistered_replacement_is_not_guessed_from_quota(tmp_path):
    f = registered(tmp_path)
    deck = reuse.read(f.manifest)
    deck['scenes'][0]['id'] = 'a_replace'
    atomic_json(tmp_path / 'replacement.json', deck)
    assert reuse.adopt(tmp_path / 'replacement.json', tmp_path / 'not_created') is None


def test_completed_capture_verifies_frozen_metadata_but_accepts_runtime_observations(tmp_path):
    f = registered(tmp_path)
    path = reuse.adopt(f.manifest, tmp_path / 'new')
    d = reuse.read(path)
    entry = d['scenes'][0]
    capture = tmp_path / 'new_capture'
    raw = reuse.read(entry['metadata'])
    raw.update(sensor_source_sha256=sha256(f.sensor), capture_runtime_objects=[{'diagnostic': True}])
    atomic_json(capture / 'metadata.json', raw)
    (capture / 'scenario.json').write_bytes(Path(entry['scenario']).read_bytes())
    (capture / 'sensor_reference.json').write_bytes(f.sensor.read_bytes())
    reuse.verify_capture(entry, capture, d)
    raw['runtime_actors'][0]['position']['z'] += .1
    atomic_json(capture / 'metadata.json', raw)
    with pytest.raises(reuse.ShadowReuseContractError):
        reuse.verify_capture(entry, capture, d)


def test_opt_in_contracts_pin_inputs_and_reject_them_before_child_launch(tmp_path):
    import run_base5_capture_campaign as campaign
    from base5_capture_safety import prepare_session
    f = registered(tmp_path)
    contract = campaign.contract(f.manifest, [0], 1)
    assert str(f.selected.resolve()) in contract['shadow_ground_reuse_source_files']
    assert 'base5_shadow_ground_reuse.py' in contract['code']
    network = tmp_path / 'network.json'
    atomic_json(network, {})
    prepare_session(tmp_path / 'session', f.manifest, network, f.sensor)
    session = reuse.read(tmp_path / 'session/session_contract.json')
    assert session['shadow_ground_reuse_source_files'] == contract['shadow_ground_reuse_source_files']
    f.selected.write_bytes(f.selected.read_bytes() + b' ')
    with pytest.raises(reuse.ShadowReuseContractError):
        campaign.contract(f.manifest, [0], 1)
    with pytest.raises(reuse.ShadowReuseContractError):
        prepare_session(tmp_path / 'session', f.manifest, network, f.sensor)


def test_quality_loop_skips_only_registered_terrain_and_keeps_native_audit(tmp_path):
    import run_base5_quality_loop as loop
    f = registered(tmp_path)
    class Child:
        returncode = 0
        def poll(self):
            return 0
    def child(cmd, **kwargs):
        path = Path(cmd[cmd.index('--manifest') + 1])
        out = Path(cmd[cmd.index('--output') + 1])
        d = reuse.read(path)
        assert reuse.KEY in reuse.read(d['scenes'][0]['metadata'])
        raw = out/'captures'/'a'
        atomic_json(raw/'metadata.json',reuse.read(d['scenes'][0]['metadata']))
        row = dict(id='a', status='accepted_candidate', capture_root=str(raw), audit=dict(review_reasons=[]))
        ledger(out / 'ledger.jsonl', [row])
        atomic_json(out / 'shutdown_safety.json', dict(park_verified=True))
        return Child()
    with patch.object(loop.subprocess, 'Popen', side_effect=child) as run, \
            patch.object(loop, 'empty_probe') as empty, \
            patch.object(loop.motion_ground, 'static_reference') as static, \
            patch.object(loop.motion_ground, 'prepare') as prepare:
        assert loop.execute(f.manifest, [0], tmp_path / 'run') == 0
    assert run.call_count == 1
    empty.assert_not_called()
    static.assert_not_called()
    prepare.assert_not_called()
    report = reuse.read(tmp_path / 'run/report.json')
    assert [a['kind'] for a in report['attempts']] == ['initial']
    assert len(report['selected']) == 1


def test_runtime_closure_discovers_opt_in_helper():
    from base5_front_release_inputs import collect_runtime
    assert str(Path(reuse.__file__).resolve()) in collect_runtime()


def test_completed_session_resume_preserves_ledger_and_rejects_changed_raw(tmp_path):
    from base5_capture_safety import prepare_session
    f = registered(tmp_path)
    path = reuse.adopt(f.manifest, tmp_path / 'planned')
    d = reuse.read(path)
    entry = d['scenes'][0]
    network = tmp_path / 'network.json'
    atomic_json(network, {})
    session = tmp_path / 'new_session'
    prepare_session(session, path, network, f.sensor)
    capture = session / 'captures/a'
    raw = reuse.read(entry['metadata'])
    raw['sensor_source_sha256'] = sha256(f.sensor)
    atomic_json(capture / 'metadata.json', raw)
    atomic_json(capture / 'audit.json', dict(review_reasons=[]))
    atomic_json(capture / 'telemetry.json', {})
    atomic_json(capture / 'collision_history.json', {})
    (capture / 'scenario.json').write_bytes(Path(entry['scenario']).read_bytes())
    (capture / 'sensor_reference.json').write_bytes(f.sensor.read_bytes())
    for camera in range(1, 5):
        folder = capture / ('CAMERA_' + str(camera))
        folder.mkdir()
        (folder / 'test_Intensity.png').write_bytes(b'raw image')
    folder = capture / 'LIDAR_5'
    folder.mkdir()
    (folder / 'test_Intensity.bin').write_bytes(b'raw points')
    (folder / 'test_Instance.txt').write_bytes(b'raw annotation')
    row = dict(id='a', status='accepted_candidate', capture_root=str(capture),
        scenario_sha256=entry['scenario_sha256'])
    target = session / 'ledger.jsonl'
    ledger(target, [row])
    before = (target.read_bytes(), target.stat().st_mtime_ns, target.stat().st_ino)
    rows, done = prepare_session(session, path, network, f.sensor)
    assert rows == [row] and done == {'a': row}
    assert before == (target.read_bytes(), target.stat().st_mtime_ns, target.stat().st_ino)
    raw[reuse.KEY]['mode'] = 'same_quota_replacement'
    atomic_json(capture / 'metadata.json', raw)
    with pytest.raises(reuse.ShadowReuseContractError):
        prepare_session(session, path, network, f.sensor)
    assert before == (target.read_bytes(), target.stat().st_mtime_ns, target.stat().st_ino)


@pytest.mark.parametrize('which', ['selected', 'terrain'])
def test_historical_incomplete_ledger_is_never_repaired(tmp_path, which):
    f = fixture(tmp_path)
    target = f.selected if which == 'selected' else f.terrain.parents[1] / 'ledger.jsonl'
    target.write_bytes(target.read_bytes().rstrip(b'\n'))
    before = target.read_bytes(), target.stat().st_mtime_ns
    with pytest.raises(reuse.ShadowReuseContractError):
        reuse.register(f.source, [f.selection], tmp_path / 'reg', f.sensor)
    assert before == (target.read_bytes(), target.stat().st_mtime_ns)


def test_subset_and_copied_manifest_keep_full_source_binding_without_hash_cycle(tmp_path):
    f = registered(tmp_path)
    deck = reuse.read(f.manifest)
    deck['status'] = 'diagnostic subset; not a source rewrite'
    atomic_json(tmp_path / 'copied_subset/manifest.json', deck)
    pins = reuse.validate_manifest(deck)
    assert pins[str(f.source.resolve())] == sha256(f.source)
    assert str(f.manifest.resolve()) not in pins
    assert reuse.adopt(tmp_path / 'copied_subset/manifest.json', tmp_path / 'adopted')


@pytest.mark.parametrize('which', ['terrain', 'reference', 'approved'])
@pytest.mark.parametrize('key,value', [('observed', False), ('no_collision_verified', False),
                                     ('non_ego_uids', [8500])])
def test_even_newly_pinned_history_must_have_safe_collision_semantics(tmp_path, which, key, value):
    f = fixture(tmp_path)
    capture = getattr(f, which)
    history = reuse.read(capture / 'collision_history.json')
    history['summary'][key] = value
    atomic_json(capture / 'collision_history.json', history)
    with pytest.raises(reuse.ShadowReuseContractError, match='collision safety'):
        reuse.register(f.source, [f.selection], tmp_path / 'registration', f.sensor)


@pytest.mark.parametrize('change', ['manifest', 'metadata', 'scenario', 'contract', 'missing'])
def test_historical_child_launch_contract_is_checked_before_registration(tmp_path, change):
    f = fixture(tmp_path)
    folder = tmp_path / 'historical/cell/motion_terrain_deck'
    if change == 'contract':
        path = f.terrain.parents[1] / 'session_contract.json'
        value = reuse.read(path)
        value['manifest_sha256'] = '0' * 64
        atomic_json(path, value)
    elif change == 'missing':
        (folder / 'manifest.json').unlink()
    elif change == 'metadata':
        path = folder / 'metadata.json'
        value = reuse.read(path)
        value['ego']['x'] += 1.
        atomic_json(path, value)
    else:
        path = folder / ('manifest.json' if change == 'manifest' else 'scene.json')
        path.write_bytes(path.read_bytes() + b' ')
    with pytest.raises(reuse.ShadowReuseContractError):
        reuse.register(f.source, [f.selection], tmp_path / 'registration', f.sensor)
