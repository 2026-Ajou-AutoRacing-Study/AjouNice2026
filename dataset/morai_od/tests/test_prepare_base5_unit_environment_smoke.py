"""새 환경 smoke는 CPU 사전 등록이며 ROS/MORAI를 실행하지 않는다."""
import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_unit_environment_smoke as smoke


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


@pytest.fixture
def selection(tmp_path, monkeypatch):
    assets = {'physical'+str(i) for i in range(19)}
    entries, metadata = [], {}

    def add(ident, family='obstacle', actors=(), spec=None, split='train'):
        row = dict(family=family, mission=False);row.update(spec or {})
        meta = dict(spec=row, runtime_actors=list(actors), split=split,
            training_split_overlay=dict(enabled=True), **{smoke.unit.KEY:dict(reference=True)})
        path = write(tmp_path/'source'/(ident+'.json'), meta)
        metadata[ident] = meta
        entries.append(dict(id=ident, metadata=str(path), scenario_sha256='a'*64, split=split))

    def actor(name, motion='static', **kwargs):
        return dict(name=name, motion=motion, source_list='objectList', **kwargs)

    for index, crossing in enumerate((*smoke.CROSSWALKS, smoke.CROSSWALKS[0])):
        add('cross'+str(index), 'pedestrian_related', [actor('Person', motion='running',
            mission_crossing_id=crossing, motion_path=dict(ground_path_evidence=dict(policy=smoke.EDITOR_POLICY)))])
    for lo, hi in ((50,80), (80,100)):
        for role in ('front','rear'):
            add('far'+str(lo)+role, 'highway', [dict(name='Car', motion='static', source_list='vehicleList',
                actual_ego_role=role, planned_center_distance_m=(lo+hi)/2)])
    for i, name in enumerate(sorted(assets)):
        add(name, actors=[actor(name, planned_center_distance_m=10 if i%2 else 25)],
            spec=dict(mission=True, static_mission_extent=dict(stratum=('before','intersection','after')[i%3]),
                avoidance_phase=smoke.PHASES[i%4]), split='val' if i%2 else 'train')
    for name in smoke.COMPOSITES:
        add(name, 'pedestrian_related', [actor(name, motion='standing')])
    for stage in ('approach_near', 'interior'):
        add('shadow_'+stage, 'shadow', [actor('other')], spec=dict(shadow_observation_stage=stage))
    for i in range(20):
        add('extra'+str(i))
    for i in range(24):
        add('moving_shadow'+str(i), 'shadow', [actor('Person', motion='walking')], spec=dict(shadow_moving=True))
    monkeypatch.setattr(smoke.layout, 'resolve', lambda overlay, ident, entry, meta:
        dict(split=meta['split'], group_id='group:'+ident))
    return dict(deck=dict(count=len(entries), scenes=entries), metadata=metadata, overlay={}, assets=assets)


def select(fixture, target=32, maximum=48):
    return smoke.select(fixture['deck'], fixture['metadata'], fixture['overlay'], fixture['assets'], target, maximum)


def test_exact32_covers_all_native_contracts_without_old_shadow_controls(selection):
    rows, report = select(selection)
    assert len(rows) == len({r['layout_group_id'] for r in rows}) == 32
    assert set(report['required']) <= set(report['covered'])
    assert report['editor_crossing_scenes'] == 3
    assert report['eligibility']['excluded_count'] == 24
    assert report['eligibility']['scene_counts']['unprepared_shadow'] == 24
    assert not report['eligibility']['old_shadow_reuse_adopted']
    assert all('moving_shadow' not in r['id'] for r in rows)
    assert {'static_shadow/approach','static_shadow/interior'} <= set(report['covered'])
    assert not report['native_outcomes_used_for_selection']


def test_source_close_policy_requires_a_valid_tagged_primary_when_present(selection, monkeypatch):
    meta = selection['metadata']['physical0']
    reference = meta[smoke.unit.KEY]
    policy = smoke.close.configuration(reference)
    native = dict(assets=dict(physical0=dict(data_id=100, source_list='objectList', native_size_lwh_m=[1.,1.,1.])))
    monkeypatch.setattr(smoke.close, 'policy_for', lambda config:(policy, native))
    config = {smoke.unit.KEY:reference, smoke.close.KEY:policy}
    selection['deck']['config'] = config
    meta['spec'].update(motion='static', avoidance_phase='alongside', role='left',
        avoidance_side='right', avoidance_gap_bounds_m=[0., .3])
    meta['runtime_actors'][0].update(data_id=100, geometry_reference_known=True)
    meta['spec'] = smoke.close.tagged_row(meta, config)
    rows, report = select(selection)
    assert 'pipeline/close_primary' in report['required']
    assert 'pipeline/close_primary' in report['covered']
    assert any(row['id'] == 'physical0' and 'pipeline/close_primary' in row['coverage'] for row in rows)
    meta['spec'][smoke.close.KEY]['primary_name'] = 'different'
    with pytest.raises(RuntimeError, match='Close-primary row/config/physical identity differs'):
        select(selection)


def test_source_close_policy_with_zero_tagged_eligibility_does_not_force_native_control(selection):
    reference = dict(reference=True)
    selection['deck']['config'] = {smoke.unit.KEY:reference,
        smoke.close.KEY:smoke.close.configuration(reference)}
    rows, report = select(selection)
    assert len(rows) == 32
    assert 'pipeline/close_primary' not in report['required']
    assert 'pipeline/close_primary' not in report['covered']


def test_order_independent_metadata_selection_and_review_outcomes_ignored(selection):
    expected = select(selection)
    selection['deck']['scenes'].reverse()
    selection['metadata'] = dict(reversed(list(selection['metadata'].items())))
    for meta in selection['metadata'].values():
        meta.update(status='review', review_reasons=['whatever'], native_outcomes=['failed'])
    assert select(selection) == expected


def test_explicit48_is_not_automatic_and_does_not_include_unprepared_shadow(selection):
    assert len(select(selection, 48, 48)[0]) == 48
    assert len(select(selection)[0]) == 32


@pytest.mark.parametrize('target,maximum', [(31,48),(49,49),(32,49),(32,31),(True,48),(32,True)])
def test_finite_target_required(selection, target, maximum):
    with pytest.raises(ValueError):
        select(selection, target, maximum)


@pytest.mark.parametrize('kind', ['physical','compound','far','crossing','extent','split','range','shadow_approach','shadow_interior'])
def test_missing_coverage_fails_before_capture(selection, kind):
    metadata = selection['metadata']
    if kind == 'physical':metadata['physical0']['runtime_actors'] = []
    if kind == 'compound':metadata['BabyStroller3']['runtime_actors'] = []
    if kind == 'far':metadata['far80rear']['runtime_actors'][0]['planned_center_distance_m'] = 49
    if kind == 'crossing':metadata['cross1']['runtime_actors'][0]['motion'] = 'standing'
    if kind == 'extent':
        for meta in metadata.values():meta['spec'].pop('static_mission_extent', None)
    if kind == 'split':
        for meta in metadata.values():meta['split'] = 'train'
    if kind == 'range':
        for meta in metadata.values():
            for actor in meta['runtime_actors']:
                if actor['name'] in selection['assets']:actor['planned_center_distance_m'] = 10
    if kind == 'shadow_approach':metadata['shadow_approach_near']['spec']['shadow_observation_stage'] = 'exit'
    if kind == 'shadow_interior':metadata['shadow_interior']['spec']['shadow_observation_stage'] = 'entrance'
    with pytest.raises(ValueError):
        select(selection)


def test_only_two_independent_crossings_is_not_three(selection):
    ident = 'cross2'
    selection['metadata'][ident]['runtime_actors'][0]['motion'] = 'standing'
    with pytest.raises(ValueError, match='Three independent'):
        select(selection)


def test_crossing_and_far_protected_controls_cannot_share_layout_group(selection, monkeypatch):
    monkeypatch.setattr(smoke.layout, 'resolve', lambda overlay, ident, entry, meta:
        dict(split=meta['split'], group_id='same' if ident.startswith('far') else ident))
    with pytest.raises(ValueError, match='Independent protected'):
        select(selection)


@pytest.mark.parametrize('change', ['unknown','partial','calibration','old_reuse','prepared_shadow'])
def test_unprepared_unknown_and_legacy_motion_never_admitted(selection, change):
    meta = selection['metadata']['moving_shadow0']
    if change == 'unknown':meta['runtime_actors'][0]['motion'] = 'teleport'
    if change == 'partial':meta['spec']['family'] = 'pedestrian_related'
    if change == 'calibration':meta['calibration_only'] = True
    if change == 'old_reuse':meta[smoke.shadow.KEY] = dict(old=True)
    if change == 'prepared_shadow':
        meta['runtime_actors'][0]['motion_path'] = dict(ground_path_evidence=dict(policy=smoke.EDITOR_POLICY))
    with pytest.raises(ValueError):
        select(selection)


@pytest.fixture
def source(tmp_path, monkeypatch):
    names = ['physical'+str(i) for i in range(19)]
    catalog = write(tmp_path/'catalog.json', dict(assets=[dict(data_id=100+i, name=name,
        source_list='objectList', semantic_class='obstacle') for i,name in enumerate(names)]))
    reference = dict(path=str(tmp_path/'unit.json'), sha256='a'*64)
    policy = dict(assets={name:{} for name in names}, catalog_reference=dict(path=str(catalog)))
    scenario = write(tmp_path/'scene.json', dict(objectList=[dict(UNIQUEID=8500, DataID=100, scale=dict(x=1,y=1,z=1))]))
    metadata = write(tmp_path/'meta.json', dict(split='train', spec=dict(family='obstacle'),
        runtime_actors=[dict(name=names[0], data_id=100, unique_id=8500, source_list='objectList', motion='static')],
        fresh_environment_provenance=dict(environment='sunny15'), **{smoke.unit.KEY:reference}))
    overlay = write(tmp_path/'overlay.json', {})
    deck = dict(count=5000, native_qualified=False, bulk_ready=False,
        scenes=[dict(id=str(i), scenario=str(scenario), metadata=str(metadata), split='train',
            scenario_sha256=smoke.sha256(scenario)) for i in range(5000)],
        split_overlay=dict(path=str(overlay)), config=dict(environment_tag='sunny15',
            collection_environment=dict(weather='sunny', hour=15), **{smoke.unit.KEY:reference}))
    full = write(tmp_path/'manifest.json', deck)
    pin = write(tmp_path/'pin.json', {});pins = {str(pin):smoke.sha256(pin)}
    receipt = write(tmp_path/'build_receipt.json', dict(policy=smoke.BUILD_POLICY, environment='sunny15',
        final_manifest=str(full), final_manifest_sha256=smoke.sha256(full),
        status='CPU_ONLY_PASS; native environment qualification required', source_and_baseline_preserved=True,
        original_input_hashes=pins, runtime_hashes=pins))
    monkeypatch.setattr(smoke.unit, 'load', lambda *args:copy.deepcopy(policy))
    monkeypatch.setattr(smoke.layout, 'validate_manifest_overlay', lambda *args:dict(passed=True))
    monkeypatch.setattr(smoke.layout, 'read_overlay', lambda *args:{})
    monkeypatch.setattr(smoke.pins, 'collect_inputs', lambda *args:{str(full):smoke.sha256(full)})
    return full, receipt, deck, metadata, scenario, pin


def test_full_admission_preserves_unit_source_and_pins_metadata_not_only_selected(source):
    full, receipt, *_ = source
    result = smoke.inspect_source(full, receipt)
    assert len(result['entries']) == 5000
    assert result['environment'] == 'sunny15'
    assert result['raw_scale_counts'] == {'objectList/explicit111':5000}
    assert str(receipt) in result['input_hashes']


@pytest.mark.parametrize('kind', ['old_environment','receipt','policy','nonunit','unknown_asset','source_env',
    'split','runtime','native','old_shadow','duplicate'])
def test_full_source_corruption_fails(source, kind):
    full, receipt, deck, meta_path, scenario, pin = source
    meta, scene, doc = smoke.read(meta_path), smoke.read(scenario), smoke.read(receipt)
    if kind == 'old_environment':deck['config']['environment_tag'] = 'sunny13'
    if kind == 'receipt':doc['final_manifest_sha256'] = '0'*64
    if kind == 'policy':deck['config']['asset_scale_policy'] = {}
    if kind == 'nonunit':scene['objectList'][0]['scale']['x'] = 2
    if kind == 'unknown_asset':meta['runtime_actors'][0]['name'] = 'different'
    if kind == 'source_env':meta['fresh_environment_provenance']['environment'] = 'sunny13'
    if kind == 'split':meta['split'] = 'val'
    if kind == 'runtime':write(pin, dict(changed=True))
    if kind == 'native':deck['native_qualified'] = True
    if kind == 'old_shadow':deck['config'][smoke.shadow.KEY] = dict(old=True)
    if kind == 'duplicate':deck['scenes'][1]['id'] = '0'
    write(meta_path, meta);write(scenario, scene);write(full, deck)
    if kind != 'receipt':doc['final_manifest_sha256'] = smoke.sha256(full)
    if kind == 'nonunit':
        for entry in deck['scenes']:entry['scenario_sha256'] = smoke.sha256(scenario)
        write(full, deck);doc['final_manifest_sha256'] = smoke.sha256(full)
    write(receipt, doc)
    with pytest.raises((ValueError, RuntimeError)):
        smoke.inspect_source(full, receipt)


def test_preparation_refuses_existing_source_or_symlinked_raw_output(source):
    full, receipt, *_ = source
    with pytest.raises(ValueError, match='outside source'):
        smoke.prepare(full, receipt, full.parent/'new')
    with pytest.raises(ValueError):
        smoke.prepare(full, receipt, full.parent)


def test_no_simulator_runner_imported_or_started():
    source = Path(smoke.__file__).read_text()
    assert 'subprocess' not in source and 'rospy' not in source
    assert 'capture_executed=False' in source


def test_runtime_closure_pins_actual_dependencies_without_unused_legacy_training_exports():
    hashes = smoke.pins.collect_runtime(entrypoints=(Path(smoke.__file__).name,))
    assert hashes[str(Path(smoke.__file__).resolve())] == smoke.sha256(smoke.__file__)
    for filename in ('base5_layout_split.py', 'base5_unit_geometry_contract.py',
                     'base5_motion_ground.py', 'audit_base5_scenarios.py'):
        assert str(Path(smoke.__file__).with_name(filename).resolve()) in hashes
    assert not any(Path(path).name == 'prepare_base5_unit_release_smoke.py' for path in hashes)


@pytest.fixture
def registered(selection, tmp_path, monkeypatch):
    full = write(tmp_path/'full'/'manifest.json', selection['deck'])
    receipt = write(tmp_path/'build_receipt.json', {})
    pin = write(tmp_path/'runtime.json', {})
    runtime = {str(pin):smoke.sha256(pin)}
    context = dict(deck=selection['deck'], metadata=selection['metadata'], overlay=selection['overlay'],
        unit_assets=selection['assets'], entries={e['id']:e for e in selection['deck']['scenes']},
        environment='sunny15', input_hashes={str(full):smoke.sha256(full)},
        build_receipt=receipt, raw_scale_counts={'objectList/explicit111':19})
    monkeypatch.setattr(smoke.pins, 'collect_runtime', lambda *args, **kwargs:dict(runtime))
    monkeypatch.setattr(smoke, 'inspect_source', lambda *args:copy.deepcopy(context))
    monkeypatch.setattr(smoke.scenario_audit, 'audit', lambda *args:
        dict(status='OFFLINE_PASS', errors=[], scenes=5000, unique_payloads=5000))
    output = tmp_path/'registration'
    smoke.prepare(full, receipt, output)
    return full, output, context, pin


def test_registration_replay_preserves_exact_source_entries_and_environment(registered):
    full, output, source, _ = registered
    context = smoke.registration_context(full, output/'registration.json')
    assert len(context['selected']) == 32
    assert context['subset'].get('config') == source['deck'].get('config')
    assert context['doc']['environment'] == 'sunny15'
    assert context['doc']['coverage']['eligibility']['excluded_count'] == 24
    assert context['subset']['scenes'] == [source['entries'][r['id']] for r in context['doc']['selected']]
    for name in ('native_qualified','bulk_ready','bulk_started','training_started','capture_executed'):
        assert context['doc'][name] is False


@pytest.mark.parametrize('collision', ['directory', 'symlink'])
def test_output_created_during_validation_is_never_overwritten(registered, tmp_path, monkeypatch, collision):
    full, _, source, _ = registered
    output = tmp_path/'competing_registration'
    protected = tmp_path/'preserved_registration'
    audit_result = dict(status='OFFLINE_PASS', errors=[], scenes=5000, unique_payloads=5000)

    def competing_audit(*args):
        if collision == 'directory':
            output.mkdir()
        else:
            protected.mkdir()
            output.symlink_to(protected, target_is_directory=True)
        write(output/'registration.json', dict(owner='other execution'))
        return audit_result

    monkeypatch.setattr(smoke.scenario_audit, 'audit', competing_audit)
    with pytest.raises(FileExistsError):
        smoke.prepare(full, source['build_receipt'], output)
    assert smoke.read(output/'registration.json') == dict(owner='other execution')
    assert not (output/'manifest.json').exists()
    assert not (output/'source_audit.json').exists()


@pytest.mark.parametrize('kind', ['helper','source','subset','receipt','audit','selected','coverage','runtime',
    'time_nan','time_future','native','bulk','completion','capture','inputs','environment'])
def test_changed_registration_never_qualifies_retroactively(registered, kind):
    full, output, _, pin = registered
    path = output/'registration.json';doc = smoke.read(path)
    if kind == 'helper':doc['helper_sha256'] = '0'*64
    if kind == 'source':write(full, dict(changed=True))
    if kind == 'subset':write(output/'manifest.json', dict(changed=True))
    if kind == 'receipt':write(Path(doc['build_receipt']['path']), dict(changed=True))
    if kind == 'audit':write(output/'source_audit.json', dict(changed=True))
    if kind == 'selected':doc['selected'].reverse()
    if kind == 'coverage':doc['coverage']['eligibility']['excluded_count'] = 0
    if kind == 'runtime':write(pin, dict(changed=True))
    if kind == 'time_nan':doc['registered_at_unix'] = float('nan')
    if kind == 'time_future':doc['registered_at_unix'] = 10**15
    if kind == 'native':doc['native_qualified'] = True
    if kind == 'bulk':doc['bulk_ready'] = True
    if kind == 'completion':doc['completion_live_qualified'] = True
    if kind == 'capture':doc['capture_executed'] = True
    if kind == 'inputs':doc['input_hashes'] = {}
    if kind == 'environment':doc['environment'] = 'sunny13'
    write(path, doc)
    with pytest.raises((ValueError, RuntimeError)):
        smoke.registration_context(full, path)
