"""실제 native annotation/corner fixture로 원형-only 읽기 전용 보고서를 검증한다."""
import copy
from collections import Counter
import itertools
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import report_base5_unit_only_stability as report


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True);path.write_text(json.dumps(value)+'\n');return path


def jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows));return path


@pytest.fixture
def fixture(tmp_path, monkeypatch):
    names = ['Asset'+str(i) for i in range(19)]
    sizes = {name:[1.+i/100, 2., .5] for i,name in enumerate(names)}
    catalog = write(tmp_path/'catalog.json', dict(assets=[dict(name=name, data_id=100+i,
        source_list='objectList', semantic_class='obstacle') for i,name in enumerate(names)]))
    origin = write(tmp_path/'native_units.json', dict(assets={name:dict(data_id=100+i,
        source_list='objectList', scale_xyz=[1.,1.,1.], bbox_size_m=sizes[name]) for i,name in enumerate(names)}))
    origin_ref = dict(path=str(origin), sha256=report.sha256(origin), assets=names)
    policy = write(tmp_path/'unit_review.json', dict(schema=report.unit.SCHEMA, training_gt_modified=False,
        sources=[origin_ref], catalog_reference=dict(path=str(catalog), sha256=report.sha256(catalog)),
        assets={name:dict(data_id=100+i, source_list='objectList', raw_class='Obstacle',
            native_size_lwh_m=sizes[name], reference_source=origin_ref) for i,name in enumerate(names)}))
    reference = dict(path=str(policy), sha256=report.sha256(policy))
    runtime_pin = write(tmp_path/'runtime.json', {})
    runtime = lambda:{str(runtime_pin):report.sha256(runtime_pin)}
    monkeypatch.setattr(report, 'runtime_hashes', runtime)
    source_entries = [];source_meta = {};records = []
    for index in range(64):
        ident = 'source_'+str(index);actors=[];raw=[]
        for ordinal, offset in enumerate((0, 1)):
            which=(index+offset)%19;name=names[which];uid=8500+ordinal
            position=dict(x=index*10.+5.+ordinal*4, y=0., z=0.)
            actors.append(dict(name=name, unique_id=uid, data_id=100+which, source_list='objectList',
                motion='static', position=position, placement_yaw_deg=0., estimated_size=sizes[name]))
            raw.append(dict(UNIQUEID=uid, DataID=100+which, pos=position,
                rot=dict(roll=0.,pitch=0.,yaw=0.), scale=dict(x=1.,y=1.,z=1.)))
        meta=dict(spec=dict(family='obstacle', mission=False), runtime_actors=actors,
            ego=dict(x=index*10., y=0., z=0., yaw=0.), split='train')
        sp=write(tmp_path/'source'/'scenarios'/(ident+'.json'), dict(objectList=raw))
        mp=write(sp.with_name(ident+'_metadata.json'), meta)
        entry=dict(id=ident, scenario=str(sp), metadata=str(mp), scenario_sha256=report.sha256(sp), split='train')
        source_entries.append(entry);source_meta[ident]=meta
        records.append(dict(id=ident, meta=meta, scenario_sha256=entry['scenario_sha256']))
    all_entries=list(source_entries)
    for index in range(64,5000):all_entries.append(dict(source_entries[index%64], id='unused_'+str(index)))
    source=write(tmp_path/'source'/'manifest.json', dict(count=5000, scenes=all_entries, config={}))
    groups, fingerprints, _=report.layout.groups(records, report.layout.configuration())
    inputs={str(path):report.sha256(path) for path in (source, policy, origin, catalog)}
    for entry in source_entries:
        for key in ('scenario','metadata'):inputs[entry[key]]=report.sha256(entry[key])
    entries=[];selected=[];metadata={}
    for index in range(80):
        source_index=index if index<64 else index-64;original=source_entries[source_index]
        ident='unit_'+str(index);phase='independent' if index<64 else 'repeat'
        tag=dict(source_id=original['id'], source_scenario_sha256=original['scenario_sha256'],
            source_metadata_path=original['metadata'], source_metadata_sha256=report.sha256(original['metadata']),
            independent_group_id=groups[original['id']], geometry_sha256=fingerprints[original['id']],
            phase=phase, sequence_ordinal=index, independent_ordinal=source_index,
            repeat_of_id='unit_'+str(source_index) if index>=64 else None, repeat_ordinal=int(index>=64),
            payload_reused_exact=index>=64, native_outcomes_used_for_selection=False)
        meta=copy.deepcopy(source_meta[original['id']]);meta.update({report.unit.KEY:reference, report.preparation.KEY:tag})
        mp=write(tmp_path/'plan'/'metadata'/(ident+'.json'), meta)
        entry=dict(original, id=ident, metadata=str(mp));entries.append(entry);metadata[ident]=meta
        selected.append(dict(id=ident, scenario_sha256=entry['scenario_sha256'],
            metadata_sha256=report.sha256(mp), scenario=entry['scenario'], metadata=str(mp), **tag))
    summary=dict(schema=report.preparation.SCHEMA, policy=report.preparation.POLICY, registered_at_unix=10.,
        source_manifest=str(source), source_manifest_sha256=report.sha256(source), unit_reference=reference,
        runtime_hashes=runtime(), input_hashes=inputs, count=80, independent_frames=64, repeated_frames=16,
        selected=selected, capture_started=False, raw_modified=False, bulk_ready=False,
        training_started=False, native_outcomes_used_for_selection=False)
    deck=dict(count=80, scenes=entries, config={report.unit.KEY:reference},
        unit_only_stability_registration=summary, bulk_ready=False)
    manifest=write(tmp_path/'plan'/'manifest.json', deck)
    registration=write(tmp_path/'plan'/'registration.json', dict(summary, manifest=str(manifest),
        manifest_sha256=report.sha256(manifest)))
    return dict(root=tmp_path, manifest=manifest, registration=registration, deck=deck, metadata=metadata,
        runtime_pin=runtime_pin, policy=policy, reference=reference, source=source)


def capture_native(fixture, entry, session, *, multiplier=1., corner_multiplier=None, missing=False,
                   other_review=False, raw_name=None):
    meta=copy.deepcopy(fixture['metadata'][entry['id']]);meta['capture_runtime_ego']={'diagnostic':True}
    raw=session/'captures'/(raw_name or entry['id'])
    write(raw/'scenario.json', json.loads(Path(entry['scenario']).read_text()))
    write(raw/'metadata.json', meta)
    components=[];summary=[];corners=[]
    for ordinal,actor in enumerate(meta['runtime_actors']):
        if missing and ordinal==0:continue
        size=np.asarray(actor['estimated_size'])*multiplier
        measured=size if corner_multiplier is None else np.asarray(actor['estimated_size'])*corner_multiplier
        center=np.asarray([5.+ordinal*5., 1., 2.]);rpy=np.asarray([.02,-.03,.4])
        uid=actor['unique_id'];vertices=np.asarray(list(itertools.product((-1.,1.),repeat=3)))*measured/2
        vertices=vertices @ report.native._rotation(rpy).T+center
        summary.append(' '.join(['Obstacle',*map(str,center),*map(str,rpy),*map(str,size),'5','0','0','0',str(uid)]))
        corners.append(' '.join(['Obstacle',*map(str,vertices.reshape(-1)),*map(str,rpy),'5','0','0','0',str(uid)]))
        components.append(dict(parent_uid=uid, component_id=f'{uid}:Obstacle:0', asset=actor['name'],
            data_id=actor['data_id'], raw_class='Obstacle', size_m=size.tolist(),
            center_lidar_m=center.tolist(), rotation_rpy=rpy.tolist()))
    annotation=raw/'LIDAR_5'/'timestamp_Instance.txt';annotation.parent.mkdir(parents=True)
    annotation.write_text('\n'.join(summary)+'\n')
    annotation.with_name('timestamp_Instance_8Points.txt').write_text('\n'.join(corners)+'\n')
    audit=dict(components=components)
    audit['native_unit_geometry_review']=report.unit.review(meta,audit)
    audit['review_reasons']=audit['native_unit_geometry_review']['review_reasons']+(['visibility_review'] if other_review else [])
    audit['accepted_candidate']=not audit['review_reasons']
    write(raw/'audit.json',audit)
    return dict(id=entry['id'], scenario_sha256=entry['scenario_sha256'], capture_root=str(raw),
        status='accepted_candidate' if audit['accepted_candidate'] else 'review', audit=audit, duration_sec=2.)


def make_session(fixture, indices, *, name='session', options=None):
    session=fixture['root']/'capture'/name
    entries=[fixture['deck']['scenes'][i] for i in indices]
    runner=dict(shared_ros_batch=False,selected_scene_ids=[entry['id'] for entry in entries])
    report.prepare_session(session, fixture['manifest'],
        report.pins.ROOT/'configs/network/ego_network_mr_base5_savesensordata_collision_local.json',
        report.pins.ROOT/'configs/sensors/Sensor_competition_local.json', runner_options=runner)
    rows=[]
    for entry in entries:rows.append(capture_native(fixture,entry,session,**((options or {}).get(entry['id'],{}))))
    ledger=jsonl(session/'ledger.jsonl',rows)
    jsonl(session/'event_cmd_requests.jsonl',[dict(scene=row['id'],recorded_at=100.+position,phase='park')
        for position,row in enumerate(rows)])
    write(session/'shutdown_safety.json',dict(park_verified=True))
    write(session/'status.json',dict(stage='complete',requested=len(entries)))
    return ledger,rows


def test_context_exact64_plus16_source_identity_and_qa19(fixture):
    context=report.load_context(fixture['manifest'])
    assert len(context['entries'])==80 and len(context['policy']['assets'])==19
    assert context['scale_counts']=={'objectList/explicit111':160}


def test_report_includes_review_native_and_actual_corner_statistics_without_raw_mutation(fixture):
    ledger,rows=make_session(fixture,[0,1],options={'unit_0':dict(other_review=True)})
    before={p:p.read_bytes() for p in fixture['root'].rglob('*') if p.is_file()}
    path=report.report(fixture['manifest'],[ledger],fixture['root']/'result.json')
    value=json.loads(path.read_text());main=value['main']
    assert main['initial_status_counts']==dict(review=1,accepted_candidate=1)
    assert main['initial_dimension_review_scenes']==0 and main['initial_other_qa_review_scenes']==1
    assert main['assets']['Asset0']['compared_initial_parent_occurrences']==1
    assert main['assets']['Asset0']['absolute_axis_error_m']['maximum']==pytest.approx([0.,0.,0.],abs=1e-6)
    assert len(main['unobserved_registered_ids'])==78 and main['terminal_park_verified']
    assert value['bulk_ready'] is False and value['completion_live_qualified'] is False
    assert value['native_gt_modified'] is False
    assert all(p.read_bytes()==data for p,data in before.items())


def test_missing_component_is_not_a_zero_error_compared_parent(fixture):
    ledger,_=make_session(fixture,[0],options={'unit_0':dict(missing=True)})
    value=json.loads(report.report(fixture['manifest'],[ledger],fixture['root']/'result.json').read_text())
    asset=value['main']['assets']['Asset0']
    assert asset['expected_initial_parent_occurrences']==1 and asset['observed_initial_parent_occurrences']==0
    assert asset['compared_initial_parent_occurrences']==0 and asset['not_compared_initial_parent_occurrences']==1
    assert asset['missing_native_parent_occurrences']==1 and asset['absolute_axis_error_m']['maximum'] is None
    assert value['main']['initial_dimension_review_reason_counts']==dict(unit_native_component_contract_changed=1)
    saved=value['main']['observations'][0]['saved_unit_review']
    assert saved['actors'][0]['compared'] is False
    assert saved['actors'][0]['reason']=='not_single_physical_obstacle_component'


def test_native_dimension_mismatch_and_unrelated_review_are_distinct_axes(fixture):
    ledger,_=make_session(fixture,[0,1],options={'unit_0':dict(multiplier=1.25),'unit_1':dict(other_review=True)})
    main=json.loads(report.report(fixture['manifest'],[ledger],fixture['root']/'result.json').read_text())['main']
    assert main['initial_dimension_review_scenes']==1 and main['initial_other_qa_review_scenes']==1
    assert main['assets']['Asset0']['native_reference_mismatch_parents']==1
    assert main['assets']['Asset0']['absolute_axis_error_m']['maximum']==pytest.approx([.25,.5,.125],abs=1e-6)


def test_corner_summary_contradiction_is_diagnostic_not_gt_repair(fixture):
    ledger,rows=make_session(fixture,[0],options={'unit_0':dict(corner_multiplier=1.25)})
    before=Path(rows[0]['capture_root'])/'audit.json';saved=before.read_bytes()
    main=json.loads(report.report(fixture['manifest'],[ledger],fixture['root']/'result.json').read_text())['main']
    assert main['initial_status_counts']==dict(accepted_candidate=1)
    assert main['initial_dimension_review_scenes']==1
    assert 'unit_native_corners_summary_mismatch' in main['observations'][0]['dimension_review_reasons']
    assert main['assets']['Asset0']['audit_corner_absolute_axis_error_m']['maximum']==pytest.approx([.25,.5,.125],abs=1e-6)
    assert before.read_bytes()==saved


def test_exact_payload_repeat_delta_does_not_add_independent_geometries(fixture):
    ledger,_=make_session(fixture,[0,64],options={'unit_64':dict(multiplier=1.25)})
    main=json.loads(report.report(fixture['manifest'],[ledger],fixture['root']/'result.json').read_text())['main']
    assert main['independent_initial_scenes']==main['repeat_initial_scenes']==1
    assert main['assets']['Asset0']['independent_geometry_groups']==1
    repeat=main['repeat_comparisons'][0]
    assert repeat['both_initial_native_observed'] and repeat['compared_parents']==2
    assert repeat['independent_samples_added']==0
    assert repeat['parents'][0]['absolute_corner_size_delta_m']==pytest.approx([.25,.5,.125],abs=1e-6)
    assert repeat['expected_parent_uids']==[8500,8501] and repeat['all_expected_compared']
    independent=main['assets_by_phase']['independent']['Asset0']
    repeated=main['assets_by_phase']['repeat']['Asset0']
    assert independent['compared_initial_parent_occurrences']==repeated['compared_initial_parent_occurrences']==1
    assert independent['absolute_axis_error_m']['maximum']==pytest.approx([0.,0.,0.],abs=1e-6)
    assert repeated['absolute_axis_error_m']['maximum']==pytest.approx([.25,.5,.125],abs=1e-6)
    assert independent['independent_geometry_groups']==1 and repeated['independent_geometry_groups']==0


def test_repeat_missing_same_parent_both_sides_is_not_complete_comparison(fixture):
    ledger,_=make_session(fixture,[0,64],options={'unit_0':dict(missing=True),'unit_64':dict(missing=True)})
    main=json.loads(report.report(fixture['manifest'],[ledger],fixture['root']/'result.json').read_text())['main']
    repeat=main['repeat_comparisons'][0]
    assert repeat['both_initial_native_observed'] and not repeat['all_expected_compared']
    assert repeat['unmatched_parent_uids']==[] and repeat['compared_parents']==1
    for side in ('before','after'):
        assert repeat[side]['expected_parents']==2
        assert repeat[side]['observed_parent_uids']==repeat[side]['compared_parent_uids']==[8501]
        assert repeat[side]['missing_parent_uids']==repeat[side]['not_compared_parent_uids']==[8500]
        assert not repeat[side]['all_expected_compared']


def test_failed_first_attempt_retry_and_unobserved_have_separate_denominators(fixture):
    ledger,rows=make_session(fixture,[0]);retry=copy.deepcopy(rows[0])
    failure=dict(id=rows[0]['id'],scenario_sha256=rows[0]['scenario_sha256'],status='failed',error='export unavailable')
    jsonl(ledger,[failure,retry])
    main=json.loads(report.report(fixture['manifest'],[ledger],fixture['root']/'result.json').read_text())['main']
    assert main['initial_status_counts']==dict(failed=1) and main['retry_status_counts']==dict(accepted_candidate=1)
    assert main['retry_attempts']==1 and len(main['unobserved_registered_ids'])==79
    assert main['initial_attempted_scenes']==1 and main['initial_native_available_scenes']==0
    assert main['initial_native_missing_or_failed_scenes']==1
    asset=main['assets']['Asset0']
    assert asset['failed_attempt_parent_occurrences']==1 and asset['compared_initial_parent_occurrences']==0
    assert asset['missing_native_parent_occurrences']==0 and asset['absolute_axis_error_m']['count']==0


@pytest.mark.parametrize('kind',['audit','raw_scenario','actor_name','actor_order','native_summary','native_corner',
    'missing_companion','forged_unit_review','park','event_time','event_nan','event_scene','session_runtime','session_manifest',
    'selected_ledger','duplicate_ledger','torn_ledger','foreign_id','foreign_root'])
def test_provenance_or_safety_contradiction_is_strict_error(fixture,kind):
    ledger,rows=make_session(fixture,[0]);raw=Path(rows[0]['capture_root']);paths=[ledger]
    if kind=='audit':
        audit=json.loads((raw/'audit.json').read_text());audit['extra']=True;write(raw/'audit.json',audit)
    if kind=='raw_scenario':write(raw/'scenario.json',dict(changed=True))
    if kind in ('actor_name','actor_order'):
        meta=json.loads((raw/'metadata.json').read_text())
        if kind=='actor_name':meta['runtime_actors'][0]['name']='HiddenAsset'
        else:meta['runtime_actors'].reverse()
        write(raw/'metadata.json',meta)
    if kind=='native_summary':
        p=raw/'LIDAR_5/timestamp_Instance.txt';fields=p.read_text().splitlines();parts=fields[0].split();parts[7]='99'
        fields[0]=' '.join(parts);p.write_text('\n'.join(fields)+'\n')
    if kind=='native_corner':
        p=raw/'LIDAR_5/timestamp_Instance_8Points.txt';fields=p.read_text().splitlines();parts=fields[0].split();parts[1]='99'
        fields[0]=' '.join(parts);p.write_text('\n'.join(fields)+'\n')
    if kind=='missing_companion':(raw/'LIDAR_5/timestamp_Instance_8Points.txt').unlink()
    if kind=='forged_unit_review':
        audit=json.loads((raw/'audit.json').read_text());audit['native_unit_geometry_review']['actors'][0]['matched']=False
        write(raw/'audit.json',audit);rows[0]['audit']=audit;jsonl(ledger,rows)
    if kind=='park':write(ledger.parent/'shutdown_safety.json',dict(park_verified=False))
    if kind.startswith('event_'):
        event=dict(scene=rows[0]['id'],recorded_at=100.)
        if kind=='event_time':event['recorded_at']=1.
        if kind=='event_nan':event['recorded_at']=float('nan')
        if kind=='event_scene':event['scene']='unrelated_scene'
        jsonl(ledger.parent/'event_cmd_requests.jsonl',[event])
    if kind.startswith('session_'):
        path=ledger.parent/'session_contract.json';value=json.loads(path.read_text())
        if kind=='session_runtime':value['safety_code_sha256']['base5_unit_geometry_contract.py']='0'*64
        else:value['manifest_sha256']='0'*64
        write(path,value)
    if kind=='selected_ledger':paths=[jsonl(ledger.with_name('selected_ledger.jsonl'),rows)]
    if kind=='duplicate_ledger':paths=[ledger,ledger]
    if kind=='torn_ledger':ledger.write_bytes(ledger.read_bytes().rstrip(b'\n'))
    if kind=='foreign_id':rows[0]['id']='foreign';jsonl(ledger,rows)
    if kind=='foreign_root':rows[0]['capture_root']=str(fixture['root']);jsonl(ledger,rows)
    with pytest.raises((ValueError,RuntimeError,KeyError)):
        report.report(fixture['manifest'],paths,fixture['root']/'result.json')
    assert not (fixture['root']/'result.json').exists()


@pytest.mark.parametrize('kind',['summary','extra_scene','source','registration_sha','metadata','repeat_geometry','runtime','reference','nonunit'])
def test_registration_source_and_geometry_tamper_fail_closed(fixture,kind):
    manifest=fixture['manifest'];deck=json.loads(manifest.read_text());registration=fixture['registration']
    doc=json.loads(registration.read_text())
    if kind=='summary':deck['unit_only_stability_registration']['count']=79;write(manifest,deck)
    if kind=='extra_scene':
        deck['scenes'].append(dict(deck['scenes'][-1],id='extra_scene'));deck['count']=81;write(manifest,deck)
        doc['manifest_sha256']=report.sha256(manifest);write(registration,doc)
    if kind=='source':write(fixture['source'],{})
    if kind=='registration_sha':doc['manifest_sha256']='0'*64;write(registration,doc)
    if kind in ('metadata','repeat_geometry'):
        entry=deck['scenes'][0 if kind=='metadata' else 64];meta=json.loads(Path(entry['metadata']).read_text())
        meta['runtime_actors'][0]['position']['x']+=1.;write(Path(entry['metadata']),meta)
    if kind=='runtime':write(fixture['runtime_pin'],dict(changed=True))
    if kind=='reference':write(fixture['policy'],{})
    if kind=='nonunit':
        entry=deck['scenes'][0];value=json.loads(Path(entry['scenario']).read_text());value['objectList'][0]['scale']['x']=1.25
        write(Path(entry['scenario']),value)
    with pytest.raises((ValueError,RuntimeError,KeyError)):report.load_context(manifest)


def test_nonobject_missing_scale_is_default_but_missing_object_scale_is_invalid(fixture):
    entry=fixture['deck']['scenes'][0];scene=json.loads(Path(entry['scenario']).read_text())
    meta=copy.deepcopy(fixture['metadata'][entry['id']]);catalog={100:dict(name='Asset0',source_list='objectList'),
        101:dict(name='Asset1',source_list='objectList')}
    scene['objectList'][1].pop('scale');scene['vehicleList']=[scene['objectList'].pop()]
    meta['runtime_actors'][1]['source_list']='vehicleList';catalog[101]['source_list']='vehicleList'
    counts=report.bind_unit(scene,meta,catalog)
    assert counts['vehicleList/serializer_default_no_override']==1
    scene['objectList'][0].pop('scale')
    with pytest.raises(ValueError):report.bind_unit(scene,meta,catalog)


def test_source_pin_race_is_rejected_before_any_report_is_written(fixture,monkeypatch):
    ledger,rows=make_session(fixture,[0]);original=report.summarize
    def mutate(context,observations):
        result=original(context,observations)
        write(Path(rows[0]['capture_root'])/'audit.json',dict(changed=True))
        return result
    monkeypatch.setattr(report,'summarize',mutate)
    with pytest.raises(ValueError):report.report(fixture['manifest'],[ledger],fixture['root']/'result.json')
    assert not (fixture['root']/'result.json').exists()


def test_existing_control_is_separate_and_never_acquires_modern_registration(fixture):
    main_ledger,_=make_session(fixture,[0],name='main')
    control=copy.deepcopy(fixture);control['deck']=copy.deepcopy(fixture['deck'])
    control['deck'].pop(report.REGISTRATION)
    control['deck'].update(count=4,scenes=control['deck']['scenes'][:4],all_physical_unit_registration=dict(source_files={}))
    control['manifest']=write(fixture['root']/'control_plan'/'manifest.json',control['deck'])
    control_ledger,_=make_session(control,[0,1,2,3],name='cold4')
    result=json.loads(report.report(fixture['manifest'],[main_ledger],fixture['root']/'result.json',
        control_cohorts=[(control['manifest'],control_ledger)]).read_text())
    assert result['main']['initial_observed_scenes']==1
    assert result['separate_controls'][0]['initial_observed_scenes']==4
    assert result['separate_controls'][0]['registered_at_verified'] is False
    assert result['separate_controls'][0]['modern_registration'] is False
    assert result['main']['independent_initial_scenes']==1 and not result['bulk_ready']
