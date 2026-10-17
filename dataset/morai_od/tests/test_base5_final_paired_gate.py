"""60쌍 계획 재현과 모든 최초 시도·비용·안전 gate의 합성 회귀 검사."""
import copy
from collections import Counter
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_base5_final_bounded_smoke as smoke
import finalize_base5_final_bounded_release as release


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value)+'\n');return path


def jsonl(path,rows):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows));return path


def paired_fixture(tmp_path,monkeypatch):
    reference=write(tmp_path/'unit_reference.json',dict(fixture=True))
    refs={str(reference):smoke.sha256(reference)}
    full=write(tmp_path/'full'/'manifest.json',dict(fixture='full5K API boundary'))
    profiles=[('CargoBox','iso300')]+[('asset'+str(index),'iso150') for index in range(16)]
    source=dict(pairs=set(profiles),entries={},metadata={},overlay={})
    entries=[];baselines=[];profile_counts=Counter()
    for index in range(60):
        source_id='source_'+str(index);name,profile=profiles[0] if index<3 else profiles[1+(index-3)%16]
        source_meta=dict(runtime_actors=[dict(name=name,asset_scale=dict(profile_id=profile))])
        source['metadata'][source_id]=source_meta;source['entries'][source_id]=dict(id=source_id)
        profile_counts[name+'/'+profile]+=1
        order=('original','scaled') if index%2==0 else ('scaled','original')
        for ordinal,condition in enumerate(order):
            ident=f'stability_{index:03d}_{condition}_{source_id}'
            actor=dict(name=name,unique_id=8500,data_id=40100001,source_list='objectList',motion='static')
            if condition=='scaled':actor['asset_scale']=dict(profile_id=profile)
            meta=dict(spec=dict(family='obstacle'),runtime_actors=[actor],split='train',
                **{smoke.unit.KEY:dict(path=str(reference),sha256=smoke.sha256(reference))},
                scale_stability=dict(pair_id='pair_'+str(index),condition=condition,pair_order='/'.join(order),
                    within_pair_position=ordinal,source_id=source_id,unit_reference_sources=refs,
                    independent_geometry_unit=source_id))
            scenario=dict(objectList=[dict(UNIQUEID=8500,DataID=40100001,pos=dict(x=index,y=0.,z=0.))])
            path=write(tmp_path/'paired'/'scenarios'/(ident+'.json'),scenario)
            metadata=write(path.with_name(ident+'_metadata.json'),meta)
            entry=dict(id=ident,scenario=str(path),metadata=str(metadata),scenario_sha256=smoke.sha256(path))
            entries.append(entry);baselines.append((copy.deepcopy(entry),scenario,meta))
    deck=dict(count=120,scenes=entries,config={smoke.unit.KEY:dict(path=str(reference),sha256=smoke.sha256(reference))},
        scale_stability_registration=dict(source_manifest=str(full),source_files={str(full):smoke.sha256(full),**refs},
            preparation_script_sha256=smoke.sha256(smoke.stability.__file__),independent_pairs=60,first_attempt_captures=120,
            selection_uses_native_outcomes=False,raw_files_modified=False,source_unchanged=True,
            balanced_order_counts=dict(original_scaled=30,scaled_original=30),
            coverage=dict(minimum_independent_scenes_per_profile=3,profile_scene_counts=dict(profile_counts))))
    path=write(tmp_path/'paired'/'manifest.json',deck);baseline_deck=copy.deepcopy(deck)
    def replay(manifest,output,unit_references,*,pairs,minimum_per_profile):
        assert Path(manifest)==full and unit_references==sorted(refs) and (pairs,minimum_per_profile)==(60,3)
        result=copy.deepcopy(baseline_deck);result['scenes']=[]
        for entry,scenario,meta in baselines:
            entry=copy.deepcopy(entry)
            scenario_path=write(Path(output)/'scenarios'/(entry['id']+'.json'),scenario)
            metadata_path=write(scenario_path.with_name(entry['id']+'_metadata.json'),meta)
            entry.update(scenario=str(scenario_path),metadata=str(metadata_path))
            result['scenes'].append(entry)
        return write(Path(output)/'manifest.json',result)
    monkeypatch.setattr(smoke.stability,'prepare',replay)
    monkeypatch.setattr(smoke.layout,'resolve',lambda overlay,ident,*args:dict(group_id=ident,split='train'))
    return full,path,deck,source,reference


def test_paired_plan_is_replayed_with_exact_source_geometry_uids_ab_and_all_rare_inventory(tmp_path,monkeypatch):
    full,path,_,source,_=paired_fixture(tmp_path,monkeypatch)
    context=smoke.stability_context(path,full,source)
    assert len(context['paired'])==60 and len(context['entries'])==120
    assert context['rare_source_ids']==['source_0','source_1','source_2']
    assert len(context['profile_scene_counts'])==17 and min(context['profile_scene_counts'].values())==3
    assert context['gate']['minimum_initial_approval_fraction']==.90
    assert context['gate']['maximum_scaled_nonaccept_excess']==.05
    assert context['gate']['maximum_scaled_cost_per_approval_ratio']==1.10


@pytest.mark.parametrize('kind',['count','source','helper','minimum','uids','metadata','geometry','order','config',
    'reference','duplicate_geometry','rare_not_full','profile_shortage'])
def test_paired_plan_cannot_be_edited_or_reinterpreted_after_registration(tmp_path,monkeypatch,kind):
    full,path,deck,source,reference=paired_fixture(tmp_path,monkeypatch)
    registration=deck['scale_stability_registration']
    if kind=='count':deck['count']=119
    if kind=='source':registration['source_manifest']=str(tmp_path/'foreign.json')
    if kind=='helper':registration['preparation_script_sha256']='0'*64
    if kind=='minimum':registration['coverage']['minimum_independent_scenes_per_profile']=2
    if kind in ('uids','geometry'):
        entry=deck['scenes'][0];scenario=smoke.read(entry['scenario'])
        if kind=='uids':scenario['objectList'][0]['UNIQUEID']=9999
        else:scenario['objectList'][0]['pos']['x']+=.01
        write(Path(entry['scenario']),scenario);entry['scenario_sha256']=smoke.sha256(entry['scenario'])
    if kind=='metadata':
        entry=deck['scenes'][0];meta=smoke.read(entry['metadata']);meta['scale_stability']['within_pair_position']=99
        write(Path(entry['metadata']),meta)
    if kind=='order':deck['scenes'][0],deck['scenes'][1]=deck['scenes'][1],deck['scenes'][0]
    if kind=='config':deck['config']['qa_waiver']=True
    if kind=='reference':write(reference,dict(changed=True))
    if kind=='duplicate_geometry':monkeypatch.setattr(smoke.layout,'resolve',lambda *args:dict(group_id='same'))
    if kind=='rare_not_full':source['metadata']['unselected_rare']=copy.deepcopy(source['metadata']['source_0'])
    if kind=='profile_shortage':source['pairs'].add(('unobserved_asset','unobserved_profile'))
    write(path,deck)
    with pytest.raises(ValueError):smoke.stability_context(path,full,source)


def native_fixture(tmp_path,monkeypatch):
    full,path,_,source,_=paired_fixture(tmp_path,monkeypatch)
    paired=smoke.stability_context(path,full,source)
    root=tmp_path/'paired_native';session=root/'session_0';rows=[];events=[]
    for index,entry in enumerate(paired['deck']['scenes']):
        ident=entry['id'];raw=session/'captures'/ident
        write(raw/'scenario.json',smoke.read(entry['scenario']))
        write(raw/'metadata.json',paired['metadata'][ident])
        audit=dict(components=[],review_reasons=[],accepted_candidate=True,asset_scale_review={})
        write(raw/'audit.json',audit)
        samples=[dict(t=t,message=dict(collision_object=[dict(unique_id=0)],collision_objecta=[])) for t in (0.,.5,1.)]
        write(raw/'collision_history.json',dict(summary=release.collision_window(samples,0.,1.),samples=samples))
        rows.append(dict(id=ident,scenario_sha256=entry['scenario_sha256'],status='accepted_candidate',duration_sec=10.,
            capture_root=str(raw),audit=audit))
        events.append(dict(scene=ident,recorded_at=float(index+1),phase='park'))
    ledger=jsonl(session/'ledger.jsonl',rows)
    jsonl(session/'event_cmd_requests.jsonl',events)
    write(session/'shutdown_safety.json',dict(park_verified=True))
    write(session/'status.json',dict(stage='complete',requested=120))
    session_contract=dict(manifest_sha256=smoke.sha256(path),sensor_sha256=smoke.sha256(smoke.pins.ROOT/'configs/sensors/Sensor_competition_local.json'),
        **{smoke.unit.KEY:paired['deck']['config'][smoke.unit.KEY]},
        runner_options=dict(shared_ros_batch=False,selected_scene_ids=[entry['id'] for entry in paired['deck']['scenes']]),
        safety_code_sha256={'base5_unit_capture_provenance.py':smoke.sha256(release.unit_provenance.__file__)})
    write(session/'session_contract.json',session_contract)
    monkeypatch.setattr(release.unit_provenance,'verify_capture',lambda *args:None)
    monkeypatch.setattr(release.scale_capture,'verify_capture',lambda *args:None)
    monkeypatch.setattr(release.scale_capture,'native_size_review',lambda *args:{})
    native_calls=[];monkeypatch.setattr(release.training,'assert_native',lambda *args:native_calls.append(args))
    monkeypatch.setattr(release,'verify_stability_session',lambda *args:paired['manifest'])
    context=dict(paired_stability=paired,pairs=source['pairs'],doc=dict(registered_at_unix=.5))
    return context,root,ledger,rows,native_calls


def test_all120_native_first_attempts_are_recounted_from_raw_ledgers_not_summary(tmp_path,monkeypatch):
    context,root,ledger,_,native=native_fixture(tmp_path,monkeypatch)
    result,blockers=release.verify_stability(context,root,[ledger])
    assert not blockers and len(native)==120
    assert result['first_attempt_observations']==120 and result['independent_pairs']==60
    assert result['metrics']['original']['initial_approval_fraction']==1.
    assert result['metrics']['scaled_cost_per_approval_ratio']==1.
    assert len(result['profiles'])==17 and result['raw_native_approvals_counted_as_final_smoke']==0
    assert result['recomputed_report']['initial_failure_cause_counts']=={}


@pytest.mark.parametrize('kind',['missing','order','duplicate_ledger','orphan_session','orphan_ledger','park','halted',
    'foreign_metadata','foreign_scenario','collision','missing_timing','session_ids','safety_helper','event_reorder','shared_park',
    'pre_registration'])
def test_missing_repeated_foreign_or_unsafe_paired_evidence_cannot_qualify(tmp_path,monkeypatch,kind):
    context,root,ledger,rows,_=native_fixture(tmp_path,monkeypatch)
    session=ledger.parent;paths=[ledger]
    if kind=='missing':rows.pop()
    if kind=='order':rows[0],rows[1]=rows[1],rows[0]
    if kind=='duplicate_ledger':paths.append(ledger)
    if kind=='orphan_session':write(root/'orphan'/'session_contract.json',{})
    if kind=='orphan_ledger':jsonl(root/'orphan'/'ledger.jsonl',[rows[0]])
    if kind=='park':write(session/'shutdown_safety.json',dict(park_verified=False))
    if kind=='halted':write(session/'status.json',dict(stage='halted',requested=120))
    if kind=='foreign_metadata':
        raw=Path(rows[0]['capture_root']);meta=release.read(raw/'metadata.json');meta['spec']['family']='foreign'
        write(raw/'metadata.json',meta)
    if kind=='foreign_scenario':
        # The real unit/scale validators own raw geometry parsing; this test
        # checks the caller does not swallow their hard source mismatch.
        def reject(*args):raise ValueError('captured scenario changed')
        monkeypatch.setattr(release.unit_provenance,'verify_capture',reject)
    if kind=='collision':
        raw=Path(rows[0]['capture_root']);value=release.read(raw/'collision_history.json');value['samples']=[]
        write(raw/'collision_history.json',value)
    if kind=='missing_timing':rows[0].pop('duration_sec')
    if kind=='session_ids':
        doc=release.read(session/'session_contract.json');doc['runner_options']['selected_scene_ids'].pop();write(session/'session_contract.json',doc)
    if kind=='safety_helper':
        doc=release.read(session/'session_contract.json');doc['safety_code_sha256']['base5_unit_capture_provenance.py']='0'*64
        write(session/'session_contract.json',doc)
    if kind=='event_reorder':
        jsonl(session/'event_cmd_requests.jsonl',[dict(recorded_at=float('nan'))])
    if kind=='shared_park':
        doc=release.read(session/'session_contract.json');doc['runner_options']['shared_ros_batch']=True
        write(session/'session_contract.json',doc)
    if kind=='pre_registration':context['doc']['registered_at_unix']=1000.
    jsonl(ledger,rows)
    with pytest.raises((ValueError,KeyError)):release.verify_stability(context,root,paths)


def test_failed_initial_attempt_is_unknown_and_cannot_be_hidden_by_successful_retry(tmp_path,monkeypatch):
    context,root,ledger,rows,_=native_fixture(tmp_path,monkeypatch)
    first=rows[0];retry=copy.deepcopy(first);retry['capture_root']=str(ledger.parent/'captures'/'retry_0')
    retry_raw=Path(retry['capture_root']);original=Path(first['capture_root'])
    for path in original.iterdir():write(retry_raw/path.name,release.read(path))
    first.update(status='failed',ordinary_deficit=False,error='annotation export unavailable')
    first.pop('capture_root');first.pop('audit')
    rows.append(retry);jsonl(ledger,rows)
    result,blockers=release.verify_stability(context,root,[ledger])
    assert result['recomputed_report']['retry_attempts']==1
    assert result['initial_failure_causes_by_condition']['original']==dict(unknown=1)
    assert result['metrics']['original']['latest_approved']==60
    assert result['metrics']['original']['all_attempt_ledger_sec']==610.
    assert 'paired/failed_attempt_requires_review' in blockers
    assert 'paired/nonordinary_configuration_or_safety_failure' in blockers


def test_cost_plus10pct_is_a_real_profile_and_overall_hold_not_rounded_pass(tmp_path,monkeypatch):
    context,root,ledger,rows,_=native_fixture(tmp_path,monkeypatch)
    for row in rows:
        if context['paired_stability']['metadata'][row['id']]['scale_stability']['condition']=='scaled':row['duration_sec']=11.01
    jsonl(ledger,rows)
    _,blockers=release.verify_stability(context,root,[ledger])
    assert 'paired/cost_per_approval_above_110pct' in blockers
    assert any(value.startswith('paired_profile_review/CargoBox/iso300/cost') for value in blockers)


def test_thresholds_use_first_attempts_and_exact_boundaries_not_latest_status():
    metadata={f'{condition}_{index}':dict(scale_stability=dict(condition=condition))
        for condition in ('original','scaled') for index in range(60)}
    rows=[dict(id=ident,status='accepted_candidate',duration_sec=10.) for ident in metadata]
    for row in rows:
        if row['id'].startswith('scaled_') and int(row['id'].split('_')[1])<3:row['status']='review'
    metrics=release.paired_cost(rows,metadata)
    assert metrics['scaled']['first_nonaccept_fraction']==pytest.approx(.05)
    assert not release.stability_threshold_blockers(metrics)
    rows[63]['status']='review'
    assert 'paired/nonaccept_excess_above_5pp' in release.stability_threshold_blockers(release.paired_cost(rows,metadata))
    for row in rows:
        if row['id'].startswith('original_') and int(row['id'].split('_')[1])<7:row['status']='review'
    assert 'paired/initial_approval_below_90pct/original' in release.stability_threshold_blockers(release.paired_cost(rows,metadata))


@pytest.mark.parametrize('kind',['valid','foreign_manifest','changed_config','missing_safety','changed_runner','extra_runner'])
def test_complete_session_contract_and_exact_paired_subset_are_reproduced(tmp_path,monkeypatch,kind):
    full,path,deck,source,_=paired_fixture(tmp_path,monkeypatch)
    paired=smoke.stability_context(path,full,source)
    selected=[entry['id'] for entry in deck['scenes'][:4]]
    session=dict(manifest_sha256=smoke.sha256(path),runner_options=dict(shared_ros_batch=False,selected_scene_ids=selected),
        safety_code_sha256={'fixture':'complete-current-contract'},safety_policy='strict-current-policy')
    baseline=copy.deepcopy(session)
    def prepare(output,manifest,network,sensor,runner_options):
        assert Path(manifest)==path and runner_options==baseline['runner_options']
        write(Path(output)/'session_contract.json',baseline)
    monkeypatch.setattr(release,'prepare_session',prepare)
    if kind=='foreign_manifest':session['manifest_sha256']='0'*64
    if kind=='changed_config':
        deck['config']['waiver']=True;write(path,deck);session['manifest_sha256']=smoke.sha256(path)
    if kind=='missing_safety':session['safety_code_sha256']={}
    if kind=='changed_runner':session['runner_options']['optimized_adapter']=dict(profile='static-ready',adapter_sha256='0'*64)
    if kind=='extra_runner':session['runner_options']['qa_waiver']=True
    if kind=='valid':assert release.verify_stability_session(session,paired,[path])==path
    else:
        with pytest.raises(ValueError):release.verify_stability_session(session,paired,[path])
