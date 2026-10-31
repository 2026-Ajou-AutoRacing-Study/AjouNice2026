"""v155 final receipt guards; temporary evidence and helper-boundary mocks only."""
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import finalize_base5_scale_extent_release as release


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value)+'\n');return path


def jsonl(path,rows):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows));return path


def junit(path,count=1000,*,failure=False,prefix='test'):
    root=ET.Element('testsuites');suite=ET.SubElement(root,'testsuite',tests=str(count),failures=str(int(failure)),errors='0',skipped='0')
    for i in range(count):
        case=ET.SubElement(suite,'testcase',classname='CPU',name=prefix+str(i))
        if i==0 and failure:ET.SubElement(case,'failure')
    ET.ElementTree(root).write(path);return path


def test_junit_requires_1000_actual_unique_passes_and_no_failures(tmp_path):
    good=junit(tmp_path/'good.xml')
    assert release.test_results([good])['unique_passed']==1000
    short=junit(tmp_path/'short.xml',999)
    with pytest.raises(ValueError):release.test_results([short])
    # Overlapping receipts cannot inflate two groups of 600 to 1200 tests.
    a=junit(tmp_path/'a.xml',600);b=junit(tmp_path/'b.xml',600)
    with pytest.raises(ValueError):release.test_results([a,b])
    bad=junit(tmp_path/'bad.xml',1000,failure=True)
    with pytest.raises(ValueError):release.test_results([good,bad])
    fake=tmp_path/'fake.xml';fake.write_text('<testsuites><testsuite tests="2000" failures="0" errors="0"/></testsuites>')
    with pytest.raises(ValueError):release.test_results([fake])


def registration_fixture(tmp_path,monkeypatch):
    # Input/runtime discovery and scale selection have their own full-source
    # suites. At those explicit API boundaries use small pinned fixture files;
    # the finalizer still checks 5000 unique logical slots and exact 17 entries.
    pin=write(tmp_path/'pin.json',{})
    hashes={str(pin):release.sha256(pin)}
    meta=write(tmp_path/'metadata.json',dict(spec=dict(family='obstacle'),runtime_actors=[]))
    scenario=write(tmp_path/'scenario.json',{})
    entries=[dict(id='slot_'+str(i),scenario=str(scenario),metadata=str(meta),scenario_sha256=release.sha256(scenario)) for i in range(5000)]
    original=write(tmp_path/'original.json',dict(count=5000,scenes=entries))
    registry=write(tmp_path/'shadow_registry.json',dict(entries=[dict(source_id='slot_0',mode='exact_source'),dict(source_id='slot_1',mode='exact_source')]))
    policy=dict(fixture='policy')
    deck=dict(count=5000,scenes=entries,config={release.scale.CONFIG_KEY:policy,release.reuse.KEY:dict(registration=dict(path=str(registry)))},
        provenance=dict(asset_scale=dict(source_manifest=str(original),source_manifest_sha256=release.sha256(original))))
    full=write(tmp_path/'full'/'manifest.json',deck)
    pairs={(name,profile) for name in release.smoke.APPROVED_ASSETS for profile in ('125','150')}
    required={'profile/'+name+'/'+profile for name,profile in pairs}
    required.update('extent_stratum/'+v for v in ('before','intersection','after'))
    required.update('extent_phase/'+v for v in release.smoke.PHASES)
    required.update(('extent_split/train','extent_split/val','scaled_context/generic'))
    optional={'scaled_context/'+v for v in release.smoke.CONTEXTS}|{'scaled_role/primary','scaled_role/auxiliary'}
    coverage=dict(required=sorted(required),covered=sorted(required),unavailable_scale_context_or_roles=sorted(optional-required),
        protected_frames=8,frames=17,maximum_frames=32,target_frames=17,extra_fixed_count_frames=0,
        coverage_passed=True,native_outcomes_used_for_selection=False)
    core_rows=[dict(id=entry['id'],selection_roles=['fixture_protected' if i<8 else 'set_cover'],coverage=sorted(required),scaled_actors=[])
        for i,entry in enumerate(entries[:17])]
    selected=[dict(row,scenario_sha256=entries[i]['scenario_sha256'],metadata_sha256=release.sha256(meta),
        source_entry_preserved=True,original_ancestor_entry_preserved=True) for i,row in enumerate(core_rows)]
    subset=copy.deepcopy(deck);subset.update(count=17,scenes=entries[:17])
    subset_path=write(tmp_path/'smoke'/'manifest.json',subset)
    doc=dict(schema=release.smoke.SCHEMA,policy=release.smoke.POLICY,source_manifest=str(full),
        source_manifest_sha256=release.sha256(full),helper_sha256=release.sha256(release.smoke.__file__),
        native_qa_unchanged=True,bulk_started=False,training_started=False,live_qualified=False,
        runtime_hashes=hashes,input_hashes=dict(hashes,**{str(original):release.sha256(original)}),
        source_audit_hashes=hashes,qualification_evidence_hashes=hashes,
        subset_manifest_sha256=release.sha256(subset_path),policy_sha256=release.scale.digest(policy),
        qualified_profiles=[list(pair) for pair in sorted(pairs)],selected=selected,coverage=coverage)
    registration=write(tmp_path/'smoke'/'registration.json',doc)
    monkeypatch.setattr(release.pins,'collect_runtime',lambda **kwargs:hashes)
    monkeypatch.setattr(release.pins,'collect_inputs',lambda *a:hashes)
    monkeypatch.setattr(release.smoke,'source_checks',lambda *a:hashes)
    monkeypatch.setattr(release.scale,'bind_config',lambda config:config)
    monkeypatch.setattr(release.smoke,'approved_pairs',lambda *a:pairs)
    monkeypatch.setattr(release.smoke,'features',lambda *a:set(required))
    monkeypatch.setattr(release.smoke,'variants',lambda *a:[])
    monkeypatch.setattr(release.reuse,'validate_manifest',lambda *a:hashes)
    def select(*args,**kwargs):
        assert kwargs.get('target')==coverage['target_frames']==17
        return copy.deepcopy(core_rows),copy.deepcopy(coverage)
    monkeypatch.setattr(release.smoke,'select',select)
    return full,registration,doc,subset_path


@pytest.mark.parametrize('kind',['valid','schema','source','runtime','subset','posthoc','profile','context','protected_selection',
                                 'missing_target','different_target','boolean_target'])
def test_registration_revalidates_current_source_runtime_inputs_and_all_coverage(tmp_path,monkeypatch,kind):
    full,registration,doc,subset=registration_fixture(tmp_path,monkeypatch)
    if kind=='schema':doc['schema']='legacy_v154'
    if kind=='source':doc['source_manifest_sha256']='0'*64
    if kind=='runtime':doc['runtime_hashes']={}
    if kind=='subset':
        value=release.read(subset);value['scenes']=value['scenes'][::-1];write(subset,value)
        doc['subset_manifest_sha256']=release.sha256(subset)
    if kind=='posthoc':doc['coverage']['native_outcomes_used_for_selection']=True
    if kind=='profile':doc['qualified_profiles'].pop()
    if kind=='context':doc['coverage']['required'].remove('scaled_context/generic')
    if kind=='protected_selection':doc['selected'][0]['selection_roles']=['set_cover']
    if kind=='missing_target':doc['coverage'].pop('target_frames')
    if kind=='different_target':doc['coverage']['target_frames']=16
    if kind=='boolean_target':doc['coverage']['target_frames']=True
    write(registration,doc)
    if kind=='valid':assert len(release.registration_context(full,registration)['selected'])==17
    else:
        with pytest.raises(ValueError):release.registration_context(full,registration)


def native_fixture(tmp_path,monkeypatch,names=None):
    capture=tmp_path/'capture';entries={};registered={};rows=[];pairs=set();cover=set()
    names=sorted(release.smoke.APPROVED_ASSETS if names is None else names)
    profiles=[(name,profile) for name in names for profile in ('profile125','profile150')]
    pairs.update(profiles)
    for i in range(17):
        ident='source_'+str(i);actors=[]
        if 8<=i<16:
            for ordinal,(name,profile) in enumerate(profiles[(i-8)*2:(i-8)*2+2]):
                actors.append(dict(name=name,asset_scale=dict(profile_id=profile,candidate_only=False,
                    profile_status='qualified',name=name,ordinal=ordinal)))
        meta=dict(spec=dict(family='obstacle',mission=False),runtime_actors=actors,split='train')
        source=tmp_path/'source'/ident
        scenario=write(source/'scenario.json',dict(id=ident));metadata=write(source/'metadata.json',meta)
        entries[ident]=dict(id=ident,scenario=str(scenario),metadata=str(metadata),scenario_sha256=release.sha256(scenario))
        raw=capture/'cells'/ident/'initial'/'captures'/ident;raw.mkdir(parents=True)
        (raw/'scenario.json').write_bytes(scenario.read_bytes())
        shadow=i<2
        if shadow:meta[release.reuse.KEY]=dict(source_id=ident)
        write(raw/'metadata.json',meta)
        features=release.smoke.features(meta,pairs);cover.update(features)
        registered[ident]=dict(id=ident,selection_roles=['protected/shadow_exact/'+ident] if shadow else ['set_cover'],
            coverage=sorted(features),scenario_sha256=entries[ident]['scenario_sha256'],metadata_sha256=release.sha256(metadata))
        audit=dict(components=[dict(component_id='native-component')],accepted_candidate=True,review_reasons=[])
        write(raw/'audit.json',audit)
        samples=[dict(t=t,message=dict(collision_object=[dict(unique_id=0)],collision_objecta=[])) for t in (0.,.5,1.)]
        summary=release.collision_window(samples,0.,1.)
        write(raw/'collision_history.json',dict(summary=summary,samples=samples))
        write(raw.parents[1]/'shutdown_safety.json',dict(park_verified=True))
        if shadow:
            adoption=write(raw.parents[2]/'motion_reuse_deck/manifest.json',
                dict(config=dict(environment_tag='sunny13'),count=1,scenes=[entries[ident]]))
            write(raw.parents[1]/'session_contract.json',dict(manifest_sha256=release.sha256(adoption)))
        else:write(raw.parents[1]/'session_contract.json',{})
        row=dict(id=ident,source_cell_id=ident,status='accepted_candidate',capture_root=str(raw),
            scenario_sha256=entries[ident]['scenario_sha256'],audit=audit)
        rows.append(row)
    jsonl(capture/'selected_ledger.jsonl',rows)
    write(capture/'status.json',dict(stage='complete',requested=17,accepted_candidate=17,review=0,elapsed_sec=10.))
    subset=write(tmp_path/'subset'/'manifest.json',{})
    context=dict(subset_path=subset,entries=entries,selected=registered,pairs=pairs,
        deck=dict(config=dict(environment_tag='sunny13')),doc=dict(coverage=dict(required=sorted(cover))))
    monkeypatch.setattr(release.completion,'verify_parent',lambda *a:(copy.deepcopy(rows),[],10.))
    native_calls=[];reuse_calls=[]
    monkeypatch.setattr(release.training,'assert_native',lambda *a:native_calls.append(a))
    monkeypatch.setattr(release.reuse,'verify_capture',lambda *a:reuse_calls.append(a))
    return context,capture,rows,native_calls,reuse_calls


def test_all17_native_collision_park_geometry_and_shadow_adoption_are_rechecked(tmp_path,monkeypatch):
    context,capture,_,native,reuse=native_fixture(tmp_path,monkeypatch)
    result,blockers=release.verify_native(context,capture)
    assert result['accepted']==17 and result['shadow_ground_adoptions']==2 and not blockers
    assert len(native)==len(reuse)==17
    assert result['preserved_new_native_reviews']==[]


@pytest.mark.parametrize('asset_count',[7,8])
def test_profile_count_comes_from_exact_registered_pairs_without_weakening17_or_review_hold(tmp_path,monkeypatch,asset_count):
    names=['CargoBox','WoodBox','YellowBarrel','PE_Drum','PE_Firewall_Orange',
           'Steel_Barricade','Concrete_Barricade01','RedBarrel'][:asset_count]
    context,capture,_,_,_=native_fixture(tmp_path,monkeypatch,names)
    result,blockers=release.verify_native(context,capture)
    assert result['requested']==result['accepted']==17 and result['registered_profiles']==asset_count*2
    assert not blockers
    jsonl(capture/'preserved_review'/'ledger.jsonl',[dict(id='must_stay_review',status='review')])
    _,blockers=release.verify_native(context,capture)
    assert 'current_smoke_has_preserved_native_review_or_failure' in blockers


@pytest.mark.parametrize('kind',['audit_tamper','scenario_tamper','collision_peer','collision_sparse','unsafe_park','foreign_metadata','shadow_contract'])
def test_native_evidence_tamper_is_not_release_qualified(tmp_path,monkeypatch,kind):
    context,capture,rows,_,_=native_fixture(tmp_path,monkeypatch)
    raw=Path(rows[0 if kind=='shadow_contract' else 8]['capture_root'])
    if kind=='audit_tamper':write(raw/'audit.json',dict(components=[],review_reasons=[]))
    if kind=='scenario_tamper':write(raw/'scenario.json',dict(changed=True))
    if kind.startswith('collision_'):
        value=release.read(raw/'collision_history.json')
        if kind=='collision_peer':value['samples'][0]['message']['collision_objecta']=[dict(unique_id=8500)]
        else:value['samples']=[]
        write(raw/'collision_history.json',value)
    if kind=='unsafe_park':write(raw.parents[1]/'shutdown_safety.json',dict(park_verified=False))
    if kind=='foreign_metadata':
        value=release.read(raw/'metadata.json');value['spec']['seed']=99;write(raw/'metadata.json',value)
    if kind=='shadow_contract':write(raw.parents[1]/'session_contract.json',dict(manifest_sha256='0'*64))
    with pytest.raises((ValueError,KeyError)):release.verify_native(context,capture)


def test_reviewed_new_native_attempt_forces_hold_even_if_selected17_succeeded(tmp_path,monkeypatch):
    context,capture,_,_,_=native_fixture(tmp_path,monkeypatch)
    jsonl(capture/'cells'/'preserved_attempt'/'ledger.jsonl',[dict(id='reviewed',status='review')])
    result,blockers=release.verify_native(context,capture)
    assert result['accepted']==17
    assert 'current_smoke_has_preserved_native_review_or_failure' in blockers
    assert result['preserved_new_native_reviews'][0]['id']=='reviewed'


def test_missing_approval_is_hold_not_an_invented_seventeenth_native_observation(tmp_path,monkeypatch):
    context,capture,rows,_,_=native_fixture(tmp_path,monkeypatch)
    missing=rows.pop()['source_cell_id'];jsonl(capture/'selected_ledger.jsonl',rows)
    write(capture/'status.json',dict(stage='complete_with_deficit',requested=17,accepted_candidate=16,review=1,elapsed_sec=10.))
    monkeypatch.setattr(release.completion,'verify_parent',lambda *a:(copy.deepcopy(rows),[missing],10.))
    result,blockers=release.verify_native(context,capture)
    assert result['accepted']==16 and result['missing']==[missing]
    assert 'current_smoke_native_approval_deficit' in blockers


def test_training_report_must_represent_the_exact_new_selected_ledger(tmp_path,monkeypatch):
    capture=tmp_path/'capture';capture.mkdir()
    doc=dict(sensor_sha256=release.sha256(release.pins.ROOT/'configs/sensors/Sensor_competition_local.json'),
        cohorts=dict(current17=dict(frames=17,source_ledger=str(capture/'selected_ledger.jsonl'))))
    path=write(tmp_path/'training.json',doc);calls=[]
    monkeypatch.setattr(release,'verify_training_inputs',lambda p,expected:(calls.append(expected) or doc))
    assert release.verify_training(path,capture,17)['smoke_frames']==17
    assert calls==[dict(current17=17)]
    doc['cohorts']['current17']['source_ledger']=str(tmp_path/'old_source.jsonl')
    with pytest.raises(ValueError):release.verify_training(path,capture,17)


@pytest.mark.parametrize('kind',['valid','collector_rejects','incomplete'])
def test_full5k_runs_actual_collector_admission_with_new_immutable_config_view(monkeypatch,kind):
    policy=dict(original='unchanged');config={release.scale.CONFIG_KEY:policy}
    deck=dict(count=5000,scenes=[dict(id='slot_'+str(i)) for i in range(5000)],config=config)
    original=copy.deepcopy(deck);snapshot=object();calls=[];ends=[]
    def bind(value):
        assert value is config
        return {release.scale.CONFIG_KEY:snapshot}
    def validate(entry,view):
        assert view is not deck and view['config'] is not config
        assert view['config'][release.scale.CONFIG_KEY] is snapshot
        assert view['scenes'] is deck['scenes']
        calls.append(entry['id'])
        if kind=='collector_rejects' and len(calls)==4:raise ValueError('Actual capture admission rejects metadata')
    monkeypatch.setattr(release.scale,'bind_config',bind)
    monkeypatch.setattr(release.collector_contract,'validate_entry',validate)
    monkeypatch.setattr(release.scale,'verify_frozen_policy',lambda value:ends.append(value))
    if kind=='incomplete':deck['scenes'].pop();original=copy.deepcopy(deck)
    if kind=='valid':
        result=release.verify_full_collector_preflight(deck)
        assert result['passed'] and result['scenes']==5000 and len(calls)==5000 and ends==[snapshot]
    else:
        with pytest.raises(ValueError):release.verify_full_collector_preflight(deck)
        assert len(calls)==(4 if kind=='collector_rejects' else 0)
        assert not ends
    assert deck==original and deck['config'] is config and deck['config'][release.scale.CONFIG_KEY] is policy


@pytest.mark.parametrize('blockers',[[],['current_smoke_native_approval_deficit']])
def test_fresh_receipt_is_oneclick_compatible_without_inventing_scaled_recovery_or_eta(tmp_path,monkeypatch,blockers):
    context,capture,_,_,_=native_fixture(tmp_path,monkeypatch)
    full=write(tmp_path/'full_source'/'manifest.json',dict(count=5000))
    registration=write(tmp_path/'registration.json',{})
    resumed=write(tmp_path/'resume.json',{});training=write(tmp_path/'training.json',{})
    test=write(tmp_path/'test_receipt.json',{})
    pin=write(tmp_path/'immutable_pin.json',{})
    checks={str(pin):release.sha256(pin)}
    context.update(full=full,runtime=checks,scale_policy=object())
    context['doc'].update(input_hashes=checks,source_audit_hashes=checks,qualification_evidence_hashes=checks)
    context['deck']['config']['obstacle_support_policy']='fixture-policy'
    monkeypatch.setattr(release,'registration_context',lambda *a:context)
    monkeypatch.setattr(release,'verify_full_collector_preflight',lambda *a:dict(passed=True,scenes=5000))
    monkeypatch.setattr(release,'verify_native',lambda *a:(dict(accepted=17),list(blockers)))
    monkeypatch.setattr(release,'verify_resume',lambda *a:dict(requested=17))
    monkeypatch.setattr(release,'test_results',lambda *a:dict(unique_passed=1200))
    monkeypatch.setattr(release,'verify_training',lambda *a:dict(smoke_frames=17))
    monkeypatch.setattr(release.audit_base5_scenarios,'audit',lambda *a:dict(status='OFFLINE_PASS',errors=[],scenes=5000,unique_payloads=5000))
    monkeypatch.setattr(release.scale,'verify_frozen_policy',lambda *a:None)
    original=release.snapshot(capture)
    args=SimpleNamespace(manifest=full,smoke_registration=registration,capture_root=capture,
        completed_resume_receipt=resumed,training_input_report=training,pytest_xml=[test],output=tmp_path/'release')
    result=release.read(release.freeze(args))
    assert result['bulk_ready']==(not blockers) and result['bulk_blockers']==blockers
    assert result['schema']=='base5_obstacle_support_release_v1'
    assert result['completion_live_qualified'] is False
    assert result['operational_hours_range']==[8,12] and result['operational_storage_budget_gib']==45.
    assert result['historical_static_extent_pilot']['native_approval_count_reused_for_this_release']==0
    assert result['runtime_hashes']==checks and result['environments']['sunny13']['manifest']==str(full)
    assert release.snapshot(capture)==original
    with pytest.raises(ValueError):release.freeze(args)
