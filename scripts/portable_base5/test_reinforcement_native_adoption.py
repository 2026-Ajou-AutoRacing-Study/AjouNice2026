"""CPU-only reinforced native adoption guards; no ROS/network/native execution."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import reinforcement_native_adoption as adoption


def save(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def test_truthful_distinct_schema_and_original_six_gates():
    assert adoption.ADOPTION_SCHEMA != adoption.assembly.SCHEMA
    assert adoption.QUALIFICATION_SCHEMA not in (
        adoption.reinforced.QUALIFICATION_SCHEMA,adoption.reinforced.fresh.QUALIFICATION_SCHEMA)
    assert adoption.GATES == adoption.original.GATES
    assert adoption.adapter_pin()['registration_schema'] == adoption.assembly.REGISTRATION_SCHEMA


def test_scope_reuses_original_finalizer_and_restores_every_binding(monkeypatch):
    original = adoption.original;assembly = adoption.assembly
    before = original.QUALIFICATION_SCHEMA,original.POLICY,original.qualify,assembly.assembled_source_context
    with adoption.installed():
        assert original.QUALIFICATION_SCHEMA == adoption.QUALIFICATION_SCHEMA
        assert original.POLICY == adoption.POLICY
        assert original.qualify is adoption.qualify_with_adoption
        assert assembly.assembled_source_context is adoption.adopted_source_context
        with pytest.raises(ValueError,match='nested'):
            with adoption.installed():pass
    assert before == (original.QUALIFICATION_SCHEMA,original.POLICY,original.qualify,assembly.assembled_source_context)
    with pytest.raises(RuntimeError,match='actual failure'):
        with adoption.installed():raise RuntimeError('actual failure')
    assert before == (original.QUALIFICATION_SCHEMA,original.POLICY,original.qualify,assembly.assembled_source_context)


def test_scope_detects_ownership_changes_and_still_restores():
    before = adoption.assembly.assembled_source_context
    with pytest.raises(ValueError,match='ownership'):
        with adoption.installed():adoption.assembly.assembled_source_context = lambda *args:None
    assert adoption.assembly.assembled_source_context is before
    with adoption.installed():pass


def test_freeze_and_verifier_delegate_to_unchanged_six_gate_finalizer(monkeypatch):
    def freeze(arguments,output):
        assert adoption.original.qualify is adoption.qualify_with_adoption
        assert adoption.original.QUALIFICATION_SCHEMA == adoption.QUALIFICATION_SCHEMA
        return 'actual release'
    mocked = Mock(side_effect=freeze)
    monkeypatch.setattr(adoption.original,'freeze',mocked)
    assert adoption.freeze({'actual':'evidence'},'new-output') == 'actual release'
    mocked.assert_called_once_with({'actual':'evidence'},'new-output')
    verifier = Mock(return_value={'passed':True})
    monkeypatch.setattr(adoption.original,'qualified_release_context',verifier)
    assert adoption.qualified_release_context('release','foggy13','full') == {'passed':True}
    verifier.assert_called_once_with('release','foggy13','full')


def test_unknown_adoption_receipt_uses_original_assembled_context(tmp_path,monkeypatch):
    receipt = save(tmp_path/'old/receipt.json',{'schema':adoption.assembly.SCHEMA})
    manifest = save(tmp_path/'source/manifest.json',{})
    original = Mock(return_value={'actual':'old-context'})
    monkeypatch.setattr(adoption,'BASE_ASSEMBLED_CONTEXT',original)
    assert adoption.adopted_source_context(manifest,receipt) == {'actual':'old-context'}
    original.assert_called_once_with(manifest,receipt)


@pytest.mark.parametrize('value',[None,{}, {'path':'/tmp/no-such','sha256':'x'*64}])
def test_explicit_references_cannot_be_missing_or_forged(value):
    with pytest.raises((ValueError,OSError)):
        adoption.verified_reference(value,'actual')


def test_event_logs_do_not_require_nonexistent_object_id(tmp_path):
    events = tmp_path/'events.jsonl'
    events.write_text('{"phase":"park","scene":"original","recorded_at":100.0}\n')
    assert adoption.event_rows(events)[0]['phase'] == 'park'
    events.write_text('{"phase":"park"}')
    with pytest.raises(ValueError,match='Complete actual'):
        adoption.event_rows(events)


def test_binding_requires_actual_summary_and_full_evidence_inventory(tmp_path,monkeypatch):
    manifest = save(tmp_path/'source/manifest.json',{'actual':'source'})
    native = tmp_path/'raw.bin';native.write_bytes(b'actual native')
    pins = {str(native):adoption.sha(native)}
    binding = save(tmp_path/'binding.json',dict(manifest=adoption.profile.pin(manifest),
        summary={'actual':32},evidence_hashes=pins))
    monkeypatch.setattr(adoption,'binding_context',lambda value:dict(summary={'actual':32},evidence_hashes=dict(pins)))
    result = adoption.verify_binding(binding,manifest)
    assert result['evidence_hashes'][str(binding)] == adoption.sha(binding)
    changed = adoption.read(binding);changed['summary']={'fake':5000};save(binding,changed)
    with pytest.raises(ValueError,match='summary differs'):
        adoption.verify_binding(binding,manifest)
    changed['summary']={'actual':32};changed['evidence_hashes']={};save(binding,changed)
    with pytest.raises(ValueError,match='inventory changed'):
        adoption.verify_binding(binding,manifest)
    with pytest.raises(ValueError,match='different final'):
        adoption.verify_binding(binding,save(tmp_path/'other.json',{}))


def test_missing_binding_cannot_qualify():
    with pytest.raises(ValueError,match='binding is required'):
        adoption.qualify_with_adoption({'manifest':'not-opened'})


@pytest.mark.parametrize('bad',['missing_gate','failed_gate','other_assembly'])
def test_adoption_never_waives_original_six_gates(tmp_path,monkeypatch,bad):
    receipt = tmp_path/'actual-receipt.json'
    verified = dict(context={'assembly_receipt':str(receipt)},evidence_hashes={})
    monkeypatch.setattr(adoption,'verify_binding',lambda *args:copy.deepcopy(verified))
    result = dict(context={'assembly_receipt':str(receipt)},input_hashes={},
        results={name:dict(passed=True) for name in adoption.GATES})
    if bad == 'missing_gate':result['results'].pop(adoption.GATES[0])
    elif bad == 'failed_gate':result['results'][adoption.GATES[0]]['passed']=False
    else:result['context']['assembly_receipt']=str(tmp_path/'another-receipt.json')
    monkeypatch.setattr(adoption,'BASE_QUALIFY',lambda args:result)
    with pytest.raises(ValueError,match='six original|another native assembly'):
        adoption.qualify_with_adoption(dict(manifest='actual',reinforcement_adoption_binding='binding'))


def test_adoption_native_pilot_and_helper_pins_reach_release(tmp_path,monkeypatch):
    receipt = tmp_path/'assembly.json';receipt.write_text('{}')
    raw = tmp_path/'native.bin';raw.write_bytes(b'actual sensor')
    pins = {str(raw):adoption.sha(raw)}
    monkeypatch.setattr(adoption,'verify_binding',lambda *args:dict(
        context={'assembly_receipt':str(receipt)},evidence_hashes=pins))
    monkeypatch.setattr(adoption,'BASE_QUALIFY',lambda args:dict(context={'assembly_receipt':str(receipt)},
        results={name:dict(passed=True) for name in adoption.GATES},input_hashes={str(receipt):adoption.sha(receipt)}))
    result = adoption.qualify_with_adoption(dict(manifest='actual',reinforcement_adoption_binding='binding'))
    assert result['input_hashes'] == dict(pins,**{str(receipt):adoption.sha(receipt)})
    raw.write_bytes(b'changed')
    with pytest.raises(ValueError,match='changed'):
        adoption.profile.inputs.verify_hashes(result['input_hashes'])


@pytest.fixture
def semantic_source(monkeypatch):
    p = adoption.profile
    actor = dict(name='Box',data_id=1,unique_id=1,source_list='objectList',population_pool='obstacle',
        motion='static',planned_center_distance_m=10.,actual_ego_role='front')
    base = dict(spec=dict(actors=1,distance_bounds=[0,20]),split='train',runtime_actors=[actor])
    assignment = dict(id='scene0000',source_spec_sha256=p.digest(base['spec']),
        actor_identity=p.actor_identity(base['runtime_actors']),capture_split='train',
        target_bounds_m=[20,30],rule='mid',role='front')
    ids = ['scene%04d'%i for i in range(5000)]
    entries = {ident:dict(id=ident,split='train',scenario_sha256='%064x'%i) for i,ident in enumerate(ids)}
    metadata = {ident:base for ident in ids}
    old = copy.deepcopy(base);old['spec']=p.changed_spec(base,assignment)
    old['runtime_actors'][0]['planned_center_distance_m']=25.
    metadata[ids[0]]=old
    source = dict(entries=entries,metadata=metadata,deck={'config':{p.unit.KEY:{'unit':'reference'}}})
    new_entries = dict(entries);new_entries[ids[0]]=dict(entries[ids[0]],scenario_sha256='f'*64)
    final = dict(entries=new_entries,metadata=dict(metadata),deck=copy.deepcopy(source['deck']))
    receipt = dict(mapping=[dict(canonical_id=ids[0],old_scenario_sha256=entries[ids[0]]['scenario_sha256'],
        new_scenario_sha256='f'*64)])
    baseline = [dict(id=ident,meta=base) for ident in ids]
    document = dict(minimum_remaining_near_actors=4000,maximum_near_loss_fraction=.1,
        rules=[dict(assets={'Box':1},minimum_asset_near_remaining=4000,maximum_asset_near_fraction=.1)])
    extension = dict(source={'path':'baseline'},allocation={'path':'allocation'},profile={'path':'profile'})
    monkeypatch.setattr(p,'records_from_manifest',lambda path:({},baseline))
    monkeypatch.setattr(adoption,'read',lambda path:({'assignments':[assignment]} if path=='allocation' else document))
    monkeypatch.setattr(p,'validate_profile',lambda value:value)
    monkeypatch.setattr(p.unit,'load',lambda ref:{'assets':{'Box':{'native_size_lwh_m':[1,1,1]}}})
    return source,final,extension,receipt


def test_actual_5000_slot_semantics_and_near_budget(semantic_source):
    source,final,extension,receipt = semantic_source
    value = adoption.validate_final_semantics(source,final,extension,receipt)
    assert value['canonical_scenes'] == 5000 and value['unchanged_payloads'] == 4999
    assert value['final_near_parent_counts'] == {'Box':4999}
    assert value['actual_near_loss'] == 1
    assert not value['historical_geometry_provenance_copied_to_new_metadata']


@pytest.mark.parametrize('bad',['missing_slot','foreign_mapping','duplicate_mapping','spec','roster','capture_split',
    'config','unmapped_payload','range','role','global_floor','asset_floor','role_stratum'])
def test_5000_slot_adoption_rejects_semantic_drift(semantic_source,monkeypatch,bad):
    source,final,extension,receipt = semantic_source
    if bad == 'missing_slot':final['entries'].pop('scene4999')
    elif bad == 'foreign_mapping':receipt['mapping'][0]['canonical_id']='foreign'
    elif bad == 'duplicate_mapping':receipt['mapping'].append(copy.deepcopy(receipt['mapping'][0]))
    elif bad == 'config':final['deck']['config']['unapproved']='policy'
    elif bad == 'unmapped_payload':final['entries']['scene0001']=dict(final['entries']['scene0001'],scenario_sha256='bad')
    elif bad in ('global_floor','asset_floor'):
        original_read=adoption.read
        def changed(path):
            doc=copy.deepcopy(original_read(path))
            if path=='profile':
                if bad=='global_floor':doc['minimum_remaining_near_actors']=5000
                else:doc['rules'][0]['minimum_asset_near_remaining']=5000
            return doc
        monkeypatch.setattr(adoption,'read',changed)
    elif bad == 'role_stratum':
        count=0
        def roles(records,sizes):
            nonlocal count
            count+=1
            return {('Box','front'):1} if count==1 else {}
        monkeypatch.setattr(adoption.profile,'near_role_counts',roles)
    else:
        meta=copy.deepcopy(final['metadata']['scene0000']);final['metadata']['scene0000']=meta
        if bad=='spec':meta['spec']['unapproved']=True
        elif bad=='roster':meta['runtime_actors'][0]['data_id']=2
        elif bad=='capture_split':meta['split']='val'
        elif bad=='range':meta['runtime_actors'][0]['planned_center_distance_m']=31.
        else:meta['runtime_actors'][0]['actual_ego_role']='left'
    with pytest.raises((ValueError,KeyError)):
        adoption.validate_final_semantics(source,final,extension,receipt)


def test_diagnostic_reviews_need_actual_replacement_not_fake_receipt(tmp_path,monkeypatch):
    source = save(tmp_path/'source.json',{})
    registration = save(tmp_path/'registration.json',{})
    extension = save(tmp_path/'extension.json',{})
    monkeypatch.setattr(adoption.reinforced,'extension_context',lambda *args,**kwargs:{})
    monkeypatch.setattr(adoption,'registration_source',lambda *args:dict(kind='registered_changed_rows'))
    pilot = save(tmp_path/'forged-success.json',{'passed':True})
    with pytest.raises(ValueError,match='not a fake successful receipt'):
        adoption.prepare(source,extension,registration,[],[],pilot,tmp_path/'new',tmp_path/'bulk')
    assert not (tmp_path/'new').exists()


def test_prepare_never_overwrites_old_evidence(tmp_path):
    output=tmp_path/'prior';output.mkdir()
    with pytest.raises(ValueError,match='Fresh adoption'):
        adoption.prepare(tmp_path/'source',tmp_path/'build',tmp_path/'reg',[],[],None,output,tmp_path/'bulk')


def test_canonical_rebase_changes_only_new_metadata_and_preserves_real_predecessor(tmp_path):
    original = dict(spec={'actors':1},runtime_actors=[{'data_id':1}],retry=7,
        quality_replacement_contract={'root_entry':{'id':'old-root'}},
        fresh_completion_source={'source_cell_id':'old-root'},replacement={'old':'geometry'},
        training_split_overlay={'old':'overlay'})
    manifest = save(tmp_path/'actual-approved-manifest.json',{'actual':'native'})
    metadata = save(tmp_path/'actual-approved-metadata.json',original)
    before = metadata.read_bytes();unchanged = copy.deepcopy(original)
    new = adoption.canonical_metadata(original,'canonical',manifest,metadata)
    assert original == unchanged and metadata.read_bytes() == before
    assert all(field not in new for field in adoption.ACTIVE_RECOVERY_FIELDS)
    assert 'training_split_overlay' not in new
    assert new['retry'] == 7 and new['spec'] == original['spec']
    assert new['native_adoption_source']['approved_metadata'] == adoption.profile.pin(metadata)
    assert new['native_adoption_source']['previous_recovery_contract_is_historical']
    assert new['native_adoption_source']['new_canonical_root']
    assert not new['native_adoption_source']['actual_native_metadata_modified']


def test_rebased_source_becomes_a_fresh_root_in_original_recovery_contract(tmp_path,monkeypatch):
    import run_base5_quality_loop as quality
    reference={'actual':'unit-reference'}
    native_meta=save(tmp_path/'actual_native/metadata.json',dict(
        spec={'actors':1},runtime_actors=[{'data_id':1}],
        quality_replacement_contract={'root_entry':{'id':'historical-root'}}))
    native_manifest=save(tmp_path/'actual_native/manifest.json',{'actual':'native'})
    metadata=adoption.canonical_metadata(adoption.read(native_meta),'canonical',native_manifest,native_meta)
    metadata[adoption.profile.unit.KEY]=reference
    metadata['training_split_overlay']={'actual':'new-overlay'}
    source_meta=save(tmp_path/'new_canonical/metadata.json',metadata)
    scenario=save(tmp_path/'new_canonical/scenario.json',{'actual':'same-approved-geometry'})
    entry=dict(id='canonical',metadata=str(source_meta),scenario=str(scenario),scenario_sha256=adoption.sha(scenario))
    deck=dict(config={adoption.profile.unit.KEY:reference},split_overlay={'actual':'new-overlay'})
    monkeypatch.setattr(adoption.profile.unit,'load',lambda value:{})
    monkeypatch.setattr(adoption.profile.unit,'validate_actor_identities',lambda *args:None)
    monkeypatch.setattr(adoption.profile.layout,'validate_manifest_overlay',lambda *args,**kwargs:None)
    monkeypatch.setattr(adoption.profile.layout,'overlay_for_metadata',lambda value:{})
    monkeypatch.setattr(adoption.profile.layout,'resolve',lambda *args:dict(split='train',group_id='new-actual-geometry'))
    monkeypatch.setattr(adoption.profile.layout,'verify_source_pins',lambda value:None)
    contract=quality.replacement_contract(deck,entry,metadata)
    assert contract['root_entry']==entry and contract['ordinal']==1
    assert contract['root_metadata_sha256']==adoption.sha(source_meta)
    assert contract['root_entry']['id']!='historical-root'


@pytest.mark.parametrize('loss,passes',[(500,True),(501,False),(750,False)])
def test_overlapping_asset_rules_use_current_allocator_strictest_cumulative_cap(
        semantic_source,monkeypatch,loss,passes):
    source,final,extension,receipt = semantic_source
    p = adoption.profile
    counts = iter([adoption.Counter(Box=5000),adoption.Counter(Box=5000-loss)])
    monkeypatch.setattr(p,'near_counts',lambda rows,sizes:next(counts))
    original_read = adoption.read
    def changed(path):
        document=copy.deepcopy(original_read(path))
        if path == 'profile':
            document['minimum_remaining_near_actors']=0
            document['maximum_near_loss_fraction']=.25
            rule=document['rules'][0]
            rule.update(minimum_asset_near_remaining=0,maximum_asset_near_fraction=.1)
            document['rules'].append(dict(rule,maximum_asset_near_fraction=.2))
        return document
    monkeypatch.setattr(adoption,'read',changed)
    if passes:
        assert adoption.validate_final_semantics(source,final,extension,receipt)['actual_near_loss'] == loss
    else:
        with pytest.raises(ValueError,match='per-asset'):
            adoption.validate_final_semantics(source,final,extension,receipt)


@pytest.mark.parametrize('missing',['replacement','original'])
def test_effective_diagnostic_never_turns_missing_native_into_success(tmp_path,monkeypatch,missing):
    monkeypatch.setattr(adoption.profile.unit,'load',lambda value:{'actual':'unit-reference'})
    registration=dict(selected=[{'id':'one'}])
    history={'one':dict(status='review',capture_root=str(tmp_path/'missing-raw'))}
    final=dict(deck={'config':{adoption.profile.unit.KEY:{}}})
    if missing == 'replacement':
        with pytest.raises(ValueError,match='lacks actual new native'):
            adoption.effective_diagnostic(registration,history,final,[])
    else:
        history['one']['status']='accepted_candidate'
        with pytest.raises((ValueError,OSError)):
            adoption.effective_diagnostic(registration,history,final,[])


def test_second_adoption_is_explicitly_bounded_to_two_stages(tmp_path,monkeypatch):
    source=save(tmp_path/'source.json',{})
    build=save(tmp_path/'build.json',{})
    parent=save(tmp_path/'parent-binding.json',dict(adoption_depth=2,build_extension=adoption.profile.pin(build)))
    monkeypatch.setattr(adoption,'verify_binding',lambda *args:{'context':{}})
    with pytest.raises(ValueError,match='bounded follow-up'):
        adoption.prepare(source,build,tmp_path/'registration',[],[],None,
            tmp_path/'next',tmp_path/'bulk',parent_binding=parent)
    assert not (tmp_path/'next').exists()
