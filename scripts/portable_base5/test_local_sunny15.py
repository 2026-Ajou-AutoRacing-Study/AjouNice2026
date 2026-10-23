"""Isolated CPU fixtures. Never starts the original parent, ROS or MORAI."""
from contextlib import contextmanager,redirect_stdout
import copy
import importlib
import io
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import local_sunny15 as local

ACTUAL_SCRIPTS = local.SCRIPTS


def write(filename,value):
    filename.parent.mkdir(parents=True,exist_ok=True)
    filename.write_text(json.dumps(value)+'\n')
    return filename


@pytest.fixture
def fixture(tmp_path,monkeypatch):
    root = tmp_path/'repo';scripts = root/'dataset/morai_od/scripts'
    scripts.mkdir(parents=True)
    final = root/'final';output = tmp_path/'morai_cap/collection_5k';output.mkdir(parents=True)
    wrapper = root/'scripts/portable_base5/local_sunny15.py';wrapper.parent.mkdir(parents=True)
    wrapper.write_text('# actual fixture entrypoint\n')
    qualifier = wrapper.with_name('local_sunny15_qualification.py');qualifier.write_text('# fixture qualifier\n')
    manifest = write(final/'source/manifest.json',dict(count=5000,scenes=[dict(id='source_%04d' % i) for i in range(5000)]))
    release = write(final/'release/release.json',dict(runtime_hashes={str(wrapper):local.sha(wrapper)},
        input_hashes={str(manifest):local.sha(manifest)},qualified_runtime=dict(runtime_profile={'exact':'profile'})))
    qualification = final/'local_population_recovery_v1/qualification.json'
    for name,value in dict(ROOT=root,SCRIPTS=scripts,FINAL=final,OUTPUT=output,MANIFEST=manifest,
        RELEASE=release,QUALIFICATION=qualification,QUALIFIER=qualifier,PARENT=scripts/'parent.py',
        RECOVERY=scripts/'run_base5_capture_campaign_direction_reference.py',
        RELEASE_SHA=local.sha(release),MANIFEST_SHA=local.sha(manifest),__file__=str(wrapper)).items():
        monkeypatch.setattr(local,name,value)
    batch = dict(manifest=local.MANIFEST_SHA,indices=list(range(5000)),chunk_size=16,bulk=True)
    parent = dict(manifest=str(manifest),manifest_sha256=local.MANIFEST_SHA,environment='sunny15',
        indices=list(range(5000)),chunk_size=16,no_automatic_completion=True,legacy_batch_contract=batch,
        runtime_hashes=json.loads(release.read_text())['runtime_hashes'],input_hashes={str(manifest):local.MANIFEST_SHA},
        runtime_profile={'exact':'profile'},qualified_release=dict(path=str(release),sha256=local.RELEASE_SHA))
    write(output/'batch_contract.json',batch);write(output/'static_attitude_batch_contract.json',parent)
    write(output/'status.json',dict(stage='prepared_waiting_for_resume',requested=5000,
        finished=0,accepted_candidate=0,review=0,failed=0))
    write(output/'quota.json',dict(target=5000,accepted=0,deficit=0,pending=5000,cells=[]))
    write(output/'deficit_manifest.json',dict(count=0,scenes=[]))
    for name in ('.batch.lock','.static_attitude_batch.lock','selected_ledger.jsonl'):
        (output/name).write_bytes(b'')
    proof = dict(schema=local.QUALIFICATION_SCHEMA,passed=True,manifest_sha256=local.MANIFEST_SHA,
        release_sha256=local.RELEASE_SHA,source_runtime_count=438,population_policy=local.population.POLICY,
        captures_requested=0,native_capture_executed=False,source_inputs_unchanged=True,native_qa_unchanged=True,
        source_inventory=dict(passed=True,source_scenes=5000),matrix=dict(all_branches_selected=True),
        replacement_count=20,problem_00082_candidate_pass=True,bounded_exhaustions=0,unexpected_contract_stops=0,
        helper_hashes=local.helper_hashes())
    write(qualification,proof)
    frozen = Mock(return_value=(json.loads(release.read_text()),None))
    monkeypatch.setattr(local,'frozen_context',frozen)
    monkeypatch.setattr(local,'parent_module',lambda:SimpleNamespace(
        runtime_hashes=lambda:json.loads(release.read_text())['runtime_hashes']))
    return SimpleNamespace(root=root,output=output,manifest=manifest,release=release,
        qualification=qualification,proof=proof,wrapper=wrapper,frozen=frozen)


def invoke(arguments):
    stream = io.StringIO()
    with redirect_stdout(stream):
        result = local.main(arguments)
    return result,stream.getvalue()


def snapshot(root):
    return {str(p):p.read_bytes() for p in root.rglob('*') if p.is_file()}


def recovery_args(index=69):
    return ['--enable-final-direction-reference','--manifest',str(local.MANIFEST),
        '--indices',str(index),'--output',str(local.OUTPUT/('chunk_%04d' % (index//16))/'recovery'/('source_%04d' % index)),
        '--resume']


def test_default_plan_is_readonly_without_qualification(fixture,monkeypatch):
    fixture.qualification.unlink()
    before = snapshot(fixture.root);before_output = snapshot(fixture.output)
    spawn = Mock();monkeypatch.setattr(local.subprocess,'Popen',spawn)
    result,text = invoke([])
    assert result == 0 and json.loads(text)['files_written'] is False
    assert snapshot(fixture.root) == before and snapshot(fixture.output) == before_output
    spawn.assert_not_called()


def test_prepare_only_adds_sidecar_and_preserves_originals(fixture,monkeypatch):
    f = fixture;before = snapshot(f.output)
    spawn = Mock();monkeypatch.setattr(local.subprocess,'Popen',spawn)
    result,text = invoke(['--prepare'])
    assert result == 0 and json.loads(text)['native_capture_executed'] is False
    assert {str(p):p.read_bytes() for p in f.output.iterdir() if p.name != local.CONTRACT} == before
    assert set(snapshot(f.output))-set(before) == {str(f.output/local.CONTRACT)}
    assert local.validate_contract(f.output/local.CONTRACT)['source_runtime_count'] == 438
    invoke(['--prepare']);spawn.assert_not_called()
    assert fixture.frozen.call_args.kwargs == dict(deep=True)


def test_original_parent_closure_may_be_strict_subset_of_release(fixture,monkeypatch):
    original = copy.deepcopy(json.loads(fixture.release.read_text())['runtime_hashes'])
    release = json.loads(fixture.release.read_text())
    extra = fixture.wrapper.with_name('additional_frozen_native_verifier.py')
    extra.write_text('# full qualification closure, not direct parent closure\n')
    release['runtime_hashes'][str(extra)] = local.sha(extra)
    write(fixture.release,release)
    monkeypatch.setattr(local,'RELEASE_SHA',local.sha(fixture.release))
    saved = json.loads((fixture.output/'static_attitude_batch_contract.json').read_text())
    saved['qualified_release']['sha256'] = local.RELEASE_SHA
    write(fixture.output/'static_attitude_batch_contract.json',saved)
    monkeypatch.setattr(local,'parent_module',lambda:SimpleNamespace(runtime_hashes=lambda:original))
    assert len(local.original_guards()) == 2
    release['runtime_hashes'][next(iter(original))] = '0'*64
    write(fixture.release,release)
    with pytest.raises(ValueError):local.original_guards()


@pytest.mark.parametrize('field,value',[
    ('passed',False),('manifest_sha256','bad'),('release_sha256','bad'),('source_runtime_count',427),
    ('population_policy','old'),('captures_requested',1),('native_capture_executed',True),
    ('source_inputs_unchanged',False),('native_qa_unchanged',False),('replacement_count',0),
    ('replacement_count',25),('problem_00082_candidate_pass',False),('bounded_exhaustions',1),
    ('unexpected_contract_stops',1),('matrix',dict(all_branches_selected=False)),
    ('source_inventory',dict(passed=True,source_scenes=4999))])
def test_prepare_requires_real_qualified_matrix(fixture,field,value):
    proof = dict(fixture.proof);proof[field] = value;write(fixture.qualification,proof)
    with pytest.raises(ValueError):local.prepare()
    assert not (fixture.output/local.CONTRACT).exists()


@pytest.mark.parametrize('change',['native_child','accepted','ledger','symlink','wrong_guard','missing_file'])
def test_prepare_refuses_retrofit_or_bad_originals(fixture,change):
    root = fixture.output
    if change == 'native_child':(root/'chunk_0000').mkdir()
    elif change == 'accepted':
        data = json.loads((root/'status.json').read_text());data['accepted_candidate'] = 1;write(root/'status.json',data)
    elif change == 'ledger':(root/'selected_ledger.jsonl').write_text('{}\n')
    elif change == 'symlink':
        (root/'quota.json').unlink();(root/'quota.json').symlink_to(fixture.qualification)
    elif change == 'wrong_guard':
        data = json.loads((root/'batch_contract.json').read_text());data['chunk_size'] = 8;write(root/'batch_contract.json',data)
    else:(root/'deficit_manifest.json').unlink()
    with pytest.raises(ValueError):local.prepare()
    assert not (root/local.CONTRACT).exists()


def test_prepare_cannot_relabel_after_native_even_with_existing_contract(fixture):
    local.prepare();(fixture.output/'chunk_0000').mkdir()
    with pytest.raises(ValueError):local.prepare()


@pytest.mark.parametrize('which',['wrapper','qualification','guard','sidecar'])
def test_resume_refuses_changed_recipe(fixture,which):
    contract = local.prepare()
    name = {'wrapper':fixture.wrapper,'qualification':fixture.qualification,
        'guard':fixture.output/'batch_contract.json','sidecar':contract}[which]
    name.write_text('{}\n')
    with pytest.raises(ValueError):local.validate_contract(contract)


def test_run_requires_ui_confirmation_and_sidecar(fixture,monkeypatch):
    original = Mock();monkeypatch.setattr(local,'original_run',original)
    with pytest.raises(ValueError):invoke(['--run'])
    with pytest.raises((ValueError,FileNotFoundError)):invoke(['--run','--confirmed-environment','sunny15'])
    original.assert_not_called()


def test_run_calls_exact_original_parent_and_restores_router(fixture,monkeypatch):
    contract = local.prepare();original = local.subprocess.Popen
    called = []
    monkeypatch.setattr(local,'original_run',lambda target,args:called.append((target,args)) or 0)
    result,_ = invoke(['--run','--confirmed-environment','sunny15'])
    assert result == 0 and local.subprocess.Popen is original
    target,args = called[0]
    assert target == local.PARENT and args[args.index('--manifest')+1] == str(local.MANIFEST)
    assert args[args.index('--output')+1] == str(local.OUTPUT)
    assert args[-3:] == ['--resume','--run','--approve-bulk-capture']
    assert '--complete' not in args and contract.exists()


def test_router_propagates_to_exact_recovery_only_preserving_kwargs(fixture,monkeypatch):
    contract = local.prepare();spawn = Mock();monkeypatch.setattr(local.subprocess,'Popen',spawn)
    with local.routed_children(contract):
        local.subprocess.Popen([local.sys.executable,str(local.RECOVERY),*recovery_args()],stdout='fixture',start_new_session=True)
        argv = spawn.call_args.args[0]
        assert argv[:3] == [local.sys.executable,'-B',str(fixture.wrapper)]
        assert argv[argv.index('--child-contract')+1] == str(contract)
        assert argv[argv.index('--')+1:] == recovery_args()
        assert spawn.call_args.kwargs == dict(stdout='fixture',start_new_session=True)
        untouched = [local.sys.executable,str(local.SCRIPTS/'run_base5_capture_static_attitude.py'),'--fixture']
        local.subprocess.Popen(untouched)
        assert spawn.call_args.args[0] == untouched
    assert local.subprocess.Popen is spawn


@pytest.mark.parametrize('command',[
    ['foreign-python','{recovery}'],['{python}','-u','{recovery}'],
    ['{python}','/foreign/run_base5_capture_campaign_direction_reference.py'],
    ['{python}','-c','{recovery}'],['/bin/sh','-c','{recovery}']])
def test_unknown_recovery_runner_fails_closed(fixture,monkeypatch,command):
    contract = local.prepare();spawn = Mock();monkeypatch.setattr(local.subprocess,'Popen',spawn)
    command = [part.format(recovery=local.RECOVERY,python=local.sys.executable) for part in command]
    with local.routed_children(contract),pytest.raises(ValueError):local.subprocess.Popen(command)
    spawn.assert_not_called()


def test_shell_form_recovery_and_router_ownership_fail_closed(fixture,monkeypatch):
    contract = local.prepare();spawn = Mock();monkeypatch.setattr(local.subprocess,'Popen',spawn)
    with local.routed_children(contract),pytest.raises(ValueError):
        local.subprocess.Popen('python3 '+str(local.RECOVERY))
    with pytest.raises(ValueError):
        with local.routed_children(contract):local.subprocess.Popen = Mock()
    assert local.subprocess.Popen is spawn


@pytest.mark.parametrize('change',['foreign_manifest','foreign_output','wrong_cell','duplicate','unknown','seed'])
def test_child_argument_scope_is_strict(fixture,change):
    args = recovery_args()
    if change == 'foreign_manifest':args[args.index('--manifest')+1] = '/foreign/manifest.json'
    elif change == 'foreign_output':args[args.index('--output')+1] = str(local.OUTPUT/'elsewhere')
    elif change == 'wrong_cell':args[args.index('--indices')+1] = '70'
    elif change == 'duplicate':args.extend(['--indices','69'])
    elif change == 'unknown':args.append('--approve-bulk-capture')
    else:args.extend(['--seed-capture','/foreign/seed.json'])
    with pytest.raises(ValueError):local.child_arguments(args)


def test_child_installs_guard_before_original_direction_compilation(fixture,monkeypatch):
    contract = local.prepare();events = [];quality = object()
    monkeypatch.setattr(local,'verify_parent',lambda *a:events.append('parent_identity'))
    monkeypatch.setattr(local.importlib,'import_module',lambda name:quality)
    @contextmanager
    def installed(quality_loop=None):
        assert quality_loop is quality
        events.append('install_population')
        try:yield
        finally:events.append('restore_population')
    monkeypatch.setattr(local.population,'installed',installed)
    def execute(target,args):
        assert target == local.RECOVERY and args == recovery_args()
        events.append('original_direction_runpy')
        return 0
    monkeypatch.setattr(local,'original_run',execute)
    args = ['--child-contract',str(contract),'--contract-sha256',local.sha(contract),
        '--parent-pid','123','--parent-start-ticks','456','--',*recovery_args()]
    assert local.main(args) == 0
    assert events == ['parent_identity','install_population','original_direction_runpy','restore_population']


def test_original_run_preserves_original_file_and_restores_argv(fixture,monkeypatch):
    previous = local.sys.argv
    seen = []
    def run_path(target,run_name):
        seen.append((target,run_name,list(local.sys.argv)))
        raise SystemExit(7)
    monkeypatch.setattr(local.runpy,'run_path',run_path)
    assert local.original_run(local.RECOVERY,['--fixture']) == 7
    assert seen == [(str(local.RECOVERY),'__main__',[str(local.RECOVERY),'--fixture'])]
    assert local.sys.argv is previous


def test_actual_direction_ast_copies_already_installed_population_replacement(monkeypatch):
    monkeypatch.syspath_prepend(str(ACTUAL_SCRIPTS))
    direction = importlib.import_module('run_base5_capture_campaign_direction_reference')
    quality = importlib.import_module('run_base5_quality_loop')
    before = quality.replacement
    with local.population.installed(quality_loop=quality):
        wrapped = quality.replacement
        compiled = direction.compiled_execute(dict(quality_source_sha256=direction.sha256(direction.SOURCE)))
        assert wrapped is not before
        assert compiled.__globals__['replacement'] is wrapped
    assert quality.replacement is before
