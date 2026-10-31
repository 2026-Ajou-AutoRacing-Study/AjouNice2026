"""CPU-only fixtures; never starts MORAI, ROS or a collection subprocess."""
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import local_sunny15_fast as fast


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n')
    return path


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@pytest.fixture
def collection(tmp_path,monkeypatch):
    original = fast.original
    root = tmp_path/'collection';root.mkdir(mode=0o775)
    source = tmp_path/'source';source.mkdir()
    frozen = source/'runtime.py';frozen.write_text('# frozen runtime\n')
    manifest = write(source/'manifest.json',dict(scenes=[]))
    qualification = write(source/'qualification.json',dict(passed=True))
    old_helper = source/'old_wrapper.py';old_helper.write_text('# old approved wrapper\n')
    raw = root/'chunk_0000/capture/captures/a/a_Intensity.bin'
    raw.parent.mkdir(parents=True);raw.write_bytes(b'original native data')
    capture = root/'chunk_0000/capture'
    write(capture/'session_contract.json',dict(contract='unchanged'))
    write(capture/'status.json',dict(stage='complete'))
    (capture/'ledger.jsonl').write_text(json.dumps(dict(id='a',status='accepted_candidate'))+'\n')
    write(capture/'shutdown_safety.json',dict(park_verified=True,gear=1,
        brake_held_during_requests=True,request_fields=dict(option=2,gear=1)))
    for path in (root/'.batch.lock',root/'.static_attitude_batch.lock',capture/'.runner.lock'):
        path.write_bytes(b'')
    write(root/'status.json',dict(stage='paused',current=None,requested=5000,finished=3,
        accepted_candidate=1,review=2,elapsed_sec=3000,updated_at=time.time()+100))
    (root/'PAUSE').write_bytes(b'original operator pause\n')
    (root/'selected_ledger.jsonl').write_text(json.dumps(dict(source_cell_id='a',status='accepted_candidate'))+'\n')
    old_sidecar = write(root/original.CONTRACT,dict(old='unchanged sidecar'))
    guards = [write(root/name,dict(old=name)) for name in
        ('batch_contract.json','static_attitude_batch_contract.json')]
    native_proof = write(source/'audit.json',dict(actual='must not cache'))
    release = write(source/'release.json',dict(runtime_hashes={str(frozen):digest(frozen)},
        input_hashes={str(manifest):digest(manifest),str(native_proof):digest(native_proof)}))
    for key,value in dict(OUTPUT=root,MANIFEST=manifest,RELEASE=release,QUALIFICATION=qualification,
        MANIFEST_SHA=digest(manifest),RELEASE_SHA=digest(release)).items():
        monkeypatch.setattr(original,key,value)
    folder = tmp_path/'evidence'
    monkeypatch.setattr(fast,'DIRECTORY',folder)
    monkeypatch.setattr(fast,'CONTRACT',folder/'contract.json')
    monkeypatch.setattr(fast,'ATTACHMENT',folder/'attachment.json')
    monkeypatch.setattr(original,'helper_hashes',lambda:{str(old_helper):digest(old_helper)})
    monkeypatch.setattr(original,'original_guards',lambda:{str(path):digest(path) for path in guards})
    monkeypatch.setattr(original,'validate_contract',Mock(return_value=dict(old=True)))
    monkeypatch.setattr(original,'frozen_context',Mock(return_value=(json.loads(release.read_text()),None)))
    monkeypatch.setattr(original,'parent_module',lambda:SimpleNamespace(completed_snapshot=lambda output:
        dict(raw_files={str(raw.relative_to(root)):dict(sha256=digest(raw),size=raw.stat().st_size)},
             selected_sha256=digest(root/'selected_ledger.jsonl'))))
    return SimpleNamespace(root=root,source=source,raw=raw,capture=capture,manifest=manifest,
        release=release,guards=guards,old_sidecar=old_sidecar,native_proof=native_proof)


def tree_bytes(root):
    return {str(path.relative_to(root)):path.read_bytes() for path in root.rglob('*') if path.is_file()}


def test_default_is_read_only_plan_even_before_attachment(collection,capsys):
    before = tree_bytes(collection.root)
    assert fast.main([]) == 0
    value = json.loads(capsys.readouterr().out)
    assert value['files_written'] is False and value['captures_requested'] == 0
    assert tree_bytes(collection.root) == before and not fast.DIRECTORY.exists()
    fast.original.frozen_context.assert_not_called()


def test_attachment_records_raw_and_original_contracts_without_collection_changes(collection):
    before = tree_bytes(collection.root)
    result = fast.attach()
    assert result['preserved_selected'] == 1 and result['native_capture_executed'] is False
    assert tree_bytes(collection.root) == before
    proof = json.loads(fast.ATTACHMENT.read_text())
    assert proof['snapshot']['raw']['raw_files'][str(collection.raw.relative_to(collection.root))]['sha256'] == digest(collection.raw)
    assert proof['safety']['recorded_park_only'] is True
    assert fast.CONTRACT.stat().st_mode & 0o777 == 0o600
    fast.original.frozen_context.assert_called_once_with(deep=True)
    fast.validate_attachment(fast.immutable_pins())
    with pytest.raises(ValueError,match='already exists'):
        fast.attach()


@pytest.mark.parametrize('change',['stop','stop_symlink','running_parent','running_child','park','gear','brake','request','nan','missing_shutdown','symlink'])
def test_attachment_rejects_unsafe_or_ambiguous_exit(collection,change):
    root,capture = collection.root,collection.capture
    if change == 'stop':(root/'STOP').write_bytes(b'safety stop')
    elif change == 'stop_symlink':(root/'STOP').symlink_to(root/'missing-target')
    elif change == 'running_parent':
        value=json.loads((root/'status.json').read_text());value['stage']='running';write(root/'status.json',value)
    elif change == 'running_child':write(capture/'status.json',dict(stage='capturing'))
    elif change == 'missing_shutdown':(capture/'shutdown_safety.json').unlink()
    elif change == 'symlink':(root/'foreign-link').symlink_to(collection.source,target_is_directory=True)
    elif change == 'nan':
        value=json.loads((root/'status.json').read_text());value['updated_at']=float('nan');write(root/'status.json',value)
    else:
        value=json.loads((capture/'shutdown_safety.json').read_text())
        field={'park':'park_verified','gear':'gear','brake':'brake_held_during_requests','request':'request_fields'}[change]
        value[field]={'park':False,'gear':2,'brake':False,'request':{'option':1,'gear':1}}[change]
        write(capture/'shutdown_safety.json',value)
    with pytest.raises((ValueError,OSError)):
        fast.attach()
    assert not fast.CONTRACT.exists() and not fast.ATTACHMENT.exists()


@pytest.mark.parametrize('name',['.batch.lock','.static_attitude_batch.lock',fast.original.CONTRACT])
def test_attachment_requires_all_existing_ownership_locks_idle(collection,name):
    with (collection.root/name).open('rb') as held:
        fcntl.flock(held,fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(BlockingIOError):fast.attach()
    assert not fast.CONTRACT.exists()


def test_allowlist_excludes_native_gt_raw_and_proofs_but_keeps_declared_sources(collection):
    # Do not inherit pytest's test_*native* directory name as a declared input.
    base=Path('/approved_immutable_sources')
    source = str(base/'metadata.json')
    paths = [source,str(base/'manifest.json'),str(base/'layout.yaml'),
        str(base/'captures/native/scenario.json'),str(base/'CAMERA_1/Instance.json'),
        str(base/'raw/scene.json'),str(base/'LIDAR_5/Instance.txt'),
        str(base/'audit.json'),str(base/'status.json'),
        str(base/'native_receipt.json'),str(base/'report.json'),
        str(base/'final_exact_native_v2/batch_contract.json'),
        str(base/'motion_native/timing.json'),
        str(base/'training_inputs_v2/final32/data/metadata.json'),
        str(base/'audit_original_static_ground.json'),
        str(base/'final_exact_partial_snapshot_v1.json')]
    result = fast.source_only_pins({path:'a'*64 for path in paths})
    assert set(result) == set(paths[:3])
    pins = fast.immutable_pins()
    assert str(collection.native_proof) not in pins
    with pytest.raises(ValueError,match='Collection output'):
        fast.merge_pins({str(collection.raw):digest(collection.raw)})
    with pytest.raises(ValueError,match='Conflicting'):
        fast.merge_pins({source:'a'*64},{source:'b'*64})


def test_cache_scoped_original_sha_restores_and_never_caches_output(collection):
    previous = fast.original.sha
    pins = fast.immutable_pins()
    with pytest.raises(RuntimeError,match='test exit'):
        with fast.installed_cache(pins) as cache:
            assert fast.original.sha(collection.manifest) == digest(collection.manifest)
            fast.original.sha(collection.manifest)
            assert fast.original.sha(collection.raw) == digest(collection.raw)
            collection.raw.write_bytes(b'changed raw')
            assert fast.original.sha(collection.raw) == digest(collection.raw)
            raise RuntimeError('test exit')
    assert fast.original.sha is previous


def test_changed_original_sidecar_or_helper_rejects_continuation(collection,monkeypatch):
    fast.attach()
    collection.old_sidecar.write_text('{"old":"changed"}\n')
    with pytest.raises(ValueError,match='continuation contract'):
        fast.validate_attachment(fast.immutable_pins())


def test_resume_requires_explicit_environment_and_flag(collection):
    for arguments in (['--run'],['--run','--confirmed-environment','sunny15'],['--resume-paused']):
        with pytest.raises(ValueError):fast.main(arguments)
    assert (collection.root/'PAUSE').exists()


def test_resume_archives_only_pause_preserves_old_selection_and_uses_original_main(collection,monkeypatch):
    fast.attach()
    before = tree_bytes(collection.root)
    original_router = fast.original.routed_children
    calls=[]
    def run(arguments):
        calls.append(arguments)
        assert fast.original.routed_children is fast.routed_children
        assert not (collection.root/'PAUSE').exists()
        return 0
    monkeypatch.setattr(fast.original,'main',run)
    assert fast.main(['--run','--confirmed-environment','sunny15','--resume-paused']) == 0
    assert calls == [['--run','--confirmed-environment','sunny15']]
    assert fast.original.routed_children is original_router
    after=tree_bytes(collection.root)
    archive=[name for name in after if name.startswith('PAUSE.resumed_')]
    assert len(archive)==1 and after.pop(archive[0]) == before.pop('PAUSE')
    assert after == before


def test_explicit_recovery_router_uses_new_actual_child_and_restores_popen(collection,monkeypatch):
    fast.attach()
    spawn=Mock(return_value='child');monkeypatch.setattr(fast.subprocess,'Popen',spawn)
    validate=Mock();monkeypatch.setattr(fast.original,'child_arguments',validate)
    command=[fast.sys.executable,str(fast.original.RECOVERY),'--manifest',str(collection.manifest)]
    with fast.routed_children(collection.old_sidecar):
        assert fast.subprocess.Popen(command) == 'child'
        actual=spawn.call_args.args[0]
        assert actual[:3] == [fast.sys.executable,'-B',str(Path(fast.__file__).absolute())]
        assert actual[actual.index('--child-contract')+1] == str(fast.CONTRACT)
        assert actual[actual.index('--')+1:] == command[2:]
        with pytest.raises(ValueError):fast.subprocess.Popen('python '+fast.original.RECOVERY.name,shell=True)
    assert fast.subprocess.Popen is spawn
    validate.assert_called_once_with(command[2:])


@pytest.mark.parametrize('change',[None,'uid','ticks','script','missing_resume','child_parent','flags'])
def test_parent_authenticates_real_new_launcher_without_spoofing_original(collection,monkeypatch,change):
    record=dict(uid=os.getuid(),ticks='123',cwd=Path('/'),
        argv=[fast.sys.executable,'-B',str(Path(fast.__file__).absolute()),
            '--run','--confirmed-environment','sunny15','--resume-paused'])
    monkeypatch.setattr(fast.os,'getppid',lambda:345)
    if change=='uid':record['uid']+=1
    elif change=='ticks':record['ticks']='wrong'
    elif change=='script':record['argv'][2]=str(Path(fast.original.__file__).absolute())
    elif change=='missing_resume':record['argv'].pop()
    elif change=='child_parent':record['argv']+=['--child-contract','foreign']
    elif change=='flags':record['argv'][1]='-c'
    monkeypatch.setattr(fast,'parent_record',lambda pid:record)
    if change is None:fast.verify_parent(345,'123')
    else:
        with pytest.raises(ValueError):fast.verify_parent(345,'123')
    with pytest.raises(ValueError):fast.verify_parent(346,'123')


def test_router_scope_restores_on_exception(collection):
    previous=fast.original.routed_children
    with pytest.raises(RuntimeError):
        with fast.routed_parent():
            assert fast.original.routed_children is fast.routed_children
            raise RuntimeError('expected')
    assert fast.original.routed_children is previous


def test_pause_changed_during_safety_inspection_is_not_archived(collection,monkeypatch):
    inspect=fast.resume_evidence
    def changed(root,stack):
        result=inspect(root,stack)
        (root/'PAUSE').write_text('new operator pause request\n')
        return result
    monkeypatch.setattr(fast,'resume_evidence',changed)
    with pytest.raises(ValueError,match='PAUSE changed'):
        fast.release_pause()
    assert (collection.root/'PAUSE').read_text() == 'new operator pause request\n'
    assert not list(collection.root.glob('PAUSE.resumed_*'))


def test_new_child_retains_original_checks_population_and_recovery_entry(collection,monkeypatch):
    fast.attach()
    authenticate=Mock();monkeypatch.setattr(fast,'verify_parent',authenticate)
    arguments=['--manifest',str(collection.manifest),'--indices','0']
    check=Mock();monkeypatch.setattr(fast.original,'child_arguments',check)
    events=[]
    quality=SimpleNamespace(name='actual test quality loop')
    real_import=fast.importlib.import_module
    monkeypatch.setattr(fast.importlib,'import_module',lambda name,*args,**kwargs:
        quality if name=='run_base5_quality_loop' else real_import(name,*args,**kwargs))
    @contextmanager
    def population(quality_loop):
        assert quality_loop is quality
        events.append('population_enter')
        try:yield
        finally:events.append('population_exit')
    monkeypatch.setattr(fast.original.population,'installed',population)
    def run(target,received):
        assert target == fast.original.RECOVERY and received == arguments
        events.append('original_recovery')
        return 17
    monkeypatch.setattr(fast.original,'original_run',run)
    monkeypatch.setattr(fast,'raw_snapshot',Mock(side_effect=AssertionError('child must not rescan attachment raw')))
    result=fast.main(['--child-contract',str(fast.CONTRACT),'--contract-sha256',digest(fast.CONTRACT),
        '--parent-pid','345','--parent-start-ticks','123','--',*arguments])
    assert result == 17
    authenticate.assert_called_once_with(345,'123')
    check.assert_called_once_with(arguments)
    assert events == ['population_enter','original_recovery','population_exit']
    fast.original.frozen_context.assert_called_with()
