"""CPU-only repeated crash handoff; original frozen verifier/capability retained."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

import recover_unified_fresh_chain as recovery
from test_recover_unified_fresh import case as initial_case, snapshot, write

common = recovery.common


@pytest.fixture
def case(initial_case):
    c = initial_case
    recovery.first.prepare(c.s)
    target = c.target.with_name('continued-again.json')
    output = c.output.with_name('continued-again')
    s = recovery.selection(c.target, target, output)
    return SimpleNamespace(first=c, s=s, output=output, target=target)


def test_nested_prepare_preserves_every_predecessor_and_original_authority(case):
    c=case;before=snapshot(c.first.old);continued=snapshot(c.first.output)
    profiles=(c.first.profile.read_bytes(), c.first.target.read_bytes())
    result=recovery.prepare(c.s)
    assert result['selected']==2043 and result['deficits']==21 and result['finished']==2064
    assert result['depth']==2 and result['captures_started']==0 and not result['unsafe_suffix_imported']
    assert snapshot(c.first.old)==before and snapshot(c.first.output)==continued
    assert profiles==(c.first.profile.read_bytes(), c.first.target.read_bytes())
    doc,receipt=recovery.prepared(c.s)
    assert doc['fresh_start'] is True
    assert doc['canonical_output']==str(c.first.old)
    assert doc['old_output']==c.first.previous['old_output']
    assert doc['source_authority']==c.first.previous['source_authority']
    assert doc['helper_pins']==c.first.previous['helper_pins']
    assert receipt['original_output']==str(c.first.output)
    assert receipt['new_native_qualification_claimed'] is False
    assert (c.output/'unified_live_qualification.json').read_bytes()==c.first.qpath.read_bytes()
    assert common.historical_verifiers.call_args.args[0]['old_output']==str(c.first.output)
    assert c.first.authority.call_args.args[0]==c.first.previous


def test_third_crash_reuses_checked_chain_without_changing_origin(case):
    c=case;recovery.prepare(c.s)
    next_profile=c.target.with_name('third.json');next_output=c.output.with_name('third')
    next_s=recovery.selection(c.target,next_profile,next_output)
    assert next_s['depth']==3 and next_s['origin']==c.first.previous
    result=recovery.prepare(next_s)
    doc,_=recovery.prepared(next_s)
    assert result['depth']==3 and doc['canonical_output']==str(c.first.old)
    assert doc['old_output']==c.first.previous['old_output']


@pytest.mark.parametrize('field,value',[('canonical_output','changed'),('source_authority',{}),
    ('old_output','changed'),('fixed_pins',{}),('crash_continuation',{}),('fresh_start',False)])
def test_predecessor_profile_tampering_never_becomes_new_authority(case,field,value):
    c=case;doc=common.read(c.first.target);doc[field]=value;write(c.first.target,doc)
    with pytest.raises((RuntimeError,ValueError)):
        recovery.selection(c.first.target,c.target,c.output)


@pytest.mark.parametrize('which',['qualification','receipt','handoff','origin_profile'])
def test_predecessor_evidence_revalidated_before_prepare(case,which):
    c=case
    paths=dict(qualification=c.first.output/'unified_live_qualification.json',
        receipt=c.first.output/'unified_fresh_crash_continuation.json',
        handoff=c.first.output/'unified_crash_handoff.json',origin_profile=c.first.profile)
    write(paths[which],dict(changed=True))
    with pytest.raises((RuntimeError,ValueError,KeyError)):
        recovery.prepare(c.s)
    assert not c.output.exists()


@pytest.mark.parametrize('kind',['same','ancestor_output','nested','profile_in_source','old_profile'])
def test_chain_output_and_profile_cannot_alias_history(case,kind):
    c=case
    output=dict(same=c.first.output,ancestor_output=c.first.old,nested=c.first.output/'nested').get(kind,c.output)
    profile=(c.first.output/'new.json' if kind=='profile_in_source' else
        c.first.profile if kind=='old_profile' else c.target)
    with pytest.raises(RuntimeError):recovery.selection(c.first.target,profile,output)


@pytest.mark.parametrize('field',['source_authority','canonical_output','old_output','fresh_start',
    'helper_pins','crash_continuation','fixed_pins'])
def test_new_profile_tampering_rejected(case,field):
    c=case;recovery.prepare(c.s)
    doc=common.read(c.target);doc[field]={} if isinstance(doc[field],dict) else 'changed';write(c.target,doc)
    with pytest.raises(RuntimeError):recovery.prepared(c.s)


@pytest.mark.parametrize('field',['unsafe_suffix_imported','new_native_qualification_claimed',
    'captures_started_during_prepare','origin_profile_sha256','depth','handoff_sha256'])
def test_new_receipt_tampering_rejected(case,field):
    c=case;recovery.prepare(c.s);path=c.output/recovery.RECEIPT
    doc=common.read(path);doc[field]='changed';write(path,doc)
    with pytest.raises(RuntimeError):recovery.prepared(c.s)


def test_adapter_verifies_actual_nested_import_but_original_empty_origin(case):
    c=case;recovery.prepare(c.s);doc,_=recovery.prepared(c.s)
    before=c.first.original_prefix.call_count
    with recovery.installed(c.s):
        assert common.source_authority(doc,c.first.parent)==c.first.previous['source_authority']
        common.verify_fresh_prefix(doc)
        with pytest.raises(RuntimeError):common.source_authority(dict(doc,output='/other'),c.first.parent)
    assert c.first.original_prefix.call_count==before+1
    assert c.first.original_prefix.call_args.args[0]==c.first.previous
    assert c.first.importer.verify.call_args.args[0]==c.output/'imported_prefix.json'


def test_actual_sealed_worker_identity_checks_use_only_new_pinned_wrapper(case,monkeypatch):
    c=case;recovery.prepare(c.s);doc,_=recovery.prepared(c.s)
    original_read=Path.read_bytes;proc=Path('/proc')/str(os.getppid())/'cmdline'
    command=[sys.executable,'-B',str(Path(recovery.__file__).absolute()),'--run']
    monkeypatch.setattr(Path,'read_bytes',lambda self:
        b'\0'.join(os.fsencode(v) for v in command)+b'\0' if self==proc else original_read(self))
    fd=common.sealed_payload(dict(profile=doc,parent=common.identity(os.getppid())))
    original_file=common.__file__
    try:
        with recovery.installed():
            assert common.worker_payload(fd,common.identity(os.getppid()))['profile']==doc
            assert common.__file__==original_file
            command[-1]='--prepare'
            with pytest.raises(RuntimeError,match='mode identity'):
                common.worker_payload(fd,common.identity(os.getppid()))
    finally:os.close(fd)


def test_native_dispatch_keeps_memfd_seals_parent_and_target_scope(case,monkeypatch):
    c=case;recovery.prepare(c.s);doc,_=recovery.prepared(c.s);calls=[]
    def base(command,*args,**kwargs):
        fd=int(command[command.index('--session-fd')+1]);assert fd in kwargs['pass_fds']
        value=json.loads(os.pread(fd,os.fstat(fd).st_size,0))
        assert value['profile']==doc and value['boundary']==dict(verified=True)
        calls.append(command)
    monkeypatch.setattr(recovery.subprocess,'Popen',base)
    target=common.SCRIPTS/'run_base5_capture_static_attitude.py';output=c.output/'chunk_0168/capture'
    command=[sys.executable,str(target),'--manifest',str(output.parent/'execution_source/manifest.json'),
        '--output',str(output),'--enable-static-surface-attitude','--allow-static-pedestrians',
        '--setup-profile','static-ready','--shared-ros-batch','--limit','16']
    with recovery.installed(c.s),common.routed_workers(doc,SimpleNamespace(snapshot=lambda:dict(verified=True))):
        recovery.subprocess.Popen(command)
        foreign=list(command);foreign[foreign.index('--output')+1]='/foreign/capture'
        with pytest.raises(RuntimeError,match='escaped'):recovery.subprocess.Popen(foreign)
    assert len(calls)==1 and calls[0][2]==str(Path(recovery.__file__).absolute())


def test_import_path_includes_frozen_verifiers_without_environment_workaround():
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(Path(recovery.__file__).parent))
    code='import recover_unified_fresh_chain as r; batch, importer=r.common.handoff._modules(); assert batch and importer'
    subprocess.run([sys.executable,'-B','-c',code],env=env,check=True,capture_output=True,timeout=20)


def test_cyclic_and_unknown_profile_lineage_rejected(case):
    c=case;recovery.prepare(c.s)
    doc=common.read(c.target);doc['crash_continuation']['source_profile']=str(c.target);write(c.target,doc)
    with pytest.raises(RuntimeError,match='cyclic'):recovery.selection(c.target,c.target.with_name('next.json'),c.output.with_name('next'))
    doc['crash_continuation']['schema']='unknown';write(c.target,doc)
    with pytest.raises(RuntimeError,match='Unknown'):recovery.selection(c.target,c.target.with_name('next.json'),c.output.with_name('next'))


@pytest.mark.parametrize('argv',[['--run'],['--prepare','--confirmed-environment','foggy13'],
    ['--stop-after-chunks','1']])
def test_cli_never_implicitly_starts_native_capture(case,argv):
    c=case
    with pytest.raises(RuntimeError):recovery.main(argv+['--source-profile',str(c.first.target),
        '--profile',str(c.target),'--output',str(c.output)])
