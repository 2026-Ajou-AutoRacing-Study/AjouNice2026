"""빌린 PC 접속 없이 새 환경 namespace·명령·인증 분리를 검사한다."""
import copy
import json
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import fresh_borrowed as fresh


def write(path, value):
    path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    path.write_text(value if isinstance(value,str) else json.dumps(value)+'\n');path.chmod(0o600)
    return path


@pytest.fixture
def deployment(tmp_path,monkeypatch):
    home=tmp_path/'home';w=home/fresh.WORKSPACE;r=w/'work/fresh-runtime-v1/sunny11'
    w.mkdir(parents=True,mode=0o700)
    r.mkdir(parents=True,mode=0o700)
    p=dict(root=r,checks=r/'checks',scripts=r/'scripts',profile=r/'profile.json',
        output=w/'collections/sunny11_5k_unified_v1',deployment=r/'deployment.json')
    for name in fresh.REQUIRED_HELPERS:write(p['checks']/name,'# pinned '+name+'\n')
    for name in fresh.REQUIRED_RUNTIME:write(p['scripts']/name,'# original '+name+'\n')
    original_repo=w/'capture_bundle/payload/home/luv/AjouNice2026'
    original_raw=w/'capture_bundle/payload/home/luv/morai_cap'
    original_repo.mkdir(parents=True,mode=0o700)
    write(original_raw/'previous_source/keep.json',dict(preserved=True))
    write(original_repo/'old_code.py','# old code\n')
    source=r/'source_payload/new_environment';target=fresh.RAW/'base5_sunny11_new'
    scenario=write(source/'scene.json',dict(scene=1));meta=write(source/'meta.json',dict(split='train'))
    rows=[dict(id='scene_%05d'%i,scenario=str(target/'scene.json'),scenario_sha256=fresh.sha(scenario),
               metadata=str(target/'meta.json'),split='train') for i in range(5000)]
    manifest=write(source/'manifest.json',dict(count=5000,scenes=rows,config=dict(environment_tag='sunny11')))
    witness=write(source/'route.json',dict(schema='base5_fresh_environment_route_witness_v1',
        environment='sunny11',manifest_sha256=fresh.sha(manifest)))
    doc=dict(schema=fresh.SCHEMA,workspace=str(w),environment='sunny11',manifest=str(target/'manifest.json'),
        manifest_sha256=fresh.sha(manifest),output=str(p['output']),
        input_hashes={str(target/f.name):fresh.sha(f) for f in source.iterdir()},
        source_runtime_hashes={str(fresh.SCRIPTS/f.name):fresh.sha(f) for f in p['scripts'].iterdir()},
        runtime_hashes={str(f):fresh.sha(f) for f in p['scripts'].iterdir()},
        helper_hashes={str(f):fresh.sha(f) for f in p['checks'].iterdir()},
        bindings=[dict(source=str(source),target=str(target))],
        route_witness=dict(path=str(target/'route.json'),sha256=fresh.sha(witness)))
    write(p['deployment'],doc)
    (w/'collections/sunny13_completed').mkdir(parents=True,mode=0o700)
    session=w/'work/ros-session-v2';session.mkdir(mode=0o700)
    for name in ('host_contract.json','qualification.json'):write(session/name,dict(old_platform=True))
    graph=dict(bindings=[dict(bundle_path='payload/home/luv/AjouNice2026',logical_root=str(fresh.PROJECT)),
                         dict(bundle_path='payload/home/luv/morai_cap',logical_root=str(fresh.RAW))])
    platform=dict(workspace=w,bundle=w/'capture_bundle',graph=graph,session=session,
        mr=home/'MoraiLauncher_Lin',home=w/'work/ros-home-v2')
    old=SimpleNamespace(load=Mock(return_value=platform),ready=Mock(),verify_qualification=Mock(),
        namespace=Mock(side_effect=lambda ctx,command:['bwrap','--ro-bind','/','/','--chdir',str(fresh.PROJECT),'--',*command]),
        environment=Mock(return_value=dict(PYTHONPATH='/run/base5-bundle/python',PATH='/usr/bin')),
        port_open=Mock(return_value=True),service_owned=Mock(return_value=True),signal_inner_app=Mock(),
        write_new=lambda path,value:write(path,value))
    monkeypatch.setattr(Path,'home',lambda:home)
    monkeypatch.setattr(fresh,'__file__',str(p['checks']/'fresh_borrowed.py'))
    monkeypatch.setattr(fresh,'load_original',lambda workspace:old)
    return SimpleNamespace(home=home,w=w,p=p,doc=doc,old=old,source=source,target=target,rows=rows,
        original_raw=original_raw,original_repo=original_repo)


def load(f):return fresh.load(f.p['deployment'],'sunny11')


def profile(f):
    write(f.source/'release.json',dict(new_release=True))
    f.doc.update(release=str(f.target/'release.json'),release_sha256=fresh.sha(f.source/'release.json'))
    write(f.p['deployment'],f.doc)
    write(f.p['profile'],dict(fresh_start=True,environment='sunny11',manifest=f.doc['manifest'],
        manifest_sha256=f.doc['manifest_sha256'],release=f.doc['release'],release_sha256=f.doc['release_sha256'],
        output=str(f.p['output']),route_witness=f.doc['route_witness']))


def test_source_only_load_does_not_claim_previous_source_qualification(deployment):
    f=deployment;before={str(p):p.read_bytes() for p in f.w.rglob('*') if p.is_file()}
    ctx=load(f)
    assert ctx['environment']=='sunny11'
    f.old.load.assert_called_once_with(f.w,verify=True);f.old.ready.assert_called_once()
    f.old.verify_qualification.assert_not_called()
    assert before=={str(p):p.read_bytes() for p in f.w.rglob('*') if p.is_file()}


@pytest.mark.parametrize('environment',fresh.ENVIRONMENTS)
def test_environment_selection_has_no_sunny11_algorithm_fork(deployment,environment):
    f=deployment;doc=copy.deepcopy(f.doc);doc['environment']=environment
    doc['output']=str(f.w/'collections'/(environment+'_5k_fresh'))
    manifest=f.source/'manifest.json';data=json.loads(manifest.read_text());data['config']['environment_tag']=environment
    write(manifest,data);doc['manifest_sha256']=fresh.sha(manifest)
    write(f.source/'route.json',dict(schema='base5_fresh_environment_route_witness_v1',
        environment=environment,manifest_sha256=doc['manifest_sha256']))
    doc['route_witness']['sha256']=fresh.sha(f.source/'route.json')
    doc['input_hashes']={str(f.target/p.name):fresh.sha(p) for p in f.source.iterdir()}
    write(f.p['deployment'],doc)
    assert fresh.load(f.p['deployment'],environment)['environment']==environment


@pytest.mark.parametrize('kind',['script','helper','source','manifest','witness','extra_helper','symlink','env','output'])
def test_changed_or_foreign_deployments_fail_closed(deployment,kind):
    f=deployment
    if kind=='script':write(f.p['scripts']/'run_base5_capture.py','# changed')
    elif kind=='helper':write(f.p['checks']/'unified_capture.py','# changed')
    elif kind=='source':write(f.source/'scene.json',dict(changed=True))
    elif kind=='manifest':write(f.source/'manifest.json',dict(count=0))
    elif kind=='witness':write(f.source/'route.json',dict(environment='sunny13'))
    elif kind=='extra_helper':write(f.p['checks']/'extra.py','# not pinned')
    elif kind=='symlink':
        p=f.p['checks']/'unified_capture.py';p.unlink();p.symlink_to(f.p['checks']/'fresh_borrowed.py')
    elif kind=='env':f.doc['environment']='sunny13';write(f.p['deployment'],f.doc)
    else:f.doc['output']=str(f.w/'collections/sunny13_completed');write(f.p['deployment'],f.doc)
    with pytest.raises((ValueError,KeyError)):load(f)


def test_missing_target_uses_mount_only_merge_without_old_source_copy(deployment):
    f=deployment;ctx=load(f);argv=fresh.namespace(ctx,['python3','read_only.py'])
    assert ['--tmpfs',str(fresh.RAW)]==argv[argv.index('--tmpfs'):argv.index('--tmpfs')+2]
    readonly=[(argv[i+1],argv[i+2]) for i,v in enumerate(argv) if v=='--ro-bind']
    assert (str(f.original_raw/'previous_source'),str(fresh.RAW/'previous_source')) in readonly
    assert (str(f.source),str(f.target)) in readonly
    assert (str(f.w/'collections/sunny13_completed'),str(f.w/'collections/sunny13_completed')) in readonly
    assert (str(f.p['output']),str(f.p['output'])) not in readonly
    assert '--unshare-net' not in argv
    assert argv[-2:]==['python3','read_only.py']


def test_prior_work_evidence_is_mounted_readonly(deployment):
    f=deployment;prior=f.w/'work/prior_native';prior.mkdir(mode=0o700)
    f.doc['preserved_roots']=[str(prior)];write(f.p['deployment'],f.doc)
    argv=fresh.namespace(load(f),['python3','read_only.py'])
    assert any(argv[i:i+3]==['--ro-bind',str(prior),str(prior)] for i in range(len(argv)-2))


def test_native_adoption_can_pin_actual_simulator_asset_but_not_other_home(deployment):
    f=deployment;ctx=load(f)
    asset=write(ctx['mr']/'MoraiLauncher_Lin_Data/SaveFile/Scenario/assets.json',{'assets':[]})
    assert fresh.mapped(ctx,str(asset))==asset
    fresh.verify_pins(ctx,{str(asset):fresh.sha(asset)})
    with pytest.raises(ValueError,match='escaped'):
        fresh.mapped(ctx,str(f.home/'unrelated.json'))


@pytest.mark.parametrize('kind',['outside','ancestor','descendant'])
def test_prior_work_mount_cannot_cover_current_deployment_or_other_home(deployment,kind):
    f=deployment
    root={'outside':f.w,'ancestor':f.p['root'].parent,'descendant':f.p['checks']}[kind]
    root.chmod(0o700)
    f.doc['preserved_roots']=[str(root)];write(f.p['deployment'],f.doc)
    with pytest.raises(ValueError,match='separate prior root'):
        fresh.namespace(load(f),['python3','read_only.py'])


def test_full_named_pin_root_overlay_needs_no_tmpfs_or_repository_copy(deployment):
    f=deployment;f.doc['bindings'].append(dict(source=str(f.p['root']/'source_payload/repository'),target=str(fresh.PROJECT)))
    replacement=Path(f.doc['bindings'][-1]['source']);replacement.mkdir(mode=0o700)
    write(f.p['deployment'],f.doc);ctx=load(f)
    assert fresh.mapped(ctx,str(fresh.PROJECT/'future.py'))==replacement/'future.py'
    # The fresh RAW child still needs a mount-only merge, the existing PROJECT root does not.
    mounts=fresh.merged_mounts(ctx)
    assert str(fresh.PROJECT) not in [mounts[i+1] for i,v in enumerate(mounts) if v=='--tmpfs']


def test_prepare_requires_new_release_then_uses_fresh_engine_api(deployment):
    f=deployment;ctx=load(f)
    with pytest.raises(ValueError,match='qualified release'):fresh.command(ctx,'prepare')
    write(f.source/'release.json',dict(new_release=True))
    f.doc.update(release=str(f.target/'release.json'),release_sha256=fresh.sha(f.source/'release.json'))
    write(f.p['deployment'],f.doc);ctx=load(f);argv=fresh.command(ctx,'prepare')
    assert '--prepare-fresh' in argv and '--old-output' not in argv and '--run' not in argv
    assert argv[argv.index('--release')+1]==f.doc['release']


def test_qualification_and_bulk_are_separate_and_env_bound(deployment):
    f=deployment;profile(f);ctx=load(f)
    qualify=fresh.command(ctx,'qualify','0,1,4840');run=fresh.command(ctx,'run')
    assert '--qualify-prototype' in qualify and '--run' not in qualify
    assert '--run' in run and '--indices' not in run and '--complete' not in run
    assert run[-1]=='sunny11'
    p=fresh.read(f.p['profile']);p['manifest_sha256']='f'*64;write(f.p['profile'],p)
    with pytest.raises(ValueError,match='escaped'):fresh.command(ctx,'run')


def test_prototype_intent_is_canonical_bounded_and_not_bulk_authority(deployment):
    f=deployment;ctx=load(f)
    subset=write(f.source/'subset.json',dict(count=2,scenes=f.rows[:2],config=dict(environment_tag='sunny11')))
    doc=fresh.build_prototype_intent(ctx,f.target/'subset.json',fresh.sha(subset),f.p['root']/'qualification/exact',limit=2)
    assert doc['prototype_only'] and doc['bulk_authorized'] is False and doc['source_qualified'] is False
    intent=write(f.p['root']/'prototype.json',doc)
    argv=fresh.prototype_command(ctx,intent,fresh.sha(intent),resume=True,stop_after_chunks=1)
    assert '--role' in argv and argv[argv.index('--role')+1]=='prototype' and '--resume' in argv
    for limit in (0,49,5000):
        with pytest.raises(ValueError,match='1..48'):
            fresh.build_prototype_intent(ctx,f.target/'subset.json',fresh.sha(subset),f.p['root']/'qualification/exact',limit=limit)
    with pytest.raises(ValueError,match='separate qualification'):
        fresh.build_prototype_intent(ctx,f.target/'subset.json',fresh.sha(subset),f.p['output'],limit=2)


def test_prototype_cannot_change_frozen_entry_to_similar_lookalike(deployment):
    f=deployment;ctx=load(f);rows=copy.deepcopy(f.rows[:2]);rows[1]['split']='val'
    path=write(f.source/'subset.json',dict(count=2,scenes=rows,config=dict(environment_tag='sunny11')))
    with pytest.raises(ValueError,match='unchanged entries'):
        fresh.build_prototype_intent(ctx,f.target/'subset.json',fresh.sha(path),f.p['root']/'qualification/test',limit=2)


def test_source_plan_is_nonmutating_and_never_starts_ros_or_capture(deployment,monkeypatch,capsys):
    f=deployment;spawn=Mock();monkeypatch.setattr(fresh.subprocess,'Popen',spawn)
    assert fresh.main(['--deployment',str(f.p['deployment']),'--environment','sunny11','source-plan'])==0
    result=json.loads(capsys.readouterr().out)
    assert result['source_qualified'] is False and result['files_written'] is False
    spawn.assert_not_called()


@pytest.mark.parametrize('tail',[['run'],['qualify','--indices','0,1'],['prepare','--confirmed-environment','sunny11'],
    ['run','--confirmed-environment','sunny13'],['source-plan','--indices','0,1']])
def test_capture_confirmation_cannot_be_omitted_or_reused_for_other_actions(tail,monkeypatch,tmp_path):
    load=Mock();monkeypatch.setattr(fresh,'load',load)
    with pytest.raises(ValueError):
        fresh.main(['--deployment',str(tmp_path/'deployment.json'),'--environment','sunny11',*tail])
    load.assert_not_called()
