"""공통 runtime 배포본은 로컬에서만 만들고 기존 파일을 덮어쓰지 않는다."""
import json
import os
from pathlib import Path
import stat
from unittest.mock import Mock

import pytest
import package_unified as package


def write(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value)+'\n' if not isinstance(value,str) else value)
    return path


@pytest.fixture
def fixture(tmp_path,monkeypatch):
    scripts=tmp_path/'repo/dataset/morai_od/scripts'
    helpers=tmp_path/'repo/scripts/portable_base5'
    model=tmp_path/'repo/perception/detection3d/bevfusion'
    runtime={}
    names=['legacy_'+str(index)+'.py' for index in range(360)]+sorted(package.borrowed.EXTRA_SCRIPTS)
    for name in names:
        path=write(scripts/name,'# frozen '+name+'\n')
        runtime[str(path)]=package.sha(path)
    for index in range(67):
        path=write(model/('model_'+str(index)+'.py'),'# preserved model '+str(index)+'\n')
        runtime[str(path)]=package.sha(path)
    for name in package.borrowed.REQUIRED_HELPERS:
        write(helpers/name,'# helper '+name+'\n')
    release=write(tmp_path/'frozen/release.json',dict(runtime_hashes=runtime,bulk_ready=True,environment='sunny15'))
    monkeypatch.setattr(package,'SCRIPTS',scripts)
    monkeypatch.setattr(package,'HELPERS',helpers)
    monkeypatch.setattr(package,'MODEL_ROOT',model)
    return dict(root=tmp_path,scripts=scripts,helpers=helpers,release=release,
        output=tmp_path/'new_package',workspace='/home/ddeungbuntu/ajounice_base5_capture',intent='a'*64)


def invoke(f,run=False):
    return package.prepare(f['release'],f['workspace'],f['intent'],f['output'],run)


def snapshot(root):
    return {str(path):path.read_bytes() for path in root.rglob('*') if path.is_file()}


def test_plan_is_readonly_with_exact438_and_helpers7(fixture):
    f=fixture;before=snapshot(f['root'])
    result=invoke(f)
    assert result['runtime_files']==438 and result['helper_files']==7
    assert result['overlay_scripts']==371 and result['preserved_external_runtime_files']==67
    assert not result['files_written'] and not result['remote_access']
    assert not result['ros_started'] and not result['capture_started']
    assert not f['output'].exists() and snapshot(f['root'])==before
    assert all(path.startswith(f['workspace']+'/work/unified-runtime-v1/scripts/')
        for path in result['deployment']['runtime_hashes'])
    assert all(path.startswith(f['workspace']+'/work/unified-runtime-v1/checks/')
        for path in result['deployment']['helper_hashes'])


def test_run_copies_exact_bytes_private_modes_and_remote_deployment_pins(fixture):
    f=fixture;before=snapshot(f['root'])
    result=invoke(f,True)
    assert result['files_written'] and not result['capture_started']
    assert len(list((f['output']/'scripts').glob('*.py')))==371
    assert len(list((f['output']/'checks').glob('*.py')))==7
    deployment=json.loads((f['output']/'deployment.json').read_text())
    assert deployment['recovery_intent_sha256']==f['intent']
    for kind,key in (('scripts','runtime_hashes'),('checks','helper_hashes')):
        for remote,digest in deployment[key].items():
            assert package.sha(f['output']/kind/Path(remote).name)==digest
    for path in [f['output'],*f['output'].rglob('*')]:
        assert stat.S_IMODE(path.stat().st_mode)==(0o700 if path.is_dir() else 0o600)
    assert all(Path(path).read_bytes()==value for path,value in before.items())
    assert not (f['output']/'scripts/generation.py').exists()


@pytest.mark.parametrize('kind',['changed','missing','extra','symlink','outside_source','wrong_count','not_qualified'])
def test_invalid_frozen_runtime_fails_before_creating_any_output(fixture,kind):
    f=fixture;release=json.loads(f['release'].read_text());source=f['scripts']/'legacy_0.py'
    if kind=='changed':
        write(source,'# changed')
    elif kind=='missing':
        source.unlink()
    elif kind=='extra':
        write(f['scripts']/'generation.py','# forbidden new script')
    elif kind=='symlink':
        source.unlink();source.symlink_to(f['scripts']/'legacy_1.py')
    elif kind=='outside_source':
        digest=release['runtime_hashes'].pop(str(source))
        release['runtime_hashes'][str(f['root']/'elsewhere.py')]=digest
    elif kind=='wrong_count':
        release['runtime_hashes'].pop(str(source))
    else:
        release['bulk_ready']=False
    write(f['release'],release)
    with pytest.raises((ValueError,OSError)):
        invoke(f,True)
    assert not f['output'].exists()


@pytest.mark.parametrize('workspace',['relative','/root/ajounice_base5_capture','/home/a/../ajounice_base5_capture',
    '/home/a/other','/home/a/ajounice_base5_capture/','//home/a/ajounice_base5_capture'])
def test_remote_workspace_is_only_a_canonical_target_not_an_arbitrary_path(workspace):
    with pytest.raises(ValueError):
        package.remote_workspace(workspace)


@pytest.mark.parametrize('intent',['', 'A'*64, 'a'*63, 'xyz'])
def test_explicit_historical_intent_sha_is_required(fixture,intent):
    fixture['intent']=intent
    with pytest.raises(ValueError):
        invoke(fixture)


@pytest.mark.parametrize('kind',['existing','partial','symlink','source_subdir','helper_subdir','release_subdir'])
def test_never_overwrites_existing_or_protected_outputs(fixture,kind):
    f=fixture
    if kind=='existing':
        f['output'].mkdir()
    elif kind=='partial':
        write(f['output']/'partial.txt','operator data')
    elif kind=='symlink':
        f['output'].symlink_to(f['scripts'],target_is_directory=True)
    else:
        f['output']={'source_subdir':f['scripts']/'new','helper_subdir':f['helpers']/'new',
                     'release_subdir':f['release'].parent/'new'}[kind]
    before=snapshot(f['root'])
    with pytest.raises(ValueError):
        invoke(f,True)
    assert before==snapshot(f['root'])


def test_racing_output_is_not_overwritten(fixture,monkeypatch):
    f=fixture;original=package.plan
    def raced(*args,**kwargs):
        result=original(*args,**kwargs)
        write(f['output']/'raced.txt','do not touch')
        return result
    monkeypatch.setattr(package,'plan',raced)
    with pytest.raises(FileExistsError):
        invoke(f,True)
    assert (f['output']/'raced.txt').read_text()=='do not touch'


def test_changed_source_during_copy_never_gets_a_success_deployment(fixture,monkeypatch):
    f=fixture;original=package.copy_verified;first=True
    def changed(source,destination,expected):
        nonlocal first
        if first:
            first=False;write(source,'# external change')
        original(source,destination,expected)
    monkeypatch.setattr(package,'copy_verified',changed)
    with pytest.raises(ValueError,match='Input changed'):
        invoke(f,True)
    assert not (f['output']/'deployment.json').exists()


def test_missing_helper_prevents_any_staging_write(fixture):
    f=fixture;(f['helpers']/'session_input_boundary.py').unlink()
    with pytest.raises(OSError):
        invoke(f,True)
    assert not f['output'].exists()


@pytest.mark.parametrize('run',[False,True])
def test_cli_defaults_to_compact_summary_without_full_input_or_deployment_inventories(fixture,capsys,run):
    f=fixture
    arguments=['--release',str(f['release']),'--workspace',f['workspace'],
        '--recovery-intent-sha256',f['intent'],'--output',str(f['output'])]
    if run:
        arguments.append('--run')
    package.main(arguments)
    output=capsys.readouterr().out
    result=json.loads(output)
    assert result['event']==('unified_package_created' if run else 'unified_package_plan')
    assert result['runtime_files']==438 and result['overlay_scripts']==371
    assert result['preserved_external_runtime_files']==67 and result['helper_files']==7
    assert result['native_captures']==0 and not result['remote_access']
    assert result['files_written'] is run
    assert 'source_pins' not in result and 'deployment' not in result
    assert len(output)<1500
    assert bool(result['deployment_sha256']) is run


def test_verbose_cli_explicitly_preserves_complete_inventory_output(fixture,capsys):
    f=fixture
    package.main(['--release',str(f['release']),'--workspace',f['workspace'],
        '--recovery-intent-sha256',f['intent'],'--output',str(f['output']),'--verbose'])
    result=json.loads(capsys.readouterr().out)
    assert len(result['deployment']['source_runtime_hashes'])==438
    assert len(result['deployment']['runtime_hashes'])==371
    assert len(result['deployment']['helper_hashes'])==7
    assert result['source_pins']
    assert not result['files_written'] and not f['output'].exists()
