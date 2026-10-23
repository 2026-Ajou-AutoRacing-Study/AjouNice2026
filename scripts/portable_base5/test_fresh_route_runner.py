"""정식 <=48장 인증의 원형 CLI·자식 프로세스 계약을 검사한다."""
import copy
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

import fresh_borrowed as borrowed
import fresh_route_runner as runner
import unified_capture as common


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    path.write_text(value if isinstance(value,str) else json.dumps(value)+'\n');path.chmod(0o600)
    return path


@pytest.fixture
def fixture(tmp_path,monkeypatch):
    root=tmp_path/'runtime';root.mkdir(mode=0o700)
    helpers=root/'checks';scripts=root/'scripts'
    own=write(helpers/'fresh_route_runner.py','# pinned runner')
    code=write(scripts/'generate_base5_independent.py','# generator')
    scene=write(root/'source/scene.json',{'scenario':1});meta=write(root/'source/meta.json',{'metadata':1})
    rows=[dict(id=str(i),scenario=str(scene),scenario_sha256=borrowed.sha(scene),metadata=str(meta)) for i in range(5000)]
    manifest=write(root/'source/manifest.json',dict(count=5000,scenes=rows,config=dict(environment_tag='foggy15')))
    subset=write(root/'source/subset.json',dict(count=2,scenes=rows[:2],config=dict(environment_tag='foggy15')))
    doc=dict(schema=runner.SCHEMA,environment='foggy15',canonical_manifest=str(manifest),
        canonical_manifest_sha256=borrowed.sha(manifest),manifest=str(subset),manifest_sha256=borrowed.sha(subset),
        output=str(root/'qualification/exact'),limit=2,chunk_size=16,prototype_only=True,bulk_authorized=False,
        source_qualified=False,runner=str(own),scripts=str(scripts),
        input_hashes={str(p):borrowed.sha(p) for p in (manifest,subset,scene,meta)},
        runtime_hashes={str(code):borrowed.sha(code)},helper_hashes={str(own):borrowed.sha(own)},
        route_witness=dict(path=str(root/'witness.json'),sha256='a'*64))
    intent=write(root/'intent.json',doc)
    monkeypatch.setattr(runner,'__file__',str(own));monkeypatch.setattr(borrowed,'SCRIPTS',scripts)
    monkeypatch.setattr(common,'SCRIPTS',scripts)
    return root,doc,intent


def parent_values(doc):
    return ['--manifest',doc['manifest'],'--output',doc['output'],'--environment',doc['environment'],
        '--confirmed-environment',doc['environment'],'--limit',str(doc['limit']),
        '--chunk-size',str(doc['chunk_size']),'--stop-after-chunks','1','--run']


def test_valid_private_source_intent_is_readonly(fixture):
    root,doc,intent=fixture;before={str(p):p.read_bytes() for p in root.rglob('*') if p.is_file()}
    assert runner.validate_intent(intent,borrowed.sha(intent))==doc
    runner.validate_arguments('prototype',parent_values(doc),doc)
    assert before=={str(p):p.read_bytes() for p in root.rglob('*') if p.is_file()}


@pytest.mark.parametrize('change',['bulk','limit','environment','helper','manifest','output'])
def test_source_intent_cannot_claim_bulk_or_change_source(fixture,change):
    root,doc,intent=fixture
    if change=='bulk':doc['bulk_authorized']=True
    elif change=='limit':doc['limit']=5000
    elif change=='environment':doc['environment']='sunny13'
    elif change=='helper':next(iter(doc['helper_hashes']));doc['helper_hashes']={str(root/'unknown.py'):'f'*64}
    elif change=='manifest':doc['manifest_sha256']='f'*64
    else:doc['output']=str(root.parent/'unapproved')
    write(intent,doc)
    with pytest.raises((RuntimeError,FileNotFoundError)):runner.validate_intent(intent,borrowed.sha(intent))


@pytest.mark.parametrize('extra',[['--approve-bulk-capture'],['--qualified-release','fake'],['--disable-qa'],
    ['--replenish'],['--network-template','fake'],['--limit','5000'],['--run']])
def test_parent_cannot_override_limits_qa_or_authority(fixture,extra):
    _,doc,_=fixture
    with pytest.raises(RuntimeError):runner.validate_arguments('prototype',parent_values(doc)+extra,doc)


def test_child_router_preserves_original_popen_kwargs_and_rechecks_scope(fixture,monkeypatch):
    _,doc,intent=fixture
    source=Path(doc['output'])/'chunk_0000/execution_source/manifest.json'
    output=Path(doc['output'])/'chunk_0000/capture'
    values=['--manifest',str(source),'--output',str(output),'--enable-static-surface-attitude',
        '--allow-static-pedestrians','--setup-profile','static-ready','--shared-ros-batch','--limit','2']
    target=borrowed.SCRIPTS/runner.TARGETS['static'];spawn=Mock(return_value='child')
    monkeypatch.setattr(runner.subprocess,'Popen',spawn)
    with runner.routed(intent,borrowed.sha(intent),doc):
        assert runner.subprocess.Popen([runner.sys.executable,str(target),*values],stdout=1,start_new_session=True)=='child'
    argv=spawn.call_args.args[0]
    assert argv[argv.index('--role')+1]=='static' and argv[-len(values):]==values
    assert spawn.call_args.kwargs==dict(stdout=1,start_new_session=True)


def test_native_capture_leaf_is_one_scene_and_cannot_escape(fixture):
    _,doc,_=fixture
    source=str(Path(doc['output'])/'retry/deck/manifest.json');output=str(Path(doc['output'])/'retry/native')
    runner.validate_arguments('capture',['--manifest',source,'--output',output,'--limit','1'],doc)
    with pytest.raises(RuntimeError):
        runner.validate_arguments('capture',['--manifest',source,'--output',output,'--limit','5000'],doc)


def test_foreign_or_shell_native_child_does_not_launch(fixture,monkeypatch):
    _,doc,intent=fixture;spawn=Mock();monkeypatch.setattr(runner.subprocess,'Popen',spawn)
    with runner.routed(intent,borrowed.sha(intent),doc):
        with pytest.raises(RuntimeError):runner.subprocess.Popen('python run_base5_capture.py')
        with pytest.raises(RuntimeError):runner.subprocess.Popen(['/foreign/python',str(borrowed.SCRIPTS/'run_base5_capture.py')])
    spawn.assert_not_called()


def test_route_and_population_context_install_for_every_role_and_original_argv_restored(fixture,monkeypatch):
    _,doc,intent=fixture;calls=[]
    from contextlib import contextmanager
    @contextmanager
    def installed(*args,**kwargs):calls.append(kwargs);yield
    gen=object();extent=object()
    monkeypatch.setattr(runner.route,'installed',installed)
    monkeypatch.setattr(runner.recovery_population,'installed',installed)
    monkeypatch.setattr(runner.importlib,'import_module',lambda name:gen if name=='generate_base5_independent' else extent)
    original=runner.sys.argv;program=Mock();monkeypatch.setattr(runner.runpy,'run_path',program)
    assert runner.run_role(intent,borrowed.sha(intent),'prototype',parent_values(doc))==0
    assert runner.sys.argv is original and len(calls)==2
    assert calls[0]['manifest']==doc['canonical_manifest']
    assert program.call_args.args[0]==str(borrowed.SCRIPTS/'run_base5_batch_static_attitude.py')
