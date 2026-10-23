"""CPU-only fixtures for a separate read-only HTTP namespace. No remote/ROS."""
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import shutil
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import remote_web_dashboard as web

SOURCE = Path(__file__).resolve().parents[2]/'dataset/morai_od/scripts'


def write(filename,value):
    filename.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
    filename.write_text(json.dumps(value)+'\n');filename.chmod(0o600)
    return filename


@pytest.fixture
def fixture(tmp_path,monkeypatch):
    workspace = tmp_path/'home/ajounice_base5_capture';checks = workspace/'checks'
    checks.mkdir(parents=True,mode=0o700)
    workspace.chmod(0o700)
    helper = checks/'remote_web_dashboard.py';helper.write_text('# deployed fixture helper\n');helper.chmod(0o600)
    monkeypatch.setattr(web,'__file__',str(helper))
    root = workspace/'collections'/web.OUTPUT_NAME;root.mkdir(parents=True,mode=0o700)
    previous = workspace/'collections/sunny13_5k';previous.mkdir(mode=0o700)
    bundle = workspace/'capture_bundle';bundle.mkdir(mode=0o700)
    physical = write(bundle/'payload/repo/source/manifest.json',dict(count=5000,
        scenes=[dict(id='source_%04d' % i) for i in range(5000)]))
    monkeypatch.setattr(web,'MANIFEST_SHA',web.sha(physical))
    source = '/home/luv/AjouNice2026/source/manifest.json'
    graph = dict(manifest=dict(source=source,sha256=web.MANIFEST_SHA,bundle_path=str(physical.relative_to(bundle))),
        release=dict(source='/home/luv/AjouNice2026/release.json',sha256=web.RELEASE_SHA),
        bindings=[dict(logical_root='/home/luv/AjouNice2026',bundle_path='payload/repo')])
    mr = tmp_path/'home/MoraiLauncher_Lin';mr.mkdir(mode=0o700)
    ctx = dict(workspace=workspace,bundle=bundle,mr=mr,graph=graph)
    def namespace(ctx,command):
        return [str(bundle/'bin/bwrap'),'--ro-bind','/','/',
            '--bind',str(workspace),str(workspace),'--bind',str(mr),str(mr),
            '--ro-bind',str(bundle/'payload/repo'),'/home/luv/AjouNice2026',
            '--chdir','/home/luv/AjouNice2026','--',*command]
    live = SimpleNamespace(load=Mock(return_value=ctx),ready=Mock(),namespace=namespace)
    monkeypatch.setattr(web,'load_live',lambda directory:live)
    prefix = write(root/'imported_prefix.json',dict(schema=1,manifest_sha256=web.MANIFEST_SHA,
        original_output=str(previous),deficits=[],selected=[dict(source_cell_id='source_%04d' % i,
            status='accepted_candidate',audit=dict(review_reasons=[]),capture_root=str(previous/('raw_%d' % i)))
            for i in range(69)]))
    intent = write(checks/'population_recovery_intent.json',dict(schema=web.SCHEMA,environment='sunny13',
        workspace=str(workspace),output=str(root),previous_output=str(previous),manifest=source,
        release=graph['release']['source'],source_runtime_count=427,qa_changed=False,
        files={'unchanged_guard':'sha'},import_receipt=str(prefix),import_receipt_sha256=web.sha(prefix)))
    write(root/'population_recovery_contract.json',dict(schema=web.SCHEMA,intent_sha256=web.sha(intent),
        previous_output_preserved=str(previous),original_native_qa=True,original_shutdown_safety=True,
        helper_hashes={'unchanged_guard':'sha'},imported_prefix_sha256=web.sha(prefix)))
    work = workspace/'work'/web.DIRECTORY;state = work/'state';state.mkdir(parents=True,mode=0o700)
    work.chmod(0o700)
    for name in web.ASSETS:
        shutil.copyfile(SOURCE/name,work/name);(work/name).chmod(0o600)
    return SimpleNamespace(workspace=workspace,checks=checks,ctx=ctx,live=live,work=work,state=state,
        root=root,previous=previous,source=source,physical=physical,intent=intent,prefix=prefix,helper=helper)


def files(root):
    return {str(p):p.read_bytes() for p in root.rglob('*') if p.is_file()}


def call(fixture,arguments):
    output = io.StringIO()
    with redirect_stdout(output):
        result = web.main(['--workspace',str(fixture.workspace),*arguments])
    return result,output.getvalue()


def test_default_is_readonly_no_exec_no_mkdir_or_signals(fixture,monkeypatch):
    before = files(fixture.workspace);execute = Mock();kill = Mock();spawn = Mock()
    monkeypatch.setattr(web.os,'execve',execute);monkeypatch.setattr(web.os,'kill',kill)
    monkeypatch.setattr(web.subprocess,'Popen',spawn)
    result,text = call(fixture,[]);plan = json.loads(text)
    assert result == 0 and not plan['files_written'] and not plan['process_started'] and not plan['signals_sent']
    assert plan['imported_accepted'] == 69 and plan['parent_counts_already_include_imported']
    assert plan['child_counts_added'] is False
    assert plan['writable_host_directory'] == str(fixture.state)
    assert files(fixture.workspace) == before
    fixture.live.load.assert_called_once_with(fixture.workspace,verify=False)
    fixture.live.ready.assert_called_once_with(fixture.ctx)
    execute.assert_not_called();kill.assert_not_called();spawn.assert_not_called()


def test_effective_final_mounts_leave_only_new_state_rw(fixture):
    config = web.configuration(fixture.workspace);command = web.namespace_command(config)
    end = command.index('--chdir')
    assert command[end-9:end] == ['--ro-bind',str(fixture.workspace),str(fixture.workspace),
        '--ro-bind',str(fixture.ctx['mr']),str(fixture.ctx['mr']),
        '--bind',str(fixture.state),str(fixture.state)]
    effective = {}
    for i,token in enumerate(command[:end]):
        if token in ('--bind','--ro-bind'):effective[command[i+2]] = token
    assert effective[str(fixture.workspace)] == '--ro-bind'
    assert effective[str(fixture.ctx['mr'])] == '--ro-bind'
    assert [target for target,mode in effective.items() if mode == '--bind'] == [str(fixture.state)]
    assert str(fixture.root/'status.json') not in effective # directory mounts preserve atomic replacement


def test_exact_live_collection_and_manifest_paths_never_alias(fixture):
    config = web.configuration(fixture.workspace);command = web.dashboard_command(config)
    assert command[:3] == ['/usr/bin/python3','-B',str(fixture.work/'base5_capture_dashboard.py')]
    assert command[command.index('--capture-root')+1] == str(fixture.root)
    assert command[command.index('--manifest')+1] == fixture.source
    assert command[command.index('--history-file')+1] == str(fixture.state/'history.jsonl')
    assert '--suite' not in command
    assert config['previous'] == fixture.previous


def test_run_supervises_only_separate_bwrap_http_observer(fixture,monkeypatch):
    child = Mock();child.wait.return_value = 0
    spawn = Mock(return_value=child);monkeypatch.setattr(web.subprocess,'Popen',spawn)
    execute = Mock();monkeypatch.setattr(web.os,'execve',execute)
    before = files(fixture.workspace)
    result,_ = call(fixture,['--run'])
    assert result == 0 and spawn.call_count == 1 and files(fixture.workspace) == before
    command = spawn.call_args.args[0];env = spawn.call_args.kwargs['env']
    assert command[0].endswith('/bin/bwrap')
    assert command[-2:] == ['--port','8075']
    assert all(env[key] == str(fixture.state) for key in ('HOME','TMPDIR','XDG_CACHE_HOME','XDG_CONFIG_HOME'))
    assert env['PYTHONDONTWRITEBYTECODE'] == '1' and 'ROS_MASTER_URI' not in env and 'SSH_AUTH_SOCK' not in env
    assert not any('run_base5_capture' in token or token == '--run' for token in command)
    child.wait.assert_called_once_with();execute.assert_not_called()


def test_helper_stays_in_child_wait_and_propagates_observer_exit(fixture,monkeypatch):
    events = []
    class Child:
        def wait(self):
            # This call is synchronous in the same live helper, not a parent
            # that returned to SSH as soon as it created the namespace.
            events.append(('waiting',os.getpid()))
            assert len(events) == 2
            return 17
    def spawn(command,env):
        events.append(('spawn',os.getpid()))
        return Child()
    monkeypatch.setattr(web.subprocess,'Popen',spawn)
    monkeypatch.setattr(web.os,'execve',Mock(side_effect=AssertionError('No execve observer parent')))
    result,_ = call(fixture,['--run'])
    assert result == 17 and events == [('spawn',os.getpid()),('waiting',os.getpid())]


@pytest.mark.parametrize('name',list(web.ASSETS))
def test_corrupt_asset_blocks_before_exec(fixture,monkeypatch,name):
    (fixture.work/name).write_text('changed original asset')
    execute = Mock();monkeypatch.setattr(web.os,'execve',execute)
    with pytest.raises(ValueError):call(fixture,['--run'])
    execute.assert_not_called()


@pytest.mark.parametrize('field,value',[
    ('output','/foreign/collection'),('previous_output','/foreign/old'),('manifest','/foreign/source'),
    ('release','/foreign/release'),('source_runtime_count',438),('qa_changed',True)])
def test_wrong_intent_source_or_collection_blocks(fixture,field,value):
    intent = json.loads(fixture.intent.read_text());intent[field] = value;write(fixture.intent,intent)
    with pytest.raises(ValueError):web.configuration(fixture.workspace)


@pytest.mark.parametrize('change',['old_alias','missing_row','review','duplicate','foreign_raw','changed_sha'])
def test_import69_cannot_be_relabelled_or_aliased(fixture,change):
    prefix = json.loads(fixture.prefix.read_text())
    if change == 'old_alias':prefix['original_output'] = '/aliases/previous'
    elif change == 'missing_row':prefix['selected'].pop()
    elif change == 'review':prefix['selected'][0]['audit']['review_reasons'] = ['real_review']
    elif change == 'duplicate':prefix['selected'][1]['source_cell_id'] = prefix['selected'][0]['source_cell_id']
    elif change == 'foreign_raw':prefix['selected'][0]['capture_root'] = '/foreign/capture'
    else:prefix['manifest_sha256'] = '0'*64
    write(fixture.prefix,prefix)
    # Even consistently re-pinned invalid headers must fail semantic checks.
    intent = json.loads(fixture.intent.read_text());intent['import_receipt_sha256'] = web.sha(fixture.prefix)
    write(fixture.intent,intent)
    guard = json.loads((fixture.root/'population_recovery_contract.json').read_text())
    guard.update(intent_sha256=web.sha(fixture.intent),imported_prefix_sha256=web.sha(fixture.prefix))
    write(fixture.root/'population_recovery_contract.json',guard)
    with pytest.raises(ValueError):web.configuration(fixture.workspace)


def test_corrupt_bundle_source_fails_without_native_runtime_checks(fixture):
    fixture.physical.write_text('{}\n')
    with pytest.raises(ValueError):web.configuration(fixture.workspace)


def test_historical_bundle_mode_is_preserved_but_actual_bytes_still_pinned(fixture):
    fixture.physical.chmod(0o664)
    before = fixture.physical.stat()
    assert web.configuration(fixture.workspace)['manifest'] == Path(fixture.source)
    after = fixture.physical.stat()
    assert before.st_mode == after.st_mode and before.st_ino == after.st_ino
    assert before.st_mtime_ns == after.st_mtime_ns
    fixture.physical.write_text('{}\n')
    with pytest.raises(ValueError):web.configuration(fixture.workspace)


@pytest.mark.parametrize('which',['asset','prefix','intent','guard'])
def test_new_dashboard_and_collection_inputs_remain_private(fixture,which):
    filename = {'asset':fixture.work/'base5_capture_dashboard.html','prefix':fixture.prefix,
        'intent':fixture.intent,'guard':fixture.root/'population_recovery_contract.json'}[which]
    filename.chmod(0o664)
    with pytest.raises(ValueError):web.configuration(fixture.workspace)


def test_state_symlink_and_misplaced_helper_fail(fixture,monkeypatch):
    alias = fixture.state/'outside';alias.symlink_to(fixture.root)
    with pytest.raises(ValueError):web.configuration(fixture.workspace)
    alias.unlink()
    monkeypatch.setattr(web,'__file__',str(fixture.work/'base5_capture_dashboard.py'))
    with pytest.raises(ValueError):web.configuration(fixture.workspace)


def test_state_hardlink_cannot_bypass_readonly_capture_mount(fixture):
    os.link(fixture.prefix,fixture.state/'history.jsonl')
    with pytest.raises(ValueError):web.configuration(fixture.workspace)


def test_loader_keeps_frozen_live_identity_and_refuses_dangling_alias(tmp_path):
    original = Path(__file__).with_name('live.py')
    shutil.copyfile(original,tmp_path/'live.py');(tmp_path/'live.py').chmod(0o600)
    module = web.load_live(tmp_path)
    assert module.__file__ == str(tmp_path/'live.py')
    assert module.DECK_SHA == web.MANIFEST_SHA
    (tmp_path/'live_v2.py').symlink_to(tmp_path/'missing')
    with pytest.raises(ValueError):web.load_live(tmp_path)


@pytest.mark.parametrize('arguments',[['--port','8076'],['--host','0.0.0.0'],['--history-file','/foreign/log'],['--start-capture']])
def test_unqualified_listener_or_write_flags_are_rejected(fixture,arguments):
    with pytest.raises(SystemExit):call(fixture,arguments)


def test_actual_original_dashboard_is_loopback_only_and_keeps_child_root_guard():
    payload = (SOURCE/'base5_capture_dashboard.py').read_text()
    html = (SOURCE/'base5_capture_dashboard.html').read_text()
    assert "ThreadingHTTPServer(('127.0.0.1',a.port),Handler)" in payload
    assert "if root.resolve() in target.parents:" in payload
    assert "elif self.path=='/status.json':" in payload
    assert "fetch('/status.json'" in html
    assert all(web.sha(SOURCE/name) == digest for name,digest in web.ASSETS.items())
