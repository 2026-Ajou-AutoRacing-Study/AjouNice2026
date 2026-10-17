"""CPU-only crash launcher fixtures; no live ROS, MORAI, or collection process."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import json
import os

import pytest

import local_sunny15_after_crash as launcher


def test_default_plan_does_not_verify_write_or_capture(monkeypatch,capsys):
    frozen = Mock(side_effect=AssertionError('No expensive context or ROS in plan'))
    monkeypatch.setattr(launcher.original,'frozen_context',frozen)
    assert launcher.main([]) == 0
    value = json.loads(capsys.readouterr().out)
    assert value['captures_requested'] == 0 and value['files_written'] is False
    frozen.assert_not_called()


@pytest.mark.parametrize('values',[
    ['--run'], ['--confirmed-environment','sunny15'],
    ['--prepare','--confirmed-environment','sunny15'],
    ['--run','--confirmed-environment','sunny15','--parent-pid','1'],
])
def test_ambiguous_authority_rejected_before_collection(values):
    with pytest.raises(ValueError):launcher.main(values)


def test_destination_adapter_checks_original_release_for_original_destination(monkeypatch,tmp_path):
    new = tmp_path/'new';old = tmp_path/'old'
    monkeypatch.setattr(launcher,'OUTPUT',new)
    monkeypatch.setattr(launcher.original,'OUTPUT',old)
    source,release = launcher.original.MANIFEST,launcher.original.RELEASE
    expected = dict(original_authority=True)
    validate = Mock(return_value=expected)
    parent = SimpleNamespace(qualified_release=validate)
    with launcher.qualified_destination(parent):
        assert parent.qualified_release(release,'sunny15',source,new,{'p':'hash'}) is expected
    validate.assert_called_once_with(release,'sunny15',source,old,{'p':'hash'})
    assert parent.qualified_release is validate


@pytest.mark.parametrize('field',['release','manifest','output','environment'])
def test_destination_adapter_cannot_authorize_arbitrary_paths(monkeypatch,tmp_path,field):
    new = tmp_path/'new'
    monkeypatch.setattr(launcher,'OUTPUT',new)
    validate = Mock()
    parent = SimpleNamespace(qualified_release=validate)
    args = dict(release=launcher.original.RELEASE,environment='sunny15',
        manifest=launcher.original.MANIFEST,output=new,runtime={})
    args[field] = 'sunny13' if field == 'environment' else tmp_path/'unapproved'
    with launcher.qualified_destination(parent):
        with pytest.raises(ValueError):parent.qualified_release(**args)
    validate.assert_not_called()
    assert parent.qualified_release is validate


def test_destination_adapter_restored_when_original_verifier_fails(monkeypatch,tmp_path):
    new = tmp_path/'new';monkeypatch.setattr(launcher,'OUTPUT',new)
    validate = Mock(side_effect=ValueError('Original native proof failed'))
    parent = SimpleNamespace(qualified_release=validate)
    with pytest.raises(ValueError,match='native proof'):
        with launcher.qualified_destination(parent):
            parent.qualified_release(launcher.original.RELEASE,'sunny15',launcher.original.MANIFEST,new,{})
    assert parent.qualified_release is validate


@pytest.fixture
def source(monkeypatch,tmp_path):
    new,manifest = tmp_path/'new',tmp_path/'manifest.json'
    manifest.write_text(json.dumps(dict(scenes=[dict(id='s%d'%i) for i in range(5000)])))
    monkeypatch.setattr(launcher,'OUTPUT',new)
    monkeypatch.setattr(launcher.original,'MANIFEST',manifest)
    return new,manifest


def child_values(new,manifest):
    return ['--manifest',str(manifest),'--indices','417','--output',
        str(new/'chunk_0026/recovery/s417'),'--enable-final-direction-reference','--resume']


def test_exact_bounded_recovery_arguments_accepted(source):
    new,manifest = source
    result = launcher.child_arguments(child_values(new,manifest))
    assert result.indices == '417'
    assert result.output == str(new/'chunk_0026/recovery/s417')


@pytest.mark.parametrize('change',['escape','wrong_chunk','wrong_id','index','duplicate','missing_flag','seed','manifest','unknown'])
def test_recovery_cannot_escape_source_slot_or_strict_qa(source,change):
    new,manifest = source;values = child_values(new,manifest)
    if change == 'escape':values[5] = str(new.parent/'old/recovery/s417')
    elif change == 'wrong_chunk':values[5] = str(new/'chunk_0027/recovery/s417')
    elif change == 'wrong_id':values[5] = str(new/'chunk_0026/recovery/s418')
    elif change == 'index':values[3] = '0417'
    elif change == 'duplicate':values += ['--indices','417']
    elif change == 'missing_flag':values.remove('--enable-final-direction-reference')
    elif change == 'seed':values += ['--seed-capture',str(new/'elsewhere_seed.json')]
    elif change == 'manifest':values[1] = str(new/'other_manifest.json')
    elif change == 'unknown':values += ['--disable-qa']
    with pytest.raises(ValueError):launcher.child_arguments(values)


def test_parent_identity_requires_exact_run_command(monkeypatch,tmp_path):
    uid,pid = os.getuid(),7788
    argv = ['/usr/bin/python3','-B','-u',str(Path(launcher.__file__).resolve()),
        '--run','--confirmed-environment','sunny15']
    record = dict(uid=uid,ticks='123',argv=argv,cwd=tmp_path)
    monkeypatch.setattr(launcher.os,'getppid',lambda:pid)
    monkeypatch.setattr(launcher.fast,'parent_record',lambda value:record)
    launcher.verify_parent(pid,'123')
    record['argv'] = argv+['--prepare']
    with pytest.raises(ValueError):launcher.verify_parent(pid,'123')
    record['argv'] = argv
    with pytest.raises(ValueError):launcher.verify_parent(pid,'different-start')
    with pytest.raises(ValueError):launcher.verify_parent(pid+1,'123')


@pytest.mark.parametrize('stage',['halted','running','halted_exit_safety','complete','stopped'])
def test_new_halt_cannot_be_relabelled_as_pause(monkeypatch,tmp_path,stage):
    root = tmp_path/'new';root.mkdir()
    (root/'status.json').write_text(json.dumps(dict(stage=stage)))
    before = (root/'status.json').read_bytes()
    monkeypatch.setattr(launcher,'OUTPUT',root)
    with pytest.raises(ValueError,match='never bypass another halt'):launcher.release_pause_if_present()
    assert (root/'status.json').read_bytes() == before


@pytest.mark.parametrize('marker',['STOP','PAUSE','stop_link','pause_link'])
def test_prepared_root_stop_markers_including_dangling_links_block(monkeypatch,tmp_path,marker):
    root = tmp_path/'new';root.mkdir()
    (root/'status.json').write_text(json.dumps(dict(stage='prepared_waiting_for_resume')))
    name = 'STOP' if marker.startswith('stop') or marker == 'STOP' else 'PAUSE'
    if marker.endswith('_link'):(root/name).symlink_to(root/'missing')
    else:(root/name).write_text('operator stop')
    monkeypatch.setattr(launcher,'OUTPUT',root)
    with pytest.raises(ValueError,match='stop/pause marker'):launcher.release_pause_if_present()


def test_router_retains_bounded_arguments_and_real_parent(source,monkeypatch):
    new,manifest = source
    target = Mock(return_value='fake-child')
    monkeypatch.setattr(launcher.subprocess,'Popen',target)
    monkeypatch.setattr(launcher.original,'parent_identity',lambda:(42,'startticks'))
    monkeypatch.setattr(launcher.fast,'sha',lambda filename:'a'*64)
    with launcher.routed_children():
        args = child_values(new,manifest)
        assert launcher.subprocess.Popen([launcher.sys.executable,str(launcher.original.RECOVERY),*args]) == 'fake-child'
    argv = target.call_args.args[0]
    assert argv[:3] == [launcher.sys.executable,'-B',str(Path(launcher.__file__).absolute())]
    assert argv[argv.index('--parent-pid')+1] == '42'
    assert argv[argv.index('--')+1:] == args
    assert launcher.subprocess.Popen is target


def test_native_commands_unchanged_but_shell_recovery_refused(source,monkeypatch):
    target = Mock(return_value='not-a-real-process')
    monkeypatch.setattr(launcher.subprocess,'Popen',target)
    monkeypatch.setattr(launcher.original,'parent_identity',lambda:(42,'ticks'))
    with launcher.routed_children():
        command = ['python3','frozen_regular_capture.py','--same-args']
        assert launcher.subprocess.Popen(command) == 'not-a-real-process'
        with pytest.raises(ValueError,match='Shell recovery'):
            launcher.subprocess.Popen('python3 '+launcher.original.RECOVERY.name,shell=True)
    target.assert_called_once_with(command)
