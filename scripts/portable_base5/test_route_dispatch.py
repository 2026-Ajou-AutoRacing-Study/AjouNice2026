"""CPU-only route dispatcher propagation tests; never starts ROS or a child."""
import contextlib
import importlib
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import route_dispatch as dispatch


SCRIPTS = Path(__file__).resolve().parents[2]/'dataset/morai_od/scripts'


class RouteDispatchTests(unittest.TestCase):
    def fixture(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        workspace = Path(temporary.name)/'ajounice_base5_capture'
        checks = workspace/'checks'
        checks.mkdir(parents=True)
        session = workspace/'work/ros-session-v2'
        session.mkdir(parents=True)
        own = checks/'route_dispatch.py'
        helpers = [own, checks/'route_compat.py', checks/'live_route_compat.py']
        for path in helpers:
            path.write_text('# isolated CPU fixture\n')
        host, qualification = session/'host_contract.json', session/'qualification.json'
        host.write_text('{}\n')
        qualification.write_text('{}\n')
        release, manifest, witness = checks/'release.json', checks/'manifest.json', checks/'witness.json'
        for path in (release, manifest, witness):
            path.write_text('{}\n')
        original_sha = dispatch.compat.sha256
        fixed = {release:(original_sha(release),dispatch.RELEASE_SHA),
            manifest:(original_sha(manifest),dispatch.compat.CANONICAL_MANIFEST_SHA256)}
        def sha(path):
            path = Path(path)
            actual = original_sha(path)
            return fixed[path][1] if path in fixed and actual == fixed[path][0] else actual
        doc = dict(schema=dispatch.SCHEMA, workspace=str(workspace), scripts=str(SCRIPTS),
            output=str(workspace/'collections/sunny13_5k'), environment='sunny13',
            source_runtime_count=427, new_live_smoke_executed=False,
            release=str(release), manifest=str(manifest), witness=str(witness),
            witness_sha256=sha(witness), host_contract_sha256=sha(host),
            qualification_sha256=sha(qualification),
            files={str(path):sha(path) for path in [*helpers,host,qualification,witness]})
        intent = checks/'route_compat_intent.json'
        intent.write_text(json.dumps(doc))
        launcher = SimpleNamespace(preflight=Mock(return_value=(
            dict(runtime_hashes={str(index):'fixture' for index in range(427)}),
            dict(manifest=str(manifest)))))
        generator, extent = SimpleNamespace(), SimpleNamespace()
        modules = {'launch_base5_obstacle_support':launcher,
            'generate_base5_independent':generator,'base5_static_mission_extent':extent}
        return SimpleNamespace(workspace=workspace, checks=checks, own=own, helpers=helpers,
            host=host, qualification=qualification, release=release, manifest=manifest,
            witness=witness, doc=doc, intent=intent, sha=sha, launcher=launcher,
            generator=generator, extent=extent, modules=modules)

    @contextlib.contextmanager
    def intent_mocks(self, fixture):
        with patch.object(dispatch,'__file__',str(fixture.own)), \
                patch.object(dispatch.compat,'sha256',fixture.sha), \
                patch.object(dispatch.importlib,'import_module',side_effect=fixture.modules.__getitem__), \
                patch.object(dispatch.compat,'verified_witness') as witness, \
                patch.object(sys,'path',list(sys.path)):
            yield witness

    def validate_fixture(self, fixture):
        fixture.intent.write_text(json.dumps(fixture.doc))
        with self.intent_mocks(fixture):
            return dispatch.validate_intent(fixture.intent)

    def test_exact_role_to_original_entrypoint_map(self):
        scripts=SCRIPTS.resolve()
        targets=dispatch.target_paths(scripts)
        self.assertEqual(set(targets),{'launcher','capture-optimized','campaign-spatial','capture'})
        expected={'launcher':'launch_base5_obstacle_support.py',
            'capture-optimized':'run_base5_capture_optimized.py',
            'campaign-spatial':'run_base5_capture_campaign_spatial.py',
            'capture':'run_base5_capture.py'}
        self.assertEqual({k:Path(v) for k,v in targets.items()},
            {k:scripts/v for k,v in expected.items()})

    def test_real_core_imports_share_one_generator_context_module(self):
        with patch.object(sys,'path',[str(SCRIPTS),*sys.path]):
            generator=importlib.import_module('generate_base5_independent')
            quality=importlib.import_module('run_base5_quality_loop')
            spatial=importlib.import_module('base5_spatial_diversity')
            probe=importlib.import_module('prepare_base5_empty_ground_probe')
            fresh=importlib.import_module('run_base5_fresh_batch')
            batch=importlib.import_module('run_base5_batch_campaign')
            campaign=importlib.import_module('run_base5_capture_campaign')
        self.assertIs(quality.gen,generator)
        self.assertIs(spatial.gen,generator)
        self.assertIs(probe.gen,generator)
        self.assertIs(fresh.optimized.batch,batch)
        self.assertIs(batch.campaign,campaign)
        self.assertIs(campaign.loop,quality)

    def test_valid_intent_checks_original_preflight_and_witness(self):
        f = self.fixture()
        with self.intent_mocks(f) as witness:
            self.assertEqual(dispatch.validate_intent(f.intent),f.doc)
            f.launcher.preflight.assert_called_once_with(f.release,'sunny13')
            witness.assert_called_once_with(str(f.witness),generator=f.generator,
                extent=f.extent,manifest=str(f.manifest),expected_witness_sha256=f.doc['witness_sha256'])

    def test_intent_profile_corruption_rejected(self):
        for key,value in [('schema','foreign'),('environment','foggy13'),
                ('source_runtime_count',426),('new_live_smoke_executed',True),
                ('scripts','/tmp/foreign'),('output','/tmp/foreign')]:
            with self.subTest(key=key):
                f = self.fixture()
                f.doc[key] = value
                with self.assertRaises(ValueError):
                    self.validate_fixture(f)

    def test_intent_scope_cannot_be_moved(self):
        f = self.fixture()
        foreign = f.checks/'another_intent.json'
        foreign.write_text(json.dumps(f.doc))
        with self.intent_mocks(f),self.assertRaisesRegex(ValueError,'scope'):
            dispatch.validate_intent(foreign)

    def test_missing_dispatcher_or_compatibility_helper_pin_rejected(self):
        for index in range(3):
            with self.subTest(index=index):
                f = self.fixture()
                del f.doc['files'][str(f.helpers[index])]
                with self.assertRaisesRegex(ValueError,'pins missing'):
                    self.validate_fixture(f)

    def test_changed_helper_bytes_rejected(self):
        f = self.fixture()
        f.helpers[1].write_text('# modified\n')
        with self.assertRaisesRegex(ValueError,'changed'):
            self.validate_fixture(f)

    def test_pin_outside_approved_evidence_rejected(self):
        f = self.fixture()
        foreign = f.workspace/'foreign.py'
        foreign.write_text('# foreign\n')
        f.doc['files'][str(foreign)] = f.sha(foreign)
        with self.assertRaisesRegex(ValueError,'outside'):
            self.validate_fixture(f)

    def test_symlinked_helper_rejected(self):
        f = self.fixture()
        helper = f.helpers[1]
        real = f.checks/'real_compat.py'
        helper.rename(real)
        helper.symlink_to(real)
        with self.assertRaises(dispatch.compat.RouteCompatibilityError):
            self.validate_fixture(f)

    def test_changed_original_host_or_qualification_pin_rejected(self):
        for name in ('host_contract_sha256','qualification_sha256'):
            with self.subTest(name=name):
                f = self.fixture()
                f.doc[name] = '0'*64
                with self.assertRaisesRegex(ValueError,'host qualification'):
                    self.validate_fixture(f)

    def test_changed_original_release_or_manifest_bytes_rejected(self):
        for name in ('release','manifest'):
            with self.subTest(name=name):
                f = self.fixture()
                getattr(f,name).write_text('{"changed":true}\n')
                with self.assertRaisesRegex(ValueError,'release/source'):
                    self.validate_fixture(f)

    def test_original_runtime_count_and_source_preflight_rechecked(self):
        for runtime_count,manifest in [(426,None),(427,'/tmp/foreign.json')]:
            with self.subTest(runtime_count=runtime_count,manifest=manifest):
                f = self.fixture()
                f.launcher.preflight.return_value=(dict(runtime_hashes=list(range(runtime_count))),
                    dict(manifest=manifest or str(f.manifest)))
                with self.assertRaisesRegex(ValueError,'runtime/source'):
                    self.validate_fixture(f)

    def test_witness_failure_does_not_get_treated_as_an_accepted_intent(self):
        f = self.fixture()
        with self.intent_mocks(f) as witness:
            witness.side_effect = dispatch.compat.RouteCompatibilityError('witness changed')
            with self.assertRaisesRegex(dispatch.compat.RouteCompatibilityError,'witness changed'):
                dispatch.validate_intent(f.intent)

    def test_exact_three_collectors_routed_without_kwargs_or_argument_mutation(self):
        intent = Path('/tmp/fixture_intent.json')
        for role,path in dispatch.target_paths(SCRIPTS).items():
            if role == 'launcher':
                continue
            with self.subTest(role=role),patch.object(dispatch.subprocess,'Popen') as popen:
                command = [sys.executable,str(path),'--manifest','source.json','--run']
                unchanged = list(command)
                kwargs = dict(start_new_session=True,stdout=17,stderr=19,cwd='/tmp')
                with dispatch.routed_popen(intent,{},SCRIPTS):
                    result = dispatch.subprocess.Popen(command,**kwargs)
                popen.assert_called_once_with(dispatch.dispatcher_command(intent,role,command[2:]),**kwargs)
                self.assertIs(result,popen.return_value)
                self.assertEqual(command,unchanged)

    def test_python_no_bytecode_argument_preserved_as_one_router_flag(self):
        path = dispatch.target_paths(SCRIPTS)['capture']
        with patch.object(dispatch.subprocess,'Popen') as popen:
            with dispatch.routed_popen('/tmp/intent.json',{},SCRIPTS):
                dispatch.subprocess.Popen([sys.executable,'-B',str(path),'--run'])
            popen.assert_called_once_with(dispatch.dispatcher_command('/tmp/intent.json','capture',['--run']))

    def test_unknown_python_option_cannot_silently_bypass_child_routing(self):
        path = dispatch.target_paths(SCRIPTS)['capture']
        for command in [[sys.executable,'-u',str(path),'--run'],
                [sys.executable,'-B','-u',str(path),'--run'],
                ['env','python3',str(path),'--run']]:
            with self.subTest(command=command),patch.object(dispatch.subprocess,'Popen') as popen:
                with dispatch.routed_popen('/tmp/intent.json',{},SCRIPTS):
                    with self.assertRaisesRegex(ValueError,'argument form'):
                        dispatch.subprocess.Popen(command)
                popen.assert_not_called()

    def test_same_basename_foreign_script_or_python_rejected(self):
        path = dispatch.target_paths(SCRIPTS)['capture']
        for command in [[sys.executable,'/tmp/'+path.name],['/foreign/python',str(path)]]:
            with self.subTest(command=command),patch.object(dispatch.subprocess,'Popen') as popen:
                with dispatch.routed_popen('/tmp/intent.json',{},SCRIPTS):
                    with self.assertRaisesRegex(ValueError,'Foreign'):
                        dispatch.subprocess.Popen(command)
                popen.assert_not_called()

    def test_shell_form_or_shell_true_collectors_rejected(self):
        path = dispatch.target_paths(SCRIPTS)['capture']
        for command,kwargs in [('python3 '+str(path),{}),
                [('python3 '+str(path)).encode(),{}],([sys.executable,str(path)],dict(shell=True))]:
            with self.subTest(command=command),patch.object(dispatch.subprocess,'Popen') as popen:
                with dispatch.routed_popen('/tmp/intent.json',{},SCRIPTS):
                    with self.assertRaises(ValueError):
                        dispatch.subprocess.Popen(command,**kwargs)
                popen.assert_not_called()

    def test_unrelated_native_commands_are_untouched(self):
        commands = [['roslaunch','package','node.launch'],['/bin/true'],
            'rosnode list',[sys.executable,str(SCRIPTS/'unrelated.py')]]
        for command in commands:
            with self.subTest(command=command),patch.object(dispatch.subprocess,'Popen') as popen:
                with dispatch.routed_popen('/tmp/intent.json',{},SCRIPTS):
                    dispatch.subprocess.Popen(command,env={'FIXTURE':'1'})
                popen.assert_called_once_with(command,env={'FIXTURE':'1'})

    def test_router_restored_when_original_raises(self):
        with patch.object(dispatch.subprocess,'Popen',side_effect=OSError('fixture')) as popen:
            with self.assertRaises(OSError):
                with dispatch.routed_popen('/tmp/intent.json',{},SCRIPTS):
                    dispatch.subprocess.Popen(['/bin/true'])
            self.assertIs(dispatch.subprocess.Popen,popen)

    def test_router_ownership_change_fails_and_restores_original(self):
        with patch.object(dispatch.subprocess,'Popen') as popen:
            with self.assertRaisesRegex(ValueError,'ownership'):
                with dispatch.routed_popen('/tmp/intent.json',{},SCRIPTS):
                    dispatch.subprocess.Popen = Mock()
            self.assertIs(dispatch.subprocess.Popen,popen)

    def test_explicit_completion_denied_even_on_completed_resume(self):
        f = self.fixture()
        for role in [*dispatch.target_paths(SCRIPTS),'finish-support']:
            for complete in ('--complete','--complete=true'):
                with self.subTest(role=role,complete=complete),self.assertRaisesRegex(ValueError,'completion'):
                    dispatch.validate_arguments(role,['--resume',complete],f.doc)

    def test_launcher_requires_exact_root_release_environment_and_ui_confirmation(self):
        f = self.fixture()
        base = ['--output',f.doc['output'],'--release',f.doc['release'],'--environment','sunny13']
        dispatch.validate_arguments('launcher',base,f.doc)
        dispatch.validate_arguments('launcher',base+['--run','--confirmed-environment','sunny13'],f.doc)
        for change in [('output',str(f.workspace/'elsewhere')),('release','/tmp/other.json'),
                ('environment','sunny15')]:
            with self.subTest(change=change):
                arguments = list(base)
                arguments[arguments.index('--'+change[0])+1] = change[1]
                with self.assertRaises(ValueError):
                    dispatch.validate_arguments('launcher',arguments,f.doc)
        with self.assertRaises(ValueError):
            dispatch.validate_arguments('launcher',base+['--run'],f.doc)
        with self.assertRaises(ValueError):
            dispatch.validate_arguments('launcher',base+['--run','--confirmed-environment','sunny15'],f.doc)

    def test_leaf_requires_pinned_root_descendant_and_explicit_source(self):
        f = self.fixture()
        root = Path(f.doc['output'])
        for role in ('capture-optimized','campaign-spatial','capture'):
            with self.subTest(role=role):
                arguments = ['--output',str(root/'chunk_0'),'--manifest',str(f.manifest)]
                dispatch.validate_arguments(role,arguments,f.doc)
                for output in (root,f.workspace/'foreign',root.parent/'sunny13_5k_suffix/chunk_0'):
                    with self.assertRaises(ValueError):
                        dispatch.validate_arguments(role,['--output',str(output),'--manifest',str(f.manifest)],f.doc)
                with self.assertRaises(ValueError):
                    dispatch.validate_arguments(role,['--output',str(root/'chunk_0'),'--manifest','/tmp/foreign.json'],f.doc)

    def test_missing_or_duplicate_cli_paths_rejected(self):
        f = self.fixture()
        valid = ['--output',str(Path(f.doc['output'])/'chunk_0'),'--manifest',str(f.manifest)]
        for invalid in [[],valid+['--output','/tmp/second'],valid[:2],['--output']]:
            with self.subTest(arguments=invalid),self.assertRaises(ValueError):
                dispatch.validate_arguments('capture',invalid,f.doc)

    def test_lexical_parent_escape_and_output_symlink_rejected(self):
        f = self.fixture()
        root = Path(f.doc['output'])
        root.mkdir(parents=True)
        (root/'foreign').symlink_to(f.checks,target_is_directory=True)
        for output in [str(root/'../outside'),str(root/'foreign/capture')]:
            with self.subTest(output=output),self.assertRaises(dispatch.compat.RouteCompatibilityError):
                dispatch.validate_arguments('capture',['--output',output,'--manifest',str(f.manifest)],f.doc)

    def test_collection_guard_created_only_by_fresh_launcher_and_pinned_on_resume(self):
        f = self.fixture()
        with self.assertRaisesRegex(ValueError,'retrofit'):
            dispatch.collection_guard(f.doc,f.intent)
        guard = dispatch.collection_guard(f.doc,f.intent,create=True)
        before = guard.read_bytes()
        self.assertEqual(dispatch.collection_guard(f.doc,f.intent),guard)
        self.assertEqual(guard.read_bytes(),before)
        for key in ('original_native_qa','original_bounded_recovery','original_shutdown_safety'):
            self.assertIs(json.loads(before)[key],True)
        f.intent.write_text(f.intent.read_text()+'\n')
        with self.assertRaisesRegex(ValueError,'contract changed'):
            dispatch.collection_guard(f.doc,f.intent)

    def test_collection_guard_cannot_retrofit_existing_batch_or_status(self):
        for existing in ('batch_contract.json','status.json'):
            with self.subTest(existing=existing):
                f = self.fixture()
                root = Path(f.doc['output'])
                root.mkdir(parents=True)
                (root/existing).write_text('{}\n')
                with self.assertRaisesRegex(ValueError,'retrofit'):
                    dispatch.collection_guard(f.doc,f.intent,create=True)
                self.assertFalse((root/'route_compat_contract.json').exists())

    def test_completed_guard_tampering_is_not_bypassed(self):
        f = self.fixture()
        guard = dispatch.collection_guard(f.doc,f.intent,create=True)
        data = json.loads(guard.read_text())
        data['original_native_qa'] = False
        guard.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError,'contract changed'):
            dispatch.collection_guard(f.doc,f.intent)

    def test_main_passes_original_arguments_after_separator(self):
        args = ['--release','original.json','--run','--resume','--environment','sunny13']
        with patch.object(dispatch,'run_role',return_value=17) as run:
            self.assertEqual(dispatch.main(['--intent','/tmp/intent.json','--role','launcher','--',*args]),17)
        run.assert_called_once_with(Path('/tmp/intent.json'),'launcher',args)

    def test_cpu_finish_invocation_receipt_does_not_mutate_frozen_collection(self):
        f = self.fixture()
        root = Path(f.doc['output'])
        root.mkdir(parents=True)
        frozen = root/'frozen.txt'
        frozen.write_text('preserved\n')
        before = {str(path.relative_to(root)):path.read_bytes() for path in root.rglob('*') if path.is_file()}
        dispatch.record_entry(f.doc,f.intent,'finish-support',['finish','--run'])
        dispatch.record_entry(f.doc,f.intent,'finish-support',['finish','--run'])
        after = {str(path.relative_to(root)):path.read_bytes() for path in root.rglob('*') if path.is_file()}
        self.assertEqual(before,after)
        receipts = list((f.workspace/'work/route-compat-finish-invocations').glob('*.json'))
        self.assertEqual(len(receipts),2)
        for receipt in receipts:
            self.assertEqual(json.loads(receipt.read_text())['role'],'finish-support')

    def test_actual_runpy_preserves_original_file_and_argv_without_any_process(self):
        f = self.fixture()
        target = f.checks/'isolated_original.py'
        target.write_text('import builtins, sys\n'
            'builtins._base5_route_test_proof.append((__file__, list(sys.argv)))\n')
        generator = SimpleNamespace(context=Mock(),geo=SimpleNamespace(
            DEFAULT_RUNTIME_ASSET_DIR='runtime',DEFAULT_MGEO='mgeo'))
        import builtins
        proof = []
        original_argv = sys.argv
        with patch.object(builtins,'_base5_route_test_proof',proof,create=True), \
                patch.object(dispatch,'validate_intent',return_value=f.doc), \
                patch.object(dispatch,'validate_arguments'),patch.object(dispatch,'collection_guard'), \
                patch.object(dispatch.importlib,'import_module',side_effect={
                    'generate_base5_independent':generator,'base5_static_mission_extent':object()}.__getitem__), \
                patch.object(dispatch.compat,'installed',return_value=contextlib.nullcontext()), \
                patch.object(dispatch,'record_entry'), \
                patch.object(dispatch,'target_paths',return_value={'capture':target}), \
                patch.object(dispatch.subprocess,'Popen',side_effect=AssertionError('No process is allowed')):
            self.assertEqual(dispatch.run_role(f.intent,'capture',['--fixture','original']),0)
        self.assertEqual(proof,[(str(target),[str(target),'--fixture','original'])])
        self.assertIs(sys.argv,original_argv)

    def test_run_role_rechecks_intent_guard_context_before_original_runpy(self):
        f = self.fixture()
        seen = []
        generator = SimpleNamespace(context=Mock(side_effect=lambda *args:seen.append('context')),
            geo=SimpleNamespace(DEFAULT_RUNTIME_ASSET_DIR='runtime',DEFAULT_MGEO='mgeo'))
        extent = SimpleNamespace()
        @contextlib.contextmanager
        def installed(*args,**kwargs):
            seen.append('installed')
            try:
                yield
            finally:
                seen.append('uninstalled')
        original_argv = sys.argv
        target = dispatch.target_paths(SCRIPTS)['capture']
        def run(path,run_name):
            self.assertEqual(path,str(target))
            self.assertEqual(run_name,'__main__')
            self.assertEqual(sys.argv,[str(target),'--original-flag'])
            seen.append('runpy')
        with patch.object(dispatch,'validate_intent',side_effect=lambda path:seen.append('intent') or f.doc), \
                patch.object(dispatch,'validate_arguments',side_effect=lambda *args:seen.append('args')), \
                patch.object(dispatch,'collection_guard',side_effect=lambda *args,**kwargs:seen.append('guard')), \
                patch.object(dispatch.importlib,'import_module',side_effect={
                    'generate_base5_independent':generator,'base5_static_mission_extent':extent}.__getitem__), \
                patch.object(dispatch.compat,'installed',installed), \
                patch.object(dispatch,'record_entry',side_effect=lambda *args:seen.append('record')), \
                patch.object(dispatch.runpy,'run_path',side_effect=run), \
                patch.object(dispatch.subprocess,'Popen') as popen:
            self.assertEqual(dispatch.run_role(f.intent,'capture',['--original-flag']),0)
            self.assertIs(dispatch.subprocess.Popen,popen)
        self.assertIs(sys.argv,original_argv)
        self.assertEqual(seen,['intent','args','guard','installed','context','record','runpy','uninstalled'])

    def test_run_role_exit_codes_and_failure_restore_argv_and_router(self):
        f = self.fixture()
        generator = SimpleNamespace(context=Mock(),geo=SimpleNamespace(
            DEFAULT_RUNTIME_ASSET_DIR='runtime',DEFAULT_MGEO='mgeo'))
        original_argv = sys.argv
        for error,expected in [(SystemExit(7),7),(SystemExit(None),0),(SystemExit('failure'),1),
                (RuntimeError('fixture'),None)]:
            with self.subTest(error=error), \
                    patch.object(dispatch,'validate_intent',return_value=f.doc), \
                    patch.object(dispatch,'validate_arguments'),patch.object(dispatch,'collection_guard'), \
                    patch.object(dispatch.importlib,'import_module',side_effect={
                        'generate_base5_independent':generator,'base5_static_mission_extent':object()}.__getitem__), \
                    patch.object(dispatch.compat,'installed',return_value=contextlib.nullcontext()), \
                    patch.object(dispatch,'record_entry'), \
                    patch.object(dispatch.runpy,'run_path',side_effect=error), \
                    patch.object(dispatch.subprocess,'Popen') as popen:
                if expected is None:
                    with self.assertRaises(RuntimeError):
                        dispatch.run_role(f.intent,'capture',[])
                else:
                    self.assertEqual(dispatch.run_role(f.intent,'capture',[]),expected)
                self.assertIs(sys.argv,original_argv)
                self.assertIs(dispatch.subprocess.Popen,popen)


if __name__=='__main__':
    unittest.main()
