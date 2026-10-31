"""Borrowed-host completion wrapper: CPU-only, no SSH/ROS/MORAI operations."""
from contextlib import nullcontext
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import complete_borrowed as wrapper
import complete_unified as exact_module
import unified_borrowed as borrowed_module


def write(path, value):
    path.parent.mkdir(mode=0o700,parents=True,exist_ok=True)
    path.write_text(value if isinstance(value,str) else json.dumps(value)+'\n')
    path.chmod(0o600)
    return path


class BorrowedCompletionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.home = Path(self.tmp.name)/'home'
        self.workspace = self.home/'ajounice_base5_capture'
        self.workspace.mkdir(mode=0o700,parents=True)
        self.root = self.workspace/wrapper.RUNTIME_REL
        self.profile = write(self.workspace/wrapper.COMMON_REL/'crash-continuation-v1/sunny13_profile.json',
            dict(environment='sunny13',output=str(self.workspace/'collections/main'),manifest='/logical/manifest',release='/logical/release'))
        for name in wrapper.FILES:
            write(self.root/name, '# immutable '+name+'\n')
        self.baseline = self.workspace/'collections/main'
        self.baseline.mkdir(mode=0o700,parents=True)
        self.seed = self.workspace/'collections'/wrapper.SEED_NAME
        self.output = self.workspace/'collections'/wrapper.OUTPUT_NAME
        self.doc = dict(schema=wrapper.SCHEMA,workspace=str(self.workspace),environment='sunny13',
            source_profile=str(self.profile),source_profile_sha256=borrowed_module.sha(self.profile),
            baseline=str(self.baseline),seed=str(self.seed),output=str(self.output),
            helper_hashes={str(self.root/name):borrowed_module.sha(self.root/name) for name in wrapper.FILES})
        write(self.root/'deployment.json',self.doc)
        self.exact = SimpleNamespace(common=SimpleNamespace(physical=exact_module.common.physical,
            read=exact_module.read,validate_profile=lambda value:value),__file__=str(self.root/'complete_unified.py'))
        self.file_patch = patch.object(wrapper,'__file__',str(self.root/'complete_borrowed.py'))
        self.file_patch.start();self.addCleanup(self.file_patch.stop)
        self.home_patch = patch.object(wrapper.pwd,'getpwuid',return_value=SimpleNamespace(pw_dir=str(self.home)))
        self.home_patch.start();self.addCleanup(self.home_patch.stop)

    def scope(self):
        return wrapper.deployment(self.root,borrowed_module,self.exact)

    def test_scope_preserves_original_common_and_separate_outputs(self):
        s=self.scope()
        self.assertEqual(s['profile'],self.profile)
        self.assertEqual(s['baseline'],self.baseline)
        self.assertEqual(s['seed'],self.seed)
        self.assertEqual(s['output'],self.output)

    def test_session_home_is_not_account_scope(self):
        with patch.dict(wrapper.os.environ,HOME='/isolated/ros-session/home'):
            self.assertEqual(self.scope()['workspace'],self.workspace)

    def test_deployment_cannot_escape_account_workspace(self):
        with patch.object(wrapper.pwd,'getpwuid',return_value=SimpleNamespace(pw_dir='/different/account')):
            with self.assertRaisesRegex(ValueError,'approved borrowed-host'):
                self.scope()

    def test_extra_helper_inventory_is_rejected_without_touching_original(self):
        write(self.root/'extra.py','# unqualified\n')
        before=self.profile.read_bytes()
        with self.assertRaisesRegex(ValueError,'inventory'):
            self.scope()
        self.assertEqual(self.profile.read_bytes(),before)

    def test_changed_helper_sha_is_rejected(self):
        write(self.root/'flexible_complete.py','# changed\n')
        with self.assertRaisesRegex(ValueError,'SHA'):
            self.scope()

    def test_source_profile_sha_change_is_rejected(self):
        write(self.profile,dict(environment='sunny15',output=str(self.baseline)))
        with self.assertRaisesRegex(ValueError,'scope or changed'):
            self.scope()

    def test_output_cannot_overlap_original_or_escape_collections(self):
        for key,value in [('seed',str(self.baseline)),('output',str(self.baseline)),
                          ('output',str(self.workspace/'elsewhere'/wrapper.OUTPUT_NAME))]:
            with self.subTest(key=key,value=value):
                doc=dict(self.doc);doc[key]=value;write(self.root/'deployment.json',doc)
                with self.assertRaisesRegex(ValueError,'Separate fixed'):
                    self.scope()
        write(self.root/'deployment.json',self.doc)

    def test_namespace_preserves_old_function_and_read_only_main_not_seed(self):
        scope=self.scope()
        original=Mock(return_value=['bwrap','--ro-bind','/','/','--chdir','/logical','--','app'])
        borrowed=SimpleNamespace(namespace=original)
        with wrapper.preserved_namespace(borrowed,scope):
            command=borrowed.namespace({},['app'])
            mounts=command[:command.index('--chdir')]
            for path in (scope['root'],scope['profile'],scope['baseline']):
                self.assertIn(str(path),mounts)
            self.assertNotIn(str(scope['seed']),mounts)
        self.assertIs(borrowed.namespace,original)

    def test_namespace_restores_after_exception(self):
        borrowed=SimpleNamespace(namespace=Mock(return_value=['bwrap','--chdir','/','--']))
        original=borrowed.namespace
        with self.assertRaisesRegex(RuntimeError,'candidate'):
            with wrapper.preserved_namespace(borrowed,self.scope()):
                raise RuntimeError('candidate failed')
        self.assertIs(borrowed.namespace,original)

    def test_capture_requires_exact_environment_confirmation(self):
        for argv in (['run'],['prepare','--confirmed-environment','sunny13'],['status','--confirmed-environment','sunny13']):
            with self.subTest(argv=argv),self.assertRaisesRegex(ValueError,'environment confirmation'):
                wrapper.main(argv)

    def test_prepare_cpu_bound_and_actual_run_flags_are_distinct(self):
        scope=self.scope()
        import flexible_complete as flexible
        args=wrapper.parser().parse_args(['prepare','--stop-after-cells','9'])
        prepared=wrapper.flexible_args(flexible,args,scope)
        self.assertTrue(prepared.prepare);self.assertFalse(prepared.run)
        self.assertEqual(prepared.stop_after_cells,1)
        args=wrapper.parser().parse_args(['run','--confirmed-environment','sunny13','--stop-after-cells','2'])
        run=wrapper.flexible_args(flexible,args,scope)
        self.assertTrue(run.run);self.assertEqual(run.stop_after_cells,2)
        self.assertEqual(run.confirmed_environment,'sunny13')
        self.assertEqual(run.attempts,3)


class CpuSeedTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        self.profile=write(self.root/'profile.json',dict(qualified=True))
        self.baseline=self.root/'baseline';self.baseline.mkdir()
        self.seed=self.root/'seed'
        self.raw=write(self.baseline/'preserved_native.json',dict(never_rewrite=True))
        write(self.baseline/'status.json',dict(elapsed_sec=100))
        self.selected=[dict(id='a',source_cell_id='a',status='accepted_candidate')]
        self.deck=dict(scenes=[dict(id='a'),dict(id='b')])
        self.pins={str(self.raw):exact_module.common.sha(self.raw)}
        self.doc=dict(manifest_sha256='source-sha')
        self.scope=dict(seed=self.seed,profile=self.profile,baseline=self.baseline)
        common=SimpleNamespace(sha=exact_module.common.sha,physical=exact_module.common.physical,
            write_new=exact_module.common.write_new,handoff=SimpleNamespace(idle_locks=lambda baseline:nullcontext()),)
        helper=write(self.root/'complete_unified.py','# pinned\n')
        cache=write(self.root/'overlay_read_cache.py','# pinned\n')
        self.boundary=SimpleNamespace(verify=Mock())
        self.published=[]
        def publish(folder,state,deck):
            self.published.append(copy.deepcopy(state))
            write(folder/'state.json',state)
            write(folder/'status.json',dict(stage='prepared',accepted_candidate=len(state['baseline_selected'])))
            write(folder/'quota.json',dict(accepted=1,deficit=1))
            exact_module.atomic_rows(folder/'selected_ledger.jsonl',state['baseline_selected'])
            write(folder/'adopted_manifest.json',dict(count=1,scenes=[dict(id='a',native=True)]))
        self.exact=SimpleNamespace(common=common,read=exact_module.read,rows=exact_module.rows,
            __file__=str(helper),overlay_read_cache=SimpleNamespace(__file__=str(cache)),
            POLICY=exact_module.POLICY,summary=Mock(return_value=(self.doc,self.baseline,self.seed,self.deck,self.selected,[dict(id='b')],self.pins)),
            verify_baseline=Mock(return_value=self.boundary),prior_payloads=Mock(return_value={'old-a','old-failed-b'}),publish=Mock(side_effect=publish))
        common.helpers=lambda:{str(helper):common.sha(helper)}

    def test_seed_uses_original_contract_and_preserves_prefix_without_capture_or_geometry(self):
        before=self.raw.read_bytes()
        receipt=wrapper.prepare_seed(self.exact,self.scope)
        state=self.published[0]
        self.assertEqual(state['attempts'],[])
        self.assertEqual(state['baseline_selected'],self.selected)
        self.assertEqual(state['deficits'],['b'])
        self.assertEqual(state['contract']['policy'],exact_module.POLICY)
        self.assertEqual(state['contract']['attempts'],2)
        self.assertEqual(receipt['native_captures'],0)
        self.assertEqual(receipt['geometry_proposals'],0)
        self.assertFalse(receipt['old_native_qa_recomputed'])
        self.assertEqual(before,self.raw.read_bytes())
        self.boundary.verify.assert_called_once()

    def test_existing_seed_validation_is_read_only_and_does_not_rerun_old_qa(self):
        wrapper.prepare_seed(self.exact,self.scope)
        before={p:p.read_bytes() for p in self.seed.rglob('*') if p.is_file()}
        self.exact.summary.reset_mock();self.exact.verify_baseline.reset_mock()
        wrapper.prepare_seed(self.exact,self.scope)
        self.exact.summary.assert_not_called();self.exact.verify_baseline.assert_not_called()
        self.assertEqual(before,{p:p.read_bytes() for p in self.seed.rglob('*') if p.is_file()})

    def test_changed_seed_cannot_be_overwritten(self):
        wrapper.prepare_seed(self.exact,self.scope)
        path=self.seed/'selected_ledger.jsonl';path.write_text('changed\n')
        with self.assertRaisesRegex(ValueError,'never overwrite'):
            wrapper.prepare_seed(self.exact,self.scope)
        self.assertEqual(path.read_text(),'changed\n')

    def test_parent_change_fails_before_new_output_creation(self):
        write(self.raw,dict(never_rewrite=False))
        with self.assertRaisesRegex(ValueError,'Terminal parent changed'):
            wrapper.prepare_seed(self.exact,self.scope)
        self.assertFalse(self.seed.exists())
        self.boundary.verify.assert_called_once()

    def test_partial_existing_seed_is_not_silently_reprepared(self):
        self.seed.mkdir()
        with self.assertRaises(FileNotFoundError):
            wrapper.prepare_seed(self.exact,self.scope)
        self.exact.summary.assert_not_called()

    def paused_output(self):
        output=self.root/'output';output.mkdir()
        self.scope['output']=output
        write(output/'status.json',dict(stage='paused',error=None,failed=0,current=None))
        write(output/'state.json',dict(stage='paused',error=None,current=None,
            contract=dict(parent=str(self.seed),profile_sha256=self.exact.common.sha(self.profile))))
        write(output/'PAUSE','operator pause history\n')
        return output

    def test_paused_resume_preserves_marker_history(self):
        output=self.paused_output()
        saved=wrapper.resume_pause(self.exact,self.scope)
        self.assertEqual(saved.read_text(),'operator pause history\n')
        self.assertFalse((output/'PAUSE').exists())
        self.assertIsNone(wrapper.resume_pause(self.exact,self.scope))

    def test_stop_and_halted_pause_cannot_be_consumed(self):
        output=self.paused_output();write(output/'STOP','explicit stop')
        with self.assertRaisesRegex(ValueError,'STOP'):
            wrapper.resume_pause(self.exact,self.scope)
        (output/'STOP').unlink()
        write(output/'status.json',dict(stage='halted',failed=1,error='Park unverified',current=None))
        with self.assertRaisesRegex(ValueError,'safe unchanged'):
            wrapper.resume_pause(self.exact,self.scope)
        self.assertTrue((output/'PAUSE').exists())

    def test_symlink_pause_is_rejected(self):
        output=self.paused_output();(output/'PAUSE').unlink()
        (output/'PAUSE').symlink_to(self.raw)
        with self.assertRaisesRegex(RuntimeError,'nonsymlink'):
            wrapper.resume_pause(self.exact,self.scope)

    def test_time_budget_pause_needs_no_marker_mutation(self):
        output=self.paused_output();(output/'PAUSE').unlink()
        before={p:p.read_bytes() for p in output.iterdir()}
        self.assertIsNone(wrapper.resume_pause(self.exact,self.scope))
        self.assertEqual(before,{p:p.read_bytes() for p in output.iterdir()})


if __name__ == '__main__':
    unittest.main()
