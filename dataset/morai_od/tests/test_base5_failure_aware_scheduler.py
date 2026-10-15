from argparse import Namespace
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_base5_failure_aware_replenishment_fast as fast


class SchedulerTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        r=Path(self.tmp.name);m=r/'manifest.json';m.write_text(json.dumps(dict(config={})))
        self.args=Namespace(manifest=m,baseline=r/'old',output=r/'new',resume=True,run=True,
            stop_after_cells=3,max_attempts=2,candidates=16,time_budget_sec=900)

    def test_shared_context_is_prewarmed_without_changing_config(self):
        ctx={};config=dict(placement_ground={'enabled':True},regional_policy={'policy':'x'})
        with patch.object(fast,'configure') as ground,patch.object(fast,'regions') as region:
            self.assertIs(fast.prewarm(ctx,config),ctx)
            ground.assert_called_once_with(ctx,config['placement_ground']);region.assert_called_once_with(fast.fa.gen,ctx)

    def test_explicit_resume_clears_only_owned_budget_pause(self):
        self.args.output.mkdir();fast.fa.atomic_json(self.args.output/'PAUSE',dict(owner='failure_aware_time_budget'))
        original=fast.fa.gen.context
        with patch.object(fast.fa,'execute',return_value=0) as execute:
            self.assertEqual(fast.run(self.args),0);execute.assert_called_once()
        self.assertIs(fast.fa.gen.context,original);self.assertFalse((self.args.output/'PAUSE').exists())

    def test_user_pause_is_not_removed(self):
        self.args.output.mkdir();pause=self.args.output/'PAUSE';pause.write_text('user pause')
        with patch.object(fast.fa,'execute',return_value=0):fast.run(self.args)
        self.assertEqual(pause.read_text(),'user pause')

    def test_wrapper_guard_rejects_changed_code(self):
        self.args.output.mkdir();fast.fa.atomic_json(self.args.output/'fast_scheduler_contract.json',dict(changed=True))
        with patch.object(fast.fa,'execute') as execute:
            with self.assertRaisesRegex(ValueError,'scheduler changed'):fast.run(self.args)
            execute.assert_not_called()

    def test_bad_time_budget_rejected_before_output_or_ros(self):
        self.args.time_budget_sec=7200
        with self.assertRaises(ValueError):fast.run(self.args)
        self.assertFalse(self.args.output.exists())

    def test_budget_requests_pause_not_process_kill(self):
        args=self.args
        class ImmediateTimer:
            def __init__(self,seconds,callback):self.callback=callback
            def start(self):self.callback()
            def cancel(self):pass
        def execute(*a):
            self.assertEqual(fast.fa.read(args.output/'PAUSE')['owner'],'failure_aware_time_budget')
            return 0
        with patch.object(fast.threading,'Timer',ImmediateTimer),patch.object(fast.fa,'execute',side_effect=execute):
            self.assertEqual(fast.run(args),0)

    def test_context_restored_when_scheduler_child_raises(self):
        original=fast.fa.gen.context
        with patch.object(fast.fa,'execute',side_effect=RuntimeError('test failure')):
            with self.assertRaises(RuntimeError):fast.run(self.args)
        self.assertIs(fast.fa.gen.context,original)


if __name__=='__main__':unittest.main()
