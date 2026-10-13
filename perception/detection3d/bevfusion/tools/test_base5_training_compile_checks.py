"""Checkpoint proof comparison and profiling-hook order regression checks."""
import unittest
from types import SimpleNamespace

import numpy as np
import torch

from smoke_base5_direct_training import state_equal
from benchmark_base5_real_loader import Base5BoundedCost, Base5TrainingProfile
from mmengine.runner.priority import get_priority


class TrainingCompileChecks(unittest.TestCase):
    def audit_runner(self):
        model=torch.nn.Linear(2,1)
        for p in model.parameters():p.grad=torch.ones_like(p)
        calls=[]
        wrapper=SimpleNamespace(step=lambda: calls.append(True),
            loss_scaler=SimpleNamespace(get_scale=lambda:32.))
        runner=SimpleNamespace(model=model,optim_wrapper=wrapper,iter=0)
        hook=Base5BoundedCost(24,8,'unused_cost.json',4,audit_gradients=True)
        hook.before_train(runner)
        return runner,hook,calls

    def test_audit_does_not_skip_the_real_optimizer_step(self):
        runner,hook,calls=self.audit_runner()
        runner.optim_wrapper.step()
        self.assertEqual(calls,[True])
        self.assertTrue(hook.gradient_audits[0]['finite'])
        self.assertEqual(hook.gradient_audits[0]['missing_gradient_names'],[])
        self.assertTrue(runner._base5_audited_step)

    def test_missing_gradient_is_rejected_before_update(self):
        runner,hook,calls=self.audit_runner()
        runner.model.bias.grad=None
        with self.assertRaisesRegex(ValueError,'Missing trainable-parameter gradients'):
            runner.optim_wrapper.step()
        self.assertFalse(calls)

    def test_nonfinite_gradient_is_rejected_before_update(self):
        runner,hook,calls=self.audit_runner()
        runner.model.weight.grad[0,0]=float('nan')
        with self.assertRaisesRegex(ValueError,'Nonfinite training gradients'):
            runner.optim_wrapper.step()
        self.assertFalse(calls)

    def test_recursive_state_equality_checks_actual_tensor_values(self):
        a=dict(optimizer=dict(state={0:dict(step=torch.tensor(2.),
            exp_avg=torch.tensor([.2,.4]))}),rng=('numpy',np.array([1,2]),3))
        b=dict(optimizer=dict(state={0:dict(step=torch.tensor(2.),
            exp_avg=torch.tensor([.2,.4]))}),rng=('numpy',np.array([1,2]),3))
        self.assertTrue(state_equal(a,b))
        b['optimizer']['state'][0]['exp_avg'][0]+=.01
        self.assertFalse(state_equal(a,b))

    def test_missing_keys_rng_or_shape_changes_do_not_pass(self):
        self.assertFalse(state_equal(dict(a=1),dict(a=1,b=2)))
        self.assertFalse(state_equal(np.array([1,2]),np.array([1,3])))
        self.assertFalse(state_equal(torch.ones(2),torch.ones(1,2)))
        self.assertFalse(state_equal([1,2],(1,2)))

    def test_audit_installs_before_profiler_and_logger(self):
        # Profiler restoration must restore an audited callable, not silently
        # replace it with the unaudited optimizer after the profiling window.
        self.assertLess(get_priority(Base5BoundedCost.priority),get_priority(Base5TrainingProfile.priority))
        self.assertLess(get_priority(Base5BoundedCost.priority),get_priority('BELOW_NORMAL'))


if __name__=='__main__':unittest.main()
