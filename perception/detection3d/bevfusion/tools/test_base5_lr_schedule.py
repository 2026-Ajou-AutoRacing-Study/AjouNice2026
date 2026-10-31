import copy
import unittest
import torch
from perception.detection3d.bevfusion.lr_schedule import restart_factor
from perception.detection3d.bevfusion.plugin.morai_lr_schedule import MoraiWarmupCosineRestarts


class RestartScheduleTests(unittest.TestCase):
    def optimizer(self):
        self.x = torch.nn.Parameter(torch.ones(1))
        self.y = torch.nn.Parameter(torch.ones(1))
        return torch.optim.AdamW([{'params': [self.x], 'lr': 1e-4},
                                 {'params': [self.y], 'lr': 2e-5}])

    def test_curve_and_group_ratios(self):
        optimizer = self.optimizer()
        scheduler = MoraiWarmupCosineRestarts(optimizer, cycle_steps=20, warmup_steps=3, end=40)
        for step in range(40):
            expected = restart_factor(step, 20, 3)
            self.assertAlmostEqual(optimizer.param_groups[0]['lr'], 1e-4 * expected)
            self.assertAlmostEqual(optimizer.param_groups[1]['lr'], 2e-5 * expected)
            optimizer.step(); scheduler.step()
        self.assertEqual(restart_factor(0, 20, 3), .1)
        self.assertEqual(restart_factor(3, 20, 3), 1.)
        self.assertEqual(restart_factor(20, 20, 3), .1)

    def test_resume_lr_and_adam_state_across_boundary(self):
        for split in (1, 3, 19, 20, 21, 39):
            optimizer = self.optimizer()
            scheduler = MoraiWarmupCosineRestarts(optimizer, 20, 3, end=40)
            for _ in range(split):
                (self.x.square() + self.y.square()).sum().backward()
                optimizer.step(); optimizer.zero_grad(); scheduler.step()
            opt_state = copy.deepcopy(optimizer.state_dict())
            sched_state = copy.deepcopy(scheduler.state_dict())
            other = self.optimizer()
            resumed = MoraiWarmupCosineRestarts(other, 20, 3, end=40)
            other.load_state_dict(opt_state); resumed.load_state_dict(sched_state)
            for _ in range(40 - split):
                self.assertEqual([g['lr'] for g in optimizer.param_groups], [g['lr'] for g in other.param_groups])
                optimizer.step(); scheduler.step(); other.step(); resumed.step()
            for state in other.state.values():
                self.assertEqual(state['step'].item(), split)

    def test_invalid_parameters(self):
        for cycle, warmup in ((1, 1), (20, 0), (20, 20)):
            with self.assertRaises(ValueError): restart_factor(0, cycle, warmup)


if __name__ == '__main__': unittest.main()
