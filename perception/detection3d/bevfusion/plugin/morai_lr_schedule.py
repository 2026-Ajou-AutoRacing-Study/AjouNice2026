"""Opt-in MMEngine scheduler; no extra scheduler hook or optimizer reset."""
from mmengine.optim.scheduler import _ParamScheduler
from mmengine.registry import PARAM_SCHEDULERS

from perception.detection3d.bevfusion.lr_schedule import restart_factor


@PARAM_SCHEDULERS.register_module()
class MoraiWarmupCosineRestarts(_ParamScheduler):
    def __init__(self, optimizer, cycle_steps, warmup_steps, start_factor=.1,
                 min_factor=.01, begin=0, end=1000000000, last_step=-1,
                 by_epoch=False, verbose=False):
        if by_epoch:
            raise ValueError('Base5 restarts must step per iteration')
        restart_factor(0, cycle_steps, warmup_steps, start_factor, min_factor)
        self.cycle_steps = cycle_steps
        self.warmup_steps = warmup_steps
        self.start_factor = start_factor
        self.min_factor = min_factor
        super().__init__(optimizer, 'lr', begin=begin, end=end, last_step=last_step,
                         by_epoch=False, verbose=verbose)

    def _get_value(self):
        factor = restart_factor(self.last_step, self.cycle_steps, self.warmup_steps,
                                self.start_factor, self.min_factor)
        return [base * factor for base in self.base_values]
