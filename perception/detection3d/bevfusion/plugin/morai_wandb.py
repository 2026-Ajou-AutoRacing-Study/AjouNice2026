"""Canonical W&B names shared with the team's other 3D detectors."""

from mmengine.hooks import Hook
from mmengine.registry import HOOKS, VISBACKENDS
from mmengine.visualization import WandbVisBackend


@VISBACKENDS.register_module()
class MoraiWandbVisBackend(WandbVisBackend):
    """Namespace MMEngine scalars without changing the runner itself."""

    def _init_env(self):
        super()._init_env()
        self._wandb.define_metric('global_step', hidden=True)
        self._wandb.define_metric('epoch', hidden=True)
        self._wandb.define_metric('train/*', step_metric='global_step')
        self._wandb.define_metric('val/*', step_metric='epoch', hidden=True)
        for name in (
                'val/center/mAP',
                'val/center/vehicle_mean_AP',
                'val/center/pedestrian_mean_AP',
                'val/center/obstacle_mean_AP'):
            self._wandb.define_metric(
                name, step_metric='epoch', hidden=False,
                summary='max', overwrite=True)
        for class_name in ('vehicle', 'pedestrian', 'obstacle'):
            for metric in ('mATE', 'mASE', 'mAOE'):
                self._wandb.define_metric(
                    f'val/tp/{class_name}_{metric}',
                    step_metric='epoch', hidden=False,
                    summary='min', overwrite=True)

    @staticmethod
    def _canonicalize(scalars, step):
        is_eval = any(
            key.startswith(('val/', 'test/')) for key in scalars)
        prefix = 'val/' if is_eval else 'train/'
        payload = {}
        for key, value in scalars.items():
            if key in {'epoch', 'global_step'} or key.startswith(
                    ('train/', 'val/', 'test/')):
                payload[key] = value
            else:
                payload[prefix + key] = value
        payload.setdefault('global_step', step)
        return payload

    def add_scalar(self, name, value, step=0, **kwargs):
        self.experiment.log(
            self._canonicalize({name: value}, step), commit=self._commit)

    def add_scalars(self, scalar_dict, step=0, file_path=None, **kwargs):
        self.experiment.log(
            self._canonicalize(scalar_dict, step), commit=self._commit)


@HOOKS.register_module()
class MoraiEpochLrHook(Hook):
    """Publish an epoch-aligned LR key used by existing W&B panels."""

    priority = 'LOWEST'

    def after_train_epoch(self, runner):
        learning_rates = runner.optim_wrapper.get_lr()
        if isinstance(learning_rates, dict):
            learning_rates = next(iter(learning_rates.values()))
        if isinstance(learning_rates, (list, tuple)):
            learning_rate = learning_rates[0]
        else:
            learning_rate = learning_rates
        runner.visualizer.add_scalars({
            'epoch': runner.epoch + 1,
            'train/lr_epoch': float(learning_rate),
        }, step=runner.iter)
