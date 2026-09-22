"""Canonical W&B names shared with the team's other 3D detectors."""

from mmengine.hooks import Hook
from mmengine.registry import HOOKS, VISBACKENDS
from mmengine.visualization import WandbVisBackend


@VISBACKENDS.register_module()
class MoraiWandbVisBackend(WandbVisBackend):
    """Namespace MMEngine scalars without changing the runner itself."""

    def _init_env(self):
        super()._init_env()
        self._current_epoch = 0
        self._wandb.define_metric('global_step', hidden=True)
        self._wandb.define_metric('epoch', hidden=True)
        self._wandb.define_metric('epoch/*', step_metric='epoch')
        self._wandb.define_metric('train/*', step_metric='epoch')
        self._wandb.define_metric('val/*', step_metric='epoch', hidden=True)
        self._wandb.define_metric('eval/*', hidden=True)
        self._wandb.define_metric('test/*', step_metric='epoch', hidden=True)
        for name in (
                'val/center_mAP',
                'val/center/mAP',
                'val/center/vehicle_mean_AP',
                'val/center/pedestrian_mean_AP',
                'val/center/obstacle_mean_AP'):
            self._wandb.define_metric(
                name, step_metric='epoch', hidden=False,
                summary='max', overwrite=True)
        for class_name in ('vehicle', 'pedestrian', 'obstacle'):
            for threshold in ('0.5', '1', '2', '4'):
                self._wandb.define_metric(
                    f'val/center/{class_name}_AP_{threshold}m',
                    step_metric='epoch', hidden=False,
                    summary='max', overwrite=True)
            for metric in ('F1', 'precision', 'recall'):
                self._wandb.define_metric(
                    f'val/operating/{class_name}_1m_{metric}',
                    step_metric='epoch', hidden=False,
                    summary='max', overwrite=True)
            for metric in ('mATE', 'mASE', 'mAOE'):
                self._wandb.define_metric(
                    f'val/tp/{class_name}_{metric}',
                    step_metric='epoch', hidden=False,
                    summary='min', overwrite=True)
            for distance_range in ('0_10', '10_20', '20_30', '30_50'):
                self._wandb.define_metric(
                    f'val/range/{distance_range}m/{class_name}_mean_AP',
                    step_metric='epoch', hidden=False,
                    summary='max', overwrite=True)
            for threshold in ('0.25', '0.30', '0.50'):
                for metric in ('AP', 'rec'):
                    self._wandb.define_metric(
                        f'val/iou/{class_name}_{metric}_{threshold}',
                        step_metric='epoch', hidden=False,
                        summary='max', overwrite=True)
        for threshold in ('0.25', '0.30', '0.50'):
            for metric in ('mAP', 'mAR'):
                self._wandb.define_metric(
                    f'val/iou/{metric}_{threshold}',
                    step_metric='epoch', hidden=False,
                    summary='max', overwrite=True)

    def _canonicalize(self, scalars, step):
        if 'epoch' in scalars:
            self._current_epoch = scalars['epoch']
        is_eval = any(
            key.startswith(('val/', 'test/')) for key in scalars)
        prefix = 'val/' if is_eval else 'train/'
        payload = {}
        for key, value in scalars.items():
            if key in {'epoch', 'global_step'} or key.startswith(
                    ('epoch/', 'train/', 'val/', 'eval/', 'test/')):
                payload[key] = value
            else:
                payload[prefix + key] = value
        payload.setdefault('global_step', step)
        # MMEngine supplies epoch on regular logger payloads. Retain the most
        # recent value for isolated scalars (for example scheduler/LR hooks)
        # so every PointPainting-compatible chart has the same x axis.
        payload.setdefault('epoch', self._current_epoch)
        return payload

    def add_scalar(self, name, value, step=0, **kwargs):
        self.experiment.log(
            self._canonicalize({name: value}, step), commit=self._commit)

    def add_scalars(self, scalar_dict, step=0, file_path=None, **kwargs):
        self.experiment.log(
            self._canonicalize(scalar_dict, step), commit=self._commit)


@HOOKS.register_module()
class MoraiEpochLrHook(Hook):
    """Publish PointPainting-style epoch summaries and learning rate."""

    priority = 'LOWEST'

    @staticmethod
    def _history_mean(runner, name, window_size):
        for candidate in (f'train/{name}', name):
            try:
                history = runner.message_hub.get_scalar(candidate)
            except (KeyError, RuntimeError):
                continue
            try:
                return float(history.mean(window_size=window_size))
            except (TypeError, ValueError):
                return float(history.mean())
        return None

    def after_train_epoch(self, runner):
        learning_rates = runner.optim_wrapper.get_lr()
        if isinstance(learning_rates, dict):
            learning_rates = next(iter(learning_rates.values()))
        if isinstance(learning_rates, (list, tuple)):
            learning_rate = learning_rates[0]
        else:
            learning_rate = learning_rates
        payload = {
            'epoch': runner.epoch + 1,
            'train/lr_epoch': float(learning_rate),
            # These two spellings match the PointPainting epoch summary.
            'epoch/learning_rate_last': float(learning_rate),
        }
        # Param-wise LR experiments keep the spatial backbone conservative
        # while training newly initialized branches faster.  MMEngine's
        # default scalar exposes only the first group, so publish the effective
        # LR of the two Base4-specific groups explicitly when present.
        optimizer = runner.optim_wrapper.optimizer
        parameter_names = {
            id(parameter): name
            for name, parameter in runner.model.named_parameters()
        }
        named_groups = {
            'train/lr_velocity_head_epoch':
                'bbox_head.prediction_heads.0.vel',
            'train/lr_temporal_input_epoch':
                'pts_middle_encoder.conv_input.0',
        }
        for metric_name, name_fragment in named_groups.items():
            matched_lrs = []
            for group in optimizer.param_groups:
                names = [
                    parameter_names.get(id(parameter), '')
                    for parameter in group['params']
                ]
                if any(name_fragment in name for name in names):
                    matched_lrs.append(float(group['lr']))
            if matched_lrs:
                payload[metric_name] = max(matched_lrs)
        window_size = len(runner.train_dataloader)
        aliases = {
            'loss': 'epoch/train_loss_mean',
            'loss_heatmap': 'epoch/train_loss_heatmap_mean',
            'layer_-1_loss_cls': 'epoch/train_loss_cls_mean',
            'layer_-1_loss_bbox': 'epoch/train_loss_bbox_mean',
        }
        for source, target in aliases.items():
            value = self._history_mean(runner, source, window_size)
            if value is not None:
                payload[target] = value
        runner.visualizer.add_scalars(payload, step=runner.iter)
