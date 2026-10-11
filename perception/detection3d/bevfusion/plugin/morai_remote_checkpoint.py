"""Opt-in single-GPU, epoch-boundary resume evidence and atomic local saves.

Worker RNG/prefetch and arbitrary mid-epoch replay are NOT guaranteed. No
production config imports this module. The direct Base5 runner imports it.
"""
import json
import os
from pathlib import Path
import random
import uuid

import numpy as np
import torch
from mmengine.hooks import Hook
from mmengine.runner import Runner
from mmengine.model import is_model_wrapper
from mmengine.runner.checkpoint import _load_checkpoint_to_model
from mmdet3d.registry import HOOKS


def optimizer_parameter_names(runner):
    model = runner.model.module if is_model_wrapper(runner.model) else runner.model
    names = {id(parameter): name for name, parameter in model.named_parameters()}
    return [[names[id(parameter)] for parameter in group['params']]
            for group in runner.optim_wrapper.optimizer.param_groups]


def validate_optimizer_parameter_order(runner, checkpoint):
    """Optimizer state loads positionally; shape-compatible reordering is unsafe.

    A pretty-printed resolved config can sort SeparateHead dictionaries while
    retaining identical semantic config values. Never silently attach its Adam
    moments to a different branch. Legacy Base5 has one full parameter group;
    its original parameter order is recoverable from ordered state_dict keys.
    """
    current = optimizer_parameter_names(runner)
    expected = checkpoint.get('base5_optimizer_parameter_names')
    if expected is None:
        if len(current) != 1:
            raise ValueError('Legacy resume needs an explicit optimizer parameter-name map')
        parameters = set(current[0])
        expected = [[key for key in checkpoint['state_dict'] if key in parameters]]
        if len(expected[0]) != len(current[0]):
            raise ValueError('Cannot recover legacy optimizer parameter order')
    if current != expected:
        raise ValueError('Resume rejected: optimizer parameter names/order differ; '
                         'use the original source config, not its sorted resolved dump')
    saved = checkpoint['optimizer']
    groups = runner.optim_wrapper.optimizer.param_groups
    if len(saved['param_groups']) != len(groups):
        raise ValueError('Resume rejected: optimizer parameter groups differ')
    for old_group, new_group in zip(saved['param_groups'], groups):
        if len(old_group['params']) != len(new_group['params']):
            raise ValueError('Resume rejected: optimizer parameter count differs')
        for old_id, parameter in zip(old_group['params'], new_group['params']):
            for key in ('exp_avg', 'exp_avg_sq', 'max_exp_avg_sq'):
                value = saved['state'].get(old_id, {}).get(key)
                if value is not None and value.shape != parameter.shape:
                    raise ValueError('Resume rejected: optimizer moment shape differs')


@HOOKS.register_module()
class MoraiResumeEvidenceHook(Hook):
    priority = 'VERY_LOW'

    def __init__(self, contract_sha256):
        self.contract_sha256 = contract_sha256
        self.pending_rng = None

    def before_save_checkpoint(self, runner, checkpoint):
        checkpoint['base5_contract_sha256'] = self.contract_sha256
        checkpoint['base5_optimizer_parameter_names'] = optimizer_parameter_names(runner)
        checkpoint['base5_rng'] = dict(python=random.getstate(), numpy=np.random.get_state(),
            torch=torch.get_rng_state(), cuda=torch.cuda.get_rng_state_all() if torch.cuda.is_available() else [])
        checkpoint['base5_resume_scope'] = 'epoch boundary; worker RNG/prefetch and exact mid-epoch replay not guaranteed'

    def after_load_checkpoint(self, runner, checkpoint):
        if not runner.cfg.get('resume', False):
            return
        if checkpoint.get('base5_contract_sha256') != self.contract_sha256:
            raise ValueError('Resume rejected: dataset/model/schedule contract differs')
        required = ('optimizer', 'param_schedulers', 'message_hub', 'base5_rng')
        if any(key not in checkpoint for key in required):
            raise ValueError('Incomplete resumable checkpoint')
        validate_optimizer_parameter_order(runner, checkpoint)
        self.pending_rng = checkpoint['base5_rng']

    def before_train(self, runner):
        if self.pending_rng is not None:
            rng = self.pending_rng
            random.setstate(rng['python']); np.random.set_state(rng['numpy'])
            torch.set_rng_state(rng['torch'].cpu())
            if rng['cuda']:
                torch.cuda.set_rng_state_all([item.cpu() for item in rng['cuda']])
        wrapper = runner.optim_wrapper
        steps = sorted(set(float(item['step']) for item in wrapper.optimizer.state.values() if 'step' in item))
        result = dict(resumed=self.pending_rng is not None, epoch=runner.epoch, iteration=runner.iter,
            lr=wrapper.get_lr(), optimizer_steps=steps,
            loss_scaler=wrapper.loss_scaler.state_dict() if hasattr(wrapper, 'loss_scaler') else None,
            contract_sha256=self.contract_sha256,
            scope='main-process RNG plus full optimizer/scheduler/scaler; worker/prefetch replay not guaranteed')
        name = 'resume_start.json' if self.pending_rng is not None else 'fresh_start.json'
        (Path(runner.work_dir) / name).write_text(json.dumps(result, indent=2) + '\n')


class Base5AtomicRunner(Runner):
    def load_checkpoint(self, filename, map_location='cpu', strict=False,
                        revise_keys=[(r'^module\.', '')]):
        # MMEngine 0.10.7 calls torch.load without weights_only; Torch 2.7's
        # default cannot read full MMEngine HistoryBuffer/RNG resume state.
        # Limit the explicit unsafe pickle loader to TRUSTED LOCAL files in
        # this opt-in runner; never change global torch.load defaults.
        path = Path(filename).resolve(strict=True)
        checkpoint = torch.load(path, map_location=map_location, weights_only=False)
        self.call_hook('after_load_checkpoint', checkpoint=checkpoint)
        model = self.model.module if is_model_wrapper(self.model) else self.model
        checkpoint = _load_checkpoint_to_model(model, checkpoint,
            strict or self.cfg.get('resume', False), logger=self.logger, revise_keys=revise_keys)
        self._has_loaded = True
        self.logger.info('Loaded trusted local checkpoint: %s', path)
        return checkpoint

    def save_checkpoint(self, out_dir, filename, *args, **kwargs):
        # Only local filesystem work_dirs are supported. A uploader sends a
        # completed checkpoint separately; readers never see partial .pth.
        target = Path(out_dir) / filename
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = filename + '.partial-' + uuid.uuid4().hex
        partial = Path(out_dir) / temporary
        try:
            super().save_checkpoint(out_dir, temporary, *args, **kwargs)
            with partial.open('rb') as handle:
                os.fsync(handle.fileno())
            os.replace(partial, target)
        finally:
            partial.unlink(missing_ok=True)
