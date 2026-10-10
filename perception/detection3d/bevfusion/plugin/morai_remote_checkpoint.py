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


@HOOKS.register_module()
class MoraiResumeEvidenceHook(Hook):
    priority = 'VERY_LOW'

    def __init__(self, contract_sha256):
        self.contract_sha256 = contract_sha256
        self.pending_rng = None

    def before_save_checkpoint(self, runner, checkpoint):
        checkpoint['base5_contract_sha256'] = self.contract_sha256
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
