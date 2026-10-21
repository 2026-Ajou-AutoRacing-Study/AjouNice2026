#!/usr/bin/env python3
"""Exact E1-fixture target/loss/output-gradient gate for discarded-target removal.

Build through the ordered full-data benchmark preparation, then use up to three
real augmented batches. No optimizer step, model/checkpoint write, W&B or network.
Forward the E1 model once per fixture; A/B uses those exact detached head outputs
so stochastic augmentation, feature kernels and BatchNorm cannot confound it.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import random
import sys
import time

import numpy as np

sys.dont_write_bytecode = True


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def freeze_tree(value, path='', flags=None):
    """Keep each original requires_grad flag without retaining its model graph."""
    import torch
    if flags is None: flags = {}
    if torch.is_tensor(value):
        flags[path] = bool(value.requires_grad)
        return value.detach().clone(), flags
    if isinstance(value, dict):
        result = {key: freeze_tree(item, f'{path}/{key}', flags)[0]
                  for key, item in value.items()}
    elif isinstance(value, (tuple, list)):
        values = [freeze_tree(item, f'{path}/{i}', flags)[0] for i, item in enumerate(value)]
        result = tuple(values) if isinstance(value, tuple) else values
    else: result = copy.deepcopy(value)
    return result, flags


def thaw_tree(value, flags, path='', leaves=None):
    """Use nonleaf +0 so the original in-place clip_sigmoid remains legal."""
    import torch
    if leaves is None: leaves = {}
    if torch.is_tensor(value):
        leaf = value.detach().clone().requires_grad_(flags[path])
        if flags[path]: leaves[path] = leaf
        return leaf+0 if flags[path] else leaf, leaves
    if isinstance(value, dict):
        result = {key: thaw_tree(item, flags, f'{path}/{key}', leaves)[0]
                  for key, item in value.items()}
    elif isinstance(value, (tuple, list)):
        values = [thaw_tree(item, flags, f'{path}/{i}', leaves)[0]
                  for i, item in enumerate(value)]
        result = tuple(values) if isinstance(value, tuple) else values
    else: result = copy.deepcopy(value)
    return result, leaves


def rng_digest():
    import torch
    value = hashlib.sha256(repr(random.getstate()).encode())
    state = np.random.get_state()
    value.update(state[0].encode()); value.update(state[1].tobytes())
    value.update(repr(state[2:]).encode())
    value.update(torch.get_rng_state().numpy().tobytes())
    for state in torch.cuda.get_rng_state_all(): value.update(state.cpu().numpy().tobytes())
    return value.hexdigest()


def cpu_targets(targets):
    import torch
    return [value.detach().cpu().clone() if torch.is_tensor(value) else copy.deepcopy(value)
            for value in targets]


def targets_equal(left, right):
    import torch
    return len(left) == len(right) and all(torch.equal(a, b) if torch.is_tensor(a)
                                          else a == b for a, b in zip(left, right))


def loss_and_gradients(head, template, flags, ground_truth):
    import torch
    predictions, leaves = thaw_tree(template, flags)
    with torch.autocast('cuda', dtype=torch.float16):
        losses = head.loss_by_feat(predictions, ground_truth)
        total = sum(value.mean() if torch.is_tensor(value) else value
                    for name, value in losses.items() if 'loss' in name)
    total.backward(); torch.cuda.synchronize()
    scalars = {key: float(value.detach().cpu()) if torch.is_tensor(value) else float(value)
               for key, value in losses.items()}
    gradients = {key: value.grad.detach().cpu().clone() if value.grad is not None else None
                 for key, value in leaves.items()}
    finite = all(np.isfinite(value) for value in scalars.values()) and all(
        value is None or bool(torch.isfinite(value).all()) for value in gradients.values())
    return scalars, gradients, finite


def gradient_comparison(left, right):
    import torch
    exact = left.keys() == right.keys(); maximum = 0.; details = {}
    for key in left.keys() | right.keys():
        a, b = left.get(key), right.get(key)
        same = (a is None and b is None) or (a is not None and b is not None and torch.equal(a, b))
        delta = float((a.float()-b.float()).abs().max()) if a is not None and b is not None else None
        if delta is not None: maximum = max(maximum, delta)
        exact = exact and same
        details[key] = dict(exact=bool(same), max_abs=delta,
                            shape=list(a.shape) if a is not None else None)
    return dict(exact=bool(exact), max_abs=maximum, leaves=details,
                scope='dLoss/dHeadOutput on identical actual E1 predictions; no optimizer update')


def main(argv=None):
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument('--heatmap-extension', type=Path, required=True)
    parser.add_argument('--heatmap-sha256', required=True)
    parser.add_argument('--fixture-batches', type=int, default=3)
    parser.add_argument('--target-repeats', type=int, default=4)
    own, forwarded = parser.parse_known_args(argv)
    common = load_module(Path(__file__).with_name('benchmark_base5_full_runtime.py'), '_head_gate_common')
    args = common.parse_args(forwarded)
    common.require(args.mode == 'val' and 1 <= own.fixture_batches <= 3
                   and 2 <= own.target_repeats <= 6, 'Use bounded val-mode fixture preparation')
    common.require(own.heatmap_extension.is_absolute()
                   and common.digest(own.heatmap_extension) == own.heatmap_sha256,
                   'Explicit qualified heatmap extension and SHA required')
    candidate = load_module(own.heatmap_extension, '_qualified_skip_heatmap')
    import torch
    from torch.utils.data import Subset
    from mmengine.runner import Runner
    from mmengine.utils import import_modules_from_strings
    torch.set_num_threads(args.torch_threads)
    args.output.mkdir(parents=True)
    cfg, manifest = common.reconstruct(args)
    import_modules_from_strings(**cfg.custom_imports)
    from perception.detection3d.bevfusion.plugin.morai_remote_checkpoint import Base5AtomicRunner
    runner = Base5AtomicRunner.from_cfg(cfg)
    runner._init_model_weights(); runner.load_or_resume(); runner.model.train()
    initial = Runner.build_dataloader(copy.deepcopy(cfg.train_dataloader), seed=args.seed)
    dataset = initial.dataset
    indices = common.cohort_indices(dataset, own.fixture_batches*4)
    loader_cfg = copy.deepcopy(cfg.train_dataloader)
    loader_cfg.update(dataset=Subset(dataset, indices), batch_size=4)
    loader = Runner.build_dataloader(loader_cfg, seed=args.seed)
    del initial
    report = dict(valid=True, complete=False, expected_fixture_batches=own.fixture_batches,
        contract_sha256=manifest['contract_sha256'],
        checkpoint=str(args.checkpoint), checkpoint_sha256=common.digest(args.checkpoint),
        extension=candidate.receipt(),
        no_optimizer_step=True, no_checkpoint_write=True, model_original_unmodified=True,
        compile_camera=False, fixtures=[], selected_indices=indices)
    try:
        for fixture_index, batch in enumerate(loader):
            if fixture_index >= own.fixture_batches: break
            candidate.uninstall()
            processed = runner.model.data_preprocessor(batch, training=True)
            ground_truth = [sample.gt_instances_3d for sample in processed['data_samples']]
            metas = [sample.metainfo for sample in processed['data_samples']]
            with torch.no_grad(), torch.autocast('cuda', dtype=torch.float16):
                features = runner.model.extract_feat(processed['inputs'], metas)
            with torch.autocast('cuda', dtype=torch.float16):
                raw = runner.model.bbox_head(features, metas)
            template, flags = freeze_tree(raw)
            del raw, features
            head = runner.model.bbox_head
            before_rng = rng_digest()
            with torch.no_grad():
                baseline_targets = cpu_targets(head.get_targets(ground_truth, template[0]))
            baseline_losses, baseline_gradients, baseline_finite = loss_and_gradients(
                head, template, flags, ground_truth)
            common.require(before_rng == rng_digest(), 'Baseline targets/loss consumed RNG')
            candidate.install()
            with torch.no_grad():
                candidate_targets = cpu_targets(head.get_targets(ground_truth, template[0]))
            candidate_losses, candidate_gradients, candidate_finite = loss_and_gradients(
                head, template, flags, ground_truth)
            common.require(before_rng == rng_digest(), 'Candidate targets/loss consumed RNG')
            gradient = gradient_comparison(baseline_gradients, candidate_gradients)
            exact_targets = targets_equal(baseline_targets, candidate_targets)
            exact_losses = baseline_losses == candidate_losses
            timings = dict(baseline=[], candidate=[])
            # ABBA reverses timing order within the same unchanged real fixture.
            with torch.no_grad():
                for repeat in range(own.target_repeats):
                    order = ('baseline', 'candidate') if repeat % 2 == 0 else ('candidate', 'baseline')
                    for name in order:
                        candidate.install() if name == 'candidate' else candidate.uninstall()
                        torch.cuda.synchronize(); start = time.perf_counter()
                        targets = head.get_targets(ground_truth, template[0])
                        torch.cuda.synchronize(); timings[name].append((time.perf_counter()-start)*1000)
                        del targets
            row = dict(index=fixture_index,
                tokens=[str(sample.metainfo['sample_idx']) for sample in processed['data_samples']],
                gt_counts=[len(gt.labels_3d) for gt in ground_truth],
                targets_exact=exact_targets, losses_exact=exact_losses,
                baseline_losses=baseline_losses, candidate_losses=candidate_losses,
                gradient=gradient, finite=baseline_finite and candidate_finite,
                RNG_exact=before_rng == rng_digest(),
                target_wall_ms={name: common.summary(values) for name, values in timings.items()})
            report['valid'] = report['valid'] and all((exact_targets, exact_losses, gradient['exact'], row['finite'], row['RNG_exact']))
            report['fixtures'].append(row)
            (args.output/'qualification.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
            print(json.dumps(row), flush=True)
            del template, ground_truth, baseline_targets, candidate_targets
            del baseline_gradients, candidate_gradients, processed
            torch.cuda.empty_cache()
        common.require(len(report['fixtures']) == own.fixture_batches, 'Fixture collection incomplete')
        common.require(not any(parameter.grad is not None for parameter in runner.model.parameters()),
                       'Isolated head-output derivative gate unexpectedly accumulated model gradients')
        common.require(report['valid'], 'Exact target/loss/gradient gate failed; do not promote')
        report['complete'] = True
        (args.output/'qualification.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    finally:
        candidate.uninstall()
        iterator = getattr(loader, '_iterator', None)
        if iterator is not None: iterator._shutdown_workers()
    print(json.dumps(dict(valid=report['valid'], fixtures=len(report['fixtures']),
                          scope='exact used-target/loss/output-gradient gate; not full model throughput')), flush=True)


if __name__ == '__main__': main()
