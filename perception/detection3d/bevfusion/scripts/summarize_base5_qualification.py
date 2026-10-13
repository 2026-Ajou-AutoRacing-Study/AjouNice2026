#!/usr/bin/env python3
"""Separate representative/stress costs; extrapolations are not full-epoch measurements."""
import argparse
import json
from pathlib import Path

import numpy as np


def stats(values):
    values = np.asarray(values, dtype=float)
    return dict(n=len(values), mean=float(values.mean()),
                p50=float(np.percentile(values, 50)), p95=float(np.percentile(values, 95)),
                p99=float(np.percentile(values, 99)), maximum=float(values.max()))


def summarize_inference(result):
    samples = {r['id']: r for r in result['source']['samples']}
    output = []
    for case in result['cases']:
        if not case.get('valid'):
            output.append(dict(shape=case['shape'], mode=case['mode'], valid=False))
            continue
        rows = case['per_sample']
        if len(rows) != 300 or len({r['id'] for r in rows}) != 300:
            raise ValueError('Expected exactly 300 distinct measurement scenes per case')
        groups = {}
        for group in ('representative', 'stress'):
            subset = [r for r in rows if samples[r['id']]['sample_group'] == group]
            groups[group] = {key: stats([r[key] for r in subset])
                             for key in ('prepare_ms', 'model_ms', 'service_ms')}
            groups[group]['service_above_13hz_budget'] = sum(r['service_ms'] > 1000/13 for r in subset)
        output.append(dict(shape=case['shape'], mode=case['mode'], valid=True, groups=groups,
                           steady_peak_allocated_mib=case['steady_peak_allocated_mib'],
                           stages=case['stages']))
    return output


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    inference = json.loads((args.root/'inference_300.json').read_text())
    costs = []
    cost_files = {f.parent.name: f for f in sorted((args.root/'training_probes').glob('*/cost.json'))}
    cost_files.update({f.parent.name: f for f in sorted((args.root/'final_checks').glob('*/cost.json'))})
    for file in cost_files.values():
        cost = json.loads(file.read_text())
        if not cost.get('valid'):
            costs.append(dict(name=file.parent.name, valid=False, oom=cost.get('oom', False)))
            continue
        time_ms = cost['timing']['total_ms']['mean']
        costs.append(dict(name=file.parent.name, profile=cost['profile'], valid=True,
            batch=cost['batch'], examples_per_second=cost['batch']*1000/time_ms,
            measured_steps=cost['measured_steps'], timing=cost['timing'],
            peak_allocated_mib=cost['peak_allocated_mib'], peak_reserved_mib=cost['peak_reserved_mib'],
            memory_measurement_valid=bool(cost.get('memory_measurement')),
            warmup_peak_allocated_mib=cost.get('warmup_peak_allocated_mib'),
            epoch_5k_train_seconds=int(np.ceil(4255/cost['batch']))*time_ms/1000,
            epoch_30k_train_seconds=int(np.ceil(25500/cost['batch']))*time_ms/1000,
            overhead='excludes validation, checkpoint/backup and prolonged thermal/storage changes'))
    result = dict(inference=summarize_inference(inference), training_costs=costs,
                  full_dataset_training_started=False,
                  scope='Cost and resume qualification only, not learned AP or ROS/full-stack latency')
    service = args.root/'service_300.json'
    if service.exists():
        result['uninstrumented_service'] = summarize_inference(json.loads(service.read_text()))
    numeric = args.root/'numerical_64_repaired.json'
    if numeric.exists():
        result['numerical_equivalence'] = json.loads(numeric.read_text())['equivalence']
    result['resume_proofs'] = [json.loads(f.read_text()) for f in
                             sorted((args.root/'training_probes').glob('*_resume/report.json'))]
    result['validation_proofs'] = [{k:v for k,v in json.loads(f.read_text()).items()
                                   if k!='per_sample'} for f in
                                  sorted(args.root.glob('validation_*/validation_cost.json'))]
    tail = args.root/'tail_batch3/cost.json'
    if tail.exists():
        result['tail_batch3'] = {k:v for k,v in json.loads(tail.read_text()).items() if k!='per_step'}
    # A small local backup unit does not qualify a real external full checkpoint.
    external = list((args.root/'training_probes').glob('*_resume/external_backup_report.json'))
    result['external_backup_qualified'] = any(json.loads(f.read_text()).get('valid') for f in external)
    watcher = args.root/'backup_watcher_report.json'
    result['backup_watcher_qualified'] = watcher.exists() and (
        json.loads(watcher.read_text()).get('successful_polls',0)>=2 and
        json.loads(watcher.read_text()).get('full_readback_verified',False))
    result['ready_to_start_full_training'] = (result['external_backup_qualified'] and
        result['backup_watcher_qualified'] and len(result['resume_proofs'])==2 and
        len(result['validation_proofs'])==2 and any(c.get('valid') and
            c.get('profile')=='512x1056' and c.get('batch')==4 and c.get('memory_measurement_valid')
            for c in costs)) and all(
        proof.get('valid') for proof in result['resume_proofs']+result['validation_proofs']) and (
        len(result.get('numerical_equivalence',[]))==4 and
        all(p['passed'] for p in result['numerical_equivalence']))
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
