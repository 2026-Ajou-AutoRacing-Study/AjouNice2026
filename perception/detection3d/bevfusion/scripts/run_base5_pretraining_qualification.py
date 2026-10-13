#!/usr/bin/env python3
"""Sequential, bounded cost/resume checks; never start full dataset training."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--init-checkpoint', type=Path, required=True)
    parser.add_argument('--phase', choices=('all', 'finalize'), default='all')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Use a fresh output; probe results must not be overwritten')
    args.output.mkdir(parents=True)
    base = Path(__file__).resolve().parents[1]
    results = []

    def run(name, command, timeout=900):
        started = time.monotonic()
        print('START ' + name, flush=True)
        with (args.output / (name + '.log')).open('w') as log:
            try:
                result = subprocess.run([sys.executable, *map(str, command)],
                                        stdout=log, stderr=subprocess.STDOUT,
                                        timeout=timeout)
                code = result.returncode
            except subprocess.TimeoutExpired:
                code = 'timeout'
        row = dict(name=name, returncode=code, seconds=time.monotonic()-started)
        results.append(row)
        (args.output/'execution.json').write_text(json.dumps(results, indent=2)+'\n')
        print('DONE ' + json.dumps(row), flush=True)
        return row

    selection = json.loads((args.output.parent/'samples/manifest.json').read_text())
    selection['samples'] = [r for group in ('representative', 'stress')
                            for r in [s for s in selection['samples']
                                      if s['sample_group'] == group][:32]]
    selection['scope']='64 distinct numerical-equivalence frames; not AP or latency population statistics'
    from collections import Counter
    selection['sampled_families']=dict(Counter(r['family'] for r in selection['samples']))
    selection['sampled_groups']=dict(Counter(r['sample_group'] for r in selection['samples']))
    numeric_samples = args.output/'numerical_samples'
    numeric_samples.mkdir()
    (numeric_samples/'manifest.json').write_text(json.dumps(selection, indent=2)+'\n')
    run('numerical_64', [base/'scripts/validate_base5_pool_model.py',
        '--root', '/root/ajou_bench/ajounice_input_pool_gpu/payload',
        '--samples', numeric_samples, '--infos', args.data/'infos/448x896',
        '--output', args.output/'numerical_64.json', '--skip-training'])
    for profile in ('448x896', '512x1056'):
        config = base/'configs'/('bevfusion_base5_dense_' + profile.split('x')[0] + '_separated_candidate.py')
        for batch in (2, 4, 1, 8):
            if args.phase == 'finalize':
                previous = args.output.parent/'training_probes'/(profile+'_b'+str(batch))/'cost.json'
                if not previous.exists():
                    continue
                previous_cost = json.loads(previous.read_text())
                if not previous_cost.get('valid') or previous_cost.get('memory_measurement'):
                    continue
            if batch == 8:
                previous = args.output/(profile+'_b4')/'cost.json'
                if not previous.exists():
                    continue
                cost = json.loads(previous.read_text())
                if not cost.get('valid') or cost['peak_reserved_mib'] > 27000:
                    continue
            name = profile+'_b'+str(batch)
            run(name, [base/'scripts/benchmark_base5_real_loader.py',
                '--data', args.data, '--profile', profile, '--config', config,
                '--work-dir', args.output/name, '--init-checkpoint', args.init_checkpoint,
                '--batch', batch, '--workers', 4, '--steps', 80, '--warmup', 16])
        if args.phase == 'all':
            run(profile+'_resume', [base/'tools/smoke_base5_direct_training.py',
                '--data', args.data, '--profile', profile, '--config', config,
                '--output', args.output/(profile+'_resume'),
                '--init-checkpoint', args.init_checkpoint])
    run('exact_resume_unit', [base/'tools/test_base5_remote_resume.py'])
    run('checkpoint_backup_unit', [base/'tools/test_base5_checkpoint_backup.py'])
    run('direct_training_unit', [base/'tools/test_base5_direct_training.py'])
    run('service_300_uninstrumented', [base/'scripts/benchmark_base5_input_optimization.py',
        '--root', '/root/ajou_bench/ajounice_input_pool_gpu/payload',
        '--samples', args.output.parent/'samples', '--output', args.output.parent/'service_300.json',
        '--iterations', 300, '--shapes', '448x896', '512x1056', '--modes', 'separated',
        '--codec', 'jpeg90', '--skip-preprocess', '--no-stages'])
    print('QUALIFICATION FINISHED; FULL TRAINING NOT STARTED', flush=True)
    if any(r['returncode'] not in (0, 2) for r in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
