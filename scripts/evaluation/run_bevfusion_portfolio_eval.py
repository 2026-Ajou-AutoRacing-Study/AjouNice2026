#!/usr/bin/env python3
"""Serial 5060 Ti evaluation queue; no live bridge, ports, training or deployment."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[2]
IMAGE='ajounice/bevfusion:team-trt-cu128-261023-v1'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();out=a.output.resolve()
    inventory=json.loads((out/'INVENTORY.json').read_text())
    if (out/'QUEUE_STARTED.json').exists():raise ValueError('Queue already started; do not duplicate GPU work')
    engine=json.loads((Path.home()/'bevfusion_artifacts/tensorrt/e12/runtime.json').read_text())
    base=ROOT/'perception/detection3d/bevfusion'
    config=base/'configs/bevfusion_base5_fine_512_depth25_nearest_depth_runtime.py'
    script=ROOT/'scripts/evaluation/export_bevfusion_portfolio_native.py'
    common=base/'tools/compare_base5_common30.py'
    history=[]
    (out/'QUEUE_STARTED.json').write_text(json.dumps(dict(pid=os.getpid(),image=IMAGE,
        gpu=subprocess.check_output(['nvidia-smi','-q'],text=True),inventory=inventory),indent=2)+'\n')
    def run(name,args):
        if (out/(name+'.log')).exists():raise ValueError('Fresh task log required: '+name)
        command=['docker','run','--rm','--name','portfolio-'+name.replace('_','-')+'-261109',
            '--gpus','device=0','--network','none','--shm-size','2g',
            '-e','PYTHONDONTWRITEBYTECODE=1','-e','OMP_NUM_THREADS=4',
            '-e',f'PYTHONPATH={ROOT}:{ROOT}/docker/bevfusion',
            '-v',f'{ROOT}:{ROOT}:ro','-v',f'{ROOT}:/workspace/AjouNice2026:ro',
            '-v',f'{Path.home()}/bevfusion_artifacts:{Path.home()}/bevfusion_artifacts:ro',
            '-v',f'{out}:/results','-v','bevfusion_bevfusion_torch_extensions:/root/.cache/torch_extensions',
            '--entrypoint','python',IMAGE,*map(str,args)]
        print(json.dumps(dict(event='start',task=name)),flush=True)
        at=time.monotonic()
        with (out/(name+'.log')).open('x') as log:
            result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
        history.append(dict(task=name,exit_code=result.returncode,seconds=time.monotonic()-at,command=command))
        (out/'QUEUE_PROGRESS.json').write_text(json.dumps(history,indent=2)+'\n')
        print(json.dumps(dict(event='finish',**history[-1])),flush=True)
        return result.returncode
    # Existing pilot-qualified E25 export is owned by this evaluation turn.
    while subprocess.check_output(['docker','ps','--filter','name=^/portfolio-e25-full-261109$',
                                  '--format','{{.Names}}'],text=True).strip():
        time.sleep(2)
    if not (out/'full_e25_fp32/receipt.json').exists():raise RuntimeError('E25 export did not finish')
    for backend in ('fp32','amp','trt_head'):
        args=[script,'--config',config,'--checkpoint',inventory['checkpoints']['e12']['path'],
            '--info','/results/pilot_infos.pkl','--output','/results/pilot_e12_'+backend,
            '--backend',backend,'--name','E12_'+backend+'_pilot','--expected-frames','8']
        if backend=='trt_head':args+=['--engine',engine['engine_path'],'--metadata',engine['metadata_path']]
        if run('pilot_e12_'+backend,args):raise RuntimeError('E12 pilot failed: '+backend)
    for backend in ('fp32','amp','trt_head'):
        args=[script,'--config',config,'--checkpoint',inventory['checkpoints']['e12']['path'],
            '--info',inventory['info'],'--output','/results/full_e12_'+backend,
            '--backend',backend,'--name','E12_'+backend.upper()]
        if backend=='trt_head':args+=['--engine',engine['engine_path'],'--metadata',engine['metadata_path']]
        if run('full_e12_'+backend,args):raise RuntimeError('E12 full validation failed: '+backend)
    predictions=['/results/full_e25_fp32/predictions.json.gz']+[
        '/results/full_e12_'+backend+'/predictions.json.gz' for backend in ('fp32','amp','trt_head')]
    if run('common30_metrics',[common,'compare','--predictions',*predictions,'--output','/results/common30_metrics']):
        raise RuntimeError('Native common30 scoring failed')
    # Each timing case uses its own process. Reverse order exposes drift.
    cases=['original','cache','separated','parallel','trt','trt_head']
    for round_index,order in enumerate((cases,list(reversed(cases)))):
        for case in order:
            name=f'timing_r{round_index}_{case}'
            args=[ROOT/'scripts/evaluation/benchmark_bevfusion_portfolio.py',
                '--inputs','/results/timing_inputs.json','--config',config,
                '--checkpoint',inventory['checkpoints']['e12']['path'],
                '--sensor',ROOT/'configs/sensors/Sensor_competition_local.json',
                '--output','/results/'+name,'--case',case,'--samples','96','--repeats','2',
                '--engine',engine['engine_path'],'--metadata',engine['metadata_path']]
            rc=run(name,args)
            if rc:
                failed=out/name/'FAILED.json'
                if not failed.exists() or not json.loads(failed.read_text()).get('oom'):
                    raise RuntimeError('Timing case failed (not a recorded OOM): '+name)
    if run('summary',[ROOT/'scripts/evaluation/summarize_bevfusion_portfolio_eval.py','--output','/results']):
        raise RuntimeError('Summary failed')
    # Only our generated outputs; never chown raw data or the repository.
    subprocess.run(['docker','run','--rm','--network','none','-v',f'{out}:/results',
        '--entrypoint','chown',IMAGE,'-R',f'{os.getuid()}:{os.getgid()}','/results'],check=True)
    (out/'QUEUE_COMPLETE.json').write_text(json.dumps(dict(complete=True,tasks=len(history)),indent=2)+'\n')


if __name__=='__main__':main()
