#!/usr/bin/env python3
"""Serial bounded collector qualification; fresh 25K generation, NOT bulk capture."""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys
import time

from base5_capture_safety import atomic_json,sha256
from prepare_base5_runtime_benchmark import prepare as register
from run_base5_motion_contract_repair import motion_matches


def child(command,output,timeout=1800):
    output.parent.mkdir(parents=True,exist_ok=True)
    with (output.parent/(output.name+'.supervisor.log')).open('a') as log:
        proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:return proc.wait(timeout=timeout)
        except (subprocess.TimeoutExpired,KeyboardInterrupt):
            # Ask the supervised parent to stop safely, rather than kill it and
            # leave its separate ROS worker running in the background.
            atomic_json(output/'STOP',dict(owner='qualification_supervisor',reason='bounded stop'))
            try:proc.wait(timeout=60)
            except subprocess.TimeoutExpired:
                from run_base5_batch_campaign import stop_child
                stop_child(proc,output)
            raise RuntimeError('Qualification interrupted/timed out; inspect Park before any new load')


def capture(root,name,manifest,script,chunk=4,limit=64,extra=()):
    output=root/name
    cmd=[sys.executable,str(Path(__file__).with_name(script)),'--manifest',str(manifest),
        '--output',str(output),'--chunk-size',str(chunk),'--limit',str(limit),*extra]
    if limit>64:cmd.append('--approve-bulk-capture')  # bounded200 regression, not 25K collection
    if output.exists():cmd.append('--resume')
    start=time.monotonic();code=child(cmd,output)
    status=json.loads((output/'status.json').read_text())
    result=dict(name=name,stage=status['stage'],accepted=status['accepted_candidate'],review=status['review'],
        parent_active_sec=status['elapsed_sec'],invocation_wall_sec=time.monotonic()-start,limit=limit,chunk=chunk)
    if code or status['stage'] not in ('complete','complete_with_deficit','paused'):
        raise RuntimeError('Unsafe/halted experiment '+name+': '+str(status.get('error')))
    print(result,flush=True);return result


def verify_resume(root,name,manifest,script,chunk,limit=64,extra=()):
    output=root/name;ledger=output/'selected_ledger.jsonl';before=sha256(ledger)
    files={str(p):p.stat().st_size for p in output.rglob('*_Intensity.bin')}
    result=capture(root,name,manifest,script,chunk,limit,extra)
    if sha256(ledger)!=before or {str(p):p.stat().st_size for p in output.rglob('*_Intensity.bin')}!=files:
        raise ValueError('Completed resume modified selection or added capture')
    result['completed_resume_additional_raw']=0
    return result


def run(a):
    root=a.root;bench=root/'runtime_benchmark';deadline=time.monotonic()+1800
    while not (bench/'comparison.json').exists():
        if time.monotonic()>deadline:raise RuntimeError('Prior runtime comparison did not finish; no new MR work')
        time.sleep(2)
    old=json.loads((bench/'comparison.json').read_text())
    if not old['all_64_approved']:raise ValueError('Prior comparison has quality deficits; inspect first')
    manifest=bench/'static64/manifest.json';results=[]
    for name,script,chunk in [('combined_f1','run_base5_batch_optimized.py',16),
                              ('combined_f2','run_base5_batch_optimized.py',16),
                              ('baseline_a3','run_base5_batch_optimized.py',4)]:
        extra=('--setup-profile','baseline' if name=='baseline_a3' else 'static-ready')
        results.append(capture(bench,name,manifest,script,chunk,extra=extra))
        atomic_json(root/'qualification_progress.json',results)
    clean_baseline=json.loads((bench/'baseline_a2/status.json').read_text())['elapsed_sec']
    baseline=(clean_baseline+results[-1]['parent_active_sec'])/2
    combined=(results[0]['parent_active_sec']+results[1]['parent_active_sec'])/2
    passed=all(r['accepted']==64 and r['review']==0 for r in results) and combined<.8*baseline
    summary=dict(cases=results,clean_baseline_mean_sec=baseline,overlap_combined_mean_sec=combined,
        reduction_fraction=1-combined/baseline,static64_speed_gate=passed,bulk_promoted=False,
        runtime_overlap=False,overlap_candidate_rejected='Early probe can acknowledge previous SceneNetwork service; e1 failed service unavailable, Park verified. No bypass or unsafe resume.')
    atomic_json(root/'qualification_comparison.json',summary)
    if passed:
        registration=root/'longrun_registration'
        if not registration.exists():register(a.sunny13_collection/'adopted_manifest.json',registration,static_count=200,moving_count=8)
        results.append(capture(root,'combined_longrun200',registration/'static64/manifest.json',
            'run_base5_batch_optimized.py',16,200,('--setup-profile','static-ready')))
        results.append(verify_resume(root,'combined_longrun200',registration/'static64/manifest.json',
            'run_base5_batch_optimized.py',16,200,('--setup-profile','static-ready')))
        passed=results[-1]['accepted']==200 and results[-1]['review']==0
    # Animated setup is deliberately unchanged; regression checks native QA.
    results.append(capture(root,'animated_regression8',bench/'animated8/manifest.json',
        'run_base5_batch_optimized.py',4,8,('--setup-profile','static-ready')))
    refs={e['id']:json.loads(Path(e['metadata']).read_text())
        for e in json.loads((bench/'animated8/manifest.json').read_text())['scenes']}
    motion_failures=[]
    for line in (root/'animated_regression8/selected_ledger.jsonl').read_text().splitlines():
        row=json.loads(line);actual=json.loads((Path(row['capture_root'])/'metadata.json').read_text())
        if not motion_matches(refs[row['source_cell_id']],actual):motion_failures.append(row['source_cell_id'])
    passed=passed and results[-1]['accepted']==8 and not results[-1]['review'] and not motion_failures
    summary.update(longrun_and_animated_gate=passed,animated_motion_failures=motion_failures)
    atomic_json(root/'qualification_comparison.json',summary)
    results.append(verify_resume(bench,'combined_f2',manifest,'run_base5_batch_optimized.py',16,extra=('--setup-profile','static-ready')))
    atomic_json(root/'qualification_live_results.json',dict(cases=results,bulk_started=False))
    # Never run CPU-heavy full generation concurrently with timing comparisons.
    suite=root/'fresh_suite_final_v141'
    if not (suite/'suite.json').exists():
        code=child([sys.executable,str(Path(__file__).with_name('prepare_base5_fresh_suite.py')),
            '--manifest',str(a.manifest),'--sunny13-collection',str(a.sunny13_collection),
            '--output',str(suite)],suite,timeout=1800)
        if code:raise ValueError('Fresh suite generation failed; keep evidence, no bulk capture')
    # Qualification geometry seeded for sunny11 is captured under the CURRENT
    # Sunny13 UI, not labelled as a real sunny11 dataset or imported into 30K.
    sample=root/'fresh_geometry_registration'
    if not sample.exists():register(suite/'sunny11/manifest.json',sample)
    pilot=sample/'static64/manifest.json';deck=json.loads(pilot.read_text())
    deck['config']['collection_environment']=dict(weather='sunny',hour=13,policy='Validation only under current Sunny13 UI; not sunny11 training data')
    deck['config']['environment_tag']='qualification_sunny13'
    qualified=sample/'validation_manifest.json'
    if not qualified.exists():atomic_json(qualified,deck)
    profile='static-ready' if passed else 'baseline'
    fresh_extra=('--setup-profile',profile)
    results.append(capture(root,'fresh_geometry_static64',qualified,'run_base5_fresh_batch.py',
        4,64,(*fresh_extra,'--stop-after-chunks','1')))
    results.append(capture(root,'fresh_geometry_static64',qualified,'run_base5_fresh_batch.py',
        4,64,fresh_extra))
    results.append(verify_resume(root,'fresh_geometry_static64',qualified,'run_base5_fresh_batch.py',
        4,64,fresh_extra))
    atomic_json(root/'qualification_live_results.json',dict(cases=results,bulk_started=False,
        suite=str(suite/'suite.json'),new_environment_capture_started=False,
        limitation='Fresh geometry pilot is current Sunny13 only. Other weather UI and 25K collection remain operator/user actions.'))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--sunny13-collection',type=Path,required=True)
    run(p.parse_args())
