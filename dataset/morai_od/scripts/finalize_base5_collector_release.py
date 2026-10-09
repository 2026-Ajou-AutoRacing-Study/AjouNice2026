#!/usr/bin/env python3
"""Freeze qualification evidence and runtime entrypoint, never start bulk capture."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
import shutil

from base5_capture_safety import atomic_json,sha256
from build_base5_bevfusion_infos import convert
from merge_base5_environment_infos import merge
from report_base5_postcollection import timing_report
from report_base5_prebulk_cost import report as cost_report
import generate_base5_independent as gen


def read(path):return json.loads(Path(path).read_text())


def summarize_plan(manifest,pools):
    deck=read(manifest);counts=Counter();families=Counter();assets=Counter()
    for e in deck['scenes']:
        m=read(e['metadata']);families[m['spec']['family']+(':mission' if m['spec']['mission'] else ':generic')]+=1
        for a in m['runtime_actors']:counts[pools[a['data_id']]]+=1;assets[a['name']]+=1
    return dict(frames=len(deck['scenes']),families=dict(families),planned_parent_exposure=dict(counts),
        planned_asset_exposure=dict(assets),manifest_sha256=sha256(manifest),
        meaning='Placed parent actors, NOT visible component GT or final captured counts')


def finalize(root,sunny13,output):
    if output.exists():raise FileExistsError('Fresh release proof required')
    summary=read(root/'qualification_comparison.json');live=read(root/'qualification_live_results.json')
    static=read(root/'fresh_geometry_static64/status.json')
    motion=read(root/'fresh_geometry_animated8/live_motion_validation.json')
    completion=read(root/'completion_negative_control/live_validation.json')
    if not motion['qualified']:raise ValueError('New moving geometry qualification not complete')
    if not summary.get('longrun_and_animated_gate') or not summary.get('static64_speed_gate'):
        raise ValueError('Runtime quality/time qualification failed; no automatic promotion')
    if static['stage']!='complete' or static['accepted_candidate']!=64 or static['review']:
        raise ValueError('Fresh-geometry pilot not completely accepted')
    resumes=[r for r in live['cases'] if r.get('completed_resume_additional_raw')==0]
    if len(resumes)<3:raise ValueError('Missing completed resume evidence')
    suite=Path(live['suite']);audit=read(suite.parent/'suite_audit.json')
    if audit['fresh_scenes']!=25000 or audit['unique_tokens']!=25000:raise ValueError('Incomplete independent environment suite')
    if audit['physical_anchor_link_split_conflicts']:raise ValueError('Spatial anchor split conflicts')
    output.mkdir(parents=True)
    repo=Path(__file__).resolve().parents[3];scripts=Path(__file__).parent
    files=['launch_base5_fresh.py','run_base5_fresh_batch.py','run_base5_fresh_completion.py',
        'run_base5_batch_optimized.py','run_base5_capture_optimized.py','run_base5_capture_campaign_spatial.py',
        'base5_spatial_diversity.py','prepare_base5_fresh_environment.py','prepare_base5_fresh_suite.py',
        'generate_base5_independent.py','run_base5_capture.py','run_base5_batch_campaign.py',
        'run_base5_capture_campaign.py','run_base5_adaptive_completion.py','run_base5_flexible_completion.py']
    hashes={str(scripts/n):sha256(scripts/n) for n in files}
    ledger=sunny13/'selected_ledger.jsonl'
    if sha256(ledger)!='026e6244aa311f6342429a30893694ce646b49222ab354f97cfa08539df74a43':raise ValueError('Original Sunny13 selection changed')
    pilot_infos=output/'fresh_geometry_cpu_infos'
    converted=convert(root/'fresh_geometry_static64/selected_ledger.jsonl',pilot_infos,
        repo/'configs/sensors/Sensor_competition_local.json',environment_tag='qualification_sunny13')
    if sum(converted['splits'].values())!=64 or converted['excluded_review_scenes']:raise ValueError('Fresh pilot conversion dropped approved rows')
    # Test interface compatibility only: never use validation pilot in training.
    merge_proof=merge([sunny13/'cpu_infos_final',pilot_infos],output/'merge_compatibility_only')
    if merge_proof['samples']!=5064 or merge_proof['group_split_leakage']:raise ValueError('Fresh identity/old infos merge incompatible')
    cost=cost_report(sunny13/'adopted_manifest.json',
        [root/'combined_longrun200',root/'animated_regression8'],detailed_groups=True)
    atomic_json(output/'cost_estimate.json',cost)
    stages={name:timing_report([root/'runtime_benchmark'/name]) for name in ('baseline_a2','baseline_a3','combined_f1','combined_f2')}
    atomic_json(output/'stage_comparison.json',stages)
    ctx=gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR,gen.geo.DEFAULT_MGEO)
    pools={a['data_id']:p for p in ('vehicle','static_vehicle','obstacle','person','dummy','animal','pet_person','stroller','rider') for a in ctx['pools'][p]}
    plans={};total=Counter()
    for env in read(suite)['environments']:
        # Sunny13 actually changed 26 cells; use its adopted manifest, not the
        # original exact slot list, when aggregating the 30K planned snapshot.
        manifest=sunny13/'adopted_manifest.json' if env['environment']=='sunny13' else Path(env['manifest'])
        plans[env['environment']]=summarize_plan(manifest,pools);total.update(plans[env['environment']]['planned_parent_exposure'])
    atomic_json(output/'planned_30k_exposure.json',dict(environments=plans,total_planned_parent_exposure=dict(total),
        policy='Five newly seeded 5K plans plus adopted Sunny13 plan; exposure != visible GT'))
    free=shutil.disk_usage(root).free/2**30;estimate=cost['extrapolated_gib_including_qa_and_rejected_group_weighted']
    doc=dict(schema=1,collector_qualified=True,suite=str(suite),suite_sha256=sha256(suite),
        runtime_hashes=hashes,recommended_chunk_size=16,recommended_setup_profile='static-ready',recommended_runtime_overlap=False,
        sunny13_preserved=str(sunny13),sunny13_ledger_sha256=sha256(ledger),
        static64_reduction_fraction=summary['reduction_fraction'],fresh_pilot_accepted=64,
        bounded_completion_policy='Three total fresh proposals per missing slot; 1800s per invocation; finite resume; native QA unchanged',
        completion_live_qualified=completion['parent_review']>0 and completion['attempts']>0 and completion['parent_preserved'] and completion['additional_completed_resume_raw']==0,
        estimated_5k_hours=cost['extrapolated_hours_group_weighted'],estimated_5k_gib=estimate,
        disk_free_gib_at_freeze=free,estimated_remaining_25k_gib=5*estimate if estimate else None,
        estimated_free_after_remaining_25k_gib=free-5*estimate if estimate else None,
        bulk_started=False,new_environment_capture_started=False,operator_UI_confirmation_required=True,
        limitations=['Static pilot causality is not a guaranteed full-5K speedup.',
            'Fresh geometry pilot used CURRENT Sunny13, not true sunny11/foggy captures.',
            'Continuous road anchors do not make map background and 100m views geographically disjoint.',
            'CPU infos and rich-cue stats use provisional crop; resolve crop before final learning infos.',
            'Rejected early service-probe overlap is NOT enabled. Confirmed EventCmd/order and per-scene network remain.',
            'New generic completion tested with static negative controls; animated native preparation tested separately, source gait/count policy unit-tested.'])
    atomic_json(output/'release.json',doc);return doc


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True)
    p.add_argument('--sunny13-collection',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(json.dumps(finalize(a.root,a.sunny13_collection,a.output),ensure_ascii=False,indent=2))
