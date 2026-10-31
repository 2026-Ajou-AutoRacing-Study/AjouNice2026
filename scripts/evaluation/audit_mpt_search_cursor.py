#!/usr/bin/env python3
"""Deterministic scheduling counterexample, NOT a solver/driving performance test.

Models only first-mask choice, budgeted enumeration, cursor advancement and an
externally supplied feasibility oracle. Constraint, collision and fallback logic
are deliberately not replaced or relaxed. The patch is kept outside runtime.
"""
import argparse
import hashlib
import json
from pathlib import Path


def first_mask(cursor, mask_count, prefer_previous_seed, previous_mask,
               have_validated_incumbent, candidate):
    use_hint=prefer_previous_seed and (have_validated_incumbent if candidate else True)
    return previous_mask%mask_count if use_hint else cursor%mask_count


def budgeted_search(*, mask_count, preferred_mask, feasible_masks, budget_masks,
                    callbacks, candidate, incumbent=False, previous_seed=True):
    cursor=0;trace=[]
    for cycle in range(callbacks):
        start=first_mask(cursor,mask_count,previous_seed,preferred_mask,incumbent,candidate)
        examined=[];winner=None
        for order in range(min(mask_count,budget_masks)):
            mask=(start+order)%mask_count
            cursor=(mask+1)%mask_count
            examined.append(mask)
            if mask in feasible_masks:
                winner=mask
                break
        trace.append(dict(cycle=cycle,examined=examined,next_cursor=cursor,winner=winner))
        if winner is not None:break
    return trace


def source_contract(source):
    body=source.split('PathOptimizer::optimizeObstacleCandidates(',1)[1]
    start=body.index('size_t preferred_mask = obstacle_search_cursor_ % mask_count;')
    finish=body.index('const size_t first_mask = preferred_mask;',start)
    block=body[start:finish]
    return dict(has_saved_cursor=True,
        unconditional_previous_seed_override='if (prefer_previous_seed) {' in block,
        candidate_incumbent_gated_override='if (prefer_previous_seed && have_validated_incumbent) {' in block,
        cursor_advanced='obstacle_search_cursor_ = (mask + 1) % mask_count;' in body,
        first_mask_selection=block)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    root=Path(__file__).resolve().parents[2]
    parser.add_argument('--source',type=Path,default=root/'planning_ws/src/autoware_path_optimizer/src/node.cpp')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('Fresh output required')
    text=args.source.read_text()
    cases=[]
    for count,budget,feasible in [(2,1,{1}),(4,1,{3}),(8,2,{6})]:
        params=dict(mask_count=count,preferred_mask=0,feasible_masks=feasible,
                    budget_masks=budget,callbacks=16)
        cases.append(dict(mask_count=count,budget_masks=budget,feasible_masks=sorted(feasible),
            pre_fix_model=budgeted_search(**params,candidate=False),
            candidate_model=budgeted_search(**params,candidate=True)))
    result=dict(schema='mpt_cursor_scheduling_counterexample_v1',source=str(args.source),
        source_sha256=hashlib.sha256(text.encode()).hexdigest(),
        source_contract=source_contract(text),cases=cases,
        limits=['Scheduling model only; not exact solver/path replay.',
                'The feasible-mask oracle is a deterministic fixture, not bag-derived feasibility.',
                'No proof that this mechanism caused either recorded optimizer failure.',
                'Runtime source, collision checks and fallback were not changed by this audit.'])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(dict(output=str(args.output),contract=result['source_contract'])))


if __name__=='__main__':main()
