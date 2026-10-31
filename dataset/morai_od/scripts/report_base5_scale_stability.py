#!/usr/bin/env python3
"""Summarize first-attempt scale failures and retry cost without survivorship bias.

Use raw attempt ledgers, not selected-success ledgers. Failure causation is never
deduced merely from nonunit scale or a size-guard reason. An optional explicit,
pinned cause review may classify verified scale causes separately.
"""
import argparse
from collections import Counter,defaultdict
import json
import math
from pathlib import Path
import statistics

from base5_capture_safety import atomic_json,sha256


CAUSES={'confirmed_scale','not_scale','unknown','infrastructure','safety'}


def timing(rows):
    values=[r['duration_sec'] for r in rows if type(r.get('duration_sec')) in (int,float)
            and math.isfinite(r['duration_sec']) and r['duration_sec']>=0]
    values.sort()
    return dict(observed_durations=len(values),missing_durations=len(rows)-len(values),
        sum_sec=sum(values) if values else None,
        mean_sec=statistics.mean(values) if values else None,
        median_sec=statistics.median(values) if values else None,
        p95_sec=values[max(0,math.ceil(.95*len(values))-1)] if values else None,
        scope='Per-ledger duration only; process startup, operator, stopped sessions and omitted attempts may be outside this field')


def rate(failed,total):
    return dict(failures=failed,observed_initial_scenes=total,
        fraction=failed/total if total else None,
        one_sided_95_upper_if_zero=(1-.05**(1/total)) if total and failed==0 else None,
        independence_limitation='Independent source geometries required; repeat captures and parents within one scene are not independent trials',
        sampling_limitation='Unweighted conditional sample; purposeful profile/context coverage is not an estimate of the weighted full-5K production failure probability')


def load_reviews(path,pins):
    result={}
    if path is None:
        return result
    path=Path(path).resolve(strict=True)
    pins[str(path)]=sha256(path)
    doc=json.loads(path.read_text())
    for row in doc.get('attempts',[]):
        key=(row['id'],row['attempt_ordinal'])
        if (key in result or type(row['attempt_ordinal']) is not int or row['attempt_ordinal']<0
                or row['classification'] not in CAUSES or not isinstance(row.get('reason'),str) or not row['reason']):
            raise ValueError('Explicit unique attempt cause classification and reason required')
        if row['classification']=='confirmed_scale' and not row.get('evidence'):
            raise ValueError('Confirmed scale causation requires pinned independent evidence')
        for evidence in row.get('evidence',[]):
            source=Path(evidence['path']).resolve(strict=True)
            if sha256(source)!=evidence['sha256']:
                raise ValueError('Cause-review evidence changed')
            pins[str(source)]=evidence['sha256']
        result[key]=row
    return result


def report(manifest,ledgers,output,*,cause_review=None):
    source=Path(manifest).resolve(strict=True)
    output=Path(output).resolve()
    if output.exists():
        raise FileExistsError('Fresh report output required')
    paths=[Path(path).resolve(strict=True) for path in ledgers]
    if len(set(paths))!=len(paths) or not paths:
        raise ValueError('Unique nonempty raw attempt ledgers required')
    deck=json.loads(source.read_text())
    entries={entry['id']:entry for entry in deck['scenes']}
    if len(entries)!=len(deck['scenes']):
        raise ValueError('Duplicate source scene identifiers')
    pins={str(path):sha256(path) for path in [source]+paths}
    metas={}
    for ident,entry in entries.items():
        mp=Path(entry['metadata']).resolve(strict=True)
        pins[str(mp)]=sha256(mp)
        metas[ident]=json.loads(mp.read_text())
    reviewed=load_reviews(cause_review,pins)
    attempts=defaultdict(list)
    capture_roots=set()
    for path in paths:
        for line in path.read_text().splitlines():
            if not line.strip():
                continue
            row=json.loads(line)
            ident=row['id']
            if ident not in entries or row.get('scenario_sha256')!=entries[ident]['scenario_sha256']:
                raise ValueError('Foreign or changed scenario in raw attempt ledger')
            root=row.get('capture_root')
            if root:
                root=Path(root).resolve()
                if root in capture_roots:
                    raise ValueError('Duplicate captured attempt across ledgers; do not mix selected and raw ledgers')
                capture_roots.add(root)
                if output==root or root in output.parents:
                    raise ValueError('Report cannot be written inside preserved native captures')
            ordinal=len(attempts[ident])
            cause=reviewed.get((ident,ordinal),{}).get('classification','unknown')
            attempts[ident].append(dict(row,attempt_ordinal=ordinal,cause_classification=cause,
                source_ledger=str(path)))
    if set(reviewed)-{(ident,index) for ident,rows in attempts.items() for index in range(len(rows))}:
        raise ValueError('Cause review references an unobserved attempt')
    initial=[rows[0] for rows in attempts.values()]
    all_rows=[row for rows in attempts.values() for row in rows]
    retries=[row for rows in attempts.values() for row in rows[1:]]
    def condition(ident):
        meta=metas[ident]
        return (meta.get('scale_stability') or meta.get('scale_probe') or {}).get('condition','unclassified')
    def initial_rate(rows):
        result=rate(sum(row.get('status')!='accepted_candidate' for row in rows),len(rows))
        result['unique_scenario_payloads']=len({row['scenario_sha256'] for row in rows})
        geometry_keys={metas[row['id']].get('scale_stability',{}).get('independent_geometry_unit')
                       or row['scenario_sha256'] for row in rows}
        result['distinct_source_geometry_units']=len(geometry_keys)
        if result['unique_scenario_payloads']<len(rows) or len(geometry_keys)<len(rows):
            result['one_sided_95_upper_if_zero']=None
            result['ci_withheld_reason']='Repeated payloads or matched conditions share source geometry; no independent-trial CI'
        return result
    groups={value:[row for row in initial if condition(row['id'])==value]
            for value in sorted({condition(ident) for ident in entries})}
    failure=lambda row:row.get('status')!='accepted_candidate'
    scaled=groups.get('scaled',[])
    confirmed=[row for row in scaled if failure(row) and row['cause_classification']=='confirmed_scale']
    confirmed_rate=rate(len(confirmed),len(scaled))
    if (any(failure(row) and row['cause_classification']=='unknown' for row in scaled)
            or initial_rate(scaled)['one_sided_95_upper_if_zero'] is None):
        confirmed_rate['one_sided_95_upper_if_zero']=None
    by_group={name:dict(initial_rate(rows),
        status_counts=dict(Counter(row.get('status','missing') for row in rows)),timing=timing(rows))
        for name,rows in groups.items()}
    pair_results=[]
    paired=defaultdict(dict)
    for ident,meta in metas.items():
        registered=meta.get('scale_stability',{})
        if registered.get('pair_id'):
            paired[registered['pair_id']][registered['condition']]=ident
    for pair,values in sorted(paired.items()):
        available=all(ident in attempts for ident in values.values()) and set(values)=={'original','scaled'}
        pair_results.append(dict(pair_id=pair,source_ids=values,both_initial_observed=available,
            initial_status={name:attempts[ident][0]['status'] if ident in attempts else None
                            for name,ident in values.items()},
            difference_scale_causation_proven=False))
    payload=dict(schema='base5_scale_stability_report_v1',source_manifest=str(source),source_pins=pins,
        registered_scenes=len(entries),observed_initial_scenes=len(initial),
        unobserved_registered_ids=sorted(set(entries)-set(attempts)),
        initial_all_failure_rate=initial_rate(initial),
        initial_by_condition=by_group,
        confirmed_scale_failure_rate=dict(confirmed_rate,
            unknown_or_other_scaled_failures=sum(failure(r) for r in scaled)-len(confirmed),
            zero_confirmed_does_not_bound_unknown_scale_causes=True),
        initial_failure_cause_counts=dict(Counter(r['cause_classification'] for r in initial if failure(r))),
        first_attempt_timing=timing(initial),retry_timing=timing(retries),all_attempt_timing=timing(all_rows),
        total_attempts=len(all_rows),retry_attempts=len(retries),
        latest_status_counts=dict(Counter(rows[-1].get('status','missing') for rows in attempts.values())),
        review_reason_counts=dict(Counter(reason for row in all_rows
            for reason in row.get('audit',{}).get('review_reasons',[]))),
        pair_results=pair_results,
        registered_conditions={ident:meta.get('scale_stability',meta.get('redbarrel_warm_probe',meta.get('scale_probe',{})))
                               for ident,meta in metas.items()},
        attempts=all_rows,raw_modified=False,ap_or_training_claim=False,
        chronological_limitation='Attempts within a scene follow supplied raw-ledger order; pass ledgers in actual session order. Process state is not inferred from order.',
        denominator_contract='Initial attempts use observed scene IDs, not accepted outputs, object parents or retry counts. Repeated payloads must be excluded from independent-geometry CI interpretation.')
    if any(sha256(path)!=checksum for path,checksum in pins.items()):
        raise ValueError('Source evidence changed during read-only report')
    atomic_json(output,payload)
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--ledger',type=Path,action='append',required=True)
    parser.add_argument('--cause-review',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(report(args.manifest,args.ledger,args.output,cause_review=args.cause_review))
