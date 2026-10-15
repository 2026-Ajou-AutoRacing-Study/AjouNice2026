#!/usr/bin/env python3
"""Evaluate only the frozen dev-selected candidate on preregistered regression bags."""
import argparse
import gzip
import json
from pathlib import Path

from experiment_base5_map_cv_candidates import ROOT, SPEC, digest, evaluate, gates, prepare_network


def main():
    from perception_object_pipeline.map_based_prediction import LaneletNetwork
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--development',type=Path,required=True)
    parser.add_argument('--pack',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('Fresh report required')
    dev=json.loads((args.development/'development.json').read_text())
    prereg=json.loads((args.development/'preregistered.json').read_text())
    if prereg['spec']!=json.loads(json.dumps(SPEC)):
        raise ValueError('Preregistered experiment spec changed')
    candidate_source=ROOT/'perception/detection3d/bevfusion/tools/experiment_base5_map_cv_candidates.py'
    if digest(candidate_source)!=prereg['source_sha256']:
        raise ValueError('Candidate source changed after preregistration')
    candidate=dev['selected_for_reserved']
    if candidate is None:raise ValueError('No dev-qualified candidate; do not select using regression metrics')
    data=json.load(gzip.open(args.pack,'rt'))
    if data['preregistration_sha256']!=digest(args.development/'preregistered.json'):
        raise ValueError('Reserved pack belongs to another preregistration')
    allowed={str((ROOT/p).resolve()) for p in prereg['reserved_bags']}
    actual={v['provenance']['path'] for v in data['cases'].values()}
    if not actual<=allowed:raise ValueError('Pack contains an unregistered bag')
    if data['catalog_sha256']!=digest(ROOT/'dataset/morai_od/configs/morai_asset_bbox_offsets_status_size_v2.json'):
        raise ValueError('Catalog changed since packing')
    network=LaneletNetwork.load(ROOT/'kcity_map/lanelet_map/lanelet2_map_v85.osm',ROOT/'kcity_map/lanelet_map/map_projector_info_v1.yaml')
    prepare_network(network)
    results={}
    for case,packed in data['cases'].items():
        gt_frames={}
        for frame in packed['gt_frames']:
            gt_frames[(case,round(frame['stamp'],6))]=[dict(o,key=tuple(o['key'])) for o in frame['objects']]
        gt_series={tuple(row['key']):row['series'] for row in packed['gt_trajectories']}
        results[case]=evaluate(network,packed['frames'],(gt_frames,gt_series),case,('baseline',candidate))
        results[case]['provenance']=packed['provenance']
        print(case,{v:{h:m['top1'] for h,m in result.items()} for v,result in results[case]['metrics'].items()},flush=True)
    acceptance=gates(results,candidate)
    report=dict(candidate=candidate,spec=SPEC,results=results,acceptance=acceptance,
                all_preregistered_cases_present=actual==allowed,
                pack_sha256=digest(args.pack),development_sha256=digest(args.development/'development.json'),
                source_sha256=digest(__file__),candidate_source_sha256=digest(ROOT/'perception/detection3d/bevfusion/tools/experiment_base5_map_cv_candidates.py'),
                production_enabled=False,
                caveat='Frozen choice, no retuning after these metrics. Historical recordings not a globally untouched test. '
                       'Different old detectors/tracker inputs and conditional catalog GT support; no E24 whole-stack safety claim.')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Reserved gate pass:',acceptance['pass_all'],flush=True)


if __name__=='__main__':main()
