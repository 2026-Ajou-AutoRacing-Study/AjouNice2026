#!/usr/bin/env python3
"""Summarize paired 30/40m service trials without conflating stress/population."""
import argparse
import json
from pathlib import Path

from summarize_base5_qualification import stats, summarize_inference


def compare(folder):
    results = []
    expected_ids = None
    expected_parameters = None
    source_hash = None
    for name in ('r1_y30', 'r2_y40', 'r3_y40', 'r4_y30'):
        report = json.loads((folder / (name + '.json')).read_text())
        if len(report['cases']) != 1 or report['codec'] != 'jpeg90':
            raise ValueError('Expected one JPEG90 case per trial')
        case = report['cases'][0]
        if not case.get('valid') or case.get('stage_instrumented'):
            raise ValueError('Expected successful uninstrumented trials')
        if case['shape'] != '512x1056' or case['mode'] != 'separated':
            raise ValueError('Image dimensions or pooling mode changed')
        ids = [row['id'] for row in case['per_sample']]
        if expected_ids is None:
            expected_ids = ids
            expected_parameters = case['parameter_sha256']
            source_hash = report['source']['source_sha256']
        if (ids != expected_ids or case['parameter_sha256'] != expected_parameters
                or source_hash != report['source']['source_sha256']):
            raise ValueError('Inputs/order/model parameters are not controlled')
        expected_roi = [-100., -report['lateral_range'], -3.,
                        100., report['lateral_range'], 2.]
        expected_nx = [672, 216 if report['lateral_range'] == 30 else 272, 1]
        if (case['semantic_roi'] != expected_roi or case['voxel'] != [.075, .075, .2]
                or case['depth'] != [1., 100., .5] or case['camera_bev'][2] != .3
                or case['camera_bev_ybound'][2] != .3 or case['view_nx'] != expected_nx):
            raise ValueError('Metric spacing or semantic range changed unexpectedly')
        results.append(dict(name=name, lateral=report['lateral_range'],
                            report=report, case=case,
                            individual=summarize_inference(report)[0]))
    groups = {item['id']: item['sample_group']
              for item in results[0]['report']['source']['samples']}
    summary = {}
    for lateral in (30, 40):
        runs = [r for r in results if r['lateral'] == lateral]
        rows = [row for r in runs for row in r['case']['per_sample']]
        grouped = {}
        for group in ('representative', 'stress'):
            subset = [row for row in rows if groups[row['id']] == group]
            grouped[group] = {key: stats([row[key] for row in subset])
                              for key in ('prepare_ms', 'model_ms', 'service_ms',
                                          'retained_points')}
            grouped[group]['unique_scenes'] = len({row['id'] for row in subset})
            grouped[group]['service_above_13hz_budget'] = sum(
                row['service_ms'] > 1000/13 for row in subset)
        summary[str(lateral)] = dict(groups=grouped,
            steady_peak_allocated_mib=[r['case']['steady_peak_allocated_mib'] for r in runs],
            sparse_shape=runs[0]['case']['sparse_shape'],
            camera_bev_ybound=runs[0]['case']['camera_bev_ybound'])
    return dict(summary=summary, distinct_scenes=len(set(expected_ids)),
                measured_inferences=sum(len(r['case']['per_sample']) for r in results),
                parameter_sha256=expected_parameters, source_sha256=source_hash,
                order=[r['name'] for r in results],
                trials=[dict(name=r['name'], lateral=r['lateral'],
                             **r['individual']) for r in results],
                scope='Cost-only random four-class weights; no learned AP, training, ROS or full stack')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.folder)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: value for key, value in result.items() if key != 'trials'}, indent=2))


if __name__ == '__main__':
    main()
