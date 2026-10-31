#!/usr/bin/env python3
"""Independently audit saved E12 motion ablation rows; no inference or ROS.

Reuses the previously audited input-coordinate/GT contract, NOT the new motion
analyzer's predictors or metrics. Analytic CV/CTRV and metric reductions below
are separate implementations. Existing outputs are never overwritten.
"""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from analyze_e12_map_horizon import prepare_case, future_reference, key_tuple, current_assignment

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'artifacts/evaluation/bevfusion/e12_postprocessing_261023_v1'


def read(path):
    with (gzip.open if str(path).endswith('.gz') else open)(path, 'rt') as stream:
        return json.load(stream)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(8 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def key(row):
    return row['case'], float(row['stamp']), str(row['id']), key_tuple(row['gt_key'])


def analytic_paths(obj):
    times = np.arange(1, 31, dtype=float) / 10.
    center = np.asarray(obj['center'][:2])
    cv = center + times[:, None] * np.asarray(obj['world_velocity'])[None]
    yaw, omega, speed = obj['yaw'], obj['yaw_rate'], obj['body_velocity'][0]
    if abs(omega) < .001:
        ctrv = center + times[:, None] * [speed * math.cos(yaw), speed * math.sin(yaw)]
    else:
        theta = yaw + times * omega
        ctrv = center + np.column_stack((speed / omega * (np.sin(theta) - math.sin(yaw)),
            speed / omega * (math.cos(yaw) - np.cos(theta))))
    return {'cv': cv, 'ctrv': ctrv}


def distribution(values):
    array = np.asarray(values, dtype=float)
    if not len(array):
        return {'n': 0}
    return dict(n=len(array), mean=float(array.mean()), p50=float(np.median(array)),
                p90=float(np.percentile(array, 90)), p95=float(np.percentile(array, 95)))


def check_close(first, second, note, tolerance=1.e-8):
    a, b = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
    if a.shape != b.shape or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError('Invalid comparison shape/finite data: ' + note)
    error = float(np.max(np.abs(a - b))) if a.size else 0.
    if error > tolerance:
        raise ValueError(f'{note}: error {error:g} > {tolerance:g}')
    return error


def audit(directory, output):
    pack_path = BASE / 'tracker_v1/replay_pack.json.gz'
    frozen_path = BASE / 'map_horizon/map_horizon_v4/rows.json.gz'
    transform_path = ROOT / 'configs/tf/morai_lanelet_static_tf.yaml'
    row_path, report_path = directory / 'rows.json.gz', directory / 'report.json'
    complete_path = directory / 'COMPLETE.json'
    paths = [pack_path, frozen_path, transform_path, row_path, report_path, complete_path, Path(__file__)]
    pins = {str(path): sha(path) for path in paths}
    pack, frozen, rows = read(pack_path), read(frozen_path), read(row_path)
    report = read(report_path)
    receipt = read(complete_path)
    if receipt.get('complete') is not True or receipt['report_sha256'] != sha(report_path) or receipt['rows_sha256'] != sha(row_path):
        raise ValueError('Only a completed ablation may be audited')
    for path, expected in report['source_sha256'].items():
        if sha(path) != expected:
            raise ValueError('Analyzer source/input differs from its completion receipt: ' + path)
    transform = yaml.safe_load(transform_path.read_text())['map_to_morai_map']
    frozen_by_key = {(key(row), row['horizon_s']): row for row in frozen
                     if row['output_horizon_s'] == 3}
    expected_keys = {item[0] for item in frozen_by_key if item[1] == 3}
    observed_keys = [key(row) for row in rows]
    if len(set(observed_keys)) != len(observed_keys) or set(observed_keys) != expected_keys:
        raise ValueError('Ablation rows differ from the frozen same-3s cohort')
    normalized, references, matches, gt_samples = {}, {}, set(), {}
    for case_name, case in pack['cases'].items():
        frames, trajectories = prepare_case(case, transform)
        for identity, series in trajectories.items():
            gt_samples[(case_name, identity)] = {t: (np.asarray(p[:2]), index)
                for t, p, index in zip(*series)}
        for frame in frames:
            for obj in frame['objects']:
                normalized[(case_name, frame['stamp'], str(obj['id']))] = obj
            if not frame['scoring_eligible']:
                continue
            for oi, gi, _ in current_assignment(frame['objects'], frame['gt']):
                obj, truth = frame['objects'][oi], frame['gt'][gi]
                gt_key = key_tuple(truth['key'])
                reference, reason = future_reference(trajectories[gt_key], frame['stamp'], 3)
                if reason is None:
                    item = case_name, frame['stamp'], str(obj['id']), gt_key
                    matches.add(item)
                    references[item] = reference
    if matches != expected_keys:
        raise ValueError('Independent origin association/future support differs from frozen cohort')
    maxima = dict(reference=0., cv=0., ctrv=0., map_endpoint=0., map_metric=0.,
                  causal_gt_velocity=0., diagnostic_cv=0., diagnostic_ctrv=0., axis=0., reduced_metric=0.)
    values = {method: [] for method in ('cv', 'ctrv', 'mapcv3_top1')}
    decomposition_samples = causal_origins = 0
    for row in rows:
        item = key(row)
        obj = normalized[item[:3]]
        reference = references[item]
        maxima['reference'] = max(maxima['reference'], check_close(row['reference_xy'], reference, 'future GT'))
        generated = analytic_paths(obj)
        for method in ('cv', 'ctrv'):
            maxima[method] = max(maxima[method], check_close(row['paths'][method], generated[method], method))
        if row.get('gt_causal_velocity') is not None:
            span = row['gt_velocity_window']
            stamps, indices = span['stamps'], span['native_indices']
            elapsed = stamps[-1] - stamps[0]
            if (len(stamps) < 2 or abs(stamps[-1] - row['stamp']) > 1.e-7
                    or not .1 - 1.e-6 <= elapsed <= .25 + 1.e-6
                    or any(not 0 < b-a <= .15 for a, b in zip(stamps, stamps[1:]))
                    or any(b != a+1 for a, b in zip(indices, indices[1:]))):
                raise ValueError('Invalid past-only contiguous GT velocity window')
            samples = gt_samples[(item[0], item[3])]
            if any(t not in samples or samples[t][1] != index for t, index in zip(stamps, indices)):
                raise ValueError('Causal velocity used excluded or unrelated GT')
            velocity = (samples[stamps[-1]][0] - samples[stamps[0]][0]) / elapsed
            maxima['causal_gt_velocity'] = max(maxima['causal_gt_velocity'],
                check_close(velocity, row['gt_causal_velocity'], 'causal GT secant'))
            cosine, sine = math.cos(obj['yaw']), math.sin(obj['yaw'])
            longitudinal = float(velocity @ [cosine, sine])
            lateral = obj['body_velocity'][1]
            oracle = dict(obj, body_velocity=[longitudinal, lateral],
                world_velocity=[cosine*longitudinal-sine*lateral, sine*longitudinal+cosine*lateral])
            for method, predicted in analytic_paths(oracle).items():
                maxima['diagnostic_' + method] = max(maxima['diagnostic_' + method], check_close(
                    row['paths'][method + '_gt_longitudinal'], predicted, 'body-x-only diagnostic ' + method))
            causal_origins += 1
        elif any('gt_longitudinal' in name for name in row['paths']):
            raise ValueError('Diagnostic path exists without a valid causal GT velocity')
        points = np.vstack([np.asarray(row['gt_center'])[:2], reference])
        axis_velocity = np.empty_like(reference)
        axis_velocity[:-1] = (points[2:] - points[:-2]) / .2
        axis_velocity[-1] = (points[-1] - points[-2]) / .1
        for observed, velocity in zip(row['evaluation_axes_xy'], axis_velocity):
            speed = np.linalg.norm(velocity)
            if speed < .5:
                if observed is not None:
                    raise ValueError('Stationary/slow evaluation tangent was not excluded')
            else:
                maxima['axis'] = max(maxima['axis'], check_close(observed, velocity / speed, 'future-GT evaluation axis'))
        map_path = np.asarray(row['paths']['mapcv3_top1'])
        for horizon in (1, 2, 3):
            expected = frozen_by_key[(item, horizon)]
            last = horizon * 10
            maxima['map_endpoint'] = max(maxima['map_endpoint'], check_close(
                map_path[last-1], expected['top1_endpoint'], 'frozen map endpoint'))
            errors = np.linalg.norm(map_path[:last] - reference[:last], axis=1)
            maxima['map_metric'] = max(maxima['map_metric'], check_close(
                [float(errors.mean()), float(errors[-1])],
                [expected['top1_ADE_m'], expected['top1_FDE_m']], 'frozen map ADE/FDE'))
        for method in values:
            path = np.asarray(row['paths'][method], dtype=float)
            values[method].append(float(np.linalg.norm(path[-1] - reference[-1])))
            if 'evaluation_axes_xy' in row:
                for index, forward in enumerate(row['evaluation_axes_xy']):
                    if forward is None:
                        continue
                    direction = np.asarray(forward)
                    check_close(np.linalg.norm(direction), 1., 'unit reference tangent')
                    error = path[index] - reference[index]
                    along = float(np.dot(error, direction))
                    left = float(np.dot(error, [-direction[1], direction[0]]))
                    check_close(along*along + left*left, float(np.dot(error, error)), 'orthogonal split', 1.e-6)
                    decomposition_samples += 1
        for method, path in row['paths'].items():
            error = np.asarray(path) - reference
            for horizon in (1, 2, 3):
                count = horizon * 10
                metric = row['metrics'][method][str(horizon)]
                norms = np.linalg.norm(error[:count], axis=1)
                maxima['reduced_metric'] = max(maxima['reduced_metric'], check_close(
                    [metric['ADE_m'], metric['FDE_m']], [norms.mean(), norms[-1]], 'independent ADE/FDE'))
                for component in ('longitudinal', 'lateral'):
                    component_values = []
                    for index, axis in enumerate(row['evaluation_axes_xy'][:count]):
                        if axis is not None:
                            direction = axis if component == 'longitudinal' else [-axis[1], axis[0]]
                            component_values.append(float(error[index] @ direction))
                    expected = {
                        'signed_ADE_m': float(np.mean(component_values)) if component_values else None,
                        'abs_ADE_m': float(np.mean(np.abs(component_values))) if component_values else None,
                    }
                    axis = row['evaluation_axes_xy'][count-1]
                    signed = (float(error[count-1] @ (axis if component == 'longitudinal' else [-axis[1], axis[0]]))
                              if axis is not None else None)
                    expected.update(signed_FDE_m=signed, abs_FDE_m=abs(signed) if signed is not None else None)
                    for suffix, target in expected.items():
                        observed = metric[component + '_' + suffix]
                        if target is None:
                            if observed is not None:
                                raise ValueError('Undefined component reported as a numeric error')
                        else:
                            maxima['reduced_metric'] = max(maxima['reduced_metric'], check_close(
                                observed, target, 'independent signed/absolute component'))
    if pins != {path: sha(path) for path in pins}:
        raise RuntimeError('Input changed during independent audit')
    result = dict(schema='e12_motion_ablation_independent_qa_v1', complete=True,
        source_sha256=pins, origin_count=len(rows), same_3s_cohort_exact=True,
        current_association_and_future_support_exact=True, maximum_abs_difference=maxima,
        decomposition_samples=decomposition_samples, causal_velocity_origins=causal_origins,
        independently_reduced_FDE3_m={method: distribution(value) for method, value in values.items()},
        production_changed=False, inference_rerun=False, ros_started=False,
        limitations=['Reuses existing audited normalization/GT helper, not the new analyzer metric/predictor functions.',
                     'Diagnostic oracles are not deployment policies; this audit alone is not a runtime safety test.'])
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=BASE / 'motion_ablation_v1')
    parser.add_argument('--output', type=Path, default=BASE / 'MOTION_ABLATION_INDEPENDENT_QA.json')
    args = parser.parse_args()
    print(json.dumps(audit(args.input, args.output), indent=2))


if __name__ == '__main__':
    main()
