#!/usr/bin/env python3
"""CPU-only vehicle reacquisition evidence from the unmodified native EKF.

This is measurement-stamp replay, not the asynchronous 30 Hz ROS scheduler.
Frames and native priors are saved for independent candidate comparisons.
No simulator, ROS master, runtime parameters, or production source is changed.
"""
import argparse
import gzip
import json
import math
from pathlib import Path

import numpy as np
import yaml

from audit_tracking_continuity import DET
from audit_tracking_continuity_events import build_original, load, replay, sha


def pair_gate(prior, observation, maximum_distance=5.0, measurement_variance=None):
    """Native XY gate or isolated P+R proposal; NLL ranking is separate.

    The default reproduces current native P-only eligibility. A non-None
    variance adds the same high/low-score observation covariance as update,
    never changing the hard Cartesian radius. Invalid covariance fails closed
    in this evidence helper; it must not fabricate an admitted pair.
    """
    delta = np.asarray(observation['center'][:2]) - prior['center'][:2]
    distance = float(np.linalg.norm(delta))
    xx, xy, yy = prior['covariance']
    covariance = np.asarray([[xx, xy], [xy, yy]])
    if measurement_variance is not None:
        if not math.isfinite(measurement_variance) or measurement_variance <= 0:
            raise ValueError('Positive finite measurement variance required')
        covariance += np.eye(2) * measurement_variance * (10.0 if observation.get('score', 1.0) < .5 else 1.0)
    if not np.isfinite(covariance).all() or covariance[0, 0] <= 0 or np.linalg.det(covariance) <= 1.e-12:
        raise ValueError('Positive definite finite covariance required')
    mahalanobis = math.sqrt(max(0.0, float(delta @ np.linalg.solve(covariance, delta))))
    mahalanobis = min(max(mahalanobis, distance / 5.0), distance * 3.0)
    native_eligible = distance <= maximum_distance and mahalanobis < maximum_distance
    return dict(distance_m=distance, mahalanobis=mahalanobis,
                euclidean_admitted=distance <= maximum_distance,
                covariance_admitted=mahalanobis < maximum_distance,
                native_eligible=native_eligible)


def vehicle_births(frames, core, maximum_distance=5.0):
    """Report possible stale/fresh pairs, without asserting shared identity."""
    events = []
    for frame, result in zip(frames, core):
        stamp = result['stamp']
        prior_by_id = {track['id']: track for track in result['before']}
        births = [track for track in result['tracks']
                  if track['id'] not in prior_by_id and track['core_class'] in (1, 2)]
        for born in births:
            observation = min((obj for obj in frame['world_objects']
                               if obj['class_name'] == 'vehicle'),
                              key=lambda obj: math.dist(obj['center'][:2], born['center'][:2]))
            stale = []
            for prior in prior_by_id.values():
                if prior['core_class'] not in (1, 2) or not prior['confirmed']:
                    continue
                after = next((track for track in result['tracks'] if track['id'] == prior['id']), None)
                if after is not None and after['associated']:
                    continue
                age = stamp - prior['last_measurement_time']
                if age <= 0 or math.dist(prior['center'][:2], born['center'][:2]) > 10.0:
                    continue
                stale.append(dict(prior=prior, after=after,
                                  last_association_age_sec=age,
                                  gate=pair_gate(prior, observation, maximum_distance)))
            if stale:
                events.append(dict(stamp=stamp, born=born,
                                   raw_measurement=observation, stale_candidates=stale))
    return events


def write_gzip(path, value):
    with gzip.open(path, 'wt') as handle:
        json.dump(value, handle, allow_nan=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parameters', type=Path, required=True)
    parser.add_argument('--frames', type=Path, action='append', default=[])
    parser.add_argument('--bag', type=Path, action='append', default=[])
    parser.add_argument('--prepared-inputs', type=Path)
    parser.add_argument('--prepared-predictions', type=Path)
    parser.add_argument('--sensor', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Fresh evidence directory required')
    if not args.frames and not args.bag and not args.prepared_inputs:
        raise ValueError('At least one frozen frame cache or bag required')
    args.output.mkdir(parents=True)
    parameters = yaml.safe_load(args.parameters.read_text())['configure']
    # Current production switches this ranking ON. Vehicle eligibility and
    # ranking remain untouched; the recorded older bag did not use this knob.
    binary, receipt = build_original(args.output / 'core', parameters,
                                     dict(gaussian_xy_ranking='pedestrian'), extended_state=True)
    report = dict(binary=str(binary), build_receipt=receipt,
                  parameters_sha256=sha(args.parameters), cases={},
                  scope='Native core at detector stamps; not ROS scheduler equivalence.',
                  production_changed=False)
    sources = [(path.stem.replace('.json', ''), path, None) for path in args.frames]
    sources += [(path.stem, None, path) for path in args.bag]
    prepared = {}
    if args.prepared_inputs:
        if not args.prepared_predictions or not args.sensor:
            raise ValueError('Prepared regression requires predictions and sensor calibration')
        from audit_tracking_continuity_saved_outputs import prepared_cases
        prepared, _ = prepared_cases(args.prepared_inputs, args.prepared_predictions, args.sensor)
        sources += [(name, None, None) for name in prepared]
    for name, cache, bag in sources:
        if name in report['cases']:
            raise ValueError('Duplicate case name')
        if cache:
            with gzip.open(cache, 'rt') as handle:
                frames = json.load(handle)
            frames = [frame for frame in frames if frame['world_objects'] is not None]
            provenance = dict(frames_cache=str(cache), sha256=sha(cache))
        elif bag:
            _, topics, excluded = load(bag)
            frames = [frame for frame in topics[DET] if frame['world_objects'] is not None]
            provenance = dict(bag=str(bag), sha256=sha(bag), excluded_tf=len(excluded))
        else:
            frames = prepared[name]['frames']
            provenance = dict(raw_bag=prepared[name]['raw_bag'],
                              prepared_infos_sha256=sha(args.prepared_inputs / 'infos_val.pkl'),
                              predictions_sha256=sha(args.prepared_predictions),
                              sensor_sha256=sha(args.sensor))
            write_gzip(args.output / (name + '_gt.json.gz'), prepared[name]['gt'])
        if not frames:
            raise ValueError('No detector frames available; raw sensor bags need frozen predictions: ' + name)
        result = replay(binary, frames)
        write_gzip(args.output / (name + '_frames.json.gz'), frames)
        write_gzip(args.output / (name + '_core.json.gz'), result)
        events = vehicle_births(frames, result, parameters['max_association_dist_m'])
        write_gzip(args.output / (name + '_births.json.gz'), events)
        report['cases'][name] = dict(provenance=provenance, frames=len(frames),
                                  nearby_stale_vehicle_birth_candidates=len(events))
        (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        print(json.dumps(dict(case=name, frames=len(frames), births=len(events))), flush=True)


if __name__ == '__main__':
    main()
