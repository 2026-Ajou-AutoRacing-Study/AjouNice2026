#!/usr/bin/env python3
"""CPU-only detector/UUID timeline and original-EKF-core diagnostic replay.

No ROS master or nodes. The optional replay is not the ROS scheduler: it
advances the original core at measurement stamps using recorded TF, keeping
all detector history rather than resetting the tracker around a bad frame.
"""
import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import shlex
import subprocess
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml

from audit_tracking_continuity import ROOT, DET, TRACK, PRED, object_row, stats


def sha(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for chunk in iter(lambda: handle.read(8*1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


def rotation_matrix(rotation):
    # Ubuntu20's system SciPy predates as_matrix; do not require a new
    # training/runtime dependency merely for a read-only bag audit.
    return rotation.as_matrix() if hasattr(rotation, 'as_matrix') else rotation.as_dcm()


def gaussian_xy_rank(innovation, covariance, measurement_variance):
    """Unit-referenced XY Gaussian NLL, ranking only (never the legacy gate)."""
    s = np.asarray(covariance, dtype=float)+np.eye(2)*measurement_variance
    s = .5*(s+s.T)
    sign, logdet = np.linalg.slogdet(s)
    if sign <= 0 or not np.isfinite(s).all() or np.linalg.eigvalsh(s).min() <= 0:
        raise ValueError('Positive definite finite innovation covariance required')
    d = np.asarray(innovation, dtype=float)
    return .5*(float(d @ np.linalg.solve(s, d))+float(logdet))


def matrix_at(rows, stamp, static=False, max_gap=.2):
    times = rows[0]
    index = bisect_left(times, stamp)
    if static:
        index = 0
    if index < len(times) and (static or abs(times[index]-stamp) < 1.e-8):
        position, quaternion = rows[1][index]
        rotation = rotation_matrix(Rotation.from_quat(quaternion))
    elif 0 < index < len(times) and times[index]-times[index-1] <= max_gap:
        a, b = rows[1][index-1], rows[1][index]
        alpha = (stamp-times[index-1])/(times[index]-times[index-1])
        position = (1-alpha)*np.asarray(a[0])+alpha*np.asarray(b[0])
        # Avoid loss of sub-microsecond precision in Slerp's origin choice.
        rotation = rotation_matrix(Slerp([0., 1.], Rotation.from_quat([a[1], b[1]]))([alpha]))[0]
    else:
        return None
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = position
    return result


def load(path):
    import rosbag
    edges = defaultdict(dict)
    static = set()
    frames = defaultdict(list)
    with rosbag.Bag(str(path)) as bag:
        start = bag.get_start_time()
        for topic, msg, receipt in bag.read_messages(topics=[DET, TRACK, PRED, '/tf', '/tf_static']):
            if topic in ('/tf', '/tf_static'):
                for tr in msg.transforms:
                    key = (tr.header.frame_id.lstrip('/'), tr.child_frame_id.lstrip('/'))
                    stamp = tr.header.stamp.to_sec()
                    value = ([float(getattr(tr.transform.translation, k)) for k in 'xyz'],
                             [float(getattr(tr.transform.rotation, k)) for k in 'xyzw'])
                    edges[key][stamp] = value
                    if topic == '/tf_static':
                        static.add(key)
            else:
                frames[topic].append(dict(stamp=msg.header.stamp.to_sec(), receipt=receipt.to_sec(),
                                          frame_id=msg.header.frame_id.lstrip('/'),
                                          objects=[object_row(o, topic == PRED) for o in msg.objects]))
    packed_edges = {key: (sorted(values), [values[t] for t in sorted(values)]) for key, values in edges.items()}
    # Resolve only the recorded parent chain, not a guessed Ego/calibration.
    def transform(frame, stamp):
        result = np.eye(4)
        visited = set()
        while frame != 'map':
            if frame in visited:
                raise ValueError('TF cycle')
            visited.add(frame)
            choices = [key for key in packed_edges if key[1] == frame]
            if len(choices) != 1:
                raise ValueError('Missing or ambiguous recorded TF parent for '+frame)
            key = choices[0]
            value = matrix_at(packed_edges[key], stamp, key in static)
            if value is None:
                return None
            result = value @ result
            frame = key[0]
        return result
    excluded = []
    for frame in frames[DET]:
        t = frame['stamp']
        matrix = transform(frame['frame_id'], t)
        if matrix is None:
            excluded.append(t)
            frame['world_objects'] = None
            continue
        world = []
        for obj in frame['objects']:
            center = matrix @ [*obj['center'], 1.]
            # Object box orientation is local yaw; compose in 3-D with TF.
            rotation = matrix[:3, :3] @ rotation_matrix(Rotation.from_euler('z', obj['yaw']))
            world.append(dict(obj, center=center[:3].tolist(), yaw=math.atan2(rotation[1, 0], rotation[0, 0])))
        frame['world_objects'] = world
    return start, frames, excluded


def build_original(output, params, policy=None):
    sys.path.insert(0, str(ROOT/'perception/detection3d/bevfusion/tools'))
    from build_base5_low_score_probe import HARNESS, TRACKER
    source = TRACKER/'src/algorithm/ekf_multi_object_tracking.cpp'
    header = TRACKER/'include/algorithm/ekf_multi_object_tracking.hpp'
    original = {str(p): sha(p) for p in (source, header)}
    policy = policy or {}
    if 'gaussian_xy_ranking' in policy and policy['gaussian_xy_ranking'] not in ('pedestrian', 'all'):
        raise ValueError('Unknown Gaussian ranking scope')
    header_text = header.read_text()
    # Current core exposes the qualified candidate as a runtime option. Always
    # pin this audit's baseline OFF, independently of deployed YAML defaults.
    params = dict(params)
    native_gaussian = 'bool pedestrian_gaussian_association_ranking{' in header_text
    if native_gaussian:
        params['pedestrian_gaussian_association_ranking'] = policy.get('gaussian_xy_ranking') in ('pedestrian', 'all')
    fields = set(re.findall(r'^\s+(?:bool|double|int)\s+(\w+)\s*(?:\{|=)', header_text, re.M))
    settings = []
    for key, value in params.items():
        if key in fields and isinstance(value, (bool, int, float)):
            settings.append('  config.{} = {};'.format(key, str(value).lower() if isinstance(value, bool) else value))
    text = HARNESS.replace('  auto tracker =', '\n'.join(settings)+'\n  auto tracker =', 1)
    # Printing predicted prior covariance makes l2/Mahalanobis rejection
    # auditable without changing the core or exposing private methods.
    anchor = '    tracker->RunUpdate(frame);'
    before = '''
    const auto prior = tracker->GetTrackResults();
    for (const auto &t : prior.track) if (t.is_init) {
      std::cout << "BEFORE " << time << " " << t.track_id << " " << t.getRepClass() << " "
                << t.is_confirmed << " " << t.state_vec(S_X) << " " << t.state_vec(S_Y) << " "
                << t.state_vec(S_YAW) << " " << t.state_vec(S_VX) << " " << t.state_vec(S_VY) << " "
                << t.state_cov(S_X,S_X) << " " << t.state_cov(S_X,S_Y) << " " << t.state_cov(S_Y,S_Y) << " "
                << t.last_measurement_time << " " << t.age << "\\n";
    }
'''
    if text.count(anchor) != 1:
        raise ValueError('Harness source anchor changed')
    text = text.replace(anchor, before+anchor)
    output.mkdir(parents=True)
    harness = output/'original_diagnostic_harness.cpp'
    harness.write_text(text)
    linked_source = source
    linked_include = TRACKER/'include'
    if policy:
        source_text = source.read_text()
        def substitute(old, new):
            nonlocal source_text
            if source_text.count(old) != 1:
                raise ValueError('Candidate source anchor changed: '+old)
            source_text = source_text.replace(old, new, 1)
        if 'pedestrian_birth_floor' in policy:
            anchor = '        if (ShouldSuppressDuplicateTrackBirth(measurements.meas[meas_idx])) {'
            prefix = ('        if (measurements.meas[meas_idx].classification == mc_mot::ObjectClass::PEDESTRIAN &&\n'
                      '            measurements.meas[meas_idx].detection_confidence < '+str(policy['pedestrian_birth_floor'])+') continue;\n')
            substitute(anchor, prefix+anchor)
        if 'pedestrian_duplicate_distance' in policy:
            substitute('    const double measurement_max_dimension = std::max(',
                       '    const double minimum_dimension = measurement.classification == mc_mot::ObjectClass::PEDESTRIAN ?\n'
                       '        .25 : config_.duplicate_birth_minimum_max_dimension_m;\n'
                       '    const double suppression_distance = measurement.classification == mc_mot::ObjectClass::PEDESTRIAN ?\n'
                       '        '+str(policy['pedestrian_duplicate_distance'])+' : config_.duplicate_birth_suppression_distance_m;\n'
                       '    const double measurement_max_dimension = std::max(')
            source_text = source_text.replace('config_.duplicate_birth_minimum_max_dimension_m) {', 'minimum_dimension) {')
            substitute('            config_.duplicate_birth_suppression_distance_m) {', '            suppression_distance) {')
            if 'pedestrian_duplicate_height_ratio' in policy:
                extra = ('        if (measurement.classification == mc_mot::ObjectClass::PEDESTRIAN &&\n'
                         '            (std::max(measurement.dimension.height, track.dimension.height) /\n'
                         '                 std::max(1.e-6, std::min(measurement.dimension.height, track.dimension.height)) >\n'
                         '                 '+str(policy['pedestrian_duplicate_height_ratio'])+' ||\n'
                         '             std::abs(measurement.state.z - track.object_z) > '+str(policy['pedestrian_duplicate_z_m'])+')) continue;\n')
                substitute('        const double dimension_ratio = std::max(', extra+'        const double dimension_ratio = std::max(')
        if 'confirmed_pedestrian_cost_bias' in policy:
            extra = ('            if (measurements.meas[meas_idx].classification == mc_mot::ObjectClass::PEDESTRIAN &&\n'
                     '                all_tracks_[track_idx].is_init && !all_tracks_[track_idx].is_confirmed &&\n'
                     '                all_tracks_[track_idx].getRepClass() == mc_mot::ObjectClass::PEDESTRIAN && maha_distance < 1000.)\n'
                     '                maha_distance += '+str(policy['confirmed_pedestrian_cost_bias'])+';\n')
            substitute('            cost_matrix(meas_idx, track_idx) = maha_distance;', extra+'            cost_matrix(meas_idx, track_idx) = maha_distance;')
        if native_gaussian and policy.get('gaussian_xy_ranking') == 'all':
            # Research-only all-class comparison; never a deployment option.
            substitute('                if (measurements.meas[meas_idx].classification == mc_mot::ObjectClass::PEDESTRIAN &&\n'
                       '                    all_tracks_[track_idx].is_init && maha_distance < config_.max_association_dist_m) {',
                       '                if (all_tracks_[track_idx].is_init && maha_distance < config_.max_association_dist_m) {')
        if 'gaussian_xy_ranking' in policy and not native_gaussian:
            mode = policy['gaussian_xy_ranking']
            if mode not in ('pedestrian', 'all'):
                raise ValueError('Unknown Gaussian ranking scope')
            # Keep legacy cost_matrix for eligibility, separately rank the
            # exact same admitted edges. Never compare NLL against 5m/5maha.
            substitute('    cost_matrix.setConstant(std::numeric_limits<double>::max());',
                       '    cost_matrix.setConstant(std::numeric_limits<double>::max());\n'
                       '    base5_gaussian_rank_ = cost_matrix;')
            condition = ('measurements.meas[meas_idx].classification == mc_mot::ObjectClass::PEDESTRIAN && '
                         if mode == 'pedestrian' else '')
            ranking = '''
            base5_gaussian_rank_(meas_idx, track_idx) = maha_distance;
            if (SCOPE all_tracks_[track_idx].is_init && maha_distance < config_.max_association_dist_m) {
                const auto &track = all_tracks_[track_idx];
                const auto &observation = measurements.meas[meas_idx];
                Eigen::Vector2d innovation;
                innovation << observation.state.x - track.state_vec(S_X), observation.state.y - track.state_vec(S_Y);
                Eigen::Matrix2d covariance = track.state_cov.block<2,2>(S_X, S_X);
                const double multiplier = observation.detection_confidence < .5 ? 10. : 1.;
                covariance += multiplier * R_.block<2,2>(0,0);
                covariance = (.5 * (covariance + covariance.transpose())).eval();
                const double determinant = covariance.determinant();
                if (covariance.allFinite() && determinant > 1.e-12 && covariance(0,0) > 0. && covariance(1,1) > 0.) {
                    const double squared = innovation.transpose() * covariance.inverse() * innovation;
                    // covariance is in m^2; determinant is relative to 1 m^4.
                    // The common 2*log(2*pi) constant cannot affect ranking.
                    base5_gaussian_rank_(meas_idx, track_idx) = .5 * (squared + std::log(determinant));
                }
            }
'''.replace('SCOPE ', condition)
            substitute('            cost_matrix(meas_idx, track_idx) = maha_distance;',
                       '            cost_matrix(meas_idx, track_idx) = maha_distance;\n'+ranking)
            substitute('                pairs.push_back({i, j, cost_matrix(i, j)});',
                       '                pairs.push_back({i, j, base5_gaussian_rank_(i, j)});')
            linked_include = output/'include'
            (linked_include/'algorithm').mkdir(parents=True)
            generated_header = header_text.replace('    int cur_track_id_{0}; // Current track id',
                '    Eigen::MatrixXd base5_gaussian_rank_; // Research-only ranking; legacy gate untouched\n'
                '    int cur_track_id_{0}; // Current track id', 1)
            if generated_header == header_text:
                raise ValueError('Pinned header rank matrix anchor changed')
            (linked_include/'algorithm'/header.name).write_text(generated_header)
        linked_source = output/'candidate_core.cpp'
        linked_source.write_text(source_text)
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'roscpp'], text=True))
    binary = output/'original_core'
    command = ['g++', '-std=c++17', '-O1', '-I/usr/include/eigen3', '-I'+str(linked_include),
               str(harness), str(linked_source), '-o', str(binary), *flags]
    subprocess.run(command, check=True)
    if original != {p: sha(p) for p in original}:
        raise ValueError('Production core changed during build')
    receipt = dict(production_source_sha256=original, harness_sha256=sha(harness), binary_sha256=sha(binary),
                   settings=settings, unused_runtime_parameters=sorted(set(params)-fields), production_modified=False)
    receipt['policy_candidate'] = policy
    receipt['linked_source_sha256'] = sha(linked_source)
    receipt['linked_header_sha256'] = sha(linked_include/'algorithm'/header.name)
    (output/'build_receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
    return binary, receipt


def replay(binary, frames):
    selected = [f for f in frames if f['world_objects'] is not None]
    times = [f['stamp'] for f in selected]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('Non-increasing detector stamps need explicit handling before core replay')
    lines = []
    for frame in selected:
        objects = frame['world_objects']
        lines.append('{} {}'.format(format(frame['stamp'], '.17g'), len(objects)))
        for obj in objects:
            label = ('vehicle', 'pedestrian', 'obstacle').index(obj['class_name'])
            lines.append(' '.join(format(v, '.17g') for v in (label, *obj['center'], obj['yaw'], *obj['size'], obj['score'])))
    result = subprocess.run([str(binary.resolve())], input='\n'.join(lines)+'\n',
                            text=True, capture_output=True, check=True, timeout=240)
    before = defaultdict(list)
    rows = []
    for line in result.stdout.splitlines():
        tokens = line.split()
        if not tokens:
            continue
        if tokens[0] == 'BEFORE':
            stamp, identity, label, confirmed = float(tokens[1]), int(tokens[2]), int(tokens[3]), bool(int(tokens[4]))
            before[stamp].append(dict(id=identity, core_class=label, confirmed=confirmed,
                                      center=[float(tokens[5]), float(tokens[6])], yaw=float(tokens[7]),
                                      velocity=[float(tokens[8]), float(tokens[9])],
                                      covariance=[float(tokens[10]), float(tokens[11]), float(tokens[12])],
                                      last_measurement_time=float(tokens[13]), age=int(tokens[14])))
        elif tokens[0] == 'FRAME':
            rows.append(dict(stamp=float(tokens[1]), expected_tracks=int(tokens[2]), tracks=[]))
        elif tokens[0] == 'TRACK':
            rows[-1]['tracks'].append(dict(id=int(tokens[1]), core_class=int(tokens[2]), confirmed=bool(int(tokens[3])),
                                          associated=bool(int(tokens[4])), center=list(map(float, tokens[5:8])),
                                          velocity=list(map(float, tokens[8:10])), yaw=float(tokens[10]),
                                          size=list(map(float, tokens[11:14])), score=float(tokens[14]),
                                          last_measurement_time=float(tokens[15]), age=int(tokens[16])))
    if len(rows) != len(selected) or any(len(r['tracks']) != r['expected_tracks'] for r in rows):
        raise ValueError('Incomplete core replay')
    for row in rows:
        row['before'] = before[row['stamp']]
    return rows


def core_summary(rows, gt_case=None):
    histories = defaultdict(list)
    counters = Counter()
    output_counts = []
    for frame in rows:
        tracks = [t for t in frame['tracks'] if t['confirmed']]
        for track in tracks:
            cls = 'pedestrian' if track['core_class'] in (3, 4) else 'vehicle' if track['core_class'] in (1, 2) else 'obstacle'
            histories[(cls, track['id'])].append(frame['stamp'])
            counters[cls+'/observations'] += 1
            counters[cls+'/associated'] += bool(track['associated'])
            counters[cls+'/coasting'] += not track['associated']
        output_counts.append(sum(t['core_class'] == 3 for t in tracks))
    result = dict(counts=dict(counters), confirmed_unique_ids={cls: len({i for c, i in histories if c == cls})
                  for cls in ('vehicle', 'pedestrian', 'obstacle')},
                  pedestrian_confirmed_count=stats(output_counts))
    if gt_case is not None:
        from audit_tracking_continuity import evaluate_gt
        case = dict(gt_case)
        case['frames'] = {TRACK: [dict(stamp=f['stamp'], objects=[dict(
            t, class_name='pedestrian' if t['core_class'] in (3, 4) else 'vehicle' if t['core_class'] in (1, 2) else 'obstacle',
            id=str(t['id'])) for t in f['tracks'] if t['confirmed']]) for f in rows]}
        report, _ = evaluate_gt(case, 2.)
        result['gt_gate2m'] = report
    return result


def close_pedestrian_case(separation, second_score=.8):
    frames = []
    for index in range(30):
        objects = [dict(class_name='pedestrian', center=[0., 0., 0.], yaw=0., size=[.5, .3, 1.7], score=.8)]
        if index >= 10:
            objects.append(dict(class_name='pedestrian', center=[0., separation, 0.], yaw=0., size=[.5, .3, 1.7], score=second_score))
        frames.append(dict(stamp=100.+index*.1, world_objects=objects))
    return frames


def compound_case(child_height, child_z):
    frames = close_pedestrian_case(.2)
    for frame in frames:
        frame['world_objects'][0]['center'][2] = .85
        if len(frame['world_objects']) > 1:
            child = frame['world_objects'][1]
            child['center'][2] = child_z
            child['size'] = [.7, .3, child_height]
    return frames


def overlapping_compound_case():
    frames = compound_case(.7, .35)
    for index, frame in enumerate(frames):
        if len(frame['world_objects']) > 1:
            frame['world_objects'][1]['center'][1] = 0.
            if index % 2:
                frame['world_objects'].reverse()
    return frames


def compound_shape_identity(rows):
    z_error, height_error = [], []
    for frame in rows:
        for track in frame['tracks']:
            if track['confirmed'] and track['core_class'] == 3:
                expected_z, expected_height = (.85, 1.7) if track['id'] == 0 else (.35, .7)
                z_error.append(abs(track['center'][2]-expected_z))
                height_error.append(abs(track['size'][2]-expected_height))
    return dict(z_error_m=stats(z_error), height_error_m=stats(height_error),
                caveat='Same XY, differing Z/height, alternating detector order: retaining two IDs is not proof of retaining adult/child identity.')


def crossing_case(occlude=False):
    frames, truth = [], []
    for i in range(70):
        stamp = 100.+i*.1
        objects = []
        gt = []
        for name, x, y, yaw in [('a', -1.4+.04*i, -.175, 0.), ('b', 1.4-.04*i, .175, math.pi)]:
            obj = dict(class_name='pedestrian', center=[x, y, .85], yaw=yaw, size=[.5, .3, 1.7], score=.8)
            gt.append(dict(obj, key=name, asset='synthetic_person'))
            if not (occlude and name == 'a' and 33 <= i <= 36):
                objects.append(obj)
        if i % 2:
            objects.reverse()  # detector ordering is not actor identity
        frames.append(dict(stamp=stamp, world_objects=objects))
        truth.append(gt)
    gt_case = dict(ego=([f['stamp'] for f in frames], [dict(center=[0., 0., 0.], yaw=0.) for _ in frames]),
                   gt=[([f['stamp'] for f in frames], [r[j] for r in truth]) for j in (0, 1)], unresolved=[])
    return frames, gt_case


def stale_covariance_case():
    """A missing actor then a different nearby newcomer; no identity oracle enters core."""
    frames = []
    for i in range(25):
        objects = []
        if i < 8:
            objects.append(dict(class_name='pedestrian', center=[0., 0., .85], yaw=0., size=[.5, .3, 1.7], score=.8))
        elif i >= 11:
            objects.append(dict(class_name='pedestrian', center=[.8, 0., .85], yaw=0., size=[.5, .3, 1.7], score=.8))
        frames.append(dict(stamp=100.+i*.1, world_objects=objects))
    return frames


def stale_competitor_case(detector_bias=0.):
    """An old confirmed actor and a legitimately new uncertain near actor."""
    frames = []
    truth = []
    for i in range(18):
        stamp = 100.+i*.1
        old = dict(class_name='pedestrian', center=[0., 0., .85], yaw=0., size=[.5, .3, 1.7], score=.8)
        gt = [dict(old, key='old_actor', asset='synthetic_person')]
        objects = [old] if i < 4 else []
        if i >= 5:
            newcomer = dict(old, center=[2.1 if i == 5 else 1.7, 0., .85])
            gt.append(dict(newcomer, key='new_actor', asset='synthetic_person'))
            measured = dict(newcomer, center=list(newcomer['center']))
            if i == 6:
                measured['center'][0] += detector_bias
            objects.append(measured)
        frames.append(dict(stamp=stamp, world_objects=objects))
        truth.append(gt)
    times = [f['stamp'] for f in frames]
    gt_case = dict(ego=(times, [dict(center=[0., 0., 0.], yaw=0.) for _ in frames]),
                   gt=[(times, [r[0] for r in truth]), (times[5:], [r[1] for r in truth[5:]])], unresolved=[])
    return frames, gt_case


def event_causes(core, frames, start):
    """Report births, coexistence and loss from real core states, no heuristics."""
    by_stamp = {f['stamp']: f for f in frames}
    births, deaths = [], []
    previous = {}
    for frame in core:
        now = {t['id']: t for t in frame['tracks']}
        prior = {t['id']: t for t in frame['before']}
        det = by_stamp[frame['stamp']]
        for identity in now.keys()-previous.keys():
            t = now[identity]
            if t['core_class'] == 3:
                births.append(dict(stamp=frame['stamp'], seconds_from_bag_start=frame['stamp']-start,
                                   id=identity, new_track=t, detector=det['world_objects'],
                                   mature_associated_ids=[o['id'] for o in now.values() if o['confirmed'] and o['associated']],
                                   detector_header_age_sec=det['receipt']-det['stamp'] if 'receipt' in det else None))
        for identity in previous.keys()-now.keys():
            t = previous[identity]
            if t['core_class'] == 3:
                age = frame['stamp']-t['last_measurement_time']
                deaths.append(dict(stamp=frame['stamp'], seconds_from_bag_start=frame['stamp']-start,
                                   id=identity, last_association_gap_sec=age, confirmed=t['confirmed'],
                                   bounded_coast_expiry_consistent=bool(t['confirmed'] and age > .5),
                                   last_state=t, predicted_before=prior.get(identity)))
        previous = now
    return dict(pedestrian_births=births, pedestrian_deaths=deaths,
                note='Core states only; deployed ROS confirmed-copy lifecycle is separately compared, never inferred from 30Hz headers.')


def draw_events(windows, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    selected = []
    for window in windows:
        if not selected or window['event']['stamp']-selected[-1]['event']['stamp'] > 1.:
            selected.append(window)
    fig, axes = plt.subplots(len(selected), 3, figsize=(15, 4*len(selected)), squeeze=False)
    for row, window in enumerate(selected):
        center = window['event']['stamp']
        for frame in window['recorded_frames'][DET]:
            for obj in frame.get('world_objects', []):
                if obj['class_name'] == 'pedestrian':
                    axes[row, 0].scatter(frame['stamp']-center, obj['score'], s=12, color='tab:blue')
        axes[row, 0].set_ylabel('Delivered pedestrian score')
        axes[row, 0].set_ylim(0., 1.)
        for frame in window['recorded_frames'][TRACK]:
            for obj in frame['objects']:
                axes[row, 1].scatter(frame['stamp']-center, int(obj['id'][-8:], 16), s=10, color='tab:orange')
        axes[row, 1].set_ylabel('Recorded confirmed UUID suffix')
        for frame in window['original_core']:
            for obj in frame['tracks']:
                if obj['core_class'] == 3:
                    axes[row, 2].scatter(frame['stamp']-center, obj['id'], s=18,
                                         color='tab:green' if obj['associated'] else 'tab:red',
                                         marker='o' if obj['confirmed'] else 'x')
        axes[row, 2].set_ylabel('Original core ID (green=updated/red=coast)')
        for axis in axes[row]:
            axis.axvline(0., color='black', ls='--', lw=1)
            axis.set_xlabel('Seconds from GT-nearest UUID switch')
            axis.grid(alpha=.2)
    fig.suptitle('One Man1: low-IoU duplicate births compete for association; no >0.5s detector outage')
    fig.tight_layout()
    fig.savefig(output/'pedestrian_birth_association_timeline.png', dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--continuity-report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--replay-original-core', action='store_true')
    parser.add_argument('--policy-sensitivity', action='store_true',
                        help='Offline-only generated core candidates and close-two-person counterexamples; no promotion')
    parser.add_argument('--extended-candidates', action='store_true',
                        help='Add narrow size/Z guard and tentative ownership sensitivity; research only')
    parser.add_argument('--gaussian-ranking-research', action='store_true',
                        help='Only baseline/ped-only/all-class NLL ranking; keep original gate and no promotion')
    args = parser.parse_args()
    if (args.extended_candidates or args.gaussian_ranking_research) and not args.policy_sensitivity:
        raise ValueError('Extended candidates require explicit policy-sensitivity mode')
    if args.output.exists():
        raise FileExistsError('Fresh output required')
    args.output.mkdir(parents=True)
    continuity = json.loads(args.continuity_report.read_text())
    start, frames, excluded = load(args.bag)
    parameters = yaml.safe_load((args.bag.parent/'ros_parameters.yaml').read_text())
    core, build = [], None
    if args.replay_original_core:
        binary, build = build_original(args.output/'core_build', parameters['configure'])
        core = replay(binary, frames[DET])
        with gzip.open(args.output/'original_core_rows.json.gz', 'wt') as handle:
            json.dump(core, handle, allow_nan=False)
        (args.output/'original_core_causes.json').write_text(json.dumps(event_causes(core, frames[DET], start),
                                                                       indent=2, allow_nan=False)+'\n')
    sensitivity = None
    if args.policy_sensitivity:
        if not core:
            raise ValueError('Policy sensitivity requires original core control')
        from audit_tracking_continuity import read_case
        case = read_case(args.bag, assisted=True)
        reference = replay(binary, frames[DET])
        if core != reference:
            raise ValueError('Independent original-core repeat differs; no candidate attribution')
        policies = dict(baseline={}, birth020={'pedestrian_birth_floor': .2},
                        birth030={'pedestrian_birth_floor': .3},
                        guard025={'pedestrian_duplicate_distance': .25},
                        guard040={'pedestrian_duplicate_distance': .4})
        if args.extended_candidates:
            policies.update(guard020_shape_z=dict(pedestrian_duplicate_distance=.2,
                                                  pedestrian_duplicate_height_ratio=1.5, pedestrian_duplicate_z_m=.25),
                            confirmed_bias050=dict(confirmed_pedestrian_cost_bias=.5))
        if args.gaussian_ranking_research:
            policies = dict(baseline={}, gaussian_ped=dict(gaussian_xy_ranking='pedestrian'),
                            gaussian_all=dict(gaussian_xy_ranking='all'))
        sensitivity = dict(reference_repeat_identical=True, production_changed=False,
                           frozen_candidates=policies, results={})
        for name, policy in policies.items():
            candidate_binary = binary if not policy else build_original(args.output/('candidate_'+name), parameters['configure'], policy)[0]
            candidate = core if not policy else replay(candidate_binary, frames[DET])
            result = core_summary(candidate, case)
            result['synthetic_close_two_people'] = {}
            for separation in (.2, .35, .5, .75):
                synthetic = replay(candidate_binary, close_pedestrian_case(separation))
                result['synthetic_close_two_people'][str(separation)] = core_summary(synthetic)
            result['synthetic_weak_new_person_score015'] = core_summary(replay(candidate_binary, close_pedestrian_case(.75, .15)))
            result['synthetic_compound'] = {name: core_summary(replay(candidate_binary, compound_case(height, z)))
                                             for name, height, z in [('baby', .8, .5), ('dog', .7, .35)]}
            overlap_rows = replay(candidate_binary, overlapping_compound_case())
            result['synthetic_overlapping_compound'] = dict(core_summary(overlap_rows), **compound_shape_identity(overlap_rows))
            result['synthetic_crossing'] = {}
            for occlude in (False, True):
                crossing, crossing_gt = crossing_case(occlude)
                result['synthetic_crossing']['with_dropout' if occlude else 'all_observed'] = core_summary(
                    replay(candidate_binary, crossing), crossing_gt)
            stale = replay(candidate_binary, stale_covariance_case())
            result['synthetic_stale_newcomer'] = dict(core_summary(stale),
                id_after_gap=[t['id'] for t in stale[11]['tracks'] if t['confirmed']],
                id_before_gap=[t['id'] for t in stale[7]['tracks'] if t['confirmed']],
                caveat='Different physical actor deliberately appears within the unchanged original gate; pure ranking cannot reject the only eligible old track.')
            competitor_frames, competitor_gt = stale_competitor_case()
            competitor = replay(candidate_binary, competitor_frames)
            result['synthetic_stale_competitor'] = dict(core_summary(competitor, competitor_gt),
                associated_at_06s=[t['id'] for t in competitor[6]['tracks'] if t['associated']],
                prior_at_06s=competitor[6]['before'],
                caveat='Old physical actor is missing after0.3s; a new actor appears at2.1m then1.7m. Identity transfer to old confirmed ID is wrong despite being within the original gate.')
            with gzip.open(args.output/(name+'_stale_competitor.json.gz'), 'wt') as handle:
                json.dump(competitor, handle, allow_nan=False)
            result['synthetic_noisy_newcomer'] = {}
            for bias in (-.7, 0., .7):
                noisy_frames, noisy_gt = stale_competitor_case(bias)
                noisy = replay(candidate_binary, noisy_frames)
                result['synthetic_noisy_newcomer'][str(bias)] = dict(core_summary(noisy, noisy_gt),
                    associated_at_06s=[t['id'] for t in noisy[6]['tracks'] if t['associated']],
                    intended_new_actor_id=1, fixed_single_observation_bias_m=bias)
                with gzip.open(args.output/(name+'_newcomer_bias_'+str(bias)+'.json.gz'), 'wt') as handle:
                    json.dump(noisy, handle, allow_nan=False)
            sensitivity['results'][name] = result
            with gzip.open(args.output/(name+'_core_rows.json.gz'), 'wt') as handle:
                json.dump(candidate, handle, allow_nan=False)
        (args.output/'policy_sensitivity.json').write_text(json.dumps(sensitivity, indent=2, allow_nan=False)+'\n')
    events = continuity['gt_gate_sensitivity']['2.0']['switch_events']
    windows = []
    for event in events:
        center = event['stamp']
        selected = {}
        for topic in (DET, TRACK, PRED):
            rows = [f for f in frames[topic] if center-1.5 <= f['stamp'] <= center+1.5]
            if topic != DET:
                rows = [dict(f, objects=[o for o in f['objects'] if o['class_name'] == event['class_name']]) for f in rows]
            selected[topic] = rows
        core_rows = [r for r in core if center-1.5 <= r['stamp'] <= center+1.5]
        windows.append(dict(event=event, seconds_from_bag_start=center-start,
                            detector_header_age_sec=stats([f['receipt']-f['stamp'] for f in selected[DET]]),
                            detector_intervals_sec=stats([b['stamp']-a['stamp'] for a, b in zip(selected[DET], selected[DET][1:])]),
                            recorded_frames=selected, original_core=core_rows))
    payload = dict(bag=str(args.bag), bag_sha256=sha(args.bag), continuity_report_sha256=sha(args.continuity_report),
                   build=build, windows=windows, tf_unresolved_detector_stamps=excluded,
                   policy_sensitivity_ran=bool(sensitivity),
                   limitations=['Original EKF at detector measurement stamps only, not exact ROS scheduler/arrival/TF-buffer replay.',
                                'Recorded TF interpolation <=0.2s; frames lacking bracketed TF omitted and reported.',
                                'Birth/deletion/association in core replay are diagnosable; equality with deployed binary requires separate qualification.',
                                'No model inference, score threshold, production policy or planner modifications.'])
    with gzip.open(args.output/'event_windows.json.gz', 'wt') as handle:
        json.dump(payload, handle, allow_nan=False)
    compact = dict(payload)
    compact['windows'] = [dict(event=w['event'], seconds_from_bag_start=w['seconds_from_bag_start'],
                               detector_header_age_sec=w['detector_header_age_sec'], detector_intervals_sec=w['detector_intervals_sec'],
                               recorded_counts={k: len(v) for k, v in w['recorded_frames'].items()}, core_frames=len(w['original_core'])) for w in windows]
    (args.output/'event_summary.json').write_text(json.dumps(compact, indent=2, allow_nan=False)+'\n')
    if windows:
        draw_events(windows, args.output)
    print(json.dumps(compact, indent=2), flush=True)


if __name__ == '__main__':
    main()
