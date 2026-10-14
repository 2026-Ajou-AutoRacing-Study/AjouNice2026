#!/usr/bin/env python3
"""Audit MORAI packet grouping without a ROS master, GPU, or live mutation.

Footer timestamps identify simulator source batches, not exact per-ray time.
Phase-aligned fixed50 is an offline counterfactual; a live fixed50 driver does
not automatically seek that phase. Point counts are validated against original
cloud counts, but this tool does not claim decoded XYZ equality or detector AP.
"""
import argparse
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import struct

import numpy as np

from lidar_packet_reassembly import (POLICIES, audit, coverage_deg,
    footer_nearest_hour, group_indices, quantiles, read_source, rotations)


def raw_return_counts(packet, minimum=.4, maximum=130.):
    data = bytes(packet.data)
    if len(data) != 1206:
        raise ValueError('Expected standard 1206-byte VLP16 packet')
    ranges = np.ndarray((12, 32), dtype='<u2', buffer=data, offset=4,
                        strides=(100, 3)).astype(np.float64) * .002
    return np.count_nonzero((ranges >= minimum) & (ranges <= maximum), axis=1)


def repeated_direction_blocks(blocks):
    """Repeated angular sectors, NOT identical rays or duplicate packet bytes."""
    angles = np.asarray(blocks, dtype=np.int64).reshape(-1)
    if not len(angles):
        return np.zeros(0, dtype=bool)
    # MORAI has nominal 0.6-degree blocks with centidegree rounding. Quantize
    # relative to this frame's first angle so 0.01-degree rounding is harmless.
    bins = np.rint(((angles - angles[0]) % 36000) / 60).astype(int) % 600
    seen, repeated = set(), []
    for value in bins:
        repeated.append(int(value) in seen)
        seen.add(int(value))
    return np.asarray(repeated, dtype=bool)


def fixed_groups(size, count, offset=0):
    return [(i, i+count) for i in range(offset, size-count+1, count)]


def phase_sweep(stamps, footers, first_angles):
    rows = []
    changed = np.r_[0, np.cumsum(np.diff(footers) != 0)]
    for offset in range(50):
        groups = fixed_groups(len(stamps), 50, offset)
        mixing = [1 + int(changed[end-1]-changed[start]) for start, end in groups]
        rows.append(dict(offset_packets=offset, frames=len(groups),
            first_angle_deg=float(first_angles[offset])/100,
            source_footer_counts=dict(Counter(mixing)),
            single_footer_fraction=float(np.mean(np.asarray(mixing) == 1)) if mixing else None,
            host_packet_span_ms=quantiles([(stamps[e-1]-stamps[s])*1000 for s, e in groups])))
    return rows


def ego_arrays(source):
    # Equal timestamps are legal duplicate status receipts: keep latest receipt.
    by_stamp = {t: m for t, _, m in source['messages']['/Ego_topic']}
    times = np.asarray(sorted(by_stamp), dtype=np.float64)
    if not len(times):
        return None
    messages = [by_stamp[t] for t in times]
    xyz = np.asarray([[m.position.x, m.position.y, m.position.z] for m in messages])
    jump = np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1) > np.maximum(5., np.diff(times)*80.)
    return dict(times=times, segments=np.r_[0, np.cumsum(jump)],
        xyz=xyz,
        yaw=np.unwrap(np.radians([m.heading for m in messages])),
        speed=np.asarray([np.linalg.norm([m.velocity.x, m.velocity.y, m.velocity.z])
                          for m in messages]))


def sample_ego(ego, stamp, maximum_gap=.15):
    if ego is None:
        return None
    times = ego['times']
    index = int(np.searchsorted(times, stamp))
    if index < len(times) and times[index] == stamp:
        return dict(xyz=ego['xyz'][index], yaw=float(ego['yaw'][index]), speed=float(ego['speed'][index]),
                    segment=int(ego.get('segments', np.zeros(len(times), dtype=int))[index]))
    if index == 0 or index == len(times) or times[index]-times[index-1] > maximum_gap:
        return None
    segments = ego.get('segments', np.zeros(len(times), dtype=int))
    if segments[index] != segments[index-1]:
        return None
    alpha = (stamp-times[index-1])/(times[index]-times[index-1])
    return {**{name: (1-alpha)*values[index-1]+alpha*values[index]
               for name, values in ego.items() if name not in ('times', 'segments')},
            'segment': int(segments[index])}


def summarize_rows(rows):
    fields = ('host_packet_span_ms', 'latest_footer_minus_header_ms', 'first_to_last_footer_ms',
              'raw_in_range_points', 'repeated_angular_sector_blocks',
              'repeated_angular_sector_return_fraction', 'non_dominant_source_return_fraction', 'source_pose_translation_m',
              'source_pose_yaw_change_deg', 'ego_speed_kmh', 'reference_yaw_rate_deg_s',
              'recorded_scan_receipt_minus_header_ms', 'recorded_cloud_receipt_minus_header_ms',
              'recorded_cloud_receipt_minus_scan_receipt_ms')
    return dict(frames=len(rows), source_footer_counts=dict(Counter(r['source_footer_count'] for r in rows)),
        source_pose_exclusion_reasons=dict(Counter(r.get('source_pose_exclusion_reason') for r in rows if r.get('source_pose_exclusion_reason'))),
        **{name: quantiles([r[name] for r in rows if r.get(name) is not None]) for name in fields})


def evaluate(path):
    source = read_source(path)
    packets = source['packets']
    baseline = audit(path, source)
    stamps = np.asarray([p.stamp.to_sec() for p in packets])
    blocks = np.asarray([rotations(p) for p in packets])
    angles = blocks[:, 0]
    footer = np.asarray([struct.unpack_from('<I', bytes(p.data), 1200)[0] for p in packets], dtype=np.int64)
    footer_times = np.asarray([footer_nearest_hour(s, f) for s, f in zip(stamps, footer)])
    returns = np.asarray([raw_return_counts(p) for p in packets])
    counts = returns.sum(axis=1)
    prefix = np.r_[0, np.cumsum(counts)]
    starts = np.r_[0, np.flatnonzero(np.diff(footer) != 0)+1]
    ends = np.r_[starts[1:], len(packets)]
    ego = ego_arrays(source)
    baseline_counts = []
    for scan in source['scans']:
        cloud = source['clouds'].get(scan['stamp'])
        baseline_counts.append(dict(stamp=scan['stamp'], packet_start=scan['start'], packet_end=scan['end'],
            raw_count=int(prefix[scan['end']]-prefix[scan['start']]),
            original_cloud_count=None if cloud is None else cloud.width*cloud.height))
    missing_clouds = [r for r in baseline_counts if r['original_cloud_count'] is None]
    mismatches = [r for r in baseline_counts if r['original_cloud_count'] is not None and r['raw_count'] != r['original_cloud_count']]
    if mismatches:
        raise ValueError('Raw in-range count differs from recorded decoder cloud: %s' % mismatches[:3])
    policies = {p: group_indices(angles, p) for p in POLICIES}
    # These are observed native publication groups, not candidate reconstruction.
    # Receipt age is only assigned when the entire original group matches.
    original_groups = {(scan['start'], scan['end']): scan for scan in source['scans']}
    cloud_receipts = {stamp: received for stamp, received, _ in source['messages']['/sensor/lidar1']}
    policies['recorded'] = list(original_groups)
    aligned_start = int(starts[1]) if len(starts) > 1 else 0
    policies['fixed50_source_phase_aligned'] = fixed_groups(len(packets), 50, aligned_start)
    all_rows, summaries = {}, {}
    for name, groups in policies.items():
        rows = []
        for index, (start, end) in enumerate(groups):
            repeated = repeated_direction_blocks(blocks[start:end])
            count = int(prefix[end]-prefix[start])
            batch_returns = Counter()
            for packet_index in range(start, end):
                batch_returns[int(footer[packet_index])] += int(counts[packet_index])
            first_pose, last_pose = sample_ego(ego, footer_times[start]), sample_ego(ego, footer_times[end-1])
            reference_pose = sample_ego(ego, stamps[start])
            yaw_before, yaw_after = sample_ego(ego, stamps[start]-.05), sample_ego(ego, stamps[start]+.05)
            row = dict(index=index, packet_start=start, packet_end=end, header=stamps[start],
                last_packet_stamp=stamps[end-1], earliest_footer=footer_times[start], latest_footer=footer_times[end-1],
                first_angle_deg=float(angles[start])/100, extent_deg=coverage_deg(blocks, start, end),
                source_footer_count=len(set(footer[start:end])),
                host_packet_span_ms=float((stamps[end-1]-stamps[start])*1000),
                latest_footer_minus_header_ms=float((footer_times[end-1]-stamps[start])*1000),
                first_to_last_footer_ms=float((footer_times[end-1]-footer_times[start])*1000),
                raw_in_range_points=count,
                repeated_angular_sector_blocks=int(np.count_nonzero(repeated)),
                repeated_angular_sector_return_fraction=float(returns[start:end].reshape(-1)[repeated].sum()/count) if count else 0.,
                non_dominant_source_return_fraction=float(1-max(batch_returns.values())/count) if count else 0.,
                source_batch_return_counts=dict(batch_returns),
                source_pose_translation_m=None, source_pose_yaw_change_deg=None, source_pose_exclusion_reason=None,
                ego_speed_kmh=None if reference_pose is None else float(reference_pose['speed']*3.6), reference_yaw_rate_deg_s=None)
            if yaw_before is not None and yaw_after is not None and yaw_before['segment'] == yaw_after['segment']:
                row['reference_yaw_rate_deg_s'] = float(np.degrees(yaw_after['yaw']-yaw_before['yaw'])/.1)
            original = original_groups.get((start, end))
            row.update(recorded_scan_receipt_minus_header_ms=None, recorded_cloud_receipt_minus_header_ms=None,
                       recorded_cloud_receipt_minus_scan_receipt_ms=None)
            if original is not None:
                row['recorded_scan_receipt_minus_header_ms'] = float((original['receipt']-original['stamp'])*1000)
                received = cloud_receipts.get(original['stamp'])
                if received is not None:
                    row['recorded_cloud_receipt_minus_header_ms'] = float((received-original['stamp'])*1000)
                    row['recorded_cloud_receipt_minus_scan_receipt_ms'] = float((received-original['receipt'])*1000)
            if first_pose is None or last_pose is None:
                row['source_pose_exclusion_reason'] = 'no_safe_ego_interpolation'
            elif first_pose['segment'] != last_pose['segment']:
                row['source_pose_exclusion_reason'] = 'ego_pose_discontinuity'
            else:
                row['source_pose_translation_m'] = float(np.linalg.norm(last_pose['xyz'][:2]-first_pose['xyz'][:2]))
                row['source_pose_yaw_change_deg'] = float(abs(np.degrees(last_pose['yaw']-first_pose['yaw'])))
            rows.append(row)
        complete = [r for r in rows if r['extent_deg'] >= 350]
        all_rows[name] = rows
        summaries[name] = dict(all=summarize_rows(rows), complete=summarize_rows(complete),
            steady_highway_40kmh=summarize_rows([r for r in complete if r['ego_speed_kmh'] is not None and r['ego_speed_kmh'] >= 40]),
            moving_curve_yawrate_3deg_s=summarize_rows([r for r in complete if r['reference_yaw_rate_deg_s'] is not None
                and abs(r['reference_yaw_rate_deg_s']) >= 3. and r['ego_speed_kmh'] is not None and r['ego_speed_kmh'] >= 5.]),
            source_mixing_yaw_gt0p1deg=summarize_rows([r for r in complete if r['source_pose_yaw_change_deg'] is not None and r['source_pose_yaw_change_deg'] >= .1]))
    whole_steps = np.diff(blocks.reshape(-1)) % 36000
    # Avoid calling centidegree rounding (0.61 instead of 0.60) a lost block.
    block_discontinuities = int(np.count_nonzero((whole_steps < 58) | (whole_steps > 62)))
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(8*1024*1024), b''):
            digest.update(chunk)
    return dict(bag=str(path.resolve()), bag_sha256=digest.hexdigest(),
        baseline_audit=baseline, raw_pointcount_validation=dict(frames=len(baseline_counts), exact_count_matches=len(baseline_counts)-len(missing_clouds),
            missing_original_clouds=missing_clouds, mismatches=[]),
        source_batch_packet_counts=dict(Counter(int(e-s) for s, e in zip(starts, ends))),
        source_batch_first_angle_deg=quantiles(angles[starts[1:]]/100),
        nominal_block_step_deg=quantiles(whole_steps/100), block_order_discontinuities=block_discontinuities,
        ego_speed_kmh=quantiles([] if ego is None else ego['speed']*3.6),
        ego_pose_discontinuity_count=0 if ego is None else int(ego['segments'][-1]),
        policies=summaries, fixed50_start_phase_sweep=phase_sweep(stamps, footer, angles), rows=all_rows)


def run(args):
    campaign_path = getattr(args, 'campaign_progress', None)
    campaign = None if campaign_path is None else json.loads(campaign_path.read_text())
    inputs = []
    if campaign is not None:
        for completed in campaign['completed']:
            if not completed['audit']['pass_']:
                raise ValueError('Completed native input audit is not passing')
            inputs.append((Path(completed['bag']), completed))
    else:
        inputs = [(path, None) for path in args.bag]
    if not inputs:
        raise ValueError('No complete source bags')
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(scope='Offline same-packet grouping and source-pose diagnostics; not live timing/AP/closed-loop certification.',
        source_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        grouping_code_sha256=hashlib.sha256(Path(__file__).with_name('lidar_packet_reassembly.py').read_bytes()).hexdigest(),
        campaign_progress=None if campaign_path is None else str(campaign_path.resolve()),
        campaign_progress_sha256=None if campaign_path is None else hashlib.sha256(campaign_path.read_bytes()).hexdigest(),
        contract=dict(packet_bytes=1206, source='MORAI VLP16', range_m=[.4,130.],
            point_count='Standard VLP16 0.002m range, zero laser distance corrections; original cloud count must match.',
            direction_repeat='Repeated angular sectors across batches, not identical rays.',
            fixed50_source_phase_aligned='Skip leading partial footer batch; counterfactual, not live fixed50 behavior.'),
        limitations=['Footer is a source batch identifier, not synchronized per-ray acquisition time.',
            'Candidate host packet span is not measured publication latency; observed receipt age only attaches to exact recorded groups.',
            'Ego motion uses independently interpolated status at footer timestamps; absent coverage is excluded.',
            'Point count equality is not XYZ/decoder-field equality or visibility-qualified object evidence.',
            'No inferred zero-return object, detector AP, tracker, or LAN generalization claim.'], cases=[])
    for index, (path, completed) in enumerate(inputs):
        case = evaluate(path)
        if completed is not None:
            if case['bag_sha256'] != completed['bag_sha256']:
                raise ValueError('Completed source bag hash differs')
            expected_count = int(completed['policy'][5:])
            if set(case['baseline_audit']['original_packet_counts']) != {expected_count}:
                raise ValueError('Native packet counts do not match campaign policy')
            case['scene'] = completed['scene']
            case['native_policy'] = completed['policy']
            case['mission'] = completed['mission']
            case['scenario_sha256'] = completed['source_sha256']
        rows = case.pop('rows')
        with gzip.open(args.output/('case%d_rows.json.gz' % index), 'wt', encoding='utf-8') as stream:
            json.dump(rows, stream, allow_nan=False)
        case['row_artifact'] = 'case%d_rows.json.gz' % index
        report['cases'].append(case)
        (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps(dict(case=index, bag=path.name, max_speed_kmh=case['ego_speed_kmh'].get('max'),
            raw_count_exact=case['raw_pointcount_validation']['exact_count_matches'],
            policies={p: {'span_ms': v['complete']['host_packet_span_ms'].get('mean'),
                'footer_counts':v['complete']['source_footer_counts']} for p,v in case['policies'].items()})), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--bag', type=Path, action='append')
    source.add_argument('--campaign-progress', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    run(parser.parse_args())
