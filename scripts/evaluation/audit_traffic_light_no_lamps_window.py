#!/usr/bin/env python3
"""Replay lamp handoff only; no publishers, GPU inference, simulator or bag edits.

Compares bounded no-lamps retention with illustrative full-phase majority votes.
Known R/Y reaction is assessed against the recorded classifier, not native GT.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'base_ws/src/perception/traffic_light_detection'
sys.path.insert(0, str(PACKAGE / 'scripts'))
from signal_phase_policy import SignalPhasePolicy
from signal_smoother import SignalSmoother
from traffic_light_observation_handoff import no_lamps_context


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4*1024**2), b''):
            value.update(block)
    return value.hexdigest()


def allows(lamps, group):
    if any(x[0] == 'yellow' for x in lamps):
        return False
    return any(x[0] == 'green' and x[1] == ('arrow' if group == 201326 else 'circle')
               for x in lamps)


def from_callback(path):
    data = json.loads(path.read_text())
    sequences = {}
    for row in data['records']:
        outcome = row['outcomes']['yaw4_pad120']
        d, s, c = outcome['decision'], outcome['selection'], outcome['classifier']
        ns = row['source_stamp_ns']
        now = round(d['observation_timing']['guard_clock_sec']*1e9)
        packet = dict(status='ready', group_id=d['group_id'], source_stamp_ns=ns,
                      generated_stamp_ns=now, rois=[s['roi']] if s.get('roi') else [])
        sequences.setdefault(row['source'], []).append(dict(t=row['t'], source=ns,
            decision=d, selection=s, classifier=c, packet=packet))
    return sequences


def from_bag(path):
    import rosbag
    stem = '/traffic_light_node/debug_'
    packets, selections, classifiers, rows = {}, {}, {}, []
    topics = [stem+'selection', stem+'classifier', stem+'runtime',
              '/perception/traffic_light_recognition/map_rois']
    with rosbag.Bag(str(path)) as bag:
        start = bag.get_start_time()
        for topic, msg, receipt in bag.read_messages(topics=topics):
            x = json.loads(msg.data)
            if topic == topics[3]:
                packets[x['source_stamp_ns']] = x
            elif topic == topics[0]:
                selections[x['source_stamp_ns']] = x
            elif topic == topics[1]:
                classifiers[x['source_stamp_ns']] = x
            elif isinstance(x.get('frame_decision'), dict):
                d = x['frame_decision']
                ns = d['observation_timing']['current_image_source_stamp_ns']
                rows.append(dict(t=receipt.to_sec()-start, source=ns, decision=d))
    for r in rows:
        r.update(packet=packets.get(r['source']), selection=selections.get(r['source'], {}),
                 classifier=classifiers.get(r['source'], {}))
    return {path.parent.parent.parent.name: rows}


def evaluate(rows):
    counts, changes, mismatches = Counter(), [], []
    methods = {str(t): SignalPhasePolicy.from_yaml(PACKAGE/'config/traffic_signal_profiles.yaml')
               for t in (0., .1, .2, .3)}
    smoothers = {k: SignalSmoother(7, 4) for k in methods}
    votes = {'vote3of2': SignalSmoother(3, 2), 'vote7of4': SignalSmoother(7, 4)}
    previous_group, previous_head = None, None
    last_valid = None
    for row in rows:
        d, c, s, packet = row['decision'], row['classifier'], row['selection'], row['packet']
        group, ns = d['group_id'], row['source']
        clock = d['observation_timing']['guard_clock_sec']
        if group != previous_group:
            previous_head, last_valid = None, None
            for v in votes.values(): v.reset()
        context = no_lamps_context(packet, s, c, previous_head, group, ns, round(clock*1e9))
        normalized = [tuple(x) for x in d['normalized_tuples']]
        eligible = d.get('detection_miss_hold', {})
        soft = d.get('low_confidence_hold', {})
        counts['frames'] += 1
        if c.get('reason') == 'no_lamps':
            counts['no_lamps'] += 1
            counts['no_lamps_eligible'] += context['eligible']
        outputs = {}
        for key, policy in methods.items():
            result, _, reason = policy.process_normalized(normalized, d['policy_reason'], group,
                clock, max(.01, d['final_confidence']) if normalized else 0., smoothers[key],
                source_stamp_ns=ns, use_smoothing=False,
                observation_head_id=d.get('head_id') or eligible.get('head_id'),
                detection_miss_hold_eligible=eligible.get('eligible', False),
                low_confidence_hold_eligible=soft.get('eligible', False),
                low_confidence_confident_red=soft.get('confident_red', False),
                no_lamps_hold_eligible=context['eligible'], no_lamps_hold_sec=float(key))
            result = [list(x) for x in result]
            outputs['hold'+key] = result
            if key == '0.0' and result != d['final_tuples']:
                mismatches.append(dict(t=row['t'], expected=d['final_tuples'], actual=result))
            if reason == 'direct_observation:no_lamps_hold':
                counts['hold'+key+':held'] += 1
                counts['hold'+key+':held_go'] += allows(result, group)
                meta = policy.output_observation_metadata(group)
                changes.append(dict(t=row['t'], group=group, method=key, head=s.get('head_id'),
                    output=result, previous=last_valid, current_source=ns,
                    adopted_source=meta['adopted_observation_source_stamp_ns'],
                    age_sec=(round(clock*1e9)-meta['adopted_observation_source_stamp_ns'])/1e9))
        for name, voter in votes.items():
            value = voter.update(normalized, payload=normalized)
            outputs[name] = [list(x) for x in value] if value else []
        for name, value in outputs.items():
            if normalized and not allows(normalized, group) and allows(value, group):
                counts[name+':explicit_stop_but_old_go'] += 1
        if normalized:
            last_valid = dict(t=row['t'], source=ns, lamps=d['normalized_tuples'])
        previous_group = group
        previous_head = d.get('head_id') or (eligible.get('head_id') if eligible.get('eligible') else None)
    return dict(counts=dict(counts), baseline_mismatches=mismatches, held_frames=changes)


def synthetic_transitions():
    """Controlled 10 Hz inputs; not observed MORAI/native accuracy evidence."""
    report = {}
    green = [('green', 'circle', False)]
    for label, next_lamps in (('red', [('red', 'circle', False)]),
                              ('yellow', [('yellow', 'circle', False)])):
        values = {}
        for window, majority in ((3, 2), (7, 4)):
            voter = SignalSmoother(window, majority)
            for _ in range(10): voter.update(green)
            for index in range(7):
                result = voter.update(next_lamps)
                if result == next_lamps:
                    values[f'vote{window}of{majority}_delay_sec'] = round(index*.1, 3)
                    break
        policy = SignalPhasePolicy.from_yaml(PACKAGE/'config/traffic_signal_profiles.yaml')
        smoother = SignalSmoother(7, 4)
        for clock, lamps, dark in ((10., green, False), (10.1, [], True),
                                   (10.15, next_lamps, False)):
            result, _, _ = policy.process_normalized(lamps, 'synthetic', 201365,
                clock, .99 if lamps else 0., smoother, source_stamp_ns=round(clock*1e9),
                use_smoothing=False, observation_head_id='test_head',
                no_lamps_hold_eligible=dark, no_lamps_hold_sec=.2)
        assert result == next_lamps
        values['bounded_no_lamps_explicit_phase_delay_sec'] = 0.
        report[label] = values
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--callback', type=Path)
    p.add_argument('--bag', type=Path, action='append', default=[])
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    output = a.output.resolve()
    output.relative_to(ROOT/'artifacts/evaluation')
    if output.exists(): raise FileExistsError('Fresh output required')
    files = ([a.callback] if a.callback else []) + a.bag
    before = {str(path.resolve()): (path.stat().st_size, path.stat().st_mtime_ns) for path in files}
    datasets = from_callback(a.callback) if a.callback else {}
    for path in a.bag: datasets.update(from_bag(path))
    results = {k: evaluate(v) for k, v in datasets.items()}
    if any((Path(p).stat().st_size, Path(p).stat().st_mtime_ns) != v for p,v in before.items()):
        raise RuntimeError('Input changed during audit')
    sources = {str(path.relative_to(ROOT)): digest(path) for path in
        (PACKAGE/'scripts/signal_phase_policy.py', PACKAGE/'scripts/traffic_light_observation_handoff.py',
         PACKAGE/'scripts/signal_smoother.py', Path(__file__).resolve())}
    report = dict(results=results, synthetic_10hz_transitions=synthetic_transitions(),
        input_identity=before, sources=sources,
        limitations=['Recorded classifier observations, no independent lamp/native GT.',
            'Policy tuple replay; cached wire confidence is not reconstructed.',
            'Vote3/2 and7/4 are illustrative full-phase majorities, not the complete old runtime.',
            'Callback input eligibility uses already-validated current selected ROI; bag input uses original packet.',
            'No live planner/vehicle is run; held observations retain the old source timestamp.'])
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k: dict(counts=v['counts'], baseline_mismatches=len(v['baseline_mismatches']))
                      for k,v in results.items()}, ensure_ascii=False))


if __name__ == '__main__': main()
