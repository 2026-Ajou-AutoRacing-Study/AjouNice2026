#!/usr/bin/env python3
"""Fill terminal deficits with fresh same-family scenes, never weaken capture QA."""
import argparse
from collections import Counter, defaultdict
import copy
import fcntl
import hashlib
import json
from pathlib import Path
import random
import time

import run_base5_adaptive_completion as ac
import run_base5_motion_contract_repair as repair

POLICY = 'same-family-fresh-scene-v140'
TERMINAL = ('complete', 'complete_with_deficit')


def movers(meta):
    return Counter(a['motion'] for a in meta['runtime_actors'] if a['motion'] not in ('static', 'standing'))


def compatible(source, candidate, pool_by_id):
    """Keep mission context, semantic pool, split and actual moving modes, not exact cells."""
    a, b = source['spec'], candidate['spec']
    if (a['family'], bool(a['mission']), source['split']) != (b['family'], bool(b['mission']), candidate['split']):
        return False
    actors_a, actors_b = source['runtime_actors'], candidate['runtime_actors']
    if not actors_a or not actors_b:
        return False
    if pool_by_id[actors_a[0]['data_id']] != pool_by_id[actors_b[0]['data_id']]:
        return False
    expected, actual = movers(source), movers(candidate)
    if bool(expected) != bool(actual):
        return False
    return all(actual[mode] >= count for mode, count in expected.items())


def candidate_delta(source, actual):
    return dict(original_primary=source['runtime_actors'][0]['name'],
                replacement_primary=actual['runtime_actors'][0]['name'],
                original_actor_count=len(source['runtime_actors']),
                replacement_actor_count=len(actual['runtime_actors']),
                original_distance_bounds=source['spec']['distance_bounds'],
                replacement_distance_bounds=actual['spec']['distance_bounds'],
                original_role=source['spec']['role'], replacement_role=actual['spec']['role'],
                original_movers=dict(movers(source)), replacement_movers=dict(movers(actual)))


def choose(source, donors, deck, ctx, calibration, ordinal, ident, known_shas, pool_by_id):
    # Draw asset identities uniformly, rather than favoring frequent donor scenes.
    eligible = [d for d in donors if compatible(source, d, pool_by_id)]
    if not eligible:
        raise ValueError('No compatible replacement donors')
    by_asset = defaultdict(list)
    for donor in eligible:
        by_asset[donor['runtime_actors'][0]['data_id']].append(donor)
    seed = int(hashlib.sha256(f'{POLICY}:{ident}:{ordinal}'.encode()).hexdigest()[:12], 16)
    rng = random.Random(seed)
    ids = sorted(by_asset)
    alternatives = [i for i in ids if i != source['runtime_actors'][0]['data_id']]
    donor_id = rng.choice(alternatives or ids)
    options = by_asset[donor_id]
    # Prefer ordinary observation distances over the failed near-blind geometry.
    moderate = [m for m in options if 10 <= m.get('planned_primary_distance_m', 0) <= 60
                and m['runtime_actors'][0].get('planned_surface_clearance_m', 0) >= .5]
    donor = rng.choice(moderate or options)
    spec = copy.deepcopy(donor['spec'])
    spec['seed'] = seed
    ranked = []
    for retry in range(512):
        try:
            scene, meta = ac.fa.gen.build(spec, deck['config'], ctx, retry)
        except ValueError:
            continue
        if not compatible(source, meta, pool_by_id):
            continue
        # Fresh generation, not a copy of any previously observed raw scene.
        payload = hashlib.sha256(json.dumps(scene, ensure_ascii=False, indent=2).encode()).hexdigest()
        if payload in known_shas:
            continue
        proof = ac.proposal_score(meta, calibration, 'all_actor_visibility')
        ranked.append((proof['score'], scene, meta, proof, retry))
        if len(ranked) >= 8:
            break
    if not ranked:
        raise ValueError('No compatible fresh candidate within bounded search')
    _, scene, meta, proxy, retry = max(ranked, key=lambda x: x[0])
    meta['scene_group_id'] = ident
    meta['environment_split_contract'] = dict(environment=deck['config']['environment_tag'],
        scene_group_id=ident, split=meta['split'], policy=POLICY,
        limitation='New geometry retains source split; neighboring links can still share views')
    meta['flexible_completion'] = dict(policy=POLICY, source_cell_id=ident,
        donor_spec_seed=donor['spec']['seed'], donor_primary=donor_id, attempt=ordinal,
        generation_retry=retry, delta=candidate_delta(source, meta), proxy=proxy,
        qa_policy='Unchanged native annotation, visibility, motion, ground, Ego, collision and Park gates')
    return scene, meta


def prepare(source_entry, source_meta, donors, deck, ctx, calibration, ordinal, folder, known_shas, pool_by_id):
    if folder.exists():
        raise FileExistsError('Never overwrite a candidate')
    scene, meta = choose(source_meta, donors, deck, ctx, calibration, ordinal,
                         source_entry['id'], known_shas, pool_by_id)
    folder.mkdir(parents=True)
    sid = source_entry['id'] + f'_fx{ordinal:02}'
    scenario, metadata = folder / (sid + '.json'), folder / (sid + '_metadata.json')
    ac.fa.atomic_json(scenario, scene)
    ac.fa.atomic_json(metadata, meta)
    if ac.fa.sha256(scenario) in known_shas:
        raise ValueError('Candidate repeats a known scenario')
    # The child has its OWN declared contract; never pretend its spec equals the old failed cell.
    entry = dict(id=sid, source_cell_id=sid, scenario=str(scenario.resolve()),
        metadata=str(metadata.resolve()), scenario_sha256=ac.fa.sha256(scenario),
        split=meta['split'], scene_group_id=source_entry['id'], family=meta['spec']['family'],
        mission=meta['spec']['mission'], role=meta['spec']['role'],
        distance_bounds=meta['spec']['distance_bounds'], subfamily=meta['spec'].get('subfamily'),
        motion=meta['runtime_actors'][0]['motion'])
    manifest = folder / 'manifest.json'
    ac.fa.atomic_json(manifest, ac.fa.batch.campaign.subset_deck(deck, [entry]))
    return manifest, entry


def verify(attempt, deck, source_meta, pool_by_id):
    row = ac.verify(attempt, attempt['entry'], deck)
    if row['status'] == 'accepted_candidate':
        meta = ac.fa.read(Path(row['capture_root']) / 'metadata.json')
        if not compatible(source_meta, meta, pool_by_id):
            return dict(status='review', reasons=['flexible_family_motion_contract_changed'], observed_selected=row)
    return row


def selected_rows(state):
    rows = copy.deepcopy(state['baseline_selected'])
    used = {r['source_cell_id'] for r in rows}
    for attempt in state['attempts']:
        result = attempt.get('result')
        if not result or result['status'] != 'accepted_candidate':
            continue
        if attempt['id'] in used:
            raise ValueError('A quota slot cannot be filled twice')
        row = copy.deepcopy(result)
        row.update(source_cell_id=attempt['id'], scene_group_id=attempt['id'],
                   flexible_completion=dict(policy=POLICY, manifest=attempt['manifest'],
                       original_cell_id=attempt['id'], delta=attempt['delta']))
        rows.append(row)
        used.add(attempt['id'])
    rows.sort(key=lambda row: row['source_cell_id'])
    return rows, used


def execute(root, output, run=False, resume=False, attempts=5, time_budget=1800):
    if not 1 <= attempts <= 5 or not 30 <= time_budget <= 3600:
        raise ValueError('Bounded attempt/time limits required')
    parent = root / 'collection_5k_sunny13_motion_contract_v139'
    source = root / 'environment_suite/sunny13/manifest.json'
    if output.exists() and not resume:
        raise FileExistsError('Existing completion requires --resume')
    output.mkdir(parents=True, exist_ok=True)
    locks = []
    state = None
    started = time.monotonic()
    try:
        if repair.execute(root / 'collection_5k_sunny13_adaptive_v137', parent, resume=True):
            raise ValueError('Frozen parent verification failed')
        for path in (output / '.flexible.lock', parent / '.repair.lock'):
            lock = path.open('a'); fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB); locks.append(lock)
        deck = ac.fa.read(source)
        lookup = {e['id']: e for e in deck['scenes']}
        parent_status = ac.fa.read(parent / 'status.json')
        if parent_status['stage'] not in TERMINAL:
            raise ValueError('Parent must be terminal')
        contract = dict(policy=POLICY, parent=str(parent),
            parent_hashes={n: ac.fa.sha256(parent / n) for n in ('state.json', 'selected_ledger.jsonl', 'quota.json', 'validation_report.json')},
            source_sha256=ac.fa.sha256(source), driver_sha256=ac.fa.sha256(__file__),
            capture_contract=ac.fa.batch.campaign.contract(source, list(range(len(deck['scenes']))), 2),
            attempts=attempts)
        sp = output / 'state.json'
        if sp.exists():
            state = ac.fa.read(sp)
            if state['contract'] != contract:
                raise ValueError('Resume contract changed')
            if state['stage'] == 'halted':
                raise ValueError('Safety diagnosis required before halted resume')
        else:
            baseline = ac.fa.load_ledger(parent / 'selected_ledger.jsonl')
            report = ac.fa.read(parent / 'validation_report.json')
            if report['selection_sha256'] != ac.fa.sha256(parent / 'selected_ledger.jsonl'):
                raise ValueError('Parent selection validation changed')
            state = dict(contract=contract, baseline_selected=baseline,
                deficits=[c['id'] for c in ac.fa.read(parent / 'quota.json')['cells']],
                attempts=[], stage='prepared', elapsed_sec=0., current=None)
            ac.fa.atomic_json(sp, state)
        prior = state['elapsed_sec']
        ctx = ac.prewarm(ac.fa.gen.context(ac.fa.gen.geo.DEFAULT_RUNTIME_ASSET_DIR, ac.fa.gen.geo.DEFAULT_MGEO), deck['config'])
        pool_by_id = {a['data_id']: pool for pool in ('vehicle', 'static_vehicle', 'obstacle', 'person', 'dummy', 'animal', 'pet_person', 'stroller', 'rider') for a in ctx['pools'][pool]}
        source_meta = {i: ac.fa.read(lookup[i]['metadata']) for i in state['deficits']}
        for a in state['attempts']:
            if a.get('result') is not None and verify(a, deck, source_meta[a['id']], pool_by_id) != a['result']:
                raise ValueError('Completed capture evidence changed')
        rows, used = selected_rows(state)
        if state['stage'] in TERMINAL:
            if rows != ac.fa.load_ledger(output / 'selected_ledger.jsonl'):
                raise ValueError('Final canonical selection changed')
            return 0

        def save(stage, error=None):
            state.update(stage=stage, error=error, current=None if stage != 'running' else state['current'],
                elapsed_sec=prior + time.monotonic() - started if run else prior)
            ac.fa.atomic_json(sp, state)
            rows, used = selected_rows(state)
            rows.sort(key=lambda r: r['source_cell_id'])
            body = ''.join(json.dumps(r, ensure_ascii=False) + '\n' for r in rows)
            ledger = output / 'selected_ledger.jsonl'
            if not ledger.exists() or ledger.read_text() != body:
                temp = output / '.selected.tmp'; temp.write_text(body); temp.replace(ledger)
            missing = [i for i in state['deficits'] if i not in used]
            ac.fa.atomic_json(output / 'quota.json', dict(target=len(deck['scenes']), accepted=len(rows), deficit=len(missing), pending=0,
                cells=[dict(id=i, reason='fresh_family_replacement_pending_or_review') for i in missing]))
            ac.fa.atomic_json(output / 'deficit_manifest.json', ac.fa.batch.campaign.subset_deck(deck, [lookup[i] for i in missing]))
            ac.fa.atomic_json(output / 'status.json', dict(stage=stage, error=error, mode='failure_aware_replenishment',
                requested=len(deck['scenes']), finished=len(deck['scenes']), accepted_candidate=len(rows), review=len(missing),
                remaining_deficit=len(missing), initial_accepted=len(state['baseline_selected']), replenished=len(rows)-len(state['baseline_selected']),
                elapsed_sec=parent_status['elapsed_sec']+state['elapsed_sec'], replenishment_elapsed_sec=state['elapsed_sec'],
                current=state['current'], updated_at=time.time(), eta_sec=0,
                flexible_completion=True, flexible_attempts=len(state['attempts']),
                accepted_bins=dict(Counter(json.dumps([r.get('family'),r.get('distance_bounds'),r.get('role')]) for r in rows))))
            ac.fa.atomic_json(output / 'replacement_mapping.json', dict(policy=POLICY,
                parent=str(parent), raw_preserved=True, replacements=[dict(source_cell_id=a['id'],
                    capture_root=a['result']['capture_root'], delta=a['delta']) for a in state['attempts']
                    if a.get('result', {}).get('status') == 'accepted_candidate']))
            return used

        used = save(state['stage'])
        if not run:
            return 0
        donors = [ac.fa.read(Path(r['capture_root']) / 'metadata.json') for r in state['baseline_selected']]
        calibration = ac.fa.normalized_sensor_calibration(ac.fa.SENSOR)
        known = {e['scenario_sha256'] for e in deck['scenes']}
        known.update(r['scenario_sha256'] for r in state['baseline_selected'])
        known.update(a['entry']['scenario_sha256'] for a in state['attempts'])

        def heartbeat(child=None):
            status = ac.fa.read(output / 'status.json')
            status.update(updated_at=time.time(), current=str(child) if child else state['current'])
            ac.fa.atomic_json(output / 'status.json', status)

        for ordinal in range(1, attempts + 1):
            for ident in state['deficits']:
                if ident in used:
                    continue
                attempt = next((a for a in state['attempts'] if a['id']==ident and a['ordinal']==ordinal), None)
                if attempt and attempt.get('result') is not None:
                    continue
                if time.monotonic()-started >= time_budget or (output/'PAUSE').exists() or (output/'STOP').exists():
                    save('paused'); return 0
                state['current'] = ident; save('running')
                if attempt is None:
                    folder = output / 'cells' / ident / f'attempt_{ordinal}'
                    manifest, entry = prepare(lookup[ident], source_meta[ident], donors, deck, ctx, calibration,
                        ordinal, folder / 'placement', known, pool_by_id)
                    meta = ac.fa.read(entry['metadata'])
                    attempt = dict(id=ident, ordinal=ordinal, strategy=POLICY,
                        manifest=str(manifest), manifest_sha256=ac.fa.sha256(manifest), entry=entry,
                        capture=str(folder/'capture'), moving=bool(movers(meta)),
                        delta=candidate_delta(source_meta[ident], meta))
                    state['attempts'].append(attempt); known.add(entry['scenario_sha256']); save('running')
                code = ac.run_child(attempt, heartbeat)
                if code:
                    raise RuntimeError('Capture child failed; no automatic safety bypass')
                attempt['result'] = verify(attempt, deck, source_meta[ident], pool_by_id)
                used = save('running')
                print(json.dumps(dict(cell=ident, attempt=ordinal, status=attempt['result']['status'],
                    accepted=len(used), deficit=len(deck['scenes'])-len(used)), ensure_ascii=False), flush=True)
        save('complete' if len(used)==len(deck['scenes']) else 'complete_with_deficit')
        return 0
    except BaseException as exc:
        if state is not None and 'save' in locals():
            save('halted', str(exc))
        else:
            ac.fa.atomic_json(output / 'preflight_error.json', dict(error=str(exc)))
        if isinstance(exc, KeyboardInterrupt):
            return 130
        if not isinstance(exc, Exception):
            raise
        print(str(exc), flush=True)
        return 1
    finally:
        for lock in reversed(locks):
            lock.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ac.DEFAULT_ROOT)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--time-budget-sec', type=int, default=1800)
    args = parser.parse_args()
    raise SystemExit(execute(args.root, args.output or args.root/'collection_5k_sunny13_flexible_v140',
        args.run, args.resume, time_budget=args.time_budget_sec))
