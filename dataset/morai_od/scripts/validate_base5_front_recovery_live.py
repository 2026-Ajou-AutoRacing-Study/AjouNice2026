#!/usr/bin/env python3
"""Four registered v151 replacement-path probes, never a fabricated deficit.

Default execution prepares four new geometries on CPU. ``--run`` explicitly
permits one native capture per registered source, with no quality retries or
bulk flag. Original 5K plans, parent collections and failed evidence are read
only. Terminal resume re-verifies native evidence without another ROS child.
"""
import argparse
import copy
import fcntl
import hashlib
import json
from pathlib import Path
import time

import base5_obstacle_front_support as front
import base5_front_completion_adapter as adapter
from base5_capture_safety import atomic_json, sha256
import prepare_base5_front_support_qualification as qualification
import run_base5_fresh_completion as completion


SCHEMA = 'base5_front_recovery_probe_v1'
POLICY = 'four-registered-frozen-front-recovery-probes-v151'
TERMINAL = ('complete', 'complete_with_review')
ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = ROOT / 'artifacts/evaluation/base5_generator_recovery_261015/front_recovery4'


def read(path):
    return json.loads(Path(path).read_text())


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def order(ident):
    return digest([POLICY, ident]), ident


def select_strata(entries, metadata):
    """Register selected/legacy x small/large before any new native outcome."""
    chosen, ids, assets = [], set(), set()
    for selected, large in ((True, False), (False, False), (True, True), (False, True)):
        options = []
        for ident, meta in metadata.items():
            plan = front.validate_plan(meta['spec'])
            if plan and plan['selected'] == selected and plan['large_footprint'] == large:
                options.append(ident)
        options = sorted((ident for ident in options if ident not in ids), key=order)
        distinct = [ident for ident in options
                    if metadata[ident]['spec'][front.PLAN_KEY]['data_id'] not in assets]
        if not options:
            raise ValueError('Four distinct selected/legacy x small/large probe sources required')
        ident = (distinct or options)[0]
        meta, entry = metadata[ident], entries[ident]
        plan = meta['spec'][front.PLAN_KEY]
        actor = meta['runtime_actors'][plan['ordinal']]
        adapter.verify_preserved(meta, meta)
        chosen.append(dict(source_id=ident, selected=selected, large_footprint=large,
            data_id=plan['data_id'], asset=actor['name'],
            source_scenario_sha256=entry['scenario_sha256'],
            source_metadata_sha256=sha256(entry['metadata'])))
        ids.add(ident)
        assets.add(plan['data_id'])
    return chosen


def source_snapshot(manifest):
    deck = read(manifest)
    entries, metadata, _ = qualification.validate_source(deck)
    inventory = [dict(id=entry['id'], scenario_sha256=entry['scenario_sha256'],
                      metadata_sha256=sha256(entry['metadata'])) for entry in deck['scenes']]
    plans = [meta['spec'][front.PLAN_KEY] for meta in metadata.values()
             if front.PLAN_KEY in meta['spec']]
    counts = dict(planned=len(plans), selected=sum(p['selected'] for p in plans),
                  legacy=sum(not p['selected'] for p in plans),
                  assets=len({p['data_id'] for p in plans}))
    return deck, entries, metadata, digest(inventory), counts


def runtime_hashes():
    scripts = Path(__file__).parent
    names = (Path(__file__).name, 'run_base5_fresh_completion.py',
             'base5_front_completion_adapter.py', 'base5_obstacle_front_support.py',
             'base5_obstacle_support_policy.py', 'base5_obstacle_rich_floor.py',
             'base5_spatial_diversity.py', 'run_base5_flexible_completion.py',
             'run_base5_adaptive_completion.py', 'prepare_base5_front_support_qualification.py',
             'run_base5_capture_optimized.py', 'run_base5_capture_campaign_spatial.py',
             'run_base5_batch_campaign.py')
    return {name: sha256(scripts / name) for name in names}


def contract(manifest, deck, entries, metadata, inventory, counts):
    strata = select_strata(entries, metadata)
    positions = {entry['id']: index for index, entry in enumerate(deck['scenes'])}
    return dict(schema=SCHEMA, policy=POLICY,
        source_manifest=str(manifest.resolve()), source_manifest_sha256=sha256(manifest),
        source_inventory_sha256=inventory, full_deck_plan_counts=counts, strata=strata,
        runtime_hashes=runtime_hashes(),
        capture_contract=completion.adaptive.fa.batch.campaign.contract(
            manifest, [positions[row['source_id']] for row in strata], 2),
        planned_probe=True, fabricated_parent_deficit=False, training_data=False,
        maximum_native_captures=4, bulk_started=False)


def raw_inventory(state):
    """Hash owned raw and its native ledgers; do not touch global SensorData."""
    files, ledgers, captures = {}, {}, 0
    for attempt in state['attempts']:
        root = Path(attempt['capture'])
        ledger = root / 'ledger.jsonl'
        if ledger.exists():
            ledgers[str(ledger.resolve())] = sha256(ledger)
            rows = [json.loads(line) for line in ledger.read_text().splitlines() if line]
            captures += sum(bool(row.get('capture_root')) for row in rows)
        for name in ('ledger.jsonl', 'session_contract.json', 'shutdown_safety.json'):
            path = root / name
            if path.exists(): files[str(path.resolve())] = sha256(path)
        for path in sorted((root / 'captures').rglob('*')):
            if path.is_file(): files[str(path.resolve())] = sha256(path)
    return dict(files=files, ledger_sha256map=ledgers, captured_rows=captures)


def verify_attempt(attempt, deck, source, pools):
    """Require the official native verifier and final Park even on reviews."""
    root = Path(attempt['capture'])
    shutdown = root / 'shutdown_safety.json'
    if not shutdown.exists() or read(shutdown).get('park_verified') is not True:
        raise ValueError('Probe child final Park unverified; do not load another scenario')
    result = completion.verify(attempt, deck, source, pools)
    if result['status'] not in ('accepted_candidate', 'review', 'failed'):
        raise ValueError('Unknown native probe outcome')
    return result


def verify_prepared(output, state, deck, metadata):
    """The journal cannot enlarge four registered sources into more captures."""
    attempts, strata = state['attempts'], state['contract']['strata']
    if len(attempts) > 4 or len(strata) != 4:
        raise ValueError('Exactly four registered sources; never a fifth probe')
    known = {entry['scenario_sha256'] for entry in deck['scenes']}
    for index, attempt in enumerate(attempts):
        registered = strata[index]
        ident = registered['source_id']
        folder = output / 'probes' / ('%02d_' % (index + 1) + ident)
        if (attempt.get('source_id') != ident or attempt.get('id') != ident
                or attempt.get('stratum') != registered or attempt.get('ordinal') != 1
                or attempt.get('moving') is not False
                or Path(attempt['capture']).resolve() != (folder / 'capture').resolve()
                or Path(attempt['manifest']).resolve() != (folder / 'placement/manifest.json').resolve()):
            raise ValueError('Prepared probe differs from its registered source/order/path')
        if sha256(attempt['manifest']) != attempt['manifest_sha256']:
            raise ValueError('Prepared probe manifest changed')
        completion.adaptive.fa.check_placement(attempt['entry'], Path(attempt['manifest']), deck['config'])
        actual = read(attempt['entry']['metadata'])
        adapter.verify_preserved(metadata[ident], actual)
        payload = sha256(attempt['entry']['scenario'])
        if payload != attempt['entry']['scenario_sha256'] or payload in known:
            raise ValueError('Prepared probe repeats a source/other probe or changed payload')
        known.add(payload)


def proof(output, state, additional=None):
    inventory = raw_inventory(state)
    accepted = sum((a.get('result') or {}).get('status') == 'accepted_candidate'
                   for a in state['attempts'])
    doc = dict(schema=SCHEMA, policy=POLICY,
        source_manifest=state['contract']['source_manifest'],
        source_manifest_sha256=state['contract']['source_manifest_sha256'],
        source_inventory_sha256=state['contract']['source_inventory_sha256'],
        full_deck_plan_counts=state['contract']['full_deck_plan_counts'],
        state_path=str((output / 'state.json').resolve()), state_sha256=sha256(output / 'state.json'),
        passed=(state['stage'] == 'complete' and len(state['attempts']) == accepted == 4 and additional == 0),
        prepared_count=len(state['attempts']), accepted_count=accepted,
        planned_probe=True, fabricated_parent_deficit=False, training_data=False, bulk_started=False,
        strata=state['contract']['strata'], additional_completed_resume_raw=additional,
        ledger_sha256map=inventory['ledger_sha256map'],
        attempts=[dict(source_id=a['source_id'], manifest=a['manifest'], capture=a['capture'],
                       result_status=(a.get('result') or {}).get('status')) for a in state['attempts']],
        limitation='Four planned replacement-path captures, not actual parent deficit repair, 5K collection or model AP evidence')
    atomic_json(output / 'proof.json', doc)
    return doc


def execute(args):
    output, manifest = args.output.resolve(), args.manifest.resolve()
    if not 30 <= args.time_budget_sec <= 1800:
        raise ValueError('Finite 30..1800 second invocation budget required')
    if output.exists() and not args.resume:
        raise FileExistsError('New probe output or explicit --resume required')
    if output.resolve() == manifest.parent.resolve():
        raise ValueError('Never write probes into the source deck')
    output.mkdir(parents=True, exist_ok=True)
    started, state = time.monotonic(), None
    with (output / '.recovery_probe.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            deck, entries, metadata, inventory, counts = source_snapshot(manifest)
            expected = contract(manifest, deck, entries, metadata, inventory, counts)
            state_path = output / 'state.json'
            if state_path.exists():
                state = read(state_path)
                if state['contract'] != expected:
                    raise ValueError('Recovery probe source/runtime contract changed')
                if state['stage'] == 'halted':
                    raise ValueError('Inspect interrupted/unsafe probe; never automatically reload it')
            else:
                state = dict(contract=expected, stage='preparing', attempts=[], current=None)
                atomic_json(state_path, state)
            verify_prepared(output, state, deck, metadata)

            def save(stage):
                state['stage'] = stage
                atomic_json(state_path, state)
                proof(output, state)

            if state['stage'] in TERMINAL:
                before = raw_inventory(state)
                if before != state['terminal_raw_inventory']:
                    raise ValueError('Terminal probe raw/ledger changed')
                ctx = completion.spatial.gen.context(completion.spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                                     completion.spatial.gen.geo.DEFAULT_MGEO)
                pools = completion.pool_map(ctx)
                for attempt in state['attempts']:
                    actual = verify_attempt(attempt, deck, metadata[attempt['source_id']], pools)
                    if actual != attempt['result']:
                        raise ValueError('Terminal native verification changed')
                after = raw_inventory(state)
                if after != before:
                    raise ValueError('Completed resume changed native raw/ledger')
                additional = after['captured_rows'] - before['captured_rows']
                proof(output, state, additional)
                return 0

            needs_prepare = len(state['attempts']) < 4
            if needs_prepare or args.run:
                ctx = completion.spatial.gen.context(completion.spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                                     completion.spatial.gen.geo.DEFAULT_MGEO)
                if needs_prepare:
                    ctx = completion.adaptive.prewarm(ctx, deck['config'])
                pools = completion.pool_map(ctx)
            if needs_prepare:
                calibration = completion.adaptive.fa.normalized_sensor_calibration(completion.adaptive.fa.SENSOR)
                known = {entry['scenario_sha256'] for entry in deck['scenes']}
                known.update(a['entry']['scenario_sha256'] for a in state['attempts'])
                for index, stratum in enumerate(expected['strata']):
                    if index < len(state['attempts']): continue
                    ident = stratum['source_id']
                    folder = output / 'probes' / ('%02d_' % (index + 1) + ident) / 'placement'
                    attempt = completion.prepare(entries[ident], metadata[ident], [], deck, ctx,
                        calibration, 1, folder, known, pools)
                    if attempt['moving']:
                        raise ValueError('Front replacement probe must remain static')
                    attempt.update(source_id=ident, stratum=copy.deepcopy(stratum))
                    adapter.verify_preserved(metadata[ident], read(attempt['entry']['metadata']))
                    state['attempts'].append(attempt)
                    known.add(attempt['entry']['scenario_sha256'])
                    save('preparing')
                # Read every original again: probes cannot alter the 5K deck or
                # introduce/reallocate an addition in any unplanned source.
                _, _, _, after_inventory, after_counts = source_snapshot(manifest)
                if after_inventory != inventory or after_counts != counts or runtime_hashes() != expected['runtime_hashes']:
                    raise ValueError('Source allocation/runtime changed during CPU preparation')
                save('prepared')
            verify_prepared(output, state, deck, metadata)
            if not args.run:
                return 0

            def heartbeat(root):
                atomic_json(output / 'heartbeat.json', dict(current=str(root), updated_at=time.time()))

            for attempt in state['attempts']:
                ident = attempt['source_id']
                if attempt.get('result') is not None:
                    actual = verify_attempt(attempt, deck, metadata[ident], pools)
                    if actual != attempt['result']:
                        raise ValueError('Previously verified native probe changed')
                    continue
                if time.monotonic() - started >= args.time_budget_sec or any((output / n).exists() for n in ('STOP', 'PAUSE')):
                    state['current'] = None
                    save('paused')
                    return 0
                if runtime_hashes() != expected['runtime_hashes']:
                    raise ValueError('Probe runtime changed before capture')
                state['current'] = ident
                save('running')
                # A leftover child is verification-only: never ask a new ROS
                # process to reload it, even if its prior parent was interrupted.
                if Path(attempt['capture']).exists():
                    code = None
                else:
                    code = completion.run_child(attempt, output, heartbeat)
                result = verify_attempt(attempt, deck, metadata[ident], pools)
                if code and result['status'] not in ('review', 'failed'):
                    raise RuntimeError('Probe child failed outside verified quality/contact boundary')
                attempt['result'] = result
                save('running')
            state['current'] = None
            state['terminal_raw_inventory'] = raw_inventory(state)
            save('complete' if all(a['result']['status'] == 'accepted_candidate' for a in state['attempts'])
                 else 'complete_with_review')
            return 0
        except BaseException as exc:
            if state is not None and state.get('stage') not in TERMINAL:
                state.update(stage='halted', error=str(exc))
                atomic_json(output / 'state.json', state)
            atomic_json(output / 'preflight_error.json', dict(error=str(exc), planned_probe=True,
                fabricated_parent_deficit=False, new_capture_not_authorized_by_this_error=True))
            if isinstance(exc, KeyboardInterrupt): return 130
            if not isinstance(exc, Exception): raise
            print(str(exc), flush=True)
            return 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--prepare-only', action='store_true', help='Default: CPU preparation, no native capture')
    mode.add_argument('--run', action='store_true', help='Explicitly permit at most four registered native captures')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--time-budget-sec', type=int, default=1200)
    raise SystemExit(execute(parser.parse_args()))
