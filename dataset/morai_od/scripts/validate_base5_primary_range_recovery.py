#!/usr/bin/env python3
"""Four registered primary-range recovery paths, not fabricated parent deficits.

CPU preparation is the default. --run permits at most four native children in
this immutable registration. Completed resume verifies raw without reloading.
"""
import argparse
import fcntl
import json
from pathlib import Path
import time

import base5_front_completion_adapter as front
import base5_primary_obstacle_range as policy
import base5_front_release_inputs as pins
from base5_capture_safety import atomic_json, sha256
import prepare_base5_front_support_qualification as full_source
import run_base5_fresh_completion as completion

SCHEMA = 'base5_primary_range_recovery_v1'
TERMINAL = ('complete', 'complete_with_review')


def read(path):
    return json.loads(Path(path).read_text())


def source_inputs(manifest, registration):
    deck, registered = read(manifest), read(registration)
    entries, metadata, _ = full_source.validate_source(deck)
    if registered['source_manifest_sha256'] != sha256(manifest):
        raise ValueError('Range probe registration belongs to another full source')
    strata = registered['strata4']
    if (len(strata) != 4 or len({s['source_id'] for s in strata}) != 4
            or {(s['has_front_plan'], s['cap_band']) for s in strata}
            != {(front_plan, band) for front_plan in (False, True) for band in ('20', '30plus')}):
        raise ValueError('Exactly four registered range/front strata required')
    for row in strata:
        ident = row['source_id']
        original, entry = metadata[ident], entries[ident]
        plan = policy.verify_preserved(original, original)
        if (plan != row['range_plan'] or front.has_plan(original) != row['has_front_plan']
                or entry['scenario_sha256'] != row['source_scenario_sha256']
                or sha256(entry['metadata']) != row['source_metadata_sha256']
                or original['runtime_actors'][0]['data_id'] != row['data_id']):
            raise ValueError('Range probe source identity or plan changed')
    return deck, entries, metadata, strata


def runtime_hashes():
    return pins.collect_runtime(entrypoints=pins.ENTRYPOINTS + (Path(__file__).name,))


def contract(manifest, registration, deck, strata):
    indices = {entry['id']: index for index, entry in enumerate(deck['scenes'])}
    return dict(schema=SCHEMA, source_manifest=str(manifest.resolve()),
        source_manifest_sha256=sha256(manifest), registration=str(registration.resolve()),
        registration_sha256=sha256(registration), strata4=strata, maximum_native_children=4,
        runtime_hashes=runtime_hashes(), planned_probe=True, actual_parent_deficit=False,
        training_data=False, bulk_started=False,
        capture_contract=completion.adaptive.fa.batch.campaign.contract(
            manifest, [indices[row['source_id']] for row in strata], 2))


def snapshot(state):
    files, ledgers, rows = {}, {}, 0
    for attempt in state['attempts']:
        root = Path(attempt['capture'])
        for name in ('ledger.jsonl', 'session_contract.json', 'shutdown_safety.json'):
            path = root / name
            if path.is_file():
                files[str(path.resolve())] = sha256(path)
        ledger = root / 'ledger.jsonl'
        if ledger.is_file():
            ledgers[str(ledger.resolve())] = sha256(ledger)
            rows += sum(bool(json.loads(line).get('capture_root'))
                        for line in ledger.read_text().splitlines() if line)
        for path in (root / 'captures').rglob('*'):
            if path.is_file():
                stat = path.stat()
                files[str(path.resolve())] = dict(sha256=sha256(path), size=stat.st_size,
                    inode=stat.st_ino, mtime_ns=stat.st_mtime_ns)
    return dict(files=files, ledger_sha256=ledgers, captured_rows=rows)


def verify_prepared(output, state, deck, metadata):
    attempts, strata = state['attempts'], state['contract']['strata4']
    if len(attempts) > 4:
        raise ValueError('Never add a fifth native range probe')
    known = {entry['scenario_sha256'] for entry in deck['scenes']}
    for index, attempt in enumerate(attempts):
        row = strata[index]
        ident = row['source_id']
        folder = output / 'probes' / ('%02d_' % (index + 1) + ident)
        if (attempt.get('id') != ident or attempt.get('source_id') != ident
                or attempt.get('stratum') != row or attempt.get('ordinal') != 1
                or attempt.get('moving') is not False
                or Path(attempt['capture']).resolve() != (folder / 'capture').resolve()
                or Path(attempt['manifest']).resolve() != (folder / 'placement/manifest.json').resolve()):
            raise ValueError('Range probe order, identity or child path changed')
        if sha256(attempt['manifest']) != attempt['manifest_sha256']:
            raise ValueError('Prepared range probe manifest changed')
        completion.adaptive.fa.check_placement(attempt['entry'], Path(attempt['manifest']), deck['config'])
        actual = read(attempt['entry']['metadata'])
        policy.verify_preserved(metadata[ident], actual)
        front.verify_preserved(metadata[ident], actual)
        payload = sha256(attempt['entry']['scenario'])
        if payload != attempt['entry']['scenario_sha256'] or payload in known:
            raise ValueError('Range recovery repeats or changes a known payload')
        known.add(payload)


def verify_attempt(attempt, deck, source, pools):
    root = Path(attempt['capture'])
    if not (root / 'shutdown_safety.json').is_file() or read(root / 'shutdown_safety.json').get('park_verified') is not True:
        raise ValueError('Range probe lacks final Park; no next scenario load')
    result = completion.verify(attempt, deck, source, pools)
    if result['status'] not in ('accepted_candidate', 'review', 'failed'):
        raise ValueError('Unrecognized native range probe result')
    return result


def proof(output, state, additional=None):
    inventory = snapshot(state)
    accepted = sum((a.get('result') or {}).get('status') == 'accepted_candidate' for a in state['attempts'])
    doc = dict(schema=SCHEMA, source_manifest_sha256=state['contract']['source_manifest_sha256'],
        registration_sha256=state['contract']['registration_sha256'], strata4=state['contract']['strata4'],
        state_path=str((output / 'state.json').resolve()), state_sha256=sha256(output / 'state.json'),
        prepared_count=len(state['attempts']), accepted_count=accepted,
        passed=state['stage'] == 'complete' and accepted == 4 and additional == 0,
        planned_probe=True, actual_parent_deficit=False, training_data=False, bulk_started=False,
        additional_completed_resume_raw=additional, ledger_sha256=inventory['ledger_sha256'])
    atomic_json(output / 'proof.json', doc)
    return doc


def execute(args):
    output, manifest, registration = args.output.resolve(), args.manifest.resolve(), args.registration.resolve()
    if output.exists() and not args.resume:
        raise FileExistsError('New range recovery output or explicit resume required')
    if output == manifest.parent or not 30 <= args.time_budget_sec <= 1800:
        raise ValueError('Separate output and finite invocation budget required')
    output.mkdir(parents=True, exist_ok=True)
    started, state = time.monotonic(), None
    with (output / '.range_probe.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            deck, entries, metadata, strata = source_inputs(manifest, registration)
            expected = contract(manifest, registration, deck, strata)
            path = output / 'state.json'
            state = read(path) if path.is_file() else dict(contract=expected, stage='preparing', attempts=[])
            if state['contract'] != expected or state['stage'] == 'halted':
                raise ValueError('Range source/runtime changed or unsafe interrupted probe; do not reload')
            verify_prepared(output, state, deck, metadata)
            def save(stage):
                state['stage'] = stage
                atomic_json(path, state)
                proof(output, state)
            ctx = completion.spatial.gen.context(completion.spatial.gen.geo.DEFAULT_RUNTIME_ASSET_DIR,
                                                 completion.spatial.gen.geo.DEFAULT_MGEO)
            pools = completion.pool_map(ctx)
            if state['stage'] in TERMINAL:
                before = snapshot(state)
                if before != state['terminal_inventory']:
                    raise ValueError('Terminal range raw or ledger changed')
                for attempt in state['attempts']:
                    if verify_attempt(attempt, deck, metadata[attempt['id']], pools) != attempt['result']:
                        raise ValueError('Terminal range native result changed')
                after = snapshot(state)
                if after != before:
                    raise ValueError('Completed range resume changed raw')
                proof(output, state, after['captured_rows'] - before['captured_rows'])
                return 0
            if len(state['attempts']) < 4:
                ctx = completion.adaptive.prewarm(ctx, deck['config'])
                calibration = completion.adaptive.fa.normalized_sensor_calibration(completion.adaptive.fa.SENSOR)
                known = {e['scenario_sha256'] for e in deck['scenes']}
                known.update(a['entry']['scenario_sha256'] for a in state['attempts'])
                for index, row in enumerate(strata):
                    if index < len(state['attempts']):
                        continue
                    ident = row['source_id']
                    folder = output / 'probes' / ('%02d_' % (index + 1) + ident) / 'placement'
                    attempt = completion.prepare(entries[ident], metadata[ident], [], deck, ctx,
                        calibration, 1, folder, known, pools)
                    attempt.update(source_id=ident, stratum=row)
                    state['attempts'].append(attempt)
                    known.add(attempt['entry']['scenario_sha256'])
                    save('preparing')
                save('prepared')
            verify_prepared(output, state, deck, metadata)
            if not args.run:
                return 0
            for attempt in state['attempts']:
                ident = attempt['id']
                if attempt.get('result') is not None:
                    if verify_attempt(attempt, deck, metadata[ident], pools) != attempt['result']:
                        raise ValueError('Prior range probe native result changed')
                    continue
                if time.monotonic() - started >= args.time_budget_sec or any((output / name).exists() for name in ('STOP', 'PAUSE')):
                    save('paused')
                    return 0
                pins.verify_hashes(expected['runtime_hashes'], 'Range probe runtime')
                save('running')
                code = None if Path(attempt['capture']).exists() else completion.run_child(attempt, output, lambda root: None)
                result = verify_attempt(attempt, deck, metadata[ident], pools)
                if code and result['status'] not in ('review', 'failed'):
                    raise RuntimeError('Range child failed outside a verified QA/contact boundary')
                attempt['result'] = result
                save('running')
            state['terminal_inventory'] = snapshot(state)
            save('complete' if all(a['result']['status'] == 'accepted_candidate' for a in state['attempts'])
                 else 'complete_with_review')
            return 0
        except BaseException as error:
            if state is not None and state.get('stage') not in TERMINAL:
                state.update(stage='halted', error=str(error))
                atomic_json(output / 'state.json', state)
            atomic_json(output / 'preflight_error.json', dict(error=str(error), no_bulk_capture=True))
            if isinstance(error, KeyboardInterrupt):
                return 130
            if not isinstance(error, Exception):
                raise
            print(str(error), flush=True)
            return 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'registration', 'output'):
        parser.add_argument('--' + name, required=True, type=Path)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--time-budget-sec', type=int, default=1200)
    raise SystemExit(execute(parser.parse_args()))
