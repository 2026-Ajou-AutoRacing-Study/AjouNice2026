"""Explicit, immutable reuse of independently measured shadow motion ground.

Historical raw is evidence only. Each adoption creates a fresh planned deck;
the normal current collector still captures and audits a new training frame.
"""
import copy
import hashlib
import json
from pathlib import Path
import shutil

from base5_capture_safety import atomic_json, sha256

POLICY = 'explicit-shadow-motion-ground-reuse-v1'
KEY = 'shadow_motion_ground_reuse'
ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SENSOR = ROOT / 'configs/sensors/Sensor_competition_local.json'
IDENTITY = ('unique_id', 'data_id', 'name', 'source_list', 'motion', 'requested_motion', 'population_pool')
GROUND = 'independent-terrain-sampled-v46'


class ShadowReuseContractError(RuntimeError):
    """A provenance error must halt, never become an ordinary quota deficit."""


def read(path):
    return json.loads(Path(path).read_text())


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def pin(path):
    path = Path(path).resolve()
    return dict(path=str(path), sha256=sha256(path))


def _add(pins, path, expected=None):
    item = pin(path)
    if expected is not None and item['sha256'] != expected:
        raise ShadowReuseContractError('Shadow preparation evidence changed: ' + item['path'])
    if item['path'] in pins and pins[item['path']] != item['sha256']:
        raise ShadowReuseContractError('Shadow preparation changed during registration')
    pins[item['path']] = item['sha256']
    return item


def _moving(meta):
    return [a for a in meta.get('runtime_actors', []) if a.get('motion') not in ('static', 'standing')]


def _scope(meta):
    if (meta.get('calibration_only') or meta.get('spec', {}).get('family') != 'shadow'
            or not meta['spec'].get('shadow_moving') or not _moving(meta)):
        raise ShadowReuseContractError('Reuse requires an actual shadow-moving training source')


def _identity(meta):
    return [{k: a.get(k) for k in IDENTITY} for a in meta['runtime_actors']]


def _single(path, pins):
    _add(pins, path)
    deck = read(path)
    if deck.get('count') != 1 or len(deck.get('scenes', [])) != 1:
        raise ShadowReuseContractError('Prepared/input deck must be single-scene')
    row = deck['scenes'][0]
    _add(pins, row['scenario'], row['scenario_sha256'])
    _add(pins, row['metadata'])
    return deck, row, read(row['metadata']), read(row['scenario'])


def _ledger(path, pins):
    _add(pins, path)
    payload = Path(path).read_bytes()
    if not payload or not payload.endswith(b'\n'):
        raise ShadowReuseContractError('Historical ledger must be complete, never repaired')
    rows = [json.loads(line) for line in payload.splitlines()]
    if not rows or any(not isinstance(row, dict) for row in rows):
        raise ShadowReuseContractError('Historical ledger contains malformed rows')
    return rows


def _safe_collision(root, audit):
    summary = read(Path(root) / 'collision_history.json').get('summary', {})
    if (summary.get('observed') is not True or summary.get('no_collision_verified') is not True
            or summary.get('non_ego_uids') != []
            or audit.get('capture_safety', {}).get('collision') != summary):
        raise ShadowReuseContractError('Historical preparation collision safety is unverified')


def _session_binding(session, root, row, manifest, pins):
    """Bind a known explicit child deck to the historical completed session."""
    contract = read(session / 'session_contract.json')
    _add(pins, manifest, contract['manifest_sha256'])
    _, entry, expected, _ = _single(manifest, pins)
    raw = read(root / 'metadata.json')
    if (entry['id'] != row['id'] or entry['scenario_sha256'] != row['scenario_sha256']
            or any(raw.get(k) != v for k, v in expected.items())):
        raise ShadowReuseContractError('Historical raw differs from its frozen launch manifest')


def _session(root, source_sha, before, sensor_sha, pins):
    """Read old contracts, never resume them with current code."""
    root = Path(root).resolve()
    session = root.parents[1]
    for name in ('ledger.jsonl', 'session_contract.json', 'shutdown_safety.json'):
        _add(pins, session / name)
    ledger = _ledger(session / 'ledger.jsonl', pins)
    if (len(ledger) != 1 or ledger[0].get('status') not in ('accepted_candidate', 'review')
            or Path(ledger[0]['capture_root']).resolve() != root
            or read(session / 'shutdown_safety.json').get('park_verified') is not True
            or read(session / 'session_contract.json').get('sensor_sha256') != sensor_sha):
        raise ShadowReuseContractError('Preparation session/ledger/Park/sensor mismatch')
    for name in ('scenario.json', 'metadata.json', 'audit.json', 'telemetry.json',
                 'sensor_reference.json', 'collision_history.json'):
        _add(pins, root / name)
    if sha256(root / 'scenario.json') != ledger[0]['scenario_sha256']:
        raise ShadowReuseContractError('Preparation raw scenario changed')
    if sha256(root / 'sensor_reference.json') != sensor_sha:
        raise ShadowReuseContractError('Preparation sensor reference differs from current sensor')
    # These are the quality loop's explicit, local sibling names, not a global
    # search or geometry-based cache match. Unknown historical layouts fail.
    if session.name in ('motion_terrain', 'motion_static'):
        child = session.parent / (session.name + '_deck') / 'manifest.json'
    elif session.name.startswith('motion_near_terrain_') and session.name[20:].isdigit():
        child = session.parent / ('motion_near_terrain_deck_' + session.name[20:]) / 'manifest.json'
    else:
        raise ShadowReuseContractError('Unknown historical calibration manifest layout')
    _session_binding(session, root, ledger[0], child, pins)
    meta, audit = read(root / 'metadata.json'), read(root / 'audit.json')
    _safe_collision(root, audit)
    probe = meta.get('ground_probe_source')
    expected_spec = copy.deepcopy(before['spec'])
    if probe:
        expected_spec.update(family='background', actors=0, motion='static')
    if (meta.get('calibration_only') is not True or meta.get('sensor_source_sha256') != sensor_sha
            or meta.get('spec') != expected_spec
            or any('collision' in str(r) or 'ego_vertical' in str(r) for r in audit.get('review_reasons', []))
            or ledger[0].get('audit') != audit):
        raise ShadowReuseContractError('Unsafe/foreign preparation raw evidence')
    if probe:
        targets = {a['unique_id']: a for a in before['runtime_actors']}
        if (probe.get('scenario_sha256') != source_sha or meta.get('runtime_actors')
                or not probe.get('targets')
                or any(targets.get(a['unique_id']) != a for a in probe['targets'])):
            raise ShadowReuseContractError('Terrain belongs to another source scenario')
        return 'terrain'
    identities = lambda m: [{k: a.get(k) for k in IDENTITY if k != 'motion'}
                           for a in m.get('runtime_actors', [])]
    if (meta.get('static_motion_reference') is not True
            or identities(meta) != identities(before)
            or any(a.get('motion') != 'standing' for a in meta['runtime_actors'])
            or meta.get('ego') != before.get('ego')):
        raise ShadowReuseContractError('Unknown preparation capture purpose')
    return 'static_reference'


def _selected(selection, original, prepared, meta, sensor_sha, pins):
    """Bind explicit historical selection to its completed capture, not raw reuse."""
    rows = _ledger(selection['selected_ledger'], pins)
    rows = [row for row in rows if row.get('scenario_sha256') == prepared['scenario_sha256']]
    if (len(rows) != 1 or rows[0].get('status') != 'accepted_candidate'
            or rows[0].get('id') != prepared['id']
            or rows[0].get('source_cell_id', rows[0]['id']) != original['id']
            or rows[0].get('audit', {}).get('review_reasons')):
        raise ShadowReuseContractError('Explicit historical selected ledger does not approve this source')
    row = rows[0]
    root = Path(row['capture_root']).resolve()
    session = root.parents[1]
    local = _ledger(session / 'ledger.jsonl', pins)
    if len(local) != 1 or any(local[0].get(k) != row.get(k)
            for k in ('id', 'status', 'scenario_sha256', 'capture_root', 'audit')):
        raise ShadowReuseContractError('Historical selected/local completed ledger disagrees')
    for name in ('session_contract.json', 'shutdown_safety.json'):
        _add(pins, session / name)
    for name in ('scenario.json', 'metadata.json', 'audit.json', 'telemetry.json',
                 'sensor_reference.json', 'collision_history.json'):
        _add(pins, root / name)
    _session_binding(session, root, row, selection['prepared_manifest'], pins)
    raw = read(root / 'metadata.json')
    _safe_collision(root, read(root / 'audit.json'))
    if (sha256(root / 'scenario.json') != prepared['scenario_sha256']
            or sha256(root / 'sensor_reference.json') != sensor_sha
            or read(session / 'session_contract.json').get('sensor_sha256') != sensor_sha
            or read(session / 'shutdown_safety.json').get('park_verified') is not True
            or raw.get('sensor_source_sha256') != sensor_sha
            or any(raw.get(k) != v for k, v in meta.items())
            or read(root / 'audit.json') != row['audit']):
        raise ShadowReuseContractError('Historical approved capture provenance changed')
    return pin(selection['selected_ledger'])


def _selection(source, original, selection, sensor_sha, pins):
    mode = selection['mode']
    if mode not in ('exact_source', 'same_quota_replacement'):
        raise ShadowReuseContractError('Adoption mode must be explicit')
    _add(pins, original['scenario'], original['scenario_sha256'])
    _add(pins, original['metadata'])
    old = read(original['metadata'])
    _scope(old)
    prepared = Path(selection['prepared_manifest']).resolve()
    prepared_deck, row, meta, scene = _single(prepared, pins)
    # This exact input manifest was written by the original quality loop,
    # adjacent to motion_ground_deck; it binds the measured terrain to XY.
    input_manifest = prepared.parent.parent / 'manifest.json'
    _, input_row, before, _ = _single(input_manifest, pins)
    _scope(meta)
    _scope(before)
    if (source['config']['environment_tag'] != prepared_deck['config']['environment_tag']
            or old['spec'] != before['spec'] or old['spec'] != meta['spec']
            or old['split'] != before['split'] or old['split'] != meta['split']
            or _identity(old) != _identity(before) or _identity(old) != _identity(meta)
            or any(old.get(k) != meta.get(k) for k in ('scene_group_id', 'environment_split_contract',
                'logical_quota_slot_id', 'fresh_environment_provenance'))):
        raise ShadowReuseContractError('Prepared spec/asset/motion/split/group differs from source')
    exact = (input_row['scenario_sha256'] == original['scenario_sha256']
             and sha256(input_row['metadata']) == sha256(original['metadata']))
    if mode == 'exact_source' and not exact:
        raise ShadowReuseContractError('Exact reuse requires the original source payload and metadata')
    if mode == 'same_quota_replacement' and exact:
        raise ShadowReuseContractError('Unchanged source must use exact_source mode')
    from base5_motion_ground import validate_motion_ground
    validate_motion_ground(meta, scene)
    files = {}
    for actor in _moving(meta):
        proof = actor.get('motion_path', {}).get('ground_path_evidence', {})
        if proof.get('policy') != GROUND:
            raise ShadowReuseContractError('Only independently measured ground paths are reusable')
        for evidence in proof['files']:
            _add(pins, evidence['path'], evidence['sha256'])
            files[evidence['path']] = evidence['sha256']
    purposes = []
    for path in files:
        if Path(path).name == 'metadata.json':
            purposes.append(_session(Path(path).parent, input_row['scenario_sha256'],
                before, sensor_sha, pins))
    if not {'terrain', 'static_reference'} <= set(purposes):
        raise ShadowReuseContractError('Terrain and independent static reference are both required')
    selected_ledger = _selected(selection, original, row, meta, sensor_sha, pins)
    return dict(source_id=original['id'], mode=mode, source_entry=copy.deepcopy(original),
        source_metadata_sha256=sha256(original['metadata']), prepared_manifest=pin(prepared),
        input_manifest=pin(input_manifest), prepared_entry=copy.deepcopy(row),
        prepared_metadata_sha256=sha256(row['metadata']), terrain_input_scenario_sha256=input_row['scenario_sha256'],
        actor_identity=_identity(old), frozen_spec_sha256=digest(old['spec']),
        selected_ledger=selected_ledger,
        calibration_capture_count=len(purposes), proof_file_count=len(files))


def register(source_manifest, selections, output, sensor=DEFAULT_SENSOR):
    """Create a new explicit registration and derived-config manifest, offline."""
    try:
        source_manifest, output = Path(source_manifest).resolve(), Path(output).resolve()
        if output.exists():
            raise ShadowReuseContractError('Registration requires a fresh output; old ledgers are immutable')
        source = read(source_manifest)
        if KEY in source.get('config', {}):
            raise ShadowReuseContractError('Source already carries a reuse registration')
        originals = {r['id']: r for r in source['scenes']}
        if len(originals) != len(source['scenes']) or not selections:
            raise ShadowReuseContractError('Unique source IDs and explicit nonempty mapping required')
        pins = {}
        _add(pins, source_manifest)
        sensor_pin = _add(pins, sensor)
        result = []
        for item in selections:
            if item['source_id'] not in originals or item['source_id'] in {r['source_id'] for r in result}:
                raise ShadowReuseContractError('Missing/duplicate explicit source mapping')
            result.append(_selection(source, originals[item['source_id']], item, sensor_pin['sha256'], pins))
        registry = dict(policy=POLICY, helper_sha256=sha256(__file__), source_manifest=pin(source_manifest),
            environment_tag=source['config']['environment_tag'], sensor=sensor_pin,
            entries=result, input_sha256=dict(sorted(pins.items())), old_raw_preserved=True,
            old_sessions_resumed=False, new_native_capture_required=True, bulk_started=False)
        if any(sha256(path) != expected for path, expected in pins.items()):
            raise ShadowReuseContractError('Preparation source changed during registration')
        atomic_json(output / 'registry.json', registry)
        configured = copy.deepcopy(source)
        configured['config'][KEY] = dict(policy=POLICY, helper_sha256=sha256(__file__),
            registration=pin(output / 'registry.json'))
        atomic_json(output / 'manifest.json', configured)
        return output / 'manifest.json'
    except ShadowReuseContractError:
        raise
    except (OSError, ValueError, TypeError, KeyError, IndexError, StopIteration) as error:
        raise ShadowReuseContractError('Malformed shadow reuse registration: ' + str(error)) from error


def _registered(deck, sensor=None):
    config = deck.get('config', {}).get(KEY)
    if config is None:
        return None, {}
    try:
        if config['policy'] != POLICY or config['helper_sha256'] != sha256(__file__):
            raise ShadowReuseContractError('Shadow reuse helper/policy changed')
        reference = config['registration']
        if sha256(reference['path']) != reference['sha256']:
            raise ShadowReuseContractError('Shadow reuse registration changed')
        registry = read(reference['path'])
        if (registry['policy'] != POLICY or registry['helper_sha256'] != config['helper_sha256']
                or registry['environment_tag'] != deck['config']['environment_tag']
                or registry['sensor']['sha256'] != sha256(sensor or registry['sensor']['path'])):
            raise ShadowReuseContractError('Shadow reuse environment/sensor/policy mismatch')
        pins = registry['input_sha256']
        if not pins or any(sha256(path) != expected for path, expected in pins.items()):
            raise ShadowReuseContractError('Shadow reuse frozen input/terrain/session/ledger changed')
        return registry, dict(pins, **{str(Path(reference['path']).resolve()): reference['sha256']})
    except ShadowReuseContractError:
        raise
    except (OSError, ValueError, TypeError, KeyError) as error:
        raise ShadowReuseContractError('Malformed shadow reuse policy: ' + str(error)) from error


def _provenance(config, row):
    return dict(policy=POLICY, registration=copy.deepcopy(config['registration']),
        helper_sha256=config['helper_sha256'], source_id=row['source_id'], mode=row['mode'],
        source_scenario_sha256=row['source_entry']['scenario_sha256'],
        source_metadata_sha256=row['source_metadata_sha256'],
        prepared_manifest=copy.deepcopy(row['prepared_manifest']),
        historical_raw_is_not_new_training_data=True, native_qa_unchanged=True)


def _metadata(config, row):
    meta = read(row['prepared_entry']['metadata'])
    meta[KEY] = _provenance(config, row)
    return meta


def _without_verified_review_overlays(meta, entry, deck):
    """Validate only explicit derived QA/split tags; protect every ground field.

    Historical untagged inputs still take the original byte-hash path. New
    overlays cannot change actors, prepared paths, source splits or ground
    provenance. This helper never authorizes an actual geometry transition.
    """
    result=copy.deepcopy(meta)
    unit_key='native_unit_geometry_reference'
    if unit_key in result or deck.get('config',{}).get(unit_key) is not None:
        from base5_unit_geometry_contract import validate_entry as validate_unit
        validate_unit(entry,deck)
        result.pop(unit_key,None)
    if 'training_split_overlay' in result or deck.get('split_overlay') is not None:
        from base5_layout_split import overlay_for_metadata,resolve
        reference=deck.get('split_overlay')
        tag=result.get('training_split_overlay')
        if reference is None or tag is None or any(tag.get(k)!=reference.get(k) for k in ('path','sha256')):
            raise ShadowReuseContractError('Shadow derived split reference differs')
        overlay=overlay_for_metadata(meta)
        assignment=resolve(overlay,entry['id'],entry,meta)
        if (entry.get('derived_training_split')!=assignment['split']
                or entry.get('derived_layout_group_id')!=assignment['group_id']):
            raise ShadowReuseContractError('Shadow derived split/group differs')
        result.pop('training_split_overlay')
    return result


def validate_manifest(deck, sensor=None):
    """Opt-in preflight for fresh, adopted and completed-resume manifests."""
    registry, pins = _registered(deck, sensor)
    if registry is None:
        return {}
    try:
        index = {r['source_id']: r for r in registry['entries']}
        if len(index) != len(registry['entries']):
            raise ShadowReuseContractError('Duplicate registered shadow source')
        for entry in deck['scenes']:
            meta = read(entry['metadata'])
            row = index.get(entry['id'])
            if row is None:
                if KEY in meta:
                    raise ShadowReuseContractError('Foreign source claims prepared shadow adoption')
                continue
            if sha256(entry['scenario']) != entry['scenario_sha256']:
                raise ShadowReuseContractError('Shadow source scenario checksum mismatch')
            clean_meta=_without_verified_review_overlays(meta,entry,deck)
            ignored={'scenario','metadata','scenario_sha256'}
            if deck.get('split_overlay') is not None:
                ignored.update(('derived_training_split','derived_layout_group_id'))
            quota = lambda r: {k: v for k, v in r.items() if k not in ignored}
            if quota(entry) != quota(row['source_entry']):
                raise ShadowReuseContractError('Registered source entry quota/identity changed')
            if KEY in meta:
                if (entry['scenario_sha256'] != row['prepared_entry']['scenario_sha256']
                        or clean_meta != _metadata(deck['config'][KEY], row)):
                    raise ShadowReuseContractError('Adopted shadow plan or metadata changed')
            else:
                exact_metadata=(clean_meta==read(row['source_entry']['metadata'])
                    if clean_meta!=meta else sha256(entry['metadata'])==row['source_metadata_sha256'])
                if (entry['scenario_sha256']!=row['source_entry']['scenario_sha256'] or not exact_metadata):
                    raise ShadowReuseContractError('Registered source differs from frozen input')
        return pins
    except ShadowReuseContractError:
        raise
    except (OSError, ValueError, TypeError, KeyError) as error:
        raise ShadowReuseContractError('Malformed shadow source/adoption: ' + str(error)) from error


def verify_capture(entry, capture, deck):
    """Completed-resume may add runtime observations, never alter adopted inputs."""
    if KEY not in deck.get('config', {}):
        return
    validate_manifest(deck)
    try:
        raw = read(Path(capture) / 'metadata.json')
        expected = read(entry['metadata']) if entry is not None else {}
        if KEY not in expected:
            if KEY in raw:
                raise ShadowReuseContractError('Foreign completed capture claims shadow adoption')
            return
        sensor_sha = read(deck['config'][KEY]['registration']['path'])['sensor']['sha256']
        if (entry is None or any(raw.get(k) != v for k, v in expected.items())
                or raw.get('sensor_source_sha256') != sensor_sha
                or sha256(Path(capture) / 'sensor_reference.json') != sensor_sha
                or sha256(Path(capture) / 'scenario.json') != entry['scenario_sha256']):
            raise ShadowReuseContractError('Completed adopted capture input/provenance changed')
    except ShadowReuseContractError:
        raise
    except (OSError, ValueError, TypeError, KeyError) as error:
        raise ShadowReuseContractError('Malformed completed adoption capture: ' + str(error)) from error


def adopt(source_manifest, output):
    """Return a fresh planned deck or None; no old capture/ledger is copied."""
    try:
        source_manifest, output = Path(source_manifest), Path(output).resolve()
        deck = read(source_manifest)
        validate_manifest(deck)
        registry, _ = _registered(deck)
        if registry is None:
            return None
        if len(deck['scenes']) != 1:
            raise ShadowReuseContractError('Shadow adoption is a single-cell operation')
        original = deck['scenes'][0]
        row = next((r for r in registry['entries'] if r['source_id'] == original['id']), None)
        if row is None:
            return None
        metadata = _metadata(deck['config'][KEY], row)
        current_meta=read(original['metadata'])
        if KEY in current_meta:
            # Already materialized before split allocation: retain the exact
            # validated payload and tags instead of adopting a second geometry.
            return source_manifest
        for tag in ('native_unit_geometry_reference','training_split_overlay'):
            if tag in current_meta:metadata[tag]=copy.deepcopy(current_meta[tag])
        if output.exists():
            if (not (output / 'manifest.json').is_file()
                    or any((output / name).exists() for name in
                        ('ledger.jsonl', 'selected_ledger.jsonl', 'session_contract.json'))):
                raise ShadowReuseContractError('Incomplete adoption output must not be overwritten')
            existing = read(output / 'manifest.json')
            validate_manifest(existing)
            if (existing['config'] != deck['config'] or len(existing['scenes']) != 1
                    or existing['scenes'][0]['id'] != original['id']
                    or KEY not in read(existing['scenes'][0]['metadata'])):
                raise ShadowReuseContractError('Adoption resume source/config differs')
            return output / 'manifest.json'
        row_out = copy.deepcopy(original)
        scenario = output / 'scenario.json'
        sidecar = output / 'metadata.json'
        output.mkdir(parents=True)
        shutil.copyfile(row['prepared_entry']['scenario'], scenario)
        if sha256(scenario) != row['prepared_entry']['scenario_sha256']:
            raise ShadowReuseContractError('Prepared scenario copy changed')
        atomic_json(sidecar, metadata)
        row_out.update(scenario=str(scenario), metadata=str(sidecar), scenario_sha256=sha256(scenario))
        adopted = copy.deepcopy(deck)
        adopted.update(scenes=[row_out], count=1)
        atomic_json(output / 'manifest.json', adopted)
        validate_manifest(adopted)
        return output / 'manifest.json'
    except ShadowReuseContractError:
        raise
    except (OSError, ValueError, TypeError, KeyError, IndexError) as error:
        raise ShadowReuseContractError('Shadow ground adoption failed: ' + str(error)) from error
