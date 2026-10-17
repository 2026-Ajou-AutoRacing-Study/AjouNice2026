"""Normalize only fresh quality-loop preparation decks for a layout source.

Calibration is explicitly outside the training overlay and cannot fill a quota.
Actual training geometry changes are delegated to the canonical recovery API.
No source/preparation/raw file or native annotation is rewritten.
"""
import copy
import json
import math
from pathlib import Path
import shutil

import base5_front_release_inputs as pins
import base5_layout_split as layout
import base5_scale_capture_contract as scale
import base5_shadow_ground_reuse as shadow
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json, sha256

KEY = 'calibration_layout_provenance'
TRAINING_KEY = 'prepared_layout_provenance'
SCHEMA = 'base5_quality_preparation_layout_v1'
CONTAINERS = ('objectList', 'vehicleList', 'pedestrianList', 'aiPedstrianList')


class CalibrationLayoutError(RuntimeError):
    """Source/QA/preparation damage is fatal, never an ordinary deficit."""


def require(condition, message):
    if not condition:
        raise CalibrationLayoutError(message)


def read(path):
    return json.loads(Path(path).read_text())


def add_pin(inventory, path, expected=None):
    path = Path(path).resolve(strict=True)
    require(path.is_file(), 'Preparation requires an existing regular file')
    digest = sha256(path)
    require(expected is None or digest == expected, 'Preparation/source SHA differs: '+str(path))
    require(str(path) not in inventory or inventory[str(path)] == digest, 'Preparation input changed')
    inventory[str(path)] = digest
    return path


def raw_unit(scene, metadata, catalog):
    actual = {};counts = {}
    for container in CONTAINERS:
        values = scene.get(container, [])
        require(isinstance(values, list), 'Explicit actor list required')
        for actor in values:
            uid, data_id = actor.get('UNIQUEID'), actor.get('DataID')
            require(type(uid) is int and type(data_id) is int and uid not in actual,
                'Unique raw actor UID/DataID required')
            actual[uid] = (data_id, container)
            if 'scale' not in actor:
                require(container != 'objectList', 'Physical/objectList requires explicit111')
                key = container+'/serializer_default_no_override'
            else:
                value = actor['scale']
                require(isinstance(value, dict) and all(type(value.get(a)) in (int,float)
                    and math.isfinite(value[a]) and value[a] == 1. for a in 'xyz'), 'Prepared actors must remain unit111')
                for axis in 'xyz':
                    if '_'+axis in value:
                        mirror = value['_'+axis]
                        try:valid = type(mirror) in (str,int,float) and math.isfinite(float(mirror)) and float(mirror) == 1.
                        except (ValueError, TypeError, OverflowError):valid = False
                        require(valid, 'Prepared scale UI mirror differs from111')
                key = container+'/explicit111'
            counts[key] = counts.get(key,0)+1
    expected = {}
    for actor in metadata.get('runtime_actors', []):
        uid, data_id = actor.get('unique_id'), actor.get('data_id')
        asset = catalog.get(data_id,{})
        require(type(uid) is int and uid not in expected and asset.get('name') == actor.get('name')
            and asset.get('source_list') == actor.get('source_list'), 'Prepared actor catalog identity differs')
        require(not actor.get('asset_scale') and not actor.get('asset_scale_geometry'), 'Unit-only preparation cannot gain a scale assignment')
        expected[uid] = (data_id,actor['source_list'])
    require(actual == expected, 'Prepared raw and metadata actor inventories differ')
    for container in CONTAINERS:
        require([a['UNIQUEID'] for a in scene.get(container,[])] ==
            [a['unique_id'] for a in metadata.get('runtime_actors',[]) if a['source_list'] == container],
            'Prepared raw and metadata ordered actor inventories differ')
    return counts


def normalize_prepared_deck(prepared_manifest, source_deck, source_entry, output, stage_kind,
                            *, bind_training=None, context=None):
    """Return a newly bound deck, a verified cached deck, or an already-bound input.

    ``source_deck/source_entry`` are the immutable training source for this cell.
    A caller handling nested replacements supplies its already-verified canonical
    ``context`` and the public quality-loop ``bind_training`` callback.
    """
    prepared_manifest = Path(prepared_manifest).resolve(strict=True)
    if source_deck.get('split_overlay') is None:
        return prepared_manifest  # historical untagged workflows retain their contract
    output = Path(output).resolve()
    require(isinstance(stage_kind,str) and bool(stage_kind.strip()), 'Explicit preparation stage required')
    require(output != prepared_manifest.parent and prepared_manifest.parent not in output.parents,
        'Normalization must not write inside preserved preparation')
    require(source_entry in source_deck.get('scenes',[]), 'Frozen source entry is not in its declared deck')
    inventory = {}
    add_pin(inventory,source_entry['scenario'],source_entry['scenario_sha256'])
    add_pin(inventory,source_entry['metadata'])
    source_meta = read(source_entry['metadata'])
    layout.validate_manifest_overlay(source_deck,allow_subset=True)
    source_overlay = layout.overlay_for_metadata(source_meta)
    source_binding = layout.resolve(source_overlay,source_entry['id'],source_entry,source_meta)
    expected = source_deck.get('config',{}).get(unit.KEY)
    require(expected is not None and source_meta.get(unit.KEY) == expected, 'Pinned source unit QA required')
    policy = unit.load(expected)
    for reference in [expected,policy['catalog_reference'],*policy['sources']]:
        add_pin(inventory,reference['path'],reference['sha256'])
    catalog = {a['data_id']:a for a in read(policy['catalog_reference']['path'])['assets']}
    add_pin(inventory,source_deck['split_overlay']['path'],source_deck['split_overlay']['sha256'])
    require(not source_deck['config'].get('asset_scale_policy'), 'Only unit source preparation is supported')
    prepared_path = add_pin(inventory,prepared_manifest)
    deck = read(prepared_path)
    require(deck.get('count') == len(deck.get('scenes',[])) == 1, 'Single-cell preparation required')
    entry = deck['scenes'][0];meta = read(add_pin(inventory,entry['metadata']))
    scene = read(add_pin(inventory,entry['scenario'],entry['scenario_sha256']))
    require(entry.get('split') == meta.get('split') == source_entry['split'] == source_meta['split'],
        'Preparation must preserve its original capture/source split')
    require(deck['config'].get(unit.KEY) == expected and meta.get(unit.KEY) in (None,expected), 'Prepared unit QA differs')
    source_config, prepared_config = copy.deepcopy(source_deck['config']), copy.deepcopy(deck['config'])
    old_budget = source_config.get('capture',{}).pop('natural_setup_budget_sec',None)
    new_budget = prepared_config.get('capture',{}).pop('natural_setup_budget_sec',None)
    require(source_config == prepared_config, 'Preparation changed a frozen non-budget config')
    require(old_budget == new_budget or (meta.get('setup_budget_pilot',{}).get('budget_sec') == new_budget
        and type(old_budget) in (int,float) and type(new_budget) in (int,float)
        and old_budget <= new_budget <= 9), 'Undeclared preparation budget change')
    counts = raw_unit(scene,meta,catalog)
    require(not meta.get('scale_probe') and not meta.get('asset_scale_preparation')
        and not meta.get('spec',{}).get('asset_scale_plan'), 'Unit preparation cannot gain scale/probe metadata')
    calibration = meta.get('calibration_only') is True
    if calibration:
        identity = lambda actor: tuple(actor.get(key) for key in ('unique_id','data_id','name','source_list'))
        source_actors = {identity(actor) for actor in source_meta.get('runtime_actors',[])}
        require(all(identity(actor) in source_actors for actor in meta.get('runtime_actors',[])),
            'Calibration actor must be an exact source UID/DataID/name/container subset')
    elif entry['scenario_sha256'] == source_entry['scenario_sha256']:
        require(meta.get('ego') == source_meta.get('ego')
            and meta.get('runtime_actors') == source_meta.get('runtime_actors'),
            'Identical payload QA-only preparation changed exact Ego/ordered actor metadata')
        fixed = ('spec','base','scene_group_id','environment_split_contract','logical_quota_slot_id',
            'fresh_environment_provenance','obstacle_support_pair')
        require(all(meta.get(key) == source_meta.get(key) for key in fixed),
            'Identical payload QA-only preparation changed frozen spec/base/source quota metadata')
        paths_or_derived = {'id','scenario','metadata','source_cell_id','derived_training_split','derived_layout_group_id'}
        require({k:v for k,v in entry.items() if k not in paths_or_derived} ==
            {k:v for k,v in source_entry.items() if k not in paths_or_derived},
            'Identical payload QA-only preparation changed frozen source entry quota')
    if not calibration and deck.get('split_overlay') is not None and meta.get('training_split_overlay') is not None:
        # A replacement may already have its correct canonical recovery overlay.
        # Do not undo/rebind a successful earlier normalization.
        try:
            unit.validate_entry(entry,deck)
            layout.validate_manifest_overlay(deck,allow_subset=True)
            bound = layout.overlay_for_metadata(meta)
            actual = layout.resolve(bound,entry['id'],entry,meta)
            canonical = (context or {}).get('root_entry',source_entry)['id']
            require(bound.get('recovery_aliases',{}).get(entry['id'],entry['id']) == canonical
                and actual['split'] == source_binding['split'] and actual['group_id'] == source_binding['group_id'],
                'Prepared binding changed its canonical source split/group')
            pins.verify_hashes(inventory,'Already-bound preparation inputs')
            return prepared_manifest
        except (ValueError,unit.UnitGeometryContractError):
            pass
    tag = meta.get('training_split_overlay')
    if tag is not None:
        inherited = layout.overlay_for_metadata(meta)  # SHA/source-pretag pins, not changed geometry approval
        layout.verify_source_pins(inherited)
        canonical = (context or {}).get('root_entry',source_entry)['id']
        require(canonical in inherited['assignments'] and inherited['assignments'][canonical]['group_id'] == source_binding['group_id']
            and inherited['assignments'][canonical]['split'] == source_binding['split'], 'Inherited overlay belongs to another source')
        add_pin(inventory,tag['path'],tag['sha256'])
        add_pin(inventory,tag['source_metadata_path'],tag['source_metadata_sha256'])
    require(not calibration or (tag is not None and meta.get(unit.KEY) == expected),
        'Calibration must preserve its explicit source overlay and unit reference before detachment')
    request = dict(schema=SCHEMA,helper_sha256=sha256(__file__),stage_kind=stage_kind,
        source_entry=copy.deepcopy(source_entry),source_metadata_sha256=inventory[str(Path(source_entry['metadata']).resolve())],
        source_deck_digest=layout.digest(source_deck),source_overlay=copy.deepcopy(source_deck['split_overlay']),
        source_effective_split=source_binding['split'],source_layout_group_id=source_binding['group_id'],
        prepared_manifest=str(prepared_manifest),prepared_manifest_sha256=inventory[str(prepared_manifest)],
        input_hashes=inventory,calibration_only=calibration,raw_or_native_modified=False)
    receipt = output/'normalization.json'
    if output.exists():
        require(receipt.is_file(), 'Incomplete normalization must not be overwritten')
        saved = read(receipt)
        require(saved.get('request') == request, 'Prepared/source/helper normalization contract changed')
        output_hashes = saved.get('output_hashes')
        actual_outputs = {str(path.resolve()) for path in output.rglob('*') if path.is_file() and path != receipt}
        required_outputs = {str((output/name).resolve()) for name in ('manifest.json','metadata.json','scenario.json')}
        require(isinstance(output_hashes,dict) and set(output_hashes) == actual_outputs
            and required_outputs <= actual_outputs, 'Cached normalization complete output inventory differs')
        pins.verify_hashes(output_hashes,'Cached preparation normalization')
        pins.verify_hashes(inventory,'Cached preparation source inputs')
        return output/'manifest.json'
    require(output != Path(source_entry['metadata']).parent and Path(source_entry['metadata']).parent not in output.parents,
        'Normalization output must not overlap source metadata')
    output.mkdir(parents=True)
    candidate = copy.deepcopy(entry);proposed = copy.deepcopy(meta);candidate_deck = copy.deepcopy(deck)
    scenario_path,metadata_path = output/'scenario.json',output/'metadata.json'
    shutil.copyfile(entry['scenario'],scenario_path)
    candidate.update(scenario=str(scenario_path),metadata=str(metadata_path))
    if calibration:
        require(all(a.get('motion') in ('static','standing') for a in proposed.get('runtime_actors',[])),
            'Calibration cannot contain moving actors')
        proposed[KEY] = dict(request,training_quota_eligible=False,training_overlay_detached=True,
            inherited_training_overlay=copy.deepcopy(tag),inherited_shadow_adoption=proposed.get(shadow.KEY),
            independent_terrain_preparation_claimed=False)
        proposed.pop('training_split_overlay',None);proposed.pop(shadow.KEY,None)
        candidate_deck.pop('split_overlay',None)
        for key in ('derived_training_split','derived_layout_group_id'):candidate.pop(key,None)
        candidate_deck.update(calibration_only=True,training_quota_eligible=False,bulk_ready=False,
            status='CALIBRATION_ONLY_LAYOUT_DETACHED; never training quota')
    else:
        proposed[unit.KEY] = copy.deepcopy(expected)
        proposed[TRAINING_KEY] = dict(request,independent_terrain_preparation_claimed=False,
            layout_split_finalization_pending=True)
        same = (entry['scenario_sha256'] == source_entry['scenario_sha256']
            and layout.geometry(proposed) == layout.geometry(source_meta))
        if same:
            # QA-only metadata is a new pinned view of the identical payload.
            require(proposed['spec'] == source_meta['spec'], 'QA-only preparation changed frozen spec')
            candidate = dict(copy.deepcopy(source_entry),scenario=str(scenario_path),metadata=str(metadata_path))
            body = {k:v for k,v in proposed.items() if k != 'training_split_overlay'}
            pretag_path = output/'source_metadata.json';atomic_json(pretag_path,body)
            proposed['training_split_overlay'] = dict(copy.deepcopy(source_meta['training_split_overlay']),
                source_metadata_path=str(pretag_path),source_metadata_sha256=sha256(pretag_path))
        else:
            require(callable(bind_training), 'Canonical recovery callback required for actual training geometry change')
            if candidate['id'] == source_entry['id']:
                candidate['id'] += '_prepared'
            candidate_deck['scenes'] = [candidate]
            bind_training(source_entry,source_meta,candidate,proposed,candidate_deck,output,context=context)
        candidate_deck.update(bulk_ready=False,layout_split_finalization_pending=True)
    atomic_json(metadata_path,proposed)
    candidate_deck.update(count=1,scenes=[candidate])
    atomic_json(output/'manifest.json',candidate_deck)
    unit.validate_entry(candidate,candidate_deck);scale.validate_entry(candidate,candidate_deck)
    if not calibration:layout.validate_manifest_overlay(candidate_deck,allow_subset=True)
    shadow.validate_manifest(candidate_deck)
    pins.verify_hashes(inventory,'Preserved preparation/source inputs')
    outputs = {str(path.resolve()):sha256(path) for path in output.rglob('*') if path.is_file()}
    atomic_json(receipt,dict(request=request,output_hashes=outputs,training_started=False,bulk_started=False))
    return output/'manifest.json'
