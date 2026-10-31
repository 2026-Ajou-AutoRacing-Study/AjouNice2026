#!/usr/bin/env python3
"""Preregister independent scale scenes and balanced, matched unit controls.

No capture outcomes are read. Controls keep actor identity, XY, yaw, motion and
all other actors; only enlarged parents return to unit scale with a pinned unit
ground initializer. They are counterfactual diagnostics, not new production
distribution examples. No simulator, ROS, GPU or native annotation is touched.
"""
import argparse
from collections import Counter
import copy
import hashlib
import json
import math
from pathlib import Path

import base5_asset_scale as scale
import base5_shadow_ground_reuse as shadow
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json, sha256
from base5_reference_frames import references
from base5_scale_capture_contract import validate_entry
from prepare_base5_scale_qualification import merge_references,fresh_output


def read(path):
    return json.loads(Path(path).read_text())


def diagnostic_copy(value):
    """Detach source-ID contracts, preserving native QA, geometry and split.

    Removed values are returned for immutable source provenance only. They
    cannot remain active on new diagnostic IDs or counterfactual geometry.
    """
    copied=copy.deepcopy(value)
    removed={key:copied.pop(key) for key in list(copied) if key in (
        'training_split_overlay','split_overlay',shadow.KEY) or key.startswith('derived_')}
    return copied,removed


def diagnostic_source(deck):
    """Validate original adoption before detaching its old-ID registry.

    A matched control is not a new registration of independently prepared
    terrain. Its actual prepared motion/path stays unchanged, its old adoption
    tag becomes immutable provenance, and native ground QA remains enabled.
    """
    from base5_layout_split import validate_manifest_overlay
    pins=shadow.validate_manifest(deck)
    validate_manifest_overlay(deck)
    result,removed=diagnostic_copy(deck)
    registration=result['config'].pop(shadow.KEY,None)
    if registration is not None:
        removed['config/'+shadow.KEY]=registration
    return result,removed,pins


def pin_training_provenance(value,pins):
    if isinstance(value,dict):
        for path_key,hash_key in (('path','sha256'),('source_metadata_path','source_metadata_sha256')):
            if path_key in value or hash_key in value:
                if path_key not in value or hash_key not in value or sha256(value[path_key])!=value[hash_key]:
                    raise ValueError('Source training-overlay provenance changed')
                pins[str(Path(value[path_key]).resolve())]=value[hash_key]
        for child in value.values():
            pin_training_provenance(child,pins)
    elif isinstance(value,list):
        for child in value:
            pin_training_provenance(child,pins)


def variants(meta):
    return [(actor['name'], actor['asset_scale']['profile_id']) for actor in meta['runtime_actors']
            if actor.get('asset_scale') and actor['asset_scale']['profile_id'] != scale.BASE]


def features(meta):
    spec = meta['spec']
    values = {'family/'+spec['family'], 'split/'+meta['split'],
              'load/'+('single' if len(meta['runtime_actors']) == 1 else 'multi')}
    extent = spec.get('static_mission_extent', {})
    if extent:
        values.add('extent/'+str(extent.get('stratum')))
        values.add('phase/'+str(spec.get('avoidance_phase')))
    for ordinal, actor in enumerate(meta['runtime_actors']):
        assignment = actor.get('asset_scale')
        if not assignment or assignment['profile_id'] == scale.BASE:
            continue
        values.add('role/'+('primary' if ordinal == 0 else 'auxiliary'))
        values.add('distance/'+str(int(actor['planned_center_distance_m']//10)))
        values.add('yaw/'+str(int((actor['placement_yaw_deg'] % 360)//90)))
    return values


def order_key(ident):
    return hashlib.sha256(('scale-stability-261017:'+ident).encode()).hexdigest()


def select(metadata, profiles, target, minimum):
    if (type(target) is not int or not 1 <= target <= 300 or type(minimum) is not int
            or minimum < 1 or minimum > target):
        raise ValueError('Finite positive scene target and explicit profile minimum required')
    covers = {ident:set(variants(meta)) for ident, meta in metadata.items()}
    if any(not value <= profiles for value in covers.values()):
        raise ValueError('Source includes a nonqualified scale profile')
    counts, selected = Counter(), []
    category_counts = Counter()
    all_categories = set().union(*(features(meta) for meta in metadata.values())) if metadata else set()
    available = Counter(profile for value in covers.values() for profile in value)
    if len(metadata) < target or any(available[p] < minimum for p in profiles):
        raise ValueError('Insufficient independent source scenes for the declared target/profile minimum')
    while len(selected) < target:
        remaining = [ident for ident in metadata if ident not in selected]
        needed = {p for p in profiles if counts[p] < minimum}
        missing_categories = all_categories-set(category_counts)
        choices = ([ident for ident in remaining if covers[ident] & needed] if needed else
                   [ident for ident in remaining if features(metadata[ident]) & missing_categories]
                   if missing_categories else remaining)
        if not choices:
            raise ValueError('Independent source scene budget cannot satisfy preregistered profile coverage')
        ident = min(choices, key=lambda ident:(
            -sum(minimum-counts[p] for p in covers[ident] & needed),
            -len(features(metadata[ident]) & missing_categories),
            sum(category_counts[token] for token in features(metadata[ident])), order_key(ident)))
        selected.append(ident)
        counts.update(covers[ident])
        category_counts.update(features(metadata[ident]))
    if any(counts[p] < minimum for p in profiles):
        raise ValueError('Scene cap insufficient for explicit profile minimum; do not weaken it silently')
    if all_categories-set(category_counts):
        raise ValueError('Scene cap insufficient for available context coverage; do not drop categories silently')
    return selected, dict(profile_scene_counts={name+'/'+profile:counts[(name,profile)]
        for name,profile in sorted(profiles)}, category_scene_counts=dict(sorted(category_counts.items())),
        minimum_independent_scenes_per_profile=minimum,
        missing_available_categories=sorted(set().union(*(features(m) for m in metadata.values()))-set(category_counts)))


def envelope(actor):
    from shapely.geometry import Polygon
    x, y = actor['geometric_center_estimate'][:2]
    theta = math.radians(actor['placement_yaw_deg'])
    length, width = actor['estimated_size'][:2]
    return Polygon([(x+math.cos(theta)*a-math.sin(theta)*b,
                     y+math.sin(theta)*a+math.cos(theta)*b)
                    for a,b in ((-length/2,-width/2),(length/2,-width/2),
                                (length/2,width/2),(-length/2,width/2))])


def unit_control(scene, metadata, units):
    scene, meta = copy.deepcopy(scene), copy.deepcopy(metadata)
    before = copy.deepcopy(meta['runtime_actors'])
    plan = meta['spec'].get(scale.PLAN_KEY)
    if not plan:
        raise ValueError('Matched unit control requires an explicit source scale plan')
    meta['spec'] = copy.deepcopy(plan['source_spec'])
    for actor in meta['runtime_actors']:
        assignment = actor.pop('asset_scale', None)
        old_geometry = actor.pop('asset_scale_geometry', None)
        if not assignment or assignment['profile_id'] == scale.BASE:
            continue
        if actor['name'] not in units:
            raise ValueError('Matched unit control lacks pinned native reference: '+actor['name'])
        reference = units[actor['name']]
        size = list(reference['bbox_size_m'])
        offset = list(reference['pivot_to_bbox_center_local_m'])
        if any(unit > enlarged + .03 for unit,enlarged in zip(size,actor['estimated_size'])):
            raise ValueError('Unit control would enlarge an envelope; dedicated placement review required')
        unit_height = size[2]/2-offset[2]
        old_pivot = (old_geometry or {}).get('placement_pivot_m')
        if (not isinstance(old_pivot,(list,tuple)) or len(old_pivot)!=3
                or any(type(value) not in (int,float) or not math.isfinite(value) for value in old_pivot)):
            raise ValueError('Matched control requires its source qualified placement pivot')
        # Preserve the *planned* envelope bottom at precisely the source XY.
        # Do not promote an unavailable MGeo height to contact ground truth.
        old_bottom = old_pivot[2]-actor['estimated_size'][2]/2
        new_bottom = offset[2]-size[2]/2
        z_delta = old_bottom-new_bottom
        z = actor['position']['z']+z_delta
        actor['position']['z'] = z
        actor['estimated_size'] = size
        actor['geometry_reference_known'] = True
        theta = math.radians(actor['placement_yaw_deg'])
        center = [actor['position']['x']+math.cos(theta)*offset[0]-math.sin(theta)*offset[1],
                  actor['position']['y']+math.sin(theta)*offset[0]+math.cos(theta)*offset[1]]
        actor['geometric_center_estimate'] = center
        actor['planned_center_distance_m'] = math.hypot(center[0]-meta['ego']['x'],center[1]-meta['ego']['y'])
        frame = references(center,meta['ego'],meta['base']['yaw'])
        actor['reference_frames'] = frame
        actor['longitudinal_m'],actor['lateral_m'] = frame['ego_longitudinal_m'],frame['ego_lateral_m']
        actor['static_height_initializer'] = dict(usable=True,
            policy='pinned-native-unit-counterfactual-initializer-261017',
            pivot_height_candidate_m=unit_height, contact_verified=False,
            source_envelope_bottom_preserved=True,source_to_control_pivot_z_delta_m=z_delta,
            limitation='Planned source envelope bottom preserved using unit native reference; this does not certify actual mesh or map ground support')
        matches = [raw for raw in scene[actor['source_list']] if raw['UNIQUEID']==actor['unique_id']]
        if len(matches) != 1:
            raise ValueError('Counterfactual scenario actor identity differs')
        raw = matches[0]
        raw['scale'] = dict(x=1.,y=1.,z=1.,_x='1.000',_y='1.000',_z='1.000')
        raw['pos']['z'] = z
        if '_z' in raw['pos']:
            raw['pos']['_z'] = f'{z:.3f}'
    ego = dict(geometric_center_estimate=[meta['ego']['x']+1.35*math.cos(math.radians(meta['ego']['yaw'])),
                meta['ego']['y']+1.35*math.sin(math.radians(meta['ego']['yaw']))],
               placement_yaw_deg=meta['ego']['yaw'],estimated_size=[4.8,2.,1.])
    ego_shape = envelope(ego)
    for index, actor in enumerate(meta['runtime_actors']):
        gap = envelope(actor).distance(ego_shape)
        old_gap = envelope(before[index]).distance(ego_shape)
        if gap < min(.3,old_gap)-.03:
            raise ValueError('Unit control worsens the frozen Ego clearance')
        actor['planned_surface_clearance_m'] = gap
        for other in range(index):
            if envelope(actor).distance(envelope(meta['runtime_actors'][other])) + .03 < \
                    envelope(before[index]).distance(envelope(before[other])):
                raise ValueError('Unit control worsens a frozen actor envelope clearance')
    return scene, meta


def prepare(manifest, output, unit_references, *, pairs=60, minimum_per_profile=4):
    source = Path(manifest).resolve(strict=True)
    output = fresh_output(output,[source.parent]+[Path(path).resolve().parent for path in unit_references])
    deck,deck_training_tags,ground_pins = diagnostic_source(read(source))
    if deck.get('count') != 5000 or len(deck.get('scenes',[])) != 5000:
        raise ValueError('An exact frozen full 5K source is required')
    policy = scale.freeze_policy(deck['config'].get(scale.CONFIG_KEY))
    deck['config'][scale.CONFIG_KEY] = policy
    profiles = {(name,ident) for name,record in policy['assets'].items()
                for ident,profile in record['profiles'].items() if scale._qualified(profile)}
    if not profiles:
        raise ValueError('No native-qualified production scale profiles')
    ref,pins = merge_references(unit_references,{name for name,_ in profiles})
    pins.update(ground_pins)
    pins[str(source)] = sha256(source)
    pin_training_provenance(deck_training_tags,pins)
    metadata,entries = {},{}
    for entry in deck['scenes']:
        meta = read(entry['metadata'])
        if not variants(meta):
            continue
        if entry['id'] in entries or sha256(entry['scenario']) != entry['scenario_sha256']:
            raise ValueError('Duplicate or changed source scenario')
        validate_entry(entry,deck)
        unit.validate_entry(entry,deck)
        metadata[entry['id']],entries[entry['id']] = meta,entry
    chosen,coverage = select(metadata,profiles,pairs,minimum_per_profile)
    fresh_output(output,[Path(entries[ident][field]).parent for ident in chosen for field in ('scenario','metadata')])
    result = []
    for ordinal,ident in enumerate(chosen):
        entry,meta = entries[ident],metadata[ident]
        pins[entry['scenario']],pins[entry['metadata']] = sha256(entry['scenario']),sha256(entry['metadata'])
        diagnostic_entry,entry_training_tags = diagnostic_copy(entry)
        diagnostic_meta,metadata_training_tags = diagnostic_copy(meta)
        pin_training_provenance(entry_training_tags,pins)
        pin_training_provenance(metadata_training_tags,pins)
        scaled = (read(entry['scenario']),diagnostic_meta)
        original = unit_control(*scaled,ref['assets'])
        order = ('original','scaled') if ordinal%2 == 0 else ('scaled','original')
        for position,condition in enumerate(order):
            scene,current = copy.deepcopy(original if condition=='original' else scaled)
            new_id = f'stability_{ordinal:03d}_{condition}_{ident}'
            current['scale_stability'] = dict(pair_id=f'pair_{ordinal:03d}',condition=condition,
                pair_order='/'.join(order),within_pair_position=position,source_id=ident,
                source_scenario_sha256=entry['scenario_sha256'],
                profiles=[list(value) for value in variants(meta)],features=sorted(features(meta)),
                process_state='operator_record_required',source_actor_uids=[a['unique_id'] for a in meta['runtime_actors']],
                source_actor_load=len(meta['runtime_actors']),unit_reference_sources=ref['sources'],
                independent_geometry_unit=ident,source_xy_yaw_identity_preserved=True,
                counterfactual_distribution_claim=False,native_qa_unchanged=True,
                provenance=dict(source_manifest=str(source),source_manifest_sha256=pins[str(source)],
                    removed_training_tags=dict(manifest=deck_training_tags,entry=entry_training_tags,
                                               metadata=metadata_training_tags),
                    source_id_contracts_detached=True,prepared_motion_ground_geometry_preserved=True,
                    diagnostic_only=True,training_overlay_applied=False))
            sp,mp = (output/'scenarios'/(new_id+suffix) for suffix in ('.json','_metadata.json'))
            atomic_json(sp,scene)
            atomic_json(mp,current)
            new_entry = dict(diagnostic_entry,id=new_id,scenario=str(sp),metadata=str(mp),scenario_sha256=sha256(sp))
            result.append(new_entry)
    subset = copy.deepcopy(deck)
    subset.update(dataset_id='base5_scale_stability_261017',scenes=result,count=len(result),
        status='REGISTERED_MATCHED_SCALE_STABILITY_ONLY; no new production or AP approval',
        scale_stability_registration=dict(source_manifest=str(source),source_files=pins,
            preparation_script_sha256=sha256(Path(__file__)),independent_pairs=pairs,
            first_attempt_captures=len(result),balanced_order_counts=dict(original_scaled=(pairs+1)//2,scaled_original=pairs//2),
            coverage=coverage,selection_uses_native_outcomes=False,raw_files_modified=False,
            process_cold_warm_state_not_inferred=True,source_unchanged=True))
    for entry in result:
        validate_entry(entry,subset)
        unit.validate_entry(entry,subset)
    # Old registered IDs must not be active under diagnostic counterfactual IDs.
    shadow.validate_manifest(subset)
    if any(sha256(path)!=value for path,value in pins.items()):
        raise ValueError('Source/reference changed during matched-deck preparation')
    scale.verify_frozen_policy(policy)
    subset['config'] = scale.plain_config(subset['config'])
    atomic_json(output/'manifest.json',subset)
    return output/'manifest.json'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--unit-reference',type=Path,action='append',required=True)
    parser.add_argument('--pairs',type=int,default=60)
    parser.add_argument('--minimum-per-profile',type=int,default=4)
    args = parser.parse_args()
    print(prepare(args.manifest,args.output,args.unit_reference,pairs=args.pairs,
                  minimum_per_profile=args.minimum_per_profile))
