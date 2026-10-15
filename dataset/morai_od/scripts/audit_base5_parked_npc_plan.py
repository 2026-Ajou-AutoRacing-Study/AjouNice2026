#!/usr/bin/env python3
"""Audit explicit NPC footprint/yaw deltas without altering source captures."""
import argparse
from collections import Counter
import json
from pathlib import Path

import base5_parked_npc_placement as placement
from base5_capture_safety import atomic_json, sha256


def audit(source, candidate):
    old = json.loads(source.read_text()); new = json.loads(candidate.read_text())
    originals = {entry['id']: entry for entry in old['scenes']}
    errors, changed, unchanged, guarded, overrides = [], 0, 0, 0, 0
    changed_families = Counter(); changed_assets = Counter()
    if new['config'].get('parked_npc_placement') != placement.DEFAULT:
        errors.append(['deck', 'unfrozen NPC screening policy'])
    if len(new['scenes']) != len(old['scenes']) or set(e['id'] for e in new['scenes']) != set(originals):
        errors.append(['deck', 'logical slot coverage changed'])
    for entry in new['scenes']:
        original = originals[entry['id']]
        before = json.loads(Path(original['metadata']).read_text())
        after = json.loads(Path(entry['metadata']).read_text())
        provenance = after['parked_npc_support_source']
        if (provenance['source_id'] != entry['id'] or provenance['source_scenario_sha256'] != original['scenario_sha256']
                or provenance['source_metadata_sha256'] != sha256(original['metadata'])):
            errors.append([entry['id'], 'source provenance changed'])
        if sha256(entry['scenario']) != entry['scenario_sha256']:
            errors.append([entry['id'], 'candidate scenario checksum mismatch'])
        for key in ('split', 'scene_group_id', 'environment_split_contract', 'logical_quota_slot_id'):
            if before.get(key) != after.get(key):
                errors.append([entry['id'], key + ' changed'])
        old_actors, actors = before['runtime_actors'], after['runtime_actors']
        signature = lambda aa: [(a['data_id'], a['source_list'], a['motion']) for a in aa]
        if signature(old_actors) != signature(actors):
            errors.append([entry['id'], 'asset/container/actual motion changed'])
        allowed = ('frozen_population_data_ids', 'parked_npc_yaw_overrides')
        old_spec, spec = before['spec'], after['spec']
        for key in set(old_spec) | set(spec):
            if key not in allowed and old_spec.get(key) != spec.get(key):
                errors.append([entry['id'], 'spec changed outside NPC footprint delta: ' + key])
        yaw_overrides = spec.get('parked_npc_yaw_overrides', {})
        if yaw_overrides != provenance['explicit_yaw_overrides']:
            errors.append([entry['id'], 'yaw delta provenance differs from spec'])
        risks = [i for i, actor in enumerate(old_actors)
                 if (placement.actor_screen(actor, placement.DEFAULT, old_spec) or {}).get('road_supported') is False]
        if risks != provenance['original_risk_ordinals']:
            errors.append([entry['id'], 'source risk selection changed'])
        if bool(risks) != provenance['geometry_changed']:
            errors.append([entry['id'], 'geometry delta does not match screening'])
        if risks:
            changed += 1; changed_families[old_spec['family']] += 1
            changed_assets.update(old_actors[i]['name'] for i in risks)
            if spec.get('frozen_population_data_ids') != [a['data_id'] for a in old_actors]:
                errors.append([entry['id'], 'repaired population not frozen'])
        else:
            unchanged += 1
            if entry['scenario_sha256'] != original['scenario_sha256']:
                errors.append([entry['id'], 'unaffected payload changed'])
        for index, actor in enumerate(actors):
            proof = placement.actor_screen(actor, placement.DEFAULT, spec)
            if proof is not None:
                guarded += 1
                if not proof['road_supported'] or proof != actor.get('parked_npc_placement'):
                    errors.append([entry['id'], 'NPC footprint proof missing/invalid'])
            if str(index) in yaw_overrides:
                overrides += 1
                try:
                    placement.yaw_override(spec, actor, index, actor['motion'], actor['estimated_size'])
                except ValueError as error:
                    errors.append([entry['id'], str(error)])
                original_actor = old_actors[index]
                if (index not in risks or original_actor.get('static_yaw_policy', {}).get('mode') != 'uniform'
                        or actor.get('static_yaw_policy', {}).get('mode') != 'road_aligned'
                        or yaw_overrides[str(index)]['original_yaw'] != original_actor.get('static_yaw_policy')):
                    errors.append([entry['id'], 'yaw fallback not bound to original infeasible long NPC'])
    frozen = Path(new['config']['frozen_specs_path'])
    if sha256(frozen) != new['config']['frozen_specs_sha256']:
        errors.append(['deck', 'frozen specs hash mismatch'])
    if json.loads(frozen.read_text()) != [json.loads(Path(e['metadata']).read_text())['spec'] for e in new['scenes']]:
        errors.append(['deck', 'frozen specs do not match actual metadata'])
    return dict(source_sha256=sha256(source), candidate_sha256=sha256(candidate), frames=len(new['scenes']),
                changed_scenes=changed, unchanged_payloads=unchanged, guarded_long_npcs=guarded,
                explicit_yaw_fallback_actors=overrides, changed_families=dict(changed_families),
                changed_assets=dict(changed_assets), errors=errors, bulk_started=False,
                live_qualified=False, limitation='Offline source and road footprint proof, not native contact or AP')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source', 'candidate', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args(); result = audit(args.source, args.candidate)
    atomic_json(args.output, result); print(json.dumps(result, ensure_ascii=False, indent=2))
    if result['errors']:
        raise SystemExit(1)
