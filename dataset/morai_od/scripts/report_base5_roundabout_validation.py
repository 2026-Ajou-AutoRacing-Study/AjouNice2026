#!/usr/bin/env python3
"""Read-only roundabout geometry/visible-sensor evidence; no model accuracy claims."""
import argparse
import copy
import math
from collections import Counter
from pathlib import Path

import generate_base5_independent as gen
from base5_capture_safety import atomic_json, sha256
from build_base5_bevfusion_infos import in_roi
from base5_static_vehicle import park,runtime_review,POLICY as STATIC_POLICY


def load(path):
    return gen.geo.load_json(Path(path))


def cells(deck):
    return [(e, load(e['metadata'])) for e in deck['scenes']
            if e['family'] == 'roundabout' and e['mission']]


def planned(deck, source):
    rows = cells(deck)
    old = {e['id']: e for e in source['scenes']}
    unchanged = [];byte_preserved=[]
    errors = []
    for e in deck['scenes']:
        if sha256(Path(e['scenario'])) != e['scenario_sha256']:
            errors.append('changed payload ' + e['id'])
        if not (e['family'] == 'roundabout' and e['mission']):
            prior = old[e['id']]
            left,right=load(prior['scenario']),load(e['scenario'])
            for payload in (left,right):
                for actor in payload['vehicleList']:park(actor)
            lm,rm=load(prior['metadata']),load(e['metadata'])
            for meta in (lm,rm):
                for actor in meta['runtime_actors']:actor.pop('stationary_vehicle_policy',None)
            unchanged.append(left==right and lm==rm)
            byte_preserved.append(e==prior)
    gaps = [a['planned_surface_clearance_m'] for _, m in rows for a in m['runtime_actors']]
    return dict(status='PASS' if all(unchanged) and not errors else 'FAIL',
                manifest_scenes=len(deck['scenes']), roundabout_mission_scenes=len(rows),
                preserved_other_scene_geometry=sum(unchanged), byte_preserved_other_scenes=sum(byte_preserved),errors=errors,
                phase_counts=dict(Counter(m['spec']['roundabout_phase'] for _, m in rows)),
                observation_states=dict(Counter(m['spec'].get('roundabout_observation_state',
                    m['spec']['roundabout_phase']) for _, m in rows)),
                density_counts=dict(Counter(m['spec']['roundabout_density'] for _, m in rows)),
                actor_count_histogram=dict(Counter(len(m['runtime_actors']) for _, m in rows)),
                primary_model_counts=dict(sorted(Counter(m['runtime_actors'][0]['name'] for _, m in rows).items())),
                planned_actor_count=sum(len(m['runtime_actors']) for _, m in rows),
                minimum_planned_surface_gap_m=min(gaps),
                limitations=['Preserved metadata compares current source bytes; source is never overwritten.',
                             'Planned density does not certify visibility, physical collision or future NPC traffic.'])


def accepted_rows(ledgers, roundabout_only=True):
    result = []
    for ledger in ledgers:
        import json
        for line in Path(ledger).read_text().splitlines():
            entry = json.loads(line)
            if entry['status'] != 'accepted_candidate':
                continue
            root = Path(entry['capture_root'])
            meta = load(root / 'metadata.json')
            if roundabout_only and not meta['spec'].get('roundabout_phase'):
                continue
            result.append((entry, meta, load(root / 'audit.json')))
    return result


def integration(ledgers):
    """Independently check the control-only migration's non-mission pilot slice."""
    rows = [r for r in accepted_rows(ledgers, roundabout_only=False)
            if not r[1]['spec'].get('roundabout_phase')]
    errors = []; evidence = []
    for entry, meta, audit in rows:
        reasons, runtime = runtime_review(meta)
        collision = audit.get('capture_safety', {}).get('collision', {})
        park_verified = entry.get('between_scene_park', {}).get('park_verified')
        if reasons or audit['review_reasons'] or not runtime or not collision.get('no_collision_verified') or not park_verified:
            errors.append('control-only static NPC integration failed '+entry['id'])
        evidence.append(dict(id=entry['id'], family=meta['spec']['family'],
            mission=meta['spec']['mission'], stationary_vehicle_runtime=runtime,
            collision_verified=collision.get('no_collision_verified'), park_verified=park_verified))
    return dict(status=('PASS' if not errors else 'FAIL') if rows else 'NOT_TESTED',
        accepted_scenes=len(rows), errors=errors, evidence=evidence)


def verified_selections(deck, ledgers):
    """Only exact-payload, collision-safe captures from safely stopped children."""
    rows = accepted_rows(ledgers, roundabout_only=False)
    verified = {}
    originals = {e['id']: e for e in deck['scenes']}
    for entry, _, audit in rows:
        key = entry.get('source_cell_id', entry['id'])
        if key not in originals or entry['scenario_sha256'] != originals[key]['scenario_sha256']:
            raise ValueError('Prior selection is not the exact predetermined pilot payload')
        if not entry.get('between_scene_park', {}).get('park_verified') or audit['review_reasons'] or not audit.get('capture_safety', {}).get('collision', {}).get('no_collision_verified'):
            raise ValueError('Unsafe capture cannot be reused')
        # Only rows durably committed by a completed, safe child are reusable.
        capture_root = Path(entry['capture_root'])
        child = capture_root.parent.parent
        if not load(child/'shutdown_safety.json').get('park_verified'):
            raise ValueError('Unsafe child shutdown; no partial reuse')
        if key in verified:
            raise ValueError('Duplicate prior source cell '+key)
        verified[key] = entry
    return verified


def remaining_pilot(deck, ledgers, output):
    """Keep the original predetermined cells; never resume an unsafe session."""
    if output.exists():
        raise FileExistsError('Fresh remaining-pilot manifest required')
    verified = verified_selections(deck, ledgers)
    result = copy.deepcopy({k: v for k, v in deck.items() if k != 'scenes'})
    result.update(scenes=[e for e in deck['scenes'] if e['id'] not in verified],
                  count=len(deck['scenes'])-len(verified), status='REMAINING_PREDETERMINED_PILOT_PENDING')
    atomic_json(output, result)
    return dict(safe_prior_cells=len(verified), remaining_cells=result['count'])


def combined_ledger(deck, ledgers, output):
    """A separate selection artifact, never a rewrite of a halted campaign."""
    import json
    verified = verified_selections(deck, ledgers)
    if set(verified) != {e['id'] for e in deck['scenes']}:
        raise ValueError('Combined pilot is incomplete; do not certify the original deck')
    with output.open('x') as stream:
        for original in deck['scenes']:
            stream.write(json.dumps(verified[original['id']], ensure_ascii=False)+'\n')
    return dict(selected_cells=len(verified), ledger=str(output.resolve()))


def actual(ledgers, roi):
    rows = accepted_rows(ledgers)
    errors = []
    evidence = []
    for entry, meta, audit in rows:
        uidset = {a['unique_id'] for a in meta['runtime_actors']}
        visible = [c for c in audit['components'] if c['parent_uid'] in uidset
                   and c['positive_lidar1_camera0'] and in_roi(c['center_lidar_m'], roi)]
        visible_uids = {c['parent_uid'] for c in visible}
        first_two = [a['unique_id'] for a in meta['runtime_actors'][:2]]
        # Use actual captured geometry in Ego coordinates, not Object_topic pivots.
        ep = gen.rectangle([1.35, 0.], [4.8, 2.], 0.)
        polys = [gen.rectangle(c['center_ego_m'][:2], c['size_m'],
                 math.degrees(c['rotation_rpy'][2])) for c in visible]
        gaps = [ep.distance(p) for p in polys]
        phase = meta['spec']['roundabout_phase']
        front_back = (len(first_two) == 2 and all(u in visible_uids for u in first_two)
                      and any(c['center_ego_m'][0] > 0 and c['parent_uid'] in first_two for c in visible)
                      and any(c['center_ego_m'][0] < 0 and c['parent_uid'] in first_two for c in visible))
        collision = audit.get('capture_safety', {}).get('collision', {})
        park = entry.get('between_scene_park', {})
        if audit['review_reasons'] or not collision.get('no_collision_verified') or not park.get('park_verified'):
            errors.append('unsafe or reviewed accepted capture ' + entry['id'])
        if phase == 'circulation' and not front_back:
            errors.append('circulation visible front/back missing ' + entry['id'])
        if len(first_two) == 2 and phase == 'entry' and not all(u in visible_uids for u in first_two):
            errors.append('entry first two visible NPCs missing ' + entry['id'])
        observed_meta=copy.deepcopy(meta)
        for actor in observed_meta['runtime_actors']:
            if actor['source_list']=='vehicleList' and actor['motion'] in ('static','standing'):
                actor['stationary_vehicle_policy']=STATIC_POLICY
        runtime_reasons,runtime=runtime_review(observed_meta)
        if runtime_reasons:errors.extend(reason+' '+entry['id'] for reason in runtime_reasons)
        # Match clearances by UID, never assume annotation order is actor order.
        first_two_gaps=[min((g['annotation_surface_gap_m'] for g in audit['actual_annotation_clearances']
                       if g['parent_uid']==uid),default=math.nan) for uid in first_two]
        if phase=='circulation':
            lo,hi={'tight':(.5,2.5),'normal':(4.,10.),'loose':(10.,22.)}[meta['spec']['roundabout_density']]
            if len(first_two_gaps)!=2 or any(not math.isfinite(g) or g<lo-.3 or g>hi+.3 for g in first_two_gaps):
                errors.append('actual circulation density gap differs '+entry['id'])
        evidence.append(dict(id=entry['id'], source_cell_id=entry.get('source_cell_id', entry['id']),
            capture_root=entry['capture_root'], primary_asset=meta['runtime_actors'][0]['name'],
            phase=phase, density=meta['spec']['roundabout_density'],
            placed_actors=len(uidset), visible_positive_actors=len(visible_uids),
            visible_front_back=front_back, annotation_gaps_m=gaps,
            first_two_actual_gaps_m=first_two_gaps,
            stationary_vehicle_runtime=runtime,
            annotation_pair_gaps_m=[p.distance(q) for i, p in enumerate(polys) for q in polys[:i]],
            ground_gaps_m=[a.get('minimum_annotation_ground_gap_m')
                          for a in audit['ground_support']['actors'] if a.get('verified')],
            ground_warning_count=len(audit.get('ground_warnings', [])),
            collision_verified=collision.get('no_collision_verified'), park_verified=park.get('park_verified')))
    values = [g for e in evidence for g in e['annotation_gaps_m']]
    return dict(status='PASS' if rows and not errors else 'FAIL', errors=errors,
        accepted_scenes=len(rows), source_scenes=len({e['source_cell_id'] for e in evidence}),
        primary_models=len({e['primary_asset'] for e in evidence}),
        phases=dict(Counter(e['phase'] for e in evidence)),
        densities=dict(Counter(e['density'] for e in evidence)),
        placed_actors=sum(e['placed_actors'] for e in evidence),
        visible_positive_actors=sum(e['visible_positive_actors'] for e in evidence),
        minimum_actual_annotation_surface_gap_m=min(values) if values else None,
        maximum_abs_measured_ground_gap_m=max((abs(g) for e in evidence for g in e['ground_gaps_m']), default=None),
        maximum_observed_static_pivot_xy_error_m=max((a['maximum_pivot_xy_error_m'] for e in evidence
            for a in (e.get('stationary_vehicle_runtime') or {}).get('actors',[])
            if a['maximum_pivot_xy_error_m'] is not None),default=None),
        evidence=evidence,
        limitations=['Original sensor annotation is used; no detector/tracker was run.',
                     'Annotation clearance is a horizontal OBB gap, not the physical collider.',
                     'CollisionData window, Park and advisory local-ground QA are separate evidence.',
                     'Sparse ground remains a warning, not certified mesh contact.'])


def additional_pilot(deck, ledgers, output):
    """Choose missing models before observing their capture outcome, plus edge cells."""
    seen = accepted_rows(ledgers)
    tested = {m['runtime_actors'][0]['data_id'] for _, m, _ in seen}
    used = {e.get('source_cell_id', e['id']) for e, _, _ in seen}
    candidates = cells(deck)
    selected = []
    for phase in deck['config']['mission_roundabout_observation']['phase_counts']:
        for density in deck['config']['mission_roundabout_observation']['density_weights']:
            matches=[(e,m) for e,m in candidates if e['id'] not in used and
                     m['spec']['roundabout_phase']==phase and m['spec']['roundabout_density']==density]
            if matches:
                entry,meta=matches[0];selected.append(entry);used.add(entry['id'])
                tested.add(meta['runtime_actors'][0]['data_id'])
    all_models = sorted({m['runtime_actors'][0]['data_id'] for _, m in candidates})
    for ordinal, model in enumerate(all_models):
        if model in tested:
            continue
        phase = ('approach', 'entry', 'circulation', 'exit')[ordinal % 4]
        options = [(e, m) for e, m in candidates if m['runtime_actors'][0]['data_id'] == model
                   and e['id'] not in used]
        options.sort(key=lambda pair: (pair[1]['spec']['roundabout_phase'] != phase,
                                      pair[1]['spec']['roundabout_density'] != 'tight', pair[0]['id']))
        entry, _ = options[0]
        selected.append(entry); used.add(entry['id'])
    # Route-end snapshots and closest front/back are not selected by good outcomes.
    for key in (lambda m: m['base']['route_arc_m'], lambda m: -m['base']['route_arc_m'],
                lambda m: min(a['planned_surface_clearance_m'] for a in m['runtime_actors'])
                if m['spec']['roundabout_phase'] == 'circulation' else math.inf):
        options = [(e, m) for e, m in candidates if e['id'] not in used]
        entry, _ = min(options, key=lambda pair: key(pair[1]))
        selected.append(entry); used.add(entry['id'])
    # State-only migration also affects non-roundabout static NPCs. Use a small
    # preselected integration slice, not new geometry or outcome-picked retries.
    for family,mission in (('highway',False),('roundabout',False),('obstacle',True),('pedestrian_related',False)):
        for entry in deck['scenes']:
            if entry['id'] in used or entry['family']!=family or entry['mission']!=mission:continue
            metadata=load(entry['metadata'])
            if not any(a.get('stationary_vehicle_policy') for a in metadata['runtime_actors']):continue
            selected.append(entry);used.add(entry['id']);break
    result = copy.deepcopy({k: v for k, v in deck.items() if k != 'scenes'})
    result.update(scenes=selected, count=len(selected), status='MODEL_AND_ROUTE_EDGE_PILOT_PENDING')
    atomic_json(output, result)
    return len(selected)


def plot(deck, ledgers, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from PIL import Image
    ctx = gen.context(gen.geo.DEFAULT_RUNTIME_ASSET_DIR, gen.geo.DEFAULT_MGEO)
    path = gen.roundabout_mission.corridor(gen, ctx)
    fig, axes = plt.subplots(1, 2, figsize=(15, 7))
    axes[0].plot([p['x'] for p in path['route']], [p['y'] for p in path['route']],
                 label='Sample NPC start -> despawn', color='black')
    route = [p for p, s in zip(ctx['route'], ctx['arc']) if 868 <= s <= 936]
    axes[0].plot([p['x'] for p in route], [p['y'] for p in route], '--', label='Ego mission route')
    colors = dict(approach='tab:blue', entry='tab:orange', circulation='tab:green', exit='tab:red')
    for phase, color in colors.items():
        poses = [m['ego'] for _, m in cells(deck) if m['spec']['roundabout_phase'] == phase]
        axes[0].scatter([p['x'] for p in poses], [p['y'] for p in poses], s=14, color=color, label=phase)
    axes[0].set_aspect('equal'); axes[0].legend(); axes[0].set_title('250 planned Ego views / sample NPC corridor')
    axes[0].set_xlabel('map x (m)'); axes[0].set_ylabel('map y (m)')
    rows = accepted_rows(ledgers)
    tight = [(e, m, a) for e, m, a in rows if m['spec']['roundabout_phase'] == 'circulation'
             and m['spec']['roundabout_density'] == 'tight']
    if tight:
        e, m, a = min(tight, key=lambda t: min(c['annotation_surface_gap_m'] for c in t[2]['actual_annotation_clearances']))
        ep = gen.rectangle([1.35, 0.], [4.8, 2.], 0.)
        axes[1].fill(*ep.exterior.xy, alpha=.5, color='black', label='Ego')
        for c in a['components']:
            if not c['positive_lidar1_camera0']:continue
            poly = gen.rectangle(c['center_ego_m'][:2], c['size_m'], math.degrees(c['rotation_rpy'][2]))
            axes[1].fill(*poly.exterior.xy, alpha=.4, label=c['asset'])
        axes[1].set_aspect('equal'); axes[1].legend(fontsize=8); axes[1].set_title('Actual captured GT: '+e['id'])
        axes[1].set_xlabel('Ego forward (m)');axes[1].set_ylabel('Ego left (m)')
        camera_fig, camera_axes = plt.subplots(2, 2, figsize=(16, 9))
        for i, ax in enumerate(camera_axes.flat, 1):
            image_path = next(Path(e['capture_root']).glob(f'CAMERA_{i}/*_Intensity.png'))
            with Image.open(image_path) as img:ax.imshow(img)
            ax.set_title(f'{e["id"]}: CAMERA_{i}');ax.axis('off')
        camera_fig.tight_layout();camera_fig.savefig(output.with_name(output.stem+'_cameras.png'), dpi=120)
        plt.close(camera_fig)
    fig.tight_layout();fig.savefig(output, dpi=140);plt.close(fig)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--source-manifest', type=Path, required=True)
    p.add_argument('--ledger', type=Path, action='append', default=[])
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--additional-pilot', type=Path)
    p.add_argument('--remaining-pilot', type=Path)
    p.add_argument('--combined-ledger', type=Path)
    p.add_argument('--plot', type=Path)
    a = p.parse_args();deck = load(a.manifest)
    result = dict(planned=planned(deck, load(a.source_manifest)))
    if a.ledger:
        result['actual'] = actual(a.ledger, deck['config']['training_roi_candidate'])
        result['control_only_integration'] = integration(a.ledger)
    if a.additional_pilot:result['additional_pilot_count'] = additional_pilot(deck, a.ledger, a.additional_pilot)
    if a.remaining_pilot:result['remaining_pilot'] = remaining_pilot(deck, a.ledger, a.remaining_pilot)
    if a.combined_ledger:result['combined_ledger'] = combined_ledger(deck, a.ledger, a.combined_ledger)
    atomic_json(a.output, result)
    if a.plot:plot(deck, a.ledger, a.plot)
    print({k: {n: v for n, v in r.items() if n not in ('evidence', 'primary_model_counts')}
           if isinstance(r, dict) else r for k, r in result.items()})
    if any(r.get('status') == 'FAIL' for r in result.values() if isinstance(r, dict)):
        raise SystemExit(1)
