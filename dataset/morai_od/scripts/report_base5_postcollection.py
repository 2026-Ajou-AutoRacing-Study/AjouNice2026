#!/usr/bin/env python3
"""Read-only current-format timing and valid GT/parent population audit.

Input roots may overlap or import each other. Never sum their cumulative parent
elapsed values; report the chosen canonical elapsed and unique native timers.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

from base5_capture_safety import atomic_json, sha256
from build_base5_bevfusion_infos import in_roi, DEFAULT_ROI


def timing_report(roots):
    seen = set(); stages = Counter(); groups = defaultdict(Counter); sessions = []
    errors = []
    for root in roots:
        for path in sorted(Path(root).rglob('timing.json')):
            st = path.stat(); key = (st.st_dev, st.st_ino)
            if key in seen: continue
            seen.add(key)
            doc = json.loads(path.read_text())
            if 'stages_sec' not in doc: continue
            values = doc['stages_sec']
            if any(not math.isfinite(v) or v < 0 for v in values.values()):
                errors.append(str(path) + ': invalid duration'); continue
            if abs(sum(values.values()) - doc['total_sec']) > .001:
                errors.append(str(path) + ': stages do not partition total')
            ledger = path.parent/'ledger.jsonl'
            rows = [json.loads(s) for s in ledger.read_text().splitlines()] if ledger.exists() else []
            outcomes = Counter(r['status'] for r in rows)
            # Native timer may be overwritten after an interrupted-session resume.
            # Do NOT call all rows in its cumulative ledger fresh attempts.
            attempts = sum(i['stage'] == 'scenario_install' for i in doc['intervals'])
            kind = 'unknown'; metadata = list((path.parent/'captures').glob('*/metadata.json'))
            motions = set()
            for p in metadata:
                m = json.loads(p.read_text())
                if m.get('calibration_only'): motions.add('calibration')
                elif any(a['motion'] not in ('static','standing') for a in m['runtime_actors']):
                    motions.add('shadow_animated' if m['spec'].get('shadow_moving') else 'intersection_animated')
                else: motions.add('static')
            if len(motions) == 1: kind = next(iter(motions))
            elif motions: kind = 'mixed'
            stages.update(values); groups[kind].update(values)
            sessions.append(dict(path=str(path.resolve()),sha256=sha256(path),
                total_sec=doc['total_sec'],scenario_install_intervals=attempts,
                cumulative_ledger_outcomes=dict(outcomes),group=kind))
    total = sum(stages.values()); attempts = sum(s['scenario_install_intervals'] for s in sessions)
    return dict(unique_sessions=len(sessions),timed_scenario_loads=attempts,
        measured_native_sec=total,stage_sec=dict(stages),
        stage_sec_per_timed_load={k:v/attempts for k,v in stages.items()} if attempts else {},
        stage_fraction={k:v/total for k,v in stages.items()} if total else {},
        groups={k:dict(v) for k,v in groups.items()},sessions=sessions,errors=errors,
        limitation='Native timers exclude interpreter startup and parent QA/generation. A resumed timer can replace its earlier segment; this is NOT the complete cumulative wall cost. Parent elapsed imports are not additive.')


def population_report(ledger, roi=DEFAULT_ROI):
    frames = Counter(); components = Counter(); cells = defaultdict(Counter)
    assets = defaultdict(Counter); sizes = defaultdict(Counter); logical = set(); errors = []
    for line in Path(ledger).read_text().splitlines():
        row = json.loads(line)
        if row['status'] != 'accepted_candidate': continue
        ident = row.get('source_cell_id',row['id'])
        if ident in logical: raise ValueError('Duplicate selected logical scene '+ident)
        logical.add(ident)
        root = Path(row['capture_root']); m = json.loads((root/'metadata.json').read_text())
        a = json.loads(Path(row.get('audit_path',root/'audit.json')).read_text())
        if m.get('calibration_only') or a.get('review_reasons'):
            errors.append('Invalid selected training frame '+ident); continue
        spec = m['spec']; split = m['split']; mode = m.get('population_budget',{}).get('mode','legacy')
        category = 'shadow' if spec.get('shadow_phase') else spec['family']
        cell = cells[f'{category}:{"mission" if spec["mission"] else "generic"}:{split}']
        frames['selected'] += 1; frames[split] += 1; cell['frames'] += 1
        actors = {x['unique_id']:x for x in m['runtime_actors']}
        primary = next(iter(actors),None); kept = defaultdict(list)
        for c in a['components']:
            components['raw'] += 1
            if not c['positive_lidar1_camera0'] or not in_roi(c['center_lidar_m'],roi):
                components['excluded'] += 1; continue
            kept[c['parent_uid']].append(c); cls = c['training_class']
            components['valid'] += 1; components[cls] += 1; cell['valid_gt'] += 1
            ap = assets[c['asset']]; ap['valid_gt'] += 1; ap[split+'_valid_gt'] += 1
            strong = c['lidar_points'] >= 20 or (not c.get('camera_component_ambiguous') and any(
                cam['crop_320x896_instance_pixels'] >= 64 and min(cam['model_extent_px']) >= 8
                for cam in c['cameras']))
            if strong:
                components['rich_cue'] += 1; ap['rich_cue_gt'] += 1; ap[split+'_rich_cue_gt'] += 1
            if 1 <= c['lidar_points'] <= 4 and not c['camera_visible_candidate']:
                components['lidar_1_4_camera_unconfirmed'] += 1; ap['sparse_camera_unconfirmed_gt'] += 1
            if not c['lidar_points'] and c['camera_visible_candidate']: components['camera_only'] += 1
        cell['placed_parents'] += len(actors); cell['visible_parents'] += len(kept)
        for uid, actor in actors.items():
            kind = 'primary' if uid == primary else 'secondary'
            ap = assets[actor['name']]; ap['placed_parents'] += 1; ap[kind+'_parents'] += 1
            ap['visible_parents'] += int(uid in kept)
            cost = actor.get('population_cost',0)
            # Comparable population budget conditioning: regular/dense only.
            if mode not in ('generic_regular','generic_dense'): continue
            b = sizes[str(cost)]; b[kind+'_placed'] += 1; b[kind+'_visible'] += int(uid in kept)
            b[kind+'_valid_gt'] += len(kept[uid])
    return dict(ledger_sha256=sha256(ledger),frames=dict(frames),components=dict(components),
        regular_dense_cost_buckets={k:dict(v) for k,v in sorted(sizes.items())},
        family_split={k:dict(v) for k,v in cells.items()},assets={k:dict(v) for k,v in sorted(assets.items())},errors=errors,
        policy='Provisional current 320x896 crop + lidar1 OR camera; raw/native GT unchanged.',
        limitation='Parent size cost != apparent pixel size. Rich cue is analysis-only, not model accuracy. Same-parent same-class camera ambiguity can undercount visible cues; reevaluate after crop choice.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--collection',type=Path,required=True)
    p.add_argument('--timing-root',type=Path,action='append',default=[])
    p.add_argument('--output',type=Path,required=True); a = p.parse_args()
    canonical = json.loads((a.collection/'status.json').read_text())
    result = dict(schema=1,canonical_collection=str(a.collection.resolve()),
        canonical_status=canonical['stage'],canonical_cumulative_active_sec=canonical.get('elapsed_sec'),
        population=population_report(a.collection/'selected_ledger.jsonl'),
        timing=timing_report(a.timing_root))
    atomic_json(a.output,result)
    print(json.dumps(dict(active_hours=result['canonical_cumulative_active_sec']/3600,
        components=result['population']['components'],cost_buckets=result['population']['regular_dense_cost_buckets'],
        timing_loads=result['timing']['timed_scenario_loads'],stage_seconds_per_load=result['timing']['stage_sec_per_timed_load']),indent=2))
    if result['population']['errors'] or result['timing']['errors']: raise SystemExit(1)


if __name__ == '__main__': main()
