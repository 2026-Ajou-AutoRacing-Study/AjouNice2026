#!/usr/bin/env python3
"""Paired input-Z diagnostic only; raw annotation and production remain unchanged."""
import argparse
import copy
from pathlib import Path
import generate_base5_independent as gen

ASSETS = ((1, 'Dog2'), (3, 'Ped_Pet1'), (7, 'Scooter1'))


def shift_input(scene, metadata, delta):
    if not -.75 <= delta <= .10:
        raise ValueError('Diagnostic Z delta outside [-0.75, 0.10] m')
    if len(metadata['runtime_actors']) != 1:
        raise ValueError('Single-actor diagnostic required')
    record = metadata['runtime_actors'][0]
    uid = record['unique_id']
    if record['source_list'] not in ('objectList', 'pedestrianList'):
        raise ValueError('Unsupported height-control container')
    actor = next(a for a in scene[record['source_list']] if a['UNIQUEID'] == uid)
    actor['pos']['z'] += delta
    if '_z' in actor['pos']:
        actor['pos']['_z'] = f"{actor['pos']['z']:.3f}"
    record['position']['z'] += delta
    for key in ('initPos', 'standardPos'):
        if key in actor:
            actor[key]['y'] += delta
            if '_y' in actor[key]:
                actor[key]['_y'] = f"{actor[key]['y']:.3f}"
    path = record.get('motion_path')
    if path:
        for point in path['points']:
            point['z'] += delta
        if 'target_position' in path:
            path['target_position']['z'] += delta
        raw = next(p for p in scene['waypointDataList'] if p['pathName'].endswith(' - '+str(uid)))
        for point in raw['waypointData']:
            point['pos']['y'] += delta
            if '_y' in point['pos']:
                point['pos']['_y'] = f"{point['pos']['y']:.3f}"
    probe = metadata.get('motion_probe', {})
    if 'target_position' in probe:
        probe['target_position']['z'] += delta
    probe.pop('height_calibration', None)


def measured_corrections(report):
    if not report['shutdown'] or not report['shutdown']['park_verified']:
        raise ValueError('Unsafe calibration session')
    out={}
    for _,name in ASSETS:
        for mode in ('static','moving'):
            candidates=[r for r in report['rows'] if r.get('asset')==name and r['mode']==mode
                and r.get('ground_gap_m') is not None and r.get('collision_clear')]
            if not candidates:raise ValueError('Missing measured ground: '+name+'/'+mode)
            row=min(candidates,key=lambda r:abs(r['delta_m']))
            baseline_gap=row['ground_gap_m']-row['delta_m']
            # Only measured above-ground states receive a bounded downward
            # control. Never infer an arbitrary upward terrain correction.
            shift=-baseline_gap-.03 if baseline_gap>.15 else 0.
            if not -.75<=shift<=0:raise ValueError('Measured control out of bound')
            out[(name,mode)]=dict(shift_m=shift,source_capture=row['capture_root'],
                baseline_gap_estimate_m=baseline_gap,observed_delta_m=row['delta_m'],
                source_scenario_sha256=row['source_sha256'])
    return out


def prepare(source, output, deltas=(0., .02, .05, .10), repeats=1, assets=ASSETS, ground_report=None):
    if output.exists():
        raise FileExistsError('Diagnostic overwrite refused')
    if repeats not in (1, 2) or not deltas or len(assets)*2*len(deltas)*repeats > 64:
        raise ValueError('Bounded diagnostic only, maximum 64 scenes')
    corrections=measured_corrections(gen.geo.load_json(ground_report)) if ground_report else {}
    output.mkdir(parents=True)
    entries = []
    for repeat in range(repeats):
        for delta in deltas:
            for index, name in assets:
                for mode in ('static', 'moving'):
                    sid = f'b5asset_{index:02d}_{mode}_r0'
                    sp, mp = source/(sid+'.json'), source/(sid+'_metadata.json')
                    scene = gen.geo.load_json(sp)
                    metadata = gen.geo.load_json(mp)
                    if metadata['runtime_actors'][0]['name'] != name:
                        raise ValueError('Asset source mismatch')
                    before = copy.deepcopy(metadata['runtime_actors'][0]['position'])
                    correction=corrections.get((name,mode))
                    if correction:
                        if correction['source_scenario_sha256']!=gen.geo.sha256(sp):
                            raise ValueError('Measured control belongs to a different placement')
                        shift_input(scene, metadata, correction['shift_m'])
                    shift_input(scene, metadata, delta)
                    # Never confuse late messages from a preceding scene that
                    # reused UID 8500 with a physical Z jump of this actor.
                    record = metadata['runtime_actors'][0]
                    old_uid, new_uid = record['unique_id'], 12000+len(entries)
                    next(a for a in scene[record['source_list']] if a['UNIQUEID']==old_uid)['UNIQUEID'] = new_uid
                    for path in scene.get('waypointDataList', []):
                        if path['pathName'].endswith(' - '+str(old_uid)):
                            path['pathName'] = path['pathName'][:-(len(str(old_uid)))]+str(new_uid)
                    record['unique_id'] = new_uid
                    metadata['ground_support_check'] = dict(enabled=True, max_range_m=20., maximum_gap_m=.15)
                    metadata['z_margin_probe'] = dict(asset=name, mode=mode, delta_m=delta,
                        repeat=repeat, source_scenario=str(sp.resolve()), source_sha256=gen.geo.sha256(sp),
                        baseline_input_position=before, diagnostic_only=True)
                    if correction:metadata['z_margin_probe']['measured_ground_control']=correction
                    ident = f'b5z_{index:02d}_{mode}_{round(delta*1000):+04d}_r{repeat}'
                    target, meta = output/(ident+'.json'), output/(ident+'_metadata.json')
                    gen.geo.write_json(target, scene)
                    gen.geo.write_json(meta, metadata)
                    spec = metadata['spec']
                    entries.append(dict(id=ident, scenario=str(target.resolve()), metadata=str(meta.resolve()),
                        scenario_sha256=gen.geo.sha256(target), family='pedestrian_related',
                        role=spec['role'], distance_bounds=spec['distance_bounds'],
                        subfamily='z_margin_diagnostic', motion=metadata['runtime_actors'][0]['motion'], split='train'))
    manifest = dict(schema_version=1, dataset_id='base5_z_margin_diagnostic',
        config=gen.geo.load_json(gen.CONFIG), scenes=entries, count=len(entries), start=0,
        status='DIAGNOSTIC_ONLY_NOT_TRAINING_QUOTA')
    gen.geo.write_json(output/'manifest.json', manifest)
    return output/'manifest.json'


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--deltas', type=float, nargs='+', default=[0., .02, .05, .10])
    p.add_argument('--repeats', type=int, default=1)
    p.add_argument('--ground-reference-report',type=Path)
    a = p.parse_args()
    print(prepare(a.source, a.output, a.deltas, a.repeats, ground_report=a.ground_reference_report))
