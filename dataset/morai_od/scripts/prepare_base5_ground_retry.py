#!/usr/bin/env python3
"""Explicit bounded input-Z retry from a local LiDAR plane; never rewrites GT."""
import argparse
import copy
import json
from pathlib import Path
import generate_base5_independent as gen


def shift_actor(scene,metadata,uid,shift):
    if not -.75<=shift<0:raise ValueError('Ground shift outside conservative downward bound')
    record=next(a for a in metadata['runtime_actors'] if a['unique_id']==uid)
    if record['source_list'] not in ('objectList','pedestrianList'):raise ValueError('Unsupported height-control container')
    actor=next(a for a in scene[record['source_list']] if a['UNIQUEID']==uid)
    actor['pos']['z']+=shift;record['position']['z']+=shift
    if '_z' in actor['pos']:actor['pos']['_z']=f"{actor['pos']['z']:.3f}"
    for key in ('initPos','standardPos'):
        if key in actor:
            actor[key]['y']+=shift
            if '_y' in actor[key]:actor[key]['_y']=f"{actor[key]['y']:.3f}"
    path=record.get('motion_path')
    if path:
        for point in path['points']:point['z']+=shift
        if path.get('ground_path_evidence'):
            path['ground_path_evidence']['asset_pivot_offset_m']+=shift
        if 'target_position' in path:path['target_position']['z']+=shift
        raw_path=next(p for p in scene['waypointDataList'] if p['pathName'].endswith(' - '+str(uid)))
        for point in raw_path['waypointData']:
            point['pos']['y']+=shift
            if '_y' in point['pos']:point['pos']['_y']=f"{point['pos']['y']:.3f}"
    record['input_ground_z_shift_m']=shift


def prepare(manifest_path,captures,output):
    if output.exists():raise FileExistsError('Ground retry overwrite refused')
    manifest=gen.geo.load_json(manifest_path);entries=[];output.mkdir(parents=True)
    ledger=[json.loads(line) for line in (captures/'ledger.jsonl').read_text().splitlines() if line]
    latest={r['id']:r for r in ledger}
    for entry in manifest['scenes']:
        result=latest.get(entry['id'],{});audit=result.get('audit',{})
        corrections=[a for a in (audit.get('ground_support') or {}).get('actors',[])
                     if a.get('reason')=='actor_annotation_above_ground' and a.get('plane')]
        if not corrections:continue
        scene=gen.geo.load_json(Path(entry['scenario']));meta=gen.geo.load_json(Path(entry['metadata']))
        applied=[]
        for row in corrections:
            try:shift_actor(scene,meta,row['uid'],row['suggested_z_shift_m'])
            except ValueError:continue
            applied.append(row)
        if not applied:continue
        sid=entry['id']+'_ground';sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
        meta['ground_retry_provenance']=dict(source_scene_sha256=entry['scenario_sha256'],source_capture=result['capture_root'],
            measured_corrections=applied,limitation='Input-Z adjustment only; must recapture and pass independent ground/Z/collision/visibility audit')
        meta['ground_support_check']=dict(enabled=True,max_range_m=20.,maximum_gap_m=.15)
        # A previous matched-height reference is not a ground reference and cannot
        # remain authoritative after changing input Z. Keep it only as provenance.
        probe=meta.get('motion_probe',{})
        if 'height_calibration' in probe:
            meta['ground_retry_provenance']['superseded_height_calibration']=probe.pop('height_calibration')
        gen.geo.write_json(sp,scene);gen.geo.write_json(mp,meta)
        new=copy.deepcopy(entry);new.update(id=sid,source_cell_id=entry['id'],scenario=str(sp.resolve()),
            metadata=str(mp.resolve()),scenario_sha256=gen.geo.sha256(sp),role=meta['spec']['role'],
            distance_bounds=meta['spec']['distance_bounds'],subfamily=meta['spec']['subfamily']);entries.append(new)
    manifest.update(count=len(entries),scenes=entries,status='GROUND_RETRY_PENDING_RECAPTURE')
    gen.geo.write_json(output/'manifest.json',manifest)
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True);p.add_argument('--captures',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(prepare(a.manifest,a.captures,a.output))
