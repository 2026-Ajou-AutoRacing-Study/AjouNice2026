#!/usr/bin/env python3
"""Use independently observed empty terrain, adjust input Z only, then recapture."""
import argparse
import copy
import json
from pathlib import Path
import generate_base5_independent as gen
from base5_capture_audit import rotation_matrix
from base5_ground_probe import world_cloud,surface_at,ego_to_world,plane_height,supported
from base5_capture_safety import atomic_json,sha256
from audit_morai_lidar_gt_visibility import parse_instances
from prepare_base5_ground_retry import shift_actor
import numpy as np


def prepare(source,original,probes,output):
    if output.exists():raise FileExistsError('Probed retry overwrite refused')
    if not json.loads((probes/'shutdown_safety.json').read_text())['park_verified']:raise ValueError('Unsafe probe exit')
    if not json.loads((original/'shutdown_safety.json').read_text())['park_verified']:raise ValueError('Unsafe source exit')
    m=gen.geo.load_json(source);byid={r['id']:r for r in m['scenes']}
    oldrows={r['id']:r for r in map(json.loads,(original/'ledger.jsonl').read_text().splitlines())}
    probe_rows={r['id']:r for r in map(json.loads,(probes/'ledger.jsonl').read_text().splitlines())}
    if not 1<=len(probe_rows)<=8:raise ValueError('Probed retry limited to eight cells')
    output.mkdir(parents=True);entries=[];rejected=[]
    for result in probe_rows.values():
        if result['status']!='accepted_candidate':rejected.append(dict(id=result['id'],reason='probe capture not accepted'));continue
        root=Path(result['capture_root']);pm=gen.geo.load_json(root/'metadata.json');pt=gen.geo.load_json(root/'telemetry.json')
        if not pm.get('calibration_only') or pm['runtime_actors']:raise ValueError('Ground probe is not empty calibration')
        error=(pt['ego_before']['heading']-1.347-pm['ego']['yaw']+180)%360-180
        if abs(error)>.2:raise ValueError('Probe MR heading contract mismatch')
        sourceid=pm['ground_probe_source']['id'];entry=byid[sourceid]
        if entry['scenario_sha256']!=pm['ground_probe_source']['scenario_sha256']:raise ValueError('Probe source changed')
        scene=gen.geo.load_json(Path(entry['scenario']));meta=gen.geo.load_json(Path(entry['metadata']))
        old=oldrows[sourceid];cap=Path(old['capture_root']);cm=gen.geo.load_json(cap/'metadata.json');ct=gen.geo.load_json(cap/'telemetry.json')
        boxes=parse_instances(next(cap.glob('LIDAR_*/*_Instance.txt')));cloud=world_cloud(root,pm,pt)
        refs=[];shifts=[]
        for actor in meta['runtime_actors']:
            # ego_to_world currently knows yaw, not physical Ego roll/pitch.
            # A nearby witness viewed from a distant Ego can therefore create
            # an apparent vertical gap. Keep distant actors required in the
            # audit, but never lower them from this yaw-only cross-view proof.
            if actor['planned_center_distance_m']>20.:continue
            uid=actor['unique_id'];cs=[]
            for box in boxes:
                if box['unique_id']!=uid:continue
                signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
                lidar=signs*box['size']/2@rotation_matrix(box['rotation_rpy']).T+box['center']
                ego=(np.c_[lidar,np.ones(8)]@np.array(cm['lidar_to_ego']).T)[:,:3]
                cs.extend(ego_to_world(ego,ct['ego_before']))
            if not cs:continue
            cs=np.array(cs);plane=surface_at(cloud,cs[:,:2].mean(axis=0),support_xy=cs[:,:2])
            if plane is None or not all(supported(plane,c[:2]) for c in cs):continue
            refs.append(dict(uid=uid,plane_world=plane,probe_capture=str(root),
                probe_scenario_sha256=result['scenario_sha256'],probe_lidar_sha256=sha256(next(root.glob('LIDAR_*/*_Intensity.bin'))),
                MR_heading_correction_deg=-1.347,maximum_world_support_extrapolation_m=.75))
            gap=float(np.min(cs[:,2]-[plane_height(plane,c[:2]) for c in cs]))
            if gap>.15:
                delta=-gap-.03
                try:shift_actor(scene,meta,uid,delta)
                except ValueError:continue
                shifts.append(dict(uid=uid,delta_z_m=delta,before_ground_gap_m=gap))
            # An apparent below-ground local fit is never reason to raise the
            # asset without a confirmed independent physical placement model.
        if not refs:rejected.append(dict(id=sourceid,reason='no supported independent surface'));continue
        sid=sourceid+'_probed';sp=output/(sid+'.json');mp=output/(sid+'_metadata.json')
        previous={r['uid']:r for r in meta.get('ground_probe_references',[])}
        previous.update({r['uid']:r for r in refs})
        meta['ground_probe_references']=list(previous.values())
        meta['ground_support_check']=copy.deepcopy(meta.get('ground_support_check',{}))
        meta['ground_support_check'].update(enabled=True,max_range_m=20.,maximum_gap_m=.15)
        meta['probed_retry_provenance']=dict(source_capture=str(cap),source_scenario_sha256=entry['scenario_sha256'],
            input_z_corrections=shifts,limitation='Same-scene empty surface witness; no universal ground offset or raw GT rewrite')
        gen.geo.write_json(sp,scene);gen.geo.write_json(mp,meta)
        row=copy.deepcopy(entry);row.update(id=sid,source_cell_id=sourceid,scenario=str(sp.resolve()),metadata=str(mp.resolve()),scenario_sha256=sha256(sp));entries.append(row)
    m.update(count=len(entries),scenes=entries,status='PROBED_GROUND_RETRY_PENDING_RECAPTURE')
    atomic_json(output/'manifest.json',m);atomic_json(output/'preparation_report.json',dict(prepared=len(entries),rejected=rejected))
    return output/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--original-captures',type=Path,required=True);p.add_argument('--probe-captures',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(prepare(a.manifest,a.original_captures,a.probe_captures,a.output))
