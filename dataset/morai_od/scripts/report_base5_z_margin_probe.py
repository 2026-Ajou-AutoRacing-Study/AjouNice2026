#!/usr/bin/env python3
"""Input-Z response, receive-time transitions and raw RGB evidence, not OD scores."""
import argparse
import json
import math
from collections import defaultdict
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from base5_capture_safety import atomic_json, load_ledger


def summarize_trace(samples, uid, loaded_at, requested_at):
    trace = sorted((s for s in samples if s['uid'] == uid and s['t'] >= loaded_at), key=lambda s:s['t'])
    trace = [s for s in trace if all(math.isfinite(s[k]) for k in ('t', 'z'))]
    adjacent = [(b['t']-loaded_at, b['z']-a['z']) for a,b in zip(trace, trace[1:]) if 0 < b['t']-a['t'] <= .2]
    late = [s['z'] for s in trace if s['t'] >= requested_at-1.]
    return dict(samples=len(trace), first_z_m=trace[0]['z'] if trace else None,
        first_sample_after_load_sec=trace[0]['t']-loaded_at if trace else None,
        last_z_m=trace[-1]['z'] if trace else None,
        z_span_m=max(s['z'] for s in trace)-min(s['z'] for s in trace) if trace else None,
        maximum_adjacent_z_step_m=max((abs(s) for _,s in adjacent), default=None),
        transitions_over_10cm=[dict(after_load_sec=t, delta_z_m=z) for t,z in adjacent if abs(z)>.1],
        capture_window_z_median=float(np.median(late)) if late else None,
        capture_window_z_span_m=max(late)-min(late) if late else None)


def report(root, output):
    output.mkdir(parents=True, exist_ok=False)
    rows=[]; tiles=[]; curves=[]
    for result in load_ledger(root/'ledger.jsonl'):
        if not result.get('capture_root'):
            rows.append(dict(id=result['id'], status=result['status'], error=result.get('error')))
            continue
        cap=Path(result['capture_root'])
        meta=json.loads((cap/'metadata.json').read_text())
        telemetry=json.loads((cap/'telemetry.json').read_text())
        history=json.loads((cap/'collision_history.json').read_text())
        probe=meta['z_margin_probe']; actor=meta['runtime_actors'][0]; uid=actor['unique_id']
        load=history['load_time_monotonic']; request=telemetry['capture_requested_at_monotonic']
        # The first campaign reused the old probe UID. Before pose confirmation
        # late old-scene messages must not be called physics jumps.
        confirmed=history['pose_confirmed_monotonic']
        verified_samples=[s for s in telemetry['object_position_samples'] if s['t']>=confirmed]
        stats=summarize_trace(verified_samples, uid, load, request)
        audit=result['audit']; ground=next((a for a in audit['ground_support']['actors'] if a['uid']==uid), {})
        row=dict(id=result['id'], **probe, **stats, input_z_m=actor['position']['z'],
            capture_root=str(cap), status=result['status'], review_reasons=audit['review_reasons'],
            ground_gap_m=ground.get('minimum_annotation_ground_gap_m'), ground_verified=ground.get('verified'),
            ground_reason=ground.get('reason'), collision_clear=audit['capture_safety']['collision']['no_collision_verified'])
        row['pose_confirmed_after_load_sec']=confirmed-load
        row['pre_confirmation_samples_not_physics_evidence']=sum(s['uid']==uid and s['t']<confirmed for s in telemetry['object_position_samples'])
        if stats['capture_window_z_median'] is not None:
            row['runtime_minus_input_z_m']=stats['capture_window_z_median']-actor['position']['z']
        rows.append(row)
        traces=[s for s in verified_samples if s['uid']==uid]
        curves.append((row, traces, load, request))
        cams=[(c, cam) for c in audit['components'] for cam in c['cameras'] if cam.get('native_rect')]
        if not cams: continue
        component,cam=max(cams, key=lambda item:item[1]['native_instance_pixels'])
        raw=Image.open(next((cap/cam['camera']).glob('*_Intensity.png'))).convert('RGB')
        x0,y0,x1,y1=cam['native_rect']
        crop=raw.crop((max(0,int(x0)-30),max(0,int(y0)-30),min(raw.width,math.ceil(x1)+30),min(raw.height,math.ceil(y1)+40)))
        tile=Image.new('RGB',(260,320),'white')
        factor=min(240/crop.width,230/crop.height)
        crop=crop.resize((max(1,round(crop.width*factor)),max(1,round(crop.height*factor))),resample=Image.NEAREST)
        tile.paste(crop,((260-crop.width)//2,70))
        draw=ImageDraw.Draw(tile)
        draw.text((8,5), f"{probe['asset']} {probe['mode']} {probe['delta_m']:+.2f}m", fill='black')
        draw.text((8,22), f"pivot-input {row.get('runtime_minus_input_z_m', float('nan')):+.3f}m", fill='black')
        gap=row.get('ground_gap_m')
        draw.text((8,39), f"BBox-ground {gap:+.3f}m" if gap is not None else 'ground unverified', fill='black')
        tile.save(output/(result['id']+'.png')); tiles.append((row,tile))
    groups=defaultdict(list)
    for row in rows:
        if 'asset' in row:groups[(row['asset'],row['mode'])].append(row)
    summary=[]
    for (asset,mode),items in sorted(groups.items()):
        valid=[r for r in items if r.get('ground_gap_m') is not None]
        response=[]
        for r in sorted(items,key=lambda r:(r['repeat'],r['delta_m'])):
            baseline=next((b for b in items if b['repeat']==r['repeat'] and b['delta_m']==0),None)
            if baseline and r.get('capture_window_z_median') is not None and baseline.get('capture_window_z_median') is not None:
                response.append(dict(delta_m=r['delta_m'],repeat=r['repeat'],runtime_z_response_m=r['capture_window_z_median']-baseline['capture_window_z_median'],
                    ground_gap_m=r['ground_gap_m'],maximum_adjacent_z_step_m=r['maximum_adjacent_z_step_m']))
        summary.append(dict(asset=asset,mode=mode,captures=len(items),accepted=sum(r['status']=='accepted_candidate' for r in items),
            height_response=response, minimum_measured_ground_gap_m=min((r['ground_gap_m'] for r in valid),default=None)))
    if tiles:
        keys=sorted(groups); deltas=sorted({r['delta_m'] for r,_ in tiles}); repeats=sorted({r['repeat'] for r,_ in tiles})
        sheet=Image.new('RGB',(260*len(deltas),320*len(keys)*len(repeats)),'#cccccc')
        for row,tile in tiles:
            y=keys.index((row['asset'],row['mode']))*len(repeats)+repeats.index(row['repeat'])
            sheet.paste(tile,(260*deltas.index(row['delta_m']),320*y))
        sheet.save(output/'contact_sheet.png')
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(len(keys),1,figsize=(11,3*len(keys)),squeeze=False)
        for row,trace,load,request in curves:
            ax=axes[keys.index((row['asset'],row['mode']))][0]
            ax.plot([s['t']-load for s in trace],[s['z']-row['input_z_m'] for s in trace],label=f"{row['delta_m']:+.2f}m r{row['repeat']}")
        for key,ax in zip(keys,axes[:,0]):
            ax.set_title(' / '.join(key)); ax.set_ylabel('Object Z - input Z (m)'); ax.set_xlabel('Receive time since load (s)'); ax.grid(); ax.legend()
        fig.tight_layout(); fig.savefig(output/'z_response_traces.png'); plt.close(fig)
    shutdown=json.loads((root/'shutdown_safety.json').read_text()) if (root/'shutdown_safety.json').exists() else None
    data=dict(rows=rows,summary=summary,shutdown=shutdown,
        limitation='Receive-time traces cannot observe pre-publication physics. Object pivot != mesh bottom; local annotation-ground gap is not collider contact proof. Diagnostic only, not training selection.')
    atomic_json(output/'report.json',data)
    print(json.dumps(summary,indent=2)); return data


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();report(a.capture_root,a.output)
