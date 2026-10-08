#!/usr/bin/env python3
"""Two-cell MR integration probe, with explicitly SYNTHETIC contact input.

Never use this manifest/raw as training data. Injection proves workflow handling,
not simulator physical collision behaviour. No bulk collection is started.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import time

import run_base5_batch_campaign as batch
from base5_capture_safety import atomic_json, load_ledger, sha256


def fingerprints(root):
    return {str(p.relative_to(root)):[sha256(p),p.stat().st_ino,p.stat().st_mtime_ns,p.stat().st_size]
        for p in root.rglob('*') if p.is_file() and
        (p.name.endswith('_Intensity.png') or p.name.endswith('_Intensity.bin') or p.name.endswith('_Instance.txt'))}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--allow-test-contact-injection',action='store_true')
    args=parser.parse_args()
    if not args.allow_test_contact_injection:parser.error('Synthetic ROS contact injection requires explicit opt-in')
    if args.output.exists():raise FileExistsError('Fresh isolated diagnostic output required')
    original=json.loads(args.manifest.read_text());args.output.mkdir(parents=True)
    rows=[]
    for i,old in enumerate(original['scenes'][:2]):
        meta=json.loads(Path(old['metadata']).read_text())
        if any(a['motion'] not in ('static','standing') for a in meta['runtime_actors']):
            raise ValueError('Integration probe requires two stationary cells')
        ident=old['id']+'_replenishprobe';metadata=args.output/(ident+'_metadata.json')
        meta['diagnostic_only']='SYNTHETIC contact lifecycle test; not training; not proof of physical collision'
        atomic_json(metadata,meta)
        row=copy.deepcopy(old);row.update(id=ident,metadata=str(metadata.resolve()))
        rows.append(row)
    deck=batch.campaign.subset_deck(original,rows)
    deck.update(status='DIAGNOSTIC_ONLY_NOT_TRAINING',dataset_id='base5_replenishment_two_cell_probe')
    manifest=args.output/'manifest.json';atomic_json(manifest,deck)
    collection=args.output/'collection';first=rows[0]['id']
    expected=json.loads(Path(rows[0]['metadata']).read_text())['ego']
    import rospy
    from morai_msgs.msg import CollisionData
    rospy.init_node('base5_replenishment_contact_probe',anonymous=True,disable_signals=True)
    publisher=rospy.Publisher('/CollisionData',CollisionData,queue_size=1)
    injected=[]
    enabled=True
    def callback(message):
        if not enabled or message.header.frame_id=='BASE5_SYNTHETIC_CONTACT':return
        status=collection/'chunk_0000/capture/status.json'
        ledger=collection/'chunk_0000/capture/ledger.jsonl'
        if ledger.exists() and any(r['id']==first for r in load_ledger(ledger)):return
        try:current=json.loads(status.read_text()).get('current')
        except (OSError,ValueError):return
        ego=next((a for a in message.collision_object if a.unique_id==0),None)
        if current!=first or ego is None or math.hypot(ego.position.x-expected['x'],ego.position.y-expected['y'])>.25:return
        if not publisher.get_num_connections():return
        synthetic=copy.deepcopy(message);synthetic.header.frame_id='BASE5_SYNTHETIC_CONTACT'
        peer=copy.deepcopy(ego);peer.unique_id=987654;peer.name='TEST_INJECTED_NOT_PHYSICAL'
        synthetic.collision_objecta.append(peer)
        publisher.publish(synthetic)
        injected.append(dict(time=time.time(),seq=message.header.seq,uid=987654,
            source_stamp=message.header.stamp.to_sec(),label='SYNTHETIC_NOT_PHYSICAL'))
    subscriber=rospy.Subscriber('/CollisionData',CollisionData,callback,queue_size=50)
    try:initial=batch.execute(manifest,[0,1],collection,1,resume=True)
    finally:
        enabled=False;subscriber.unregister();publisher.unregister()
        atomic_json(args.output/'synthetic_contact_input.json',dict(events=injected,
            limitation='Injected ROS messages, not evidence of MORAI physical contact'))
    initial_status=json.loads((collection/'status.json').read_text())
    before=fingerprints(collection)
    selected_before=load_ledger(collection/'selected_ledger.jsonl')
    atomic_json(args.output/'initial_snapshot.json',dict(status=initial_status,selected=selected_before,sensors=before))
    if initial or initial_status['review']!=1 or initial_status['accepted_candidate']!=1 or not injected:
        raise RuntimeError('Probe did not reach one accepted / one synthetic-contact deficit; no replenishment')
    started=time.monotonic()
    code=batch.execute(manifest,[0,1],collection,1,resume=True,replenish=True)
    after=fingerprints(collection);selected_after=load_ledger(collection/'selected_ledger.jsonl')
    preserved=all(after.get(k)==v for k,v in before.items())
    state=json.loads((collection/'status.json').read_text())
    merged_bytes=(collection/'selected_ledger.jsonl').read_bytes()
    all_before=fingerprints(collection);events_before={str(p):sha256(p) for p in collection.rglob('event_cmd_requests.jsonl')}
    verify=batch.execute(manifest,[0,1],collection,1,resume=True)
    repeat=batch.execute(manifest,[0,1],collection,1,resume=True,replenish=True)
    verification=(fingerprints(collection)==all_before and
        events_before=={str(p):sha256(p) for p in collection.rglob('event_cmd_requests.jsonl')} and
        merged_bytes==(collection/'selected_ledger.jsonl').read_bytes() and
        state['elapsed_sec']==json.loads((collection/'status.json').read_text())['elapsed_sec'])
    report=dict(initial_code=initial,replenish_code=code,verify_code=verify,repeat_code=repeat,
        initial_accepted=len(selected_before),final_accepted=len(selected_after),
        replenished=state.get('replenished'),remaining_deficit=state['review'],
        original_sensor_files=len(before),original_sensors_preserved=preserved,
        verification_no_capture=verification,existing_success_preserved=all(r in selected_after for r in selected_before),
        replenish_and_verify_wall_sec=time.monotonic()-started,
        limitation='Two stationary MR cells with SYNTHETIC contact input; no bulk or physical-contact claim')
    atomic_json(args.output/'report.json',report);print(json.dumps(report,indent=2),flush=True)
    return 0 if not any((code,verify,repeat)) and len(selected_after)==2 and preserved and verification else 1


if __name__=='__main__':raise SystemExit(main())
