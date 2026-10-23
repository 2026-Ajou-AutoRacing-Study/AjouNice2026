#!/usr/bin/env python3
"""새 CPU 배치와 완료한 신뢰된 로컬 전달본의 geometry/split을 함께 감사한다.

--collected-package는 자체 SHA manifest와 train/val infos를 이미 검증한
로컬 패키지만 지정한다. infos pickle은 임의 외부 다운로드를 허용하는 입력이 아니다.
원본·GT·분할은 수정하지 않으며 보고서만 새로운 출력 폴더에 작성한다.
"""
import argparse
from collections import Counter,defaultdict
import copy
import json
from pathlib import Path
import pickle
import sys

SCRIPTS=Path(__file__).resolve().parents[2]/'dataset/morai_od/scripts'
sys.path.insert(0,str(SCRIPTS))
import base5_layout_split as layout
import base5_unit_geometry_contract as unit
from base5_capture_safety import atomic_json,sha256


def require(value,message):
    if not value:raise ValueError(message)


def read(path):return json.loads(Path(path).read_text())


def compact(meta,group):
    actors=[]
    keys=('name','data_id','source_list','position','placement_yaw_deg','estimated_size',
          'motion','motion_path','asset_scale','planned_center_distance_m','actual_ego_role')
    for actor in meta['runtime_actors']:
        actors.append({key:copy.deepcopy(actor[key]) for key in keys if key in actor})
    return dict(ego=copy.deepcopy(meta['ego']),runtime_actors=actors,spec=copy.deepcopy(meta['spec']),
                scene_group_id=group,logical_quota_slot_id=meta.get('logical_quota_slot_id'))


def package_records(root,label,pins):
    root=Path(root).resolve(strict=True);manifest=root/'manifest.json';doc=read(manifest)
    require(doc.get('scenes')==5000,'Complete 5K trusted package required: '+label)
    files=doc['files'];pins[str(manifest)]=sha256(manifest)
    def checked(relative):
        path=(root/relative).resolve(strict=True)
        require(root in path.parents,'Package input escaped the root')
        require(relative in files and sha256(path)==files[relative]['sha256'],
                'Package declared file SHA differs: '+relative)
        pins[str(path)]=files[relative]['sha256']
        return path
    result=[];seen=set();splits=Counter()
    for split in ('train','val'):
        path=checked('infos/512x1056/morai_base5_infos_'+split+'.pkl')
        # Explicitly trusted local package; checksum is checked before deserialization.
        infos=pickle.loads(path.read_bytes())['data_list']
        for info in infos:
            ident=info['sample_idx']
            require(type(ident) is str and Path(ident).name==ident and ident not in seen,
                    'Invalid or duplicate selected package sample')
            seen.add(ident);splits[split]+=1
            meta=read(checked('data/'+ident+'/metadata.json'))
            checked('data/'+ident+'/scenario.json')
            group=info.get('scene_group_id') or info['scene_id']
            result.append(dict(id=label+':'+ident,environment=label,
                meta=compact(meta,group),split=split))
    require(len(result)==5000 and splits==Counter(train=4250,val=750),
            'Selected package must have the actual final 4250/750 split')
    return result


def candidate_records(path,pins):
    path=Path(path).resolve(strict=True);deck=read(path);pins[str(path)]=sha256(path)
    label=deck['config']['environment_tag'];result=[]
    require(deck.get('count')==len(deck['scenes'])==5000,'Complete fresh candidate required')
    for entry in deck['scenes']:
        require(sha256(entry['scenario'])==entry['scenario_sha256'],'Candidate scenario SHA changed')
        pins[str(Path(entry['scenario']).resolve())]=entry['scenario_sha256']
        meta=read(entry['metadata']);pins[str(Path(entry['metadata']).resolve())]=sha256(entry['metadata'])
        group=entry.get('derived_layout_group_id') or meta['scene_group_id']
        split=entry.get('derived_training_split',entry['split'])
        result.append(dict(id=label+':'+entry['id'],environment=label,
            meta=compact(meta,group),split=split,index=len(result)))
    return deck,result


def distribution(rows,physical_assets):
    count=Counter();assets=Counter();family=Counter();split=Counter()
    for row in rows:
        meta=row['meta'];family[meta['spec']['family']]+=1;split[row['split']]+=1
        moving=any(actor['motion'] not in ('static','standing') for actor in meta['runtime_actors'])
        count['moving_scenes' if moving else 'static_scenes']+=1
        if moving and meta['spec']['family']=='shadow':count['shadow_moving_scenes']+=1
        for actor in meta['runtime_actors']:
            count['planned_parents']+=1;assets[actor['name']]+=1
            distance=actor['planned_center_distance_m']
            if actor['name'] in physical_assets:
                count['physical_'+('0_20m' if distance<20 else '20_30m' if distance<30 else '30plus_m')]+=1
            if actor['source_list']=='vehicleList':
                count['vehicle_'+('0_30m' if distance<30 else '30_50m' if distance<50 else '50_80m' if distance<80 else '80_100m' if distance<=100 else '100plus_m')]+=1
                if distance>=50:count['far_vehicle_'+actor.get('actual_ego_role','unknown')]+=1
    return dict(counts=dict(count),asset_parents=dict(sorted(assets.items())),family=dict(family),split=dict(split),
        counts_are_scenario_parent_layouts_not_visible_gt=True)


def joint_audit(rows,candidate_environment):
    group_of,fingerprints,near=layout.groups(rows,layout.configuration())
    members=defaultdict(list)
    for row in rows:
        members[group_of[row['id']]].append(dict(id=row['id'],environment=row['environment'],
            split=row['split'],geometry_sha256=fingerprints[row['id']]))
    cross=[];conflicts=[];new=[]
    for group,items in members.items():
        if len({row['environment'] for row in items})>1:
            item=dict(group=group,members=items);cross.append(item)
            if len({row['split'] for row in items})>1:conflicts.append(item)
            if any(row['environment']==candidate_environment for row in items):new.append(item)
    return dict(total_scenes=len(rows),groups=len(members),near_pairs=near,
        cross_environment_groups=cross,cross_split_groups=conflicts,
        candidate_duplicate_groups=new,passed=not new,
        known_map_background_overlap_allowed=True,
        geometry_is_actual_selected_initializer_not_sensor_frame_gt=True)


def audit(candidate,packages,output):
    output=Path(output).absolute();require(not output.exists(),'Fresh audit output required')
    pins={};deck,new=candidate_records(candidate,pins);rows=list(new);labels={deck['config']['environment_tag']}
    native=unit.load(deck['config'][unit.KEY]);physical_assets=set(native['assets'])
    stats={deck['config']['environment_tag']:distribution(new,physical_assets)}
    for label,root in packages:
        require(label not in labels,'Environment must be unique in this audit')
        labels.add(label);previous=package_records(root,label,pins);rows.extend(previous)
        stats[label]=distribution(previous,physical_assets)
    joint=joint_audit(rows,deck['config']['environment_tag'])
    for path,expected in pins.items():require(sha256(path)==expected,'Audit input changed: '+path)
    output.mkdir(mode=0o700)
    atomic_json(output/'input_pins.json',pins)
    report=dict(candidate=str(Path(candidate).resolve()),candidate_sha256=sha256(candidate),
        environments=sorted(labels),distribution=stats,joint=joint,
        input_pins_sha256=sha256(output/'input_pins.json'),input_files_verified=len(pins),
        source_and_training_splits_modified=False,native_captures=0,bulk_ready=False,
        passed=joint['passed'])
    atomic_json(output/'report.json',report)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate',type=Path,required=True)
    parser.add_argument('--collected-package',action='append',required=True,help='ENVIRONMENT=/trusted/local/package')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    pairs=[value.split('=',1) for value in args.collected_package]
    require(all(len(pair)==2 for pair in pairs),'ENVIRONMENT=PATH required')
    report=audit(args.candidate,pairs,args.output)
    print(json.dumps(dict(passed=report['passed'],scenes=report['joint']['total_scenes'],
        groups=report['joint']['groups'],candidate_duplicates=len(report['joint']['candidate_duplicate_groups']),
        cross_split_groups=len(report['joint']['cross_split_groups']),output=str(args.output)),indent=2))


if __name__=='__main__':main()
