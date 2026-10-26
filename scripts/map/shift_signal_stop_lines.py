#!/usr/bin/env python3
"""Create an immutable map candidate by translating signal stop-line nodes.

Preserve every original byte except lat/lon and optional local_x/local_y of
exclusive ref_line nodes. Lane boundaries, route topology, light housings,
elevations and regulatory IDs never move. Source/target must be different.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from pyproj import Transformer

GROUPS = dict(A=201351, B=201365, C=201290, D=201326)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def shifted_map(data, shifts, geometry):
    if set(shifts)-set(GROUPS) or any(isinstance(v, bool) or not isinstance(v,(int,float))
            or not math.isfinite(v) or not 0 <= v <= 10 for v in shifts.values()):
        raise ValueError('Use explicit A/B/C/D upstream distances in [0,10] metres')
    text = data.decode('utf-8')
    root = ET.fromstring(text)
    nodes = {n.get('id'):n for n in root.findall('node')}
    ways = {w.get('id'):w for w in root.findall('way')}
    relations = {r.get('id'):r for r in root.findall('relation')}
    usage = {}
    for way in ways.values():
        for nd in way.findall('nd'):
            usage.setdefault(nd.get('ref'),set()).add(way.get('id'))
    projected = Transformer.from_crs('EPSG:4326','EPSG:32652',always_xy=True)
    inverse = Transformer.from_crs('EPSG:32652','EPSG:4326',always_xy=True)
    updates, report = {}, []
    for mission, gid in GROUPS.items():
        distance = float(shifts.get(mission,0.))
        if distance == 0: continue
        relation = relations[str(gid)]
        lines = [m.get('ref') for m in relation.findall('member')
                 if m.get('type')=='way' and m.get('role')=='ref_line']
        if len(lines)!=1: raise ValueError('Exactly one signal ref_line required: '+mission)
        way = ways[lines[0]]
        if {t.get('k'):t.get('v') for t in way.findall('tag')}.get('subtype')!='stop_line':
            raise ValueError('ref_line is not a stop line')
        headings = [h['heading'] for h in geometry['heads'] if h.get('group_id')==gid]
        if not headings or max(headings)-min(headings)>1.:
            raise ValueError('Unambiguous approach direction required')
        direction = math.radians(headings[0]+180.)
        dx,dy = -distance*math.cos(direction),-distance*math.sin(direction)
        changes = []
        for nd in way.findall('nd'):
            nid = nd.get('ref')
            if usage[nid]!={way.get('id')} or nid in updates:
                raise ValueError('Stop node shared with another way; do not move road geometry')
            node = nodes[nid]
            x,y = projected.transform(float(node.get('lon')),float(node.get('lat')))
            lon,lat = inverse.transform(x+dx,y+dy)
            matches = list(re.finditer(r'<node\b(?=[^>]*\bid="'+re.escape(nid)+r'")[^>]*>.*?</node>',text,re.S))
            if len(matches)!=1: raise ValueError('Unique tagged node block required')
            old = matches[0].group(); new = old
            for key,value in (('lat',lat),('lon',lon)):
                new,n = re.subn(r'\b'+key+r'="[^"]+"',key+'="'+format(value,'.11f')+'"',new,count=1)
                if n!=1: raise ValueError('Missing coordinate')
            for key,delta in (('local_x',dx),('local_y',dy)):
                tags = [t for t in node.findall('tag') if t.get('k')==key]
                if tags:
                    value = float(tags[0].get('v'))+delta
                    new,n = re.subn(r'(<tag\s+k="'+key+r'"\s+v=")[^"]+("\s*/>)',
                                   lambda m:m[1]+format(value,'.6f')+m[2],new,count=1)
                    if n!=1: raise ValueError('Unsupported local coordinate tag format')
            updates[nid] = (old,new)
            changes.append(dict(node_id=int(nid),old_utm_xy=[x,y],new_utm_xy=[x+dx,y+dy]))
        report.append(dict(mission=mission,regulatory_id=gid,way_id=int(way.get('id')),
                           upstream_shift_m=distance,delta_xy_m=[dx,dy],nodes=changes))
    for old,new in updates.values():
        if text.count(old)!=1: raise ValueError('Node replacement is not unique')
        text = text.replace(old,new,1)
    # A byte-exact reversal proves nothing outside the qualified nodes changed.
    reversed_text = text
    for old,new in updates.values(): reversed_text = reversed_text.replace(new,old,1)
    if reversed_text.encode()!=data: raise AssertionError('Unexpected non-target map change')
    ET.fromstring(text)
    return text.encode(),dict(source_sha256=sha(data),candidate_sha256=sha(text.encode()),
        changed_node_ids=sorted(map(int,updates)),changes=report,
        unchanged='all bytes outside selected exclusive stop-line node blocks',
        direction='MGeo physical head heading+180, in the existing UTM axes; no Ego yaw correction')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--geometry',type=Path,required=True)
    p.add_argument('--report',type=Path,required=True)
    p.add_argument('--shift',action='append',default=[],metavar='A=0.75')
    a = p.parse_args()
    if a.source.resolve()==a.output.resolve() or a.output.exists() or a.report.exists():
        p.error('Use a new candidate and report; never overwrite a source or existing result')
    shifts={}
    for item in a.shift:
        name,value=item.split('=',1)
        if name in shifts:p.error('Duplicate mission shift')
        shifts[name]=float(value)
    raw=a.source.read_bytes()
    result,report=shifted_map(raw,shifts,json.loads(a.geometry.read_text()))
    report.update(source=str(a.source.resolve()),output=str(a.output.resolve()),
                  geometry_sha256=sha(a.geometry.read_bytes()))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.report.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('xb') as f:f.write(result)
    with a.report.open('x') as f:json.dump(report,f,ensure_ascii=False,indent=2)
    assert a.source.read_bytes()==raw
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
