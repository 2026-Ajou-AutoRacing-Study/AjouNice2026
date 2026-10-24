#!/usr/bin/env python3
"""Build v87: straighten GIS-referenced merge borders and compact aligned lanes.

The entry keeps one necessary split beside 201456: merging across that split
would remove LC edges in stock Lanelet2. Other parallel compactions preserve
whole shared boundaries. No planner/safety-parameter change.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
from collections import defaultdict
import json
import math
from pathlib import Path

from lxml import etree as E
from pyproj import Transformer
import shapefile

from build_v86_highway_lane_end_lc import tags, refs, member, set_refs, sha256, primitive_diff

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'lanelet_map/lanelet2_map_v86.osm'
OUTPUT=ROOT/'lanelet_map/lanelet2_map_v87.osm'
REPORT=ROOT/'docs/kcity_v87_highway_gis_compaction_build.json'
BASE_SHA='55ee0ce9c4b8489941994cb50e29eb3526d17e7512a41bb86886ebd5576e86bd'
GROUPS={
    201468:[201438,201468], 201467:[200099,201467],
    201466:[201435,201466], 201465:[201436,201465], 201464:[201437,201464],
    201439:[200127,201473,201439], 201441:[200130,201474,201441],
    201446:[200134,201475,201446], 201440:[200148,201472,201440],
}
REMAPPED={i:keep for keep,group in GROUPS.items() for i in group if i!=keep}
SOURCE_LANES={i for group in GROUPS.values() for i in group}
EDIT_LANES=SOURCE_LANES|{201456,201457,201458}


def build(output=OUTPUT, report=REPORT):
    if output.exists(): raise RuntimeError(f'Refusing to overwrite: {output}')
    assert sha256(BASE)==BASE_SHA
    tree=E.parse(str(BASE), E.XMLParser(remove_blank_text=False,huge_tree=True));root=tree.getroot()
    old=deepcopy(root)
    nodes={int(e.get('id')):e for e in root.findall('node')}
    ways={int(e.get('id')):e for e in root.findall('way')}
    rels={int(e.get('id')):e for e in root.findall('relation')}
    before_rels={i:deepcopy(rels[i]) for i in SOURCE_LANES}
    before_way_refs={i:refs(w) for i,w in ways.items()}
    f=Transformer.from_crs(4326,32652,always_xy=True);inv=Transformer.from_crs(32652,4326,always_xy=True)
    ox,oy=f.transform(126.7730819386,37.2380038082)
    xy_cache={}
    def xyz(i):
        if i not in xy_cache:
            n=nodes[i];x,y=f.transform(float(n.get('lon')),float(n.get('lat')))
            xy_cache[i]=(x-ox,y-oy,float(tags(n).get('ele',0)))
        return xy_cache[i]
    def wid(rel,role):return int(member(rel,role).get('ref'))
    def set_xyz(i,p):
        lon,lat=inv.transform(p[0]+ox,p[1]+oy);n=nodes[i]
        n.set('lon',f'{lon:.14f}');n.set('lat',f'{lat:.14f}')
        e=next((x for x in n.findall('tag') if x.get('k')=='ele'),None)
        if e is None:e=E.SubElement(n,'tag',k='ele')
        e.set('v',f'{p[2]:.9f}');xy_cache[i]=tuple(p)
    new_id=max(nodes)+1
    def add_node(p):
        nonlocal new_id
        i=new_id;new_id+=1
        n=E.Element('node',id=str(i));nodes[i]=n
        root.insert(root.index(root.find('way')),n);set_xyz(i,p);return i
    def concatenate(way_ids):
        result=[]
        for i in way_ids:
            seq=before_way_refs[i]
            if result:
                assert result[-1]==seq[0],(way_ids,result[-1],seq[0])
                result.extend(seq[1:])
            else:result.extend(seq)
        return result

    affected_ways={wid(rels[i],role) for i in EDIT_LANES for role in ['left','right','centerline']}
    candidates={n for w in affected_ways for n in before_way_refs[w]}
    # Moved shared points may be used only by this cross-section.
    node_way_users=defaultdict(set)
    for i,w in ways.items():
        for n in refs(w):node_way_users[n].add(i)
    for n in [131840,131841,131839]:assert node_way_users[n]<=affected_ways,(n,node_way_users[n]-affected_ways)

    # Fit each long separator with fixed external endpoints. GIS supplies the
    # physical identity and deviation audit; fixed interfaces preserve all
    # outside lanelets. The short 7 m wiggle is removed completely.
    moved=[]
    for node,first,last in [(131840,231223,10002),(131841,231226,45476)]:
        original=xyz(node);a,b=xyz(first),xyz(last)
        t=(original[1]-a[1])/(b[1]-a[1]);p=(a[0]+t*(b[0]-a[0]),original[1],original[2])
        set_xyz(node,p);moved.append({'node':node,'before':original,'after':p,'reason':'collinear shared entry separator with fixed external endpoints'})

    gis={}
    for sr in shapefile.Reader(str(ROOT/'gis_map/B2_SURFACELINEMARK'),encoding='utf-8').iterShapeRecords():
        d=sr.record.as_dict()
        if d['ID'] in ['B2256W000038','B2256W000041','B2256W000043','B2256W000025','B2256W001693','B2256W000061','B2256W000042']:
            gis[d['ID']]=[(p[0]-ox,p[1]-oy) for p in sr.shape.points]
    # At the start of the triangle use the surveyed road exterior, not the
    # v86 offset corner. Downstream tip remains the external v86 interface.
    original=xyz(131839);points=gis['B2256W000061']
    for a,b in zip(points,points[1:]):
        if a[1]>=original[1]>=b[1]:
            t=(original[1]-a[1])/(b[1]-a[1]);p=(a[0]+t*(b[0]-a[0]),original[1],original[2]);break
    else:raise AssertionError('GIS exterior does not cover entry joint')
    set_xyz(131839,p);moved.append({'node':131839,'before':original,'after':p,'reason':'entry exterior joint on raw GIS B2256W000061'})

    # A shared way chain always resolves to one shared way ID after merging.
    # Keep each downstream way ID; delete superseded target ways only later.
    cache={};merge_log=[]
    for keep,group in GROUPS.items():
        assert all(tags(before_rels[i])==tags(before_rels[keep]) for i in group),group
        for i in group:
            assert not [m for m in before_rels[i].findall('member') if m.get('type')!='way'],i
        row={'retained':keep,'absorbed':[i for i in group if i!=keep],'boundaries':{}}
        for role in ['left','right']:
            chain_ids=tuple(wid(before_rels[i],role) for i in group)
            if chain_ids not in cache:
                seq=concatenate(chain_ids);dest=chain_ids[-1]
                # The far-left external borders are not part of the requested
                # straightening; preserve their exact original polylines.
                preserve=keep in [201464,201440] and role=='left'
                set_refs(ways[dest],seq if preserve else [seq[0],seq[-1]])
                cache[chain_ids]=dest
            dest=cache[chain_ids]
            member(rels[keep],role).set('ref',str(dest))
            row['boundaries'][role]={'way':dest,'old_ways':list(chain_ids)}
        # A terminating lane has left-only LC over its whole compacted span.
        if keep in [201468,201441,201446]:
            edge=ways[wid(rels[keep],'left')]
            for tag in list(edge.findall('tag')):edge.remove(tag)
            E.SubElement(edge,'tag',k='type',v='virtual')
            E.SubElement(edge,'tag',k='lane_change:left',v='yes')
            E.SubElement(edge,'tag',k='lane_change:right',v='no')
        merge_log.append(row)

    # Keep 201456 and 201457 separate to preserve early LC. Main-line
    # 200099+201467 aligns with the merged 201438+201468 triangle.
    for i in REMAPPED:
        root.remove(rels[i]);del rels[i]
    for rel in rels.values():
        for m in rel.findall('member'):
            assert not (m.get('type')=='relation' and int(m.get('ref')) in REMAPPED),dict(m.attrib)

    # Regenerate only local centerlines. Reuse old outside joint point IDs
    # where possible, without moving any point used outside this edit area.
    final_lanes=set(GROUPS)|{201456,201457,201458}
    center_endpoints={}
    for i in sorted(final_lanes):
        left=refs(ways[wid(rels[i],'left')]);right=refs(ways[wid(rels[i],'right')])
        for end in [0,-1]:
            key=(left[end],right[end]);p=tuple((xyz(key[0])[j]+xyz(key[1])[j])/2 for j in range(3))
            center_endpoints[key]=p
    external_centers=defaultdict(list)
    for i,rel in rels.items():
        if i in final_lanes or tags(rel).get('type')!='lanelet':continue
        ms={m.get('role'):int(m.get('ref')) for m in rel.findall('member') if m.get('type')=='way'}
        if not {'left','right','centerline'}<=ms.keys():continue
        left,right,center=[refs(ways[ms[x]]) for x in ['left','right','centerline']]
        for end in [0,-1]:external_centers[(left[end],right[end])].append(center[end])
    joint_nodes={}
    for key,p in sorted(center_endpoints.items()):
        if key[0]==key[1]:joint_nodes[key]=key[0];continue
        existing=sorted(external_centers.get(key,[]))
        valid=[n for n in existing if math.dist(xyz(n)[:2],p[:2])<.001]
        joint_nodes[key]=valid[0] if valid else add_node(p)
    def interp(seq,t):
        ps=[xyz(n) for n in seq];y=ps[0][1]+t*(ps[-1][1]-ps[0][1])
        for a,b in zip(ps,ps[1:]):
            if a[1]+1e-6>=y>=b[1]-1e-6:
                f=(y-a[1])/(b[1]-a[1]);return tuple(a[j]+f*(b[j]-a[j]) for j in range(3))
        raise AssertionError((seq,t,y))
    for i in sorted(final_lanes):
        rel=rels[i];left,right=[refs(ways[wid(rel,x)]) for x in ['left','right']]
        ts={0.,1.}
        for seq in [left,right]:
            a,b=xyz(seq[0])[1],xyz(seq[-1])[1]
            ts.update((xyz(n)[1]-a)/(b-a) for n in seq[1:-1])
        seq=[]
        for t in sorted(ts):
            if t==0:seq.append(joint_nodes[(left[0],right[0])])
            elif t==1:seq.append(joint_nodes[(left[-1],right[-1])])
            else:
                a,b=interp(left,t),interp(right,t);seq.append(add_node(tuple((a[j]+b[j])/2 for j in range(3))))
        set_refs(ways[wid(rel,'centerline')],seq)

    way_refs={int(m.get('ref')) for rel in rels.values() for m in rel.findall('member') if m.get('type')=='way'}
    removed_ways=affected_ways-way_refs
    for i in removed_ways:root.remove(ways[i])
    used_nodes={int(n.get('ref')) for w in root.findall('way') for n in w.findall('nd')}
    used_nodes|={int(m.get('ref')) for rel in rels.values() for m in rel.findall('member') if m.get('type')=='node'}
    for i in candidates-used_nodes:
        assert not set(tags(nodes[i]))-{'local_x','local_y','ele'},i
        root.remove(nodes[i])
    root.find('MetaInfo').set('map_version','87')
    diff=primitive_diff(old,root)
    assert set(diff['relation']['changed'])<=set(GROUPS)
    assert diff['relation']['removed']==sorted(REMAPPED)
    assert not diff['relation']['added']
    assert set(diff['way']['changed'])<=affected_ways
    assert not diff['way']['added']
    assert set(diff['node']['changed'])=={131839,131840,131841}
    output.parent.mkdir(parents=True,exist_ok=True)
    tree.write(str(output),encoding='UTF-8',xml_declaration=True,pretty_print=False)
    result={'baseline':str(BASE),'baseline_sha256':sha256(BASE),'output':str(output),'sha256':sha256(output),
            'version':87,'groups':merge_log,'removed_to_retained':REMAPPED,'primitive_diff':diff,
            'allowed_source_lanelets':sorted(EDIT_LANES),'moved_boundary_nodes':moved,
            'entry_exception':'201457 remains separate from 201467 to preserve LC beside separate 201456 and merged 201468. Their shared border is collinear.',
            'parallel_compaction_reason':'Merge the minimal adjacent lane cross-section to retain shared whole-way LC edges.',
            'gis_sources':{'entry_separator':'B2256W000038','entry_exterior_joint':'B2256W000061','entry_main_left':'B2256W000041','bottleneck_main_separator':'B2256W000043','bottleneck_virtual_divider':'A2_LINK 000422/000445 geometry and taper endpoints; no B2 paint feature'},
            'outside_region':'No existing primitive outside target lanelets and their owned local geometry modified.',
            'planner_changes':False}
    report.parent.mkdir(parents=True,exist_ok=True);report.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,default=OUTPUT);p.add_argument('--report',type=Path,default=REPORT);a=p.parse_args()
    print(json.dumps(build(a.output,a.report),ensure_ascii=False,indent=2))
