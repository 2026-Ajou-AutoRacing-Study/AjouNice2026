#!/usr/bin/env python3
"""Independently validate v87 scope, geometry, compaction, LC, and GIS alignment."""
from __future__ import annotations
import argparse
from collections import defaultdict
import ctypes
import itertools
import json
import math
from pathlib import Path
import sys

from lxml import etree as E
import shapefile
from pyproj import Transformer
from shapely.geometry import LineString,Point,Polygon
import yaml

from build_v87_highway_gis_compaction import ROOT,BASE,BASE_SHA,OUTPUT,GROUPS,REMAPPED,SOURCE_LANES,EDIT_LANES
from build_v86_highway_lane_end_lc import sha256,tags,refs,member,primitive_diff
from validate_v86_highway_lane_end_lc import signature


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--map',type=Path,default=OUTPUT)
    p.add_argument('--report',type=Path,default=ROOT/'docs/kcity_v87_highway_gis_compaction_validation.json')
    p.add_argument('--profiles',action='store_true');args=p.parse_args()
    assert sha256(BASE)==BASE_SHA
    old_root,new_root=[E.parse(str(f),E.XMLParser(remove_blank_text=False,huge_tree=True)).getroot() for f in [BASE,args.map]]
    old={k:{int(x.get('id')):x for x in old_root.findall(k)} for k in ['node','way','relation']}
    new={k:{int(x.get('id')):x for x in new_root.findall(k)} for k in old}
    diff=primitive_diff(old_root,new_root)
    assert new_root.find('MetaInfo').get('map_version')=='87'
    assert diff['relation']=={'added':[],'removed':sorted(REMAPPED),'changed':[]},diff['relation']
    allowed_ways={int(m.get('ref')) for i in EDIT_LANES for m in old['relation'][i].findall('member') if m.get('type')=='way'}
    allowed_nodes={n for i in allowed_ways for n in refs(old['way'][i])}
    assert not diff['way']['added']
    assert set(diff['way']['changed']+diff['way']['removed'])<=allowed_ways
    assert set(diff['node']['changed'])=={131839,131840,131841}
    assert set(diff['node']['removed'])<=allowed_nodes
    for i in set(old['node'])&set(new['node']):
        if i not in {131839,131840,131841}:
            assert E.tostring(old['node'][i],with_tail=False)==E.tostring(new['node'][i],with_tail=False),i
    for i,w in old['way'].items():
        if i not in allowed_ways:
            assert E.tostring(w,with_tail=False)==E.tostring(new['way'][i],with_tail=False),i
            assert not set(refs(w))&{131839,131840,131841},i
    for k,items in new.items():assert len(items)==len(new_root.findall(k)),('duplicate ID',k)
    for i,w in new['way'].items():assert set(refs(w))<=new['node'].keys(),i
    for i,r in new['relation'].items():
        for m in r.findall('member'):assert int(m.get('ref')) in new[m.get('type')],(i,dict(m.attrib))
    for keep,group in GROUPS.items():
        assert keep in new['relation'] and all(i not in new['relation'] for i in group if i!=keep)
        assert all(tags(old['relation'][i])==tags(new['relation'][keep]) for i in group)

    ctypes.CDLL(str(ROOT.parent/'planning_ws/install_isolated/lib/libautoware_lanelet2_extension_lib.so'),mode=ctypes.RTLD_GLOBAL)
    import lanelet2
    projector=lanelet2.projection.UtmProjector(lanelet2.io.Origin(37.2380038082,126.7730819386))
    before,after=[lanelet2.io.load(str(f),projector) for f in [BASE,args.map]]
    rules=lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,lanelet2.traffic_rules.Participants.Vehicle)
    bg,ag=[lanelet2.routing.RoutingGraph(m,rules) for m in [before,after]]
    assert len(after.laneletLayer)==len(before.laneletLayer)-len(REMAPPED)==1302
    assert not ag.checkValidity(False),ag.checkValidity(False)
    def normalized(s):
        return {k:sorted({REMAPPED.get(i,i) for i in v}) if isinstance(v,list) else REMAPPED.get(v,v) for k,v in s.items()}
    interface=[]
    for lane in after.laneletLayer:
        if lane.id in EDIT_LANES:continue
        a,b=signature(bg,before.laneletLayer[lane.id]),signature(ag,lane)
        assert normalized(a)==b,(lane.id,a,b)
        if a!=b:interface.append(lane.id)
    invalid=[];outside=[];polygons={};vmb_center_outside=[];duplicates=[];wrong_direction=[]
    for lane in after.laneletLayer:
        left=[(x.x,x.y) for x in lane.leftBound];right=[(x.x,x.y) for x in lane.rightBound]
        shape=Polygon(left+list(reversed(right)));polygons[lane.id]=shape
        if not shape.is_valid or shape.area<=1e-6:invalid.append(lane.id)
        cl=LineString([(x.x,x.y) for x in lane.centerline])
        if lane.id in EDIT_LANES and not shape.buffer(.001).covers(cl):outside.append(lane.id)
        if any(m.get('role')=='centerline' for m in new['relation'][lane.id].findall('member')):
            if any(not shape.buffer(.001).covers(Point(x.x,x.y)) for x in list(lane.centerline)[1:-1]):vmb_center_outside.append(lane.id)
        if math.dist(left[0],right[0])+math.dist(left[-1],right[-1])>math.dist(left[0],right[-1])+math.dist(left[-1],right[0])+1e-6:wrong_direction.append(lane.id)
    for line in after.lineStringLayer:
        for a,b in zip(list(line),list(line)[1:]):
            if a.id==b.id or math.hypot(a.x-b.x,a.y-b.y)<1e-6:duplicates.append(line.id)
    assert not invalid,invalid
    assert not outside,outside
    assert not vmb_center_outside,vmb_center_outside
    assert not duplicates,duplicates
    assert not wrong_direction,wrong_direction

    pairs=[(201456,201457),(201457,201458),(201458,201459),(201459,201463),
           (201468,201467),(201467,201466),(201466,201465),(201465,201464),
           (201446,201441),(201441,201439),(201439,201440)]
    for a,b in pairs:
        assert ag.left(after.laneletLayer[a]) and ag.left(after.laneletLayer[a]).id==b,(a,b)
        assert polygons[a].intersection(polygons[b]).area<1e-6,(a,b)
    for i in [201468,201441,201446]:
        l=after.laneletLayer[i]
        assert not ag.following(l),i
        assert len(l.leftBound)==len(l.rightBound)==2,i
        assert l.leftBound[-1].id==l.rightBound[-1].id,i
        assert len({q.id for q in list(l.leftBound)+list(l.rightBound)})==3,i
    for a,b in [(201456,201468),(201457,201467),(201458,201466),(201459,201465),(201463,201464)]:
        assert [x.id for x in ag.following(after.laneletLayer[a])]==[b],(a,b)
    assert set(x.id for x in ag.following(after.laneletLayer[201439]))=={200185,200360}
    assert [x.id for x in ag.following(after.laneletLayer[201440])]==[200147]
    paths=[]
    for a,b in [(201456,200128),(201468,200128),(201446,200185),(201441,200185)]:
        path=ag.shortestPath(after.laneletLayer[a],after.laneletLayer[b]);assert path,(a,b)
        ids=[x.id for x in path];assert any(q not in signature(ag,after.laneletLayer[p])['following'] for p,q in zip(ids,ids[1:])),ids
        visited=set();todo=[a]
        while todo:
            i=todo.pop()
            if i in visited:continue
            visited.add(i);todo.extend(x.id for x in ag.following(after.laneletLayer[i]))
        assert b not in visited,(a,b)
        paths.append({'from':a,'to':b,'path':ids,'successor_only_reachable':False})
    # Independent collinearity check across the retained entry LC split.
    straight=[]
    for ids in [[105497,105550],[105500,105546]]:
        lines=[after.lineStringLayer[i] for i in ids]
        a,b=lines[0][0],lines[-1][-1];chord=LineString([(a.x,a.y),(b.x,b.y)])
        error=max(chord.distance(Point(q.x,q.y)) for l in lines for q in l)
        assert error<1e-6,(ids,error)
        straight.append({'ways':ids,'max_joint_deviation_m':error})

    # GIS evidence for the actual straight separators, sampled every metre.
    f=Transformer.from_crs(4326,32652,always_xy=True);ox,oy=f.transform(126.7730819386,37.2380038082)
    gis={}
    for sr in shapefile.Reader(str(ROOT/'gis_map/B2_SURFACELINEMARK'),encoding='utf-8').iterShapeRecords():
        d=sr.record.as_dict()
        if d['ID'] in ['B2256W000038','B2256W000043']:
            gis[d['ID']]=LineString([(p[0]-ox,p[1]-oy) for p in sr.shape.points])
    deviations=[]
    for ids,gid in [([105497,105550],'B2256W000038'),([105520],'B2256W000043')]:
        points=[]
        for i in ids:points.extend((p.x,p.y) for p in after.lineStringLayer[i])
        line=LineString(points);distances=[gis[gid].distance(line.interpolate(line.length*j/max(1,math.ceil(line.length)))) for j in range(math.ceil(line.length)+1)]
        assert max(distances)<.4,(gid,max(distances))
        deviations.append({'ways':ids,'gis':gid,'max_distance_to_raw_gis_m':max(distances)})

    profile_result=None
    if args.profiles:
        conf=ROOT.parent/'base_ws/src/planning_addons/competition_route_requester/config'
        route=yaml.safe_load((conf/'kcity_v87_competition_route.yaml').read_text())
        assert route['map']['version']==87 and route['map']['sha256']==sha256(args.map)
        assert (ROOT.parent/route['map']['file']).resolve()==args.map.resolve()
        cfg=yaml.safe_load((conf/'competition_route_requester_v87.param.yaml').read_text())
        sys.path.insert(0,str(ROOT.parent/'base_ws/src/planning_addons/competition_route_requester/src'))
        from competition_route_requester import policy
        from competition_route_requester.geometry import Geometry
        policy.validate(cfg,route);geometry=Geometry(str(ROOT.parent),route)
        for name in ['route_a','route_b','fallback_route']:
            ids=route[name]['expected_path_lanelet_ids']
            for a,b in zip(ids,ids[1:]):assert ag.shortestPath(after.laneletLayer[a],after.laneletLayer[b]),(name,a,b)
        for name,p in route['poses'].items():
            assert after.laneletLayer.exists(p['lanelet_id']),name
            assert polygons[p['lanelet_id']].buffer(.001).covers(Point(p['x'],p['y'])),name
        mission=yaml.safe_load((ROOT.parent/'base_ws/src/planning_addons/ajounice_planning_integration/config/mission_planner_kcity_v87.yaml').read_text())
        for key,ids in mission['route_section_primitive_overrides'].items():
            assert after.laneletLayer.exists(int(key)),key
            assert all(after.laneletLayer.exists(i) for i in ids),(key,ids)
        assert all(after.laneletLayer.exists(i) for i in mission['route_section_excluded_lanelet_ids'])
        profile_result={'route_policy_validation':'PASS','geometry_constructor':'PASS','preferred_paths_reachable':True,
                        'waypoints_inside_lanelets':True,'mission_ids_exist':True,'speed_policy':'OSM original attributes; no override files'}
    result={'status':'PASS','map':str(args.map),'sha256':sha256(args.map),'baseline_sha256':sha256(BASE),'map_version':87,
            'lanelets':len(after.laneletLayer),'primitive_diff':diff,'outside_region_primitives_unchanged':True,
            'outside_graph_changes_only_remapped_ids':interface,'invalid_polygons':invalid,'local_centerlines_outside':outside,
            'explicit_centerline_interior_outside':vmb_center_outside,'duplicates':duplicates,'wrong_bound_directions':wrong_direction,
            'lc_pairs':pairs,'required_lc_paths':paths,'straightness':straight,'gis_deviation':deviations,'profiles':profile_result,
            'entry_main_split_required_for_lc':True,'live_driving':'not run'}
    args.report.parent.mkdir(parents=True,exist_ok=True);args.report.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='primitive_diff'},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
