#!/usr/bin/env python3
"""Audit v75 lane preservation, source alignment, smooth widths, overlap and graph joints."""
from pathlib import Path
import json,math,hashlib,itertools
from lxml import etree as E
import numpy as np,lanelet2 as l,shapefile
from shapely.geometry import Polygon,LineString,Point
from pyproj import Transformer
from build_v75_merge_corridors import ROOT,PAIRS,OBSOLETE_WAYS

def main():
 paths=[ROOT/f'lanelet_map/lanelet2_map_v{v}.osm' for v in [74,75]];roots=[E.parse(str(p),E.XMLParser(huge_tree=True)).getroot() for p in paths]
 idx=lambda r,t:{int(x.get('id')):x for x in r.findall(t)}
 def sem(e):return (e.tag,tuple(sorted(e.attrib.items())),tuple(sem(x) for x in e))
 touched={i for a,b,_ in PAIRS for i in [a,b]}
 for typ in ['node','way','relation']:
  old,new=[idx(r,typ) for r in roots];assert set(old)-set(new)==(OBSOLETE_WAYS if typ=='way' else set())
  changes={i for i in set(old)&set(new) if sem(old[i])!=sem(new[i])}
  assert changes==(touched if typ=='relation' else set()),(typ,changes)
  if typ=='relation':assert set(old)==set(new)
 nodes=idx(roots[1],'node');ways=idx(roots[1],'way');relations=idx(roots[1],'relation')
 for way in ways.values():
  refs=[int(x.get('ref')) for x in way.findall('nd')];assert set(refs)<=nodes.keys()
  for a,b in zip(refs,refs[1:]):assert a!=b and (nodes[a].get('lat'),nodes[a].get('lon'))!=(nodes[b].get('lat'),nodes[b].get('lon'))
 for rel in relations.values():
  for x in rel.findall('member'):assert int(x.get('ref')) in {'node':nodes,'way':ways,'relation':relations}[x.get('type')]
 projector=l.projection.UtmProjector(l.io.Origin(37.2380038082,126.7730819386));m=l.io.load(str(paths[1]),projector);g=l.routing.RoutingGraph(m,l.traffic_rules.create(l.traffic_rules.Locations.Germany,l.traffic_rules.Participants.Vehicle))
 polygons={}
 for lane in m.laneletLayer:
  p=Polygon([(x.x,x.y) for x in lane.leftBound]+[(x.x,x.y) for x in reversed(lane.rightBound)]);assert p.is_valid and p.area>0,lane.id;polygons[lane.id]=p
 transformer=Transformer.from_crs(4326,32652,always_xy=True);origin=transformer.transform(126.7730819386,37.2380038082)
 links={x['idx']:x for x in json.load(open(ROOT/'mgeo_map/link_set.json'))};mgorigin=json.load(open(ROOT/'mgeo_map/global_info.json'))['local_origin_in_global'];rows=[]
 gis={x.record.as_dict()['ID']:x for x in shapefile.Reader(str(ROOT/'gis_map/A2_LINK'),encoding='utf-8').iterShapeRecords()}
 for first,second,key in PAIRS:
  source=LineString([(p[0]+mgorigin[0]-origin[0],p[1]+mgorigin[1]-origin[1]) for p in links[key]['points']]);gl=LineString([(p[0]-origin[0],p[1]-origin[1]) for p in gis[key].shape.points]);assert source.hausdorff_distance(gl)<.02
  assert second in [x.id for x in g.following(m.laneletLayer[first])]
  assert 200360 in [x.id for x in g.following(m.laneletLayer[second])]
  for i in [first,second]:
   lane=m.laneletLayer[i];left=np.array([(p.x,p.y) for p in lane.leftBound]);right=np.array([(p.x,p.y) for p in lane.rightBound]);center=np.array([(p.x,p.y) for p in lane.centerline]);width=np.linalg.norm(left-right,axis=1);assert width.min()>3.2 and width.max()<3.6
   cl=LineString(center);assert cl.difference(polygons[i].buffer(.001)).length<.001
   error=max(source.distance(Point(p)) for p in center);assert error<.35,(i,error)
   angles=[]
   for points in [left,right,center]:
    v=np.diff(points,axis=0);h=np.unwrap(np.arctan2(v[:,1],v[:,0]));angles.append(float(np.degrees(abs(np.diff(h))).max()))
   assert max(angles)<5,(i,angles)
   rows.append({'lanelet':i,'width_min':float(width.min()),'width_max':float(width.max()),'max_heading_step_deg':max(angles),'mgeo_max_center_error_m':error})
  # No geometric jump at the internal split; exact shared node references.
  for role in ['leftBound','rightBound','centerline']:
   assert getattr(m.laneletLayer[first],role)[-1].id==getattr(m.laneletLayer[second],role)[0].id
 report={'sha256':hashlib.sha256(paths[1].read_bytes()).hexdigest(),'lanelets':len(m.laneletLayer),'deleted_lanelets':[],'invalid_polygons':0,'missing_references':0,'corridors':rows,'permitted_overlaps_m2':{f'{a}/{b}':polygons[a].intersection(polygons[b]).area for a,b in itertools.combinations([201439,201441,201446],2)}}
 (ROOT/'docs/kcity_v75_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
