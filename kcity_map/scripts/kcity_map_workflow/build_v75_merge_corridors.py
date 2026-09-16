#!/usr/bin/env python3
"""Rebuild independent overlapping merge corridors from MGeo, preserving IDs and joints."""
from pathlib import Path
import json,hashlib,math
import shapefile
import numpy as np
from lxml import etree as E
from pyproj import Transformer
from shapely.geometry import LineString,Point,Polygon
ROOT=Path(__file__).resolve().parents[2]
OBSOLETE_WAYS={101991,104170,104166,104200,104182,104213,104211,104193,104214,104212}
PAIRS=[(200127,201439,'A2256W000423'),(200130,201441,'A2256W000422'),(200134,201446,'A2256W000445')]
def main():
 dst=ROOT/'lanelet_map/lanelet2_map_v75.osm'
 if dst.exists():raise RuntimeError('v75 already exists')
 tree=E.parse(str(ROOT/'lanelet_map/lanelet2_map_v74.osm'),E.XMLParser(huge_tree=True));root=tree.getroot()
 nodes={int(x.get('id')):x for x in root.findall('node')};ways={int(x.get('id')):x for x in root.findall('way')};rels={int(x.get('id')):x for x in root.findall('relation')}
 forward=Transformer.from_crs(4326,32652,always_xy=True);inverse=Transformer.from_crs(32652,4326,always_xy=True)
 def xyz(i):
  e=nodes[i];x,y=forward.transform(float(e.get('lon')),float(e.get('lat')));t={a.get('k'):a.get('v') for a in e.findall('tag')};return np.array([x,y,float(t.get('ele','0'))])
 def refs(i,role):
  m=next(x for x in rels[i].findall('member') if x.get('role')==role);return [int(x.get('ref')) for x in ways[int(m.get('ref'))].findall('nd')]
 ni=max(nodes)+1;wi=max(ways)+1;new_nodes=[];new_ways=[]
 def addnode(p):
  nonlocal ni
  lon,lat=inverse.transform(p[0],p[1]);e=E.Element('node',id=str(ni),lat=f'{lat:.14f}',lon=f'{lon:.14f}',version='1',visible='true');E.SubElement(e,'tag',k='ele',v=f'{p[2]:.9f}');new_nodes.append(e);nodes[ni]=e;ni+=1;return ni-1
 def addway(ids,role):
  nonlocal wi
  e=E.Element('way',id=str(wi),version='1',visible='true')
  for i in ids:E.SubElement(e,'nd',ref=str(i))
  if role!='centerline':E.SubElement(e,'tag',k='type',v='virtual')
  new_ways.append(e);ways[wi]=e;wi+=1;return wi-1
 links={x['idx']:x for x in json.load(open(ROOT/'mgeo_map/link_set.json'))};origin=np.array(json.load(open(ROOT/'mgeo_map/global_info.json'))['local_origin_in_global'])
 marks={x.record.as_dict()['ID']:list(x.shape.points) for x in shapefile.Reader(str(ROOT/'gis_map/B2_SURFACELINEMARK'),encoding='utf-8').iterShapeRecords()}
 road=Polygon(marks['B2256W000042']+(marks['B2256W000025']+marks['B2256W001693'][1:])[::-1])
 report=[]
 for first,second,key in PAIRS:
  source=np.array(links[key]['points'])+origin;dist=np.r_[0,np.cumsum(np.linalg.norm(np.diff(source[:,:2],axis=0),axis=1))];line=LineString(source[:,:2])
  cut=line.project(Point(xyz(refs(first,'centerline')[-1])[:2]));samples=np.unique(np.r_[np.arange(0,line.length,0.5),cut,line.length]);idx=int(np.where(samples==cut)[0][0]);center=np.column_stack([np.interp(samples,dist,source[:,j]) for j in range(3)])
  # Blend terminal tangent into the common competition exit over the last 10 m.
  out=refs(200360,'centerline');target=xyz(out[1])[:2]-xyz(out[0])[:2];target/=np.linalg.norm(target)
  initial=(center[-1,:2]-center[-2,:2])/(samples[-1]-samples[-2]);remaining=line.length-samples;u=np.clip(remaining/10.,0,1)
  center[:,:2]+=(-remaining*(1-u)**2)[:,None]*(target-initial)
  delta=np.gradient(center[:,:2],axis=0);normal=np.column_stack([-delta[:,1],delta[:,0]]);normal/=np.linalg.norm(normal,axis=1)[:,None]
  # Source width metadata is 3.5 m. Smoothly reconcile endpoint offsets to exact map joints.
  for role,sign in [('left',1),('right',-1),('centerline',0)]:
   start=refs(first,role)[0];end=refs(second,role)[-1]
   coords=center.copy();coords[:,:2]+=sign*1.75*normal
   for s,endpoint,at_start in [(samples,start,True),(line.length-samples,end,False)]:
    u=np.clip(s/10.,0,1);weight=1-3*u*u+2*u*u*u;error=xyz(endpoint)-coords[0 if at_start else -1];coords+=weight[:,None]*error
   if key=='A2256W000445' and role=='right':
    # The GIS outer edge is authoritative; overlap with other lanes is allowed.
    for j in range(1,len(coords)-1):
     if not road.covers(Point(coords[j,:2])):
      ray=LineString([center[j,:2],center[j,:2]-normal[j]*10]);hit=ray.intersection(road.boundary)
      candidates=list(hit.geoms) if hasattr(hit,'geoms') else [hit]
      candidates=[h for h in candidates if h.geom_type=='Point']
      if candidates:coords[j,:2]=np.array(min(candidates,key=lambda h:h.distance(Point(center[j,:2]))).coords[0])
   ids=[start]+[addnode(p) for p in coords[1:-1]]+[end]
   for lane,subset in [(first,ids[:idx+1]),(second,ids[idx:])]:
    m=next(x for x in rels[lane].findall('member') if x.get('role')==role);m.set('ref',str(addway(subset,role)))
  report.append({'lanelets':[first,second],'mgeo':key,'split_s':cut,'source_length':line.length})
 used={int(m.get('ref')) for a in root.findall('relation') for m in a.findall('member') if m.get('type')=='way'}
 assert not OBSOLETE_WAYS & used
 for i in OBSOLETE_WAYS:root.remove(ways[i])
 for e in new_nodes+new_ways:root.append(e)
 root[:]=sorted(root,key=lambda e:{'MetaInfo':0,'node':1,'way':2,'relation':3}.get(e.tag,0))
 root.find('MetaInfo').set('map_version','75');E.indent(tree,space='  ');tree.write(str(dst),encoding='UTF-8',xml_declaration=True)
 result={'sha256':hashlib.sha256(dst.read_bytes()).hexdigest(),'corridors':report,'lanelets_deleted':[],'obsolete_ways_removed':sorted(OBSOLETE_WAYS),'nominal_width_m':3.5}
 (ROOT/'docs/kcity_v75_merge_corridors.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
