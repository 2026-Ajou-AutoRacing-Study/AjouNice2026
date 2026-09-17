import lanelet2 as l,json,sys
from shapely.geometry import Polygon,LineString,Point
import numpy as np
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
p=str(ROOT/'lanelet_map/lanelet2_map_v76.osm');r=json.load(open(ROOT/'docs/kcity_v76_highway_lc_build.json'));pr=l.projection.UtmProjector(l.io.Origin(37.2380038082,126.7730819386));m=l.io.load(p,pr);g=l.routing.RoutingGraph(m,l.traffic_rules.create(l.traffic_rules.Locations.Germany,l.traffic_rules.Participants.Vehicle));old=l.io.load(str(ROOT/'lanelet_map/lanelet2_map_v75.osm'),pr);og=l.routing.RoutingGraph(old,l.traffic_rules.create(l.traffic_rules.Locations.Germany,l.traffic_rules.Participants.Vehicle))
bad=[]
for a,b in r['bidirectional_pairs']:
 if not g.right(m.laneletLayer[a]) or g.right(m.laneletLayer[a]).id!=b or not g.left(m.laneletLayer[b]) or g.left(m.laneletLayer[b]).id!=a:bad.append(('LC',a,b))
for a in m.laneletLayer:
 poly=Polygon([(p.x,p.y) for p in a.leftBound]+[(p.x,p.y) for p in reversed(a.rightBound)])
 if not poly.is_valid:bad.append(('POLY',a.id))
 if dict(a.attributes).get('speed_limit')=='150':
  c=LineString([(p.x,p.y) for p in a.centerline]);outside=c.difference(poly.buffer(.01)).length
  if outside>.05:bad.append(('CENTER',a.id,round(outside,2)))
  L=LineString([(p.x,p.y) for p in a.leftBound]);R=LineString([(p.x,p.y) for p in a.rightBound]);width=min(L.interpolate(t,normalized=True).distance(R) for t in np.linspace(0,1,50))
  if width<2.5:bad.append(('WIDTH',a.id,round(width,2)))
for a in old.laneletLayer:
 after=r['splits'].get(str(a.id),{}).get('downstream',a.id)
 for b in og.following(a):
  if b.id not in [x.id for x in g.following(m.laneletLayer[after])]:bad.append(('LOST_NEXT',a.id,after,b.id))
for a,d in r['splits'].items():
 if d['downstream'] not in [x.id for x in g.following(m.laneletLayer[int(a)])]:bad.append(('SPLIT_NEXT',a,d))
# Speed limits and original relation IDs must be preserved.
for a in old.laneletLayer:
 assert dict(a.attributes)==dict(m.laneletLayer[a.id].attributes),a.id
for a,d in r['splits'].items():
 assert dict(m.laneletLayer[int(a)].attributes)==dict(m.laneletLayer[d['downstream']].attributes)
# Every active way and relation must reference existing primitives; no adjacent duplicates.
import xml.etree.ElementTree as E,hashlib
root=E.parse(p).getroot();ids={k:{int(e.get('id')) for e in root.findall(k)} for k in ['node','way','relation']}
for w in root.findall('way'):
 ns=[int(x.get('ref')) for x in w.findall('nd')];assert set(ns)<=ids['node']
 assert all(a!=b for a,b in zip(ns,ns[1:])),w.get('id')
for e in root.findall('relation'):
 for x in e.findall('member'):assert int(x.get('ref')) in ids[x.get('type')]
coverage=[]
for a in sorted(m.laneletLayer,key=lambda a:a.id):
 if dict(a.attributes).get('speed_limit')!='150':continue
 for role in ['leftBound','rightBound','centerline']:
  ps=np.array([(x.x,x.y) for x in getattr(a,role)]);v=np.diff(ps,axis=0);angle=np.degrees(abs(np.diff(np.unwrap(np.arctan2(v[:,1],v[:,0])))))
  if len(angle) and max(angle)>10:bad.append(('ANGLE',a.id,role,float(max(angle))))
 coverage.append({'lanelet':a.id,'left':g.left(a).id if g.left(a) else None,'right':g.right(a).id if g.right(a) else None,'following':[b.id for b in g.following(a)]})
report={'sha256':hashlib.sha256(Path(p).read_bytes()).hexdigest(),'lanelets':len(m.laneletLayer),'bidirectional_pairs':len(r['bidirectional_pairs']),'failures':bad,'highway_coverage':coverage}
(ROOT/'docs/kcity_v76_highway_lc_validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='highway_coverage'},indent=2));assert not bad,bad
