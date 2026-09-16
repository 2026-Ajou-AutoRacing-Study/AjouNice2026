#!/usr/bin/env python3
"""Validate map preservation scope, geometry, retained LC/merge topology and route IDs."""
from pathlib import Path
import hashlib,json
from collections import Counter
from lxml import etree as E
import lanelet2 as l
import yaml
from shapely.geometry import Polygon
from build_v74_drivable_lanes import ROOT,ROUTE_EXCLUDED,SOLID

def main():
 p=ROOT/'lanelet_map/lanelet2_map_v74.osm';old=ROOT/'lanelet_map/lanelet2_map_v73.osm'
 a=E.parse(str(p),E.XMLParser(huge_tree=True)).getroot();b=E.parse(str(old),E.XMLParser(huge_tree=True)).getroot()
 idx=lambda root,kind:{int(e.get('id')):e for e in root.findall(kind)}
 canonical=lambda e:E.tostring(e,method='c14n').strip()
 nodes=idx(a,'node');ways=idx(a,'way');rels=idx(a,'relation')
 for kind in ['node','way','relation']:
  x=idx(a,kind);y=idx(b,kind)
  assert set(x)==set(y)
  assert not set(x)-set(y)
  changes={i for i in x if canonical(x[i])!=canonical(y[i])}
  assert changes==(SOLID if kind=='way' else set()),(kind,changes)
 assert a.find('MetaInfo').get('map_version')=='74'
 for way in ways.values():
  refs=[int(x.get('ref')) for x in way.findall('nd')];assert set(refs)<=nodes.keys()
  for x,y in zip(refs,refs[1:]):
   assert x!=y
   assert (nodes[x].get('lat'),nodes[x].get('lon'))!=(nodes[y].get('lat'),nodes[y].get('lon'))
 for rel in rels.values():
  for member in rel.findall('member'):
   assert int(member.get('ref')) in {'node':nodes,'way':ways,'relation':rels}[member.get('type')]
 m=l.io.load(str(p),l.projection.UtmProjector(l.io.Origin(37.2380038082,126.7730819386)))
 rules=l.traffic_rules.create(l.traffic_rules.Locations.Germany,l.traffic_rules.Participants.Vehicle)
 g=l.routing.RoutingGraph(m,rules);lanes={x.id:x for x in m.laneletLayer}
 assert ROUTE_EXCLUDED <= lanes.keys()
 for x in lanes.values():
  poly=Polygon([(p.x,p.y) for p in x.leftBound]+[(p.x,p.y) for p in reversed(x.rightBound)])
  assert poly.is_valid and poly.area>0,(x.id,'invalid polygon')
 pairs=[(201459,201458),(201458,201457),(201457,201456),(201436,201435),(201435,200099),(200138,200137),(200137,200128),(201444,200187),(200187,200102),(200127,200130),(200130,200134),(200147,200185),(201448,201447)]
 for x,y in pairs:
  assert g.right(lanes[x]).id==y and g.left(lanes[y]).id==x,(x,y)
 for x,y in [(200186,200138),(200141,201444),(200148,200127),(201400,201399)]:
  assert g.right(lanes[x]) is None and g.left(lanes[y]) is None,(x,y,'solid boundary LC')
 for chain in [(201456,201438,200128),(200127,201439,200185),(200130,201441,200360),(200134,201446,200360)]:
  for x,y in zip(chain,chain[1:]):assert y in [z.id for z in g.following(lanes[x])],(x,y)
 r=yaml.safe_load((ROOT.parent/'base_ws/src/planning_addons/competition_route_requester/config/kcity_v74_competition_route.yaml').read_text())
 sha=hashlib.sha256(p.read_bytes()).hexdigest();assert r['map']['sha256']==sha and r['map']['version']==74
 for key in ['route_a','route_b','fallback_route']:
  assert set(r[key]['expected_path_lanelet_ids'])<=lanes.keys()
 report={'sha256':sha,'lanelets':len(lanes),'removed_ids':[],'preserved_route_excluded_ids':sorted(ROUTE_EXCLUDED),'invalid_polygons':0,'missing_references':0,'adjacent_duplicate_coordinates':0,'reciprocal_lc_checks':2*len(pairs),'merge_chain_checks':4,'speed_distribution':dict(Counter(x.attributes['speed_limit'] for x in lanes.values() if 'subtype' in x.attributes and x.attributes['subtype']=='road'))}
 (ROOT/'docs/kcity_v74_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
