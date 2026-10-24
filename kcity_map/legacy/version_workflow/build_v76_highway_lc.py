#!/usr/bin/env python3
"""Restore passable shared boundaries only where independent vehicle-width lanes exist.

Work in an unpublished output file first. Preserve v75 centerlines, speed tags and
wide overlapping corridors beyond the cross-section where a lane actually merges.
"""
import argparse, copy, hashlib, json
from pathlib import Path
import numpy as np
from lxml import etree as E
from pyproj import Transformer

ROOT=Path(__file__).resolve().parents[2]

def build(output):
 if output.exists():raise RuntimeError(f'Output already exists: {output}')
 tree=E.parse(str(ROOT/'lanelet_map/lanelet2_map_v75.osm'),E.XMLParser(huge_tree=True));root=tree.getroot()
 nodes={int(e.get('id')):e for e in root.findall('node')};ways={int(e.get('id')):e for e in root.findall('way')};rels={int(e.get('id')):e for e in root.findall('relation')}
 tx=Transformer.from_crs(4326,32652,always_xy=True);inv=Transformer.from_crs(32652,4326,always_xy=True);ox,oy=tx.transform(126.7730819386,37.2380038082)
 def tags(e):return {t.get('k'):t.get('v') for t in e.findall('tag')}
 def settag(e,k,v):
  t=next((t for t in e.findall('tag') if t.get('k')==k),None)
  if t is None:t=E.SubElement(e,'tag',k=k)
  t.set('v',str(v))
 def xyz(i):
  n=nodes[i];x,y=tx.transform(float(n.get('lon')),float(n.get('lat')));return np.array([x-ox,y-oy,float(tags(n).get('ele',0))])
 def put(i,p):
  lon,lat=inv.transform(p[0]+ox,p[1]+oy);nodes[i].set('lon',f'{lon:.14f}');nodes[i].set('lat',f'{lat:.14f}');settag(nodes[i],'ele',f'{p[2]:.9f}')
 ni=max(nodes)+1;wi=max(ways)+1;ri=max(rels)+1
 def node(p):
  nonlocal ni
  i=ni;ni+=1;nodes[i]=E.SubElement(root,'node',id=str(i),version='1',visible='true');put(i,p);return i
 def way(refs,template=None):
  nonlocal wi
  i=wi;wi+=1;e=E.SubElement(root,'way',id=str(i),version='1',visible='true');ways[i]=e
  for n in refs:E.SubElement(e,'nd',ref=str(n))
  if template is not None:
   for t in template.findall('tag'):e.append(copy.deepcopy(t))
  return i
 def mem(i,role):return next(m for m in rels[i].findall('member') if m.get('role')==role)
 def wid(i,role):return int(mem(i,role).get('ref'))
 def refs(w):return [int(n.get('ref')) for n in ways[w].findall('nd')]
 def points(w):return np.array([xyz(i) for i in refs(w)])
 splitcache={};splits={};segments={i:[i] for i in rels}
 def splitway(w,y):
  key=(w,y)
  if key in splitcache:return splitcache[key]
  ns=refs(w);ps=points(w)
  for j,(a,b) in enumerate(zip(ps,ps[1:])):
   if a[1]>=y>=b[1] and a[1]>b[1]:
    ratio=(a[1]-y)/(a[1]-b[1]);n=ns[j] if ratio<1e-7 else ns[j+1] if ratio>1-1e-7 else node(a+ratio*(b-a))
    up=ns[:j+1]+([] if n==ns[j] else [n]);down=([n] if n!=ns[j+1] else [])+ns[j+1:]
    pair=(way(up,ways[w]),way(down,ways[w]));splitcache[key]=pair;return pair
  raise ValueError(('no cross section',w,y,ps[[0,-1],1]))
 def split(i,y):
  nonlocal ri
  j=ri;ri+=1;rels[j]=copy.deepcopy(rels[i]);rels[j].set('id',str(j));root.append(rels[j])
  for role in ['left','right','centerline']:
   up,down=splitway(wid(i,role),y);mem(i,role).set('ref',str(up));mem(j,role).set('ref',str(down))
  splits[str(i)]={'downstream':j,'cut_map_y':y};return j
 # Align transverse slices; lateral graph edges require identical whole ways.
 early={i:split(i,686.5) for i in [201437,201436,201435,200099,201438]}
 mid={i:split(i,447.9) for i in [201444,200187,200102]}
 merge={i:split(i,390.5) for i in [200148,200127,200130,200134]}
 late={i:split(i,300.0) for i in [201453,200360]}
 pairs=[(201463,201459),(201437,201436),(201436,201435),(201435,200099),(200099,201438),
        (early[201437],early[201436]),(early[201436],early[201435]),(early[201435],early[200099]),
        (200186,200138),(200138,200137),(200137,200128),
        (200141,201444),(201444,200187),(200187,200102),
        (201445,mid[201444]),(mid[201444],mid[200187]),(mid[200187],mid[200102]),
        (200148,200127),(200127,200130),(200130,200134),
        (merge[200148],merge[200127]),(merge[200127],merge[200130]),
        (201440,201439),(200147,200185),(late[201453],late[200360]),
        (201448,201447),(201400,201399)]
 # Reuse untouched shared boundaries. Build averaged boundaries only for missing
 # or misaligned edges. Union endpoint IDs and blend their displacement into
 # adjoining corridors, so no sharp corner is introduced at an LC/merge seam.
 parent={};moves={};canonical=set();lc=[]
 def find(n):
  while n in parent:n=parent[n]
  return n
 def union(a,b,p):
  a,b=find(a),find(b)
  if a!=b:parent[b]=a
  moves[a]=p;return a
 # Preserve original coordinates for blending before any endpoint changes.
 original={i:xyz(i) for i in nodes}
 for left,right in pairs:
  wa,wb=wid(left,'right'),wid(right,'left')
  if wa==wb:
   settag(ways[wa],'type','line_thin');settag(ways[wa],'subtype','dashed');lc.append([left,right]);continue
  pa,pb=points(wa),points(wb)
  # Trim the malformed shared v75 way to the outside centerline's proper span.
  hi=(pa[0,1]+pb[0,1])/2;lo=(pa[-1,1]+pb[-1,1])/2 if abs(pa[-1,1]-pb[-1,1])<1 else max(pa[-1,1],pb[-1,1])
  ys=np.linspace(hi,lo,max(3,int((hi-lo)/.5)+1))
  def interp(p):return np.column_stack([np.interp(ys,p[::-1,1],p[::-1,j]) for j in range(3)])
  pc=(interp(pa)+interp(pb))/2
  # Endpoint references must correspond to this lane's actual end, not a bound
  # extending across its successor. The outside lane provides the seam node.
  ends=[]
  for k in [0,-1]:
   na=refs(wa)[k];nb=refs(wb)[k]
   if abs(pa[k,1]-pc[k,1])>1:
    na=node(pc[k]);original[na]=pc[k].copy()
   if abs(pb[k,1]-pc[k,1])>1:
    nb=node(pc[k]);original[nb]=pc[k].copy()
   ends.append(union(na,nb,pc[k]))
  nw=way([ends[0]]+[node(p) for p in pc[1:-1]]+[ends[1]])
  settag(ways[nw],'type','line_thin');settag(ways[nw],'subtype','dashed');canonical.add(nw)
  mem(left,'right').set('ref',str(nw));mem(right,'left').set('ref',str(nw));lc.append([left,right])
 # Reconnect the original split outside lane using the new shared seam.
 seam=refs(wid(200141,'right'))[-1]
 old=refs(wid(201445,'right'))[0]
 if find(old)!=find(seam):parent[find(old)]=find(seam)
 # A split outside centerline has a tiny original mismatch: retain its physical
 # point and use its IDs on the neighboring split boundaries via the unions.
 active={int(m.get('ref')) for e in rels.values() if tags(e).get('type')=='lanelet' for m in e.findall('member') if m.get('role') in ['left','right','centerline']}
 for w in active-canonical:
  ns=refs(w);ps=points(w);disp=[]
  for k in [0,-1]:
   n=find(ns[k]);target=moves.get(n,original.get(n,xyz(n)));disp.append(target-ps[k])
  if max(np.linalg.norm(x) for x in disp)>1e-7:
   lengths=np.r_[0,np.cumsum(np.linalg.norm(np.diff(ps[:,:2],axis=0),axis=1))]
   for distances,delta in [(lengths,disp[0]),(lengths[-1]-lengths,disp[1])]:
    u=np.clip(distances/10,0,1);ps+=(1-3*u*u+2*u*u*u)[:,None]*delta
   new=[find(ns[0])]+[node(p) for p in ps[1:-1]]+[find(ns[-1])]
   for nd in list(ways[w].findall('nd')):ways[w].remove(nd)
   for n in reversed(new):ways[w].insert(0,E.Element('nd',ref=str(n)))
 for w in active:
  for nd in ways[w].findall('nd'):nd.set('ref',str(find(int(nd.get('ref')))))
 for n,p in moves.items():
  if find(n)==n:put(n,p)
 # Endpoint identity for the malformed outside longitudinal joint.
 for role in ['left','right','centerline']:
  end=refs(wid(200141,role))[-1];start=ways[wid(201445,role)].find('nd');start.set('ref',str(end))
 # Fit each newly shared boundary smoothly with fixed endpoints. Source ways
 # contain different sampling and short endpoint blends; averaging their raw
 # samples otherwise creates kinks even though both corridors are smooth.
 for w in canonical:
  ns=refs(w);ps=points(w);u=np.linspace(0,1,len(ns));base=(1-u)[:,None]*ps[0]+u[:,None]*ps[-1]
  basis=np.column_stack([u*(1-u),u*u*(1-u)])
  fitted=base+basis@np.linalg.lstsq(basis,ps-base,rcond=None)[0]
  for n,p in zip(ns[1:-1],fitted[1:-1]):put(n,p)
 root.find('MetaInfo').set('map_version','76');root[:]=sorted(root,key=lambda e:{'MetaInfo':0,'node':1,'way':2,'relation':3}.get(e.tag,0));E.indent(tree,space='  ');tree.write(str(output),encoding='UTF-8',xml_declaration=True)
 report={'input_sha256':hashlib.sha256((ROOT/'lanelet_map/lanelet2_map_v75.osm').read_bytes()).hexdigest(),'output_sha256':hashlib.sha256(output.read_bytes()).hexdigest(),'splits':splits,'bidirectional_pairs':lc,'merge_only_pairs':[[early[200099],early[201438]],[merge[200130],merge[200134]],[201439,201441],[201441,201446],[200185,200360]],'separated_road_edges':[[201449,201448],[201452,201451]]}
 output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);build(ap.parse_args().output)
