#!/usr/bin/env python3
"""Plot actual v68 ring and immediate entry/exit lanelets for review."""
from pathlib import Path
from lxml import etree as E
from pyproj import Transformer
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from build_v68_roundabout import ROOT, ROLES

root = E.parse(str(ROOT/'lanelet_map/lanelet2_map_v68.osm'))
project = Transformer.from_crs(4326,32652,always_xy=True)
nodes = {n.get('id'): np.array(project.transform(float(n.get('lon')),float(n.get('lat')))) for n in root.findall('node')}
ways = {w.get('id'):np.array([nodes[n.get('ref')] for n in w.findall('nd')]) for w in root.findall('way')}
rels = {r.get('id'):r for r in root.findall('relation')}
def line(i,role):
    return ways[rels[i].find(f"member[@role='{role}']").get('ref')]
origin = np.mean(np.concatenate([line(i,'centerline') for i in ROLES['internal']]),axis=0)
fig,ax = plt.subplots(figsize=(10,9))
for i,r in rels.items():
    if r.find("member[@role='centerline']") is None:
        continue
    p = line(i,'centerline')-origin
    if np.min(np.linalg.norm(p,axis=1))<50:
        ax.plot(*p.T,color='#cccccc',lw=1,zorder=0)
colors={'entry':'#138a4b','internal':'#2878c8','exit':'#df7e17'}
for role,ids in ROLES.items():
    for index,i in enumerate(ids):
        l,r = line(i,'left')-origin,line(i,'right')-origin
        ax.add_patch(Polygon(np.concatenate([l,r[::-1]]),facecolor=colors[role],alpha=.28,edgecolor=colors[role]))
        p = line(i,'centerline')-origin
        ax.plot(*p.T,color=colors[role],lw=2,label=role if index==0 else None)
        j=len(p)//2
        ax.annotate('',xy=p[j+1],xytext=p[max(0,j-2)],arrowprops=dict(arrowstyle='->',color=colors[role],lw=2))
        pos=p[j]; radial=pos/(np.linalg.norm(pos)+1e-9)
        ax.annotate(i,xy=pos,xytext=pos+radial*5,fontsize=10,ha='center',bbox=dict(facecolor='white',alpha=.85,edgecolor='none'),arrowprops=dict(arrowstyle='-',color=colors[role]))
ax.set(xlim=(-45,45),ylim=(-45,45),xlabel='East [m]',ylabel='North [m]',title='K-City v68 | Roundabout 201455\n4 entry + 4 internal + 4 exit lanelets')
ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(loc='upper right');fig.tight_layout()
destination=ROOT/'docs/assets/kcity_v68_roundabout.png'
fig.savefig(destination,dpi=170)
fig.savefig(destination.with_suffix('.svg'))
svg = destination.with_suffix('.svg')
svg.write_text('\n'.join(row.rstrip() for row in svg.read_text().splitlines())+'\n')
print(destination)
