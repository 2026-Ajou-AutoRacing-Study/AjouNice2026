#!/usr/bin/env python3
"""Draw actual v86/v87 OSM geometry, including the retained LC interface."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from lxml import etree as E
from pyproj import Transformer
import shapefile
from build_v87_highway_gis_compaction import ROOT
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams['font.family']=font_manager.FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc').get_name()
plt.rcParams['svg.fonttype']='none'
f=Transformer.from_crs(4326,32652,always_xy=True);ox,oy=f.transform(126.7730819386,37.2380038082)
entry86=[[201463,201437,201464],[201459,201436,201465],[201458,201435,201466],[201457,200099,201467],[201456,201438,201468]]
entry87=[[201463,201464],[201459,201465],[201458,201466],[201457,201467],[201456,201468]]
bottle86=[[201445,200148,201472,201440],[201469,200127,201473,201439],[201470,200130,201474,201441],[201471,200134,201475,201446]]
bottle87=[[201445,201440],[201469,201439],[201470,201441],[201471,201446]]
colors=['#dce7f2','#cce9df','#d9dcf4','#f8e1b8','#f1c7c7']

def read(v):
 root=E.parse(str(ROOT/f'lanelet_map/lanelet2_map_v{v}.osm'))
 nodes={int(n.get('id')):f.transform(float(n.get('lon')),float(n.get('lat'))) for n in root.findall('node')}
 ways={int(w.get('id')):[(nodes[int(n.get('ref'))][0]-ox,nodes[int(n.get('ref'))][1]-oy) for n in w.findall('nd')] for w in root.findall('way')}
 rels={int(r.get('id')):{m.get('role'):int(m.get('ref')) for m in r.findall('member') if m.get('type')=='way'} for r in root.findall('relation')}
 return ways,rels

def panel(ax,v,groups,limits,title):
 ways,rels=read(v)
 for j,group in enumerate(groups):
  for lane in group:
   a=ways[rels[lane]['left']];b=ways[rels[lane]['right']]
   ax.add_patch(Polygon(a+b[::-1],facecolor=colors[j],edgecolor='#8493a5',linewidth=.8))
   for side in [a,b]:ax.plot(*zip(*side),color='#344b61',lw=1.3)
   # Place each label halfway along its own lane, inside the broad portion of triangles.
   t=.38 if lane in [201468,201441,201446] and v==87 else .5
   la=(a[0][0]*(1-t)+a[-1][0]*t,a[0][1]*(1-t)+a[-1][1]*t)
   rb=(b[0][0]*(1-t)+b[-1][0]*t,b[0][1]*(1-t)+b[-1][1]*t)
   y=(la[1]+rb[1])/2;x=(la[0]+rb[0])/2
   if limits[2]<y<limits[3]:ax.text(x,y,str(lane),fontsize=9,ha='center',va='center',rotation=90,bbox=dict(facecolor='white',alpha=.7,edgecolor='none',pad=1))
 ax.set(xlim=limits[:2],ylim=limits[2:],title=title,xlabel='횡방향 map x [m]',ylabel='주행방향 map y [m] (위 → 아래)')
 ax.grid(alpha=.15);ax.annotate('주행 ↓',xy=(.99,.96),xycoords='axes fraction',ha='right',color='#345')
 for sr in shapefile.Reader(str(ROOT/'gis_map/B2_SURFACELINEMARK'),encoding='utf-8').iterShapeRecords():
  if sr.record.as_dict()['ID'] in (['B2256W000038'] if limits[2]>600 else ['B2256W000043']):
   ax.plot(*zip(*[(p[0]-ox,p[1]-oy) for p in sr.shape.points]),color='#cc3e8b',ls='--',lw=1,label='GIS 원본 구분선')
 ax.legend(loc='lower left',fontsize=8)
fig,axs=plt.subplots(2,2,figsize=(16,15))
panel(axs[0,0],86,entry86,(191,213,630,809),'진입부 v86: 짧은 조각과 꺾인 경계')
panel(axs[0,1],87,entry87,(191,213,630,809),'진입부 v87: 직선 경계 + 소실 삼각형 201468')
panel(axs[1,0],86,bottle86,(197,215,323,410),'병목 v86: 세로로 나뉜 조각')
panel(axs[1,1],87,bottle87,(197,215,323,410),'병목 v87: 본선 201439 / 삼각형 201441·201446')
fig.suptitle('K-City v87 실제 OSM 비교 — 진입·병목 국소 수정',fontsize=20,y=.985)
fig.text(.5,.015,'실제 좌표를 사용하되 차로 폭을 보기 위해 가로·세로 축척은 다르게 표시.\n진입 본선 201457 → 201467 분할은 별도 201456·201468과의 LC 연결을 위해 유지. 세 소실 차로의 정상 후속 연결은 없음.',ha='center',fontsize=12)
fig.tight_layout(rect=[0,.055,1,.96])
for ext in ['svg','png']:
    target=ROOT/f'docs/kcity_v87_highway_gis_compaction.{ext}'
    fig.savefig(target,dpi=150)
    if ext=='svg':target.write_text('\n'.join(line.rstrip() for line in target.read_text().splitlines())+'\n')
print('Created actual-map SVG and PNG')
