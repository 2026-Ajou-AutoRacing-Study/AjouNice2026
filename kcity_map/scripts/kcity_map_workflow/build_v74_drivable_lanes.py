#!/usr/bin/env python3
"""Create immutable v74 by deleting the eight excluded highway lanelet relations."""
from pathlib import Path
import hashlib,json
from lxml import etree as E
ROOT=Path(__file__).resolve().parents[2]
REMOVED={201400,201440,200148,201445,200141,200186,201437,201463}
SOLID={102241,102237,101992,105462}
def main():
 src=ROOT/'lanelet_map/lanelet2_map_v73.osm';dst=ROOT/'lanelet_map/lanelet2_map_v74.osm'
 if dst.exists(): raise RuntimeError('Version already exists: '+str(dst))
 tree=E.parse(str(src),E.XMLParser(huge_tree=True));root=tree.getroot()
 relations={int(e.get('id')):e for e in root.findall('relation')}
 assert REMOVED <= relations.keys()
 for e in relations.values():
  for m in e.findall('member'):
   assert not (m.get('type')=='relation' and int(m.get('ref')) in REMOVED), ('incoming reference',e.get('id'))
 for i in REMOVED:root.remove(relations[i])
 ways={int(e.get('id')):e for e in root.findall('way')}
 for i in SOLID:
  tags={x.get('k'):x for x in ways[i].findall('tag')}
  assert tags['subtype'].get('v')=='dashed'
  tags['subtype'].set('v','solid')
 root.find('MetaInfo').set('map_version','74')
 tree.write(str(dst),encoding='UTF-8',xml_declaration=True,pretty_print=True)
 report={'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'output_sha256':hashlib.sha256(dst.read_bytes()).hexdigest(),'removed_lanelets':sorted(REMOVED),'restored_solid_boundaries':sorted(SOLID),'preserved_merge_chain':[201456,201438,200128]}
 (ROOT/'docs/kcity_v74_drivable_lanes_report.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()
