#!/usr/bin/env python3
"""Release v68: restrict roundabout 201455 to its ring and immediate arms."""
from pathlib import Path
from copy import deepcopy
import hashlib
import json
from lxml import etree as E

ROOT = Path(__file__).resolve().parents[2]
ROLES = {
    'entry': ['201095', '201101', '201090', '201146'],
    'internal': ['201094', '201097', '201093', '201144'],
    'exit': ['201098', '201091', '201147', '201108'],
}

def main():
    source = ROOT / 'lanelet_map/lanelet2_map_v67.osm'
    output = ROOT / 'lanelet_map/lanelet2_map_v68.osm'
    if output.exists():
        raise RuntimeError('Released maps are immutable: output already exists')
    tree = E.parse(str(source))
    root = tree.getroot()
    baseline = deepcopy(root)
    relations = {r.get('id'): r for r in root.findall('relation')}
    selected = set(sum(ROLES.values(), []))
    reg = relations['201455']
    old_members = [m.get('ref') for m in reg.findall('member')]
    for m in list(reg.findall('member')):
        reg.remove(m)
    for role, ids in reversed(list(ROLES.items())):
        for lane_id in reversed(ids):
            reg.insert(0, E.Element('member', type='relation', ref=lane_id, role=role))
    for lane_id, relation in relations.items():
        for m in list(relation.findall("member[@role='regulatory_element'][@ref='201455']")):
            relation.remove(m)
        if lane_id in selected:
            relation.insert(0, E.Element('member', type='relation', ref='201455', role='regulatory_element'))
    root.find('MetaInfo').set('map_version', '68')
    # Exhaustive semantic diff: only this regulatory membership and version may change.
    normalized = deepcopy(root)
    normalized.find('MetaInfo').set('map_version', '67')
    for doc in (baseline, normalized):
        for relation in doc.findall('relation'):
            if relation.get('id') == '201455':
                for m in list(relation.findall('member')):
                    relation.remove(m)
            for m in list(relation.findall("member[@role='regulatory_element'][@ref='201455']")):
                relation.remove(m)
        for element in doc.iter():
            element.text = (element.text or '').strip() or None
            element.tail = None
    assert E.tostring(baseline) == E.tostring(normalized)
    attached = {i for i,r in relations.items() if r.find("member[@role='regulatory_element'][@ref='201455']") is not None}
    assert attached == selected
    # Check directed endpoint connections, independently of the membership list.
    nodes = {n.get('id'): (float(n.get('lon')), float(n.get('lat'))) for n in root.findall('node')}
    ways = {w.get('id'): [nodes[n.get('ref')] for n in w.findall('nd')] for w in root.findall('way')}
    def ends(i):
        p = ways[relations[i].find("member[@role='centerline']").get('ref')]
        return p[0], p[-1]
    def close(a,b):
        return sum((x-y)**2 for x,y in zip(a,b)) < 1e-14
    ring = ROLES['internal']
    for a,b in zip(ring,ring[1:]+ring[:1]):
        assert close(ends(a)[1],ends(b)[0]), (a,b)
    for i in ROLES['entry']:
        assert any(close(ends(i)[1],ends(j)[0]) for j in ring), i
    for i in ROLES['exit']:
        assert any(close(ends(i)[0],ends(j)[1]) for j in ring), i
    tree.write(str(output), encoding='UTF-8', xml_declaration=True, pretty_print=True)
    report = dict(version=68, regulatory_element=201455, roles=ROLES,
                  removed=sorted(set(old_members)-selected), added=sorted(selected-set(old_members)),
                  geometry_and_other_semantics_unchanged=True,
                  sha256=hashlib.sha256(output.read_bytes()).hexdigest())
    (ROOT / 'docs/kcity_v68_roundabout_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__ == '__main__':
    main()
