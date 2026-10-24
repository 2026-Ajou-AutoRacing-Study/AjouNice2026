#!/usr/bin/env python3
"""Build v86 from v85, changing only the entry and final bottleneck tapers.

Retain the through lanes and road exterior. Replace overlapping successor
corridors with disjoint terminating lanes that share LC-enabled boundaries.
The released v85 is immutable. No planner, speed, route, or launch tuning.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path

from lxml import etree as E

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'lanelet_map/lanelet2_map_v85.osm'
OUTPUT = ROOT / 'lanelet_map/lanelet2_map_v86.osm'
REPORT = ROOT / 'docs/kcity_v86_highway_lane_end_lc_build.json'
EXPECTED_SHA = '5111ae16ba788b059e40e88201b51b0cb4d1d662dcd8456f43cda9705066be58'
# right-side terminating lane -> (left-side through lane, shared boundary).
SHARED = {201468: (201467, 105550), 201475: (201474, 105588),
          201446: (201441, 105526), 201441: (201439, 105520)}
OBSOLETE_BOUNDARIES = {105554, 105592, 105530, 105524}
CHANGED_CENTERLINES = {105558, 105596, 105534, 105528}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def tags(element):
    return {x.get('k'): x.get('v') for x in element.findall('tag')}


def refs(way):
    return [int(x.get('ref')) for x in way.findall('nd')]


def member(relation, role):
    result = [x for x in relation.findall('member') if x.get('role') == role]
    assert len(result) == 1, (relation.get('id'), role)
    return result[0]


def set_tag(element, key, value):
    found = next((x for x in element.findall('tag') if x.get('k') == key), None)
    if found is None:
        found = E.SubElement(element, 'tag', k=key)
    found.set('v', value)


def set_refs(way, ids):
    for x in list(way.findall('nd')):
        way.remove(x)
    for i, node in enumerate(ids):
        way.insert(i, E.Element('nd', ref=str(node)))


def primitive_diff(old, new):
    result = {}
    for kind in ['node', 'way', 'relation']:
        a = {int(x.get('id')): E.tostring(x, with_tail=False) for x in old.findall(kind)}
        b = {int(x.get('id')): E.tostring(x, with_tail=False) for x in new.findall(kind)}
        result[kind] = {'added': sorted(b.keys()-a.keys()), 'removed': sorted(a.keys()-b.keys()),
                        'changed': sorted(i for i in a.keys() & b.keys() if a[i] != b[i])}
    return result


def build(output=OUTPUT, report=REPORT):
    if output.exists():
        raise RuntimeError(f'Refusing to overwrite existing version: {output}')
    assert sha256(BASE) == EXPECTED_SHA, 'v85 has changed'
    tree = E.parse(str(BASE), E.XMLParser(remove_blank_text=False, huge_tree=True))
    root = tree.getroot()
    old = E.fromstring(E.tostring(root))
    assert root.find('MetaInfo').get('map_version') == '85'
    nodes = {int(x.get('id')): x for x in root.findall('node')}
    ways = {int(x.get('id')): x for x in root.findall('way')}
    relations = {int(x.get('id')): x for x in root.findall('relation')}

    for lane, (left_lane, boundary) in SHARED.items():
        assert int(member(relations[left_lane], 'right').get('ref')) == boundary
        member(relations[lane], 'left').set('ref', str(boundary))
        # The virtual marking remains virtual: do not invent a painted line.
        # Permit only escape from the right-side dying lane to its left.
        assert tags(ways[boundary])['type'] == 'virtual'
        set_tag(ways[boundary], 'lane_change:left', 'yes')
        set_tag(ways[boundary], 'lane_change:right', 'no')

    # Triangular lane tips are actual boundary points, not duplicate-position
    # nodes or a full-width cross-section connected to the through successor.
    set_refs(ways[105558], [refs(ways[105558])[0], 10002])
    set_refs(ways[105528], [refs(ways[105528])[0], 228079])

    # Use one shared centerline node at the retained 201475 -> 201446 joint.
    # Add a node rather than moving any v85 node used elsewhere in the map.
    from pyproj import Transformer
    forward = Transformer.from_crs(4326, 32652, always_xy=True)
    inverse = Transformer.from_crs(32652, 4326, always_xy=True)
    def xyz(i):
        node = nodes[i]
        x, y = forward.transform(float(node.get('lon')), float(node.get('lat')))
        return x, y, float(tags(node).get('ele', 0))
    a, b = xyz(232279), xyz(232732)
    midpoint = tuple((a[i]+b[i])/2 for i in range(3))
    lon, lat = inverse.transform(*midpoint[:2])
    new_id = max(nodes)+1
    point = E.Element('node', id=str(new_id), lat=f'{lat:.14f}', lon=f'{lon:.14f}')
    E.SubElement(point, 'tag', k='ele', v=f'{midpoint[2]:.9f}')
    root.insert(root.index(root.find('way')), point)
    nodes[new_id] = point
    set_refs(ways[105596], [refs(ways[105596])[0], new_id])
    set_refs(ways[105534], [new_id, 228079])

    # Erase superseded target geometry instead of leaving display-only
    # diagonal corridors in VMB. Remove only now-unused nodes of those ways.
    candidates = set()
    for way_id in OBSOLETE_BOUNDARIES:
        candidates.update(refs(ways[way_id]))
    for way_id in CHANGED_CENTERLINES:
        previous = next(x for x in old.findall('way') if int(x.get('id')) == way_id)
        candidates.update(refs(previous))
    # A superseded, unreferenced center point lies 4.7 cm from the new local
    # bottleneck joint. It is inside this edit area; retain no duplicate marker.
    assert sum((xyz(132193)[i]-midpoint[i])**2 for i in range(2)) < 0.1**2
    candidates.add(132193)
    way_users = defaultdict(list)
    for rel in relations.values():
        for item in rel.findall('member'):
            if item.get('type') == 'way':
                way_users[int(item.get('ref'))].append(int(rel.get('id')))
    for way_id in OBSOLETE_BOUNDARIES:
        assert not way_users[way_id], (way_id, way_users[way_id])
        root.remove(ways[way_id])
    used_nodes = {int(x.get('ref')) for way in root.findall('way') for x in way.findall('nd')}
    used_nodes |= {int(x.get('ref')) for rel in relations.values() for x in rel.findall('member') if x.get('type') == 'node'}
    for i in candidates-used_nodes:
        assert not set(tags(nodes[i]))-{'local_x', 'local_y', 'ele'}, i
        root.remove(nodes[i])

    root.find('MetaInfo').set('map_version', '86')
    diff = primitive_diff(old, root)
    assert diff['node']['changed'] == []
    assert diff['relation'] == {'added': [], 'removed': [], 'changed': sorted(SHARED)}
    assert diff['way']['added'] == []
    assert diff['way']['removed'] == sorted(OBSOLETE_BOUNDARIES)
    assert diff['way']['changed'] == sorted(CHANGED_CENTERLINES | {v[1] for v in SHARED.values()})
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(output), encoding='UTF-8', xml_declaration=True, pretty_print=False)
    result = {'input': str(BASE.relative_to(ROOT)), 'input_sha256': sha256(BASE),
              'output': str(output), 'output_sha256': sha256(output), 'map_version': 86,
              'primitive_diff': diff,
              'new_centerline_joint_node': new_id,
              'lane_change_edges': [{'from': i, 'to': v[0], 'shared_boundary': v[1]} for i, v in SHARED.items()],
              'unchanged': ['all existing node coordinates', 'all lanelet attributes including speeds and regulatory references',
                            'all through-lane boundaries and road exterior coordinates', 'all regions outside entry/final bottleneck',
                            'planner code, safety margins, and runtime settings'],
              'note': 'Terminating taper geometry is not a guarantee of LC execution or moving merge performance.'}
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=OUTPUT)
    parser.add_argument('--report', type=Path, default=REPORT)
    args = parser.parse_args()
    print(json.dumps(build(args.output, args.report), ensure_ascii=False, indent=2))
