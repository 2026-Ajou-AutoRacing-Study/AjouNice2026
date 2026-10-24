#!/usr/bin/env python3
"""Independently check v86 scope, VMB geometry classes, LC graph, and routes.

Run after sourcing ROS Noetic and planning_ws/install_isolated. This is a
static validator, not a replacement for VMB's own problem list or live LC.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import ctypes
import itertools
import json
import math
from pathlib import Path

from lxml import etree as E
from shapely.geometry import LineString, Point, Polygon
import yaml

from build_v86_highway_lane_end_lc import (
    BASE, EXPECTED_SHA, ROOT, OUTPUT, SHARED, OBSOLETE_BOUNDARIES,
    CHANGED_CENTERLINES, member, primitive_diff, refs, sha256, tags,
)

EDIT_LANES = {201468, 201467, 201475, 201474, 201446, 201441, 201439}
GRAPH_INTERFACE_LANES = EDIT_LANES | {200128, 200185, 200360}
REMOVED_NODES = {132193, 232581, 233838, 233876, 233881, 233882, 233902}


def xml(path):
    return E.parse(str(path), E.XMLParser(remove_blank_text=False, huge_tree=True)).getroot()


def layers(root):
    return {kind: {int(x.get('id')): x for x in root.findall(kind)} for kind in ['node', 'way', 'relation']}


def signature(graph, lane):
    def one(method):
        value = method(lane)
        return value.id if value else None
    return {'following': sorted(x.id for x in graph.following(lane)),
            'previous': sorted(x.id for x in graph.previous(lane)),
            'left': one(graph.left), 'right': one(graph.right),
            'adjacent_left': one(graph.adjacentLeft), 'adjacent_right': one(graph.adjacentRight),
            'conflicting': sorted(x.id for x in graph.conflicting(lane))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=OUTPUT)
    parser.add_argument('--report', type=Path, default=ROOT/'docs/kcity_v86_highway_lane_end_lc_validation.json')
    args = parser.parse_args()
    assert sha256(BASE) == EXPECTED_SHA
    old_root, new_root = xml(BASE), xml(args.map)
    old, new = layers(old_root), layers(new_root)
    assert old_root.find('MetaInfo').get('map_version') == '85'
    assert new_root.find('MetaInfo').get('map_version') == '86'
    diff = primitive_diff(old_root, new_root)
    assert diff['node'] == {'added': [233914], 'removed': sorted(REMOVED_NODES), 'changed': []}, diff['node']
    assert diff['way'] == {'added': [], 'removed': sorted(OBSOLETE_BOUNDARIES),
                           'changed': sorted(CHANGED_CENTERLINES | {v[1] for v in SHARED.values()})}, diff['way']
    assert diff['relation'] == {'added': [], 'removed': [], 'changed': sorted(SHARED)}, diff['relation']

    # Preserve all original primitive values outside the exact allowlist.
    # Existing nodes are never moved. Removal candidates are target-owned or
    # the proven orphan in the bottleneck; unrelated users cannot be removed.
    changed_ways = set(diff['way']['changed']) | OBSOLETE_BOUNDARIES
    for i, rel in old['relation'].items():
        if i in SHARED:
            assert rel.attrib == new['relation'][i].attrib
            assert tags(rel) == tags(new['relation'][i])
            a, b = list(rel.findall('member')), list(new['relation'][i].findall('member'))
            assert len(a) == len(b)
            for first, second in zip(a, b):
                if first.get('role') != 'left': assert dict(first.attrib) == dict(second.attrib), i
        else:
            assert E.tostring(rel, with_tail=False) == E.tostring(new['relation'][i], with_tail=False), i
            used = {int(x.get('ref')) for x in rel.findall('member') if x.get('type') == 'way'}
            assert not used & changed_ways or i in EDIT_LANES, (i, used & changed_ways)
    for i in REMOVED_NODES:
        users = {wi for wi, way in old['way'].items() if i in refs(way)}
        assert users <= changed_ways, (i, users)
    for kind, items in new.items():
        assert len(items) == len(new_root.findall(kind)), ('duplicate ID', kind)
    missing_refs = []
    for i, way in new['way'].items():
        missing_refs += [(i, j) for j in refs(way) if j not in new['node']]
    for i, rel in new['relation'].items():
        for item in rel.findall('member'):
            if int(item.get('ref')) not in new[item.get('type')]: missing_refs.append((i, dict(item.attrib)))
    assert not missing_refs, missing_refs

    ctypes.CDLL(str(ROOT.parent/'planning_ws/install_isolated/lib/libautoware_lanelet2_extension_lib.so'), mode=ctypes.RTLD_GLOBAL)
    import lanelet2
    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(37.2380038082, 126.7730819386))
    before = lanelet2.io.load(str(BASE), projector)
    after = lanelet2.io.load(str(args.map), projector)
    rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
    bg, ag = [lanelet2.routing.RoutingGraph(m, rules) for m in [before, after]]
    assert not ag.checkValidity(False), ag.checkValidity(False)
    assert {x.id for x in before.laneletLayer} == {x.id for x in after.laneletLayer}
    changed_graph = []
    graph_rows = {}
    polygon = {}
    invalid, outside, wrong_direction, consecutive_duplicates = [], [], [], []
    baseline_outside, vmb_interior_outside = [], []
    for lane in after.laneletLayer:
        i = lane.id
        b, a = signature(bg, before.laneletLayer[i]), signature(ag, lane)
        if a != b:
            changed_graph.append(i)
            assert i in GRAPH_INTERFACE_LANES, (i, b, a)
            graph_rows[str(i)] = {'before': b, 'after': a}
        left = [(p.x, p.y) for p in lane.leftBound]
        right = [(p.x, p.y) for p in lane.rightBound]
        shape = Polygon(left+list(reversed(right)))
        polygon[i] = shape
        if not shape.is_valid or shape.area <= 1e-6: invalid.append(i)
        forward = math.dist(left[0], right[0])+math.dist(left[-1], right[-1])
        reverse = math.dist(left[0], right[-1])+math.dist(left[-1], right[0])
        if reverse < forward-1e-6: wrong_direction.append(i)
        center = LineString([(p.x, p.y) for p in lane.centerline])
        if not shape.buffer(.001).covers(center): outside.append(i)
        prev = before.laneletLayer[i]
        prev_shape = Polygon([(p.x, p.y) for p in prev.leftBound] + [(p.x, p.y) for p in reversed(prev.rightBound)])
        if not prev_shape.buffer(.001).covers(LineString([(p.x, p.y) for p in prev.centerline])):
            baseline_outside.append(i)
        if any(x.get('role') == 'centerline' for x in new['relation'][i].findall('member')):
            if any(not shape.buffer(.001).covers(Point(p.x, p.y)) for p in list(lane.centerline)[1:-1]):
                vmb_interior_outside.append(i)
    assert not invalid, invalid
    assert not set(outside) & EDIT_LANES, sorted(set(outside) & EDIT_LANES)
    assert set(outside) == set(baseline_outside), sorted(set(outside)^set(baseline_outside))
    assert not vmb_interior_outside, vmb_interior_outside
    assert not wrong_direction, wrong_direction
    for way in after.lineStringLayer:
        for a, b in zip(list(way), list(way)[1:]):
            if a.id == b.id or math.hypot(a.x-b.x, a.y-b.y) < 1e-6:
                consecutive_duplicates.append(way.id)
    assert not consecutive_duplicates, consecutive_duplicates

    expected_following = {201468: [], 201467: [200128], 201475: [201446],
                          201474: [201441], 201446: [], 201441: [], 201439: [200185, 200360]}
    for i, expected in expected_following.items():
        assert sorted(x.id for x in ag.following(after.laneletLayer[i])) == expected, i
    for i, (left_id, boundary_id) in SHARED.items():
        source, dest = after.laneletLayer[i], after.laneletLayer[left_id]
        assert source.leftBound.id == dest.rightBound.id == boundary_id
        assert ag.left(source) and ag.left(source).id == left_id, i
        assert not ag.right(dest), left_id
        assert ag.adjacentRight(dest) and ag.adjacentRight(dest).id == i
        assert tags(new['way'][boundary_id]) == {'type': 'virtual', 'lane_change:left': 'yes', 'lane_change:right': 'no'}
    # Existing safe upstream parallel change regions must remain available.
    for i, target in [(201456, 201457), (201438, 200099), (200134, 200130), (200130, 200127), (201474, 201473)]:
        assert ag.left(after.laneletLayer[i]).id == target

    overlaps = []
    for group in [[201468, 201467], [201475, 201474, 201473], [201446, 201441, 201439]]:
        for a, b in itertools.combinations(group, 2):
            area = polygon[a].intersection(polygon[b]).area
            assert area < 1e-6, (a, b, area)
            overlaps.append({'a': a, 'b': b, 'overlap_m2': area})

    # No successor-only escape from a dying lane into the downstream through
    # road. Shortest paths must instead include an actual lateral LC edge.
    probes = []
    for start, goal in [(201468, 200128), (201456, 200128), (201475, 200185), (201446, 200185), (201441, 200185)]:
        source, dest = after.laneletLayer[start], after.laneletLayer[goal]
        route = ag.shortestPath(source, dest)
        assert route, (start, goal)
        path = [x.id for x in route]
        lc = []
        for a, b in zip(path, path[1:]):
            s = signature(ag, after.laneletLayer[a])
            if b not in s['following']:
                assert b in [s['left'], s['right']], (a, b)
                lc.append([a, b])
        assert lc, (start, goal, path)
        seen, todo = set(), [start]
        while todo:
            i = todo.pop()
            if i in seen: continue
            seen.add(i)
            todo.extend(x.id for x in ag.following(after.laneletLayer[i]))
        assert goal not in seen, (start, goal)
        probes.append({'start': start, 'goal': goal, 'path': path, 'required_lc_edges': lc,
                       'successor_only_reachable': False})

    # The v85 preferred route IDs and waypoint coordinates remain usable.
    definition = yaml.safe_load((ROOT.parent/'base_ws/src/planning_addons/competition_route_requester/config/kcity_v85_competition_route.yaml').read_text())
    routes = {}
    for name in ['route_a', 'route_b', 'fallback_route']:
        seq = definition[name]['expected_path_lanelet_ids']
        assert all(after.laneletLayer.exists(i) for i in seq)
        for a, b in zip(seq, seq[1:]):
            assert ag.shortestPath(after.laneletLayer[a], after.laneletLayer[b]), (name, a, b)
        routes[name] = {'all_ids_exist': True, 'all_preferred_transitions_reachable': True}
    for name, pose in definition['poses'].items():
        lane = after.laneletLayer[pose['lanelet_id']]
        assert polygon[lane.id].buffer(.001).covers(Point(pose['x'], pose['y'])), (name, lane.id)

    # VMB checks identity of the centerline node at longitudinal joints.
    def center_joints(layers):
        starts = defaultdict(list)
        ends = []
        for i, rel in layers['relation'].items():
            if tags(rel).get('type') != 'lanelet': continue
            if not any(x.get('role') == 'centerline' for x in rel.findall('member')): continue
            chains = [refs(layers['way'][int(member(rel, role).get('ref'))]) for role in ['left', 'right', 'centerline']]
            left, right, center = chains
            starts[(left[0], right[0])].append((i, center[0]))
            ends.append((i, (left[-1], right[-1]), center[-1]))
        return {(i, j) for i, key, end in ends for j, start in starts.get(key, []) if end != start}
    old_joints, new_joints = center_joints(old), center_joints(new)
    assert not new_joints-old_joints, sorted(new_joints-old_joints)
    speed_before = Counter(dict(x.attributes).get('speed_limit') for x in before.laneletLayer)
    speed_after = Counter(dict(x.attributes).get('speed_limit') for x in after.laneletLayer)
    assert speed_before == speed_after
    report = {'status': 'PASS', 'map': str(args.map), 'sha256': sha256(args.map),
              'baseline_sha256': sha256(BASE), 'map_version': 86, 'lanelet_count': len(after.laneletLayer),
              'primitive_diff': diff, 'graph_changed_lanelets': sorted(changed_graph), 'graph_changes': graph_rows,
              'invalid_lanelet_polygons': invalid, 'vmb_explicit_centerline_interior_outside': vmb_interior_outside,
              'edited_centerlines_outside_lanelets': sorted(set(outside) & EDIT_LANES),
              'existing_global_whole_centerline_outside_count': len(baseline_outside),
              'new_global_whole_centerline_outside': sorted(set(outside)-set(baseline_outside)),
              'wrong_bound_directions': wrong_direction, 'adjacent_duplicate_points': consecutive_duplicates,
              'new_disconnected_centerline_joints': sorted(new_joints-old_joints),
              'existing_disconnected_centerline_joints': len(old_joints),
              'local_overlaps': overlaps, 'route_probes': probes, 'existing_preferred_route_compatibility': routes,
              'speed_distribution_unchanged': True, 'outside_edit_scope_unchanged': True,
              'vmb_direct_validation': 'See separate actual-VMB problem-list report; not inferred by this script.',
              'live_lc_execution': 'not run'}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['graph_changes']}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
