#!/usr/bin/env python3
"""Read-only actual-route/localization replay; no ROS publisher or Ego oracle.

This is a route-provider audit, not an image identity or driving safety score.
Use the bag's original map when available. Missing lane IDs are reported.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'base_ws/src/perception/traffic_light_detection/scripts'))


def sha(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024**2), b''):
            value.update(block)
    return value.hexdigest()


def audit(bag_path, map_path, projector):
    import rosbag
    import rospy
    from lanelet_route_matcher import LaneletRouteGroupIdMatcher
    from traffic_light_route_group_id_node import restrict_route_groups, MISSION_GROUP_IDS

    matcher = LaneletRouteGroupIdMatcher(str(map_path), 'LocalCartesianUTM', str(projector),
                                        excluded_group_ids={201395, 201396})
    if not matcher.available:
        raise RuntimeError('System-Python Lanelet2 map load failed')
    matcher.lanelet_group_id = restrict_route_groups(matcher.lanelet_group_id, sorted(MISSION_GROUP_IDS))
    map_ids = {int(lane.id) for lane in matcher.map.laneletLayer}
    clock = [rospy.Time(0)]
    counts, transitions, routes = Counter(), [], []
    previous = object()
    topics = ['/planning/mission_planning/route', '/planning/route_state', '/localization/kinematic_state']
    with patch('rospy.Time.now', side_effect=lambda: clock[0]), rosbag.Bag(str(bag_path)) as bag:
        start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=topics):
            clock[0] = stamp
            elapsed = stamp.to_sec() - start
            if topic == topics[0]:
                matcher._route_callback(msg)
                ids = {item for segment in matcher._route_segments for item in segment}
                routes.append(dict(t=elapsed, segments=len(matcher._route_segments),
                                   absent_map_lanelet_ids=sorted(ids - map_ids)))
            elif topic == topics[1]:
                matcher._route_state_callback(msg)
            else:
                matcher._odom_callback(msg)
                group = matcher.get_group_id()
                counts[str(group)] += 1
                if group != previous:
                    transitions.append(dict(t=elapsed, group=group, xy=list(matcher._ego_xy)))
                    previous = group
    return dict(schema='traffic_light_route_replay_v1',
                inputs={str(p): sha(p) for p in (bag_path, map_path, projector)},
                route_provider_sha256=sha(ROOT / 'base_ws/src/perception/traffic_light_detection/scripts/lanelet_route_matcher.py'),
                observation_counts=dict(counts), transitions=transitions, routes=routes,
                map_compatible=bool(routes) and not any(r['absent_map_lanelet_ids'] for r in routes),
                limitations=['Receipt-time replay of source matcher, no ROS output.',
                             'No image/native/Ego identity ground truth or planning policy certification.',
                             'Missing map IDs prevent an original-map equivalence claim.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--map', type=Path, default=ROOT / 'kcity_map/lanelet_map/lanelet2_map_v87.osm')
    parser.add_argument('--projector', type=Path, default=ROOT / 'kcity_map/lanelet_map/map_projector_info_v1.yaml')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    target = args.output.resolve()
    target.relative_to(ROOT / 'artifacts/evaluation')
    if target.exists():
        parser.error('Use a fresh output; evidence is never overwritten')
    result = audit(args.bag.resolve(), args.map.resolve(), args.projector.resolve())
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open('x') as stream:
        json.dump(result, stream, indent=2); stream.write('\n')
    print(json.dumps(dict(output=str(target), map_compatible=result['map_compatible'],
                          observation_counts=result['observation_counts'], transitions=result['transitions'])))


if __name__ == '__main__':
    main()
