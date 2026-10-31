#!/usr/bin/env python3
"""Read-only gate-transition and class-count audit for an existing ROS bag."""

import argparse
from collections import Counter
from pathlib import Path
import sys

import rosbag
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'base_ws/src/perception_object_pipeline/src'))
from perception_object_pipeline.map_based_prediction import LaneletNetwork, MapPredictionConfig
from perception_object_pipeline.vehicle_only_corridor import VehicleOnlyCorridor
from perception_object_pipeline.kinematic_prediction import quaternion_to_yaw


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path)
    parser.add_argument('--map-version', type=int, choices=(88, 94), default=94)
    args = parser.parse_args()
    network = LaneletNetwork.load(
        str(ROOT / f'kcity_map/lanelet_map/lanelet2_map_v{args.map_version}.osm'),
        str(ROOT / 'kcity_map/lanelet_map/map_projector_info_v1.yaml'),
        MapPredictionConfig())
    route = yaml.safe_load((ROOT / (
        'base_ws/src/planning_addons/competition_route_requester/config/'
        f'kcity_v{args.map_version}_competition_route.yaml')).read_text())
    gate = VehicleOnlyCorridor(network, route)
    counts = Counter()
    tracked_counts = Counter()
    transitions = []
    active = False
    start_time = None
    with rosbag.Bag(str(args.bag)) as bag:
        for topic, message, stamp in bag.read_messages(topics=(
                '/localization/kinematic_state',
                '/perception/object_recognition/objects',
                '/perception/object_recognition/tracking/objects')):
            if start_time is None:
                start_time = stamp.to_sec()
            if topic == '/localization/kinematic_state':
                if message.header.frame_id != 'map':
                    continue
                pose = message.pose.pose
                next_active, _, _ = gate.evaluate(
                    pose.position.x, pose.position.y,
                    quaternion_to_yaw(pose.orientation))
                if next_active != active:
                    active = next_active
                    transitions.append((round(stamp.to_sec() - start_time, 2),
                                        'ON' if active else 'OFF',
                                        round(pose.position.x, 2),
                                        round(pose.position.y, 2)))
            elif topic == '/perception/object_recognition/objects':
                for obj in message.objects:
                    label = obj.classification[0].label if obj.classification else -1
                    counts[(active, label)] += 1
            else:
                for obj in message.objects:
                    label = obj.classification[0].label if obj.classification else -1
                    tracked_counts[(active, label)] += 1
    print('transitions (bag second, state, map x, map y):')
    for item in transitions:
        print(*item)
    print('prediction object occurrences by (corridor active, top class):')
    for key in sorted(counts):
        print(key, counts[key])
    print('upstream tracked object occurrences by (corridor active, top class):')
    for key in sorted(tracked_counts):
        print(key, tracked_counts[key])


if __name__ == '__main__':
    main()
