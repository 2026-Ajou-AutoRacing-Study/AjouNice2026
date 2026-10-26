#!/usr/bin/env python3
"""Render the production vehicle-only corridor against the entire Lanelet2 map."""

import argparse
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'base_ws/src/perception_object_pipeline/src'))
from perception_object_pipeline.map_based_prediction import LaneletNetwork, MapPredictionConfig
from perception_object_pipeline.vehicle_only_corridor import VehicleOnlyCorridor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-version', type=int, choices=(88, 94), default=94)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    map_file = ROOT / f'kcity_map/lanelet_map/lanelet2_map_v{args.map_version}.osm'
    projector = ROOT / 'kcity_map/lanelet_map/map_projector_info_v1.yaml'
    route_file = ROOT / ('base_ws/src/planning_addons/competition_route_requester/config/'
                         f'kcity_v{args.map_version}_competition_route.yaml')
    network = LaneletNetwork.load(str(map_file), str(projector), MapPredictionConfig())
    route = yaml.safe_load(route_file.read_text())
    gate = VehicleOnlyCorridor(network, route)

    figure, ax = plt.subplots(figsize=(11, 14))
    for record in network.records.values():
        xy = record.centerline
        ax.plot([p[0] for p in xy], [p[1] for p in xy], color='#ccd1d5', lw=.35)
    all_ids = route['route_a']['waypoint_path_lanelet_ids']
    for lanelet_id in all_ids:
        xy = network.records[int(lanelet_id)].centerline
        ax.plot([p[0] for p in xy], [p[1] for p in xy], color='#3478bb', lw=1.0)
    section = [gate.point_at_arc(arc) for arc in
               list(range(int(gate.start_s_m), int(gate.end_s_m), 2))]
    section = [gate.start_xy] + section + [gate.end_xy]
    ax.plot([p[0] for p in section], [p[1] for p in section],
            color='#e86224', lw=2.3)
    ax.scatter(*gate.start_xy, color='#198754', s=90, zorder=5)
    ax.scatter(*gate.end_xy, color='#d63447', s=90, zorder=5)
    labels = (
        (gate.start_xy, 'ON after C crossing', (8, 15)),
        (gate.end_xy, 'OFF before GPS shadow', (10, -17)),
        ((71.4, 511.4), 'C signal', (-30, -16)),
        ((118.9, 643.2), 'D signal', (9, 6)),
        ((74.8, 534.3), 'sample dynamic pedestrian', (9, 2)),
        ((152.2, 53.5), 'nearest shadow actor in 24 cases', (9, -5)),
        ((201.9, 171.0), 'v94 phantom brake', (-105, 19)),
    )
    for point, label, offset in labels:
        ax.annotate(label, xy=point, xytext=offset, textcoords='offset points',
                    fontsize=8, color='#17212b')
    ax.scatter(74.8, 534.3, color='#6f42c1', s=25, zorder=5)
    ax.scatter(152.2, 53.5, color='#6f42c1', s=25, zorder=5)
    ax.scatter(201.9, 171.0, color='#111827', s=25, zorder=5)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title(f'K-City v{args.map_version}: vehicle-only planner prediction corridor')
    ax.set_xlabel('map x (m)')
    ax.set_ylabel('map y (m)')
    ax.grid(alpha=.12)
    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180)
    print(args.output)


if __name__ == '__main__':
    main()
