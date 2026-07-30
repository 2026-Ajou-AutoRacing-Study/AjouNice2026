#!/usr/bin/env python3
"""Validate preferred/candidate lanelets from a drive-course CSV in Lanelet2."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import lanelet2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--course-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--origin-lat", type=float, default=37.2380038082)
    parser.add_argument("--origin-lon", type=float, default=126.7730819386)
    args = parser.parse_args()

    with args.course_csv.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    preferred = [int(row["lanelet_id"]) for row in rows if row["role"] == "preferred_course"]
    candidates = [int(row["lanelet_id"]) for row in rows if row["role"] == "candidate_corridor"]

    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(args.origin_lat, args.origin_lon))
    lanelet_map = lanelet2.io.load(str(args.map), projector)
    rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    graph = lanelet2.routing.RoutingGraph(lanelet_map, rules)
    lanelets = {int(lanelet.id): lanelet for lanelet in lanelet_map.laneletLayer}

    transitions = []
    for source_id, destination_id in zip(preferred, preferred[1:]):
        source = lanelets.get(source_id)
        destination = lanelets.get(destination_id)
        following = [int(item.id) for item in graph.following(source)] if source is not None else []
        route = graph.shortestPath(source, destination) if source is not None and destination is not None else None
        route_ids = [int(item.id) for item in route] if route is not None else []
        transitions.append(
            {
                "source_lanelet": source_id,
                "destination_lanelet": destination_id,
                "direct_following": destination_id in following,
                "following_ids": following,
                "shortest_path_ids": route_ids,
                "reachable": destination_id in route_ids,
            }
        )

    candidate_rows = []
    for lanelet_id in candidates:
        lanelet = lanelets.get(lanelet_id)
        candidate_rows.append(
            {
                "lanelet_id": lanelet_id,
                "present_in_vehicle_graph": lanelet is not None,
                "following_ids": [int(item.id) for item in graph.following(lanelet)] if lanelet is not None else [],
                "previous_ids": [int(item.id) for item in graph.previous(lanelet)] if lanelet is not None else [],
                "left_id": int(graph.left(lanelet).id) if lanelet is not None and graph.left(lanelet) is not None else None,
                "right_id": int(graph.right(lanelet).id) if lanelet is not None and graph.right(lanelet) is not None else None,
            }
        )

    failed_transitions = [row for row in transitions if not row["reachable"]]
    missing_candidates = [row for row in candidate_rows if not row["present_in_vehicle_graph"]]
    payload = {
        "map": str(args.map),
        "course_csv": str(args.course_csv),
        "preferred_lanelet_rows": len(preferred),
        "candidate_lanelet_rows": len(candidates),
        "transition_count": len(transitions),
        "direct_following_count": sum(row["direct_following"] for row in transitions),
        "reachable_transition_count": sum(row["reachable"] for row in transitions),
        "failed_transitions": failed_transitions,
        "missing_candidate_lanelets": missing_candidates,
        "candidate_rows": candidate_rows,
        "transitions": transitions,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: payload[key] for key in (
        "preferred_lanelet_rows", "candidate_lanelet_rows", "transition_count", "direct_following_count",
        "reachable_transition_count", "failed_transitions", "missing_candidate_lanelets",
    )}, ensure_ascii=False, indent=2))
    return 1 if failed_transitions or missing_candidates else 0


if __name__ == "__main__":
    raise SystemExit(main())
