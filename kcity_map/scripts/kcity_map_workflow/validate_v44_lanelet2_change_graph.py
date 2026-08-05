#!/usr/bin/env python3
"""Check every permitted v44 MGeo change in the actual Lanelet2 routing graph.

Input is the host-side static audit CSV.  This script runs in the Autoware
container, loads the exact v44 OSM through Lanelet2, and verifies each MGeo
permission appears as mutually reachable ``RoutingGraph.left/right`` relation.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List

import lanelet2


def neighbor_id(graph, lanelet, side: str):
    candidate = graph.left(lanelet) if side == "left" else graph.right(lanelet)
    return int(candidate.id) if candidate is not None else None


def parse_pairs(value: str) -> Iterable[tuple[int, int]]:
    for part in value.split("|"):
        source, destination = part.split("<->")
        yield int(source), int(destination)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--audit-csv", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--origin-lat", type=float, default=37.2380038082)
    parser.add_argument("--origin-lon", type=float, default=126.7730819386)
    args = parser.parse_args()

    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(args.origin_lat, args.origin_lon))
    lanelet_map = lanelet2.io.load(str(args.map), projector)
    rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    graph = lanelet2.routing.RoutingGraph(lanelet_map, rules)
    lanelets: Dict[int, object] = {int(lanelet.id): lanelet for lanelet in lanelet_map.laneletLayer}

    rows: List[Dict[str, object]] = []
    with args.audit_csv.open(newline="", encoding="utf-8") as handle:
        for audit in csv.DictReader(handle):
            if audit["status"] != "PASS":
                continue
            side = audit["source_side"]
            opposite = "right" if side == "left" else "left"
            for source_id, destination_id in parse_pairs(audit["checked_lanelet_pairs"]):
                source = lanelets.get(source_id)
                destination = lanelets.get(destination_id)
                source_neighbor = neighbor_id(graph, source, side) if source is not None else None
                destination_neighbor = neighbor_id(graph, destination, opposite) if destination is not None else None
                passed = source_neighbor == destination_id and destination_neighbor == source_id
                rows.append(
                    {
                        "mgeo_link_a": audit["mgeo_link_a"],
                        "mgeo_link_b": audit["mgeo_link_b"],
                        "source_side": side,
                        "source_lanelet": source_id,
                        "destination_lanelet": destination_id,
                        "graph_source_neighbor": source_neighbor,
                        "graph_destination_neighbor": destination_neighbor,
                        "status": "PASS" if passed else "FAIL",
                    }
                )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        fields = [
            "mgeo_link_a", "mgeo_link_b", "source_side", "source_lanelet", "destination_lanelet",
            "graph_source_neighbor", "graph_destination_neighbor", "status",
        ]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    failures = [row for row in rows if row["status"] != "PASS"]
    summary = {
        "map": str(args.map),
        "graph_checks": len(rows),
        "status_counts": dict(sorted(Counter(row["status"] for row in rows).items())),
        "failures": failures,
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
