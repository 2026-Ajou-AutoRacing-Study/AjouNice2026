#!/usr/bin/env python3
"""Ask the live Autoware mission planner to route across every v44 MGeo change.

For each MGeo-permitted adjacent pair, initialize on the source lanelet and
request a goal in the lateral destination lanelet.  A pass requires an AD API
route, a non-empty trajectory, and the requested destination in the returned
LaneletRoute.  The static tollgate exceptions are intentionally excluded.
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List

import rclpy

from autoware_route_probe import RouteProbe, load_lanelets, route_from_path


def parse_pairs(value: str) -> Iterable[tuple[int, int]]:
    for part in value.split("|"):
        source, destination = part.split("<->")
        yield int(source), int(destination)


def write_json(path: Path, payload: Dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, type=Path)
    parser.add_argument("--audit-csv", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--origin-lat", type=float, default=37.2380038082)
    parser.add_argument("--origin-lon", type=float, default=126.7730819386)
    parser.add_argument("--service-timeout", type=float, default=20.0)
    parser.add_argument("--route-timeout", type=float, default=20.0)
    parser.add_argument("--trajectory-timeout", type=float, default=20.0)
    parser.add_argument("--no-require-trajectory", dest="require_trajectory", action="store_false")
    parser.set_defaults(require_trajectory=True)
    parser.add_argument("--max-tests", type=int, default=0, help="0 means every permitted pair")
    parser.add_argument("--start-index", type=int, default=1, help="1-based permitted-pair index for chunked execution")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    _, _, lanelets, _ = load_lanelets(str(args.map), args.origin_lat, args.origin_lon)
    cases: List[Dict[str, object]] = []
    with args.audit_csv.open(newline="", encoding="utf-8") as handle:
        for audit in csv.DictReader(handle):
            if audit["status"] != "PASS":
                continue
            for source_id, destination_id in parse_pairs(audit["checked_lanelet_pairs"]):
                if source_id not in lanelets or destination_id not in lanelets:
                    raise RuntimeError(f"Missing lanelet in v44: {source_id}->{destination_id}")
                cases.append(
                    {
                        "mgeo_link_a": audit["mgeo_link_a"],
                        "mgeo_link_b": audit["mgeo_link_b"],
                        "source_side": audit["source_side"],
                        "source_lanelet": source_id,
                        "destination_lanelet": destination_id,
                    }
                )
    if args.start_index < 1:
        raise RuntimeError("--start-index must be >= 1")
    cases = cases[args.start_index - 1:]
    if args.max_tests > 0:
        cases = cases[:args.max_tests]

    payload: Dict[str, object] = {
        "map": str(args.map),
        "audit_csv": str(args.audit_csv),
        "planned_lane_change_requests": len(cases),
        "results": [],
        "started_at_unix": time.time(),
    }
    write_json(args.output, payload)
    rclpy.init()
    node = RouteProbe()
    unexpected_error = None
    try:
        node.wait_for_services(args.service_timeout)
        for index, case in enumerate(cases, 1):
            source_id = int(case["source_lanelet"])
            destination_id = int(case["destination_lanelet"])
            spec = route_from_path(
                f"lane_change_{index:03d}_{source_id}_to_{destination_id}",
                [source_id, destination_id],
                lanelets,
                required_lanelet=destination_id,
            )
            result = node.run_route(index, spec, args)
            result.update(case)
            payload["results"].append(result)
            counts = Counter("PASS" if item["pass"] else "FAIL" for item in payload["results"])
            payload["summary"] = {
                "executed": len(payload["results"]),
                "passed": counts["PASS"],
                "failed": counts["FAIL"],
                "failed_pairs": [
                    f"{item['source_lanelet']}->{item['destination_lanelet']}"
                    for item in payload["results"]
                    if not item["pass"]
                ],
            }
            write_json(args.output, payload)
            print(
                f"[{index}/{len(cases)}] {source_id}->{destination_id} pass={result['pass']} "
                f"route_ids={result['route_id_count']} trajectory={result['trajectory_points']}",
                flush=True,
            )
    except Exception as exc:
        unexpected_error = f"{type(exc).__name__}: {exc}"
        payload["error"] = unexpected_error
    finally:
        payload["finished_at_unix"] = time.time()
        write_json(args.output, payload)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if unexpected_error is None and all(item["pass"] for item in payload["results"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
