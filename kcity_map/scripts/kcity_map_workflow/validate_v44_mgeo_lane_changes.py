#!/usr/bin/env python3
"""Independently validate v44 MGeo-authoritative lane-change boundaries.

Every MGeo link pair which allows a lane change must resolve to the same OSM
boundary way in v44 and that way must be ``line_thin/dashed``.  The only
exception is the explicitly retained physical tollgate separation around
201399, 201400, and 200147.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path
from typing import Dict, List, Mapping, Sequence

from lxml import etree
from pyproj import Transformer

from fix_v43_shared_boundaries_to_v44 import (
    ROOT,
    LaneletInfo,
    TOLLGATE_ALREADY_SHARED_PAIRS,
    TOLLGATE_COMPLEX_PAIRS,
    duplicate_ids,
    map_lanelets_to_gis,
    mgeo_change_pairs,
    read_gis_a2,
    reference_errors,
    tags,
)


def road_lanelets(root: etree._Element) -> Dict[str, LaneletInfo]:
    result: Dict[str, LaneletInfo] = {}
    for relation in root.findall("relation"):
        relation_tags = tags(relation)
        if relation_tags.get("type") != "lanelet" or relation_tags.get("subtype") != "road":
            continue
        members = {
            member.get("role"): member.get("ref")
            for member in relation.findall("member")
            if member.get("type") == "way" and member.get("role") in {"left", "right", "centerline"}
        }
        if {"left", "right", "centerline"} <= set(members):
            result[relation.get("id")] = LaneletInfo(relation.get("id"), members)
    return result


def boundary_status(
    lanelet_a: LaneletInfo,
    lanelet_b: LaneletInfo,
    source_role: str,
    ways: Mapping[str, etree._Element],
) -> tuple[bool, str, str, str]:
    role_b = "right" if source_role == "left" else "left"
    way_a = lanelet_a.members[source_role]
    way_b = lanelet_b.members[role_b]
    shared = way_a == way_b
    way_tags = tags(ways[way_a]) if shared else {}
    dashed = way_tags.get("type") == "line_thin" and way_tags.get("subtype") == "dashed"
    return shared and dashed, way_a, way_b, f"type={way_tags.get('type', '')};subtype={way_tags.get('subtype', '')}"


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "mgeo_link_a", "mgeo_link_b", "source_side", "lanelet_a", "lanelet_b",
        "checked_lanelet_pairs", "status", "shared_way_a", "shared_way_b", "way_tags", "detail",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=Path, default=ROOT / "lanelet_map/lanelet2_map_v44.osm")
    parser.add_argument(
        "--report-dir", type=Path, default=ROOT / "reports"
    )
    args = parser.parse_args()

    root = etree.parse(str(args.map)).getroot()
    ways = {way.get("id"): way for way in root.findall("way")}
    nodes = {node.get("id"): node for node in root.findall("node")}
    lanelets = road_lanelets(root)
    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
    gis_to_lanelets = map_lanelets_to_gis(
        lanelets,
        ways,
        nodes,
        read_gis_a2(ROOT / "gis_map/A2_LINK.shp"),
        to_utm,
    )
    changes = mgeo_change_pairs(ROOT / "mgeo_map/link_set.json")

    rows: List[Dict[str, object]] = []
    failures: List[Dict[str, object]] = []
    for pair_key, (source_link, source_role, destination_link) in sorted(changes.items()):
        source_lanelets = gis_to_lanelets.get(source_link, [])
        destination_lanelets = gis_to_lanelets.get(destination_link, [])
        row: Dict[str, object] = {
            "mgeo_link_a": source_link,
            "mgeo_link_b": destination_link,
            "source_side": source_role,
            "lanelet_a": "|".join(source_lanelets),
            "lanelet_b": "|".join(destination_lanelets),
            "checked_lanelet_pairs": "",
            "status": "",
            "shared_way_a": "",
            "shared_way_b": "",
            "way_tags": "",
            "detail": "",
        }

        if pair_key in TOLLGATE_COMPLEX_PAIRS:
            # The exception is physical, not merely a tag exception.
            preserved_pairs = [("201399", "201400"), ("200147", "201400")]
            checks = []
            for left_id, right_id in preserved_pairs:
                if left_id not in lanelets or right_id not in lanelets:
                    checks.append(False)
                    continue
                left_bounds = set(lanelets[left_id].members.values())
                right_bounds = set(lanelets[right_id].members.values())
                checks.append(not bool(left_bounds & right_bounds))
            row.update(
                checked_lanelet_pairs="|".join(f"{a}<>{b}" for a, b in preserved_pairs),
                status="tollgate_gap_preserved" if all(checks) else "FAIL_tollgate_gap_bridged",
                detail="201399/201400 and 200147/201400 remain physically separate.",
            )
        elif pair_key in TOLLGATE_ALREADY_SHARED_PAIRS:
            section_results = []
            section_descriptions = []
            for lanelet_a_id, lanelet_b_id in zip(source_lanelets, destination_lanelets):
                passed, way_a, way_b, way_tags = boundary_status(
                    lanelets[lanelet_a_id], lanelets[lanelet_b_id], source_role, ways
                )
                section_results.append(passed)
                section_descriptions.append(f"{lanelet_a_id}<->{lanelet_b_id}")
                row["shared_way_a"] = f"{row['shared_way_a']}|{way_a}".strip("|")
                row["shared_way_b"] = f"{row['shared_way_b']}|{way_b}".strip("|")
                row["way_tags"] = f"{row['way_tags']}|{way_tags}".strip("|")
            row.update(
                checked_lanelet_pairs="|".join(section_descriptions),
                status="PASS" if section_results and all(section_results) else "FAIL_shared_dashed",
                detail="All longitudinal sections of the split tollgate source links are shared dashed boundaries.",
            )
        elif len(source_lanelets) == 1 and len(destination_lanelets) == 1:
            lanelet_a = lanelets[source_lanelets[0]]
            lanelet_b = lanelets[destination_lanelets[0]]
            passed, way_a, way_b, way_tags = boundary_status(lanelet_a, lanelet_b, source_role, ways)
            row.update(
                checked_lanelet_pairs=f"{lanelet_a.lanelet_id}<->{lanelet_b.lanelet_id}",
                status="PASS" if passed else "FAIL_shared_dashed",
                shared_way_a=way_a,
                shared_way_b=way_b,
                way_tags=way_tags,
                detail="MGeo-authoritative lane-change boundary is one common dashed way.",
            )
        else:
            row.update(status="FAIL_unmapped", detail="Expected a deterministic source lanelet mapping.")

        rows.append(row)
        if row["status"].startswith("FAIL"):
            failures.append(row)

    meta = root.find("MetaInfo")
    summary = {
        "map": str(args.map.relative_to(ROOT)),
        "map_version": meta.get("map_version") if meta is not None else None,
        "road_lanelets": len(lanelets),
        "mgeo_lane_change_pairs": len(changes),
        "status_counts": dict(sorted(Counter(str(row["status"]) for row in rows).items())),
        "failures": len(failures),
        "failure_pairs": [
            f"{row['mgeo_link_a']}<->{row['mgeo_link_b']}" for row in failures
        ],
        "reference_errors": reference_errors(root),
        "duplicate_ids": duplicate_ids(root),
    }
    write_csv(args.report_dir / "v44_mgeo_lane_change_static_validation.csv", rows)
    (args.report_dir / "v44_mgeo_lane_change_static_validation_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 1 if failures or summary["reference_errors"] or any(summary["duplicate_ids"].values()) else 0


if __name__ == "__main__":
    raise SystemExit(main())
