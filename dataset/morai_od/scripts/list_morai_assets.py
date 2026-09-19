#!/usr/bin/env python3
"""List MORAI asset DataIDs and exact Scenario Edit names without jq."""

import argparse
import json
from pathlib import Path
import re


MORAI_OD_ROOT = Path(__file__).resolve().parents[1]
CATALOGS = {
    "competition": MORAI_OD_ROOT / "configs/morai_asset_catalog_260807.json",
    "personal": MORAI_OD_ROOT / "configs/morai_asset_catalog_personal_260807.json",
}
CATEGORIES = (
    ("vehicle", "vehicles"),
    ("pedestrian_container", "pedestrians_and_related"),
    ("object_container", "obstacles_and_related"),
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--catalog", choices=sorted(CATALOGS), default="competition"
    )
    parser.add_argument(
        "--query",
        help="Case-insensitive regular expression matched against asset name.",
    )
    parser.add_argument(
        "--enabled-only",
        action="store_true",
        help="Show only rows whose catalog generation_policy is enabled.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    pattern = re.compile(args.query, re.IGNORECASE) if args.query else None
    with CATALOGS[args.catalog].open(encoding="utf-8") as stream:
        catalog = json.load(stream)

    print("container\tdata_id\tname\tontology\tsubtype\tgeneration_policy")
    for container, key in CATEGORIES:
        for row in catalog[key]:
            if pattern and not pattern.search(row["name"]):
                continue
            if args.enabled_only and row["generation_policy"] != "enabled":
                continue
            print(
                "\t".join(
                    str(value)
                    for value in (
                        container,
                        row["data_id"],
                        row["name"],
                        row["ontology_class"],
                        row["subtype"],
                        row["generation_policy"],
                    )
                )
            )


if __name__ == "__main__":
    main()
