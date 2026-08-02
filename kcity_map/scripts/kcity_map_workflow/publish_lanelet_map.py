#!/usr/bin/env python3
"""Publish an Autoware map package's Lanelet2 OSM as the canonical K-City map.

The Autoware package layout requires a generic ``lanelet2_map.osm`` filename,
whereas this repository keeps every source map version under ``lanelet_map``.
This tool makes the latter the required publish target so a newly generated map
is not left only in ``autoware_data/maps``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = ROOT.parent


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def assert_valid_osm(path: Path, version: int) -> None:
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        raise ValueError(f"Invalid Lanelet2 OSM XML: {path}: {exc}") from exc

    if root.tag != "osm":
        raise ValueError(f"Expected an <osm> root element: {path}")

    meta = root.find("MetaInfo")
    map_version = meta.get("map_version") if meta is not None else None
    if map_version is not None and map_version != str(version):
        raise ValueError(
            f"Map version mismatch: expected v{version}, but MetaInfo has v{map_version}: {path}"
        )


def copy_atomically(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{target.name}.", dir=target.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle, source.open("rb") as input_handle:
            shutil.copyfileobj(input_handle, handle)
        shutil.copystat(source, temporary)
        os.replace(temporary, target)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Publish a versioned Lanelet2 map into kcity_map/lanelet_map."
    )
    parser.add_argument("--version", type=int, default=45, help="Map version to publish (default: 45).")
    parser.add_argument(
        "--input",
        type=Path,
        help="Generated Lanelet2 OSM; defaults to autoware_data/maps/kcity-vNN/lanelet2_map.osm.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Canonical versioned OSM; defaults to kcity_map/lanelet_map/lanelet2_map_vNN.osm.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Replace an existing canonical map when its contents differ.",
    )
    args = parser.parse_args()

    source = args.input or (
        WORKSPACE_ROOT / "autoware_data" / "maps" / f"kcity-v{args.version}" / "lanelet2_map.osm"
    )
    target = args.output or (ROOT / "lanelet_map" / f"lanelet2_map_v{args.version}.osm")
    source = source.resolve()
    target = target.resolve()

    if not source.is_file():
        parser.error(f"input map does not exist: {source}")
    assert_valid_osm(source, args.version)
    source_hash = sha256(source)

    if target.exists():
        target_hash = sha256(target)
        if target_hash == source_hash:
            print(json.dumps({"status": "already_published", "map": str(target), "sha256": source_hash}))
            return 0
        if not args.force:
            parser.error(f"canonical map already differs: {target}; rerun with --force to replace it")

    copy_atomically(source, target)
    published_hash = sha256(target)
    if published_hash != source_hash:
        raise RuntimeError(f"Publish verification failed: {target}")

    print(
        json.dumps(
            {
                "status": "published",
                "source": str(source),
                "map": str(target),
                "sha256": published_hash,
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
