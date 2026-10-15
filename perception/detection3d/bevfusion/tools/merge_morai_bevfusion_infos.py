#!/usr/bin/env python3
"""Merge compatible MORAI BEVFusion info decks without copying sensor files."""

import argparse
import copy
import json
import pickle
from pathlib import Path


SPLITS = ("train", "val", "test")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        action="append",
        nargs=3,
        metavar=("INFO_DIR", "OLD_SERIALIZED_ROOT", "NEW_SERIALIZED_ROOT"),
        required=True,
        help="repeat for every source deck",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def rewrite_paths(value, old_root, new_root):
    if isinstance(value, str):
        if value == old_root:
            return new_root
        if value.startswith(old_root + "/"):
            return new_root + value[len(old_root) :]
        return value
    if isinstance(value, list):
        return [rewrite_paths(item, old_root, new_root) for item in value]
    if isinstance(value, tuple):
        return tuple(rewrite_paths(item, old_root, new_root) for item in value)
    if isinstance(value, dict):
        return {
            key: rewrite_paths(item, old_root, new_root)
            for key, item in value.items()
        }
    return value


def load_info(path):
    with path.open("rb") as stream:
        payload = pickle.load(stream)
    if not isinstance(payload, dict) or "data_list" not in payload or "metainfo" not in payload:
        raise ValueError("invalid MMDetection3D info deck: {}".format(path))
    return payload


def main():
    args = parse_args()
    if args.output_dir.exists():
        raise FileExistsError("refusing to overwrite {}".format(args.output_dir))

    sources = []
    for info_dir, old_root, new_root in args.source:
        root = Path(info_dir).expanduser().resolve()
        if not root.is_dir():
            raise FileNotFoundError(root)
        sources.append((root, old_root.rstrip("/"), new_root.rstrip("/")))

    args.output_dir.mkdir(parents=True)
    summary = {"sources": [], "splits": {}, "duplicate_sample_ids": []}
    reference_metainfo = None

    for split in SPLITS:
        combined = []
        seen = set()
        split_sources = []
        for root, old_root, new_root in sources:
            path = root / "morai_base3_infos_{}.pkl".format(split)
            payload = load_info(path)
            metainfo = payload["metainfo"]
            if reference_metainfo is None:
                reference_metainfo = copy.deepcopy(metainfo)
            elif metainfo != reference_metainfo:
                raise ValueError("metainfo mismatch in {}".format(path))

            rewritten = rewrite_paths(payload["data_list"], old_root, new_root)
            for sample in rewritten:
                sample_id = str(sample.get("sample_idx") or sample.get("token"))
                if sample_id in seen:
                    summary["duplicate_sample_ids"].append(sample_id)
                    raise ValueError("duplicate sample id: {}".format(sample_id))
                seen.add(sample_id)
                combined.append(sample)
            split_sources.append({"info_dir": str(root), "frames": len(rewritten)})

        output = {
            "metainfo": {
                **copy.deepcopy(reference_metainfo),
                "dataset": "morai_base3_online_highway_combined",
            },
            "data_list": combined,
        }
        output_path = args.output_dir / "morai_base3_infos_{}.pkl".format(split)
        with output_path.open("wb") as stream:
            pickle.dump(output, stream, protocol=pickle.HIGHEST_PROTOCOL)
        summary["splits"][split] = {
            "frames": len(combined),
            "sources": split_sources,
        }

    summary["sources"] = [
        {
            "info_dir": str(root),
            "old_serialized_root": old_root,
            "new_serialized_root": new_root,
        }
        for root, old_root, new_root in sources
    ]
    (args.output_dir / "merge_manifest.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
