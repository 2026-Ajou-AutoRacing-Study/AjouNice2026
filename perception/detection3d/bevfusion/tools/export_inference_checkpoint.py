#!/usr/bin/env python3
"""Remove optimizer/training state from a trusted MMEngine checkpoint."""

import argparse
import hashlib
from pathlib import Path

import torch


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    source = torch.load(args.input, map_location="cpu", weights_only=False)
    if not isinstance(source, dict) or "state_dict" not in source:
        raise ValueError("expected an MMEngine checkpoint containing state_dict")

    output = {
        "meta": source.get("meta", {}),
        "state_dict": source["state_dict"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(output, args.output)
    print("input_bytes={}".format(args.input.stat().st_size))
    print("output_bytes={}".format(args.output.stat().st_size))
    print("state_tensors={}".format(len(output["state_dict"])))
    print("sha256={}".format(sha256(args.output)))


if __name__ == "__main__":
    main()
