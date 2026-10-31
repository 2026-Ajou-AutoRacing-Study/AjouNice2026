#!/usr/bin/env python3
"""Restore an exact checkpoint from lossless SSH transfer caches and pinned SHA."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path


def restore(parts,output,expected_sha256):
    parts,output=Path(parts),Path(output)
    if output.exists() or output.is_symlink():raise FileExistsError("Checkpoint output must be fresh")
    manifest=json.loads((parts/"manifest.json").read_text())
    if manifest["source_sha256"]!=expected_sha256 or manifest.get("lossless") is not True:
        raise ValueError("Transfer manifest does not match the frozen checkpoint")
    chunks=[];seen=set()
    for item in manifest["parts"]:
        name=item["file"]
        if Path(name).name!=name or name in seen:raise ValueError("Unsafe or duplicate part filename")
        seen.add(name);payload=(parts/name).read_bytes()
        if len(payload)!=item["bytes"] or hashlib.sha256(payload).hexdigest()!=item["sha256"]:
            raise ValueError("Partial or changed transfer chunk: "+name)
        chunks.append(payload)
    compressed=b"".join(chunks)
    if len(compressed)!=manifest["gzip_bytes"] or hashlib.sha256(compressed).hexdigest()!=manifest["gzip_sha256"]:
        raise ValueError("Lossless compressed stream SHA mismatch")
    restored=gzip.decompress(compressed)
    if len(restored)!=manifest["source_bytes"] or hashlib.sha256(restored).hexdigest()!=expected_sha256:
        raise ValueError("Restored frozen checkpoint SHA mismatch")
    with output.open("xb") as stream:stream.write(restored)
    return {"output":str(output),"bytes":len(restored),"sha256":expected_sha256,
            "parts":len(chunks),"bit_exact_to_frozen_remote_model":True,"model_modified":False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parts",type=Path,required=True);parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--expected-sha256",required=True)
    print(json.dumps(restore(**vars(parser.parse_args()))))


if __name__=="__main__":main()
