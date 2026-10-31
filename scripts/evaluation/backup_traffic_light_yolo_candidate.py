#!/usr/bin/env python3
"""Prepare lossless, hash-pinned final-candidate backup chunks; no network/model."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile

from traffic_light_yolo_dataset import digest


def prepare(files,output,chunk_bytes=4*1024*1024):
    output=Path(output)
    if output.exists() or output.is_symlink():raise FileExistsError("Fresh backup staging directory required")
    sources=[Path(p).resolve(strict=True) for p in files]
    if len({p.name for p in sources})!=len(sources):raise ValueError("Backup basenames must be unique")
    before={str(p):digest(p) for p in sources}
    output.mkdir(parents=True,exist_ok=False);payload=output/"payload";payload.mkdir()
    manifest={"research_only":True,"lossless":True,"files":[]}
    for source in sources:
        target=payload/source.name;shutil.copyfile(source,target)
        if digest(target)!=before[str(source)]:raise ValueError("Backup copy SHA mismatch")
        manifest["files"].append({"name":source.name,"source_path":str(source),"bytes":source.stat().st_size,
                                  "sha256":before[str(source)]})
    (payload/"backup_manifest.json").write_text(json.dumps(manifest,indent=2))
    archive=output/"backup_payload.tar.gz"
    with tarfile.open(archive,"x:gz") as tar:
        for path in sorted(payload.iterdir()):tar.add(path,arcname=path.name)
    parts=output/"parts";parts.mkdir();items=[]
    with archive.open("rb") as source:
        index=0
        while True:
            data=source.read(chunk_bytes)
            if not data:break
            name="part_%03d.tar.gzpart"%index;(parts/name).write_bytes(data)
            items.append({"file":name,"bytes":len(data),"sha256":hashlib.sha256(data).hexdigest()});index+=1
    transfer={"archive_bytes":archive.stat().st_size,"archive_sha256":digest(archive),"parts":items,
              "backup_manifest_sha256":digest(payload/"backup_manifest.json"),"lossless":True}
    (parts/"transfer_manifest.json").write_text(json.dumps(transfer,indent=2))
    if any(digest(p)!=before[str(p)] for p in sources):raise ValueError("Source changed during backup preparation")
    return {"staging":str(output),"archive_bytes":transfer["archive_bytes"],
            "archive_sha256":transfer["archive_sha256"],"parts":len(items),"source_files_unchanged":True}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file",type=Path,action="append",required=True);parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args();print(json.dumps(prepare(args.file,args.output)))


if __name__=="__main__":main()
