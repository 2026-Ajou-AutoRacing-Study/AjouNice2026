#!/usr/bin/env python3
"""Merge approved same-root housing records and package byte-exact JPEG deltas."""
import argparse
import copy
import json
from pathlib import Path
import tarfile

from traffic_light_yolo_dataset import digest,load_dataset,export_dataset


def merge(sources,output):
    sources=[Path(p).resolve(strict=True) for p in sources];output=Path(output)
    if output.exists() or output.is_symlink():raise FileExistsError("Fresh merged JSON required")
    if any(path.parent!=output.resolve().parent for path in sources):raise ValueError("One unchanged artifact root required")
    records,metadata=[],[]
    for source in sources:
        rows,source_meta=load_dataset(source,development_pilot=True)
        metadata.append(source_meta);records.extend(copy.deepcopy(json.loads(source.read_text())["records"]))
    document={"schema_version":1,"research_only":True,"not_production":True,
        "scope":"visually_QC_approved_development_housing_sources_not_fresh_unseen_test",
        "class_names":{"0":"traffic_light"},"source_datasets":metadata,
        "split_caveat":"Original pose-group splits are unchanged; observed development test is not locked fresh sequence",
        "source_records_values_unchanged":True,"script_sha256":digest(__file__),"records":records}
    # Fail closed before writing if sources contain duplicate paths or split leakage.
    paths=set();groups={};hashes={}
    for row in records:
        if row["image_path"] in paths:raise ValueError("Duplicate original image path across source datasets")
        paths.add(row["image_path"])
        for mapping,key in ((groups,row["group_id"]),(hashes,row["image_sha256"])):
            if key in mapping and mapping[key]!=row["split"]:raise ValueError("SHA or group crosses split in merged sources")
            mapping[key]=row["split"]
    with output.open("x") as stream:json.dump(document,stream,indent=2,ensure_ascii=False,allow_nan=False)
    _,verified=load_dataset(output,development_pilot=True)
    return verified


def prepare_delta(export_dir,reference_manifest,reference_dir,archive,extra_dir=None,extra_remote_dir=None):
    export_dir,reference_manifest=Path(export_dir).resolve(strict=True),Path(reference_manifest).resolve(strict=True)
    archive=Path(archive)
    if archive.exists() or archive.is_symlink():raise FileExistsError("Fresh delta archive required")
    for relative in (reference_dir,extra_remote_dir):
        if relative is not None and (not relative or Path(relative).is_absolute() or ".." in Path(relative).parts):
            raise ValueError("Safe relative remote reference directories required")
    current,_=load_dataset(export_dir/"manifest.json",development_pilot=True)
    references,_=load_dataset(reference_manifest,development_pilot=True)
    lookup={row["image_sha256"]:{"source":str(Path(reference_dir)/row["image_path"]),
             "split":row["split"],"group_id":row["group_id"]} for row in references}
    if bool(extra_dir)!=bool(extra_remote_dir):raise ValueError("Extra local and remote reference dirs must be paired")
    if extra_dir:
        for path in Path(extra_dir).glob("*.jpg"):
            sha=digest(path)
            if path.stem!=sha:raise ValueError("Extra reference JPEG basename must equal its SHA")
            lookup.setdefault(sha,{"source":str(Path(extra_remote_dir)/path.name)})
    copies=[];reused=set()
    for row in current:
        old=lookup.get(row["image_sha256"])
        if not old:continue
        if "split" in old and (old["split"],old["group_id"])!=(row["split"],row["group_id"]):
            raise ValueError("Remote reference split/group mismatch")
        reused.add(row["image_path"])
        copies.append({"source":old["source"],"target":str(Path(export_dir.name)/row["image_path"]),
                       "sha256":row["image_sha256"]})
    plan={"research_only":True,"dataset_dir":export_dir.name,"copies_of_unchanged_original_JPEG":copies,
          "source_manifest_sha256":digest(export_dir/"manifest.json"),"reused_count":len(copies),
          "transferred_new_JPEG_count":len(current)-len(copies),"remote_source_images_never_changed":True}
    with (export_dir/"delta_reuse_plan.json").open("x") as stream:json.dump(plan,stream,indent=2)
    with tarfile.open(archive,"x:gz") as tar:
        for path in sorted(export_dir.rglob("*")):
            if path.is_file() and str(path.relative_to(export_dir)) not in reused:
                tar.add(path,arcname=str(Path(export_dir.name)/path.relative_to(export_dir)))
    return {"archive":str(archive),"bytes":archive.stat().st_size,"sha256":digest(archive),
            "reused_JPEG_count":len(copies),"new_JPEG_count":len(current)-len(copies)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source",type=Path,action="append",required=True)
    parser.add_argument("--output",type=Path,required=True);parser.add_argument("--export-dir",type=Path,required=True)
    parser.add_argument("--reference-manifest",type=Path,required=True);parser.add_argument("--reference-dir",required=True)
    parser.add_argument("--extra-dir",type=Path);parser.add_argument("--extra-remote-dir")
    parser.add_argument("--archive",type=Path,required=True)
    args=vars(parser.parse_args());metadata=merge(args.pop("source"),args.pop("output"))
    export_dataset(metadata["source_path"],args["export_dir"],development_pilot=True)
    print(json.dumps({"dataset":metadata,"transfer":prepare_delta(**args)},ensure_ascii=False))


if __name__=="__main__":main()
