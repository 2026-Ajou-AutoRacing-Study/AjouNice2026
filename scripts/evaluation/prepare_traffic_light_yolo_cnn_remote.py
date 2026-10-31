#!/usr/bin/env python3
"""Prepare a fresh offline CNN bundle reusing exact JPEGs from a YOLO export."""
import argparse
import copy
import json
from pathlib import Path
import shutil
import tarfile

from traffic_light_yolo_dataset import digest,load_dataset as load_housings
from train_traffic_light_crop_probe import load_dataset as load_crops
from train_traffic_light_cnn import quality_gate

ROOT=Path(__file__).resolve().parents[2]


def prepare_local_source_pin(source_pin,output):
    """Copy exact prior fit code into a minimal repo-shaped, fresh local tree."""
    source_pin,output=Path(source_pin).resolve(strict=True),Path(output)
    if output.exists() or output.is_symlink():raise FileExistsError("Fresh local source PIN folder required")
    files=[(source_pin/"train_traffic_light_cnn.py","scripts/evaluation/train_traffic_light_cnn.py"),
           (source_pin/"train_traffic_light_crop_probe.py","scripts/evaluation/train_traffic_light_crop_probe.py"),
           (ROOT/"base_ws/src/perception/traffic_light_detection/scripts/light_color_utils.py",
            "base_ws/src/perception/traffic_light_detection/scripts/light_color_utils.py")]
    output.mkdir(parents=True,exist_ok=False);rows=[]
    for source,relative in files:
        target=output/"repo"/relative;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(source,target)
        if digest(target)!=digest(source):raise ValueError("Local source PIN copy SHA mismatch")
        rows.append({"source":str(source),"copy":str(target.resolve()),"sha256":digest(target)})
    report={"source_files":rows,"training_started":False,"model_loaded":False,
            "source_root_contract":"repo/scripts/evaluation resolves frozen HSV helper under the copied repo root"}
    (output/"source_pin_manifest.json").write_text(json.dumps(report,indent=2))
    return report


def remap_rows(rows,yolo_rows,reference_dir,extra_dir):
    for relative in (reference_dir,extra_dir):
        if Path(relative).is_absolute() or ".." in Path(relative).parts or not relative:
            raise ValueError("Remote reference/extra directories must be safe relative names")
    lookup={row["image_sha256"]:row for row in yolo_rows}
    output,missing=[],{}
    for row in rows:
        new=copy.deepcopy(row);existing=lookup.get(row["image_sha256"])
        if existing:
            if (existing["split"],existing["group_id"])!=(row["split"],row["group_id"]):
                raise ValueError("CNN/YOLO reference split or pose group differs")
            relative=str(Path(reference_dir)/existing["image_path"])
        else:
            relative=str(Path(extra_dir)/(row["image_sha256"]+".jpg"))
            missing[row["image_sha256"]]=row["image_path"]
        new["image_path"]=relative
        new.setdefault("provenance",{})["remote_path_remap"]={"original_image_path":row["image_path"],
            "source_JPEG_sha256":row["image_sha256"],"raw_bytes_reencoded":False,
            "labels_crop_split_and_QC_flags_unchanged":True}
        output.append(new)
    return output,missing


def run(dataset,yolo_manifest,backbone,output,*,reference_dir="yolo_dataset_development_v1",tag="v3",omit_backbone=False):
    dataset,yolo_manifest,backbone=(Path(p).resolve(strict=True) for p in (dataset,yolo_manifest,backbone))
    output=Path(output)
    if output.exists() or output.is_symlink():raise FileExistsError("Remote preparation directory must be fresh")
    if not tag or not all(c.isalnum() or c=="_" for c in tag):raise ValueError("Simple safe bundle tag required")
    crops,crop_meta=load_crops(dataset,allow_manual_background=True);quality_gate(crops)
    references,reference_meta=load_housings(yolo_manifest,development_pilot=True)
    document=json.loads(dataset.read_text())
    extra="yolo_cnn_extra_raw_"+tag;repo="yolo_cnn_repo_"+tag;pretrained="yolo_cnn_pretrained_"+tag
    rows,missing=remap_rows(document["records"],references,reference_dir,extra)
    output.mkdir(parents=True,exist_ok=False)
    for source,relative in ((ROOT/"scripts/evaluation/train_traffic_light_cnn.py","scripts/evaluation/train_traffic_light_cnn.py"),
                            (ROOT/"scripts/evaluation/train_traffic_light_crop_probe.py","scripts/evaluation/train_traffic_light_crop_probe.py"),
                            (ROOT/"base_ws/src/perception/traffic_light_detection/scripts/light_color_utils.py",
                             "base_ws/src/perception/traffic_light_detection/scripts/light_color_utils.py")):
        target=output/repo/relative;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,target)
    if not omit_backbone:
        pretrained_target=output/pretrained/backbone.name;pretrained_target.parent.mkdir();shutil.copyfile(backbone,pretrained_target)
        if digest(pretrained_target)!=digest(backbone):raise ValueError("Pretrained copy SHA mismatch")
    if missing:(output/extra).mkdir()
    for sha,relative in missing.items():
        destination=output/extra/(sha+".jpg");shutil.copyfile(dataset.parent/relative,destination)
        if digest(destination)!=sha:raise ValueError("Original JPEG copy SHA mismatch")
    document["records"]=rows
    document["remote_preparation"]={"source_dataset":crop_meta,"YOLO_reference":reference_meta,
        "reference_dir":reference_dir,"source_script_sha256":digest(__file__),"research_only":True,
        "training_started":False,"reused_JPEG_count":len({r["image_sha256"] for r in rows})-len(missing),
        "extra_JPEG_count":len(missing),"pretrained_sha256":digest(backbone)}
    document["remote_preparation"]["pretrained_included_in_bundle"]=not omit_backbone
    dataset_name="yolo_cnn_dataset_"+tag+".json"
    (output/dataset_name).write_text(json.dumps(document,indent=2,ensure_ascii=False))
    manifest={"research_only":True,"training_started":False,"dataset_filename":dataset_name,
        "reference_dir":reference_dir,"extra_JPEG_count":len(missing),
        "reused_JPEG_count":document["remote_preparation"]["reused_JPEG_count"],
        "files":[{"path":str(p.relative_to(output)),"bytes":p.stat().st_size,"sha256":digest(p)}
                 for p in sorted(output.rglob("*")) if p.is_file()],
        "remote_validation":"Run trainer --validate-only with dataset and explicit local backbone before any training"}
    (output/("yolo_cnn_bundle_manifest_"+tag+".json")).write_text(json.dumps(manifest,indent=2))
    archive=output.with_suffix(".tar.gz")
    if archive.exists() or archive.is_symlink():raise FileExistsError("Fresh archive required")
    with tarfile.open(archive,"x:gz") as tar:
        for item in sorted(output.iterdir()):tar.add(item,arcname=item.name)
    return {"output":str(output),"archive":str(archive),"archive_bytes":archive.stat().st_size,
            "archive_sha256":digest(archive),"extra_JPEG_count":len(missing),
            "reused_JPEG_count":manifest["reused_JPEG_count"],"training_started":False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("dataset","yolo-manifest","backbone","output"):parser.add_argument("--"+name,type=Path,required=True)
    parser.add_argument("--reference-dir",default="yolo_dataset_development_v1");parser.add_argument("--tag",default="v3")
    parser.add_argument("--omit-backbone",action="store_true",help="Use an independently SHA-verified existing remote pretrained file")
    print(json.dumps(run(**vars(parser.parse_args()))))


if __name__=="__main__":main()
