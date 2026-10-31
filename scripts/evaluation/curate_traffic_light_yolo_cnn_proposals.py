#!/usr/bin/env python3
"""Record an explicitly completed root visual review, with exact pixel rechecks."""
import argparse
from collections import Counter
import copy
import hashlib
import json
from pathlib import Path

from traffic_light_yolo_dataset import digest
from train_traffic_light_crop_probe import load_dataset
from train_traffic_light_cnn import quality_gate


def approve(original,proposed,qc_rows,evidence,*,root_visual_review_confirmed=False):
    if root_visual_review_confirmed is not True:raise ValueError("Explicit completed root visual review required")
    count=len(original["records"])
    if proposed["records"][:count]!=original["records"]:raise ValueError("Original approved records changed")
    expected=set(range(count,len(proposed["records"])))
    if set(qc_rows)!=expected:raise ValueError("QC index must cover every new proposal exactly")
    result=copy.deepcopy(proposed)
    for index in sorted(expected):
        row=result["records"][index];qc=qc_rows[index];proposal=row["provenance"]["yolo_crop_proposal"]
        source_index=proposal["source_record_index"];source=original["records"][source_index]
        if row["split"] not in ("train","val") or row.get("native_gt_verified") is not True:
            raise ValueError("Only train/val native positives can receive this review")
        if row.get("manual_qc_accepted") is not False:raise ValueError("Expected unapproved new proposal")
        for key in ("image_sha256","image_path","labels","split","group_id","environment","native_gt_verified"):
            if row[key]!=source[key]:raise ValueError("Proposal changed original native labels/identity/split")
        if qc["image_path"]!=row["image_path"] or qc["crop_xyxy"]!=row["crop_xyxy"]:
            raise ValueError("QC tile/source crop mismatch")
        if qc["pixel_sha256_rgb"]!=proposal["crop_pixel_sha256_rgb"]:
            raise ValueError("QC tile/proposal pixel SHA mismatch")
        row.update(manual_qc_accepted=True,visual_labels_known=True,visual_labels=copy.deepcopy(row["labels"]))
        review={"reviewer_kind":"root_assistant_direct_visual_review_not_independent_human_GT",
                "explicit_root_visual_review_completed":True,"observations":"Root directly viewed all 14 pages / 246 crops; no truncation or other-housing contamination and all existing R/Y/G/left-arrow bits visually agreed",
                "qc_index":evidence["qc_index"],"qc_index_sha256":evidence["qc_index_sha256"],
                "page":qc["sheet"],"page_sha256":evidence["page_sha256"][qc["sheet"]],
                "tile_xyxy":qc["tile_xyxy"],"crop_pixel_sha256_rgb":qc["pixel_sha256_rgb"],
                "source_JPEG_sha256_reverified":True,"RGB_crop_pixels_reverified":True,
                "CNN_predictions_seen_or_used":False,"independent_human_GT_certified":False}
        row["curation_provenance"]={"method":"explicit_completed_root_visual_review_of_YOLO_crop_proposal",**review}
        proposal["completed_visual_review"]=copy.deepcopy(review)
    result.update(scope="research_approved_original_and_YOLO_crop_mix_not_production_or_independent_human_GT",
                  manual_visibility_and_crop_qc_required=True,manual_visibility_and_crop_qc_performed=True)
    result["yolo_crop_curation"]={**evidence,"original_record_count":count,"approved_new_crop_count":len(expected),
        "original_records_values_unchanged":True,"test_records_added":0,"CNN_predictions_used":False,
        "independent_human_GT_certified":False}
    return result


def run(original,proposals,qc_index,output,*,root_visual_review_confirmed=False):
    original,proposals,qc_index=(Path(p).resolve(strict=True) for p in (original,proposals,qc_index))
    output=Path(output)
    if output.exists() or output.is_symlink():raise FileExistsError("Fresh curated dataset required")
    if output.resolve().parent!=original.parent or proposals.parent!=original.parent:
        raise ValueError("Original/proposals/output must share their unchanged artifact root")
    originals=json.loads(original.read_text());proposed=json.loads(proposals.read_text());qc=json.loads(qc_index.read_text())
    if qc["source_sha256"]!=digest(proposals):raise ValueError("QC index describes another proposal file")
    by_index={row["record_index"]:row for row in qc["proposal_rows"]}
    if len(by_index)!=len(qc["proposal_rows"]):raise ValueError("Duplicate QC record index")
    from PIL import Image
    page_sha={}
    for index,item in by_index.items():
        row=proposed["records"][index];image=original.parent/row["image_path"]
        if digest(image)!=row["image_sha256"]:raise ValueError("Original JPEG SHA mismatch")
        with Image.open(image) as source:
            actual=hashlib.sha256(source.convert("RGB").crop(row["crop_xyxy"]).tobytes()).hexdigest()
        if actual!=item["pixel_sha256_rgb"]:raise ValueError("Decoded crop pixels changed since visual review")
        page=qc_index.parent/item["sheet"]
        page_sha[item["sheet"]]=digest(page)
    evidence={"original_path":str(original),"original_sha256":digest(original),
        "proposal_path":str(proposals),"proposal_sha256":digest(proposals),
        "qc_index":str(qc_index),"qc_index_sha256":digest(qc_index),"page_sha256":page_sha,
        "script_sha256":digest(__file__),"explicit_root_visual_review_confirmed":root_visual_review_confirmed}
    result=approve(originals,proposed,by_index,evidence,root_visual_review_confirmed=root_visual_review_confirmed)
    with output.open("x") as stream:json.dump(result,stream,indent=2,ensure_ascii=False,allow_nan=False)
    rows,metadata=load_dataset(output,allow_manual_background=True);gate=quality_gate(rows)
    return {"output":str(output),"sha256":digest(output),"record_count":len(rows),
        "splits":dict(Counter(row["split"] for row in rows)),"quality_gate":gate,
        "new_approved":len(by_index),"QC_page_count":len(page_sha),"training_started":False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("original","proposals","qc-index","output"):parser.add_argument("--"+name,type=Path,required=True)
    parser.add_argument("--root-visual-review-confirmed",action="store_true")
    print(json.dumps(run(**vars(parser.parse_args())),ensure_ascii=False))


if __name__=="__main__":main()
