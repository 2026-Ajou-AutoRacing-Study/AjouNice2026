#!/usr/bin/env python3
"""Propose unapproved CNN crops from detector boxes; never predict lamp labels."""
import argparse
from collections import Counter
import copy
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path

from traffic_light_yolo_dataset import digest,load_dataset as load_housings,match_detections
from train_traffic_light_crop_probe import load_dataset as load_crops
from train_traffic_light_cnn import quality_gate
from train_traffic_light_yolo import offline_runtime,ULTRALYTICS_VERSION


def node_crop(box,width,height,margin=12):
    if len(box)!=4 or any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in box):
        raise ValueError("Finite detector xyxy required")
    x1,y1,x2,y2=box
    if not x1<x2 or not y1<y2:raise ValueError("Empty detector box")
    result=[max(0,int(x1)-margin),max(0,int(y1)-margin),min(width,int(x2)+margin),min(height,int(y2)+margin)]
    if result[0]>=result[2] or result[1]>=result[3]:raise ValueError("Empty clipped detector crop")
    return result


def contains(outer,inner):
    return outer[0]<=inner[0] and outer[1]<=inner[1] and outer[2]>=inner[2] and outer[3]>=inner[3]


def overlaps(a,b):
    return min(a[2],b[2])>max(a[0],b[0]) and min(a[3],b[3])>max(a[1],b[1])


def observed_native_id(row):
    provenance=row.get("provenance",{})
    values=[value for value in (provenance.get("native_observed_id"),
            provenance.get("native_gt_evidence",{}).get("observed_native_id")) if value]
    return values[0] if values and len(set(values))==1 else None


def propose_records(original,housing_by_sha,predictions_by_sha,model_metadata):
    """Pure geometry/identity join. All generated rows are deliberately QC-rejected."""
    proposals,skips=[],[]
    existing={(row["image_sha256"],tuple(int(v) for v in row["crop_xyxy"])) for row in original}
    for index,row in enumerate(original):
        def skip(reason):skips.append({"source_record_index":index,"image_sha256":row["image_sha256"],"reason":reason})
        if row["split"] not in ("train","val"):skip("test_not_augmented");continue
        if row.get("native_gt_verified") is not True:skip("not_native_positive");continue
        if not any(row["labels"].values()):skip("native_all_off_not_eligible");continue
        if row.get("manual_qc_accepted") is not True:raise ValueError("Original crop lacks strict manual QC")
        native=observed_native_id(row)
        if native is None:skip("missing_or_conflicting_observed_native_id");continue
        housing=housing_by_sha.get(row["image_sha256"])
        if housing is None:skip("no_tight_housing_annotation");continue
        if (housing["split"],housing["group_id"])!=(row["split"],row["group_id"]):
            raise ValueError("Housing/CNN image SHA has conflicting split or pose group")
        target_indices=[j for j,box in enumerate(housing["boxes"]) if box.get("native_id")==native]
        if len(target_indices)!=1:skip("native_head_missing_or_ambiguous");continue
        predictions=predictions_by_sha.get(row["image_sha256"])
        if predictions is None:skip("no_detector_prediction_for_source");continue
        matching=match_detections(housing["boxes"],predictions,.5)
        target=target_indices[0]
        match=next((m for m in matching["matches"] if m["truth_index"]==target),None)
        if match is None:skip("target_not_matched_at_IoU_0_5");continue
        prediction=predictions[match["prediction_index"]]
        crop=node_crop(prediction["xyxy"],housing["width"],housing["height"])
        if not contains(crop,housing["boxes"][target]["xyxy"]):skip("crop_does_not_contain_entire_QC_housing");continue
        if any(overlaps(crop,box["xyxy"]) for j,box in enumerate(housing["boxes"]) if j!=target):
            skip("crop_overlaps_other_QC_housing");continue
        key=(row["image_sha256"],tuple(crop))
        if key in existing:skip("duplicate_existing_or_proposed_integer_crop");continue
        existing.add(key)
        proposal=copy.deepcopy(row)
        original_qc={key:copy.deepcopy(row.get(key)) for key in
                     ("crop_xyxy","crop_integer_xyxy","manual_qc_accepted","visual_labels_known","visual_labels","curation_provenance")}
        proposal.update(crop_xyxy=crop,crop_integer_xyxy=list(crop),manual_qc_accepted=False,
                        visual_labels_known=False,visual_labels=None,
                        curation_provenance={"method":"unapproved_yolo_crop_proposal_not_visual_QC"})
        proposal.setdefault("provenance",{})["yolo_crop_proposal"]={
            "source_record_index":index,"source_JPEG_sha256":row["image_sha256"],
            "original_approved_crop_and_QC":original_qc,"native_id_joined":native,
            "detector":copy.deepcopy(model_metadata),"detector_bbox_xyxy":list(prediction["xyxy"]),
            "detector_confidence":prediction["confidence"],"matched_IoU":match["iou"],
            "matched_tight_QC_housing":copy.deepcopy(housing["boxes"][target]),
            "margin_px":12,"node_integer_truncation_matched":True,
            "entire_tight_housing_contained":True,"other_QC_housing_overlap":False,
            "collision_check_scope":"all tight boxes within the declared fully annotated front-housing study class",
            "manual_visual_review_required":True,"CNN_predictions_used":False}
        proposals.append(proposal)
    return proposals,skips


def load_predictions(path,summary,weights_sha,imgsz,conf):
    report=json.loads(Path(summary).read_text())
    if report.get("weights_sha256")!=weights_sha:raise ValueError("Prediction summary/checkpoint SHA mismatch")
    if report.get("nms_mode")!="default_one_to_many_NMS":raise ValueError("Only declared default NMS detector proposals are supported")
    if report.get("fixed_conf",1)>conf:raise ValueError("Stored predictions omit required confidence range")
    result={}
    for line in Path(path).read_text().splitlines():
        row=json.loads(line)
        if row.get("imgsz")!=imgsz or row.get("repeat",0)!=0:continue
        sha=row["image_sha256"]
        if sha in result:raise ValueError("Repeated predictions for one source image")
        boxes=[]
        for box in row["predictions"]:
            score=box.get("confidence")
            if isinstance(score,bool) or not isinstance(score,(int,float)) or not math.isfinite(score) or not 0<=score<=1:
                raise ValueError("Invalid detector confidence")
            if box.get("class_id")!=0:raise ValueError("Single housing class only")
            node_crop(box["xyxy"],1280,720)
            if score>=conf:boxes.append(box)
        result[sha]=boxes
    return result


def infer(records,weights,device,imgsz,conf):
    if importlib.metadata.version("ultralytics")!=ULTRALYTICS_VERSION:raise ValueError("Pinned Ultralytics required")
    with offline_runtime():
        import cv2
        from ultralytics import YOLO,settings
        settings.update({key:False for key in ("sync","wandb","mlflow","tensorboard") if key in settings})
        model=YOLO(str(weights))
        if dict(model.names)!={0:"traffic_light"}:raise ValueError("Single housing class only")
        results={}
        for row in records:
            if digest(row["resolved_image_path"])!=row["image_sha256"]:raise ValueError("Original JPEG SHA changed")
            image=cv2.imread(row["resolved_image_path"])
            if image is None:raise ValueError("Original JPEG decode failed")
            result=model.predict(image,imgsz=imgsz,device=device,conf=conf,nms=None,rect=True,quantize=32,
                                 save=False,verbose=False)[0]
            results[row["image_sha256"]]=[{"xyxy":box,"confidence":float(score),"class_id":int(cls)}
                for box,score,cls in zip(result.boxes.xyxy.detach().cpu().tolist(),
                    result.boxes.conf.detach().cpu().tolist(),result.boxes.cls.detach().cpu().tolist())]
        return results


def run(cnn_dataset,housing_dataset,weights,output,*,predictions=None,prediction_summary=None,
        device="cpu",imgsz=960,conf=.25):
    cnn_dataset,housing_dataset,weights=(Path(p).resolve(strict=True) for p in (cnn_dataset,housing_dataset,weights))
    output=Path(output)
    if output.is_symlink():raise FileExistsError("Fresh output must not be a symlink")
    output=output.resolve();sidecar=output.with_name(output.stem+"_summary.json")
    if output.exists() or sidecar.exists() or sidecar.is_symlink():raise FileExistsError("Fresh dataset and summary paths required")
    if output.parent!=cnn_dataset.parent or output.parent!=housing_dataset.parent:
        raise ValueError("CNN/housing/output JSON must share one common artifact root; no path rebasing or raw copies")
    if imgsz not in (640,960,1280) or not .001<=conf<=1:raise ValueError("Explicit supported inference configuration required")
    if bool(predictions)!=bool(prediction_summary):raise ValueError("Stored predictions require matching summary")
    cnn_rows,cnn_meta=load_crops(cnn_dataset,allow_manual_background=True)
    quality_gate(cnn_rows)
    housing_rows,housing_meta=load_housings(housing_dataset,development_pilot=True)
    housing_by_sha={}
    for row in housing_rows:
        if row["image_sha256"] in housing_by_sha:raise ValueError("Ambiguous duplicate housing source SHA")
        housing_by_sha[row["image_sha256"]]=row
    original=json.loads(cnn_dataset.read_text());raw_rows=original["records"]
    sha=digest(weights)
    if predictions:
        predicted=load_predictions(predictions,prediction_summary,sha,imgsz,conf)
    else:
        required={row["image_sha256"] for row in cnn_rows if row["split"] in ("train","val") and row.get("native_gt_verified") is True}
        predicted=infer([row for row in housing_rows if row["image_sha256"] in required],weights,device,imgsz,conf)
    model_meta={"weights_path":str(weights),"weights_sha256":sha,"imgsz":imgsz,"conf":conf,
                "nms_mode":"default_one_to_many_NMS","precision":"FP32","CNN_inference_performed":False}
    proposals,skips=propose_records(raw_rows,housing_by_sha,predicted,model_meta)
    from PIL import Image
    for row in proposals:
        if digest(output.parent/row["image_path"])!=row["image_sha256"]:raise ValueError("Original JPEG changed before pixel hashing")
        with Image.open(output.parent/row["image_path"]) as source:
            roi=source.convert("RGB").crop(row["crop_xyxy"])
            pixel_sha=hashlib.sha256(roi.tobytes()).hexdigest()
        row["provenance"]["yolo_crop_proposal"].update(crop_pixel_sha256_rgb=pixel_sha,
            crop_pixel_sha256_encoding="Pillow decoded RGB row-major bytes; dimensions are crop_xyxy extent; not JPEG recompression")
    if digest(weights)!=sha or digest(cnn_dataset)!=cnn_meta["sha256"] or digest(housing_dataset)!=housing_meta["source_sha256"]:
        raise ValueError("Checkpoint or source dataset changed during proposal generation")
    document=copy.deepcopy(original)
    document.update(scope="research_original_approved_records_plus_UNAPPROVED_YOLO_crop_proposals",
                    manual_visibility_and_crop_qc_required=True,manual_visibility_and_crop_qc_performed=False,
                    records=raw_rows+proposals)
    document["yolo_proposal_provenance"]={"cnn_dataset":cnn_meta,"housing_dataset":housing_meta,
        "detector":model_meta,"script_sha256":digest(__file__),"original_records_unchanged":True,
        "test_augmentation_performed":False,"proposal_count":len(proposals),
        "stored_predictions_sha256":digest(predictions) if predictions else None,
        "stored_prediction_summary_sha256":digest(prediction_summary) if prediction_summary else None}
    report={"research_only":True,"original_record_count":len(raw_rows),"proposal_count":len(proposals),
            "proposal_split_counts":dict(Counter(row["split"] for row in proposals)),
            "skip_counts":dict(Counter(item["reason"] for item in skips)),"skipped":skips,
            "all_new_manual_qc_accepted":False,"CNN_predictions_used":False,
            "training_allowed_before_manual_review":False,"model":model_meta}
    with output.open("x") as stream:json.dump(document,stream,indent=2,ensure_ascii=False,allow_nan=False)
    with sidecar.open("x") as stream:json.dump(report,stream,indent=2,ensure_ascii=False,allow_nan=False)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("cnn-dataset","housing-dataset","weights","output"):parser.add_argument("--"+name,type=Path,required=True)
    parser.add_argument("--predictions",type=Path);parser.add_argument("--prediction-summary",type=Path)
    parser.add_argument("--device",default="cpu");parser.add_argument("--imgsz",type=int,default=960)
    parser.add_argument("--conf",type=float,default=.25)
    print(json.dumps(run(**vars(parser.parse_args()))))


if __name__=="__main__":main()
