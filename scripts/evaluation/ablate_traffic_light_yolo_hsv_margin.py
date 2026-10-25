#!/usr/bin/env python3
"""CPU-only HSV margin ablation on frozen, identity-correct pipeline records."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

from traffic_light_yolo_dataset import digest
from train_traffic_light_crop_probe import metric_summary
from build_traffic_light_yolo_cnn_crops import node_crop
from evaluate_traffic_light_pipeline import (classify_crop,bits_from_tuples,SignalPhasePolicy,
    policy_diagnostic,ROOT,PACKAGE)


def eligible(record):
    mode=record.get("map_association",{})
    return mode.get("target_identity_correct") is True and mode.get("reference",{}).get("known") is True


def summarize(rows,prediction_key):
    known=[row for row in rows if row[prediction_key] is not None]
    metric=metric_summary([row["truth_bits"] for row in known],[row[prediction_key] for row in known])
    return {"eligible":len(rows),"known":len(known),"unknown":len(rows)-len(known),
            "exact_correct":metric["exact_set_correct"],
            "exact_accuracy_all_eligible":metric["exact_set_correct"]/len(rows) if rows else None,
            "known_only_metrics":metric}


def run(pipeline,output,*,expected_eligible=939):
    pipeline,output=Path(pipeline).resolve(strict=True),Path(output)
    if output.exists() or output.is_symlink():raise FileExistsError("Fresh ablation JSON required")
    if "torch" in sys.modules:raise RuntimeError("This analysis must not import Torch or a model")
    import cv2
    cv2.setNumThreads(1)  # Process-local CPU setting only, not the live pipeline.
    document=json.loads(pipeline.read_text())
    manifest=(ROOT/document["parameters"]["dataset"]).resolve(strict=True)
    if digest(manifest)!=document["sources"]["dataset_sha256"]:raise ValueError("Source capture manifest SHA changed")
    hsv_source=PACKAGE/"scripts/light_color_utils.py"
    hsv_sha=digest(hsv_source)
    if document["sources"]["initial_source_hashes"].get(str(hsv_source))!=hsv_sha:
        raise ValueError("HSV source is not the pipeline's frozen HSV source")
    policy_path=Path(document["parameters"]["policy"]).resolve(strict=True)
    if digest(policy_path)!=document["sources"]["policy_sha256"]:raise ValueError("Frozen policy YAML changed")
    policy_source=PACKAGE/"scripts/signal_phase_policy.py";policy_sha=digest(policy_source)
    if document["sources"]["initial_source_hashes"].get(str(policy_source))!=policy_sha:
        raise ValueError("Frozen policy implementation changed")
    policy=SignalPhasePolicy.from_yaml(policy_path)
    selected=[row for row in document["records"] if eligible(row)]
    if len(selected)!=expected_eligible:raise ValueError("Identity/native-known eligibility count differs from explicit expectation")
    rows=[];mismatches=[]
    for row in selected:
        mode=row["map_association"];relative=Path(row["image_path"])
        if relative.is_absolute() or ".." in relative.parts:raise ValueError("Unsafe original JPEG path")
        path=(manifest.parent/relative).resolve(strict=True)
        path.relative_to(manifest.parent)
        if digest(path)!=row["image_sha256"]:raise ValueError("Original JPEG SHA mismatch")
        image=cv2.imread(str(path))
        if image is None:raise ValueError("Original JPEG decode failed")
        height,width=image.shape[:2];bbox=mode["selected_bbox_xyxy"]
        item={"image_path":row["image_path"],"image_sha256":row["image_sha256"],"environment":row["environment"],
              "pose_group":row["pose_group"],"group_regulatory_id":row["group_regulatory_id"],
              "truth_bits":mode["reference"]["bits"],"native_id":mode["reference"]["native_id"],
              "native_state":mode["reference"]["state"],"selected_bbox_xyxy":bbox,
              "CNN_saved_margin12_bits":mode["classifiers"]["cnn"].get("bits") if mode["classifiers"]["cnn"].get("known") is True else None}
        for margin in (0,12):
            crop=node_crop(bbox,width,height,margin);x1,y1,x2,y2=crop
            tuples,counts=classify_crop(image[y1:y2,x1:x2]);bits=list(bits_from_tuples(tuples))
            item["HSV_margin%d_bits"%margin]=bits
            item["HSV_margin%d"%margin]={"crop_xyxy":crop,"tuples":tuples,"bits":bits,
                "counts":{key:int(value) for key,value in counts.items()},
                "policy":policy_diagnostic(policy,tuples,True,mode["reference"],row["group_regulatory_id"])}
        saved=mode["classifiers"]["hsv"]
        match=item["HSV_margin0_bits"]==saved.get("bits") and item["HSV_margin0"]["crop_xyxy"]==saved.get("crop_xyxy")
        if not match:mismatches.append({"image_path":row["image_path"],"saved_bits":saved.get("bits"),"recomputed_bits":item["HSV_margin0_bits"]})
        item["saved_margin0_bits_and_coordinates_match"]=match
        item["same_raw_ROI_as_saved_CNN_margin12"]=item["HSV_margin12"]["crop_xyxy"]==mode["classifiers"]["cnn"].get("crop_xyxy")
        if not item["same_raw_ROI_as_saved_CNN_margin12"]:raise ValueError("Margin12 ROI is not the saved CNN ROI")
        if digest(path)!=row["image_sha256"]:raise ValueError("Original JPEG changed during CPU analysis")
        rows.append(item)
    keys=("HSV_margin0_bits","HSV_margin12_bits","CNN_saved_margin12_bits")
    report={"research_only":True,"not_production":True,"scope":"frozen_same_selected_bbox_CPU_HSV_margin_ablation_no_model_inference",
        "source_pipeline":str(pipeline),"source_pipeline_sha256":digest(pipeline),"capture_manifest_sha256":digest(manifest),
        "HSV_source_sha256":hsv_sha,"policy_source_sha256":policy_sha,"policy_YAML_sha256":digest(policy_path),
        "opencv_version":cv2.__version__,"CPU_threads":1,"Torch_imported":False,"GPU_or_model_forward_called":False,
        "runtime_HSV_default_changed":False,"threshold_or_logic_tuned":False,"candidate_reselected":False,
        "total_records":len(document["records"]),"eligible_identity_correct_and_native_known":len(rows),
        "excluded_unknown_or_wrong_identity":len(document["records"])-len(rows),
        "saved_margin0_reproduction":{"all_match":not mismatches,"matched":len(rows)-len(mismatches),"mismatches":mismatches},
        "same_margin12_ROI_coordinates_as_saved_CNN_for_all":all(r["same_raw_ROI_as_saved_CNN_margin12"] for r in rows),
        "raw_primary":{key:summarize(rows,key) for key in keys},
        "by_environment":{env:{key:summarize([r for r in rows if r["environment"]==env],key) for key in keys} for env in sorted({r["environment"] for r in rows})},
        "by_pose_group":{group:{key:summarize([r for r in rows if r["pose_group"]==group],key) for key in keys} for group in sorted({r["pose_group"] for r in rows})},
        "policy_secondary":{str(margin):{"corrected_exact":sum(r["HSV_margin%d"%margin]["policy"].get("corrected_exact",False) for r in rows),
            "eligible":len(rows)} for margin in (0,12)},"records":rows,
        "limitations":["HSV0 vs savedCNN12 conflates classifier and crop context; primary same-ROI comparison is HSV12 vs savedCNN12",
            "Saved CNN predictions are reused, never recomputed; original raw ROI coordinates match, not identical model preprocessing tensors",
            "Reference is stable exact-ID native state plus reviewed target identity, not independent human per-pixel lamp GT",
            "Unknown native transition records remain excluded; no state/arrow inferred from history",
            "Diagnostic ablation only: runtime HSV remains margin0; no threshold, label, policy, weights, or candidate selection change",
            "Policy-corrected metrics are separate from raw bits and do not certify formal traffic-rule safety"]}
    if digest(hsv_source)!=hsv_sha or digest(policy_source)!=policy_sha:raise ValueError("Source changed during ablation")
    with output.open("x") as stream:json.dump(report,stream,indent=2,ensure_ascii=False,allow_nan=False)
    if mismatches:raise RuntimeError("Saved HSV margin0 reproduction failed; report preserved, do not claim clean ablation")
    return {"output":str(output),"eligible":len(rows),"margin0_reproduction_matches":len(rows),
            "raw_exact":{key:report["raw_primary"][key]["exact_correct"] for key in keys},"model_forward_called":False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pipeline",type=Path,required=True);parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--expected-eligible",type=int,default=939)
    print(json.dumps(run(**vars(parser.parse_args()))))


if __name__=="__main__":main()
