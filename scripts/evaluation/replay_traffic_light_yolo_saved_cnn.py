#!/usr/bin/env python3
"""CPU CNN-only replay on frozen saved ROIs; exposed development regression only."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

from traffic_light_yolo_dataset import digest
from evaluate_traffic_light_pipeline import ROOT,PACKAGE,bits_from_tuples


def summary(rows):
    known=[row for row in rows if row["v5_known"]]
    result={"n":len(rows),"known":len(known),"UNKNOWN":len(rows)-len(known),
        "exact_correct_UNKNOWN_is_failure":sum(row["v5_known"] and row["v5_bits"]==row["truth_bits"] for row in rows),
        "v4_saved_exact_correct_UNKNOWN_is_failure":sum(row["v4_known"] and row["v4_bits"]==row["truth_bits"] for row in rows),
        "by_native_state":{}}
    for state in sorted({row["native_state"] for row in rows}):
        selected=[row for row in rows if row["native_state"]==state]
        result["by_native_state"][str(state)]={"n":len(selected),"known":sum(row["v5_known"] for row in selected),
            "UNKNOWN":sum(not row["v5_known"] for row in selected),
            "correct":sum(row["v5_known"] and row["v5_bits"]==row["truth_bits"] for row in selected),
            "p_left_min":min(row["v5_probabilities"]["left_arrow"] for row in selected),
            "p_left_max":max(row["v5_probabilities"]["left_arrow"] for row in selected)}
    return result


def run(pipeline,live_audit,checkpoint,output):
    pipeline,live_audit,checkpoint=(Path(p).resolve(strict=True) for p in (pipeline,live_audit,checkpoint));output=Path(output)
    if output.exists() or output.is_symlink():raise FileExistsError("Fresh replay output required")
    import cv2
    import torch
    torch.set_num_threads(4);cv2.setNumThreads(1)
    from traffic_light_cnn_classifier import TrafficLightCnnClassifier
    sha=digest(checkpoint)
    classifier=TrafficLightCnnClassifier(checkpoint,device="cpu",min_bit_confidence=.6,expected_sha256=sha)
    output.mkdir(parents=True,exist_ok=False)
    def classify(path,expected,coordinates,truth,state,v4_known,v4_bits,metadata):
        if digest(path)!=expected:raise ValueError("Original JPEG SHA mismatch")
        image=cv2.imread(str(path));x1,y1,x2,y2=coordinates
        observation=classifier.classify_bgr(image[y1:y2,x1:x2])
        row={**metadata,"image_sha256":expected,"crop_xyxy":coordinates,"truth_bits":truth,"native_state":state,
            "v4_known":v4_known,"v4_bits":v4_bits,"v5_known":observation.known,
            "v5_bits":list(bits_from_tuples(observation.tuples)) if observation.known else None,
            "v5_probabilities":dict(zip(("red","yellow","green","left_arrow"),observation.probabilities)),
            "v5_confidence":observation.confidence,"v5_reason":observation.reason}
        if digest(path)!=expected:raise ValueError("JPEG changed during replay")
        return row
    audit=json.loads(live_audit.read_text());capture=next(Path(key) for key in audit["sources"] if key.endswith("capture/manifest.json"))
    live=[]
    for row in audit["stable_records"]:
        d=row["debug"];bits=[int(bool(row["native_state"]&mask)) for mask in (1,4,16,32)]
        oldbits=list(bits_from_tuples(d["tuples"])) if d["known"] else None
        live.append(classify(capture.parent/row["image_path"],row["image_sha256"],d["crop_xyxy"],bits,row["native_state"],
            d["known"],oldbits,{"image_path":row["image_path"],"capture_index":row["capture_index"],"source_stamp_ns":row["source_stamp_ns"]}))
    live_report={"scope":"already_exposed_live_v3_exact_stamp_stable_native_development_regression",
                 "checkpoint_sha256":sha,"summary":summary(live),"records":live,"YOLO_forward_called":False,"device":"cpu"}
    (output/"v3_regression.json").write_text(json.dumps(live_report,indent=2))
    print(json.dumps({"v3_complete":live_report["summary"]}),flush=True)
    source=json.loads(pipeline.read_text());manifest=(ROOT/source["parameters"]["dataset"]).resolve(strict=True)
    continuous=[]
    for row in source["records"]:
        mode=row["map_association"];reference=mode["reference"]
        if mode.get("target_identity_correct") is not True or reference.get("known") is not True:continue
        old=mode["classifiers"]["cnn"]
        continuous.append(classify(manifest.parent/row["image_path"],row["image_sha256"],old["crop_xyxy"],reference["bits"],reference["state"],
            old["known"],old["bits"],{"image_path":row["image_path"],"environment":row["environment"],"pose_group":row["pose_group"]}))
    report={"research_only":True,"not_production":True,"scope":"already_exposed_development_regression_not_independent_test",
        "checkpoint_sha256":sha,"device":"cpu","CPU_threads_process_local":4,"CUDA_forward_called":False,
        "YOLO_forward_called":False,"selection_or_threshold_changed":False,"adapter_min_bit_confidence":.6,
        "sources":{str(p):digest(p) for p in (pipeline,live_audit,checkpoint)},"summary":summary(continuous),
        "by_pose_group":{group:summary([r for r in continuous if r["pose_group"]==group]) for group in sorted({r["pose_group"] for r in continuous})},
        "records":continuous,"limits":["Same saved ROI and original JPEG, no target reselection or YOLO rerun",
            "V4 saved GPU output vs V5 CPU replay has a runtime/hardware confound; practical regression, not perfectly isolated causal ablation",
            "UNKNOWN is never counted correct; native state does not independently certify visible glyph",
            "Data were observed before v5 sampling decision and are not fresh holdout evidence"]}
    (output/"continuous939_regression.json").write_text(json.dumps(report,indent=2))
    print(json.dumps({"continuous_complete":report["summary"],"output":str(output)}),flush=True)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("pipeline","live-audit","checkpoint","output"):parser.add_argument("--"+name,type=Path,required=True)
    run(**vars(parser.parse_args()))


if __name__=="__main__":main()
