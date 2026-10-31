#!/usr/bin/env python3
"""Compare saved checkpoints on validation only; never inspect test predictions."""
import argparse
import json
from pathlib import Path

from evaluate_traffic_light_yolo import run as evaluate
from traffic_light_yolo_dataset import digest


def selection_key(candidate):
    """Predeclared lexicographic operating-point policy, not test tuning."""
    detection=candidate["operating_point"]
    mission=detection["GT_box_bins"].get("mission:allowed5",{})
    recall=mission.get("recall")
    tp,fp,fn=(detection[key] for key in ("tp","fp","fn"))
    f1=2*tp/(2*tp+fp+fn) if 2*tp+fp+fn else 0.
    return (recall if recall is not None else -1.,f1,-fp,candidate["official_mAP"]["mAP50_95"])


def run(dataset,weights_dir,output,*,device="0",imgsz=960,conf=.25,development_pilot=False):
    output=Path(output).resolve()
    if output.exists() or output.is_symlink():raise FileExistsError("Candidate selection output must be fresh")
    if conf<.001 or conf>1:raise ValueError("Operating confidence must be in [.001,1]")
    weights_dir=Path(weights_dir).resolve(strict=True)
    weights=[p for p in weights_dir.glob("*.pt") if p.name in ("best.pt","last.pt") or
             (p.stem.startswith("epoch") and p.stem[5:].isdigit())]
    weights.sort(key=lambda p:(0,int(p.stem[5:])) if p.stem.startswith("epoch") else (1,p.name))
    if not weights:raise ValueError("No explicit epoch/best/last checkpoints found")
    output.mkdir(parents=True,exist_ok=False)
    candidates=[]
    seen={}
    for weight in weights:
        sha=digest(weight)
        if sha in seen:
            candidates.append({**seen[sha],"checkpoint":str(weight),"identical_to":seen[sha]["checkpoint"]})
            continue
        report=evaluate(output/weight.stem,weight,dataset=dataset,split="val",imgsz=(imgsz,),
                        device=device,conf=.001,warmup=5,official_map=True,
                        development_pilot=development_pilot)
        result=report["per_resolution"][str(imgsz)]
        sweep=result["confidence_analysis"]["threshold_sweep"]
        operating=next((item for item in sweep if item["conf"]==conf),None)
        if operating is None:
            # Explicit nonstandard confidence is derived from saved validation predictions only.
            from traffic_light_yolo_dataset import match_detections,detection_summary
            rows=[]
            for line in (output/weight.stem/"predictions.jsonl").read_text().splitlines():
                row=json.loads(line);row["predictions"]=[p for p in row["predictions"] if p["confidence"]>=conf]
                row["matching"]=match_detections(row["truth_boxes"],row["predictions"],.5);rows.append(row)
            operating={"conf":conf,**detection_summary(rows)}
        candidate={"checkpoint":str(weight),"sha256":sha,"operating_point":operating,
                   "official_mAP":result["official_mAP"],"confidence_analysis":result["confidence_analysis"]}
        seen[sha]=candidate;candidates.append(candidate)
        (output/"progress.json").write_text(json.dumps({"completed":len(candidates),"total":len(weights),"last":str(weight)},indent=2))
        print(json.dumps({"candidate":str(weight),"operating_point":operating,"mAP":candidate["official_mAP"]}),flush=True)
    ranked=sorted(candidates,key=selection_key,reverse=True)
    report={"research_only":True,"not_production":True,"split":"val","test_inference_performed":False,
            "dataset_sha256":digest(dataset),"imgsz":imgsz,"fixed_conf":conf,"nms":"default_one_to_many",
            "selection_policy":["maximize mission5head recall at fixed confidence and IoU .5",
                                "then maximize overall housing F1 at the same operating point",
                                "then minimize unmatched predictions (FP)","then maximize official validation mAP50-95"],
            "selected":ranked[0],"ranked_candidates":ranked,"script_sha256":digest(__file__),
            "caveats":["Identical weight files may be listed twice but are inferred once",
                       "This ranking is a research shortlist, not an automatic production promotion; inspect precision/FP and threshold evidence and obtain root acceptance",
                       "Ultralytics epochN filenames use zero-based epoch indices",
                       "No signal color or map route-association metric is asserted",
                       "A validation-selected checkpoint still requires one final locked-test evaluation"]}
    with (output/"selection.json").open("x") as stream:json.dump(report,stream,indent=2,allow_nan=False)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset",type=Path,required=True)
    parser.add_argument("--weights-dir",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--imgsz",type=int,choices=(640,960,1280),default=960)
    parser.add_argument("--device",default="0");parser.add_argument("--conf",type=float,default=.25)
    parser.add_argument("--development-pilot",action="store_true")
    report=run(**vars(parser.parse_args()));print(json.dumps({"selected":report["selected"]["checkpoint"]}))


if __name__=="__main__":main()
