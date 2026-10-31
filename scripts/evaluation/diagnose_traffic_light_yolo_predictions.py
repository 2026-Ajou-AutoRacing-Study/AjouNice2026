#!/usr/bin/env python3
"""CPU-only forensic summaries/overlays from already frozen predictions; no model."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path

from traffic_light_yolo_dataset import digest,iou,match_detections,detection_summary
from build_traffic_light_yolo_cnn_crops import node_crop,contains


def phase_key(row,phase_lookup):
    known=phase_lookup.get(row["image_sha256"])
    if known:return "verified_target:"+known[0]+":state:"+str(known[1])
    if row.get("native_bracket_stable") is not True:return "UNKNOWN_unverified_or_unstable_native_bracket"
    def observations(entries):
        result={}
        for entry in entries:
            if entry.get("readback_success") is not True:continue
            for obs in entry.get("observations",[]):
                if obs.get("identity_matches_request") is True and obs.get("color_usable_for_gt") is True:
                    result[obs["tl_id"]]=obs.get("tl_color")
        return result
    before,after=observations(row.get("native_before",[])),observations(row.get("native_after",[]))
    stable={key:value for key,value in before.items() if value==after.get(key)}
    return "verified_target:"+"|".join(key+":state:"+str(value) for key,value in sorted(stable.items())) if stable else "UNKNOWN_no_usable_exact_target_readback"


def analyze(rows):
    fp_categories=Counter();missed=[];duplicate_pairs=0;groups=defaultdict(list)
    relaxed=[]
    for row in rows:
        groups[(row["group_id"],row["environment"],row["phase_key"])].append(row)
        truth,predictions=row["truth_boxes"],row["predictions"]
        relaxed.append({**row,"matching":match_detections(truth,predictions,.25)})
        for n,pred in enumerate(predictions):
            duplicate_pairs+=sum(iou(pred["xyxy"],other["xyxy"])>=.5 for other in predictions[:n])
        for n in row["matching"]["false_positive_indices"]:
            best=max((iou(predictions[n]["xyxy"],gt["xyxy"]) for gt in truth),default=0.)
            fp_categories["overlap_ge0_5_duplicate_of_GT" if best>=.5 else
                          "localization_overlap_0_25_to_0_5" if best>=.25 else
                          "weak_overlap_gt0_to_0_25" if best>0 else "zero_overlap_requires_visual_scope_audit"]+=1
        for n in row["matching"]["missed_truth_indices"]:
            gt=truth[n]
            best=max(((iou(gt["xyxy"],p["xyxy"]),j,p) for j,p in enumerate(predictions)),default=(0,None,None),key=lambda x:x[0])
            crop=node_crop(best[2]["xyxy"],1280,720) if best[2] is not None else None
            missed.append({"image_path":row["image_path"],"image_sha256":row["image_sha256"],
                "group_id":row["group_id"],"environment":row["environment"],"phase_key":row["phase_key"],
                "native_id":gt.get("native_id"),"GT_xyxy":gt["xyxy"],"best_pred_index":best[1],
                "best_pred":best[2],"best_IoU":best[0],"node_crop_xyxy":crop,
                "node_crop_contains_entire_GT":contains(crop,gt["xyxy"]) if crop else False,
                "presence_not_proven_from_geometry_alone":True})
    return {"strict_IoU_0_5":detection_summary(rows),
        "geometry_IoU_0_25_DIAGNOSTIC_NOT_PIPELINE_IDENTITY":detection_summary(relaxed),
        "FP_geometric_categories":dict(fp_categories),"pred_pred_overlap_ge0_5_pairs":duplicate_pairs,
        "missed_GT":missed,
        "by_group_environment_target_phase":[{"group_id":key[0],"environment":key[1],"phase_key":key[2],
            **detection_summary(items)} for key,items in sorted(groups.items())],
        "phase_caveat":"Verified native phase describes the queried target head only, never every other head; unavailable/unstable readback remains UNKNOWN. No phase inferred from filename or model.",
        "relaxed_metric_caveat":"IoU .25 is geometry-only diagnostic, not full pipeline target identity/association. Strict IoU .5 scores remain unchanged and no labels or model settings are modified."}


def visual_case(row,dataset_root,path,focus_native=None,extra_prediction=False):
    from PIL import Image,ImageDraw
    source=Image.open(dataset_root/row["image_path"]).convert("RGB")
    if digest(dataset_root/row["image_path"])!=row["image_sha256"]:raise ValueError("JPEG SHA changed")
    truth,preds=row["truth_boxes"],row["predictions"]
    focus=next((gt["xyxy"] for gt in truth if gt.get("native_id")==focus_native),None)
    if extra_prediction:
        choices=[(max((iou(p["xyxy"],gt["xyxy"]) for gt in truth),default=0.),i,p) for i,p in enumerate(preds)]
        focus=min(choices,key=lambda x:x[0])[2]["xyxy"]
    if focus is None:focus=truth[0]["xyxy"]
    x1,y1,x2,y2=focus;region=[max(0,int(x1)-24),max(0,int(y1)-24),min(1280,int(x2)+24),min(720,int(y2)+24)]
    if focus_native in ("C1256W000034","C1256W000011"):region=[1160,230,1280,292]
    annotated=source.copy();draw=ImageDraw.Draw(annotated)
    for gt in truth:draw.rectangle(gt["xyxy"],outline=(0,255,0),width=1)
    for pred in preds:draw.rectangle(pred["xyxy"],outline=(255,0,255),width=1)
    canvas=Image.new("RGB",(1280,750),(235,235,235));out=ImageDraw.Draw(canvas)
    out.text((8,5),row["group_id"]+" / "+row["environment"]+" | green=GT magenta=prediction | no model rerun",fill="black")
    nearest=getattr(Image,"Resampling",Image).NEAREST
    full=annotated.resize((640,360),nearest);canvas.paste(full,(320,25))
    def pane(image,left):
        crop=image.crop(region);scale=min(630/crop.width,330/crop.height)
        crop=crop.resize((round(crop.width*scale),round(crop.height*scale)),nearest)
        canvas.paste(crop,(left+(630-crop.width)//2,410+(330-crop.height)//2))
    pane(source,5);pane(annotated,645)
    out.text((8,390),"CLEAN pixels / source crop "+str(region),fill="black")
    out.text((648,390),"SAME pixels + unchanged GT / frozen prediction boxes",fill="black")
    canvas.save(path)
    return {"path":path.name,"sha256":digest(path),"source_JPEG_path":row["image_path"],
        "source_JPEG_sha256":row["image_sha256"],"region_xyxy":region,"group_id":row["group_id"],
        "environment":row["environment"],"focus_native_id":focus_native,"extra_prediction_focus":extra_prediction,
        "GT_boxes":truth,"predictions":preds,"phase_key":row["phase_key"]}


def run(dataset,predictions,phase_source,output):
    dataset,predictions,phase_source=(Path(p).resolve(strict=True) for p in (dataset,predictions,phase_source))
    output=Path(output)
    if output.exists() or output.is_symlink():raise FileExistsError("Fresh forensic output required")
    records=json.loads(dataset.read_text())["records"];lookup={r["image_sha256"]:r for r in records}
    phases={}
    for row in json.loads(phase_source.read_text())["records"]:
        p=row.get("provenance",{})
        if row.get("native_gt_verified") is True and p.get("native_observed_id") and p.get("native_observed_state") is not None:
            value=(p["native_observed_id"],p["native_observed_state"])
            if row["image_sha256"] in phases and phases[row["image_sha256"]]!=value:raise ValueError("Conflicting source phases")
            phases[row["image_sha256"]]=value
    collector_evidence={};collector_cache={}
    for row in records:
        if row["image_sha256"] in phases:continue
        relative=Path(row.get("source_image_path",""))
        if relative.is_absolute() or ".." in relative.parts:raise ValueError("Unsafe collector source path")
        manifest=phase_source.parent/relative.parent.parent/"manifest.json"
        if not manifest.is_file():continue
        if manifest not in collector_cache:
            collector_cache[manifest]=json.loads(manifest.read_text()).get("frames",[])
            collector_evidence[str(manifest)]=digest(manifest)
        for frame in collector_cache[manifest]:
            gt=frame.get("native_gt",{})
            if (frame.get("sha256")==row["image_sha256"] and frame.get("native_GT_verified") is True
                    and frame.get("accepted_for_sample_quota") is not False
                    and gt.get("before",{}).get("verified") is True and gt.get("after",{}).get("verified") is True
                    and gt.get("jpeg_receipt_bracketed") is True and gt.get("observed_native_id")
                    and gt.get("observed_state") is not None):
                phases[row["image_sha256"]]=(gt["observed_native_id"],gt["observed_state"])
    rows=[]
    for line in predictions.read_text().splitlines():
        row=json.loads(line);source=lookup[row["image_sha256"]]
        if source["boxes"]!=row["truth_boxes"]:raise ValueError("Saved GT differs from frozen manifest")
        row["phase_key"]=phase_key(source,phases);rows.append(row)
    report=analyze(rows);report.update(model_forward_called=False,source_prediction_sha256=digest(predictions),
        source_dataset_sha256=digest(dataset),phase_source_sha256=digest(phase_source),
        model_or_threshold_reselected=False,labels_modified=False,strict_scores_unchanged=True)
    report["collector_phase_evidence_sha256"]=collector_evidence
    output.mkdir(parents=True,exist_ok=False)
    (output/"diagnostics.json").write_text(json.dumps(report,indent=2))
    selections=[]
    for environment in ("Sunny11","Sunny13","Sunny15","Foggy11","Foggy13","Foggy15"):
        row=next(r for r in rows if r["group_id"]=="natural_D2_d46_latp1_8_yawm7" and r["environment"]==environment)
        selections.append((row,"C1256W000052",False))
    for group,native in (("C32_offset_yaw","C1256W000034"),("natural_B_d46_latp1_8_yawm7","C1256W000045")):
        for environment in ("Sunny13","Foggy15"):
            selections.append((next(r for r in rows if r["group_id"]==group and r["environment"]==environment),native,False))
    selections.append((next(r for r in rows if r["group_id"]=="natural_A_d46_latp1_8_yawm7" and r["environment"]=="Foggy11"),"C1256W000082",False))
    selections.append((next(r for r in rows if r["group_id"]=="natural_D2_d46_latp1_8_yawm7" and r["environment"]=="Sunny13"),None,True))
    cases=[visual_case(row,dataset.parent,output/("case_%02d.png"%i),native,extra) for i,(row,native,extra) in enumerate(selections)]
    (output/"case_index.json").write_text(json.dumps({"source_predictions_sha256":digest(predictions),"model_forward_called":False,"cases":cases},indent=2))
    return {"output":str(output),"cases":len(cases),"model_forward_called":False}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("dataset","predictions","phase-source","output"):parser.add_argument("--"+name,type=Path,required=True)
    print(json.dumps(run(**vars(parser.parse_args()))))


if __name__=="__main__":main()
