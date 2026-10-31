#!/usr/bin/env python3
"""Offline housing recall/mAP and synchronized resolution cost; no live ROS."""
import argparse
from collections import defaultdict
import importlib.metadata
import json
import math
from pathlib import Path
import random
import statistics
import time

from traffic_light_yolo_dataset import digest, load_dataset, match_detections, detection_summary, write_data_yaml
from train_traffic_light_yolo import offline_runtime, ULTRALYTICS_VERSION


def timing_summary(values):
    ordered = sorted(values)
    if not ordered:return {"count": 0}
    return {"count": len(values), "mean_ms": statistics.mean(values), "median_ms": statistics.median(values),
            "p95_ms": ordered[max(0, math.ceil(.95*len(values))-1)], "min_ms": ordered[0], "max_ms": ordered[-1]}


def _tensor_list(value):
    return value.detach().cpu().tolist()


def confidence_distribution(values):
    ordered=sorted(values)
    if not ordered:return {"count":0,"min":None,"p50":None,"p95":None,"max":None}
    return {"count":len(ordered),"min":ordered[0],"p50":statistics.median(ordered),
            "p95":ordered[max(0,math.ceil(.95*len(ordered))-1)],"max":ordered[-1]}


def confidence_analysis(rows, minimum_conf):
    """Validation-only score diagnostics; rematch after each explicit threshold."""
    matched,unmatched=[],[]
    for row in rows:
        matched.extend(row["predictions"][m["prediction_index"]]["confidence"] for m in row["matching"]["matches"])
        unmatched.extend(row["predictions"][i]["confidence"] for i in row["matching"]["false_positive_indices"])
    thresholds=sorted({minimum_conf,.001,.005,.01,.025,.05,.1,.15,.2,.25,.3,.4,.5,.75})
    sweep=[]
    for threshold in thresholds:
        if threshold<minimum_conf:continue
        filtered=[]
        for row in rows:
            predictions=[p for p in row["predictions"] if p["confidence"]>=threshold]
            filtered.append({**row,"predictions":predictions,
                             "matching":match_detections(row["truth_boxes"],predictions,.5)})
        sweep.append({"conf":threshold,**detection_summary(filtered)})
    return {"scope":"scores_are_truncated_below_predict_conf_not_all_raw_anchors",
            "minimum_predict_conf":minimum_conf,"matched_TP_scores":confidence_distribution(matched),
            "unmatched_FP_scores":confidence_distribution(unmatched),"threshold_sweep":sweep,
            "association_evaluated":False}


def run(output, weights, *, dataset=None, images=None, split="val", imgsz=(640,960,1280),
        device="cpu", conf=.25, warmup=5, repeats=1, max_images=None, official_map=False,
        development_pilot=False, nms_free=False, warmup_image=None):
    output, weights = Path(output).resolve(), Path(weights).resolve(strict=True)
    if output.exists() or output.is_symlink():raise FileExistsError("Evaluation output must be fresh")
    if bool(dataset) == bool(images):raise ValueError("Provide either verified detection dataset or timing-only images")
    if any(size not in (640,960,1280) for size in imgsz) or not 0 <= conf <= 1 or min(warmup,repeats) < 1:
        raise ValueError("Invalid explicit resolution/confidence/repetition")
    if dataset:
        records, source = load_dataset(dataset, development_pilot=development_pilot)
        records = [row for row in records if row["split"] == split]
        if max_images is not None:records = records[:max_images]
    else:
        records, source = [], {"scope": "timing_only_no_ground_truth_no_accuracy"}
        for path in images:
            path=Path(path).resolve(strict=True)
            records.append({"resolved_image_path": str(path), "image_path": path.name,
                            "image_sha256": digest(path), "boxes": None})
        if official_map:raise ValueError("mAP requires a verified, fully annotated dataset")
    if not records:raise ValueError("No frames in requested evaluation split")
    if importlib.metadata.version("ultralytics") != ULTRALYTICS_VERSION:raise ValueError("Pinned Ultralytics required")
    output.mkdir(parents=True, exist_ok=False)
    import cv2
    import torch
    torch.set_num_threads(4)
    torch.backends.cudnn.benchmark=False
    torch.backends.cudnn.allow_tf32=False
    torch.backends.cuda.matmul.allow_tf32=False
    gpu=None
    if device!="cpu":
        props=torch.cuda.get_device_properties(int(device) if str(device).isdigit() else device)
        gpu={"name":props.name,"total_memory_bytes":props.total_memory,"CUDA_runtime":torch.version.cuda}
    grouped, timings, shapes = defaultdict(list), defaultdict(list), defaultdict(set)
    def synchronize():
        if device != "cpu":torch.cuda.synchronize(int(device) if str(device).isdigit() else device)
    def image_for(row):
        path=Path(row["resolved_image_path"])
        if digest(path)!=row["image_sha256"]:raise ValueError("JPEG changed during evaluation")
        image=cv2.imread(str(path))
        if image is None:raise ValueError("JPEG decode failed")
        return image
    with offline_runtime():
        from ultralytics import YOLO, settings
        settings.update({key:False for key in ("sync","wandb","mlflow","tensorboard") if key in settings})
        model=YOLO(str(weights))
        if dataset and dict(model.names)!={0:"traffic_light"}:raise ValueError("Housing evaluation needs the single-class detector")
        current_size=[None]
        def capture_shape(_module,args):
            if current_size[0] is not None and args and hasattr(args[0],"shape"):
                shapes[current_size[0]].add(tuple(int(value) for value in args[0].shape))
        hook=model.model.register_forward_pre_hook(capture_shape)
        predict={"device":device,"conf":conf,"verbose":False,"save":False,"stream":False,
                 "rect":True,"quantize":32,"nms":False if nms_free else None}
        if warmup_image:
            warmup_path=Path(warmup_image).resolve(strict=True)
            first=image_for({"resolved_image_path":str(warmup_path),"image_sha256":digest(warmup_path)})
        else:first=image_for(records[0])
        for size in imgsz:
            for _ in range(warmup):model.predict(first,imgsz=size,**predict)
        synchronize()
        try:
            with (output/"predictions.jsonl").open("x",encoding="utf-8") as stream:
                rng=random.Random(0)
                for repeat in range(repeats):
                    for row in records:
                        image=image_for(row)
                        order=list(imgsz);rng.shuffle(order)
                        for size in order:
                            current_size[0]=size;synchronize();started=time.perf_counter()
                            result=model.predict(image,imgsz=size,**predict)[0]
                            boxes=[]
                            for xyxy,score,cls in zip(_tensor_list(result.boxes.xyxy),_tensor_list(result.boxes.conf),_tensor_list(result.boxes.cls)):
                                boxes.append({"xyxy":xyxy,"confidence":float(score),"class_id":int(cls)})
                            synchronize();elapsed=(time.perf_counter()-started)*1000;timings[size].append(elapsed)
                            item={"image_path":row["image_path"],"image_sha256":row["image_sha256"],"imgsz":size,
                                  "repeat":repeat,"predictions":boxes,"predict_wall_ms_synchronized":elapsed,
                                  "ultralytics_speed_ms":result.speed,"group_id":row.get("group_id"),"environment":row.get("environment")}
                            if dataset:
                                item["truth_boxes"]=row["boxes"]
                                item["matching"]=match_detections(row["boxes"],boxes,.5)
                                if repeat==0:grouped[size].append(item)
                            stream.write(json.dumps(item,allow_nan=False)+"\n")
        finally:hook.remove()
        maps={}
        if official_map:
            data_yaml=output/"resolved_data.yaml";write_data_yaml(data_yaml,Path(dataset).resolve().parent)
            for size in imgsz:
                metric=model.val(data=str(data_yaml),split=split,imgsz=size,device=device,conf=.001,
                    iou=.7,rect=False,batch=16,quantize=32,nms=False if nms_free else None,
                    plots=False,save_json=False,project=str(output),name="official_val_"+str(size),exist_ok=False)
                maps[size]={"mAP50":float(metric.box.map50),"mAP50_95":float(metric.box.map),"mAP75":float(metric.box.map75),
                            "conf":.001,"rect":False,"scope":"official_whole_split_not_fixed_conf_or_max_images_subset"}
    report={"research_only":True,"not_production":True,"scope":"verified_housing_detection" if dataset else "timing_only_no_accuracy",
        "source":source,"weights_path":str(weights),"weights_sha256":digest(weights),"ultralytics":ULTRALYTICS_VERSION,
        "device":device,"precision":"FP32","nms_mode":"one_to_one_no_NMS" if nms_free else "default_one_to_many_NMS",
        "torch_version":str(torch.__version__),"cpu_threads":4,"TF32":False,
        "GPU":gpu,
        "split":split if dataset else None,"frame_count":len(records),"warmup":warmup,"repeats":repeats,
        "warmup_image":str(Path(warmup_image).resolve()) if warmup_image else "first_requested_split_image",
        "fixed_conf":conf,"IoU_for_recall":.5,"latency_scope":"decoded_BGR_to_predictions_and_GPU_sync_excludes_JPEG_decode_not_ROS",
        "per_resolution":{str(size):{"timing":timing_summary(timings[size]),"network_input_shapes":sorted(shapes[size]),
            "detection":detection_summary(grouped[size]) if dataset else None,
            "confidence_analysis":confidence_analysis(grouped[size],conf) if dataset else None,
            "by_environment":{key:detection_summary([r for r in grouped[size] if r["environment"]==key])
                              for key in sorted({r["environment"] for r in grouped[size]})} if dataset else None,
            "by_group":{key:detection_summary([r for r in grouped[size] if r["group_id"]==key])
                        for key in sorted({r["group_id"] for r in grouped[size]})} if dataset else None,
            "official_mAP":maps.get(size)} for size in imgsz},
        "script_sha256":digest(__file__),"dataset_tool_sha256":digest(Path(__file__).with_name("traffic_light_yolo_dataset.py")),
        "caveats":["No lamp color/arrow or route-association accuracy claim","GT completeness and tight-box QC are human assertions",
                   "Correct non-mission front-head detections are not detector FP; selector must reject inactive groups",
                   "Unmatched boxes require visual audit: they can include out-of-study tiny/back faces or localization mismatches, not necessarily hallucinations",
                   "Minimum-rectangle predict and square official val use different padding","Server inference is not competition-host ROS latency"]}
    with (output/"summary.json").open("x",encoding="utf-8") as stream:json.dump(report,stream,indent=2,allow_nan=False)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--dataset",type=Path)
    source.add_argument("--image",dest="images",action="append",type=Path)
    parser.add_argument("--weights",type=Path,required=True);parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--split",choices=("train","val","test"),default="val")
    parser.add_argument("--imgsz",nargs="+",type=int,default=[640,960,1280]);parser.add_argument("--device",default="cpu")
    parser.add_argument("--conf",type=float,default=.25);parser.add_argument("--warmup",type=int,default=5)
    parser.add_argument("--warmup-image",type=Path,help="Use a validation/train JPEG for warmup so test images are inferred exactly once")
    parser.add_argument("--repeats",type=int,default=1);parser.add_argument("--max-images",type=int)
    parser.add_argument("--official-map",action="store_true");parser.add_argument("--development-pilot",action="store_true")
    parser.add_argument("--nms-free",action="store_true")
    arguments=vars(parser.parse_args());result=run(**arguments)
    print(json.dumps({"output":str(arguments["output"].resolve()),"scope":result["scope"]}))


if __name__=="__main__":main()
