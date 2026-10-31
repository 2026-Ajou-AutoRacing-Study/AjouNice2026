#!/usr/bin/env python3
"""Exact-stamp live-debug/native audit and unchanged-pixel contact sheet; no model."""
import argparse
from collections import Counter,defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics

from traffic_light_yolo_dataset import digest


def distribution(values):
    values=sorted(values)
    if not values:return {"n":0}
    return {"n":len(values),"min":values[0],"p50":statistics.median(values),
            "p90":values[max(0,math.ceil(.9*len(values))-1)],"max":values[-1]}


def verified_state(row):
    if row.get("native_bracket_stable") is not True:return None
    values=[]
    for key in ("native_before","native_after"):
        matches=[o["tl_color"] for response in row[key] if response.get("readback_success") is True
                 for o in response.get("observations",[]) if o.get("tl_id")=="C1256W000052"
                 and o.get("identity_matches_request") is True and o.get("color_usable_for_gt") is True]
        if len(matches)!=1:return None
        values.append(matches[0])
    return values[0] if values[0]==values[1] else None


def run(capture,monitor,output,sheet):
    capture,monitor=Path(capture).resolve(strict=True),Path(monitor).resolve(strict=True)
    output,sheet=Path(output),Path(sheet)
    if any(p.exists() or p.is_symlink() for p in (output,sheet)):raise FileExistsError("Fresh audit outputs required")
    frames=json.loads(capture.read_text())["records"]
    observations=[json.loads(line) for line in monitor.read_text().splitlines()]
    debug=[row["parsed_debug"] for row in observations if str(row.get("topic","")).endswith("/debug_classifier")]
    stamp=lambda row:row["header_stamp"]["secs"]*10**9+row["header_stamp"]["nsecs"]
    lo,hi=min(map(stamp,frames)),max(map(stamp,frames));lookup=defaultdict(list)
    for row in debug:lookup[row["source_stamp_ns"]].append(row)
    window=[row for row in debug if lo<=row["source_stamp_ns"]<=hi]
    records=[];unmatched=[];unstable=[];duplicates=[]
    for index,frame in enumerate(frames):
        matches=lookup[stamp(frame)]
        if not matches:unmatched.append(index);continue
        if len(matches)!=1:duplicates.append(index);continue
        row=matches[0]
        if row.get("head_id")!="C1256W000052" or row.get("group_id")!=201326:raise ValueError("Exact-stamp wrong head/group")
        item={"capture_index":index,"image_path":frame["image_path"],"image_sha256":frame["image_sha256"],
              "source_stamp_ns":stamp(frame),"native_state":verified_state(frame),"debug":row}
        if item["native_state"] is None:unstable.append(item)
        else:records.append(item)
    by_state={}
    for state in sorted({row["native_state"] for row in records}):
        selected=[row for row in records if row["native_state"]==state]
        by_state[str(state)]={"n":len(selected),"known":sum(row["debug"]["known"] is True for row in selected),
            "unknown":sum(row["debug"]["known"] is False for row in selected),
            "left_positive_known":sum(row["debug"].get("bits",{}).get("left_arrow")==1 for row in selected),
            "left_off_known":sum(row["debug"].get("bits",{}).get("left_arrow")==0 for row in selected),
            "p_left":distribution([row["debug"]["probabilities"]["left_arrow"] for row in selected]),
            "p_left_unknown":distribution([row["debug"]["probabilities"]["left_arrow"] for row in selected if row["debug"]["known"] is False])}
    chosen=[row for row in records if row["native_state"]==33]
    controls=[row for row in records if row["native_state"]==1]
    if controls:chosen.extend([controls[0],controls[-1]] if len(controls)>1 else controls)
    import cv2
    import numpy as np
    from PIL import Image,ImageDraw,ImageFont
    nearest=getattr(Image,"Resampling",Image).NEAREST
    try:font=ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",13)
    except OSError:font=ImageFont.load_default()
    cards=[]
    for item in chosen:
        path=capture.parent/item["image_path"]
        if digest(path)!=item["image_sha256"]:raise ValueError("Source JPEG SHA mismatch")
        image=cv2.imdecode(np.frombuffer(path.read_bytes(),dtype=np.uint8),cv2.IMREAD_COLOR)
        if image is None:raise ValueError("Original JPEG decode failed")
        x1,y1,x2,y2=item["debug"]["crop_xyxy"]
        rgb=cv2.cvtColor(image[y1:y2,x1:x2],cv2.COLOR_BGR2RGB)
        roi=Image.fromarray(rgb);large=roi.resize((roi.width*8,roi.height*8),nearest)
        bbox=item["debug"]["detector_bbox_xyxy"];a,b,c,d=[int(v) for v in bbox]
        tight=Image.fromarray(cv2.cvtColor(image[b:d,a:c],cv2.COLOR_BGR2RGB))
        item["source_JPEG_SHA256_verified"]=True
        item["crop_pixel_sha256_RGB"]=hashlib.sha256(rgb.tobytes()).hexdigest()
        item["sheet_ROI_is_actual_saved_CNN_input_before_letterbox"]=True
        item["sheet_original_scale"]=1;item["sheet_enlargement"]=8
        cards.append((item,roi,large,tight))
    row_height=max(image.height for _,_,image,_ in cards)+82
    canvas=Image.new("RGB",(1200,55+row_height*len(cards)),(238,238,238));draw=ImageDraw.Draw(canvas)
    draw.text((10,8),"Saved live debug + exact JPEG stamp; no inference. Native state is NOT independent pixel-visibility GT.",font=font,fill="black")
    draw.text((10,28),"Left: original ROI12 and tight bbox0 at 1:1. Right: exact SAME ROI12 pixels enlarged 8x NEAREST.",font=font,fill="black")
    for number,(item,roi,large,tight) in enumerate(cards):
        y=55+number*row_height;d=item["debug"];p=d["probabilities"]
        draw.text((10,y),"idx=%02d stamp=%s native=%s known=%s Lbit=%s pL=%.8f confidence=%.8f"%(item["capture_index"],item["source_stamp_ns"],item["native_state"],d["known"],d["bits"]["left_arrow"],p["left_arrow"],d["confidence"]),font=font,fill="black")
        draw.text((10,y+20),"bbox="+str([round(v,3) for v in d["detector_bbox_xyxy"]])+"  crop="+str(d["crop_xyxy"])+"  reason="+d["reason"],font=font,fill="black")
        draw.text((10,y+42),"ROI12 1:1       bbox0 1:1      P(R,Y,G,L)="+str([round(p[k],7) for k in ("red","yellow","green","left_arrow")]),font=font,fill="black")
        canvas.paste(roi,(25,y+95));canvas.paste(tight,(150,y+95));canvas.paste(large,(380,y+68))
    canvas.save(sheet)
    probabilities=[row for row in window if row.get("probabilities")]
    formula_errors=[abs(row["confidence"]-min(max(p,1-p) for p in row["probabilities"].values())) for row in probabilities]
    report={"read_only_model_and_data_analysis":True,"model_or_GPU_forward_called":False,"labels_weights_threshold_policy_changed":False,
        "sources":{str(path):digest(path) for path in (capture,monitor)},"capture_records":len(frames),
        "window_header_stamp_ns":[lo,hi],"window_classifier_records":len(window),
        "window_known_reason_counts":dict(Counter(str((row["known"],row["reason"])) for row in window)),
        "window_UNKNOWN_left_probability":distribution([row["probabilities"]["left_arrow"] for row in window if row["reason"]=="low_confidence_bits:left_arrow"]),
        "exact_stamp_classifier_matches":len(records)+len(unstable),"exact_stamp_unmatched_capture_indices":unmatched,
        "duplicate_debug_match_indices":duplicates,"matched_unstable_native_excluded":unstable,
        "stable_exact_stamp_native_records":len(records),"by_native_state":by_state,"stable_records":records,
        "sheet":str(sheet),"sheet_sha256":digest(sheet),"sheet_cases":len(cards),
        "controls_chosen_by":"first/last chronological stable exact-stamp native1, not probability selection",
        "confidence_semantics":"min over 4 bits of max(p,1-p); high confidence can mean confident OFF, not positive lamp probability",
        "stored_confidence_formula_max_abs_error":max(formula_errors,default=None),
        "limits":["Exact-stamp subset only; do not assign the other window UNKNOWNs a native state by nearest timestamp",
                  "Native readback is not independent visible-glyph GT; sheet supports separate direct pixel review",
                  "KNOWN is not correctness: native33 rows with left_bit0 disagree with native state",
                  "No threshold tuning, model rerun, simulator/ROS control, or raw JPEG edits"]}
    with output.open("x") as stream:json.dump(report,stream,indent=2,ensure_ascii=False,allow_nan=False)
    return {"output":str(output),"sheet":str(sheet),"sheet_cases":len(cards),"exact_stable_native":len(records),"by_native_state":by_state}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("capture","monitor","output","sheet"):parser.add_argument("--"+name,type=Path,required=True)
    print(json.dumps(run(**vars(parser.parse_args()))))


if __name__=="__main__":main()
