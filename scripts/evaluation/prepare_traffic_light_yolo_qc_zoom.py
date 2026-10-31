#!/usr/bin/env python3
"""Derived pixel-coordinate guides for human housing QC; no model/GT inference."""
import argparse
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--index",type=Path,required=True)
    parser.add_argument("--artifact-root",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--top-padding",type=int,default=22)
    args=parser.parse_args()
    import cv2
    import numpy as np
    args.output.mkdir(parents=True,exist_ok=False)
    entries=[]
    for row in json.loads(args.index.read_text()):
        if not row["group_id"].startswith(("natural_A_","natural_B_")):continue
        image=cv2.imread(str(args.artifact_root/row["image_path"]))
        for box in row["boxes"]:
            x1,y1,x2,y2=box["xyxy"]
            x0,y0=max(0,x1-10),max(0,y1-args.top_padding)
            end_x,end_y=min(image.shape[1],x2+10),min(image.shape[0],y2+12)
            raw=image[y0:end_y,x0:end_x];scale=10
            zoom=cv2.resize(raw,None,fx=scale,fy=scale,interpolation=cv2.INTER_NEAREST)
            canvas=np.full((zoom.shape[0]+70,zoom.shape[1]+75,3),255,np.uint8)
            canvas[40:40+zoom.shape[0],60:60+zoom.shape[1]]=zoom
            for x in range(x0,end_x+1):
                if x%5==0:
                    xx=60+(x-x0)*scale
                    cv2.line(canvas,(xx,40),(xx,40+zoom.shape[0]),(170,170,170),1)
                    cv2.putText(canvas,str(x),(xx-8,28),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,0,0),1)
            for y in range(y0,end_y+1):
                if y%5==0:
                    yy=40+(y-y0)*scale
                    cv2.line(canvas,(60,yy),(60+zoom.shape[1],yy),(170,170,170),1)
                    cv2.putText(canvas,str(y),(12,yy+3),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1)
            name=row["group_id"]+"_"+box["native_id"]+".png"
            cv2.imwrite(str(args.output/name),canvas)
            entries.append({"group_id":row["group_id"],"native_id":box["native_id"],"guide":name,
                            "source_crop_xyxy":[x0,y0,end_x,end_y],"scale":scale,
                            "proposal_is_not_GT":True,"original_JPEG_unchanged":True})
    with (args.output/"guides.json").open("x") as stream:json.dump(entries,stream,indent=2)


if __name__=="__main__":main()
