#!/usr/bin/env python3
"""Apply explicit crop-sheet review to named housings; never training labels."""
import argparse
import hashlib
import json
import math
from pathlib import Path

MISSION_HEAD_GROUPS = {'C1256W000077':201351,'C1256W000091':201365,
                       'C1256W000016':201290,'C1256W000018':201290,'C1256W000052':201326}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('dataset','review-index','confirmation','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    source=args.dataset.resolve(strict=True)
    review=json.loads(args.review_index.read_text())
    approval=json.loads(args.confirmation.read_text())
    if args.output.exists() or args.output.is_symlink():raise ValueError('Fresh output required')
    if args.output.resolve().parent!=source.parent:raise ValueError('Preserve original relative JPEG paths')
    if approval.get('review_index_sha256')!=digest(args.review_index):raise ValueError('Review index changed')
    if approval.get('all_listed_crops_directly_reviewed') is not True or approval.get('accepted') is not True:
        raise ValueError('Explicit completed visual review is required')
    if review.get('source_sha256')!=digest(source):raise ValueError('Source manifest changed')
    if review.get('model_predictions_used') is not False:raise ValueError('Prediction-free identity QC required')
    rows=json.loads(source.read_text())['records']
    entries=review['entries']
    if len(entries)!=approval.get('reviewed_crop_count'):raise ValueError('Crop review count mismatch')
    sheets={entry['sheet'] for entry in entries}
    if set(approval.get('reviewed_sheets',{}))!=sheets:raise ValueError('Every actual sheet must be reviewed')
    for name,sha in approval['reviewed_sheets'].items():
        if Path(name).name!=name or digest(args.review_index.parent/name)!=sha:raise ValueError('Review sheet changed')
    boxes={i:[] for i in range(len(rows))};seen=set()
    for entry in entries:
        i=entry['record_index']
        if type(i) is not int or not 0<=i<len(rows):raise ValueError('Invalid frame index')
        row=rows[i];key=(i,entry['native_id'])
        if key in seen:raise ValueError('Repeated frame/head annotation')
        seen.add(key)
        relative=Path(row['image_path'])
        if relative.is_absolute() or '..' in relative.parts:raise ValueError('Unsafe source JPEG path')
        image=(source.parent/relative).resolve(strict=True)
        if source.parent not in image.parents:raise ValueError('JPEG escapes source directory')
        if entry['source_image_sha256']!=row['image_sha256'] or digest(image)!=row['image_sha256']:
            raise ValueError('Reviewed original JPEG changed')
        if (entry['native_id'] not in MISSION_HEAD_GROUPS or
                MISSION_HEAD_GROUPS[entry['native_id']]!=row.get('group_regulatory_id')):
            raise ValueError('Named head is not in this mission group')
        from PIL import Image
        with Image.open(image) as original:width,height=original.size
        box=entry['xyxy']
        if (not isinstance(box,list) or len(box)!=4 or any(
                isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in box)
                or not 0<=box[0]<box[2]<=width or not 0<=box[1]<box[3]<=height):
            raise ValueError('Invalid named housing ROI')
        boxes[i].append({'native_id':entry['native_id'],'xyxy':entry['xyxy'],'class_id':0,
            'box_qc_accepted':True,'manual_direct_visual_qc_accepted':True,'proposal_only':False,
            'review_sheet':entry['sheet'],'review_sheet_tile':entry['tile'],
            'independent_human_GT':False})
    if any(not value for value in boxes.values()):raise ValueError('Unreviewed source frame')
    result={'scope':'named_mission_identity_QC_not_full_detector_training_annotations',
        'source_manifest_sha256':digest(source),'review_index_sha256':digest(args.review_index),
        'confirmation_sha256':digest(args.confirmation),'CNN_or_detector_predictions_used':False,
        'pixel_lamp_labels_certified':False,'records':[]}
    for i,row in enumerate(rows):
        result['records'].append({key:row[key] for key in ('image_path','image_sha256','group_id','environment','split')})
        result['records'][-1].update(annotation_complete=False,identity_annotations_reviewed=True,
            annotation_scope='named_housing_identity_evaluation_only_not_detector_training_GT',
            boxes=boxes[i])
    with args.output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps({'frames':len(rows),'named_head_boxes':len(entries),'output':str(args.output)}))


if __name__=='__main__':main()
