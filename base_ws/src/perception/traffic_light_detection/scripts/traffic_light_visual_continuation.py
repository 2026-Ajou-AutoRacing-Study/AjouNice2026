"""Bounded same-housing visual identity across map uncertainty peaks.

Never holds lamp values. A selected CURRENT YOLO crop must still pass the normal
CNN and phase policy. Nominal geometry is noneligible diagnostic evidence, not a
substitute low covariance. Fallback never renews the normal-map anchor.
"""
import math

from traffic_light_map_selector import project_heads, SELECTOR_CONTRACT_VERSION

UNCERTAINTY_REASONS=frozenset(('localization_uncertainty_exceeds_bound',
    'localization_vertical_uncertainty_exceeds_bound','projection_uncertainty_exceeds_bound'))


def nominal_evidence(geometry,camera,transform,group_id,xy_sigma,z_sigma,yaw_sigma,
                     projection_options,global_rejection,normal_rejections):
    """Reuse full FOV/facing/size geometry with unmodified actual uncertainty."""
    options=dict(projection_options)
    options.update(max_padding_px=1e6,max_vertical_padding_px=1e6)
    rois,rejected=project_heads(geometry,camera,transform,group_id,xy_sigma,yaw_sigma,
                              vertical_sigma_m=z_sigma,**options)
    reasons={row['head_id']:row['reason'] for row in normal_rejections}
    for roi in rois:
        roi['eligible']=False
        roi['normal_rejection_reason']=global_rejection or reasons.get(roi['head_id'])
    return dict(schema_version=1,status='ready',normal_eligibility=False,
                validated_image_time_tf_pose_extrinsic=True,
                actual_xy_sigma_m=xy_sigma,actual_z_sigma_m=z_sigma,
                actual_yaw_sigma_degrees=math.degrees(yaw_sigma),
                rois=rois,rejected_heads=rejected)


def _bbox(box):
    try:
        values=box[4][:4];confidence=box[4][4]
        if (len(values)!=4 or not all(math.isfinite(float(v)) for v in values)
                or isinstance(confidence,bool) or not math.isfinite(float(confidence))
                or not 0<=float(confidence)<=1):return None
        a,b,c,d=map(float,values)
        return (a,b,c,d) if a<c and b<d else None
    except (TypeError,ValueError,IndexError,OverflowError):
        return None


def _iou(a,b):
    area=max(0.,min(a[2],b[2])-max(a[0],b[0]))*max(0.,min(a[3],b[3])-max(a[1],b[1]))
    return area/((a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-area)


def _identity_iou(a,b):
    # Small detector-boundary jitter is not a different housing. This padding
    # is used ONLY by identity overlap, never by detector/CNN crop coordinates.
    margin=min(2.,.1*min(a[2]-a[0],a[3]-a[1],b[2]-b[0],b[3]-b[1]))
    padded=lambda r:(r[0]-margin,r[1]-margin,r[2]+margin,r[3]+margin)
    return _iou(a,b),_iou(padded(a),padded(b)),margin


class VisualContinuation:
    def __init__(self):self.reset()

    def reset(self):
        self.trusted=None;self.group=None;self.clock_ns=None;self.source_ns=None;self.ledger=None

    def _enter(self,group,source,now):
        if (type(group) is not int or group<=0 or type(source) is not int or source<=0
                or type(now) is not int or now<source):
            self.trusted=None
            return False
        if self.group!=group or (self.clock_ns is not None and now<self.clock_ns):self.reset()
        self.group=group;self.clock_ns=now
        if self.source_ns is not None and source<self.source_ns:
            self.trusted=None
            return False
        self.source_ns=source
        return True

    def remember(self,box,selection,group,source,now):
        if not self._enter(group,source,now):return
        if selection.get('status')!='matched' or selection.get('reason')!='map_head_association':return
        bbox=_bbox(box);head=selection.get('head_id')
        if bbox is None or not isinstance(head,str) or not head:return
        clock=(self.ledger[2] if self.ledger and self.ledger[:2]==(source,head) else now)
        self.ledger=(source,head,clock)
        self.trusted=dict(head_id=head,bbox=bbox,source_ns=source,clock_ns=clock)

    def select(self,boxes,packet,selection,group,source,now,window_sec):
        result=dict(status='missing',reason='visual_continuation_unqualified',eligible=False,
                    retry_imgsz=None)
        try:
            if (isinstance(window_sec,bool) or not isinstance(window_sec,(int,float))
                    or not math.isfinite(window_sec) or not 0<window_sec<=.3):return None,result
            if not self._enter(group,source,now) or not self.trusted:return None,result
            anchor=self.trusted
            if (now-anchor['clock_ns']>round(window_sec*1e9)
                    or not 0<=now-anchor['source_ns']<=500_000_000):
                self.trusted=None;result['reason']='visual_continuation_expired';return None,result
            if (not isinstance(packet,dict) or packet.get('group_id')!=group
                    or packet.get('source_stamp_ns')!=source
                    or packet.get('selector_contract_version')!=SELECTOR_CONTRACT_VERSION
                    or not 0<=now-packet['generated_stamp_ns']<=500_000_000
                    or not 0<=now-source<=500_000_000):return None,result
            if (selection.get('status')!='missing' or
                    any(r.get('reason')=='other_head_identity_ambiguous' for r in selection.get('rejected',[]))):
                self.trusted=None;return None,result
            if (packet.get('status')=='missing' and packet.get('reason') not in UNCERTAINTY_REASONS
                    or packet.get('status')=='ready' and selection.get('reason')!='no_unambiguous_route_detection'
                    or packet.get('status') not in ('missing','ready')):
                self.trusted=None;return None,result
            evidence=packet.get('nominal_continuation')
            if (not isinstance(evidence,dict) or evidence.get('schema_version')!=1
                    or evidence.get('status')!='ready' or evidence.get('normal_eligibility') is not False
                    or evidence.get('validated_image_time_tf_pose_extrinsic') is not True):
                self.trusted=None;return None,result
            for key,bound in (('actual_xy_sigma_m',1.5),('actual_z_sigma_m',1.5),('actual_yaw_sigma_degrees',5.)):
                value=evidence[key]
                if isinstance(value,bool) or not math.isfinite(value) or not 0<=value<=bound:return None,result
            nominal=evidence['rois']
            targets=[r for r in nominal if r['head_id']==anchor['head_id'] and r.get('group_id')==group
                     and r.get('normal_rejection_reason') in UNCERTAINTY_REASONS]
            if len(targets)!=1:
                self.trusted=None;result['reason']='current_head_not_fully_visible_or_not_uncertainty';return None,result
            target=targets[0];previous=anchor['bbox'];pw,ph=previous[2]-previous[0],previous[3]-previous[1]
            candidates=[]
            for index,box in enumerate(boxes):
                b=_bbox(box)
                if b is None:continue
                width,height=b[2]-b[0],b[3]-b[1]
                cx,cy=(b[0]+b[2])/2,(b[1]+b[3])/2
                shift=math.hypot(cx-(previous[0]+previous[2])/2,cy-(previous[1]+previous[3])/2)
                raw_iou,identity_iou,pixel_margin=_identity_iou(b,previous)
                if (not .67<=width/pw<=1.5 or not .67<=height/ph<=1.5
                        or identity_iou<.5 or shift>.35*math.hypot(pw,ph)):continue
                gate=target['gate_xyxy']
                if not gate[0]<=cx<=gate[2] or not gate[1]<=cy<=gate[3]:continue
                competitors=[r for r in nominal if r['head_id']!=anchor['head_id'] and
                    r['gate_xyxy'][0]<=cx<=r['gate_xyxy'][2] and r['gate_xyxy'][1]<=cy<=r['gate_xyxy'][3]]
                if competitors:
                    self.trusted=None;result['reason']='visual_continuation_other_head_competitor';return None,result
                candidates.append((index,box,raw_iou,identity_iou,pixel_margin))
            result.update(eligible=True,head_id=anchor['head_id'],trusted_source_stamp_ns=anchor['source_ns'],
                          anchor_age_sec=(now-anchor['clock_ns'])/1e9)
            if not candidates:
                result['reason']='visual_continuation_no_matching_detection'
                bbox=target['bbox_xyxy']
                if 3.<=target['depth_m']<=15. and bbox[2]-bbox[0]>=60.:result['retry_imgsz']=640
                return None,result
            # Duplicate primary/retry boxes for one housing are equivalent; a
            # genuinely distinct second match is ambiguity, not a confidence tie.
            if any(_iou(_bbox(left[1]),_bbox(right[1]))<.8
                   for index,left in enumerate(candidates) for right in candidates[index+1:]):
                self.trusted=None;result['reason']='visual_continuation_multiple_detections';return None,result
            index,box,raw_iou,identity_iou,pixel_margin=max(candidates,key=lambda row:float(row[1][4][4]))
            result.update(status='matched',reason='visual_continuation_match',box_index=index,
                          normal_eligibility=False,raw_identity_iou=raw_iou,
                          identity_iou=identity_iou,identity_boundary_tolerance_px=pixel_margin)
            return box,result
        except (KeyError,TypeError,ValueError,OverflowError,IndexError):
            self.trusted=None
            return None,result
