"""Pure sampling/rescue hypotheses; no model, GPU, ROS master or live nodes."""
from ablate_traffic_light_near_detector_scales import choose_samples, gate_proposal, merge_extra, distribution


def test_sampling_is_dense_only_for_early_d_and_keeps_other_missions():
    rows=[(t,{'group_id':201326,'source_stamp_ns':t}) for t in (0,.2,.5,1,40,40.5,41,44,45)]
    assert [t for t,_ in choose_samples(rows)]==[0,.5,1,40,45]
    rows=[(0,{'group_id':0}),(1,{'group_id':201351}),(2,{'group_id':201351}),
          (3,{'group_id':201351}),(4,{'group_id':201365})]
    assert [t for t,_ in choose_samples(rows)]==[1,3,4]


def test_gate_uses_active_map_depth_not_gt_or_detector_score():
    row={'group_id':201326,'map_packet':{'rejected_heads':[dict(head_id='C1256W000052',
         reason='projection_uncertainty_exceeds_bound',depth_m=8.8)]}}
    assert gate_proposal(row)
    row['map_packet']['rejected_heads'][0]['reason']='not_front_facing'
    assert not gate_proposal(row)
    row['map_packet']['rejected_heads'][0]['reason']='projection_uncertainty_exceeds_bound'
    row['map_packet']['rejected_heads'][0]['depth_m']=16
    assert not gate_proposal(row)
    assert not gate_proposal({'group_id':0})


def test_union_keeps_baseline_and_adds_no_duplicate_housing():
    a=dict(xyxy=[0,0,20,10],confidence=.8)
    same=dict(xyxy=[1,1,19,9],confidence=.9)
    other=dict(xyxy=[100,100,120,110],confidence=.7)
    assert merge_extra([a],[same,other])==[a,other]
    assert merge_extra([],[other])==[other]


def test_distribution_reports_median_and_tail_separately():
    result=distribution([1,2,3,100])
    assert result['p50']==2.5 and result['p95']==100
    assert distribution([])=={'n':0}


def test_native_truth_requires_both_verified_brackets_and_not_requested_state():
    from copy import deepcopy
    import pytest
    from evaluate_traffic_light_near_development_matrix import native_truth
    check=dict(verified=True,readback=dict(readback_success=True,observations=[
        dict(tl_id='D',tl_color=33,identity_matches_request=True,color_usable_for_gt=True)]))
    row=dict(requested_state=16,native_GT_verified=True,native_gt=dict(jpeg_receipt_bracketed=True,
        observed_native_id='D',observed_state=33,before=deepcopy(check),after=deepcopy(check)))
    assert native_truth(row)==('D',33,[1,0,0,1])
    row['native_gt']['after']['readback']['observations'][0]['tl_color']=16
    with pytest.raises(ValueError,match='disagree'):
        native_truth(row)


def test_crop_matches_runtime_integer_truncation_and_clipping():
    import numpy as np
    from evaluate_traffic_light_near_development_matrix import runtime_crop
    image=np.zeros((50,100,3),np.uint8)
    crop,bounds=runtime_crop(image,[2.9,3.9,90.9,45.9])
    assert bounds==[0,0,100,50] and crop.shape==(50,100,3)
