"""Identity-only continuation. No ROS/GPU or synthetic green permission."""
import copy
from pathlib import Path
import sys
import pytest

PACKAGE=Path(__file__).resolve().parents[1];sys.path.insert(0,str(PACKAGE/'scripts'))
from traffic_light_visual_continuation import VisualContinuation, _current_nominal_identity
from traffic_light_map_selector import SELECTOR_CONTRACT_VERSION


def box(coords=(100.,50.,180.,70.),confidence=.9):
    a,b,c,d=coords
    return ((a+c)/2560,(b+d)/1440,(c-a)/1280,(d-b)/720,(a,b,c,d,confidence))


def packet(source=10_100_000_000):
    roi=dict(head_id='D052',group_id=201326,eligible=False,depth_m=10.,
        normal_rejection_reason='localization_uncertainty_exceeds_bound',
        bbox_xyxy=[100.,50.,180.,70.],gate_xyxy=[10.,-150.,270.,270.])
    return dict(status='missing',reason='localization_uncertainty_exceeds_bound',group_id=201326,
        source_stamp_ns=source,generated_stamp_ns=source,selector_contract_version=SELECTOR_CONTRACT_VERSION,
        nominal_continuation=dict(schema_version=1,status='ready',normal_eligibility=False,
            validated_image_time_tf_pose_extrinsic=True,actual_xy_sigma_m=.03,actual_z_sigma_m=1.,
            actual_yaw_sigma_degrees=3.2,rois=[roi]))


def tracker():
    value=VisualContinuation()
    value.remember(box(),dict(status='matched',reason='map_head_association',head_id='D052'),
                   201326,10_000_000_000,10_010_000_000)
    return value


def select(value=None,p=None,boxes=None,source=10_100_000_000,now=10_110_000_000,window=.3):
    return (value or tracker()).select([box()] if boxes is None else boxes,packet(source) if p is None else p,
        dict(status='missing',reason='localization_uncertainty_exceeds_bound'),201326,source,now,window)


def test_unique_same_box_is_current_detection_not_old_lamp_hold():
    selected,decision=select()
    assert selected==box();assert decision['status']=='matched'
    assert decision['head_id']=='D052';assert decision['normal_eligibility'] is False
    assert not any('lamp' in k for k in decision)


def test_no_prior_normal_map_match_cannot_create_identity():
    selected,_=select(VisualContinuation())
    assert selected is None


def test_fallback_spam_cannot_renew_anchor_processing_or_original_source_deadline():
    value=tracker()
    for source,now in ((10_100_000_000,10_110_000_000),(10_280_000_000,10_290_000_000)):
        assert select(value,source=source,now=now)[0] is not None
    assert select(value,source=10_310_000_000,now=10_320_000_000)[0] is None
    value=VisualContinuation();value.remember(box(),dict(status='matched',reason='map_head_association',head_id='D052'),
        201326,9_800_000_000,10_010_000_000)
    assert select(value,source=10_300_000_000,now=10_310_000_000)[0] is None # original source age .51


@pytest.mark.parametrize('coords',((200.,50.,280.,70.),(100.,50.,300.,70.),(101.,50.,181.,55.)))
def test_visual_jump_or_size_change_is_not_same_housing(coords):
    assert select(boxes=[box(coords)])[0] is None


def test_other_nominal_head_and_genuinely_distinct_second_box_are_ambiguous():
    p=packet();competitor=copy.deepcopy(p['nominal_continuation']['rois'][0]);competitor['head_id']='other'
    p['nominal_continuation']['rois'].append(competitor)
    assert select(p=p)[0] is None
    assert select(boxes=[box(),box((115.,50.,195.,70.))])[0] is None


def test_near_duplicate_primary_retry_boxes_do_not_manufacture_ambiguity():
    assert select(boxes=[box(),box((101.,50.,181.,70.),.8)])[0] is not None


@pytest.mark.parametrize('reason',('housing_not_fully_in_image','not_front_facing','depth_out_of_bounds',
                                 'housing_too_small','crossing_camera_plane'))
def test_geometric_rejections_are_never_uncertainty_bridged(reason):
    p=packet();p['nominal_continuation']['rois'][0]['normal_rejection_reason']=reason
    assert select(p=p)[0] is None


def test_stale_tf_route_missing_bad_covariance_wrong_stamp_or_normal_packet_cannot_bridge():
    for modify in (lambda p:p.update(reason='image-time TF unavailable'),
                   lambda p:p.update(group_id=0),lambda p:p.update(source_stamp_ns=10_100_000_001),
                   lambda p:p.pop('nominal_continuation'),
                   lambda p:p['nominal_continuation'].update(validated_image_time_tf_pose_extrinsic=False),
                   lambda p:p['nominal_continuation'].update(actual_yaw_sigma_degrees=5.1),
                   lambda p:p['nominal_continuation'].update(actual_xy_sigma_m=1.6),
                   lambda p:p['nominal_continuation'].update(actual_z_sigma_m=float('nan'))):
        p=packet();modify(p);assert select(p=p)[0] is None


def test_zero_switch_future_source_and_group_change_do_not_reuse_anchor():
    assert select(window=0.)[0] is None
    assert select(source=10_200_000_000,now=10_100_000_000)[0] is None
    value=tracker();value.select([box()],packet(),{},201351,10_100_000_000,10_110_000_000,.3)
    assert select(value,source=10_200_000_000,now=10_210_000_000)[0] is None


def test_small_boundary_jitter_matches_identity_without_modifying_the_actual_crop_box():
    previous=(573.651123046875,197.08786010742188,619.717041015625,213.0654754638672)
    current=(571.5834350585938,191.4398193359375,619.411376953125,207.56539916992188)
    value=VisualContinuation();value.remember(box(previous),
        dict(status='matched',reason='map_head_association',head_id='D052'),201326,10_000_000_000,10_010_000_000)
    p=packet();p['nominal_continuation']['rois'][0].update(
        bbox_xyxy=list(previous),gate_xyxy=[450.,50.,750.,400.])
    selected,decision=select(value,p=p,boxes=[box(current)])
    assert selected==box(current)
    assert decision['raw_identity_iou']<.5
    assert decision['identity_iou']>=.5
    assert decision['identity_boundary_tolerance_px']<=2.
    assert decision['identity_boundary_tolerance_px']<=.1*(previous[3]-previous[1])


def test_current_nominal_projection_recovers_fast_approach_to_same_small_head():
    # At B, a 20x7px housing can move several pixels in one camera frame.
    # Prior-box IoU is then low although the current image-time map projection
    # and the current YOLO detection still point at the same isolated head.
    prior=(518.,253.,538.,261.)
    current=(517.,258.,539.,266.)
    value=VisualContinuation();value.remember(box(prior),
        dict(status='matched',reason='map_head_association',head_id='D052'),
        201326,10_000_000_000,10_010_000_000)
    p=packet();p['nominal_continuation']['rois'][0].update(
        bbox_xyxy=[521.,262.,540.,269.],gate_xyxy=[400.,200.,650.,330.])
    selected,decision=select(value,p=p,boxes=[box(current)])
    assert selected==box(current)
    assert decision['identity_iou']<.5
    assert decision['identity_basis']=='current_nominal_projection'
    assert not any('lamp' in key for key in decision)


def test_current_projection_tolerance_shrinks_for_a_small_housing():
    projected=(521.,262.,540.,269.)
    assert _current_nominal_identity((520.,263.,539.,270.),projected)
    # Six-pixel vertical center error is inside the old fixed 8px allowance,
    # but is too large for a seven-pixel-high projected housing.
    assert not _current_nominal_identity((520.,268.,539.,275.),projected)


def test_current_projection_is_not_a_broad_uncertainty_gate_or_tf_fallback():
    prior=(518.,253.,538.,261.)
    value=VisualContinuation();value.remember(box(prior),
        dict(status='matched',reason='map_head_association',head_id='D052'),
        201326,10_000_000_000,10_010_000_000)
    p=packet();p['nominal_continuation']['rois'][0].update(
        bbox_xyxy=[521.,262.,540.,269.],gate_xyxy=[400.,200.,650.,330.])
    # Both boxes lie in the huge uncertainty gate, but neither agrees with
    # the tight current projection, so neither may inherit the old identity.
    assert select(value,p=p,boxes=[box((500.,258.,522.,266.))])[0] is None
    assert select(value,p=p,boxes=[box((517.,240.,539.,248.))],
                  source=10_200_000_000,now=10_210_000_000)[0] is None
    # An exact current projection is insufficient without image-time TF proof.
    p=packet();p['nominal_continuation']['rois'][0].update(
        bbox_xyxy=[521.,262.,540.,269.],gate_xyxy=[400.,200.,650.,330.])
    p['nominal_continuation']['validated_image_time_tf_pose_extrinsic']=False
    assert select(tracker(),p=p,boxes=[box((517.,258.,539.,266.))])[0] is None


def test_current_projection_rejects_competing_mapped_head():
    prior=(518.,253.,538.,261.)
    value=VisualContinuation();value.remember(box(prior),
        dict(status='matched',reason='map_head_association',head_id='D052'),
        201326,10_000_000_000,10_010_000_000)
    p=packet();target=p['nominal_continuation']['rois'][0]
    target.update(bbox_xyxy=[521.,262.,540.,269.],gate_xyxy=[400.,200.,650.,330.])
    competitor=copy.deepcopy(target);competitor['head_id']='other-head'
    p['nominal_continuation']['rois'].append(competitor)
    selected,decision=select(value,p=p,boxes=[box((517.,258.,539.,266.))])
    assert selected is None
    assert decision['reason']=='visual_continuation_other_head_competitor'


def test_boundary_tolerance_cannot_jump_to_nearby_distinct_or_competing_housing():
    # Nine-pixel vertical displacement remains outside even the bounded IoU margin.
    assert select(boxes=[box((100.,41.,180.,61.))])[0] is None
    # A closer competing mapped head is still rejected, even if padded IoU passes.
    p=packet();competitor=copy.deepcopy(p['nominal_continuation']['rois'][0])
    competitor['head_id']='other-nearby';p['nominal_continuation']['rois'].append(competitor)
    assert select(p=p,boxes=[box((100.,45.,180.,65.))])[0] is None


def test_invalid_detector_confidence_never_enters_visual_identity_selection():
    for confidence in (float('nan'),float('inf'),-.1,1.1,True,None,'nan'):
        assert select(boxes=[box(confidence=confidence)])[0] is None


def test_three_duplicate_candidates_must_all_be_pairwise_equivalent():
    # Both side boxes overlap the central one by .818, but each other by .667.
    boxes=[box(),box((92.,50.,172.,70.)),box((108.,50.,188.,70.))]
    _,decision=select(boxes=boxes)
    assert decision['reason']=='visual_continuation_multiple_detections'
