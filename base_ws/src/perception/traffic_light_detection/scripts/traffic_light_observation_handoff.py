"""Pure eligibility check for a short, same-housing detector dropout.

This never supplies a lamp or extends its timestamp. SignalPhasePolicy owns
the small source-age budget; invalid geometry/identity is not a detector miss.
"""
import math

def detection_miss_context(packet, selection, previous_head_id, group_id,
                           source_stamp_ns, now_ns, max_packet_age_sec=.5):
    result = {"eligible": False, "head_id": previous_head_id,
              "reason": "not_a_qualified_detector_miss"}
    if not previous_head_id or not isinstance(packet, dict):
        return result
    try:
        if (packet.get("status") != "ready" or int(group_id) <= 0
                or packet.get("group_id") != int(group_id)
                or packet.get("source_stamp_ns") != int(source_stamp_ns)
                or source_stamp_ns <= 0
                or not 0 <= (now_ns-packet["generated_stamp_ns"])/1e9 <= max_packet_age_sec
                or not 0 <= (now_ns-source_stamp_ns)/1e9 <= max_packet_age_sec):
            return result
        if (selection.get("status") != "missing"
                or selection.get("reason") != "no_unambiguous_route_detection"):
            return result
        # A box explicitly ambiguous with another mapped head is not absent.
        if any(row.get("reason") != "no_route_head_match"
               for row in selection.get("rejected", [])):
            result["reason"] = "identity_rejection_not_detector_dropout"
            return result
        same_head = [r for r in packet.get("rois", [])
                     if r.get("eligible") is True and r.get("group_id") == int(group_id)
                     and r.get("head_id") == previous_head_id]
        if len(same_head) != 1:
            result["reason"] = "previous_housing_not_currently_projectable"
            return result
    except (KeyError, TypeError, ValueError, OverflowError):
        return result
    return dict(eligible=True, head_id=previous_head_id,
                reason="fresh_same_housing_detector_dropout")


def low_confidence_context(packet, selection, classifier, previous_head_id, group_id,
                           source_stamp_ns, now_ns, max_packet_age_sec=.5):
    """Qualify only a moving, identified housing's soft CNN abstention.

    This does not hold a permission, invent lamps, or renew an observation clock.
    All-off, contradictory lamps, inference errors and geometry loss are excluded.
    """
    result = dict(eligible=False, head_id=previous_head_id, reason="not_a_qualified_low_confidence_gap")
    try:
        if (not previous_head_id or not isinstance(packet, dict) or not isinstance(classifier, dict)
                or not isinstance(selection, dict)
                or packet.get('status') != 'ready' or type(group_id) is not int or group_id <= 0
                or packet.get('group_id') != group_id
                or type(source_stamp_ns) is not int or source_stamp_ns <= 0
                or packet.get('source_stamp_ns') != source_stamp_ns
                or not 0 <= (now_ns-packet['generated_stamp_ns'])/1e9 <= max_packet_age_sec
                or not 0 <= (now_ns-source_stamp_ns)/1e9 <= max_packet_age_sec
                or selection.get('status') != 'matched'
                or selection.get('head_id') != previous_head_id
                or classifier.get('known') is not False
                or not isinstance(classifier.get('reason'), str)
                or not classifier['reason'].startswith('low_confidence_bits:')):
            return result
        same_head = [r for r in packet.get('rois', []) if r.get('eligible') is True
                     and r.get('group_id') == group_id and r.get('head_id') == previous_head_id]
        if len(same_head) != 1:
            return result
        bits = classifier.get('bits')
        if (not isinstance(bits, dict) or set(bits) != {'red','yellow','green','left_arrow'}
                or any(value is not None and type(value) is not int for value in bits.values())
                or any(value not in (0,1,None) for value in bits.values())
                or None not in bits.values() or bits['yellow'] == 1):
            return result
        motion = packet.get('motion_evidence', {})
        if not isinstance(motion, dict) or motion.get('schema_version') != 1 or motion.get('valid') is not True:
            result['reason'] = 'motion_evidence_missing_or_invalid'
            return result
        for key in ('aligned_speed_mps', 'latest_speed_mps'):
            value = motion[key]
            if (isinstance(value, bool) or not isinstance(value, (int,float))
                    or not math.isfinite(value) or value < 1.):
                result['reason'] = 'not_confirmed_moving'
                return result
        for key in ('aligned_pose_stamp_ns', 'latest_pose_stamp_ns', 'latest_pose_receipt_stamp_ns'):
            value = motion[key]
            if type(value) is not int or value <= 0 or not 0 <= now_ns-value <= 500_000_000:
                return result
        if (abs(source_stamp_ns-motion['aligned_pose_stamp_ns']) > 150_000_000
                or now_ns-motion['latest_pose_receipt_stamp_ns'] > 250_000_000
                or motion['latest_pose_stamp_ns'] < motion['aligned_pose_stamp_ns']):
            return result
    except (KeyError, TypeError, ValueError, OverflowError):
        return result
    return dict(eligible=True, head_id=previous_head_id,
                reason='fresh_same_housing_moving_low_confidence', confident_red=bits['red'] == 1)
