"""Pure eligibility check for a short, same-housing detector dropout.

This never supplies a lamp or extends its timestamp. SignalPhasePolicy owns
the small source-age budget; invalid geometry/identity is not a detector miss.
"""
import math


class ObservationDistanceGate:
    """Suppress lamps until a current matched head is within its mission limit.

    Uses camera-forward map depth, not distance to a stop line. Once opened,
    ordinary identity/freshness/colour rules remain responsible for observations.
    Reset on mission change or clock rewind; same-mission teleport is not covered
    by this latch. An explicit zero base threshold disables all mission gates.
    """
    def __init__(self, threshold_m=0., d_threshold_m=None):
        for value in (threshold_m, threshold_m if d_threshold_m is None else d_threshold_m):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 120:
                raise ValueError('observation start depth must be finite in 0..120m')
        self.threshold_m = float(threshold_m)
        self.d_threshold_m = self.threshold_m if d_threshold_m is None else float(d_threshold_m)
        self.group = None
        self.last_stamp = None
        self.opened = False

    def update(self, group, source_ns, selection):
        if self.threshold_m == 0:
            return dict(suppressed=False, enabled=False, reason='disabled')
        threshold = self.d_threshold_m if group == 201326 else self.threshold_m
        if group != self.group or (type(source_ns) is int and self.last_stamp is not None and source_ns < self.last_stamp):
            self.group, self.last_stamp, self.opened = group, None, False
        valid_stamp = type(source_ns) is int and source_ns > 0
        if valid_stamp:
            self.last_stamp = source_ns
        roi = selection.get('roi') or {}
        depth = roi.get('depth_m')
        valid_depth = (valid_stamp and type(group) is int and group in (201351, 201365, 201290, 201326)
            and selection.get('status') == 'matched' and selection.get('group_id') == group
            and selection.get('source_stamp_ns') == source_ns and roi.get('group_id') == group
            and isinstance(depth, (int, float)) and not isinstance(depth, bool)
            and math.isfinite(depth) and depth > 0)
        entered = not self.opened and valid_depth and depth <= threshold
        if entered:
            self.opened = True
        return dict(enabled=True, suppressed=not (self.opened and valid_stamp),
            activated=self.opened, just_opened=entered, threshold_m=threshold,
            current_matched_camera_depth_m=float(depth) if valid_depth else None,
            reason='active_latched' if self.opened else 'awaiting_near_current_match')


def no_lamps_context(packet, selection, classifier, previous_head_id, group_id,
                     source_stamp_ns, now_ns, max_packet_age_sec=.5, *, include_soft=False):
    """Qualify a dark crop, or opt-in soft abstention, of the same housing.

    The policy supplies the small, nonrenewable time budget. No box, uncertain
    another head or absent geometry are not this particular dropout. Confident
    positive bits in a soft abstention must be checked against cached lamps.
    """
    result = dict(eligible=False, head_id=previous_head_id,
                  reason='not_a_qualified_no_lamps_gap')
    try:
        if not isinstance(classifier, dict):
            return result
        bits = classifier.get('bits')
        if (not isinstance(bits, dict) or set(bits) != {'red','yellow','green','left_arrow'}
                or any(v is not None and type(v) is not int for v in bits.values())
                or any(v not in (0, 1, None) for v in bits.values())):
            return result
        dark = (classifier.get('known') is True and classifier.get('reason') == 'no_lamps'
                and all(v == 0 for v in bits.values()))
        soft = (include_soft and classifier.get('known') is False
                and str(classifier.get('reason', '')).startswith('low_confidence_bits:')
                and None in bits.values())
        if (not previous_head_id or not isinstance(packet, dict)
                or not isinstance(selection, dict) or not isinstance(classifier, dict)
                or packet.get('status') != 'ready' or type(group_id) is not int or group_id <= 0
                or packet.get('group_id') != group_id or type(source_stamp_ns) is not int
                or source_stamp_ns <= 0 or packet.get('source_stamp_ns') != source_stamp_ns
                or not 0 <= (now_ns-packet['generated_stamp_ns'])/1e9 <= max_packet_age_sec
                or not 0 <= (now_ns-source_stamp_ns)/1e9 <= max_packet_age_sec
                or selection.get('status') != 'matched'
                or selection.get('reason') != 'map_head_association'
                or selection.get('head_id') != previous_head_id
                or not (dark or soft) or classifier.get('tuples') != []):
            return result
        same_head = [r for r in packet.get('rois', []) if r.get('eligible') is True
                     and r.get('group_id') == group_id and r.get('head_id') == previous_head_id]
        if len(same_head) != 1:
            return result
    except (KeyError, TypeError, ValueError, OverflowError):
        return result
    confident = [(('green', 'arrow') if key == 'left_arrow' else (key, 'circle'))
                 for key, value in bits.items() if value == 1]
    return dict(eligible=True, head_id=previous_head_id,
                reason='fresh_same_housing_no_lamps' if dark else 'fresh_same_housing_soft_gap',
                gap_kind='no_lamps' if dark else 'low_confidence', confident_lamps=confident)

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
