"""Pure eligibility check for a short, same-housing detector dropout.

This never supplies a lamp or extends its timestamp. SignalPhasePolicy owns
the small source-age budget; invalid geometry/identity is not a detector miss.
"""

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
