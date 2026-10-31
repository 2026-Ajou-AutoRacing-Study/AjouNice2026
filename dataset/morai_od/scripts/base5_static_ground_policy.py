"""Explicit single-capture static QA; uncertain terrain is not ground certification."""
import math

POLICY = 'static-single-capture-advisory-v115'


def lightweight_static(metadata):
    mode = metadata.get('ground_support_check', {}).get('static_policy')
    if mode is None:
        return False
    if mode != POLICY:
        raise ValueError('Unknown static ground policy: '+str(mode))
    # Dynamic calibration can itself contain standing actors: never weaken it.
    return (not metadata.get('calibration_only') and
            all(a.get('motion') in ('static', 'standing')
                for a in metadata.get('runtime_actors', [])))


def static_decision(ground, metadata=None):
    warnings, fatal = [], []
    for row in ground.get('actors', []):
        reason = row.get('reason')
        if not reason:
            continue
        gap = row.get('minimum_annotation_ground_gap_m')
        if reason in ('actor_annotation_above_ground', 'actor_annotation_deep_below_ground'):
            if gap is None or not math.isfinite(gap) or not row.get('plane'):
                fatal.append('static_ground_invalid_measurement')
            elif metadata is not None and reason == 'actor_annotation_deep_below_ground' and gap < -.40:
                from base5_ground_support import static_surface_association
                association = static_surface_association(row, ground, metadata)
                if association is not None:
                    warnings.append(dict(uid=row['uid'], reason='actor_ground_surface_association_unverified',
                        gap_m=gap, surface_association=association))
                else:
                    fatal.append('static_ground_severe_below')
            elif gap > .40:
                fatal.append('static_ground_severe_above')
            elif gap < -.40:
                fatal.append('static_ground_severe_below')
            else:
                warnings.append(dict(uid=row['uid'], reason=reason, gap_m=gap))
        elif reason in ('actor_ground_plane_unverified', 'outside_local_ground_probe_range'):
            warnings.append(dict(uid=row['uid'], reason=reason))
        else:
            # Missing GT or inconsistent provenance is not merely uncertain ground.
            fatal.append(reason)
    return dict(policy=POLICY, review_reasons=sorted(set(fatal)), warnings=warnings,
                severe_above_m=.40, severe_below_m=-.40,
                extra_ground_captures_allowed=False,
                limitation='Original local-plane results retained; warnings are not contact certification. No sensor/GT geometry edits.')
