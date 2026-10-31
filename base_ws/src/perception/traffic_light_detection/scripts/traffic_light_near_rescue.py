#!/usr/bin/env python3
"""Pure opt-in decision for a bounded second detector scale, never a GO rule.

The caller must already have validated an exact-image, fresh, current-route map
packet and must run the ordinary full identity association again after retry.
This module has no clocks, model calls, ROS imports, classifier fallback, or
permission to use a different head. Missing/stale map data must not be supplied
as if it were a validated packet. A CNN UNKNOWN is not a detector-miss retry.
"""
from dataclasses import dataclass
import math
from numbers import Real

from traffic_light_map_selector import MISSION_GROUP_IDS


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError('A finite numeric value is required')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('A finite numeric value is required')
    return result


@dataclass(frozen=True)
class NearRescueConfig:
    enabled: bool = False
    imgsz: int = 640
    max_depth_m: float = 15.
    min_projected_width_px: float = 60.

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError('Near rescue enabled must be an explicit bool')
        if type(self.imgsz) is not int or self.imgsz != 640:
            raise ValueError('Near rescue uses the fixed 640 detector scale')
        depth = _finite_number(self.max_depth_m)
        width = _finite_number(self.min_projected_width_px)
        if not 3. <= depth <= 15.:
            raise ValueError('Near rescue depth must be within 3..15 m')
        if not 32. <= width <= 1280.:
            raise ValueError('Near rescue projected width must be within 32..1280 px')


DEFAULT_NEAR_RESCUE_CONFIG = NearRescueConfig()


def near_rescue_config(overrides=None):
    """Normalize a strict configuration object; never enable by truthiness."""
    if overrides is None:
        return DEFAULT_NEAR_RESCUE_CONFIG
    if isinstance(overrides, NearRescueConfig):
        return overrides
    if not isinstance(overrides, dict) or set(overrides) - {
            'enabled', 'imgsz', 'max_depth_m', 'min_projected_width_px'}:
        raise ValueError('Unknown or non-object near rescue configuration')
    return NearRescueConfig(**overrides)


def _valid_head_id(value):
    return isinstance(value, str) and bool(value) and value == value.strip() and len(value) <= 128


def _valid_rejections(selection):
    rejected = selection.get('rejected', [])
    if not isinstance(rejected, list) or any(
            not isinstance(item, dict) or not isinstance(item.get('reason'), str)
            for item in rejected):
        return None
    return rejected


def _eligible_roi(roi, group_id, config):
    if (roi.get('eligible') is not True or type(roi.get('group_id')) is not int
            or roi['group_id'] != group_id or not _valid_head_id(roi.get('head_id'))):
        return None
    try:
        depth = _finite_number(roi['depth_m'])
        bbox = roi['bbox_xyxy']
        if not isinstance(bbox, (tuple, list)) or len(bbox) != 4:
            return None
        x1, y1, x2, y2 = [_finite_number(value) for value in bbox]
        width, height = x2 - x1, y2 - y1
        if (not 3. <= depth <= config.max_depth_m or x1 < 0 or y1 < 0
                or width < config.min_projected_width_px or height <= 0):
            return None
        return dict(head_id=roi['head_id'], depth_m=depth,
                    projected_width_px=width, projected_height_px=height)
    except (KeyError, TypeError, ValueError, OverflowError):
        return None


def near_rescue_decision(selection, rois, group_id, config=DEFAULT_NEAR_RESCUE_CONFIG):
    """Return retry metadata only for an unambiguous near/large map-head miss.

    ``selection`` is the FIRST ordinary detector/association result, not a CNN
    classification result. Existing matched detections, other-head ambiguity,
    explicit legacy/stale/invalid results, and far/small heads cannot qualify.
    The returned eligible IDs constrain what the second full association may
    accept; they do not replace its ordinary geometry/identity checks.
    """
    config = near_rescue_config(config)
    result = dict(retry=False, reason='disabled', eligible_head_ids=[],
                  imgsz=config.imgsz, group_id=group_id)

    def reject(reason):
        result['reason'] = reason
        return result

    if not config.enabled:
        return result
    if type(group_id) is not int or group_id not in MISSION_GROUP_IDS:
        return reject('invalid_mission_group')
    if not isinstance(selection, dict):
        return reject('invalid_first_selection')
    if 'mode' in selection and selection['mode'] != 'map_association':
        return reject('selection_mode_not_map_association')
    if selection.get('status') != 'missing':
        return reject('first_selection_not_missing')
    if selection.get('reason') != 'no_unambiguous_route_detection':
        return reject('first_selection_not_retryable')
    if selection.get('projection_reason') not in (None, 'projected'):
        return reject('projection_not_ready')
    rejected = _valid_rejections(selection)
    if rejected is None:
        return reject('invalid_selection_rejections')
    if any(item['reason'] == 'other_head_identity_ambiguous' for item in rejected):
        return reject('other_head_identity_ambiguous')
    if not isinstance(rois, (tuple, list)) or any(not isinstance(roi, dict) for roi in rois):
        return reject('invalid_map_rois')
    candidates = [candidate for roi in rois for candidate in [_eligible_roi(roi, group_id, config)]
                  if candidate is not None]
    if not candidates:
        return reject('no_near_large_route_head')
    result.update(retry=True, reason='near_large_route_head_retry',
                  eligible_head_ids=sorted({item['head_id'] for item in candidates}),
                  eligible_heads=candidates)
    return result


def accept_near_rescue_selection(selection, decision):
    """A retry may accept only a fully associated, originally eligible head.

    The ordinary association must already have considered every identity
    competitor. This predicate does not accept a raw detector box or CNN state.
    """
    if (not isinstance(decision, dict) or decision.get('retry') is not True
            or not isinstance(selection, dict) or selection.get('status') != 'matched'
            or selection.get('reason') != 'map_head_association'):
        return False
    group = decision.get('group_id')
    head = selection.get('head_id')
    eligible = decision.get('eligible_head_ids')
    roi = selection.get('roi')
    if (type(group) is not int or group not in MISSION_GROUP_IDS or not _valid_head_id(head)
            or not isinstance(eligible, list) or any(not _valid_head_id(item) for item in eligible)
            or head not in eligible or not isinstance(roi, dict)):
        return False
    return (roi.get('eligible') is True and type(roi.get('group_id')) is int
            and roi['group_id'] == group and roi.get('head_id') == head)
