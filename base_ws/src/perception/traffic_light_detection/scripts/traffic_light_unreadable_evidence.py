#!/usr/bin/env python3
"""Preserve observed colors without claiming an unreadable lamp's shape/state.

This is not normalization or permission: no color is inferred, no cached lamp
is used, and confidence is the unchanged current raw handoff confidence. The
caller separately requires an opt-in observed-only, current-qualified crop and
an otherwise UNKNOWN final result before putting these on the wire.
"""
import math


def unreadable_color_evidence(detected_tuples, classifier, confidence):
    if (isinstance(confidence, bool) or not isinstance(confidence, (int, float))
            or not math.isfinite(confidence) or not 0 <= confidence <= 1):
        return []
    sources = {color: [] for color in ('red', 'yellow', 'green')}
    if isinstance(detected_tuples, (tuple, list)):
        for lamp in detected_tuples:
            if (isinstance(lamp, (tuple, list)) and len(lamp) == 3
                    and isinstance(lamp[0], str) and lamp[0] in sources):
                if 'raw_observed_tuple' not in sources[lamp[0]]:
                    sources[lamp[0]].append('raw_observed_tuple')
    bits = classifier.get('bits') if isinstance(classifier, dict) else None
    if (isinstance(bits, dict) and set(bits) == {'red', 'yellow', 'green', 'left_arrow'}
            and all(value is None or type(value) is int and value in (0, 1)
                    for value in bits.values())):
        for key in ('red', 'yellow', 'green', 'left_arrow'):
            if bits[key] == 1:
                color = 'green' if key == 'left_arrow' else key
                sources[color].append('classifier_bit:' + key)
    return [dict(color=color, shape='unknown', status='unknown', confidence=confidence,
                 sources=evidence) for color, evidence in sources.items() if evidence]
