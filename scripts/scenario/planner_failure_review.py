"""Validate route-bound planner failure events before a simulator review stop."""
import json
import math


def parse_failure_event(payload, now, route_uuid, max_age=2.0):
    try:
        event = json.loads(payload)
        if not isinstance(event, dict) or not isinstance(route_uuid, str) or len(route_uuid) != 32:
            return None
        if event.get('route_uuid') != route_uuid:
            return None
        if not all(isinstance(event.get(k), str) and event[k].strip() for k in ('module', 'reason')):
            return None
        stamp = float(event['stamp'])
        if not math.isfinite(stamp) or not math.isfinite(now) or not -0.1 <= now-stamp <= max_age:
            return None
        return event
    except (ValueError, TypeError, KeyError):
        return None
