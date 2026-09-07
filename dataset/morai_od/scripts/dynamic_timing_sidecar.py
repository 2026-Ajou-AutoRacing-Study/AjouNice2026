#!/usr/bin/env python3
"""Pure helpers for compact temporal-capture timing sidecars."""

from bisect import bisect_left
import math


def samples_in_window(samples, start_monotonic, end_monotonic, padding_sec=0.0):
    """Return samples whose callback time overlaps a padded clip window."""

    lower = float(start_monotonic) - float(padding_sec)
    upper = float(end_monotonic) + float(padding_sec)
    return [
        sample
        for sample in samples
        if lower <= float(sample["callback_monotonic"]) <= upper
    ]


def nearest_sample(samples, target_monotonic):
    """Return a compact reference to the nearest timestamped sample."""

    if not samples:
        return None
    times = [float(sample["callback_monotonic"]) for sample in samples]
    index = bisect_left(times, float(target_monotonic))
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    selected = min(
        candidates,
        key=lambda item: abs(
            float(item["callback_monotonic"]) - float(target_monotonic)
        ),
    )
    return {
        "sample_id": selected["sample_id"],
        "callback_monotonic": selected["callback_monotonic"],
        "callback_unix": selected["callback_unix"],
        "offset_from_trigger_sec": (
            float(selected["callback_monotonic"]) - float(target_monotonic)
        ),
        "message_timestamp": selected.get("message_timestamp"),
        "simulation_time": selected.get("simulation_time"),
    }


def bind_trigger_timing(triggers, histories):
    """Attach nearest sidecar sample references to capture triggers."""

    for trigger in triggers:
        target = float(trigger["triggered_monotonic"])
        trigger["timing_samples"] = {
            stream: nearest_sample(samples, target)
            for stream, samples in sorted(histories.items())
        }


def timing_summary(histories, configured_topics):
    """Summarize availability without loading sidecar payloads downstream."""

    result = {}
    for stream, topic in sorted(configured_topics.items()):
        samples = histories.get(stream, [])
        result[stream] = {
            "topic": topic,
            "available": bool(samples),
            "sample_count": len(samples),
            "first_message_timestamp": (
                samples[0].get("message_timestamp") if samples else None
            ),
            "last_message_timestamp": (
                samples[-1].get("message_timestamp") if samples else None
            ),
        }
    return result


def stable_ego_motion(samples, minimum_window_sec, maximum_acceleration_mps2):
    """Return whether recent Ego samples have stable longitudinal speed.

    MORAI acceleration is expressed in world axes.  Its vector magnitude also
    contains the centripetal acceleration of an otherwise steady-speed turn,
    so using that magnitude rejects valid roundabout clips.  Project the
    acceleration onto the velocity direction and retain the old magnitude
    fallback only when velocity was not serialized by an older sidecar.
    """

    if len(samples) < 2:
        return False
    span = float(samples[-1]["callback_monotonic"]) - float(
        samples[0]["callback_monotonic"]
    )
    if span + 1e-6 < float(minimum_window_sec):
        return False
    for sample in samples:
        state = sample.get("state") or {}
        acceleration = state.get("acceleration_raw") or {}
        velocity = state.get("velocity_raw") or {}
        velocity_components = [
            float(velocity.get(axis, 0.0)) for axis in ("x", "y", "z")
        ]
        acceleration_components = [
            float(acceleration.get(axis, 0.0)) for axis in ("x", "y", "z")
        ]
        speed = math.sqrt(sum(value * value for value in velocity_components))
        if speed > 1e-3:
            longitudinal_acceleration = abs(
                sum(
                    accel * component
                    for accel, component in zip(
                        acceleration_components, velocity_components
                    )
                )
                / speed
            )
        else:
            longitudinal_acceleration = math.sqrt(
                sum(value * value for value in acceleration_components)
            )
        if longitudinal_acceleration > float(maximum_acceleration_mps2):
            return False
    return True
