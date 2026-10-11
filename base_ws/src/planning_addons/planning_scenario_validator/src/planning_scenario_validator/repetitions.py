"""Small, reproducible parameter sweeps; never change actor placement or routes.

MORAI's Variable editor does not necessarily persist its sampling bounds in a
scenario. Sample these explicitly, once per repetition, and log the realized
JSON. This seed controls our parameters, not MORAI's native RandomMode models.
"""
import copy
import hashlib
import math
import random


def repetition_settings(scenarios):
    count = scenarios.get("repeat", 1)
    if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 1000:
        raise ValueError("scenarios.repeat는 1~1000 사이 정수입니다 (JSON 한 개당 실행 횟수)")
    spec = scenarios.get("randomization", {})
    if not isinstance(spec, dict):
        raise ValueError("scenarios.randomization은 mapping이어야 합니다")
    unknown = set(spec) - {"enabled", "seed", "vehicle_speed_kmh", "path_offset_percent"}
    if unknown:
        raise ValueError("randomization에 지원하지 않는 항목: " + ", ".join(sorted(unknown)))
    if not isinstance(spec.get("enabled", False), bool):
        raise ValueError("randomization.enabled는 true/false입니다")
    seed = spec.get("seed", 0)
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed <= 2**32 - 1:
        raise ValueError("randomization.seed는 0~4294967295 사이 정수입니다")
    for field, lower, upper in (("vehicle_speed_kmh", 0, 150),
                                ("path_offset_percent", -100, 100)):
        if field not in spec:
            continue
        bounds = spec[field]
        if (not isinstance(bounds, list) or len(bounds) != 2
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in bounds)
                or not lower <= bounds[0] <= bounds[1] <= upper):
            raise ValueError("randomization.{}는 [{}, {}] 안의 [최소, 최대]입니다".format(
                field, lower, upper))
    if spec.get("enabled") and not (set(spec) & {"vehicle_speed_kmh", "path_offset_percent"}):
        raise ValueError("randomization을 켜려면 속도 또는 path offset 범위를 지정하세요")
    return count, copy.deepcopy(spec)


def sampled_scene(scene, *, source_sha256, repeat_index, settings):
    """Same input hash/seed/repetition gives the same overrides, even on resume."""
    result = copy.deepcopy(scene)
    seed = int(settings.get("seed", 0))
    digest = hashlib.sha256("{}:{}:{}".format(seed, source_sha256, repeat_index).encode()).hexdigest()
    rng = random.Random(int(digest, 16))
    audit = {"enabled": bool(settings.get("enabled", False)), "seed": seed,
             "repeat_index": repeat_index, "sampling_key": digest, "changes": [],
             "note": "이 seed는 validator 파라미터만 고정합니다. MORAI RandomMode 차종/physics는 고정하지 않습니다."}
    if not audit["enabled"]:
        return result, audit
    for container in ("vehicleList", "spawnPointList"):
        for actor in result.get(container, []):
            updates = {}
            if "vehicle_speed_kmh" in settings:
                speed = round(rng.uniform(*settings["vehicle_speed_kmh"]), 3)
                if container == "vehicleList":
                    updates.update(velocityType=2, desiredVelocity=speed, constantVelocity=speed)
                else:
                    updates.update(DesiredVelocityType=2, MinDesiredVelocity_Custom=speed,
                                   MaxDesiredVelocity_Custom=speed)
            if "path_offset_percent" in settings:
                offset = round(rng.uniform(*settings["path_offset_percent"]), 3)
                if container == "vehicleList":
                    updates.update(latBiasMode=2, bias=offset)
                else:
                    # Spawn bounds, like placed NPC bias, use percentages.
                    # Fractions would silently shrink requested offsets by 100x.
                    updates.update(latBiasMode=2, MinLatBias=offset, MaxLatBias=offset)
            audit["changes"].append({"container": container, "unique_id": actor.get("UNIQUEID"),
                                     "before": {key: actor.get(key) for key in updates},
                                     "after": updates})
            actor.update(updates)
    return result, audit
