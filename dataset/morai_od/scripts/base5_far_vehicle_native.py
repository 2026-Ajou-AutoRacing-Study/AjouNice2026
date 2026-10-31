"""Native evidence for an explicitly allocated far secondary, not detector AP.

Do not require a dense LiDAR return from every 90--100m passenger car. A
current-input instance mask can support a camera-only target. Conversely a
visible primary must not hide the complete absence of the allocated far car.
"""
import math
import json
from pathlib import Path

MODEL_CENTER_ROI = (-100., -30., -3., 100., 30., 2.)

def validate_manifest(deck, entries=None):
    """Read-only preflight, including completed resumes that never load MR.

    The candidate's checksum alone cannot detect drift of the frozen v152
    geometry that its plans reference. Revalidate those external pins too.
    """
    pins = {}
    source_indices = {}
    from base5_capture_safety import sha256
    if deck.get('config', {}).get('secondary_far_vehicle_policy') and 'scenes' not in deck:
        raise RuntimeError('Far-secondary manifest lacks scene entries')
    for entry in deck.get('scenes', []) if entries is None else entries:
        if not deck.get('config', {}).get('secondary_far_vehicle_policy'):
            # Do not turn this opt-in check into a new legacy fixture/plan
            # schema requirement. Existing source validators own missing files.
            if not entry.get('metadata') or not Path(entry['metadata']).is_file():
                continue
        metadata = json.loads(Path(entry['metadata']).read_text())
        if 'secondary_far_vehicle' not in metadata.get('spec', {}):
            continue
        from base5_secondary_far_vehicle import validate_plan, verify_preserved, verify_payload
        plan = validate_plan(metadata['spec'], deck['config'])
        verify_preserved(metadata, metadata)
        policy = deck['config']['secondary_far_vehicle_policy']
        for value in policy.get('proposal_visibility', {}).values():
            if isinstance(value, dict) and 'path' in value and 'sha256' in value:
                pins[str(Path(value['path']).resolve())] = value['sha256']
        source_manifest = str(Path(policy['source_manifest']).resolve())
        if source_manifest not in source_indices:
            source_deck = json.loads(Path(source_manifest).read_text())
            source_indices[source_manifest] = {e['id']: e for e in source_deck['scenes']}
            if len(source_indices[source_manifest]) != len(source_deck['scenes']):
                raise RuntimeError('Far-secondary source has duplicate logical slots')
        original = source_indices[source_manifest].get(plan['logical_slot_id'])
        if (original is None or original['scenario_sha256'] != plan['source']['scenario']['sha256']
                or any(Path(original[key]).resolve() != Path(plan['source'][key]['path']).resolve()
                       for key in ('scenario', 'metadata'))):
            raise RuntimeError('Far-secondary source files do not belong to the frozen logical slot')
        if sha256(entry['scenario']) != entry['scenario_sha256']:
            raise RuntimeError('Far-secondary candidate scenario changed')
        scene = json.loads(Path(entry['scenario']).read_text())
        frozen_scene = json.loads(Path(plan['source']['scenario']['path']).read_text())
        verify_payload(scene, frozen_scene, plan, metadata)
        pins[str(Path(policy['source_manifest']).resolve())] = policy['source_manifest_sha256']
        for value in plan['source'].values():
            pins[str(Path(value['path']).resolve())] = value['sha256']
    return dict(sorted(pins.items()))


def verify_capture(entry, capture):
    """Validate the stored geometry too, not just the unchanged planned files."""
    capture = Path(capture)
    actual = json.loads((capture / 'metadata.json').read_text())
    expected = json.loads(Path(entry['metadata']).read_text()) if entry and entry.get('metadata') else {}
    if ('secondary_far_vehicle' not in actual.get('spec', {})
            and 'secondary_far_vehicle' not in expected.get('spec', {})):
        return
    from base5_secondary_far_vehicle import FarContractError, verify_preserved, verify_payload
    if not entry or not entry.get('scenario_sha256'):
        raise FarContractError('Captured far assignment lacks a pinned source entry')
    plan = verify_preserved(expected, actual)
    if plan is None:
        raise RuntimeError('Captured far assignment has no frozen source entry')
    from base5_capture_safety import sha256
    if sha256(capture / 'scenario.json') != entry['scenario_sha256']:
        raise FarContractError('Captured far scenario differs from its pinned source entry')
    frozen_scene = json.loads(Path(plan['source']['scenario']['path']).read_text())
    verify_payload(json.loads((capture / 'scenario.json').read_text()), frozen_scene, plan, actual)


def camera512_supported(component):
    return any(c.get('crop_512x1056_instance_pixels', 0) >= 16
               and min(c.get('model512_extent_px', [0, 0])) >= 4
               for c in component.get('cameras', []))


def population_components(metadata, components):
    """Use current-input evidence for the allocated car's parent visibility.

    Raw component evidence and legacy primary/other-parent gates are unchanged.
    A valid camera-only far target must not be rejected a second time solely
    because the old 320 crop is smaller than the actual 512 model input.
    """
    if 'secondary_far_vehicle' not in metadata.get('spec', {}):
        return components
    uid = metadata['spec']['secondary_far_vehicle']['target_uid']
    return [dict(c, positive_lidar1_camera0=c['lidar_points'] >= 1 or camera512_supported(c))
            if c['parent_uid'] == uid else c for c in components]


def review(metadata, components):
    if 'secondary_far_vehicle' not in metadata.get('spec', {}):
        return [], None
    plan = metadata['spec']['secondary_far_vehicle']
    from base5_secondary_far_vehicle import verify_preserved
    verify_preserved(metadata, metadata)
    actor = metadata['runtime_actors'][plan['target_ordinal']]
    uid = actor['unique_id']
    targets = [c for c in components if c['parent_uid'] == uid
               and c.get('training_class') == 'vehicle']
    reasons = []
    if len(targets) != 1:
        reasons.append('far_vehicle_missing_or_ambiguous_native_vehicle_box')
    lower, upper = plan['distance_bounds_m']
    sample_lower, sample_upper = plan.get('sampling_bounds_m', [lower, upper])
    evidence = []
    for component in targets:
        camera = camera512_supported(component)
        supported = component['lidar_points'] >= 1 or camera
        lidar_center = component.get('center_lidar_m')
        if (not isinstance(lidar_center, (list, tuple)) or len(lidar_center) != 3
                or not all(math.isfinite(float(x))
                           and MODEL_CENTER_ROI[i] <= float(x) < MODEL_CENTER_ROI[i + 3]
                           for i, x in enumerate(lidar_center))):
            reasons.append('far_vehicle_outside_model_center_roi')
        center = component.get('center_ego_m')
        distance = component.get('range_ego_xy_m')
        geometry_valid = (isinstance(center, (list, tuple)) and len(center) == 3 and distance is not None
                          and all(math.isfinite(float(x)) for x in center)
                          and math.isfinite(float(distance)))
        if not geometry_valid:
            reasons.append('far_vehicle_missing_calibrated_native_geometry')
        elif (not lower - .5 <= distance < upper + .5
              or not sample_lower - .5 <= distance < sample_upper + .5
              or abs(center[1]) > 30.5 or abs(center[0]) > 100.5
              or (center[0] < 0) != (plan['role'] == 'rear')):
            reasons.append('far_vehicle_actual_distance_direction_or_roi_mismatch')
        if not supported:
            reasons.append('far_vehicle_no_current_input_sensor_evidence')
        evidence.append(dict(component_id=component['component_id'], parent_uid=uid,
            range_ego_xy_m=distance, center_ego_m=center,
            lidar_points=component['lidar_points'], camera512_gate=camera,
            center_lidar_m=lidar_center, sensor_supported=supported))
    return reasons, dict(policy='far-secondary-native-lidar1-or-camera512-v1',
        target_uid=uid, distance_bounds_m=[lower, upper], role=plan['role'],
        sampling_bounds_m=[sample_lower, sample_upper],
        components=evidence, review_reasons=reasons,
        limitation='Actual sensor support and calibrated geometry, not learned detection or ground truth recall')
