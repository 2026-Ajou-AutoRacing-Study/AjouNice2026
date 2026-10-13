"""Explicit, exactly-once ROS box-origin conversion; legacy stays default.

MMDetection3D LiDAR tensor XYZ denotes the bottom center. ROS bounding-box
poses/Marker.CUBE denote the geometric center. Never guess an untagged source
origin or change an existing model's raw export/GT to enable a trial.
"""

import math


LEGACY_OUTPUT = 'legacy_passthrough'
BOTTOM_ORIGIN = 'lidar_bottom_v1'
GRAVITY_ORIGIN = 'gravity_center_v1'


def validate_output_origin(value):
    if value not in (LEGACY_OUTPUT, GRAVITY_ORIGIN):
        raise ValueError('box_center_output_origin must be legacy_passthrough or gravity_center_v1')
    return value


def response_for_ros(response, output_origin=LEGACY_OUTPUT):
    """Preserve legacy object identity, or normalize a tagged copy for ROS.

    Raw response and nested input lists are never mutated. Corrected responses
    carry their new origin, so passing one through this function again cannot
    add half the height twice. An untagged old server remains compatible with
    legacy mode but must fail closed for a geometric-center trial.
    """
    validate_output_origin(output_origin)
    if output_origin == LEGACY_OUTPUT:
        return response
    source = response.get('box_center_origin')
    if source not in (BOTTOM_ORIGIN, GRAVITY_ORIGIN):
        raise ValueError('gravity-center trial requires an explicitly tagged server box origin')
    if source == GRAVITY_ORIGIN:
        return response
    corrected = []
    for detection in response.get('detections', []):
        center = detection.get('center', ())
        size = detection.get('size', ())
        if len(center) != 3 or len(size) != 3:
            raise ValueError('tagged bottom-center box requires three center and size values')
        numeric_center = [float(v) for v in center]
        if not all(math.isfinite(v) for v in numeric_center):
            raise ValueError('tagged bottom-center box must have a finite center')
        height = float(size[2])
        if not math.isfinite(height) or height <= 0:
            raise ValueError('tagged bottom-center box must have finite positive height')
        numeric_center[2] += height * .5
        if not math.isfinite(numeric_center[2]):
            raise ValueError('converted gravity-center height must be finite')
        converted = dict(detection)
        converted['center'] = numeric_center
        corrected.append(converted)
    result = dict(response)
    result['detections'] = corrected
    result['box_center_origin'] = GRAVITY_ORIGIN
    return result
