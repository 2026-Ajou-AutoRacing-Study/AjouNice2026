import math
from copy import deepcopy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from autoware_internal_planning_msgs.msg import (
    ControlPoint,
    PlanningFactor,
    SafetyFactor,
)
from autoware_perception_msgs.msg import ObjectClassification
from tier4_planning_msgs.msg import StopFactor, StopReason

from .path_utils import project_pose_to_path


def route_lane_ids(path_with_lane_id):
    return {
        int(lane_id)
        for path_point in path_with_lane_id.points
        for lane_id in path_point.lane_ids
    }


def ego_station(path, odometry):
    if odometry is None:
        return 0.0
    projection = project_pose_to_path(path, odometry.pose.pose)
    return float(projection[0]) if projection is not None else 0.0


def object_position(detected):
    position = detected.kinematics.initial_pose_with_covariance.pose.position
    return float(position.x), float(position.y), float(position.z)


def object_speed(detected):
    velocity = detected.kinematics.initial_twist_with_covariance.twist.linear
    return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)


def object_label(detected):
    if not detected.classification:
        return ObjectClassification.UNKNOWN
    classification = max(
        detected.classification, key=lambda item: float(item.probability)
    )
    return int(classification.label)


def to_point(value):
    point = Point()
    point.x = float(value[0])
    point.y = float(value[1])
    point.z = float(value[2]) if len(value) > 2 else 0.0
    return point


def make_stop_reason(reason_name, stop_pose, distance, factor_points):
    factor = StopFactor()
    factor.stop_pose = deepcopy(stop_pose)
    factor.dist_to_stop_pose = float(distance)
    factor.stop_factor_points = [to_point(point) for point in factor_points]
    reason = StopReason()
    reason.reason = str(reason_name)
    reason.stop_factors = [factor]
    return reason


def make_planning_factor(
    module_name,
    header,
    control_pose,
    distance,
    behavior,
    detail,
    velocity=0.0,
    safe=False,
    detected_object=None,
    safety_points=(),
    safety_type=None,
    ttc_begin=0.0,
    ttc_end=0.0,
):
    control = ControlPoint()
    control.pose = deepcopy(control_pose)
    control.velocity = float(velocity)
    control.distance = float(distance)
    factor = PlanningFactor()
    factor.module = str(module_name)
    factor.is_driving_forward = True
    factor.control_points = [control]
    factor.behavior = int(behavior)
    factor.detail = str(detail)
    factor.safety_factors.header = deepcopy(header)
    factor.safety_factors.is_safe = bool(safe)
    factor.safety_factors.detail = str(detail)
    if detected_object is not None or safety_points:
        safety = SafetyFactor()
        safety.type = int(
            safety_type if safety_type is not None else SafetyFactor.OBJECT
        )
        if detected_object is not None:
            safety.object_id = deepcopy(detected_object.object_id)
            predictions = detected_object.kinematics.predicted_paths
            if predictions:
                safety.predicted_path = deepcopy(
                    max(predictions, key=lambda item: float(item.confidence))
                )
        safety.ttc_begin = float(ttc_begin)
        safety.ttc_end = float(ttc_end)
        safety.points = [to_point(point) for point in safety_points]
        safety.is_safe = bool(safe)
        safety.detail = str(detail)
        factor.safety_factors.factors = [safety]
    return factor


def marker(header, namespace, marker_id, marker_type, rgba, scale=(0.2, 0.2, 0.2)):
    output = Marker()
    output.header = deepcopy(header)
    output.ns = str(namespace)
    output.id = int(marker_id)
    output.type = int(marker_type)
    output.action = Marker.ADD
    output.pose.orientation.w = 1.0
    output.scale.x = float(scale[0])
    output.scale.y = float(scale[1])
    output.scale.z = float(scale[2])
    output.color.r = float(rgba[0])
    output.color.g = float(rgba[1])
    output.color.b = float(rgba[2])
    output.color.a = float(rgba[3])
    return output


def line_marker(header, namespace, marker_id, points, rgba, width=0.25, closed=False):
    output = marker(
        header, namespace, marker_id, Marker.LINE_STRIP, rgba, (width, 0.0, 0.0)
    )
    values = list(points)
    if closed and values:
        values.append(values[0])
    output.points = [to_point(point) for point in values]
    return output


def polygon_marker(header, namespace, marker_id, points, rgba, z_offset=0.05):
    output = marker(
        header, namespace, marker_id, Marker.TRIANGLE_LIST, rgba, (1.0, 1.0, 1.0)
    )
    values = list(points)
    if len(values) < 3:
        return output
    center = (
        sum(point[0] for point in values) / len(values),
        sum(point[1] for point in values) / len(values),
        sum(point[2] for point in values) / len(values) + z_offset,
    )
    for first, second in zip(values, values[1:] + values[:1]):
        output.points.extend([to_point(center), to_point(first), to_point(second)])
    return output


def sphere_marker(header, namespace, marker_id, point, rgba, diameter=1.0):
    output = marker(
        header,
        namespace,
        marker_id,
        Marker.SPHERE,
        rgba,
        (diameter, diameter, diameter),
    )
    output.pose.position = to_point(point)
    return output


def text_marker(header, namespace, marker_id, point, text, rgba, height=0.8):
    output = marker(
        header, namespace, marker_id, Marker.TEXT_VIEW_FACING, rgba, (0.0, 0.0, height)
    )
    output.pose.position = to_point(point)
    output.pose.position.z += 1.5
    output.text = str(text)
    return output
