from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


def make_path(trajectory, stamp):
    output = Path()
    output.header.frame_id = trajectory.header.frame_id
    output.header.stamp = stamp
    for point in trajectory.points:
        pose = PoseStamped()
        pose.header = output.header
        pose.pose = point.pose
        output.poses.append(pose)
    return output


def make_ego_pose(debug_message, stamp):
    output = PoseStamped()
    output.header.frame_id = debug_message.gt_trajectory.header.frame_id
    output.header.stamp = stamp
    if debug_message.ego_trajectory.points:
        output.pose = debug_message.ego_trajectory.points[-1].pose
    else:
        output.pose.orientation.w = 1.0
    return output


def make_markers(debug_message, stamp):
    output = MarkerArray()
    frame_id = debug_message.gt_trajectory.header.frame_id

    gt_marker = Marker()
    gt_marker.header.frame_id = frame_id
    gt_marker.header.stamp = stamp
    gt_marker.ns = "matched_gt"
    gt_marker.id = 0
    gt_marker.type = Marker.SPHERE
    gt_marker.action = Marker.ADD
    gt_marker.pose.orientation.w = 1.0
    gt_marker.scale.x = gt_marker.scale.y = gt_marker.scale.z = 0.8
    gt_marker.color.g = 1.0
    gt_marker.color.a = 1.0

    error_line = Marker()
    error_line.header = gt_marker.header
    error_line.ns = "tracking_error"
    error_line.id = 1
    error_line.type = Marker.LINE_STRIP
    error_line.action = Marker.ADD
    error_line.pose.orientation.w = 1.0
    error_line.scale.x = 0.12
    error_line.color.r = 1.0
    error_line.color.a = 1.0

    if debug_message.gt_trajectory.points and debug_message.ego_trajectory.points:
        index = min(
            debug_message.matched_gt_index,
            len(debug_message.gt_trajectory.points) - 1,
        )
        gt_position = debug_message.gt_trajectory.points[index].pose.position
        ego_position = debug_message.ego_trajectory.points[-1].pose.position
        gt_marker.pose.position = gt_position
        error_line.points = [
            Point(x=gt_position.x, y=gt_position.y, z=gt_position.z),
            Point(x=ego_position.x, y=ego_position.y, z=ego_position.z),
        ]
    output.markers.extend([gt_marker, error_line])
    return output
