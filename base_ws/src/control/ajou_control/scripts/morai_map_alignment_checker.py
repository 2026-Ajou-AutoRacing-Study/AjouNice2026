#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Point
from morai_msgs.msg import EgoVehicleStatus, GPSMessage, MapSpecIndex
from morai_msgs.srv import MoraiMapSpecSrv
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


def norm2d(dx, dy):
    return math.hypot(dx, dy)


class MoraiMapAlignmentChecker:
    def __init__(self):
        self.map_spec_service = rospy.get_param("~map_spec_service", "/Service_MoraiMapSpec")
        self.ego_topic = rospy.get_param("~ego_topic", "/Ego_topic")
        self.gps_topic = rospy.get_param("~gps_topic", "/sensor/gps")
        self.marker_topic = rospy.get_param("~marker_topic", "/morai_map_alignment/markers")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 5.0))
        self.axis_length = float(rospy.get_param("~axis_length", 30.0))
        self.use_gps = bool(rospy.get_param("~use_gps", True))

        self.map_spec = None
        self.ego = None
        self.gps = None

        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
        self.ego_gps_delta_pub = rospy.Publisher("/morai_map_alignment/ego_gps_delta", Float32, queue_size=1)

        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.ego_callback, queue_size=1)
        if self.use_gps:
            rospy.Subscriber(self.gps_topic, GPSMessage, self.gps_callback, queue_size=1)

        self.query_map_spec()

    def query_map_spec(self):
        try:
            rospy.loginfo("Waiting for MORAI map spec service: %s", self.map_spec_service)
            rospy.wait_for_service(self.map_spec_service, timeout=5.0)
            proxy = rospy.ServiceProxy(self.map_spec_service, MoraiMapSpecSrv)
            request = MapSpecIndex(load_map_data=True)
            self.map_spec = proxy(request).response
            rospy.loginfo(
                "MORAI map spec: plane=%d utm_zone=%d offset=(%.3f, %.3f, %.3f) ellipse=%s lat=%.9f lon=%.9f",
                self.map_spec.plane_coordinate_system,
                self.map_spec.utm_num,
                self.map_spec.utm_offset.x,
                self.map_spec.utm_offset.y,
                self.map_spec.utm_offset.z,
                self.map_spec.ellipse,
                self.map_spec.central_latitude,
                self.map_spec.central_meridian,
            )
        except Exception as exc:
            rospy.logwarn("Could not query MORAI map spec yet: %s", exc)

    def ego_callback(self, msg):
        self.ego = msg

    def gps_callback(self, msg):
        self.gps = msg

    def marker_base(self, marker_id, namespace, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def arrow_marker(self, marker_id, namespace, start, end, color):
        marker = self.marker_base(marker_id, namespace, Marker.ARROW)
        marker.scale.x = 0.35
        marker.scale.y = 0.75
        marker.scale.z = 0.75
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [Point(*start), Point(*end)]
        return marker

    def sphere_marker(self, marker_id, namespace, point, scale, color):
        marker = self.marker_base(marker_id, namespace, Marker.SPHERE)
        marker.pose.position = Point(*point)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def line_marker(self, marker_id, namespace, points, width, color):
        marker = self.marker_base(marker_id, namespace, Marker.LINE_STRIP)
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [Point(*point) for point in points]
        return marker

    def text_marker(self, marker_id, point, delta):
        marker = self.marker_base(marker_id, "morai_map_alignment_text", Marker.TEXT_VIEW_FACING)
        marker.pose.position = Point(point[0], point[1], point[2] + 3.0)
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        if self.map_spec is None:
            marker.text = "MORAI map spec unavailable: %s" % self.map_spec_service
            return marker

        offset = self.map_spec.utm_offset
        marker.text = "MORAI map | UTM%d offset %.1f %.1f %.1f | ego-gps %.3fm" % (
            self.map_spec.utm_num,
            offset.x,
            offset.y,
            offset.z,
            delta if delta is not None else -1.0,
        )
        return marker

    def ego_point(self):
        if self.ego is None:
            return None
        return (self.ego.position.x, self.ego.position.y, self.ego.position.z + 1.0)

    def gps_point(self):
        if self.gps is None or self.gps.status == 0:
            return None
        return (self.gps.eastOffset, self.gps.northOffset, self.gps.altitude)

    def ego_heading_marker(self, marker_id, ego_point):
        yaw = math.radians(float(self.ego.heading))
        end = (
            ego_point[0] + 8.0 * math.cos(yaw),
            ego_point[1] + 8.0 * math.sin(yaw),
            ego_point[2],
        )
        return self.arrow_marker(marker_id, "morai_map_alignment_ego_heading", ego_point, end, (1.0, 1.0, 1.0, 1.0))

    def build_markers(self):
        markers = MarkerArray()
        marker_id = 0

        origin = (0.0, 0.0, 0.2)
        markers.markers.append(
            self.arrow_marker(marker_id, "morai_map_alignment_map_x_axis", origin, (self.axis_length, 0.0, 0.2), (1.0, 0.0, 0.0, 1.0))
        )
        marker_id += 1
        markers.markers.append(
            self.arrow_marker(marker_id, "morai_map_alignment_map_y_axis", origin, (0.0, self.axis_length, 0.2), (0.0, 1.0, 0.0, 1.0))
        )
        marker_id += 1
        markers.markers.append(self.sphere_marker(marker_id, "morai_map_alignment_map_origin", origin, 1.2, (1.0, 1.0, 0.0, 1.0)))
        marker_id += 1

        ego_point = self.ego_point()
        gps_point = self.gps_point() if self.use_gps else None
        delta = None

        if ego_point:
            markers.markers.append(self.sphere_marker(marker_id, "morai_map_alignment_ego_topic", ego_point, 1.5, (1.0, 1.0, 1.0, 1.0)))
            marker_id += 1
            markers.markers.append(self.ego_heading_marker(marker_id, ego_point))
            marker_id += 1

        if gps_point:
            markers.markers.append(self.sphere_marker(marker_id, "morai_map_alignment_gps_offset", gps_point, 1.2, (0.0, 0.55, 1.0, 1.0)))
            marker_id += 1

        if ego_point and gps_point:
            delta = norm2d(ego_point[0] - gps_point[0], ego_point[1] - gps_point[1])
            self.ego_gps_delta_pub.publish(Float32(data=delta))
            markers.markers.append(
                self.line_marker(marker_id, "morai_map_alignment_ego_gps_delta", [ego_point, gps_point], 0.25, (1.0, 0.0, 1.0, 1.0))
            )
            marker_id += 1

        text_anchor = ego_point or gps_point or (0.0, 0.0, 0.0)
        markers.markers.append(self.text_marker(marker_id, text_anchor, delta))
        return markers, delta

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if self.map_spec is None:
                self.query_map_spec()

            markers, delta = self.build_markers()
            self.marker_pub.publish(markers)

            rospy.loginfo_throttle(
                1.0,
                "MORAI map alignment: map_spec=%s ego=%s gps=%s ego_gps_delta=%s",
                self.map_spec is not None,
                self.ego is not None,
                self.gps is not None,
                "%.3fm" % delta if delta is not None else "n/a",
            )
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("morai_map_alignment_checker")
    try:
        MoraiMapAlignmentChecker().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("morai_map_alignment_checker failed: %s", exc)
        raise
