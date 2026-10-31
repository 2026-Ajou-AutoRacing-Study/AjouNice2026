#!/usr/bin/env python3
"""Exercise the node's optimized-prefix/input-path join through its ROS interface."""
import math
import threading
import time
import unittest

import rospy
import rostest
from autoware_planning_msgs.msg import Path, PathPoint, Trajectory, StaticObstacleScene, StaticObstacle
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Point32


class OptimizerExtension(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.output = None
        cls.ready = threading.Event()
        cls.pub = rospy.Publisher('/optimizer/input/path', Path, queue_size=1)
        cls.odom = rospy.Publisher('/optimizer/input/odometry', Odometry, queue_size=1, latch=True)
        cls.scene = rospy.Publisher('/planning/scenario_planning/lane_driving/behavior_planning/static_obstacle_scene', StaticObstacleScene, queue_size=1)
        cls.sub = rospy.Subscriber('/optimizer/output/path', Trajectory, cls.receive)
        deadline = time.monotonic() + 10
        while not (cls.pub.get_num_connections() and cls.odom.get_num_connections() and cls.scene.get_num_connections()):
            if time.monotonic() > deadline:
                raise RuntimeError('smoother did not connect')
            time.sleep(.02)

    @classmethod
    def receive(cls, msg):
        cls.output = msg
        cls.ready.set()

    def run_path(self, curvature, stop_s=None, length=180, obstacles=()):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        # Integrate at fine spacing, publish at the upstream path's 2 m spacing.
        x = y = yaw = 0.0
        for i in range(int(length * 10) + 1):
            s = i / 10.0
            if i % 20 == 0:
                p = PathPoint()
                p.pose.position.x, p.pose.position.y = x, y
                p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                p.longitudinal_velocity_mps = 0 if stop_s is not None and s >= stop_s else 16.0
                path.points.append(p)
                path.left_bound.append(Point(x - 4 * math.sin(yaw), y + 4 * math.cos(yaw), 0))
                path.right_bound.append(Point(x + 4 * math.sin(yaw), y - 4 * math.cos(yaw), 0))
            k = curvature(s + .05)
            x += .1 * math.cos(yaw + k * .05)
            y += .1 * math.sin(yaw + k * .05)
            yaw += .1 * k
        odom = Odometry()
        odom.pose.pose = path.points[min(10, len(path.points) - 1)].pose
        odom.twist.twist.linear.x = 16.0
        self.ready.clear()
        self.odom.publish(odom)
        time.sleep(.05)
        scene = StaticObstacleScene()
        scene.header = path.header
        scene.obstacles = list(obstacles)
        self.scene.publish(scene)
        time.sleep(.02)
        self.pub.publish(path)
        self.assertTrue(self.ready.wait(5), 'no smoothed trajectory')
        return path, self.output.points

    @staticmethod
    def kappa(points, i):
        a, b, c = [points[j].pose.position for j in (i - 5, i, i + 5)]
        u, v, w = (b.x-a.x, b.y-a.y), (c.x-a.x, c.y-a.y), (c.x-b.x, c.y-b.y)
        den = math.hypot(*u) * math.hypot(*v) * math.hypot(*w)
        return 2 * (u[0]*v[1]-u[1]*v[0]) / den if den > 1e-10 else 0

    def test_constant_curve_retains_geometry(self):
        _, points = self.run_path(lambda s: .02)
        # Exclude the fixed rear edge and the goal endpoint. The artificial
        # 15 m chord used to produce a large peak inside this interval.
        ks = [abs(self.kappa(points, i)) for i in range(60, len(points) - 20)]
        self.assertLess(max(ks), .027)
        self.assertGreater(min(ks), .013)

    def test_s_curve_retains_geometry(self):
        _, points = self.run_path(lambda s: .025 * math.sin((s - 70) / 20))
        ks = [abs(self.kappa(points, i)) for i in range(60, len(points) - 20)]
        self.assertLess(max(ks), .033)

    def test_real_tight_curve_is_preserved(self):
        _, points = self.run_path(lambda s: .05, length=110)
        ks = [abs(self.kappa(points, i)) for i in range(60, len(points) - 20)]
        self.assertLess(max(ks), .067)
        self.assertGreater(min(ks), .035)

    def test_stop_near_join_is_not_removed_or_moved_back(self):
        path, points = self.run_path(lambda s: .0, stop_s=36)
        stop = next(p.pose.position for p in path.points if p.longitudinal_velocity_mps == 0)
        zero = next(i for i, p in enumerate(points) if p.longitudinal_velocity_mps == 0)
        self.assertLess(math.hypot(points[zero].pose.position.x-stop.x,
                                   points[zero].pose.position.y-stop.y), .21)
        self.assertTrue(all(p.longitudinal_velocity_mps == 0 for p in points[zero:]))

    def test_short_path_preserves_goal(self):
        path, points = self.run_path(lambda s: .02, length=40)
        a, b = path.points[-1].pose.position, points[-1].pose.position
        self.assertLess(math.hypot(a.x-b.x, a.y-b.y), .001)

    @staticmethod
    def obstacle(half_width):
        obstacle = StaticObstacle()
        obstacle.hard_margin = .2
        obstacle.soft_margin = .3
        obstacle.polygon.points = [Point32(x, y, 0) for x, y in
                                   [(42, -half_width), (44, -half_width),
                                    (44, half_width), (42, half_width)]]
        return obstacle

    def test_obstacle_avoidance_returns_before_join(self):
        _, points = self.run_path(lambda s: 0, obstacles=[self.obstacle(.5)])
        crossing = [p for p in points if 41 < p.pose.position.x < 45]
        self.assertTrue(crossing)
        self.assertTrue(all(p.longitudinal_velocity_mps > 0 for p in crossing))
        self.assertTrue(all(abs(p.pose.position.y) > 1.64 for p in crossing))
        returned = [p for p in points if 70 < p.pose.position.x < 80]
        self.assertTrue(returned)
        self.assertLess(max(abs(p.pose.position.y) for p in returned), .03)

    def test_wall_cannot_be_hidden_by_the_join(self):
        _, points = self.run_path(lambda s: 0, obstacles=[self.obstacle(4)])
        self.assertTrue(any(p.longitudinal_velocity_mps == 0 for p in points
                            if 20 <= p.pose.position.x < 42))


if __name__ == '__main__':
    rospy.init_node('test_path_extension')
    rostest.rosrun('autoware_path_optimizer', 'path_extension', OptimizerExtension)
