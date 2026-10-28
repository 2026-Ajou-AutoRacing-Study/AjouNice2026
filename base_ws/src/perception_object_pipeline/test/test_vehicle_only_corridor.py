"""Map-verified C-to-shadow vehicle-only corridor, the v101 production map."""

import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import yaml

from perception_object_pipeline.map_based_prediction import LaneletNetwork, MapPredictionConfig
from perception_object_pipeline.vehicle_only_corridor import VehicleOnlyCorridor
from autoware_perception_msgs.msg import ObjectClassification, PredictedObject, PredictedObjects

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from map_based_prediction_node import MapBasedPredictionNode


ROOT = Path(__file__).resolve().parents[4]


class VehicleOnlyCorridorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.corridors = {}
        for version in (101,):
            network = LaneletNetwork.load(
                str(ROOT / f'kcity_map/lanelet_map/lanelet2_map_v{version}.osm'),
                str(ROOT / 'kcity_map/lanelet_map/map_projector_info_v1.yaml'),
                MapPredictionConfig())
            route = yaml.safe_load((ROOT / (
                'base_ws/src/planning_addons/competition_route_requester/config/'
                f'kcity_v{version}_competition_route.yaml')).read_text())
            cls.corridors[version] = VehicleOnlyCorridor(network, route)

    def test_scope_for_v101_map(self):
        for version, gate in self.corridors.items():
            with self.subTest(version=version):
                self.assertLess(math.dist(gate.start_xy, (71.0, 587.0)), 5.0)
                self.assertLess(math.dist(gate.end_xy, (202.1, 160.5)), 5.0)
                self.assertFalse(gate.evaluate(71.2, 557.4, math.pi / 2)[0])
                self.assertTrue(gate.evaluate(71.0, 600.0, math.pi / 2)[0])
                self.assertTrue(gate.evaluate(201.8, 171.0, -math.pi / 2)[0])
                self.assertFalse(gate.evaluate(202.2, 151.0, -math.pi / 2)[0])
                self.assertFalse(gate.evaluate(71.0, 600.0, -math.pi / 2)[0])
                self.assertFalse(gate.evaluate(100.0, 600.0, math.pi / 2)[0])

    def test_gps_shadow_objects_outside_100m_at_exit(self):
        for gate in self.corridors.values():
            self.assertGreater(math.dist(gate.end_xy, (152.2, 53.5)), 100.0)

    def test_map_mismatch_is_rejected(self):
        network = LaneletNetwork.load(
            str(ROOT / 'kcity_map/lanelet_map/lanelet2_map_v101.osm'),
            str(ROOT / 'kcity_map/lanelet_map/map_projector_info_v1.yaml'),
            MapPredictionConfig())
        route = yaml.safe_load((ROOT / (
            'base_ws/src/planning_addons/competition_route_requester/config/'
            'kcity_v101_competition_route.yaml')).read_text())
        route['map']['sha256'] = '0' * 64
        with self.assertRaisesRegex(ValueError, 'SHA'):
            VehicleOnlyCorridor(network, route)

    def test_planner_output_only_filters_nonvehicles_and_fails_open_on_stale_ego(self):
        class Publisher:
            def __init__(self):
                self.messages = []
            def publish(self, message):
                self.messages.append(message.data)

        def predicted(label):
            item = PredictedObject()
            item.classification = [ObjectClassification(label=label, probability=1.0)]
            return item

        node = MapBasedPredictionNode.__new__(MapBasedPredictionNode)
        node.vehicle_only_corridor = self.corridors[101]
        node.vehicle_only_state_pub = Publisher()
        node._vehicle_only_active = False
        node._vehicle_only_ego = (201.8, 171.0, -math.pi / 2, 9.9)
        objects = [predicted(ObjectClassification.CAR), predicted(ObjectClassification.TRUCK),
                   predicted(ObjectClassification.PEDESTRIAN), predicted(ObjectClassification.UNKNOWN)]
        output = PredictedObjects()
        output.objects = list(objects)
        with patch('map_based_prediction_node.time.monotonic', return_value=10.0), \
             patch('map_based_prediction_node.rospy.logwarn'), \
             patch('map_based_prediction_node.rospy.logwarn_throttle'):
            node._vehicle_only_policy(output)
        self.assertEqual(len(output.objects), 2)
        self.assertEqual(node.vehicle_only_state_pub.messages, [True])
        node._vehicle_only_ego = (201.8, 171.0, -math.pi / 2, 8.0)
        output.objects = list(objects)
        with patch('map_based_prediction_node.time.monotonic', return_value=10.0), \
             patch('map_based_prediction_node.rospy.logwarn'):
            node._vehicle_only_policy(output)
        self.assertEqual(len(output.objects), 4)
        self.assertEqual(node.vehicle_only_state_pub.messages, [True, False])


if __name__ == '__main__':
    unittest.main()
