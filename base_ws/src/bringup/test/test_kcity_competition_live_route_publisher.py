#!/usr/bin/env python3

import importlib.util
import os
import tempfile
import unittest


def load_route_publisher_module():
    script_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "validation",
            "scripts",
            "kcity_competition_live_route_publisher.py",
        )
    )
    spec = importlib.util.spec_from_file_location(
        "kcity_competition_live_route_publisher",
        script_path,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class EditedRouteDefinitionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_route_publisher_module()

    def test_loads_preferred_candidates_and_assignments(self):
        content = """
preferred_lane_ids: [101, 102]
candidate_lane_ids: [201, 202]
candidate_assignments:
  101: [201]
  102: [202]
"""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml") as stream:
            stream.write(content)
            stream.flush()
            route = self.module.load_route_definition(stream.name)

        self.assertEqual(route["preferred_lane_ids"], [101, 102])
        self.assertEqual(route["candidate_lane_ids"], [201, 202])
        self.assertEqual(
            self.module.parse_candidate_assignments(
                route["candidate_assignments"]
            ),
            {101: [201], 102: [202]},
        )

    def test_segments_keep_candidate_lanes_in_preferred_sections(self):
        publisher = self.module.KcityCompetitionLiveRoutePublisher.__new__(
            self.module.KcityCompetitionLiveRoutePublisher
        )
        publisher.preferred_lane_ids = [101, 102]
        publisher.candidate_lane_ids = [201, 202]
        publisher.edited_candidate_assignments = {
            101: [201],
            102: [202],
        }

        segments = publisher.make_segments()

        self.assertEqual(len(segments), 2)
        self.assertEqual(int(segments[0].preferred_primitive.id), 101)
        self.assertEqual([int(item.id) for item in segments[0].primitives], [101, 201])
        self.assertEqual(int(segments[1].preferred_primitive.id), 102)
        self.assertEqual([int(item.id) for item in segments[1].primitives], [102, 202])


if __name__ == "__main__":
    unittest.main()
