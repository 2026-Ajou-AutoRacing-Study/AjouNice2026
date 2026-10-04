import json
import unittest
from planner_failure_review import parse_failure_event


class PlannerFailureReviewTest(unittest.TestCase):
    route = '1234567890abcdef1234567890abcdef'

    def event(self, **changes):
        value = dict(stamp=100.0, module='lane_change_left', reason='no_safe_maneuver',
                     route_uuid=self.route)
        value.update(changes)
        return json.dumps(value)

    def test_fresh_matching_route(self):
        self.assertEqual(parse_failure_event(self.event(), 100.5, self.route)['reason'], 'no_safe_maneuver')

    def test_wrong_route(self):
        self.assertIsNone(parse_failure_event(self.event(route_uuid='0'*32), 100.5, self.route))

    def test_stale_or_future(self):
        for now in (102.1, 99.8):
            self.assertIsNone(parse_failure_event(self.event(), now, self.route))

    def test_nonfinite(self):
        for value in (float('nan'), float('inf')):
            self.assertIsNone(parse_failure_event(self.event(stamp=value), 100.5, self.route))
            self.assertIsNone(parse_failure_event(self.event(), value, self.route))

    def test_missing_route(self):
        self.assertIsNone(parse_failure_event(self.event(), 100.5, ''))

    def test_bad_payload(self):
        for data in ('bad', '[]', '{}', self.event(reason=''), self.event(module=4)):
            self.assertIsNone(parse_failure_event(data, 100.5, self.route))


if __name__ == '__main__':
    unittest.main()
