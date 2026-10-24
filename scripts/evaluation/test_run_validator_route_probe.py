import unittest
from unittest.mock import patch

from run_validator_route_probe import contains_in_order, source_receipt


class RouteProbeTest(unittest.TestCase):
    def test_required_ring_sequence_not_just_any_nearby_exit(self):
        self.assertFalse(contains_in_order([201095, 201098, 201096], [201097, 201093]))
        self.assertTrue(contains_in_order([201095, 201097, 201093, 201147], [201097, 201093]))

    def test_wrong_order_missing_and_duplicate_requirements_fail(self):
        self.assertFalse(contains_in_order([3, 2, 1], [1, 2]))
        self.assertFalse(contains_in_order([1, 2], [1, 1]))
        self.assertFalse(contains_in_order([], [1]))
        self.assertTrue(contains_in_order([1, 1, 2], [1, 1, 2]))

    def test_receipt_records_matching_source_install_and_map_without_live_actions(self):
        with patch('run_validator_route_probe.Path.read_bytes', return_value=b'fixture'):
            result = source_receipt()
        self.assertEqual(len(result['map_sha256']), 64)
        self.assertTrue(all(row['source_sha256'] == row['installed_sha256']
                            for row in result['source_installed']))


if __name__ == '__main__':
    unittest.main()
