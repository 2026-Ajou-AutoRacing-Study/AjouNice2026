"""Static LAN regressions; no DNS, ROS, MORAI or network sockets are used."""
import copy
import unittest
from unittest.mock import patch

from planning_scenario_validator.network_contract import inspect_network_contract


def network(host='10.42.0.252', destination='10.42.0.1'):
    return {'egoNetworkData': [{'listEgoNetworkInfo': [{
        'PUBSUB_TYPE': 783, 'netType': 1, 'config': {
            'commType': 1, 'udpConfig': {'hostIP': host, 'hostPort': 9095,
                                      'destinationIP': destination, 'destinationPort': 9096}}}]}]}


def sensors(destination='10.42.0.1'):
    return {'GPSList': [{'gc': {'commType': 1, 'udpConfig': {
        'hostIP': '10.42.0.252', 'destinationIP': destination, 'destinationPort': 5000}}}]}


class NetworkContractTests(unittest.TestCase):
    def inspect(self, host='127.0.0.1', wire_host='10.42.0.252', **morai):
        document = {'morai': {'scenario_load_host': host, **morai}}
        return inspect_network_contract(document, network(wire_host), sensors())

    def test_uploaded_wire_lan_but_yaml_loopback_is_rejected(self):
        result = self.inspect()
        self.assertEqual(len(result['errors']), 1)
        self.assertIn('morai.scenario_load_host=127.0.0.1', result['errors'][0])
        self.assertIn('10.42.0.252', result['errors'][0])
        self.assertIn('새 실험', result['errors'][0])
        self.assertEqual(result['ros_destination_ips'], ['10.42.0.1'])

    def test_omitted_scenario_target_uses_same_actual_loopback_default(self):
        result = inspect_network_contract({}, network(), sensors())
        self.assertTrue(result['errors'])
        self.assertEqual(result['scenario_host'], '127.0.0.1')

    def test_matching_explicit_lan_and_shared_path_is_valid(self):
        result = self.inspect(host='10.42.0.252', scenario_install_dir='/mnt/morai/Scenario')
        self.assertEqual(result['errors'], [])
        self.assertEqual(result['warnings'], [])

    def test_local_and_localhost_contracts_are_unchanged(self):
        for host in ('localhost', '127.0.0.1'):
            result = inspect_network_contract({'morai': {'scenario_load_host': host}},
                network('127.0.0.1', '127.0.0.1'), sensors('127.0.0.1'))
            self.assertEqual(result['errors'], [])
            self.assertEqual(result['warnings'], [])

    def test_wildcard_is_not_a_false_mismatch(self):
        for host, wire in [('10.42.0.252', '0.0.0.0'), ('0.0.0.0', '10.42.0.252')]:
            result = self.inspect(host=host, wire_host=wire)
            self.assertEqual(result['errors'], [])
            self.assertTrue(any('wildcard' in w for w in result['warnings']))

    def test_hostname_does_not_trigger_dns_or_unproven_rejection(self):
        with patch('socket.getaddrinfo', side_effect=AssertionError('No DNS allowed')):
            result = self.inspect(host='morai-host')
        self.assertEqual(result['errors'], [])
        self.assertTrue(any('hostname' in w for w in result['warnings']))

    def test_remote_auto_directory_is_a_shared_mount_warning_not_path_probe(self):
        result = self.inspect(host='10.42.0.252')
        self.assertEqual(result['errors'], [])
        self.assertTrue(any('scenario_install_dir=auto' in w for w in result['warnings']))

    def test_sensor_ego_destination_difference_is_only_warning(self):
        result = inspect_network_contract({'morai': {'scenario_load_host': '10.42.0.252',
            'scenario_install_dir': '/mnt/morai/Scenario'}}, network(), sensors('10.42.0.8'))
        self.assertEqual(result['errors'], [])
        self.assertTrue(any('destinationIP가 다릅니다' in w for w in result['warnings']))
        self.assertEqual(result['ros_destination_ips'], ['10.42.0.1', '10.42.0.8'])

    def test_inactive_udp_fields_are_ignored(self):
        wire = network('127.0.0.1', '127.0.0.1')
        entries = wire['egoNetworkData'][0]['listEgoNetworkInfo']
        inactive = copy.deepcopy(network()['egoNetworkData'][0]['listEgoNetworkInfo'][0])
        inactive['netType'] = 2
        entries.append(inactive)
        inactive = copy.deepcopy(inactive)
        inactive['netType'] = 1
        inactive['config']['commType'] = 0
        entries.append(inactive)
        result = inspect_network_contract({}, wire, sensors('127.0.0.1'))
        self.assertEqual(result['scenario_wire_host'], '127.0.0.1')
        self.assertEqual(result['errors'], [])
        self.assertEqual(result['warnings'], [])

    def test_no_argument_dictionaries_are_mutated(self):
        args = ({'morai': {'scenario_load_host': '10.42.0.252'}}, network(), sensors())
        original = copy.deepcopy(args)
        inspect_network_contract(*args)
        self.assertEqual(args, original)

    def test_second_ego_and_inactive_sensor_do_not_change_actual_first_ego_contract(self):
        wire = network('127.0.0.1', '127.0.0.1')
        wire['egoNetworkData'] += network()['egoNetworkData']
        sensor = sensors('127.0.0.1')
        sensor['cameraList'] = [{'cc': {'commType': 2, 'udpConfig': {
            'destinationIP': '10.42.0.8'}}}]
        result = inspect_network_contract({}, wire, sensor)
        self.assertEqual(result['errors'], [])
        self.assertEqual(result['warnings'], [])
        self.assertEqual(result['scenario_wire_host'], '127.0.0.1')
        self.assertEqual(result['ros_destination_ips'], ['127.0.0.1'])

    def test_control_config_udp_destination_is_included(self):
        wire = network()
        wire['egoNetworkData'][0]['listEgoNetworkInfo'].append({
            'PUBSUB_TYPE': 771, 'netType': 1, 'config': {'commType': 0},
            'EgoCtrlConfig': {'commType': 1, 'udpConfig': {
                'hostIP': '10.42.0.252', 'destinationIP': '10.42.0.7'}}})
        result = inspect_network_contract({'morai': {
            'scenario_load_host': '10.42.0.252', 'scenario_install_dir': '/mnt/scenarios'}},
            wire, sensors())
        self.assertEqual(result['errors'], [])
        self.assertEqual(result['ros_destination_ips'], ['10.42.0.1', '10.42.0.7'])
        self.assertTrue(any('destinationIP가 다릅니다' in warning for warning in result['warnings']))


if __name__ == '__main__':
    unittest.main()
