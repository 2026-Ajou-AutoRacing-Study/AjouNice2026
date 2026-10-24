import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/generate_network_profile.py'
spec = importlib.util.spec_from_file_location('validator_network_generator', SCRIPT)
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


class ValidatorLanProfileTests(unittest.TestCase):
    def test_lan_preserves_competition_entries_except_scenario_slot(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'lan.json'
            with patch.object(generator, 'LAN_OUTPUT', output):
                generator.build(lan=True, install=False)
            actual = json.loads(output.read_text())['egoNetworkData'][0]['listEgoNetworkInfo']
            original = json.loads((generator.ROOT / 'configs/network/ego_network_competition_lan.json').read_text())['egoNetworkData'][0]['listEgoNetworkInfo']
            self.assertEqual(len(actual), len(original))
            for before, after in zip(original, actual):
                if before['PUBSUB_TYPE'] != 783:
                    self.assertEqual(before, after)
            scenarios = [e for e in actual if e['PUBSUB_TYPE'] == 783]
            self.assertEqual(len(scenarios), 1)
            self.assertEqual(scenarios[0]['config']['udpConfig']['hostPort'], 9095)
            self.assertEqual(scenarios[0]['config']['udpConfig']['destinationPort'], 9096)

    def test_custom_ips_and_invalid_input(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'lan.json'
            with patch.object(generator, 'LAN_OUTPUT', output):
                generator.build(lan=True, morai_ip='192.168.10.1', ros_ip='192.168.10.2', install=False)
                entries = json.loads(output.read_text())['egoNetworkData'][0]['listEgoNetworkInfo']
                for e in entries:
                    udp = e.get('config', {}).get('udpConfig')
                    if udp:
                        self.assertEqual(udp['hostIP'], '192.168.10.1')
                        self.assertEqual(udp['destinationIP'], '192.168.10.2')
                with self.assertRaises(ValueError):
                    generator.build(lan=True, morai_ip='bad', ros_ip='192.168.10.2', install=False)
                with self.assertRaises(ValueError):
                    generator.build(lan=True, morai_ip='192.168.10.1', install=False)

    def test_local_output_is_unchanged(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'local.json'
            with patch.object(generator, 'OUTPUT', output):
                generator.build(install=False)
            original = SCRIPT.parents[1] / 'config/internal/generated/ego_network_validator_local.json'
            self.assertEqual(output.read_bytes(), original.read_bytes())
