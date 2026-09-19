#!/usr/bin/env python3
"""Verify the installed launch interface without starting its absent upstream node."""
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

launch = Path(sys.argv[1]).resolve()
root = ET.parse(launch).getroot()
args = [e.attrib for e in root.findall('arg')]
assert args == [
    {'name': 'input_path'},
    {'name': 'input_scenario'},
    {'name': 'output_trajectory'},
]
node, = root.findall('node')
assert node.attrib == {
    'pkg': 'autoware_frenet_planner', 'type': 'autoware_frenet_planner',
    'name': 'autoware_frenet_planner', 'output': 'screen',
}
assert [e.attrib for e in node.findall('remap')] == [
    {'from': '~input/path', 'to': '$(arg input_path)'},
    {'from': '~input/scenario', 'to': '$(arg input_scenario)'},
    {'from': '~output/trajectory', 'to': '$(arg output_trajectory)'},
]
assert [e.attrib for e in node.findall('param')] == [{'name': 'update_rate', 'value': '10.0'}]
package_share = subprocess.run(
    ['rospack', 'find', 'autoware_frenet_planner'],
    text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True).stdout.strip()
assert launch == Path(package_share).resolve() / 'launch/frenet_planner.launch'
nodes = subprocess.run(
    ['roslaunch', '--nodes', str(launch), 'input_path:=/in/path',
     'input_scenario:=/in/scenario', 'output_trajectory:=/out/trajectory'],
    text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True).stdout.splitlines()
assert nodes == ['/autoware_frenet_planner']
print('LAUNCH_RESOLUTION_AND_CONTRACT_PRESERVED')
