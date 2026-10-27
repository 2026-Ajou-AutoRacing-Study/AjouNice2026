"""UNKNOWN-grace release wiring, without a ROS master or live node execution.

Pure XML/source checks run by default. To resolve the real source-tree launch
files and all includes (roslaunch --dump-params ONLY):

  AJOUNICE_TEST_LAUNCH_DUMP=1 /usr/bin/python3 -m pytest -q \
    scripts/runtime/test_traffic_light_unknown_grace_release.py

After an independently authorized installation, add
AJOUNICE_TEST_LAUNCH_TREE=installed to audit install_aw149 instead. These tests
never install, start roscore/nodes, call native APIs or change ROS parameters.
"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
TRAFFIC = ROOT/'base_ws/src/perception/traffic_light_detection'
PLANNING = ROOT/'base_ws/src/planning_addons/ajounice_planning_integration'
CPP = ROOT/'planning_ws/src/autoware_behavior_velocity_traffic_light_module'
PREFIX = '/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/traffic_light/'
GRACE = 'signal_compliance_unknown_grace_sec'
EVIDENCE = 'preserve_unreadable_color_evidence'
LAUNCHES = {
    'node': (TRAFFIC, 'traffic_light_node.launch'),
    'v5': (TRAFFIC, 'traffic_light_v5_candidate.launch'),
    'legacy': (TRAFFIC, 'traffic_light_legacy.launch'),
    'planning': (PLANNING, 'planning.launch'),
    'stack': (PLANNING, 'morai_planning_stack.launch'),
}


def xml(name):
    package, filename = LAUNCHES[name]
    return ET.parse(package/'launch'/filename).getroot()


def arguments(root):
    return {row.attrib['name']: row.attrib for row in root.findall('arg')}


def included(root, filename):
    return next(row for row in root.iter('include') if row.attrib['file'].endswith('/'+filename))


def included_values(root, filename):
    return {row.attrib['name']: row.attrib['value'] for row in included(root, filename).findall('arg')}


def launch_dump(name, overrides=()):
    """Parse real launch XML; an unreachable master URI prevents live contact."""
    tree = os.environ.get('AJOUNICE_TEST_LAUNCH_TREE', 'source')
    if tree not in ('source', 'installed'):
        raise ValueError('AJOUNICE_TEST_LAUNCH_TREE must be source or installed')
    package, filename = LAUNCHES[name]
    launch = (package/'launch'/filename if tree == 'source' else
              ROOT/'base_ws/install_aw149/share'/package.name/'launch'/filename)
    command = [str(launch), 'repo_root:='+str(ROOT), *overrides]
    # Source only setup files, not aliases or launch wrappers. Explicit source
    # package routing also covers nested $(find ...)/launch includes.
    script = 'set -eo pipefail\n'
    for setup in ('/opt/ros/noetic/setup.bash',
                  str(ROOT/'planning_ws/install_isolated/setup.bash'),
                  str(ROOT/'base_ws/install_aw149/setup.bash')):
        script += 'source '+shlex.quote(setup)+'\n'
    if tree == 'source':
        script += 'export ROS_PACKAGE_PATH='+shlex.quote(str(TRAFFIC)+':'+str(PLANNING))+':"$ROS_PACKAGE_PATH"\n'
    script += 'exec roslaunch --dump-params "$@"\n'
    with tempfile.TemporaryDirectory(prefix='ajou_launch_dump_') as ros_home:
        env = dict(PATH='/usr/bin:/bin', LANG='C.UTF-8', HOME=os.environ.get('HOME', '/tmp'),
                   AJOUNICE_ROOT=str(ROOT), ROS_HOME=ros_home,
                   ROS_MASTER_URI='http://127.0.0.1:1')
        result = subprocess.run(['/bin/bash', '--noprofile', '--norc', '-c', script,
                                 'read_only_launch_dump', *command],
                                cwd=ROOT, env=env, capture_output=True, text=True, timeout=45)
    if result.returncode:
        raise AssertionError('Source/installed XML parameter resolution failed; no nodes started:\n'+
                             result.stderr[-6000:]+result.stdout[-1000:])
    payload = yaml.safe_load(result.stdout)
    if not isinstance(payload, dict):
        raise AssertionError('roslaunch --dump-params did not return a parameter mapping')
    return payload


class StaticUnknownGraceReleaseTest(unittest.TestCase):
    def test_cpp_defaults_remain_zero_and_invalid_combinations_still_fail_closed(self):
        for relative in ('src/scene.hpp', 'src/experimental/scene.hpp'):
            text = (CPP/relative).read_text()
            self.assertRegex(text, r'double\s+signal_compliance_unknown_grace_sec\s*\{0\.0\}')
        for relative in ('src/manager.cpp', 'src/experimental/manager.cpp'):
            text = (CPP/relative).read_text()
            self.assertRegex(text, r'node\.param<double>\(ns\s*\+\s*"/signal_compliance_unknown_grace_sec"[^;]+,\s*0\.0\)')
            self.assertRegex(text, r'signal_compliance_unknown_grace_sec\s*>\s*0\.3')
            self.assertRegex(text, r'signal_compliance_unknown_grace_sec\s*>\s*0\.0\s*&&\s*!planner_param_\.enable_signal_compliance_guard')
        header = (CPP/'include/autoware/behavior_velocity_traffic_light_module/signal_compliance_guard.hpp').read_text()
        self.assertRegex(header, r'double\s+unknown_grace_sec_\s*\{0\.0\}')

    def test_node_and_v5_wire_color_evidence_explicitly(self):
        node = xml('node')
        self.assertIn(EVIDENCE, arguments(node))
        parameter = next(row for row in node.iter('param') if row.attrib['name'] == EVIDENCE)
        self.assertEqual(parameter.attrib['value'], '$(arg '+EVIDENCE+')')
        self.assertEqual(parameter.attrib['type'], 'bool')
        v5 = included_values(xml('v5'), 'traffic_light_node.launch')
        self.assertEqual(v5[EVIDENCE], '$(arg '+EVIDENCE+')')
        self.assertEqual(v5['planner_compliance_observations'], '$(arg planner_compliance_observations)')

    def test_legacy_explicitly_disables_evidence_instead_of_inheriting_production(self):
        values = included_values(xml('legacy'), 'traffic_light_node.launch')
        self.assertEqual(values['planner_compliance_observations'], 'false')
        self.assertEqual(values[EVIDENCE], 'false')

    def test_stack_forwards_policy_to_both_owners_and_planner_owns_only_its_namespace(self):
        stack = xml('stack')
        self.assertIn(GRACE, arguments(stack))
        perception = included_values(stack, 'traffic_light_node.launch')
        planning = included_values(stack, 'planning.launch')
        self.assertIn(EVIDENCE, perception)
        self.assertIn(GRACE, planning)
        self.assertNotIn(GRACE, perception)
        self.assertNotIn(EVIDENCE, planning)
        parameter = next(row for row in xml('planning').iter('param')
                         if row.attrib['name'] == PREFIX+GRACE)
        self.assertEqual(parameter.attrib['type'], 'double')

    def test_marker_selection_does_not_read_planning_parameters_implicitly(self):
        node = (TRAFFIC/'scripts/traffic_light_node.py').read_text()
        # The existing read-only debug viewer may inspect planner parameters;
        # behavior configuration must instead consume its own explicit option.
        self.assertRegex(node, r'get_param\([\'"]~preserve_unreadable_color_evidence[\'"]')


@unittest.skipUnless(os.environ.get('AJOUNICE_TEST_LAUNCH_DUMP') == '1',
                     'Set AJOUNICE_TEST_LAUNCH_DUMP=1 for real read-only launch XML resolution')
class ResolvedUnknownGraceReleaseTest(unittest.TestCase):
    def test_node_v5_and_legacy_marker_defaults_are_compatible(self):
        for name in ('node', 'v5'):
            with self.subTest(launch=name):
                params = launch_dump(name)
                self.assertIs(params['/traffic_light_node/'+EVIDENCE], True)
                self.assertIs(params['/traffic_light_node/planner_compliance_observations'], True)
                self.assertEqual(params['/traffic_light_node/stamped_vote_window_size'], 0)
        params = launch_dump('legacy')
        self.assertIs(params['/traffic_light_node/'+EVIDENCE], False)
        self.assertIs(params['/traffic_light_node/planner_compliance_observations'], False)

    def test_standalone_observation_rollback_disables_marker(self):
        for name in ('node', 'v5'):
            with self.subTest(launch=name):
                params = launch_dump(name, ['planner_compliance_observations:=false'])
                self.assertIs(params['/traffic_light_node/'+EVIDENCE], False)

    def test_planning_production_grace_and_yellow_defaults(self):
        params = launch_dump('planning', ['start_requester:=false'])
        self.assertEqual(params[PREFIX+GRACE], .3)
        self.assertIs(params[PREFIX+'enable_signal_compliance_guard'], True)
        self.assertIs(params[PREFIX+'enable_yellow_passthrough'], False)
        self.assertIs(params[PREFIX+'signal_compliance_enable_timed_yellow'], False)

    def test_guard_off_normalizes_incompatible_defaults_and_explicit_requests(self):
        for extra in ([], [GRACE+':=0.3', 'defer_unseen_until_observed:=true',
                           'defer_far_unseen_unknown:=true',
                           'signal_compliance_enable_timed_yellow:=true']):
            with self.subTest(extra=extra):
                params = launch_dump('planning', ['start_requester:=false',
                    'enable_signal_compliance_guard:=false', *extra])
                self.assertEqual(params[PREFIX+GRACE], 0.)
                for key in ('defer_unseen_until_observed', 'defer_far_unseen_unknown',
                            'signal_compliance_enable_timed_yellow'):
                    self.assertIs(params[PREFIX+key], False)

    def test_planner_plugin_off_has_no_grace(self):
        params = launch_dump('planning', ['start_requester:=false',
                                         'enable_traffic_light_planner:=false'])
        self.assertEqual(params[PREFIX+GRACE], 0.)

    def test_full_stack_production_forwards_a_matched_bundle(self):
        params = launch_dump('stack', ['start_requester:=false'])
        self.assertEqual(params[PREFIX+GRACE], .3)
        self.assertIs(params['/traffic_light_node/'+EVIDENCE], True)
        self.assertIs(params['/traffic_light_node/planner_compliance_observations'], True)

    def test_full_stack_inference_off_or_observation_rollback_disables_grace(self):
        for option in ('start_traffic_light:=false', 'planner_compliance_observations:=false',
                       'enable_traffic_light_planner:=false', 'enable_signal_compliance_guard:=false'):
            with self.subTest(option=option):
                params = launch_dump('stack', ['start_requester:=false', option])
                self.assertEqual(params[PREFIX+GRACE], 0.)
                if option != 'start_traffic_light:=false':
                    self.assertIs(params['/traffic_light_node/'+EVIDENCE], False)
                else:
                    self.assertNotIn('/traffic_light_node/'+EVIDENCE, params)


if __name__ == '__main__':
    unittest.main()
