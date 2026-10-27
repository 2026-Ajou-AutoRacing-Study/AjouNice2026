"""Pure filesystem fixtures: no ROS, network, installed-tree writes, or real assets."""
import copy
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

import check_runtime_map_version as checker


ADDONS = 'base_ws/src/planning_addons'
ROUTE = ADDONS + '/competition_route_requester/config/kcity_v98_competition_route.yaml'
PARAM = ADDONS + '/competition_route_requester/config/competition_route_requester_v98.param.yaml'
ENTRY = ADDONS + '/ajounice_planning_integration/launch/competition_planning_v98.launch'
PLANNING = ADDONS + '/ajounice_planning_integration/launch/planning.launch'
MANIFEST = 'configs/map/kcity_v98_centerline_connection.yaml'
INSTALL = 'base_ws/install_aw149/share/'


class RuntimeMapAuditTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        baseline = b'<osm><MetaInfo map_version="98"/></osm>'
        sha_patch = patch.object(checker, 'SHA', hashlib.sha256(baseline).hexdigest())
        sha_patch.start()
        self.addCleanup(sha_patch.stop)
        profile_patch = patch.object(checker, 'PROFILE_SHA', checker.SHA)
        profile_patch.start()
        self.addCleanup(profile_patch.stop)
        self.write('kcity_map/lanelet_map/lanelet2_map_v98.osm', baseline)
        self.map_contract = dict(version=98,
            file='kcity_map/lanelet_map/lanelet2_map_v98.osm',
            projector_file='kcity_map/lanelet_map/map_projector_info_v1.yaml',
            sha256=checker.PROFILE_SHA)
        self.write_contracts()
        self.write(PLANNING, self.planning())
        self.write(checker.PROFILE_TEST, 'MAP = "kcity_v98_competition_route.yaml"\n')

    def write(self, relative, content):
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(content, bytes):
            path.write_bytes(content)
        else:
            path.write_text(content)
        return path

    def write_yaml(self, relative, value):
        return self.write(relative, yaml.safe_dump(value))

    def write_contracts(self):
        c = self.map_contract
        self.write_yaml(MANIFEST, dict(map_version=c['version'], map_file=c['file'],
            projector_file=c['projector_file'], map_sha256=c['sha256']))
        self.write_yaml(ROUTE, {'map': c})
        self.write_yaml(PARAM, {'route_definition_file': Path(ROUTE).name})
        self.write(ENTRY, '<launch><include file="$(find ajounice_planning_integration)/launch/planning.launch">'
            '<arg name="map_version" value="98"/></include></launch>')

    def planning(self, version='98', default_sha=None, profile_sha=None):
        hashes = {'98': default_sha or profile_sha or checker.SHA}
        return ('<launch><arg name="map_version" default="' + version + '"/>'
            '<arg name="map_file" default="$(arg repo_root)/lanelet2_map_v$(arg map_version).osm"/>'
            '<arg name="map_sha256" default="$(eval ' + repr(hashes) + "[str(arg('map_version'))])\"/>"
            '</launch>')

    def audit(self, installed=False, ignored_source_files=None):
        ignored = ignored_source_files or set()
        files = {str(path.relative_to(self.root)) for path in self.root.rglob('*') if path.is_file()}
        return checker.check_runtime(self.root, installed=installed, source_files=files - ignored,
                                     ignored_source_files=ignored)[1]

    def assert_issue(self, message, installed=False):
        self.assertTrue(any(message in issue for issue in self.audit(installed)), self.audit(installed))

    def install_profile(self):
        for relative in (ROUTE, PARAM, ENTRY, PLANNING):
            installed = INSTALL + relative.split(ADDONS + '/', 1)[1]
            self.write(installed, (self.root / relative).read_bytes())

    def test_default_profile_is_valid_with_pinned_v98_osm(self):
        self.assertEqual(self.audit(), [])
        self.assertEqual(checker.VERSION, 98)

    def test_non_profile_source_still_rejects_non_default_version(self):
        self.write('planning_ws/src/example/loader.cpp', 'map = "lanelet2_map_v87.osm";\n')
        self.assert_issue('lanelet2_map_v87')

    def test_non_profile_runtime_cannot_select_v88_as_default(self):
        self.write('scripts/runtime/example.py', 'MAP = "lanelet2_map_v88.osm"\n')
        self.assert_issue('lanelet2_map_v88')

    def test_profile_test_remains_scanned_for_other_versions(self):
        self.write(checker.PROFILE_TEST, 'MAP = "kcity_v93_competition_route.yaml"\n')
        self.assert_issue('kcity_v93')

    def test_dynamic_map_path_cannot_hide_changed_default(self):
        self.write(PLANNING, self.planning(version='88'))
        self.assert_issue('map_version default must remain 98')

    def test_dynamic_map_path_requires_a_local_default(self):
        self.write(PLANNING, '<launch><arg name="map_file" '
            'default="lanelet2_map_v$(arg map_version).osm"/></launch>')
        self.assert_issue('no local map_version default')

    def test_ordinary_include_cannot_force_old_version(self):
        self.write(PLANNING, self.planning().replace('</launch>',
            '<include file="example.launch"><arg name="map_version" value="88"/></include></launch>'))
        self.assert_issue('ordinary include overrides map_version')

    def test_forwarded_released_map_version_is_valid(self):
        self.write(PLANNING, self.planning().replace('</launch>',
            '<include file="example.launch"><arg name="map_version" '
            'value="$(arg map_version)"/></include></launch>'))
        self.assertEqual(self.audit(), [])

    def test_default_sha_lookup_remains_pinned(self):
        self.write(PLANNING, self.planning(default_sha='0' * 64))
        self.assert_issue('default map SHA selector mismatch')

    def test_explicit_sha_lookup_matches_its_manifest(self):
        self.write(PLANNING, self.planning(profile_sha='0' * 64))
        self.assert_issue('explicit v98 map SHA selector mismatch')

    def test_sha_selector_is_not_executed(self):
        expression = "$(eval __import__('os').system('false'))"
        with self.assertRaisesRegex(ValueError, 'literal dictionary'):
            checker.sha_lookup(expression)

    def test_manifest_sha_is_independently_pinned(self):
        manifest = yaml.safe_load((self.root / MANIFEST).read_text())
        manifest['map_sha256'] = '0' * 64
        self.write_yaml(MANIFEST, manifest)
        self.assert_issue('manifest version/path/projector/SHA contract mismatch')

    def test_profile_route_version_path_and_sha_must_match(self):
        for key, value in (('version', 88), ('file', 'another.osm'), ('sha256', '0' * 64)):
            with self.subTest(key=key):
                contract = copy.deepcopy(self.map_contract)
                contract[key] = value
                self.write_yaml(ROUTE, {'map': contract})
                self.assert_issue('route map contract differs from manifest')

    def test_profile_requester_cannot_select_v88_definition(self):
        self.write_yaml(PARAM, {'route_definition_file': 'kcity_v88_competition_route.yaml'})
        self.assert_issue('requester selects a different route')

    def test_profile_wrapper_must_explicitly_select_94(self):
        self.write(ENTRY, (self.root / ENTRY).read_text().replace('value="98"', 'value="88"'))
        self.assert_issue('launcher does not select planning map_version=98')

    def test_optional_provisioned_asset_cannot_have_wrong_bytes(self):
        self.write(self.map_contract['file'], b'corrupted team Drive asset')
        self.assert_issue('explicit v98 map SHA mismatch')

    def test_profile_asset_metadata_also_matches_its_sha(self):
        content = b'<osm><MetaInfo map_version="93"/></osm>'
        with patch.object(checker, 'PROFILE_SHA', hashlib.sha256(content).hexdigest()):
            self.map_contract['sha256'] = checker.PROFILE_SHA
            self.write_contracts()
            self.write(PLANNING, self.planning())
            self.write(self.map_contract['file'], content)
            self.assert_issue('OSM MetaInfo version mismatch')

    def test_released_map_hash_guard_remains_mandatory(self):
        self.write('kcity_map/lanelet_map/lanelet2_map_v98.osm', b'changed')
        self.assert_issue('Map SHA mismatch')

    def test_installed_explicit_profile_is_checked_not_rejected_as_default(self):
        self.install_profile()
        self.assertEqual(self.audit(installed=True), [])

    def test_missing_installed_profile_is_reported(self):
        self.write(INSTALL + 'ajounice_planning_integration/launch/planning.launch', self.planning())
        self.assert_issue('install_aw149 v98 profile', installed=True)

    def test_installed_profile_cannot_mix_v88_route_metadata(self):
        self.install_profile()
        contract = dict(self.map_contract, version=88)
        self.write_yaml(INSTALL + ROUTE.split(ADDONS + '/', 1)[1], {'map': contract})
        self.assert_issue('route map contract differs from manifest', installed=True)

    def test_installed_ordinary_sources_keep_the_original_scan_scope(self):
        self.install_profile()
        self.write(INSTALL + 'another_package/config/runtime.yaml', 'map: lanelet2_map_v87.osm\n')
        self.assert_issue('lanelet2_map_v87', installed=True)

    def test_original_historical_and_frozen_exclusions_are_preserved(self):
        for relative in ('configs/map/kcity_v93_stopline_candidate.yaml',
                         ADDONS + '/competition_route_requester/config/kcity_v87_competition_route.yaml',
                         'base_ws/src/example/legacy/loader.cpp',
                         'planning_ws/src/example/docs/note.py',
                         'scripts/runtime/generate_highway_lane_change_samples.py'):
            self.write(relative, 'map = "lanelet2_map_v41.osm"\n')
        self.assertEqual(self.audit(), [])

    def test_exact_preparer_tool_references_are_not_runtime_map_selection(self):
        self.write(checker.UPDATE_WRAPPER, 'MAP_VERSION=98\n'
            '/usr/bin/python3 "$ROOT/scripts/map/prepare_kcity_v98.py"\n' + checker.RETIRED_REQUESTER_CLEANUP + '\n')
        self.write('scripts/runtime/test_update_competition_runtime.py',
                   'fixture = "scripts/map/prepare_kcity_v98.py"\n')
        self.assertEqual(self.audit(), [])

    def test_preparer_exception_does_not_allow_map_reference_on_same_line(self):
        self.write(checker.UPDATE_WRAPPER, 'MAP_VERSION=98\n'
            'TOOL="scripts/map/prepare_kcity_v98.py" MAP="lanelet2_map_v88.osm"\n')
        self.assert_issue('lanelet2_map_v88')

    def test_other_callers_may_prepare_the_default_v98_asset(self):
        self.write('scripts/runtime/other.py', 'TOOL="scripts/map/prepare_kcity_v98.py"\n')
        self.assertEqual(self.audit(), [])

    def test_runtime_update_wrapper_keeps_a_literal_released_default(self):
        for default in ('88', '"88"', '${MAP_VERSION:-94}'):
            with self.subTest(default=default):
                self.write(checker.UPDATE_WRAPPER, 'MAP_VERSION=' + default + '\n')
                self.assert_issue('wrapper MAP_VERSION default must remain 98')

    def workspace_copy(self, *, installed_content=None):
        tail = 'highway/workspace/personal_snapshot.yaml'
        relative = checker.VALIDATOR_MISSIONS + '/' + tail
        content = 'map: lanelet2_map_v85.osm\n'
        self.write(relative, content)
        self.write(INSTALL + 'planning_scenario_validator/config/missions/' + tail,
                   installed_content or content)
        return relative

    def test_identical_ignored_personal_workspace_copy_matches_source_exclusion(self):
        self.install_profile()
        source = self.workspace_copy()
        self.assertEqual(self.audit(installed=True, ignored_source_files={source}), [])

    def test_workspace_without_git_ignored_evidence_is_still_scanned(self):
        self.install_profile()
        self.workspace_copy()
        self.assert_issue('lanelet2_map_v85', installed=True)

    def test_different_installed_personal_copy_is_not_silently_ignored(self):
        self.install_profile()
        source = self.workspace_copy(installed_content='map: lanelet2_map_v87.osm\n')
        issues = self.audit(installed=True, ignored_source_files={source})
        self.assertTrue(any('lanelet2_map_v87' in issue for issue in issues))

    def test_orphan_installed_editor_remains_a_reported_consumer(self):
        self.install_profile()
        self.write(INSTALL + 'bringup/launch/aw149_lanelet_speed_editor.launch',
            '<launch><param name="route_definition_file" '
            'value="kcity_v85_competition_route.yaml"/></launch>')
        self.assert_issue('kcity_v85', installed=True)


if __name__ == '__main__':
    unittest.main()
