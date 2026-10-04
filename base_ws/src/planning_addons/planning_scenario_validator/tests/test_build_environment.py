"""Fresh-shell ROS source compatibility; never launches a build or mutates ROS."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[5]
BUILD = ROOT / 'scripts/workspace/build_all.sh'


class BuildEnvironmentTests(unittest.TestCase):
    def execute(self, source, directory):
        script = Path(directory) / 'check.sh'
        script.write_text(source, encoding='utf-8')
        # Deliberately no ROS_DISTRO/ROS_MASTER_URI/Conda or inherited overlays.
        return subprocess.run(
            ['bash', '--noprofile', '--norc', str(script)],
            env={'PATH': '/usr/bin:/bin', 'HOME': directory, 'LANG': 'C.UTF-8'},
            text=True, capture_output=True, timeout=20)

    @unittest.skipUnless(Path('/opt/ros/noetic/setup.bash').exists(), 'ROS Noetic required')
    def test_fresh_noetic_setup_preserves_strict_mode(self):
        prefix = BUILD.read_text().split('PLANNING_INSTALL=', 1)[0]
        with tempfile.TemporaryDirectory() as directory:
            result = self.execute(prefix + '\n'
                'test "$ROS_DISTRO" = noetic\n'
                'test -n "$ROS_MASTER_URI"\n'
                'case "$-" in *u*) ;; *) exit 31 ;; esac\n'
                'case "$-" in *e*) ;; *) exit 32 ;; esac\n', directory)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_planning_setup_also_allows_unset_variables_then_restores_nounset(self):
        content = BUILD.read_text()
        block = content.split('set +u\nsource "$PLANNING_INSTALL/setup.bash"', 1)
        self.assertEqual(len(block), 2)
        self.assertTrue(block[1].startswith('\nset -u'))
        with tempfile.TemporaryDirectory() as directory:
            setup = Path(directory) / 'setup.bash'
            setup.write_text('if [ -n "$ROS_DISTRO" ]; then :; fi\nexport ROS_DISTRO=noetic\n')
            source = ('set -euo pipefail\nPLANNING_INSTALL="' + directory + '"\n'
                      'set +u\nsource "$PLANNING_INSTALL/setup.bash"\nset -u\n'
                      ': "$AJOUNICE_TEST_UNDEFINED"\n')
            result = self.execute(source, directory)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('AJOUNICE_TEST_UNDEFINED', result.stderr)
        self.assertNotIn('ROS_DISTRO: unbound variable', result.stderr)

    def test_source_failure_is_not_swallowed(self):
        prefix = BUILD.read_text().split('PLANNING_INSTALL=', 1)[0]
        with tempfile.TemporaryDirectory() as directory:
            setup = Path(directory) / 'bad-setup.bash'
            setup.write_text('return 7\n')
            prefix = prefix.replace('source /opt/ros/noetic/setup.bash', 'source "' + str(setup) + '"')
            result = self.execute(prefix + '\necho UNEXPECTED_CONTINUE\n', directory)
        self.assertEqual(result.returncode, 7, result.stderr)
        self.assertNotIn('UNEXPECTED_CONTINUE', result.stdout)

    def audit_fixture(self, filename, package_level, nested_installed):
        # Run the actual audit loop against only a disposable miniature tree.
        block = BUILD.read_text().split('missing_assets=0', 1)[1].split('if (( missing_assets', 1)[0]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root / 'base_ws/src/example'
            (package / 'config').mkdir(parents=True)
            (package / 'package.xml').write_text('<package><name>example</name></package>\n')
            (package / 'config' / filename).write_text('nested source')
            installed = root / 'install/share/example'
            installed.mkdir(parents=True)
            (installed / filename).write_text('root installed')
            if package_level:
                (package / filename).write_text('root source')
            if nested_installed:
                (installed / 'config').mkdir()
                (installed / 'config' / filename).write_text('nested installed')
            script = ('set -euo pipefail\nREPO_ROOT="' + directory + '"\n'
                      'FAST_LIO_INSTALL="' + str(root / 'install') + '"\n'
                      'missing_assets=0' + block + '\n'
                      'printf "audit_errors=%s\\n" "$missing_assets"\n')
            return self.execute(script, directory)

    def test_package_readme_is_not_misidentified_as_flat_runtime_asset(self):
        result = self.audit_fixture('README.md', package_level=True, nested_installed=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('audit_errors=0', result.stdout)
        self.assertNotIn('ERROR:', result.stderr)

    def test_real_missing_and_flat_runtime_assets_still_fail_audit(self):
        result = self.audit_fixture('node.yaml', package_level=False, nested_installed=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('missing installed runtime asset:', result.stderr)
        self.assertIn('ambiguous legacy flat install asset:', result.stderr)
        self.assertIn('audit_errors=2', result.stdout)


if __name__ == '__main__':
    unittest.main()
