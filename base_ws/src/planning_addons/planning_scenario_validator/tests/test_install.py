"""Check installer entry points without installing tools or writing MORAI files."""
from pathlib import Path
import json
import subprocess
import unittest


INSTALL = Path(__file__).resolve().parents[1] / "install.sh"
NETWORK = INSTALL.parent / 'config/internal/generated/ego_network_validator_local.json'


class InstallEntryTests(unittest.TestCase):
    def test_common_network_is_shipped_and_has_unique_scenario_load(self):
        data = json.loads(NETWORK.read_text(encoding='utf-8'))
        entries = data['egoNetworkData'][0]['listEgoNetworkInfo']
        slots = [entry for entry in entries if entry.get('PUBSUB_TYPE') == 783]
        self.assertEqual(len(slots), 1)
        self.assertEqual(slots[0]['netType'], 1)
        self.assertEqual(slots[0]['config']['commType'], 1)
        self.assertEqual(slots[0]['config']['udpConfig']['hostPort'], 9095)

    def test_common_network_is_not_gitignored_but_personal_generated_files_are(self):
        result = subprocess.run(['git', 'check-ignore', '-v', '--no-index', str(NETWORK)],
                                cwd=str(INSTALL.parent), capture_output=True, text=True)
        pattern = result.stdout.split('\t')[0].rsplit(':', 1)[-1]
        self.assertEqual(pattern, '!ego_network_validator_local.json', result.stdout + result.stderr)
        personal = NETWORK.with_name('planning_quick_personal.yaml')
        result = subprocess.run(['git', 'check-ignore', '--no-index', str(personal)],
                                cwd=str(INSTALL.parent), capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_help_explains_checker_scope(self):
        result = subprocess.run(["bash", str(INSTALL), "--help"],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        self.assertIn("--gui", result.stdout)
        self.assertIn("--bevfusion", result.stdout)
        self.assertIn("--dry-run", result.stdout)
        self.assertIn("SCENARIO_VALIDATOR_TEAM_NOTION_GUIDE.md", result.stdout)

    def test_unknown_and_extra_options_fail_before_setup(self):
        for options in (("--install-everything",), ("--gui", "extra"), ("--check", "--bevfusion"), ("--dry-run",)):
            with self.subTest(options=options):
                result = subprocess.run(["bash", str(INSTALL), *options],
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 2)
                self.assertIn("Usage:", result.stderr)


if __name__ == "__main__":
    unittest.main()
