"""Check installer entry points without installing tools or writing MORAI files."""
from pathlib import Path
import subprocess
import unittest


INSTALL = Path(__file__).resolve().parents[1] / "install.sh"


class InstallEntryTests(unittest.TestCase):
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
