"""Dry runner qualification: no real Docker, ROS, master, or inference service."""

import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


class FrameTimingRunnerTests(unittest.TestCase):
    def run_fixture(self, value=None, *, defined=True, local_value=None, model="e24"):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runtime = root / "scripts/runtime"
            workspace = root / "scripts/workspace"
            binaries = root / "bin"
            for directory in (runtime, workspace, binaries):
                directory.mkdir(parents=True)
            shutil.copyfile(ROOT / "scripts/runtime/run_bevfusion.sh", runtime / "run_bevfusion.sh")
            (workspace / "source_aw149.sh").write_text(":\n", encoding="utf-8")
            checkpoint = root / "checkpoint.pth"
            checkpoint.write_bytes(b"local fixture checkpoint; not model weights")
            (root / "model.yaml").write_text("fixture: true\n", encoding="utf-8")
            (root / "sensor.json").write_text("{}\n", encoding="utf-8")
            (root / "profile.fields").write_text("\n".join([
                str(checkpoint), hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
                "model.yaml", "bridge.yaml", "base5_production.launch", "true", "true", "sensor.json",
            ]) + "\n", encoding="utf-8")
            log = root / "calls.log"
            mocks = {
                "python3": '#!/bin/bash\nif [[ "$1" == *bevfusion_production_profile.py ]]; then /bin/cat "$AJOUNICE_FIXTURE_ROOT/profile.fields"; fi\n',
                "docker": '#!/bin/bash\nprintf "docker" >> "$AJOUNICE_FIXTURE_LOG"; printf " <%s>" "$@" >> "$AJOUNICE_FIXTURE_LOG"; printf "\\n" >> "$AJOUNICE_FIXTURE_LOG"\n',
                "rostopic": '#!/bin/bash\nexit 1\n',
                "roslaunch": '#!/bin/bash\nprintf "roslaunch" >> "$AJOUNICE_FIXTURE_LOG"; printf " <%s>" "$@" >> "$AJOUNICE_FIXTURE_LOG"; printf "\\n" >> "$AJOUNICE_FIXTURE_LOG"\n',
            }
            for name, text in mocks.items():
                target = binaries / name
                target.write_text(text, encoding="utf-8")
                target.chmod(0o755)
            if local_value is not None:
                (root / ".env.bevfusion.local").write_text(
                    f"BEVFUSION_PUBLISH_FRAME_TIMING={local_value}\n", encoding="utf-8")
            environment = dict(os.environ)
            # Do not inherit local model/checkpoint/sensor overrides into a dry fixture.
            for name in list(environment):
                if name.startswith("BEVFUSION_"):
                    environment.pop(name)
            environment.update(PATH=f"{binaries}:{os.environ['PATH']}",
                               AJOUNICE_FIXTURE_ROOT=str(root), AJOUNICE_FIXTURE_LOG=str(log))
            if defined and value is not None:
                environment["BEVFUSION_PUBLISH_FRAME_TIMING"] = value
            result = subprocess.run(["bash", str(runtime / "run_bevfusion.sh"), "--model", model],
                                    env=environment, text=True, capture_output=True, timeout=10)
            return result, log.read_text(encoding="utf-8") if log.exists() else ""

    def test_unset_defaults_false_and_owns_only_mock_server(self):
        result, log = self.run_fixture(defined=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("<publish_frame_timing:=false>", log)
        self.assertIn("<up> <-d> <inference>", log)
        self.assertIn("<stop> <inference>", log)
        self.assertEqual(log.count("roslaunch"), 1)

    def test_true_and_explicit_false_pass_through_without_policy_override(self):
        for value in ("true", "false"):
            with self.subTest(value=value):
                result, log = self.run_fixture(value)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn(f"<publish_frame_timing:={value}>", log)
                self.assertIn("<bevfusion_ros> <base5_production.launch>", log)
                self.assertNotIn("score_threshold", log)

    def test_invalid_values_fail_before_any_docker_or_ros_command(self):
        for value in ("TRUE", "1", "yes", "", "true false"):
            with self.subTest(value=value):
                result, log = self.run_fixture(value)
                self.assertEqual(result.returncode, 2)
                self.assertIn("must be true or false", result.stderr)
                self.assertEqual(log, "")

    def test_local_environment_config_is_loaded_then_validated(self):
        result, log = self.run_fixture(defined=False, local_value="true")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("<publish_frame_timing:=true>", log)
        result, log = self.run_fixture(defined=False, local_value="invalid")
        self.assertEqual(result.returncode, 2)
        self.assertEqual(log, "")

    def test_explicit_e25_rollback_keeps_timing_arg(self):
        result, log = self.run_fixture("true", model="e25")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("<publish_frame_timing:=true>", log)


if __name__ == "__main__":
    unittest.main()
