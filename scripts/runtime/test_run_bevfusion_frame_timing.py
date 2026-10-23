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
    def run_fixture(self, value=None, *, defined=True, local_value=None, model="e24",
                    extra_environment=None, extra_args=(), server_running=False, health_matches=True,
                    server_backend="eager", server_pool_fused_layout=False, health_fail_first=0):
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
                "python3": '''#!/bin/bash
if [[ "$1" == *bevfusion_production_profile.py ]]; then
  /bin/cat "$AJOUNICE_FIXTURE_ROOT/profile.fields"
elif [[ "$1" == *healthcheck.py ]]; then
  shift
  printf "healthcheck" >> "$AJOUNICE_FIXTURE_LOG"
  printf " <%s>" "$@" >> "$AJOUNICE_FIXTURE_LOG"
  printf "\\n" >> "$AJOUNICE_FIXTURE_LOG"
  health_count=0
  if [[ -f "$AJOUNICE_FIXTURE_ROOT/health.count" ]]; then
    read -r health_count < "$AJOUNICE_FIXTURE_ROOT/health.count"
  fi
  health_count=$((health_count + 1))
  printf '%s\\n' "$health_count" > "$AJOUNICE_FIXTURE_ROOT/health.count"
  if (( health_count <= AJOUNICE_FIXTURE_HEALTH_FAIL_FIRST )); then exit 1; fi
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --expected-camera-backend)
        [[ "$2" == "$AJOUNICE_FIXTURE_SERVER_BACKEND" ]] || exit 1
        shift 2 ;;
      --expected-pool-fused-layout)
        [[ "$2" == "$AJOUNICE_FIXTURE_SERVER_POOL" ]] || exit 1
        shift 2 ;;
      *) shift ;;
    esac
  done
  exit "$AJOUNICE_FIXTURE_HEALTH_EXIT"
fi
''',
                "docker": '''#!/bin/bash
printf "docker" >> "$AJOUNICE_FIXTURE_LOG"
printf " <%s>" "$@" >> "$AJOUNICE_FIXTURE_LOG"
printf "\\n" >> "$AJOUNICE_FIXTURE_LOG"
printf "runtime intrinsic=%s image=%s engine=%s policy=%s pool=%s\\n" "${INTRINSIC_SOURCE-unset}" "${BEVFUSION_IMAGE-unset}" "${BEVFUSION_CAMERA_TENSORRT_ENGINE-unset}" "${BEVFUSION_CAMERA_TENSORRT_POLICY-unset}" "${BEVFUSION_POOL_FUSED_LAYOUT-unset}" >> "$AJOUNICE_FIXTURE_LOG"
printf "runtime_limits cpu=%s opencv=%s workers=%s warmup=%s\\n" "${BEVFUSION_CPU_THREADS-unset}" "${BEVFUSION_OPENCV_THREADS-unset}" "${BEVFUSION_PREPROCESS_WORKERS-unset}" "${BEVFUSION_WARMUP_ITERATIONS-unset}" >> "$AJOUNICE_FIXTURE_LOG"
for argument in "$@"; do
  if [[ "$argument" == ps && "$AJOUNICE_FIXTURE_SERVER_RUNNING" == true ]]; then
    printf "fixture-preexisting-container\\n"
    break
  fi
done
''',
                "rostopic": '#!/bin/bash\nexit 1\n',
                "roslaunch": '#!/bin/bash\nprintf "roslaunch" >> "$AJOUNICE_FIXTURE_LOG"; printf " <%s>" "$@" >> "$AJOUNICE_FIXTURE_LOG"; printf "\\n" >> "$AJOUNICE_FIXTURE_LOG"\n',
                "sleep": '#!/bin/bash\nprintf "sleep <%s>\\n" "$@" >> "$AJOUNICE_FIXTURE_LOG"\n',
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
                               AJOUNICE_FIXTURE_ROOT=str(root), AJOUNICE_FIXTURE_LOG=str(log),
                               AJOUNICE_FIXTURE_SERVER_RUNNING=str(server_running).lower(),
                               AJOUNICE_FIXTURE_HEALTH_EXIT="0" if health_matches else "1",
                               AJOUNICE_FIXTURE_HEALTH_FAIL_FIRST=str(health_fail_first),
                               AJOUNICE_FIXTURE_SERVER_BACKEND=server_backend,
                               AJOUNICE_FIXTURE_SERVER_POOL=str(server_pool_fused_layout).lower())
            if defined and value is not None:
                environment["BEVFUSION_PUBLISH_FRAME_TIMING"] = value
            environment.update(extra_environment or {})
            result = subprocess.run(["bash", str(runtime / "run_bevfusion.sh"), "--model", model, *extra_args],
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

    def test_stale_intrinsic_environment_cannot_break_bridge_handshake(self):
        result, log = self.run_fixture(extra_environment={"INTRINSIC_SOURCE": "focal_length_pixel"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("runtime intrinsic=horizontal_fov", log)
        self.assertNotIn("intrinsic=focal_length_pixel", log)

    def test_safe_mode_disables_optional_acceleration_without_switching_model(self):
        result, log = self.run_fixture(extra_environment={
            "BEVFUSION_IMAGE": "wrong-trt-image", "BEVFUSION_CAMERA_TENSORRT_ENGINE": "E12.engine",
            "BEVFUSION_CAMERA_TENSORRT_METADATA": "wrong.json", "BEVFUSION_CAMERA_TENSORRT_POLICY": "require",
            "BEVFUSION_POOL_FUSED_LAYOUT": "true", "BEVFUSION_CPU_THREADS": "99",
            "BEVFUSION_OPENCV_THREADS": "99", "BEVFUSION_PREPROCESS_WORKERS": "99",
            "BEVFUSION_WARMUP_ITERATIONS": "0"}, extra_args=("--safe",))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("image=ajounice/bevfusion:team-cu128-v1 engine=unset policy=off pool=false", log)
        self.assertIn("runtime_limits cpu=4 opencv=1 workers=4 warmup=2", log)
        self.assertIn("Production e24", result.stdout)
        self.assertIn("<base5_production.launch>", log)

    def test_invalid_safe_policy_and_unknown_flags_fail_before_docker(self):
        for env, args in (({"BEVFUSION_SAFE_MODE": "yes"}, ()), ({}, ("--typo",))):
            result, log = self.run_fixture(extra_environment=env, extra_args=args)
            self.assertEqual(result.returncode, 2)
            self.assertEqual(log, "")

    def test_matching_preexisting_eager_server_is_reused_without_taking_ownership(self):
        result, log = self.run_fixture(server_running=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Reusing the already-ready matching eager server", result.stdout)
        self.assertIn("pre-existing matching server is not owned by this session", result.stdout)
        self.assertIn("<ps> <--status> <running> <-q> <inference>", log)
        self.assertIn("<--expected-camera-backend> <eager>", log)
        self.assertNotIn("<up>", log)
        self.assertNotIn("<stop>", log)
        self.assertEqual(log.count("roslaunch"), 1)
        self.assertEqual(len([line for line in log.splitlines() if line.startswith("healthcheck")]), 1)

    def test_mismatched_preexisting_server_is_refused_without_stopping_other_service(self):
        result, log = self.run_fixture(server_running=True, health_matches=False)
        self.assertEqual(result.returncode, 1)
        self.assertIn("did not become ready with the selected profile", result.stderr)
        self.assertIn("no unrelated detector is killed", result.stderr)
        self.assertNotIn("<up>", log)
        self.assertNotIn("<stop>", log)
        self.assertNotIn("roslaunch", log)
        # 60 bounded readiness attempts plus one visible diagnostic check.
        self.assertEqual(len([line for line in log.splitlines() if line.startswith("healthcheck")]), 61)
        self.assertEqual(log.splitlines().count("sleep <2>"), 60)

    def test_existing_cold_server_finishes_warmup_then_reuses_without_ownership(self):
        result, log = self.run_fixture(server_running=True, health_fail_first=3)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Waiting for the existing server to finish warmup", result.stdout)
        self.assertIn("Reusing the already-ready matching eager server", result.stdout)
        self.assertEqual(len([line for line in log.splitlines() if line.startswith("healthcheck")]), 4)
        self.assertEqual(log.splitlines().count("sleep <2>"), 3)
        self.assertNotIn("<up>", log)
        self.assertNotIn("<stop>", log)
        self.assertEqual(log.count("roslaunch"), 1)
        self.assertLess(log.rindex("healthcheck"), log.index("roslaunch"))

    def test_new_owned_server_keeps_ownership_after_same_readiness_wait(self):
        result, log = self.run_fixture(server_running=False, health_fail_first=2)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len([line for line in log.splitlines() if line.startswith("healthcheck")]), 3)
        self.assertEqual(log.splitlines().count("sleep <2>"), 2)
        self.assertIn("<up> <-d> <inference>", log)
        self.assertIn("<stop> <inference>", log)
        self.assertEqual(log.count("roslaunch"), 1)

    def test_explicit_tensorrt_options_cannot_reuse_or_reconfigure_preexisting_server(self):
        for options in (
                {"BEVFUSION_CAMERA_TENSORRT_ENGINE": "explicit.engine"},
                {"BEVFUSION_CAMERA_TENSORRT_METADATA": "explicit.json"},
                {"BEVFUSION_CAMERA_TENSORRT_ENGINE": "explicit.engine",
                 "BEVFUSION_CAMERA_TENSORRT_METADATA": "explicit.json",
                 "BEVFUSION_CAMERA_TENSORRT_POLICY": "require"}):
            with self.subTest(options=options):
                result, log = self.run_fixture(server_running=True, extra_environment=options)
                self.assertEqual(result.returncode, 1)
                self.assertIn("stop its owning session before changing acceleration options", result.stderr)
                self.assertNotIn("<up>", log)
                self.assertNotIn("<stop>", log)
                self.assertNotIn("healthcheck", log)
                self.assertNotIn("roslaunch", log)

    def test_safe_reuse_requires_eager_backend_without_changing_selected_profile(self):
        result, log = self.run_fixture(server_running=True, extra_args=("--safe",), extra_environment={
            "BEVFUSION_CAMERA_TENSORRT_ENGINE": "E12.engine",
            "BEVFUSION_CAMERA_TENSORRT_METADATA": "E12.json",
            "BEVFUSION_CAMERA_TENSORRT_POLICY": "require", "BEVFUSION_POOL_FUSED_LAYOUT": "true"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("engine=unset policy=off pool=false", log)
        self.assertIn("<--expected-camera-backend> <eager>", log)
        self.assertIn("<--require-box-origin-tag>", log)
        self.assertIn("<--expected-depth-raster-policy> <nearest_positive>", log)
        self.assertIn("<base5_production.launch>", log)
        self.assertNotIn("<up>", log)
        self.assertNotIn("<stop>", log)
        self.assertEqual(log.count("roslaunch"), 1)

    def test_require_policy_cannot_silently_reuse_existing_eager_server_without_engine_paths(self):
        result, log = self.run_fixture(server_running=True, server_backend="eager",
            extra_environment={"BEVFUSION_CAMERA_TENSORRT_POLICY": "require"})
        self.assertEqual(result.returncode, 1)
        self.assertIn("<--expected-camera-backend> <camera_tensorrt>", log)
        self.assertNotIn("<up>", log)
        self.assertNotIn("<stop>", log)
        self.assertNotIn("roslaunch", log)

    def test_safe_mode_does_not_reuse_existing_fused_pool_server(self):
        result, log = self.run_fixture(server_running=True, server_pool_fused_layout=True,
                                       extra_args=("--safe",))
        self.assertEqual(result.returncode, 1)
        self.assertIn("<--expected-camera-backend> <eager>", log)
        self.assertIn("<--expected-pool-fused-layout> <false>", log)
        self.assertNotIn("<up>", log)
        self.assertNotIn("<stop>", log)
        self.assertNotIn("roslaunch", log)


if __name__ == "__main__":
    unittest.main()
