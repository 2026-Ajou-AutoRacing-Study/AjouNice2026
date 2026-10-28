"""No ROS master, process startup or vehicle actuation in these tests."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch


SPEC = importlib.util.spec_from_file_location("gnss_delay_validator", Path(__file__).with_name("run_gnss_delay_validator.py"))
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


class CalibrationValidatorTests(unittest.TestCase):
    def test_default_forces_real_localization_and_od_gt(self):
        args = M.parse_args([])
        document = M.campaign_document(args)
        self.assertEqual(document["stack"]["detector"], "od_gt")
        self.assertEqual(document["stack"]["pose_source"], "localization")
        self.assertEqual(document["stack"]["localization_args"]["gnss_delay_ms"], 0.)
        self.assertFalse(document["stack"]["planviz"])
        self.assertFalse(document["stack"]["traffic_light"])
        self.assertFalse(document["recording"]["video"]["enabled"])
        self.assertFalse(document["recording"]["include_sensors"])
        self.assertTrue(document["validation"]["localization_safety"]["park_after_run"])
        self.assertEqual(document["scenarios"]["repeat"], 2)

    def test_explicit_on_value_does_not_double_apply_old_seconds(self):
        document = M.default_document()
        document["stack"]["localization_args"] = {"gnss_motion_compensation_sec": .02}
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "config.yaml"
            config.write_text(M.yaml.safe_dump(document))
            args = M.parse_args(["--config", str(config), "--gnss-delay-ms", "30", "--planviz"])
            changed = M.campaign_document(args)
            self.assertEqual(changed["stack"]["localization_args"]["gnss_delay_ms"], 30.)
            self.assertEqual(changed["stack"]["localization_args"]["gnss_motion_compensation_sec"], 0.)
            self.assertTrue(changed["stack"]["planviz"])
            self.assertEqual(M.yaml.safe_load(config.read_text()), document)

    def test_invalid_delay_or_provenance_rejected(self):
        for argv in (["--gnss-delay-ms", "nan"], ["--gnss-delay-ms", "-1"],
                     ["--gnss-delay-ms", "101"], ["--timeout-sec", "0"],
                     ["--repeats", "0"], ["--repeats", "4"],
                     ["--heading-offset-deg", "0"], ["--acquisition", "MRP"]):
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                M.parse_args(argv)

    def test_analyze_only_can_explicitly_use_mrp_heading(self):
        args = M.parse_args(["--analyze-execution", "/tmp/example", "--acquisition", "MRP", "--heading-offset-deg", "0"])
        self.assertEqual(args.heading_offset_deg, 0.)

    def test_check_and_dry_run_never_connect_or_create_output(self):
        for flag in ("--check", "--dry-run"):
            with tempfile.TemporaryDirectory() as directory:
                destination = Path(directory) / "must_not_exist"
                with patch.object(M, "master_preflight") as master, \
                        patch.object(M, "bounded_run") as runner, \
                        contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(M.main([flag, "--output-dir", str(destination)]), 0)
                self.assertFalse(destination.exists())
                master.assert_not_called()
                runner.assert_not_called()

    def test_manual_stack_conflict_never_kills_it(self):
        fake = Mock()
        fake.getPid.return_value = [1, "OK", 123]
        fake.getSystemState.return_value = [1, "OK", [[["/localization/kinematic_state", ["/manual_loc"]]], [], []]]
        proxy = Mock()
        proxy.__enter__ = Mock(return_value=fake)
        proxy.__exit__ = Mock(return_value=False)
        with patch.object(M.xmlrpc.client, "ServerProxy", return_value=proxy), self.assertRaisesRegex(ValueError, "Nothing was killed"):
            M.master_preflight()

    def test_missing_master_is_not_auto_started(self):
        with patch.object(M.xmlrpc.client, "ServerProxy", side_effect=OSError("refused")), \
                self.assertRaisesRegex(ValueError, "start roscore"):
            M.master_preflight()

    def test_new_private_scene_preserves_original_and_removes_shadow(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            scene = base / "original.json"
            original = {"shadedAreaList": [{"noise": 1}], "egoVehicle": {"value": "unchanged"}}
            scene.write_text(json.dumps(original))
            output = base / "run"
            output.mkdir()
            document = M.default_document()
            path, hashes = M.make_inputs(document, output, [scene])
            generated = next((output / "inputs/scenarios").glob("*.json"))
            self.assertEqual(json.loads(scene.read_text()), original)
            self.assertEqual(json.loads(generated.read_text())["shadedAreaList"], [])
            self.assertEqual(json.loads(generated.read_text())["egoVehicle"], original["egoVehicle"])
            self.assertTrue(hashes[str(scene)]["gps_shadow_removed"])
            self.assertTrue(path.is_file())
            self.assertEqual(document["campaign_name"], "gnss_delay_calibration")

    def test_keep_shadow_only_explicitly(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            scene = base / "scene.json"
            scene.write_text(json.dumps({"shadedAreaList": [1]}))
            M.make_inputs(M.default_document(), base, [scene], keep_gps_shadow=True)
            generated = next((base / "inputs/scenarios").glob("*.json"))
            self.assertEqual(json.loads(generated.read_text())["shadedAreaList"], [1])

    def test_park_audit_requires_every_case_and_actual_park_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            case = base / "success/case"
            case.mkdir(parents=True)
            receipt = case / "safety_receipt.json"
            value = {"enabled": True, "shutdown": {"status": "VERIFIED", "stop": {"park_verified": True}}}
            receipt.write_text(json.dumps(value))
            self.assertTrue(M.park_audit(base, 1)["verified"])
            self.assertFalse(M.park_audit(base, 2)["verified"])
            value["shutdown"]["stop"]["park_verified"] = False
            receipt.write_text(json.dumps(value))
            self.assertFalse(M.park_audit(base, 1)["verified"])

    def test_execution_bags_excludes_active_and_requires_finalized(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            case = base / "success/case"
            case.mkdir(parents=True)
            (case / "record.bag.active").touch()
            with self.assertRaises(ValueError):
                M.execution_bags(base)
            (case / "record.bag").touch()
            self.assertEqual(M.execution_bags(base), [case / "record.bag"])

    def test_share_payload_and_running_do_not_duplicate_bags_or_park_receipts(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            receipt = {"enabled": True, "shutdown": {"status": "VERIFIED", "stop": {"park_verified": True}}}
            for relative in ("success/case", "share_payload/success/case", "backup/success/case", "running/case"):
                case = base / relative
                case.mkdir(parents=True)
                (case / "record.bag").touch()
                (case / "safety_receipt.json").write_text(json.dumps(receipt))
            self.assertEqual(M.execution_bags(base), [base / "success/case/record.bag"])
            self.assertTrue(M.park_audit(base, 1)["verified"])
            self.assertEqual(len(M.park_audit(base, 1)["receipts"]), 1)

    def test_worker_timeout_interrupts_only_owned_worker_and_awaits_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            process = Mock(pid=1234)
            process.wait.side_effect = [subprocess.TimeoutExpired("worker", 1), 130]
            with patch.object(M.subprocess, "Popen", return_value=process):
                result = M.bounded_run(["fake-worker"], Path(directory) / "log", 1)
            process.send_signal.assert_called_once_with(signal.SIGINT)
            self.assertEqual(result["shutdown"], "WORKER_EXITED")
            self.assertEqual(result["returncode"], 130)

    def test_unfinished_shutdown_never_claims_park_or_sends_sigkill(self):
        with tempfile.TemporaryDirectory() as directory:
            process = Mock(pid=1234)
            process.wait.side_effect = subprocess.TimeoutExpired("worker", 1)
            with patch.object(M.subprocess, "Popen", return_value=process):
                result = M.bounded_run(["fake-worker"], Path(directory) / "log", 1)
            process.send_signal.assert_called_once_with(signal.SIGINT)
            self.assertIsNone(result["returncode"])
            self.assertEqual(result["shutdown"], "UNCONFIRMED_WORKER_STILL_RUNNING")

    def test_terminal_hangup_and_term_gracefully_stop_worker_and_restore_handlers(self):
        for signum in (signal.SIGHUP, signal.SIGTERM):
            with self.subTest(signum=signum), tempfile.TemporaryDirectory() as directory:
                original = {sig: signal.getsignal(sig) for sig in
                            (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
                process = Mock(pid=1234)
                def fake_wait(**kwargs):
                    if kwargs["timeout"] == 1:
                        # Invoke the installed handler, not an actual OS signal.
                        signal.getsignal(signum)(signum, None)
                    self.assertEqual(signal.getsignal(signal.SIGINT), signal.SIG_IGN)
                    self.assertEqual(signal.getsignal(signal.SIGTERM), signal.SIG_IGN)
                    self.assertEqual(signal.getsignal(signal.SIGHUP), signal.SIG_IGN)
                    return 130
                process.wait.side_effect = fake_wait
                with patch.object(M.subprocess, "Popen", return_value=process):
                    result = M.bounded_run(["fake-worker"], Path(directory) / "log", 1)
                process.send_signal.assert_called_once_with(signal.SIGINT)
                self.assertEqual(result["reason"], "USER_INTERRUPT")
                self.assertEqual(result["shutdown"], "WORKER_EXITED")
                for sig, handler in original.items():
                    self.assertEqual(signal.getsignal(sig), handler)

    def test_existing_output_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(M, "master_preflight", return_value={}), \
                    patch.object(M, "bounded_run") as runner, contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(M.main(["--output-dir", directory]), 1)
            runner.assert_not_called()
            self.assertFalse((Path(directory) / "workflow.json").exists())

    def test_analyzer_command_forwards_absolute_bags_and_provenance(self):
        args = M.parse_args([])
        with patch.object(M.subprocess, "run", return_value=Mock(returncode=2)) as run:
            self.assertEqual(M.analyze([Path("/tmp/a.bag"), Path("/tmp/b.bag")], Path("/tmp/run"), args), 2)
        command = run.call_args.args[0]
        self.assertEqual(command[7:9], ["/tmp/a.bag", "/tmp/b.bag"])
        self.assertEqual(command[:2], ["bash", "-c"])
        self.assertEqual(command[-4:], ["--heading-offset-deg", "-1.347", "--acquisition", "MR"])

    def test_worker_shell_paths_are_positional_not_interpolated(self):
        command = M.worker_command(Path("/tmp/a b; not shell.yaml"))
        self.assertEqual(command[-1], "/tmp/a b; not shell.yaml")
        self.assertNotIn("/tmp/a b", command[2])


if __name__ == "__main__":
    unittest.main()
