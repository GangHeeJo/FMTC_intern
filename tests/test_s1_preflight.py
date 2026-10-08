"""Host-only static preflight tests; no ROS transport or hardware is exercised."""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "s1_preflight.py"
SPEC = importlib.util.spec_from_file_location("s1_preflight", SCRIPT)
preflight = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preflight)


class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.repo = self.root / "workspace with spaces" / "src"
        self.config = self.repo / "s1_stack" / "config" / "s1_stack.yaml"
        self.config.parent.mkdir(parents=True)
        self.config.write_text("serial_sender:\n  ros__parameters:\n    port: '/dev/missing_s1_test_arduino'\n"
                               "sllidar_node:\n  ros__parameters:\n    serial_port: /dev/missing_s1_test_lidar\n")
        launch = self.repo / "s1_stack" / "launch" / "s1_stack.launch.py"
        launch.parent.mkdir(parents=True)
        launch.write_text("'video_device': '/dev/missing_s1_test_lane'\n"
                          "'video_device': '/dev/missing_s1_test_front'\n")
        model = self.repo / "sign_detect" / "best_0808.pt"
        model.parent.mkdir()
        model.write_bytes(b"fixture; not a real model")

    def collect(self, system="Linux", **kwargs):
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(preflight.platform, "system", return_value=system))
            stack.enter_context(mock.patch.object(preflight.shutil, "which", return_value=None))
            stack.enter_context(mock.patch.dict(preflight.os.environ, {}, clear=True))
            stack.enter_context(mock.patch.object(preflight, "module_inventory", return_value={
                name: {"available": False, "origin": None} for name in preflight.MODULES}))
            stack.enter_context(mock.patch.object(preflight, "package_inventory", return_value={
                "status": "unknown", "packages": {name: None for name in preflight.ROS_PACKAGES}}))
            return preflight.collect_report(self.repo.parent, **kwargs)

    def test_workspace_configured_paths_and_missing_devices(self):
        report = self.collect()
        self.assertEqual(report["workspace"], str(self.repo))
        self.assertEqual(report["devices"]["arduino"]["path"], "/dev/missing_s1_test_arduino")
        self.assertEqual(report["devices"]["lane_camera"]["path"], "/dev/missing_s1_test_lane")
        self.assertTrue(all(item["status"] == "missing" for item in report["devices"].values()))
        self.assertTrue(any("ROS_DISTRO is unset" in item["message"] and item["severity"] == "blocker"
                            for item in report["findings"]))
        self.assertEqual(report["ros_packages"]["status"], "unknown")

    def test_mac_does_not_fail_absent_ros_or_linux_devices(self):
        report = self.collect(system="Darwin")
        self.assertTrue(all(item["status"] == "unknown" for item in report["devices"].values()))
        self.assertFalse(any(item["severity"] == "blocker" for item in report["findings"]))
        self.assertTrue(any("not the target Linux host" in item["message"] for item in report["findings"]))

    def test_model_hash_opt_in_and_missing_model(self):
        self.assertIsNone(self.collect()["model"]["sha256"])
        report = self.collect(hash_model=True)
        self.assertEqual(report["model"]["sha256"], hashlib.sha256(b"fixture; not a real model").hexdigest())
        report = self.collect(model=str(self.root / "missing.pt"))
        self.assertFalse(report["model"]["exists"])
        self.assertTrue(any("YOLO model file not found" in item["message"] for item in report["findings"]))

    def test_timestamped_reports_existing_output_preserves_source(self):
        report = self.collect(system="Darwin")
        before = self.config.read_bytes()
        output = self.root / "reports with spaces"
        output.mkdir()
        keep = output / "keep.txt"
        keep.write_text("keep me")
        with mock.patch.object(preflight, "collect_report", return_value=report), contextlib.redirect_stdout(io.StringIO()):
            for _ in range(2):
                self.assertEqual(preflight.main(["--workspace", str(self.repo.parent), "--output", str(output)]), 0)
        reports = list(output.glob("s1_preflight_*.json"))
        self.assertEqual(len(reports), 2)
        self.assertEqual(len(list(output.glob("s1_preflight_*.txt"))), 2)
        self.assertEqual(json.loads(reports[0].read_text())["schema"], "s1_static_preflight.v1")
        self.assertEqual(self.config.read_bytes(), before)
        self.assertEqual(keep.read_text(), "keep me")

    def test_commands_are_bounded_and_shell_free(self):
        with mock.patch.object(preflight.subprocess, "run", side_effect=subprocess.TimeoutExpired("git", 3)) as run:
            self.assertEqual(preflight.run_command(["git", "status"])["status"], "timeout")
        self.assertEqual(run.call_args.kwargs["timeout"], preflight.COMMAND_TIMEOUT_S)
        self.assertNotIn("shell", run.call_args.kwargs)
        self.assertEqual(run.call_args.kwargs["env"]["GIT_OPTIONAL_LOCKS"], "0")

    def test_remotes_redact_credentials_queries_and_fragments(self):
        for remote in ("https://token123@github.com/team/repo.git?token=secret#private",
                       "https://user:token123@github.com/team/repo.git?token=secret#private"):
            self.assertEqual(preflight.safe_remote(remote), "https://github.com/team/repo.git")
        self.assertEqual(preflight.safe_remote("git@github.com:team/repo.git"), "github.com:team/repo.git")
        self.assertEqual(preflight.safe_remote("/local/private/repo"), "[local filesystem remote]")

    def test_module_inventory_discovers_without_importing_libraries(self):
        with mock.patch.object(preflight.importlib.util, "find_spec", return_value=None) as lookup:
            found = preflight.module_inventory()
        self.assertEqual(set(found), set(preflight.MODULES))
        self.assertEqual(lookup.call_count, len(preflight.MODULES))
        self.assertTrue(all(item["available"] is False for item in found.values()))

    def test_device_inventory_reads_metadata_only(self):
        target = self.root / "device fixture"
        target.write_text("must not be read")
        alias = self.root / "alias"
        alias.symlink_to(target)
        with mock.patch.object(Path, "open", side_effect=AssertionError("Devices must not be opened")), \
                mock.patch.object(preflight.shutil, "which", return_value="/usr/bin/udevadm"), \
                mock.patch.object(preflight, "run_command", return_value={"status": "ok", "stdout":
                    "ID_VENDOR_ID=2341\nID_SERIAL_SHORT=fixture123\nPRIVATE_VALUE=omit_me\n"}) as run:
            result = preflight.device_inventory({"arduino": str(alias)}, linux=True)
        self.assertEqual(result["arduino"]["resolved"], str(target))
        self.assertEqual(result["arduino"]["udev"], {"ID_VENDOR_ID": "2341", "ID_SERIAL_SHORT": "fixture123"})
        self.assertEqual(run.call_args.args[0], ["udevadm", "info", "--query=property", "--name", str(alias)])


if __name__ == "__main__":
    unittest.main()
