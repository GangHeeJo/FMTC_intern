#!/usr/bin/env python3
"""Static S1 host inventory; never opens devices or starts ROS/motor processes.

Run after sourcing the laptop's ROS and workspace setup files. Only the requested
report directory is written. This does not certify wiring, firmware, or motion.
"""
import argparse
import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
from urllib.parse import urlsplit, urlunsplit


COMMAND_TIMEOUT_S = 3
MODULES = ("rclpy", "serial", "cv2", "cv_bridge", "ultralytics", "torch", "numpy")
ROS_PACKAGES = ("joy", "usb_cam", "sllidar_ros2", "s1_stack", "lane_trace",
                "sign_detect", "obs_evade")
ENV_NAMES = ("ROS_DISTRO", "ROS_VERSION", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION",
             "ROS_LOCALHOST_ONLY", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH")
DEFAULT_DEVICES = {"arduino": "/dev/arduino_mega", "lidar": "/dev/rplidar",
                   "lane_camera": "/dev/cam_lane", "front_camera": "/dev/cam_front"}
UDEV_KEYS = {"ID_VENDOR_ID", "ID_MODEL_ID", "ID_SERIAL_SHORT", "ID_V4L_PRODUCT",
             "ID_USB_INTERFACE_NUM", "ID_PATH"}


def run_command(argv):
    """All subprocesses are read-only, shell-free and time-limited."""
    try:
        result = subprocess.run(argv, capture_output=True, text=True,
                                timeout=COMMAND_TIMEOUT_S, check=False,
                                env=dict(os.environ, GIT_OPTIONAL_LOCKS="0"))
        return {"status": "ok" if result.returncode == 0 else "error",
                "returncode": result.returncode, "stdout": result.stdout[:32768]}
    except subprocess.TimeoutExpired:
        return {"status": "timeout", "stdout": ""}
    except (OSError, UnicodeError):
        return {"status": "unavailable", "stdout": ""}


def find_repo(workspace):
    start = Path(workspace).expanduser().resolve()
    for candidate in (start, start / "src", start / "FMTC_intern"):
        if (candidate / "s1_stack" / "config" / "s1_stack.yaml").is_file():
            return candidate
    return start


def safe_remote(value):
    """Retain repo identity, removing credentials and URL query/fragment tokens."""
    value = value.strip()
    try:
        if "://" in value:
            parts = urlsplit(value)
            if not parts.hostname:
                return "[unrecognized remote]"
            netloc = parts.hostname + ((":" + str(parts.port)) if parts.port else "")
            return urlunsplit((parts.scheme, netloc, parts.path, "", ""))
        if re.match(r"^[^/@\s]+@[^/:\s]+:", value):
            return value.split("@", 1)[1].split("?", 1)[0].split("#", 1)[0]
        if value.startswith(("/", "./", "../")):
            return "[local filesystem remote]"
    except ValueError:
        pass
    return "[unrecognized remote]"


def git_inventory(repo):
    if not shutil.which("git"):
        return {"status": "unavailable"}
    def git(*args):
        return run_command(["git", "-C", str(repo)] + list(args))
    head = git("rev-parse", "HEAD")
    if head["status"] != "ok":
        return {"status": head["status"]}
    branch = git("symbolic-ref", "--short", "-q", "HEAD")
    dirty = git("status", "--porcelain", "--untracked-files=normal")
    remotes = {}
    names = git("remote")
    for name in names["stdout"].splitlines()[:16]:
        remote = git("remote", "get-url", name)
        remotes[name] = safe_remote(remote["stdout"]) if remote["status"] == "ok" else "unknown"
    return {"status": "ok", "head": head["stdout"].strip(),
            "branch": branch["stdout"].strip() or "detached/unknown",
            "dirty": bool(dirty["stdout"]) if dirty["status"] == "ok" else None,
            "changed_paths": dirty["stdout"].splitlines(), "remotes": remotes}


def module_inventory():
    found = {}
    for name in MODULES:
        try:
            spec = importlib.util.find_spec(name)
            found[name] = {"available": spec is not None,
                           "origin": str(spec.origin) if spec is not None else None}
        except (ImportError, AttributeError, ValueError):
            found[name] = {"available": None, "origin": None}
    return found


def package_inventory():
    try:
        from ament_index_python.packages import get_packages_with_prefixes
        installed = get_packages_with_prefixes()
        return {"status": "ok", "packages": {name: installed.get(name) for name in ROS_PACKAGES}}
    except (ImportError, OSError, ValueError, RuntimeError):
        return {"status": "unknown", "packages": {name: None for name in ROS_PACKAGES}}


def expected_devices(repo):
    """Read this repo's simple scalar paths without requiring PyYAML installed."""
    devices = dict(DEFAULT_DEVICES)
    config = repo / "s1_stack" / "config" / "s1_stack.yaml"
    launch = repo / "s1_stack" / "launch" / "s1_stack.launch.py"
    if config.is_file():
        source = config.read_text(encoding="utf-8")
        for key, field in (("arduino", "port"), ("lidar", "serial_port")):
            match = re.search(r"^\s+" + field + r":\s*['\"]?(/dev/[^'\"\s#]+)", source, re.M)
            if match:
                devices[key] = match.group(1)
    if launch.is_file():
        source = launch.read_text(encoding="utf-8")
        matches = re.findall(r"['\"]video_device['\"]\s*:\s*['\"](/dev/[^'\"]+)['\"]", source)
        if len(matches) == 2:
            devices["lane_camera"], devices["front_camera"] = matches
    return devices


def device_inventory(devices, linux):
    inventory = {}
    for label, value in devices.items():
        path = Path(value)
        exists = path.exists()
        item = {"path": value, "status": "present" if exists else ("missing" if linux else "unknown"),
                "is_symlink": path.is_symlink(), "resolved": str(path.resolve()),
                "readable": os.access(str(path), os.R_OK) if exists else None,
                "writable": os.access(str(path), os.W_OK) if exists else None}
        if linux and exists and shutil.which("udevadm"):
            result = run_command(["udevadm", "info", "--query=property", "--name", value])
            item["udev_status"] = result["status"]
            item["udev"] = {key: val for line in result["stdout"].splitlines()
                            if "=" in line for key, val in [line.split("=", 1)] if key in UDEV_KEYS}
        inventory[label] = item
    return inventory


def by_id_inventory(root):
    path = Path(root)
    if not path.is_dir():
        return {"status": "unavailable", "devices": []}
    try:
        names = sorted(path.iterdir())
        return {"status": "ok", "truncated": len(names) > 64,
                "devices": [{"path": str(p), "resolved": str(p.resolve()),
                             "exists": p.exists()} for p in names[:64]]}
    except OSError:
        return {"status": "unavailable", "devices": []}


def collect_report(workspace, model=None, hash_model=False):
    repo = find_repo(workspace)
    linux = platform.system() == "Linux"
    config = repo / "s1_stack" / "config" / "s1_stack.yaml"
    model_path = Path(model).expanduser().resolve() if model else repo / "sign_detect" / "best_0808.pt"
    model_info = {"path": str(model_path), "exists": model_path.is_file(), "sha256": None}
    if hash_model and model_info["exists"]:
        digest = hashlib.sha256()
        with model_path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        model_info["sha256"] = digest.hexdigest()
    report = {
        "schema": "s1_static_preflight.v1",
        "generated_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "scope": "Static host inventory only. No devices opened; no ROS daemon, camera, serial or motor commands.",
        "host": {"system": platform.system(), "release": platform.release(),
                 "architecture": platform.machine(), "python": sys.version.split()[0],
                 "python_executable": sys.executable},
        "workspace": str(repo), "config_exists": config.is_file(),
        "git": git_inventory(repo),
        "environment": {name: os.environ.get(name) for name in ENV_NAMES},
        "binaries": {name: shutil.which(name) for name in ("ros2", "colcon", "git", "lsusb", "udevadm")},
        "python_modules": module_inventory(), "ros_packages": package_inventory(),
        "devices": device_inventory(expected_devices(repo), linux),
        "device_ids": {name: by_id_inventory(path) for name, path in
                       (("serial", "/dev/serial/by-id"), ("video", "/dev/v4l/by-id"))},
        "model": model_info,
        "unverified": ["Vehicle wiring and installed firmware", "Steering calibration and motor direction",
                       "Physical stop response", "Live ROS topics, camera inference and scan quality",
                       "Controller axes/buttons and actual vehicle speed"],
    }
    os_release = Path("/etc/os-release")
    if linux and os_release.is_file():
        report["host"]["os_release"] = os_release.read_text(encoding="utf-8")[:8192]
    if linux and report["binaries"]["lsusb"]:
        report["usb"] = run_command(["lsusb"])
    report["findings"] = assess(report)
    return report


def assess(report):
    findings = []
    linux = report["host"]["system"] == "Linux"
    def add(severity, message):
        findings.append({"severity": severity, "message": message})
    if not report["config_exists"]:
        add("blocker", "S1 configuration not found. Pass the repository or colcon workspace with --workspace.")
    if not linux:
        add("warning", "This is not the target Linux host; ROS/device checks here cannot establish laptop readiness.")
    if not report["environment"]["ROS_DISTRO"]:
        add("blocker" if linux else "unknown", "ROS_DISTRO is unset. On the vehicle laptop, source ROS and workspace setup, then rerun.")
    if linux:
        for name in ("ros2", "colcon"):
            if not report["binaries"][name]:
                add("blocker", name + " executable is not on PATH in this shell. Check the target ROS/workspace environment.")
    for name, value in report["python_modules"].items():
        if value["available"] is not True:
            add("blocker" if linux and value["available"] is False else "unknown",
                "Python module {} is not discoverable in {}. Check the intended interpreter/environment.".format(name, sys.executable))
    for name, value in report["devices"].items():
        if value["status"] == "missing":
            add("blocker", "{} device alias missing: {}. Compare attached IDs with this vehicle's udev rules.".format(name, value["path"]))
        elif value["status"] == "present" and not (value["readable"] and value["writable"]):
            add("blocker" if linux else "warning", "No read/write permission for {}. Device was not opened.".format(value["path"]))
    if report["ros_packages"]["status"] == "ok":
        missing = [name for name, prefix in report["ros_packages"]["packages"].items() if prefix is None]
        if missing:
            add("blocker" if linux else "unknown", "ROS packages absent from the sourced ament index: " + ", ".join(missing))
    else:
        add("unknown", "Ament package index unavailable; installed/sourced ROS packages are unknown.")
    if not report["model"]["exists"]:
        add("blocker", "YOLO model file not found: " + report["model"]["path"])
    if report["git"].get("dirty"):
        add("warning", "Working tree has changes. Record this revision before comparing cars; source files were not altered.")
    add("unknown", "Static results do not verify installed Arduino firmware, pin wiring, calibration or physical stopping.")
    return findings


def summary(report):
    lines = ["S1 STATIC PREFLIGHT — " + report["generated_at"], report["scope"],
             "Host: {system} {release} / {architecture}".format(**report["host"]),
             "Workspace: " + report["workspace"],
             "Git: {} / {}".format(report["git"].get("branch", "unknown"), report["git"].get("head", "unknown")),
             "ROS_DISTRO: " + str(report["environment"]["ROS_DISTRO"]), "", "Findings:"]
    lines += ["[{}] {}".format(item["severity"].upper(), item["message"]) for item in report["findings"]]
    lines += ["", "Expected device aliases:"]
    lines += ["{}: {} -> {} ({})".format(name, item["path"], item["resolved"], item["status"])
              for name, item in report["devices"].items()]
    lines += ["", "Still requires field verification:"] + ["- " + item for item in report["unverified"]]
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", default=str(Path(__file__).resolve().parents[1]))
    parser.add_argument("--output", required=True, help="Directory for timestamped JSON and text reports")
    parser.add_argument("--model", help="Optional explicit YOLO model path; model is never loaded")
    parser.add_argument("--hash-model", action="store_true", help="Read the model file to record its SHA256")
    args = parser.parse_args(argv)
    report = collect_report(args.workspace, args.model, args.hash_model)
    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    stem = "s1_preflight_" + stamp
    with (output / (stem + ".json")).open("x", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    text = summary(report)
    with (output / (stem + ".txt")).open("x", encoding="utf-8") as handle:
        handle.write(text)
    print(text, end="")
    print("Reports: " + str(output / stem) + ".{json,txt}")
    # Findings describe readiness; successful report collection is exit 0.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
