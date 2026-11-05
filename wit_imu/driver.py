# SPDX-License-Identifier: MulanPSL-2.0
"""Configuration and ownership of the bundled WIT serial driver."""
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

import yaml

TOPICS = {
    "imu": "/wit_10axis/data",
    "angular_velocity": "/wit_10axis/angular_velocity",
    "magnetic_raw": "/wit_10axis/magnetic_raw",
    "pressure": "/wit_10axis/pressure",
    "barometric_altitude": "/wit_10axis/barometric_altitude",
    "status": "/wit_10axis/status",
}


def parse_config(cfg):
    if not isinstance(cfg, dict):
        raise ValueError("Configuration must be an object")
    unknown = set(cfg) - {"manage_driver", "serial", "frame_id", "topics", "sentinel_timeout_s"}
    if unknown:
        raise ValueError(f"Unknown config keys: {sorted(unknown)}")
    managed = cfg.get("manage_driver", True)
    if not isinstance(managed, bool):
        raise ValueError("manage_driver must be boolean")
    serial = cfg.get("serial", {})
    if not isinstance(serial, dict) or set(serial) - {"port", "baud", "usb_vid", "usb_pid"}:
        raise ValueError("serial accepts port, baud, usb_vid and usb_pid")
    if not managed and (serial or "frame_id" in cfg):
        raise ValueError("Serial/frame overrides require manage_driver=true; external hardware is not reconfigured")
    serial = {"port": "auto", "baud": 9600, **serial}
    port = serial["port"]
    if not isinstance(port, str) or (port != "auto" and not Path(port).is_absolute()) or any(c in port for c in "\r\n\0"):
        raise ValueError("serial.port must be auto or an absolute device path")
    if type(serial["baud"]) is not int or serial["baud"] <= 0:
        raise ValueError("serial.baud must be a positive integer")
    if ("usb_vid" in serial) != ("usb_pid" in serial):
        raise ValueError("serial.usb_vid and usb_pid must be specified together")
    for key in ("usb_vid", "usb_pid"):
        if key in serial and (type(serial[key]) is not int or not 0 <= serial[key] <= 65535):
            raise ValueError(f"serial.{key} must be an integer in [0,65535]")
    frame = cfg.get("frame_id", "wit_10axis_link")
    if not isinstance(frame, str) or not frame or any(c.isspace() or c == "\0" for c in frame):
        raise ValueError("frame_id must be a nonempty string without whitespace")
    timeout = cfg.get("sentinel_timeout_s", 10.0)
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 0 < timeout <= 120:
        raise ValueError("sentinel_timeout_s must be in (0,120] seconds")
    topics = cfg.get("topics", {})
    if not isinstance(topics, dict) or set(topics) - set(TOPICS):
        raise ValueError("Unknown topics key")
    topics = {**TOPICS, **topics}
    pattern = r"/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*"
    if any(not isinstance(topic, str) or re.fullmatch(pattern, topic) is None for topic in topics.values()):
        raise ValueError("Topics must be absolute ROS names without substitutions")
    if len(set(topics.values())) != len(topics):
        raise ValueError("Each stream must use a distinct topic")
    return {"manage_driver": managed, "serial": serial, "frame_id": frame,
            "sentinel_timeout_s": float(timeout), "topics": topics}


def resolve_port(serial):
    if serial["port"] != "auto":
        return serial["port"]
    if "usb_vid" in serial:
        from serial.tools import list_ports
        candidates = [Path(p.device) for p in list_ports.comports()
                      if p.vid == serial["usb_vid"] and p.pid == serial["usb_pid"]]
    else:
        candidates = [Path(p) for p in ("/dev/wit_imu", "/dev/imu_usb") if Path(p).exists()]
        candidates += [p for p in Path("/dev/serial/by-id").glob("*")
                       if any(label in p.name.lower() for label in ("wit", "wt901", "imu")) and p.exists()]
    unique = {p.resolve(): p for p in candidates}
    if len(unique) != 1:
        raise ValueError("Cannot uniquely identify WIT hardware; set serial.port to a stable /dev/serial/by-id path")
    return str(next(iter(unique.values())))


class Driver:
    """Own only the child and runtime parameters created by this primitive."""

    def __init__(self, root):
        self.root = Path(root)
        self.process = None
        self.params_path = None
        self.log = None

    def start(self, config):
        if self.process is not None:
            raise RuntimeError("Driver already started")
        parameters = {"port": resolve_port(config["serial"]), "baud": config["serial"]["baud"],
                      "frame_id": config["frame_id"]}
        directory = self.root / "rbnx-build/data"
        directory.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(prefix="wit-", suffix=".yaml", dir=directory)
        self.params_path = Path(name)
        try:
            with os.fdopen(fd, "w") as stream:
                yaml.safe_dump({"/**": {"ros__parameters": parameters}}, stream)
            env = dict(os.environ)
            env["PYTHONPATH"] = str(self.root / "vendor") + os.pathsep + env.get("PYTHONPATH", "")
            instance = re.sub(r"[^A-Za-z0-9_]", "_", env.get("RBNX_INSTANCE_NAME", "wit_imu"))
            command = [sys.executable, "-c", "from wit_ros2_imu.node import main; main()",
                       "--ros-args", "--params-file", str(self.params_path), "-r", f"__node:=wit_{instance}_driver"]
            for key, source in TOPICS.items():
                command += ["-r", f"{source}:={config['topics'][key]}"]
            self.log = (directory / "driver.log").open("a")
            self.process = subprocess.Popen(command, env=env, stdout=self.log, stderr=subprocess.STDOUT)
        except Exception:
            self.stop()
            raise

    def stop(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
            self.process = None
        if self.log is not None:
            self.log.close()
            self.log = None
        if self.params_path is not None:
            self.params_path.unlink(missing_ok=True)
            self.params_path = None
