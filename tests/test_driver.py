from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

from wit_imu.driver import Driver, TOPICS, parse_config, resolve_port


class DriverTest(unittest.TestCase):
    def test_defaults_and_external_mode(self):
        config = parse_config({})
        self.assertTrue(config["manage_driver"])
        self.assertEqual(config["serial"], {"port": "auto", "baud": 9600})
        self.assertEqual(config["topics"], TOPICS)
        self.assertFalse(parse_config({"manage_driver": False})["manage_driver"])
        for override in ({"serial": {"baud": 115200}}, {"frame_id": "new_frame"}):
            with self.assertRaises(ValueError):
                parse_config({"manage_driver": False, **override})

    def test_invalid_parameters(self):
        invalid = [None, {"manage_driver": "true"}, {"serial": {"port": "ttyUSB0"}},
                   {"serial": {"baud": True}}, {"serial": {"usb_vid": 0x10c4}},
                   {"serial": {"usb_vid": -1, "usb_pid": 0}}, {"frame_id": "bad frame"},
                   {"sentinel_timeout_s": float("nan")}, {"sentinel_timeout_s": 0},
                   {"topics": {"imu": "relative"}}, {"topics": {"unknown": "/test"}},
                   {"topics": {"imu": "/same", "status": "/same"}}, {"unexpected": True}]
        for config in invalid:
            with self.subTest(config=config), self.assertRaises(ValueError):
                parse_config(config)

    def test_auto_rejects_unknown_and_ambiguous_devices(self):
        with patch("wit_imu.driver.Path.exists", return_value=False), patch("wit_imu.driver.Path.glob", return_value=[]):
            with self.assertRaises(ValueError):
                resolve_port({"port": "auto"})
        with patch("wit_imu.driver.Path.exists", return_value=True), patch("wit_imu.driver.Path.glob", return_value=[]):
            with self.assertRaises(ValueError):
                resolve_port({"port": "auto"})

    def test_child_uses_bundled_driver_and_remapped_parameters(self):
        config = parse_config({"serial": {"port": "/dev/example", "baud": 115200},
                               "frame_id": "other_body_imu", "topics": {"imu": "/other/imu"}})
        with tempfile.TemporaryDirectory() as tmp:
            owner = Driver(tmp)
            with patch("wit_imu.driver.subprocess.Popen") as launch:
                launch.return_value.poll.return_value = None
                owner.start(config)
                parameters = yaml.safe_load(owner.params_path.read_text())["/**"]["ros__parameters"]
                self.assertEqual(parameters, {"port": "/dev/example", "baud": 115200, "frame_id": "other_body_imu"})
                self.assertEqual(owner.params_path.stat().st_mode & 0o777, 0o600)
                command = launch.call_args.args[0]
                self.assertIn("from wit_ros2_imu.node import main; main()", command)
                self.assertIn("/wit_10axis/data:=/other/imu", command)
                self.assertTrue(launch.call_args.kwargs["env"]["PYTHONPATH"].startswith(str(Path(tmp) / "vendor")))
                params_path = owner.params_path
                owner.stop()
                owner.stop()
                self.assertFalse(params_path.exists())
                launch.return_value.terminate.assert_called_once()

    def test_failed_spawn_cleans_parameters(self):
        with tempfile.TemporaryDirectory() as tmp:
            owner = Driver(tmp)
            config = parse_config({"serial": {"port": "/dev/example"}})
            with patch("wit_imu.driver.subprocess.Popen", side_effect=OSError("test spawn failure")):
                with self.assertRaises(OSError):
                    owner.start(config)
            self.assertIsNone(owner.params_path)
            self.assertEqual(list((Path(tmp) / "rbnx-build/data").glob("wit-*.yaml")), [])


if __name__ == "__main__":
    unittest.main()
