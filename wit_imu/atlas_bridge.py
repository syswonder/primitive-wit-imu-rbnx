# SPDX-License-Identifier: MulanPSL-2.0
import json
import logging
import math
import os
from pathlib import Path
import time

from robonix_api import Err, Ok, Primitive
from .driver import Driver, parse_config

imu = Primitive(id=os.environ.get("RBNX_INSTANCE_NAME", "wit_imu"), namespace="robonix/primitive/imu")
driver = Driver(Path(__file__).resolve().parents[1])
config = parse_config({})
declared_topics = None


@imu.on_init
def init(cfg):
    global config
    if driver.process is not None:
        return Err("Deactivate before changing WIT configuration")
    try:
        parsed = parse_config(cfg)
    except ValueError as exc:
        return Err(str(exc))
    if declared_topics is not None and parsed["topics"] != declared_topics:
        return Err("Restart the provider to change already declared topic endpoints")
    config = parsed
    return Ok()


@imu.on_activate
def activate():
    global declared_topics
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String

    context = Context()
    executor = None
    node = None
    seen = set()
    ready = False

    def on_imu(msg):
        values = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                  msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                  msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        if all(math.isfinite(value) for value in values):
            seen.add("imu")

    def on_status(msg):
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(status, dict) and status.get("serial_connected") is True and status.get("valid") is True:
            seen.add("status")

    try:
        rclpy.init(context=context)
        executor = SingleThreadedExecutor(context=context)
        node = rclpy.create_node("wit_imu_sentinel", context=context)
        executor.add_node(node)
        node.create_subscription(Imu, config["topics"]["imu"], on_imu, qos_profile_sensor_data)
        node.create_subscription(String, config["topics"]["status"], on_status, qos_profile_sensor_data)
        if config["manage_driver"] and driver.process is None:
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
            if any(node.count_publishers(topic) for topic in config["topics"].values()):
                return Err("An external WIT publisher already exists; stop it or use manage_driver=false")
            driver.start(config)
        # Only messages received after driver ownership is settled count as readiness.
        seen.clear()
        deadline = time.monotonic() + config["sentinel_timeout_s"]
        while len(seen) < 2 and time.monotonic() < deadline:
            if driver.process is not None and driver.process.poll() is not None:
                return Err("Managed WIT driver exited; see rbnx-build/data/driver.log")
            executor.spin_once(timeout_sec=min(0.1, max(0, deadline - time.monotonic())))
        if len(seen) != 2:
            return Err("No fresh WIT IMU/status samples; check serial port, baud, sensor output and ROS environment")
        for key, topic in config["topics"].items():
            if declared_topics is None:
                imu.declare_ros2_topic(f"robonix/primitive/imu/{key}", topic, qos="reliable")
        declared_topics = dict(config["topics"])
        ready = True
    except (ValueError, OSError) as exc:
        return Err(str(exc))
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
        if not ready:
            driver.stop()
    return Ok()


@imu.on_deactivate
def deactivate():
    driver.stop()
    return Ok()


@imu.on_shutdown
def shutdown():
    driver.stop()
    return Ok()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    imu.run()
