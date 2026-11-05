"""ROS 2 Humble bridge for a CP2102-connected WIT standard-protocol IMU."""

import time
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Imu
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64, String
import serial
from serial.tools import list_ports

from .protocol import FrameParser, decode_frame, quaternion_from_rpy


class WitImuNode(Node):
    def __init__(self):
        super().__init__("wit_10axis")
        self.declare_parameter("port", "auto")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("frame_id", "wit_10axis_link")
        self.declare_parameter("vendor_id", 0x10C4)
        self.declare_parameter("product_id", 0xEA60)
        self.port_name = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.vid = int(self.get_parameter("vendor_id").value)
        self.pid = int(self.get_parameter("product_id").value)
        self.imu_pub = self.create_publisher(Imu, "/wit_10axis/data", 10)
        self.gyro_pub = self.create_publisher(Vector3Stamped, "/wit_10axis/angular_velocity", 10)
        self.mag_pub = self.create_publisher(Vector3Stamped, "/wit_10axis/magnetic_raw", 10)
        self.pressure_pub = self.create_publisher(FluidPressure, "/wit_10axis/pressure", 10)
        self.altitude_pub = self.create_publisher(Float64, "/wit_10axis/barometric_altitude", 10)
        self.status_pub = self.create_publisher(String, "/wit_10axis/status", 10)
        self.parser = FrameParser()
        self.port = None
        self.last_attempt = 0.0
        self.last_warning = 0.0
        self.accel = None
        self.gyro = None
        self.accel_at = 0.; self.gyro_at = 0.; self.last_imu_at = 0.
        self.last_pressure_at = 0.; self.last_mag_at = 0.
        self.total = {"imu": 0, "mag": 0, "pressure": 0}
        self.started = time.monotonic()
        self.received = {"imu": 0, "mag": 0, "pressure": 0}
        self.create_timer(0.02, self.poll)
        self.create_timer(10.0, self.log_status)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f"WIT IMU ready: port={self.port_name}, baud={self.baud}; waiting for serial device")

    def find_port(self):
        if self.port_name != "auto":
            return self.port_name
        matches = [p.device for p in list_ports.comports() if p.vid == self.vid and p.pid == self.pid]
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            self.warn_limited(f"Found {len(matches)} matching CP210x ports; set the port parameter explicitly")
        return None

    def warn_limited(self, message):
        now = time.monotonic()
        if now - self.last_warning >= 10.0:
            self.get_logger().warn(message)
            self.last_warning = now

    def connect(self):
        now = time.monotonic()
        if now - self.last_attempt < 2.0:
            return
        self.last_attempt = now
        path = self.find_port()
        if not path:
            self.warn_limited(f"No USB serial device with {self.vid:04x}:{self.pid:04x}; still waiting")
            return
        try:
            self.port = serial.Serial(path, baudrate=self.baud, timeout=0, exclusive=True)
        except (OSError, serial.SerialException) as exc:
            self.warn_limited(f"Cannot open {path}: {exc}")
            return
        self.parser = FrameParser()
        self.accel = None; self.gyro = None
        self.accel_at = self.gyro_at = self.last_imu_at = 0.
        self.last_pressure_at = self.last_mag_at = 0.
        self.get_logger().info(f"Opened {path} at {self.baud} baud")

    def poll(self):
        if self.port is None:
            self.connect()
            return
        try:
            data = self.port.read(min(max(self.port.in_waiting, 1), 4096))
        except (OSError, serial.SerialException) as exc:
            self.get_logger().warn(f"Serial disconnected: {exc}; retrying")
            self.port.close()
            self.port = None
            self.accel = None; self.gyro = None
            return
        for kind, payload in self.parser.feed(data):
            label, value = decode_frame(kind, payload)
            if label == "acceleration_mps2":
                self.accel = value
                self.accel_at = time.monotonic()
            elif label == "angular_velocity_radps":
                self.gyro = value
                self.gyro_at = time.monotonic()
                msg = Vector3Stamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.vector.x, msg.vector.y, msg.vector.z = map(float, value)
                self.gyro_pub.publish(msg)
            elif label == "rpy_rad" and self.accel is not None and self.gyro is not None:
                if time.monotonic()-min(self.accel_at,self.gyro_at) < .25:
                    self.publish_imu(value)
            elif label == "magnetic_raw_lsb":
                msg = Vector3Stamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.vector.x, msg.vector.y, msg.vector.z = map(float, value)
                self.mag_pub.publish(msg)
                self.received["mag"] += 1
                self.total['mag'] += 1; self.last_mag_at = time.monotonic()
            elif label == "barometer":
                pressure_pa, altitude_m = value
                if pressure_pa > 0:
                    msg = FluidPressure()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.frame_id
                    msg.fluid_pressure = pressure_pa
                    self.pressure_pub.publish(msg)
                    self.altitude_pub.publish(Float64(data=altitude_m))
                    self.received["pressure"] += 1
                    self.total['pressure'] += 1; self.last_pressure_at = time.monotonic()

    def publish_imu(self, rpy):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        q = quaternion_from_rpy(*rpy)
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = self.accel
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = self.gyro
        self.imu_pub.publish(msg)
        self.received["imu"] += 1
        self.total['imu'] += 1; self.last_imu_at = time.monotonic()

    def publish_status(self):
        now = time.monotonic()
        age = lambda t: now-t if t else None
        self.status_pub.publish(String(data=json.dumps(dict(
            serial_connected=self.port is not None, port=self.port.port if self.port else self.port_name,
            baud=self.baud, frame_id=self.frame_id, timestamp_source='HOST_SERIAL_RECEIVE',
            imu_age_s=age(self.last_imu_at), gyro_age_s=age(self.gyro_at), magnetic_age_s=age(self.last_mag_at), pressure_age_s=age(self.last_pressure_at),
            valid=bool(self.last_imu_at and now-self.last_imu_at < .5),
            counts=self.total, checksum_errors=self.parser.bad_checksum,
            orientation_reference='SENSOR_INTERNAL_UNALIGNED', geographic_heading_valid=False,
            covariance_known=False), allow_nan=False)))

    def log_status(self):
        self.get_logger().info(
            f"10 s totals: imu={self.received['imu']}, mag={self.received['mag']}, "
            f"pressure={self.received['pressure']}, checksum_errors={self.parser.bad_checksum}")
        self.received = {"imu": 0, "mag": 0, "pressure": 0}

    def destroy_node(self):
        if self.port is not None:
            self.port.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = WitImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
