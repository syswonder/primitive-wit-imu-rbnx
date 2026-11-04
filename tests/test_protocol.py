import math
import struct
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "vendor"))

from wit_ros2_imu.protocol import FrameParser, decode_frame, quaternion_from_rpy


def frame(kind, payload):
    prefix = bytes((0x55, kind)) + payload
    return prefix + bytes((sum(prefix) & 0xFF,))


class ProtocolTest(unittest.TestCase):
    def test_resync_and_physical_units(self):
        accel = frame(0x51, struct.pack("<hhhh", 0, 0, 2048, 0))
        gyro = frame(0x52, struct.pack("<hhhh", 0, 0, 1475, 0))
        bad = bytearray(accel)
        bad[-1] ^= 1
        parser = FrameParser()
        self.assertEqual(parser.feed(b"noise" + bad + accel[:4]), [])
        result = parser.feed(accel[4:] + gyro)
        self.assertEqual(len(result), 2)
        self.assertGreaterEqual(parser.bad_checksum, 1)
        self.assertAlmostEqual(decode_frame(*result[0])[1][2], 9.80665, places=4)
        self.assertAlmostEqual(decode_frame(*result[1])[1][2], math.pi / 2, delta=0.002)

    def test_pressure_and_orientation(self):
        parser = FrameParser()
        decoded = parser.feed(frame(0x56, struct.pack("<Ii", 101325, 1234)))
        self.assertEqual(decode_frame(*decoded[0]), ("barometer", (101325.0, 12.34)))
        q = quaternion_from_rpy(0, 0, math.pi / 2)
        self.assertAlmostEqual(q[2], math.sqrt(.5), places=6)
        self.assertAlmostEqual(q[3], math.sqrt(.5), places=6)


if __name__ == "__main__":
    unittest.main()
