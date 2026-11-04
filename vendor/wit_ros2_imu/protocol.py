"""Decoder for WIT's 11-byte standard serial frames.

The sensor's raw magnetic counts are intentionally not converted to tesla:
the supplied documentation does not specify a model-specific scale.
"""

import math
import struct

FRAME_LENGTH = 11
GRAVITY = 9.80665


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_checksum = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= FRAME_LENGTH:
            if self.buffer[0] != 0x55:
                start = self.buffer.find(0x55)
                if start < 0:
                    self.buffer.clear()
                    break
                del self.buffer[:start]
                if len(self.buffer) < FRAME_LENGTH:
                    break
            frame = self.buffer[:FRAME_LENGTH]
            if (sum(frame[:10]) & 0xFF) != frame[10]:
                self.bad_checksum += 1
                del self.buffer[0]
                continue
            frames.append((frame[1], bytes(frame[2:10])))
            del self.buffer[:FRAME_LENGTH]
        return frames


def decode_frame(kind, payload):
    if len(payload) != 8:
        raise ValueError("WIT frame payload must be 8 bytes")
    if kind in (0x51, 0x52, 0x53, 0x54):
        x, y, z, auxiliary = struct.unpack("<hhhh", payload)
        if kind == 0x51:
            scale = 16.0 * GRAVITY / 32768.0
            return "acceleration_mps2", (x * scale, y * scale, z * scale)
        if kind == 0x52:
            scale = math.radians(2000.0) / 32768.0
            return "angular_velocity_radps", (x * scale, y * scale, z * scale)
        if kind == 0x53:
            scale = math.pi / 32768.0
            return "rpy_rad", (x * scale, y * scale, z * scale)
        return "magnetic_raw_lsb", (x, y, z)
    if kind == 0x56:
        pressure_pa, altitude_cm = struct.unpack("<Ii", payload)
        return "barometer", (float(pressure_pa), altitude_cm / 100.0)
    return None, None


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)
